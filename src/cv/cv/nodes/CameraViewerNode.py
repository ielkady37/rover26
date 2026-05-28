#!/usr/bin/env python3
"""CameraViewerNode — development/debug tool.

Subscribes to all camera topics published by CameraStreamingNode and
renders them in OpenCV windows.  One window is opened per topic:

    /{camera_name}/uncalibrated
    /{camera_name}/calibrated   (only when the topic is active)

Press  q  or  Esc  in any window to exit.

Usage
-----
    ros2 run cv camera_viewer_node
"""

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from utils.Logger import RoverLogger
from utils.Configurator import Configurator
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

_QOS_DEPTH = 10

class CameraViewerNode(Node):
    """Visualises calibrated and uncalibrated image topics in OpenCV windows."""

    def __init__(self) -> None:
        super().__init__('camera_viewer_node')
        self._log = RoverLogger()
        self._bridge = CvBridge()
        # window_title → latest numpy frame (or None)
        self._frames: dict[str, object] = {}

        self._setup_subscriptions()

    # ------------------------------------------------------------------
    # Setup

    def _setup_subscriptions(self) -> None:
        try:
            cameras_cfg = Configurator().fetchData(Configurator.CAMERAS)
        except Exception as exc:
            self._log.err(f'CameraViewerNode: failed to load cameras.yaml: {exc}')
            return

        if not cameras_cfg:
            self._log.warn('CameraViewerNode: cameras.yaml is empty — no topics to subscribe to.')
            return

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=_QOS_DEPTH,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        for cam_name, cam_cfg in cameras_cfg.items():
            uncal_title = f'{cam_name} | uncalibrated'
            self._frames[uncal_title] = None
            self.create_subscription(
                Image,
                f'/{cam_name}/uncalibrated',
                lambda msg, t=uncal_title: self._on_frame(msg, t),
                image_qos,
            )
            self._log.info(f'CameraViewerNode: subscribed to /{cam_name}/uncalibrated')

            if cam_cfg.get('calibration'):
                cal_title = f'{cam_name} | calibrated'
                self._frames[cal_title] = None
                self.create_subscription(
                    Image,
                    f'/{cam_name}/calibrated',
                    lambda msg, t=cal_title: self._on_frame(msg, t),
                    image_qos,
                )
                self._log.info(f'CameraViewerNode: subscribed to /{cam_name}/calibrated')

    # ------------------------------------------------------------------
    # Subscription callback

    def _on_frame(self, msg: Image, window_title: str) -> None:
        try:
            self._frames[window_title] = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self._log.err(f'CameraViewerNode: conversion error for "{window_title}": {exc}')

    # ------------------------------------------------------------------
    # Display (called from the main loop)

    def show_frames(self) -> None:
        """Render the latest frame for every known window."""
        for title, frame in self._frames.items():
            if frame is not None:
                cv2.imshow(title, frame)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraViewerNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.show_frames()
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):  # q or Esc
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
