#!/usr/bin/env python3
"""LaneDetectionNode — subscribes to a raw fisheye camera topic, runs YOLOPv2
lane detection, and publishes lane polynomial coefficients + an annotated frame.

Topics
------
Subscribed:  /{camera_name}/uncalibrated  (sensor_msgs/Image, BEST_EFFORT)
Published:   /lane_detection              (interfaces/LaneDetectionResult, RELIABLE)
"""

import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn

from interfaces.msg import LaneDetectionResult
from cv.services.LaneDetector import LaneDetector
from utils.Configurator import Configurator
from utils.Logger import RoverLogger

_QOS_DEPTH = 10


class LaneDetectionNode(LifecycleNode):
    """Lifecycle node that drives the LaneDetector service."""

    def __init__(self) -> None:
        super().__init__('lane_detection_node')
        self._log = RoverLogger()
        self._bridge = CvBridge()
        self._detector = None
        self._cam_name = ''
        self._cam_sub = None
        self._result_pub = None

    # ------------------------------------------------------------------
    # Lifecycle hooks

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info('LaneDetectionNode: configuring...')
        try:
            conf = Configurator()
            ld_cfg = conf.fetchData(Configurator.LANE_DETECTION)
            if not ld_cfg:
                self._log.err('LaneDetectionNode: lane_detection.yaml is empty or missing.')
                return TransitionCallbackReturn.FAILURE

            cam_name = str(ld_cfg.get('camera_name', 'fisheye_camera'))
            cameras_cfg = conf.fetchData(Configurator.CAMERAS)
            cam_cfg = (cameras_cfg or {}).get(cam_name, {})
            if not cam_cfg:
                self._log.warn(
                    f'LaneDetectionNode: camera "{cam_name}" not found in cameras.yaml'
                    ' — using defaults.'
                )

            self._cam_name = cam_name
            self._detector = LaneDetector(ld_cfg, cam_cfg)
            self._detector.load()

            self._log.succ('LaneDetectionNode: model loaded successfully.')
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self._log.err(f'LaneDetectionNode on_configure failed: {exc}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info('LaneDetectionNode: activating...')
        try:
            image_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=_QOS_DEPTH,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            result_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=_QOS_DEPTH,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self._cam_sub = self.create_subscription(
                Image,
                f'/{self._cam_name}/uncalibrated',
                self._on_frame,
                image_qos,
            )
            self._result_pub = self.create_publisher(
                LaneDetectionResult, '/lane_detection', result_qos
            )
            self._log.info(
                f'LaneDetectionNode: subscribed to /{self._cam_name}/uncalibrated'
            )
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self._log.err(f'LaneDetectionNode on_activate failed: {exc}')
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info('LaneDetectionNode: deactivating...')
        if self._cam_sub is not None:
            self.destroy_subscription(self._cam_sub)
            self._cam_sub = None
        if self._result_pub is not None:
            self.destroy_publisher(self._result_pub)
            self._result_pub = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info('LaneDetectionNode: cleaning up...')
        self._detector = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info('LaneDetectionNode: shutting down...')
        self.on_deactivate(state)
        self.on_cleanup(state)
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Frame callback

    def _on_frame(self, msg: Image) -> None:
        try:
            raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self._log.err(f'LaneDetectionNode: image conversion failed: {exc}')
            return

        try:
            annotated, left_coeffs, right_coeffs = self._detector.detect(raw)
        except Exception as exc:
            self._log.err(f'LaneDetectionNode: detect() failed: {exc}')
            return

        try:
            result = LaneDetectionResult()
            result.header.stamp = self.get_clock().now().to_msg()
            result.header.frame_id = self._cam_name
            result.left_lane_coeffs = [float(c) for c in left_coeffs]
            result.right_lane_coeffs = [float(c) for c in right_coeffs]
            result.annotated_frame = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            if self._result_pub is not None:
                self._result_pub.publish(result)
        except Exception as exc:
            self._log.err(f'LaneDetectionNode: publish failed: {exc}')


def main(args=None) -> None:
    """Entry point registered in setup.py."""
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
