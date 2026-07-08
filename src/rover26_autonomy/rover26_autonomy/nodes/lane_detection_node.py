#!/usr/bin/env python3
"""LaneDetectionNode — subscribes to a raw fisheye camera topic, runs YOLOPv2
lane detection, and publishes lane polynomial coefficients + an annotated frame.

[INSTRUMENTED DIAG BUILD] — every added line is prefixed [DIAG] and throttled.
Logs at the SOURCE so garbage coefficients can be caught before fusion:
  [DIAG:src]   Published coeffs per side (exact values sent on the wire)
  [DIAG:fit]   Per-side sanity: |a| vs sharp threshold, x-position at 3 rows,
               flags off-image fits and near-horizontal fits
  [DIAG:time]  detect() latency + inter-frame period

Topics
------
Subscribed:  /{camera_name}/uncalibrated  (sensor_msgs/Image, BEST_EFFORT)
Published:   /lane_detection              (interfaces/LaneDetectionResult, RELIABLE)
"""

import time as _time

import numpy as _np
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

# ── DIAG configuration ────────────────────────────────────────────────────────
_DIAG_THROTTLE_S = 0.5           # min seconds between diag dumps
_DIAG_IMG_W      = 640           # must match lane_goal_publisher's IMG_W
_DIAG_IMG_H      = 480           # must match lane_goal_publisher's IMG_H
_DIAG_SHARP_THR  = 0.0040        # mirror of Cfg.SHARP_THRESHOLD for flagging
_DIAG_ROWS       = (int(0.25 * _DIAG_IMG_H),
                    int(0.50 * _DIAG_IMG_H),
                    int(0.75 * _DIAG_IMG_H))


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

        # ── DIAG state ──────────────────────────────────────────────────────
        self._diag_last_log_s: float = 0.0
        self._diag_last_frame_s: float | None = None
        self._diag_frame: int = 0

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
            # NOTE [DIAG]: comment says BEST_EFFORT is required to match the
            # camera driver, but this profile is RELIABLE. If frames ever stop
            # arriving after a camera-driver change, this mismatch is why.
            image_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=_QOS_DEPTH,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            # RELIABLE for the lane result — downstream consumers must not
            # silently drop a detection.
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
    # DIAG helpers

    def _diag_side_report(self, name: str, coeffs) -> str:
        """Build a one-line sanity report for one side's polynomial."""
        if len(coeffs) != 3:
            return f'{name}: MISSING (len={len(coeffs)})'

        a, b, c = (float(v) for v in coeffs)
        xs = [float(_np.polyval([a, b, c], r)) for r in _DIAG_ROWS]
        pos = ' '.join(f'r{r}:{x:7.1f}' for r, x in zip(_DIAG_ROWS, xs))

        flags = []
        if abs(a) >= _DIAG_SHARP_THR:
            flags.append(f'|a|≥sharp_thr({_DIAG_SHARP_THR})')
        if any(x < -_DIAG_IMG_W * 0.5 or x > _DIAG_IMG_W * 1.5 for x in xs):
            flags.append('OFF-IMAGE')
        # slope at mid row — a straight lane roughly vertical in BEV should
        # have |dx/dpy| well under 1 px/row; multiple px/row is near-horizontal
        mid_slope = 2.0 * a * _DIAG_ROWS[1] + b
        if abs(mid_slope) > 1.5:
            flags.append(f'NEAR-HORIZONTAL(slope={mid_slope:+.2f}px/row)')

        flag_s = ('  ⚠ ' + ', '.join(flags)) if flags else ''
        return (f'{name}: a={a:+.4e} b={b:+.4e} c={c:+.1f}  x@[{pos}]{flag_s}')

    # ------------------------------------------------------------------
    # Frame callback

    def _on_frame(self, msg: Image) -> None:
        _t_frame = _time.monotonic()
        self._diag_frame += 1

        try:
            raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self._log.err(f'LaneDetectionNode: image conversion failed: {exc}')
            return

        _t0 = _time.monotonic()
        try:
            annotated, left_coeffs, right_coeffs = self._detector.detect(raw)
        except Exception as exc:
            self._log.err(f'LaneDetectionNode: detect() failed: {exc}')
            return
        _detect_ms = (_time.monotonic() - _t0) * 1000.0

        # ── [DIAG] throttled per-frame source dump ─────────────────────────
        if _t_frame - self._diag_last_log_s >= _DIAG_THROTTLE_S:
            self._diag_last_log_s = _t_frame
            period_ms = ((_t_frame - self._diag_last_frame_s) * 1000.0
                         if self._diag_last_frame_s else 0.0)
            self._log.info(
                f'[DIAG:src] frame#{self._diag_frame}  raw={raw.shape[1]}x{raw.shape[0]}  '
                f'[DIAG:time] detect={_detect_ms:.0f}ms  period={period_ms:.0f}ms\n'
                f'  [DIAG:fit] {self._diag_side_report("LEFT ", left_coeffs)}\n'
                f'  [DIAG:fit] {self._diag_side_report("RIGHT", right_coeffs)}'
            )
        self._diag_last_frame_s = _t_frame

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