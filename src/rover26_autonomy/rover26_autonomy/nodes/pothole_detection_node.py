"""
═════════════════════════════════════════════════════════════════════════════════
pothole_detection_node.py  —  rover26_autonomy
═════════════════════════════════════════════════════════════════════════════════

FUNCTIONALITY
─────────────
Detects white pothole blobs via the forward camera and publishes their
ground-plane coordinates and radii for downstream consumers.

PIPELINE (per frame)
────────────────────
  1. White-pixel mask extraction via HLS-based thresholding (VisionUtils)
  2. Contour detection and area filtering
  3. Shape descriptor filtering (aspect ratio, solidity, circularity)
  4. Sky/hood rejection — discard blobs in the upper third of the frame
  5. Ground-plane projection from raw pixel row → metres ahead / lateral
  6. Radius estimation from contour area
  7. Publish detections as a flat Float32MultiArray [x, y, r, x, y, r, ...]

GROUND PROJECTION FORMULA
─────────────────────────
    x_fwd = (1.0 - cy / img_h) * GROUND_HEIGHT_M
    y_lat = (0.5 - cx / img_w) * GROUND_WIDTH_M

TOPICS & SUBSCRIPTIONS
──────────────────────
Subscribed:
  • /camera/image       [sensor_msgs/Image]        ← camera driver
  • /imu                [sensor_msgs/Imu]           ← IMU driver (reserved)

Published:
  • /pothole_detections [std_msgs/Float32MultiArray] → pothole_publisher_node

PARAMETERS (from config_params.py)
──────────────────────────────────
PotholeDetection.*   Blob area, shape thresholds, detection range
Physical.*           GROUND_HEIGHT_M, GROUND_WIDTH_M
Debug.*              Verbosity, display mode

═════════════════════════════════════════════════════════════════════════════════
"""

import math
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Float32MultiArray
from rover26.msg import LaneStatus

from ..utils.vision_utils import VisionUtils
from ..config_params import (
    IMG_W, IMG_H,
    GROUND_HEIGHT_M, GROUND_WIDTH_M,
    PotholeDetection, Debug, RosTopics,
)


_HAS_DISPLAY = bool(os.environ.get('DISPLAY', ''))

_BEV_PX_PER_M_X: float = IMG_H / GROUND_HEIGHT_M
_BEV_PX_PER_M_Y: float = IMG_W / GROUND_WIDTH_M


# ═════════════════════════════════════════════════════════════════════════════
#  HELPERS
# ═════════════════════════════════════════════════════════════════════════════

def _shape_features(cnt) -> tuple:
    """
    Compute three shape descriptors for a contour.

    Returns:
        (aspect_ratio, solidity, circularity) where:
          aspect_ratio — bounding-box width / height  (high = elongated → lane line)
          solidity     — area / convex-hull area       (low  = jagged   → noise)
          circularity  — 4π·area / perimeter²          (1.0  = perfect circle → pothole)
    """
    _, _, bw, bh = cv2.boundingRect(cnt)
    aspect = bw / max(bh, 1)

    hull_area = cv2.contourArea(cv2.convexHull(cnt))
    solidity  = cv2.contourArea(cnt) / max(hull_area, 1)

    perimeter   = cv2.arcLength(cnt, True)
    circularity = 4 * np.pi * cv2.contourArea(cnt) / max(perimeter ** 2, 1)

    return aspect, solidity, circularity


def _validate_contour(cnt) -> dict | None:
    """
    Validate a contour against area and shape criteria.

    Returns:
        Dict with validated properties or None if rejected.
    """
    area = cv2.contourArea(cnt)
    if not (PotholeDetection.MIN_AREA < area < PotholeDetection.MAX_AREA):
        return None

    aspect, solidity, circularity = _shape_features(cnt)

    if aspect > PotholeDetection.MAX_ASPECT_RATIO:
        return None
    if solidity < PotholeDetection.MIN_SOLIDITY:
        return None
    if not (PotholeDetection.MIN_CIRCULARITY <= circularity <= PotholeDetection.MAX_CIRCULARITY):
        return None

    return {
        'area':        area,
        'aspect':      aspect,
        'solidity':    solidity,
        'circularity': circularity,
    }


# ═════════════════════════════════════════════════════════════════════════════
#  NODE
# ═════════════════════════════════════════════════════════════════════════════

class PotholeDetectionNode(Node):
    """
    Detects white pothole blobs in the forward camera image and publishes
    their ground-plane position and radius as a flat Float32MultiArray.

    Each detection triplet in the array is [x_fwd, y_lat, radius_m].
    An empty array is published when no potholes are detected so downstream
    consumers can clear stale state.
    """

    def __init__(self):
        super().__init__('pothole_detection_node')

        self.bridge = CvBridge()
        self.vision = VisionUtils()

        self.create_subscription(Image, RosTopics.CAMERA_IMAGE, self._image_cb, 10)
        self.create_subscription(Imu,   RosTopics.IMU,          self._imu_cb,   10)

        self._detections_pub = self.create_publisher(
            Float32MultiArray, RosTopics.POTHOLE_DETECTIONS, 10
        )

        if _HAS_DISPLAY and Debug.ENABLE_DISPLAY:
            cv2.namedWindow('Pothole Detection', cv2.WINDOW_NORMAL)

        self.get_logger().info(
            'pothole_detection_node ready\n'
            f'  ground extent : {GROUND_HEIGHT_M} m ahead  {GROUND_WIDTH_M} m wide\n'
            f'  area filter   : [{PotholeDetection.MIN_AREA}, {PotholeDetection.MAX_AREA}] px²\n'
            f'  shape filter  : circ=[{PotholeDetection.MIN_CIRCULARITY}, '
            f'{PotholeDetection.MAX_CIRCULARITY}]  '
            f'asp<{PotholeDetection.MAX_ASPECT_RATIO}  '
            f'sol>{PotholeDetection.MIN_SOLIDITY}'
        )

    # =========================================================================
    #  CALLBACKS
    # =========================================================================

    def _imu_cb(self, msg: Imu) -> None:
        """Reserved for future tilt-compensation logic."""
        pass

    def _image_cb(self, msg: Image) -> None:
        """Receive a raw camera frame and run the full detection pipeline."""
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._process(img)

    # =========================================================================
    #  DETECTION PIPELINE
    # =========================================================================

    def _process(self, img: np.ndarray) -> None:
        """
        Run the pothole detection pipeline on one BGR frame.

        Publishes a flat Float32MultiArray of [x, y, radius, x, y, radius, ...]
        for every accepted pothole. Publishes an empty array when none are found.

        Args:
            img: Native-resolution BGR camera frame.
        """
        h, w = img.shape[:2]

        white_mask = self.vision.get_white_mask(img, apply_roi=False)
        conts, _   = cv2.findContours(
            white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        detections: list  = []   # (x_fwd, y_lat, radius_m)
        debug_infos: list = []

        for cnt in conts:
            props = _validate_contour(cnt)
            if props is None:
                continue

            M_cnt = cv2.moments(cnt)
            if M_cnt['m00'] == 0:
                continue
            cx = int(M_cnt['m10'] / M_cnt['m00'])
            cy = int(M_cnt['m01'] / M_cnt['m00'])

            info = {
                'cnt':           cnt,
                'cx':            cx,
                'cy':            cy,
                'reject_reason': None,
                **props,
                'valid':         False,
            }
            debug_infos.append(info)

            if cy < h // 3:
                info['reject_reason'] = 'above road region'
                continue

            x_fwd = (1.0 - cy / h) * GROUND_HEIGHT_M
            y_lat = (0.5 - cx / w) * GROUND_WIDTH_M

            if not (PotholeDetection.MIN_DETECTION_M < x_fwd < PotholeDetection.MAX_DETECTION_M):
                info['reject_reason'] = 'outside detection range'
                continue

            m_per_px  = x_fwd / max(h - cy, 1)
            px_radius = math.sqrt(max(props['area'], 1) / math.pi)
            radius_m  = float(np.clip(
                px_radius * m_per_px * PotholeDetection.RADIUS_SCALE,
                PotholeDetection.MIN_RADIUS_M,
                PotholeDetection.MAX_RADIUS_M,
            ))
            
            detections.append((x_fwd, y_lat, radius_m))
            info.update({'valid': True, 'x_fwd': x_fwd, 'y_lat': y_lat, 'radius_m': radius_m})

        # ── Console output ─────────────────────────────────────────────────────
        if debug_infos:
            print(f"\n── Frame: {len(debug_infos)} blob(s) passed shape filter "
                  f"({len(detections)} accepted) ──", flush=True)
            for i, info in enumerate(debug_infos):
                base = (
                    f"  [{i+1}] "
                    f"pixel=({info['cx']},{info['cy']})  "
                    f"area={info['area']:.0f}px²  "
                    f"circ={info['circularity']:.2f}  "
                    f"asp={info['aspect']:.2f}  "
                    f"sol={info['solidity']:.2f}  "
                )
                if info['valid']:
                    print(
                        base +
                        f"fwd={info['x_fwd']:.2f}m  "
                        f"lat={info['y_lat']:.2f}m  "
                        f"r={info['radius_m']:.3f}m  "
                        f"ACCEPTED",
                        flush=True,
                    )
                else:
                    print(base + f"REJECTED ({info['reject_reason']})", flush=True)

        # ── Publish detections ─────────────────────────────────────────────────
        flat = []
        for x_fwd, y_lat, radius_m in detections:
            flat.extend([x_fwd, y_lat, radius_m])

        self._detections_pub.publish(Float32MultiArray(data=flat))

        # ── Debug overlay ──────────────────────────────────────────────────────
        self._debug(img, white_mask, detections, debug_infos)

    # =========================================================================
    #  DEBUG
    # =========================================================================

    def _debug(self, img: np.ndarray, white_mask: np.ndarray,
               detections: list, debug_infos: list) -> None:
        """
        Draw a colour-coded debug overlay and display it in an OpenCV window.

        Colour legend:
            Green — accepted pothole
            Red   — rejected (failed area / shape / sky / range)
        """
        if not (_HAS_DISPLAY and Debug.ENABLE_DISPLAY):
            return

        debug = cv2.cvtColor(white_mask, cv2.COLOR_GRAY2BGR)
        debug = cv2.addWeighted(debug, 0.4, img, 0.6, 0)

        for info in debug_infos:
            cx = info.get('cx')
            cy = info.get('cy')
            if cx is None:
                continue

            color = (0, 255, 0) if info['valid'] else (0, 0, 255)

            cv2.drawContours(debug, [info['cnt']], -1, color, 2)
            cv2.circle(debug, (cx, cy), 5, color, -1)

            if info['valid']:
                label = (
                    f"fwd={info['x_fwd']:.1f}m "
                    f"lat={info['y_lat']:.1f}m "
                    f"r={info['radius_m']:.2f}m"
                )
            else:
                label = (
                    f"A={info['area']:.0f} "
                    f"asp={info.get('aspect', 0):.1f} "
                    f"({info.get('reject_reason', '?')})"
                )
            cv2.putText(debug, label, (cx + 6, cy - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1)

        cv2.putText(
            debug,
            f"detected:{len(detections)}",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2,
        )

        cv2.imshow('Pothole Detection', debug)
        cv2.waitKey(1)


# ═════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = PotholeDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('pothole_detection_node shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()