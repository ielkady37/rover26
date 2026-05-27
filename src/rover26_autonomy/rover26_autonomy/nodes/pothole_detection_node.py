"""
═════════════════════════════════════════════════════════════════════════════════
pothole_detection_node.py  —  rover26_autonomy
═════════════════════════════════════════════════════════════════════════════════

FUNCTIONALITY
─────────────
Detects white pothole blobs via the forward camera and publishes them as a
PointCloud2 so Nav2's costmap marks them as obstacles. Uses color-based blob
detection with shape filtering to distinguish potholes from lane-line fragments.

PIPELINE (per frame)
────────────────────
  1. White-pixel mask extraction via HLS-based thresholding (VisionUtils)
  2. Contour detection and area filtering
  3. Shape descriptor filtering (aspect ratio, solidity, circularity)
  4. Lane-bounds filtering — only accept blobs inside the road corridor
  5. Ground-plane projection — bird's-eye pixel → metres ahead / lateral
     Radius estimated from warped contour area → disk of points published
  6. Two-tier publication:
       Speed hint  — ALL detections up to MAX_DETECTION_M
       Costmap     — disk point clouds only for potholes within COSTMAP_PUBLISH_M
  7. Debug overlay (headless-safe, skipped when $DISPLAY is unset)

GROUND PROJECTION FORMULA
─────────────────────────
Distance is computed from the bird's-eye view pixel position using the
physical ground-plane calibration constants (GROUND_HEIGHT_M, GROUND_WIDTH_M)
which describe how many real-world metres the warped image covers:

    x_fwd = (1.0 - by / img_h) * GROUND_HEIGHT_M
    y_lat = (0.5 - bx / img_w) * GROUND_WIDTH_M

This is the correct mapping: by=0 (top of BEV) → furthest ahead (GROUND_HEIGHT_M),
by=img_h (bottom) → rover's feet (0m).

NOTE: PX_PER_METRE (196 px/m) is a lane cross-track-error calibration constant
and must NOT be used for full-image distance projection. The actual bird's-eye
image scale is IMG_H / GROUND_HEIGHT_M = 48 px/m (vertical) and
IMG_W / GROUND_WIDTH_M = 71 px/m (lateral). Using PX_PER_METRE would make
every pothole appear ~4× closer than reality.

KEY DESIGN DECISIONS
────────────────────
Each pothole is published as a filled disk of points sized to its detected
radius, rather than a single point. This gives Nav2's costmap a graded cost
field proportional to real pothole size so MPPI can steer around it
accurately without relying entirely on inflation to do the work.

Detection range is split: potholes are spotted up to MAX_DETECTION_M away
for early speed reduction, but only enter Nav2's costmap at ≤COSTMAP_PUBLISH_M.
An empty cloud is published every frame when nothing qualifies, which causes
the costmap raytrace to clear any stale marks from previous frames.
Requires  potholes: clearing: True  in nav2_params.yaml.

TOPICS & SUBSCRIPTIONS
──────────────────────
Subscribed:
  • /camera/image    [sensor_msgs/Image]       ← camera driver
  • /lane_status     [rover26/LaneStatus]      ← lane_detection_node
  • /imu             [sensor_msgs/Imu]         ← IMU driver (reserved)

Published:
  • /pothole_obstacles  [sensor_msgs/PointCloud2]  → Nav2 costmap
  • /pothole_speed      [std_msgs/Float32]         → speed controller

TF FRAMES
─────────
All detections reported in frame 'base_link' — Nav2 silently drops clouds
with untransformable frames, and 'base_link' always has a valid TF from
odom_tf_broadcaster.

PARAMETERS (from config_params.py)
──────────────────────────────────
PotholeDetection.*   Blob area, shape thresholds, detection range, costmap gate,
                     speed-hint min/max factors
Physical.*           GROUND_HEIGHT_M, GROUND_WIDTH_M  (bird's-eye extent)
Debug.*              Verbosity, display mode

═════════════════════════════════════════════════════════════════════════════════
"""

import math
import os
import struct

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField
from std_msgs.msg import Float32, Header
from rover26.msg import LaneStatus

from ..utils.vision_utils import VisionUtils
from ..config_params import IMG_W, IMG_H, GROUND_HEIGHT_M, GROUND_WIDTH_M, PotholeDetection, Physical, Debug, RosTopics


# ═════════════════════════════════════════════════════════════════════════════
#  DISPLAY GUARD
#  Only open OpenCV windows when a display server is present (skip on headless).
# ═════════════════════════════════════════════════════════════════════════════
_HAS_DISPLAY = bool(os.environ.get('DISPLAY', ''))

# Actual bird's-eye image scale derived from physical ground calibration.
# These are used for area → metres² conversion in the radius estimator.
# Do NOT use PX_PER_METRE here — that constant is calibrated for lane
# cross-track error, not full-image distance projection.
_BEV_PX_PER_M_X: float = IMG_H / GROUND_HEIGHT_M   # ~48 px/m along heading axis
_BEV_PX_PER_M_Y: float = IMG_W / GROUND_WIDTH_M    # ~71 px/m along lateral axis


# ═════════════════════════════════════════════════════════════════════════════
#  MODULE-LEVEL HELPERS
# ═════════════════════════════════════════════════════════════════════════════

def _birdseye_to_ground(bx: float, by: float,
                        img_w: int = IMG_W,
                        img_h: int = IMG_H) -> tuple:
    """
    Convert a bird's-eye pixel (bx, by) to ground-plane metric coordinates.

    Uses the physical ground-plane calibration (GROUND_HEIGHT_M, GROUND_WIDTH_M)
    to map the full image extent to real-world metres. This is a linear mapping:
      by = 0      → top of BEV image → furthest ahead  → x_fwd = GROUND_HEIGHT_M
      by = img_h  → bottom of image  → rover's feet    → x_fwd = 0
      bx = 0      → left edge        → y_lat = +GROUND_WIDTH_M / 2
      bx = img_w  → right edge       → y_lat = -GROUND_WIDTH_M / 2

    Args:
        bx:    Bird's-eye pixel x (horizontal, 0 = left).
        by:    Bird's-eye pixel y (vertical,   0 = top = furthest ahead).
        img_w: Warped image width  in pixels (default IMG_W = 640).
        img_h: Warped image height in pixels (default IMG_H = 480).

    Returns:
        (x_fwd_m, y_lat_m) — metres ahead (+X) and lateral (+Y = left),
        or (None, None) if outside the configured detection range.
    """
    x_fwd = (1.0 - by / img_h) * GROUND_HEIGHT_M   # Linear map: top→far, bottom→near
    y_lat = (0.5 - bx / img_w) * GROUND_WIDTH_M    # Linear map: left→+, right→-

    if not (PotholeDetection.MIN_DETECTION_M < x_fwd < PotholeDetection.MAX_DETECTION_M):
        return None, None

    return x_fwd, y_lat


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


def _validate_contour_properties(cnt) -> dict | None:
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
        'area': area,
        'aspect': aspect,
        'solidity': solidity,
        'circularity': circularity,
    }


def _pothole_disk(x_fwd: float, y_lat: float,
                  radius_m: float,
                  n_rings: int = 2,
                  z: float = 0.05) -> list:
    """
    Generate a filled disk of (x, y, z) points representing one pothole.

    Publishes concentric rings so the costmap receives a filled area sized to
    the real pothole, rather than a single point that relies entirely on
    inflation. Nav2's VoxelLayer marks every cell that receives a point, so
    larger potholes produce proportionally larger cost regions in the costmap.

    Args:
        x_fwd:    Pothole centre — metres ahead of rover (+X).
        y_lat:    Pothole centre — metres lateral (+Y = left).
        radius_m: Estimated pothole radius in metres (from warped contour area).
        n_rings:  Number of concentric rings (0 = centre only, 1 = 50% r, 2 = full r).
        z:        Height of all points (metres). Must be within
                  [min_obstacle_height, max_obstacle_height] in nav2_params.yaml.

    Returns:
        List of (x, y, z) tuples ready for _build_pointcloud2().
    """
    points = [(x_fwd, y_lat, z)]   # Centre point always included

    for ring in range(1, n_rings + 1):
        r = radius_m * (ring / n_rings)   # Radius of this concentric ring

        # Enough points to avoid gaps at costmap resolution 0.03 m/cell.
        circumference = 2.0 * math.pi * r
        n_pts = max(8, int(circumference / 0.025))

        for i in range(n_pts):
            angle = 2.0 * math.pi * i / n_pts
            px = x_fwd + r * math.cos(angle)
            py = y_lat + r * math.sin(angle)
            points.append((px, py, z))

    return points


def _build_pointcloud2(points_xyz: list, frame_id: str, stamp) -> PointCloud2:
    """
    Pack a list of (x, y, z) float tuples into a sensor_msgs/PointCloud2 message.

    Frame ID must be 'base_link': Nav2 silently drops clouds with untransformable
    frames, and 'base_link' always has a valid TF from odom_tf_broadcaster.

    Args:
        points_xyz: List of (x_fwd, y_lat, z) tuples in metres.
        frame_id:   TF frame the points are expressed in.
        stamp:      ROS timestamp to stamp the cloud header with.

    Returns:
        A fully populated PointCloud2 message ready to publish.
    """
    fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]

    data = bytearray()
    for x, y, z in points_xyz:
        data += struct.pack('fff', float(x), float(y), float(z))

    header          = Header()
    header.frame_id = frame_id
    header.stamp    = stamp

    cloud              = PointCloud2()
    cloud.header       = header
    cloud.height       = 1
    cloud.width        = len(points_xyz)
    cloud.fields       = fields
    cloud.is_bigendian = False
    cloud.point_step   = 12                     # 3 × float32
    cloud.row_step     = 12 * len(points_xyz)
    cloud.data         = bytes(data)
    cloud.is_dense     = True

    return cloud


# ═════════════════════════════════════════════════════════════════════════════
#  POTHOLE DETECTION NODE
# ═════════════════════════════════════════════════════════════════════════════

class PotholeDetectionNode(Node):
    """
    ROS 2 node that detects white pothole blobs via the forward camera,
    projects them onto the ground plane, and publishes disk-shaped point clouds
    for Nav2's costmap.

    Each accepted pothole is represented as a filled disk of points whose
    radius is estimated from the warped contour area. This gives the MPPI
    controller a cost field proportional to the real pothole size.

    Key design decisions:
      • Detection range is split: potholes are spotted up to MAX_DETECTION_M away
        for early speed reduction, but only enter Nav2's costmap at ≤COSTMAP_PUBLISH_M.
      • Lane polynomial coefficients from /lane_status constrain the search
        to the road interior, rejecting blobs on road edges or verges.
      • An empty PointCloud2 is published every frame when no close potholes
        exist, allowing the costmap raytrace to clear any stale marks.
        Requires  potholes: clearing: True  in nav2_params.yaml.

    Attributes:
        bridge (CvBridge):          ROS image message ↔ OpenCV conversion.
        vision (VisionUtils):       Bird's-eye warping and white-mask extraction.
        last_left_coeffs  (ndarray | None): Cached left-lane polynomial  (a, b, c).
        last_right_coeffs (ndarray | None): Cached right-lane polynomial (a, b, c).
    """

    # =========================================================================
    #  CONSTRUCTOR
    # =========================================================================

    def __init__(self):
        super().__init__('pothole_detection_node')

        self.bridge = CvBridge()
        self.vision = VisionUtils()

        self.last_left_coeffs:  np.ndarray | None = None
        self.last_right_coeffs: np.ndarray | None = None

        # ── Subscriptions ──────────────────────────────────────────────────────
        self.create_subscription(Image,      RosTopics.CAMERA_IMAGE, self._image_cb, 10)
        self.create_subscription(LaneStatus, RosTopics.LANE_STATUS,  self._lane_cb,  10)
        self.create_subscription(Imu,        RosTopics.IMU,          self._imu_cb,   10)

        # ── Publishers ─────────────────────────────────────────────────────────
        self._cloud_pub = self.create_publisher(PointCloud2, RosTopics.POTHOLE_OBSTACLES, 10)
        self._speed_pub = self.create_publisher(Float32,     RosTopics.POTHOLE_SPEED,     10)

        if _HAS_DISPLAY and Debug.ENABLE_DISPLAY:
            cv2.namedWindow('Pothole Detection', cv2.WINDOW_NORMAL)

        self.get_logger().info(
            'pothole_detection_node ready\n'
            f'  ground extent  : {GROUND_HEIGHT_M} m ahead  {GROUND_WIDTH_M} m wide\n'
            f'  bev px/m       : {_BEV_PX_PER_M_X:.1f} (heading)  {_BEV_PX_PER_M_Y:.1f} (lateral)\n'
            f'  costmap gate   : ≤ {PotholeDetection.COSTMAP_PUBLISH_M} m  (disk points)\n'
            f'  speed hint     : up to {PotholeDetection.MAX_DETECTION_M} m\n'
            f'  area filter    : [{PotholeDetection.MIN_AREA}, {PotholeDetection.MAX_AREA}] px²\n'
            f'  shape filter   : circ=[{PotholeDetection.MIN_CIRCULARITY}, '
            f'{PotholeDetection.MAX_CIRCULARITY}]  '
            f'asp<{PotholeDetection.MAX_ASPECT_RATIO}  '
            f'sol>{PotholeDetection.MIN_SOLIDITY}'
        )

    # =========================================================================
    #  CALLBACKS
    # =========================================================================

    def _imu_cb(self, msg: Imu) -> None:
        """Reserved for future tilt-compensation logic (IMU yaw / pitch correction)."""
        pass

    def _lane_cb(self, msg: LaneStatus) -> None:
        """Cache the latest lane polynomial fits to restrict detection to the road interior."""
        self.last_left_coeffs  = np.array(msg.left_coeffs)  if msg.left_detected  else None
        self.last_right_coeffs = np.array(msg.right_coeffs) if msg.right_detected else None

    def _image_cb(self, msg: Image) -> None:
        """
        Receive a raw camera frame and run the full pothole detection pipeline.

        Works at native camera resolution so blob shapes are not distorted by
        forced resizing. Image dimensions h and w are derived from img.shape[:2].
        """
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._process(img, msg.header.stamp)

    # =========================================================================
    #  MAIN DETECTION PIPELINE
    # =========================================================================

    def _process(self, img: np.ndarray, stamp) -> None:
        """
        Run the full pothole detection pipeline on one BGR frame.

        Accepted potholes are tracked as (x_fwd, y_lat, radius_m) centre tuples.
        Two output groups are built from those centres:
          pothole_centers — all detections within MAX_DETECTION_M (speed hint)
          costmap_disks   — disk point clouds for potholes ≤ COSTMAP_PUBLISH_M

        Args:
            img:   Native-resolution BGR camera frame.
            stamp: ROS timestamp from the source image header.
        """
        h, w = img.shape[:2]

        # ── Step 1: White-pixel mask + contour extraction ──────────────────────
        # apply_roi=False: skip the SRC trapezoid crop so blobs anywhere in the
        # full camera frame are found. The lane polynomial bounds in Step 4
        # below act as the spatial gate instead of the hard-coded trapezoid.
        white_mask = self.vision.get_white_mask(img, apply_roi=False)
        conts, _   = cv2.findContours(
            white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        pothole_centers: list = []   # (x_fwd, y_lat, radius_m) per accepted pothole
        debug_infos:     list = []

        for cnt in conts:
            # ── Validate area and shape ────────────────────────────────────────
            props = _validate_contour_properties(cnt)
            if props is None:
                continue

            # ── Extract centroid ───────────────────────────────────────────────
            M_cnt = cv2.moments(cnt)
            if M_cnt['m00'] == 0:
                continue
            cx = int(M_cnt['m10'] / M_cnt['m00'])
            cy = int(M_cnt['m01'] / M_cnt['m00'])

            info = {
                'cnt': cnt,
                'cx': cx,
                'cy': cy,
                **props,
                'valid': False,
            }

            # ── Step 4: Lane-bounds filter ─────────────────────────────────────
            # Warp the centroid to bird's-eye space.
            # The perspective transform M was calibrated against a trapezoid
            # that only covers a sub-region of the raw frame (TOP_Y → IMG_H-220).
            # Centroids outside that region extrapolate beyond the BEV rectangle
            # and must be rejected — clamping them would project to x_fwd ≈ 0
            # (rover's feet) or beyond MAX_DETECTION_M, both wrong.
            pt_orig = np.float32([[[cx, cy]]])
            pt_bird = cv2.perspectiveTransform(pt_orig, self.vision.M)
            bx_raw = pt_bird[0][0][0]
            by_raw = pt_bird[0][0][1]

            # Reject if the warped point falls outside the calibrated BEV patch.
            if not (0.0 <= bx_raw <= w - 1 and 0.0 <= by_raw <= h - 1):
                continue

            bx = int(bx_raw)
            by = int(by_raw)

            left_x, right_x = self._lane_bounds(by, w)
            if not (left_x <= bx <= right_x):
                continue

            # ── Step 5: Ground projection + radius estimation ──────────────────
            x_fwd, y_lat = _birdseye_to_ground(bx, by, w, h)
            if x_fwd is None:
                continue

            # Estimate pothole radius from the contour area in bird's-eye space.
            # Warping the contour gives the correct bird's-eye area so that the
            # per-axis px/m scale factors (_BEV_PX_PER_M_X, _BEV_PX_PER_M_Y)
            # convert pixels² to metres² correctly.
            cnt_bird  = cv2.perspectiveTransform(
                cnt.astype(np.float32).reshape(-1, 1, 2), self.vision.M
            )
            area_bird = cv2.contourArea(cnt_bird)
            area_m2   = area_bird / (_BEV_PX_PER_M_X * _BEV_PX_PER_M_Y)
            radius_m  = float(np.clip(
                math.sqrt(max(area_m2, 1e-6) / math.pi),
                0.05,   # Minimum radius: 5 cm
                0.80,   # Maximum radius: 80 cm
            ))

            pothole_centers.append((x_fwd, y_lat, radius_m))
            info.update({'valid': True, 'x_fwd': x_fwd, 'y_lat': y_lat, 'radius_m': radius_m})
            debug_infos.append(info)

        # ── Step 6: Costmap gate — build disk point cloud ──────────────────────
        # Each qualifying pothole is expanded to a filled disk of points whose
        # radius matches the detected pothole size. An empty cloud is published
        # when nothing qualifies so the costmap raytrace clears stale marks.
        costmap_disks: list = []
        for x_fwd, y_lat, radius_m in pothole_centers:
            if x_fwd <= PotholeDetection.COSTMAP_PUBLISH_M:
                costmap_disks.extend(_pothole_disk(x_fwd, y_lat, radius_m))

        # Stamp with current sim time so the costmap TF lookup never fails due
        # to drift between the camera image timestamp and the costmap frame.
        cloud = _build_pointcloud2(
            costmap_disks,
            frame_id='base_link',
            stamp=self.get_clock().now().to_msg(),
        )
        self._cloud_pub.publish(cloud)

        # ── Step 7: Speed hint ─────────────────────────────────────────────────
        # Uses ALL detections up to MAX_DETECTION_M so the rover starts slowing
        # early. Speed is computed from centre distances so large disk radii
        # don't skew the result toward the near edge of the disk.
        if pothole_centers:
            closest = min(c[0] for c in pothole_centers)
            speed   = float(np.clip(
                closest / PotholeDetection.MAX_DETECTION_M,
                PotholeDetection.SPEED_HINT_MIN_FACTOR,
                PotholeDetection.SPEED_HINT_MAX_FACTOR,
            ))
        else:
            speed = PotholeDetection.SPEED_HINT_MAX_FACTOR

        self._speed_pub.publish(Float32(data=speed))

        # ── Step 8: Debug overlay ──────────────────────────────────────────────
        costmap_count = sum(
            1 for c in pothole_centers if c[0] <= PotholeDetection.COSTMAP_PUBLISH_M
        )
        self._debug(img, white_mask, pothole_centers, costmap_count, debug_infos)

    # =========================================================================
    #  HELPERS
    # =========================================================================

    def _lane_bounds(self, y: float, w: int) -> tuple:
        """
        Return the left and right lane x-pixel bounds at bird's-eye row y.

        Falls back to the full image width when no lane polynomials are available,
        so detection still works before the lane node has published its first fit.

        Args:
            y: Bird's-eye image row to evaluate the polynomials at.
            w: Image width in pixels (for the fallback clamp).

        Returns:
            (left_x, right_x) pixel bounds — always within [0, w].
        """
        if self.last_left_coeffs is None or self.last_right_coeffs is None:
            return 0.0, float(w)

        lx = float(np.polyval(self.last_left_coeffs,  y))
        rx = float(np.polyval(self.last_right_coeffs, y))

        return np.clip(lx, 0, w), np.clip(rx, 0, w)

    def _debug(self, img: np.ndarray, white_mask: np.ndarray,
               pothole_centers: list, costmap_count: int,
               debug_infos: list) -> None:
        """
        Draw a colour-coded debug overlay and display it in an OpenCV window.

        Colour legend:
            Green  — accepted, within COSTMAP_PUBLISH_M → disk published to costmap
            Yellow — accepted, held by the costmap distance gate (speed hint only)
            Red    — rejected (failed area / shape / lane-bounds / projection)
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

            if info['valid']:
                in_costmap = info['x_fwd'] <= PotholeDetection.COSTMAP_PUBLISH_M
                color      = (0, 255, 0) if in_costmap else (0, 200, 255)
            else:
                color = (0, 0, 255)

            cv2.drawContours(debug, [info['cnt']], -1, color, 2)
            cv2.circle(debug, (cx, cy), 5, color, -1)

            if info['valid']:
                in_costmap = info['x_fwd'] <= PotholeDetection.COSTMAP_PUBLISH_M
                tag   = '→costmap' if in_costmap else f'held>{PotholeDetection.COSTMAP_PUBLISH_M}m'
                label = (
                    f"fwd={info['x_fwd']:.1f}m "
                    f"lat={info['y_lat']:.1f}m "
                    f"r={info['radius_m']:.2f}m "
                    f"{tag}"
                )
            else:
                label = (
                    f"A={info['area']:.0f} "
                    f"asp={info.get('aspect', 0):.1f} "
                    f"sol={info.get('solidity', 0):.2f}"
                )
            cv2.putText(debug, label, (cx + 6, cy - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1)

        closest    = min((c[0] for c in pothole_centers), default=PotholeDetection.MAX_DETECTION_M)
        speed_disp = float(np.clip(
            closest / PotholeDetection.MAX_DETECTION_M,
            PotholeDetection.SPEED_HINT_MIN_FACTOR,
            PotholeDetection.SPEED_HINT_MAX_FACTOR,
        )) if pothole_centers else PotholeDetection.SPEED_HINT_MAX_FACTOR

        cv2.putText(
            debug,
            f"detected:{len(pothole_centers)}  costmap:{costmap_count}  speed×{speed_disp:.2f}",
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