"""
═════════════════════════════════════════════════════════════════════════════════
generic_points_publisher_node.py  —  rover26_autonomy
═════════════════════════════════════════════════════════════════════════════════

FUNCTIONALITY
─────────────
Converts lane polynomial detections from lane_detection_node into ground-plane
point clouds and a synthetic LaserScan for Nav2 integration. Publishes detected
lane edges as RViz visualization markers and a synthetic /detection/scan meant
to be added as a SECOND observation source on Nav2's local costmap (alongside
the real LiDAR /scan) — not merged into a single scan topic.

PIPELINE (per cycle):
  1. Cache latest LaneDetectionResult message (polynomial coefficients)
  2. Sample polynomial(s) at regular intervals to produce ground-plane points
  3. On SCAN_HZ timer: publish synthetic LaserScan (/detection/scan)
  4. On MARKER_HZ timer: publish RViz markers and PointCloud2 for debug

Key Design:
  • Scan published at constant SCAN_HZ rate (20 Hz) so a downstream consumer
    (e.g. Nav2's local costmap obstacle layer) is never starved.
  • Markers/clouds published at lower MARKER_HZ rate (5 Hz) for bandwidth efficiency.
  • Published in LIDAR_FRAME so it lines up with the real /scan when both are
    added as observation sources on the same costmap.
  • Publishes whichever lane side IS currently detected — does not require
    both sides to be present. A side with fewer than 3 coefficients is
    treated as "not detected" and simply contributes no points this cycle.

TOPICS & SUBSCRIPTIONS
──────────────────────
Subscribed:
  • /lane_detection  [interfaces/LaneDetectionResult]  ← lane_detection_node

Published:
  • /detection/scan     [sensor_msgs/LaserScan]         → Nav2 local costmap (2nd observation source)
  • /detection/markers  [visualization_msgs/MarkerArray] → RViz (MARKER_HZ)
  • /detection/points   [sensor_msgs/PointCloud2]       → RViz (MARKER_HZ)

TF FRAMES
─────────
Input frame:  N/A (LaneDetectionResult contains relative polynomials in bird's-eye space)
Output frame: lidar_link (for /detection/scan — aligns with hardware LiDAR)
              base_footprint (for markers/points — RViz visualization)

PARAMETERS (from config_params.py)
──────────────────────────────────
LanePublisher.*  Publishing rates, visualization settings, coordinate frames
Physical.*       Ground plane extent, pixel-to-metre scaling

═════════════════════════════════════════════════════════════════════════════════
"""

import math
from dataclasses import dataclass, field

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from interfaces.msg import LaneDetectionResult

# All tunable constants come from the master config — no magic numbers here.
from rover26_autonomy.config_params import IMG_W, IMG_H, LanePublisher, Physical, RosTopics


# =============================================================================
# INTERNAL DATA TYPE
# =============================================================================

@dataclass
class DetectionPoint:
    """A single ground-plane detection point with optional metadata."""
    x: float
    y: float
    z: float = 0.0
    confidence: float = 0.85
    metadata: dict = field(default_factory=dict)


# =============================================================================
# COORDINATE CONVERSION
# =============================================================================

def pixel_to_ground(px: float, py: float) -> tuple[float, float]:
    """
    Convert a bird's-eye pixel coordinate to robot-body ground coordinates.

    Output convention matches base_footprint: x = forward, y = left.
    """
    x_forward = (1.0 - py / IMG_H) * Physical.GROUND_HEIGHT_M * 0.95
    y_left    = (0.5 - px / IMG_W) * Physical.GROUND_WIDTH_M
    return x_forward, y_left


def handle_lane(msg: LaneDetectionResult) -> list[DetectionPoint]:
    """
    Convert polynomial lane coefficients from a LaneDetectionResult message
    into a list of DetectionPoints in base_footprint ground coordinates.

    A side is treated as "not detected" if its coeff list isn't length 3 —
    lane_detection_node is expected to publish [] for a side it couldn't fit.
    Publishes whichever side IS available; does not require both.
    """
    points = []

    left_c  = np.array(msg.left_lane_coeffs)  if len(msg.left_lane_coeffs)  == 3 else None
    right_c = np.array(msg.right_lane_coeffs) if len(msg.right_lane_coeffs) == 3 else None

    if left_c is None and right_c is None:
        return points

    # Sample evenly along the image height (skip top 15% and bottom 10%
    # to avoid warp distortion near the horizon and rover body)
    # Note: the lane detection node already applies a mask to ignore these regions.
    y_vals = np.linspace(0, IMG_H, LanePublisher.N_LANE_SAMPLES)

    def process_side(coeffs, side: str):
        if coeffs is None:
            return
        x_vals = np.polyval(coeffs, y_vals)
        for px, py in zip(x_vals, y_vals):
            if not (0 <= px < IMG_W):
                continue
            x_fwd, y_lat = pixel_to_ground(px, py)
            # Small 0.12 m longitudinal offset places the lane origin just
            # ahead of the rover's footprint to avoid self-occlusion.
            points.append(DetectionPoint(
                x=x_fwd + 0.12,
                y=y_lat,
                metadata={'side': side}
            ))

    process_side(left_c,  'left')
    process_side(right_c, 'right')
    return points


# =============================================================================
# NODE
# =============================================================================

class GenericPointsPublisher(Node):
    """
    Bridges lane detection output to Nav2's costmap and RViz.

    Subscribes to /lane_detection and publishes:
      • /detection/scan    (LaserScan,   LIDAR_FRAME, SCAN_HZ)   — 2nd costmap observation source
      • /detection/points  (PointCloud2, ROBOT_FRAME, MARKER_HZ) — for RViz debugging
      • /detection/markers (MarkerArray, ROBOT_FRAME, MARKER_HZ) — for RViz debugging

    The scan is published on a fast timer so the costmap's obstacle layer is
    never starved between lane detection frames (which may arrive at a lower rate).
    """

    def __init__(self):
        super().__init__('generic_points_publisher')

        # ── Cached state ──────────────────────────────────────────────────────
        # The lane callback writes here; the timers read it.
        # This decouples publish rate from detection rate.
        self._latest_points: list[DetectionPoint] = []

        # Diagnostic counters (reset every 5 s in _log_rates)
        self._lane_msg_count = 0
        self._scan_pub_count = 0

        # ── Publishers ────────────────────────────────────────────────────────
        # LaserScan uses RELIABLE QoS so the costmap never drops a frame.
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=20
        )
        self._pub_markers = self.create_publisher(MarkerArray, '/detection/markers', 10)
        self._pub_cloud   = self.create_publisher(PointCloud2, '/detection/points',  10)
        self._pub_scan    = self.create_publisher(LaserScan,   RosTopics.LANE_SCAN,    scan_qos)

        # ── Subscription ──────────────────────────────────────────────────────
        # RELIABLE to match lane_detection_node's /lane_detection publisher.
        lane_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=15
        )
        self.create_subscription(
            LaneDetectionResult, RosTopics.LANE_DETECTION, self._lane_callback, lane_qos
        )

        # ── Timers ────────────────────────────────────────────────────────────
        # Fast timer: publishes /detection/scan at SCAN_HZ even between frames.
        self.create_timer(1.0 / LanePublisher.SCAN_HZ,   self._scan_timer)
        # Slow timer: RViz markers and PointCloud2 — 5 Hz is more than enough.
        self.create_timer(1.0 / LanePublisher.MARKER_HZ, self._marker_timer)
        # Diagnostic timer: logs actual Hz every 5 s.
        self.create_timer(5.0, self._log_rates)

        self.get_logger().info(
            f'Generic Points Publisher started — '
            f'scan: {LanePublisher.SCAN_HZ} Hz ({LanePublisher.LIDAR_FRAME}) | '
            f'markers/cloud: {LanePublisher.MARKER_HZ} Hz ({LanePublisher.ROBOT_FRAME})'
        )

    # ── Callbacks & timers ────────────────────────────────────────────────────

    def _lane_callback(self, msg: LaneDetectionResult):
        """Convert incoming LaneDetectionResult polynomials to ground points and cache them."""
        try:
            self._latest_points = handle_lane(msg)
            self._lane_msg_count += 1
        except Exception as e:
            self.get_logger().error(f'Lane callback error: {e}')

    def _scan_timer(self):
        """
        Publish /detection/scan at SCAN_HZ from the cached point list.

        Always publishes (even when empty) so a downstream consumer relying
        on a steady rate is never starved.
        """
        stamp = self.get_clock().now().to_msg()
        self._pub_scan.publish(self._build_laser_scan(self._latest_points, stamp))
        self._scan_pub_count += 1

    def _marker_timer(self):
        """Publish RViz markers and PointCloud2 for debug visualisation."""
        if not self._latest_points:
            return
        stamp = self.get_clock().now().to_msg()
        self._pub_markers.publish(self._build_markers(self._latest_points, stamp))
        cloud = self._build_pointcloud(self._latest_points, stamp)
        if cloud is not None:
            self._pub_cloud.publish(cloud)

    def _log_rates(self):
        """Log actual publish rates every 5 s to catch timer drift early."""
        self.get_logger().info(
            f'/lane_detection in:    {self._lane_msg_count / 5.0:.1f} Hz | '
            f'/detection/scan out:   {self._scan_pub_count / 5.0:.1f} Hz '
            f'(target {LanePublisher.SCAN_HZ} Hz)'
        )
        self._lane_msg_count = 0
        self._scan_pub_count = 0

    # ── Message builders ──────────────────────────────────────────────────────

    def _build_laser_scan(self, points: list[DetectionPoint], stamp) -> LaserScan:
        """
        Build a full-circle LaserScan from ground DetectionPoints.

        Published in LIDAR_FRAME so it aligns with the physical /scan when
        both are used as observation sources on the same costmap.
        Each angular bin keeps only the closest point (nearest-wins).
        Empty bins are set to inf so the costmap drops them cleanly.
        """
        scan = LaserScan()
        scan.header.stamp    = stamp
        scan.header.frame_id = LanePublisher.LIDAR_FRAME

        scan.angle_min       = -math.pi
        scan.angle_max       =  math.pi
        scan.angle_increment = (2 * math.pi) / LanePublisher.N_SCAN_BINS

        scan.scan_time      = 1.0 / LanePublisher.SCAN_HZ
        scan.time_increment = scan.scan_time / LanePublisher.N_SCAN_BINS
        scan.range_min      = 0.1
        scan.range_max      = 12.0

        # Initialise all bins to infinity (costmap drops inf ranges when use_inf=False)
        scan.ranges      = [float('inf')] * LanePublisher.N_SCAN_BINS
        scan.intensities = [0.0]         * LanePublisher.N_SCAN_BINS

        for p in points:
            distance = math.hypot(p.x, p.y)
            if not (scan.range_min <= distance <= scan.range_max):
                continue

            # Convert (x_forward, y_left) to a bearing angle, then to a bin index
            angle = math.atan2(p.y, p.x)
            idx   = int((angle - scan.angle_min) / scan.angle_increment)
            idx   = max(0, min(idx, LanePublisher.N_SCAN_BINS - 1))

            # Keep the closest detection per bin
            if distance < scan.ranges[idx]:
                scan.ranges[idx]      = distance
                scan.intensities[idx] = 900.0

        return scan

    def _build_markers(self, points: list[DetectionPoint], stamp) -> MarkerArray:
        """
        Build LINE_STRIP markers for left (green) and right (red) lane edges.

        Published in ROBOT_FRAME (base_footprint) so RViz places them in front
        of the rover, not at the lidar sensor position.
        """
        marker_array = MarkerArray()
        left_pts  = [p for p in points if p.metadata.get('side') == 'left']
        right_pts = [p for p in points if p.metadata.get('side') == 'right']

        def make_strip(pts: list[DetectionPoint], ns: str, marker_id: int,
                       r: float, g: float, b: float):
            if len(pts) < 2:
                return
            m = Marker()
            m.header.stamp    = stamp
            m.header.frame_id = LanePublisher.ROBOT_FRAME
            m.ns    = ns
            m.id    = marker_id
            m.type  = Marker.LINE_STRIP
            m.scale.x = LanePublisher.MARKER_LINE_WIDTH_M
            m.color   = ColorRGBA(r=r, g=g, b=b, a=0.95)
            # Sort front-to-back so the LINE_STRIP doesn't jump around
            for p in sorted(pts, key=lambda pt: pt.x):
                m.points.append(Point(x=p.x, y=p.y, z=p.z + LanePublisher.MARKER_Z_LIFT_M))
            marker_array.markers.append(m)

        make_strip(left_pts,  'lane_left',  0, 0.0, 1.0, 0.0)  # green
        make_strip(right_pts, 'lane_right', 1, 1.0, 0.0, 0.0)  # red
        return marker_array

    def _build_pointcloud(self, points: list[DetectionPoint], stamp) -> PointCloud2 | None:
        """
        Build an XYZI PointCloud2 for RViz debugging.

        Published in ROBOT_FRAME (base_footprint) — same reason as markers.
        Returns None when the point list is empty so callers can skip publish.
        """
        if not points:
            return None

        # Flatten to [x, y, z, intensity, x, y, z, intensity, ...] for tobytes()
        point_data = []
        for p in points:
            point_data.extend([p.x, p.y, p.z, p.confidence])

        cloud = PointCloud2()
        cloud.header.stamp    = stamp
        cloud.header.frame_id = LanePublisher.ROBOT_FRAME
        cloud.height       = 1
        cloud.width        = len(points)
        cloud.is_bigendian = False
        cloud.is_dense     = True
        cloud.point_step   = 16                            # 4 fields × 4 bytes (float32)
        cloud.row_step     = cloud.point_step * cloud.width
        cloud.fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.data = np.array(point_data, dtype=np.float32).tobytes()
        return cloud


# =============================================================================
# ENTRY POINT
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = GenericPointsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down generic_points_publisher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()