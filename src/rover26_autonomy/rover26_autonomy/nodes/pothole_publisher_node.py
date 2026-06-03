"""
═════════════════════════════════════════════════════════════════════════════════
pothole_publisher_node.py  —  rover26_autonomy
═════════════════════════════════════════════════════════════════════════════════

FUNCTIONALITY
─────────────
Receives pothole detections as (x, y, radius) triplets from any upstream
source and publishes them as a filled disk PointCloud2 for Nav2's costmap.

This node is intentionally decoupled from detection logic — any node can
feed it coordinates by publishing a flat Float32MultiArray to /pothole_detections
in the format [x1, y1, r1, x2, y2, r2, ...].

PIPELINE (per message)
──────────────────────
  1. Receive flat Float32MultiArray — unpack into (x, y, radius) triplets
  2. Expand each triplet into a filled disk of concentric point rings
  3. Publish the combined disk cloud to /pothole_obstacles
  4. Publish an empty cloud when no detections arrive (clears costmap raytrace)

DISK GENERATION
───────────────
Each pothole is represented as a filled disk of concentric rings sized to its
detected radius. Nav2's VoxelLayer marks every cell that receives a point, so
larger potholes produce proportionally larger cost regions without relying
entirely on costmap inflation.

Requires  potholes: clearing: True  in nav2_params.yaml so the costmap
raytrace clears stale marks when an empty cloud is received.

TOPICS & SUBSCRIPTIONS
──────────────────────
Subscribed:
  • /pothole_detections  [std_msgs/Float32MultiArray]  ← any detection source
    Format: flat array [x1, y1, r1, x2, y2, r2, ...]  (metres, base_link frame)

Published:
  • /pothole_obstacles   [sensor_msgs/PointCloud2]      → Nav2 costmap

TF FRAMES
─────────
All points published in 'base_link'. Nav2 silently drops clouds with
untransformable frames, and 'base_link' always has a valid TF from
odom_tf_broadcaster.

PARAMETERS (from config_params.py)
──────────────────────────────────
PotholeDetection.COSTMAP_PUBLISH_M  — only potholes within this range enter the costmap
PotholeDetection.MIN_DETECTION_M    — potholes closer than this are ignored

═════════════════════════════════════════════════════════════════════════════════
"""

import math
import struct

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, Header

from ..config_params import PotholeDetection, RosTopics


# ═════════════════════════════════════════════════════════════════════════════
#  HELPERS
# ═════════════════════════════════════════════════════════════════════════════

def _pothole_disk(x_fwd: float, y_lat: float,
                  radius_m: float,
                  n_rings: int = 2,
                  z: float = 0.05) -> list:
    """
    Generate a filled disk of (x, y, z) points for one pothole.

    Args:
        x_fwd:    Centre — metres ahead of rover (+X).
        y_lat:    Centre — metres lateral (+Y = left).
        radius_m: Pothole radius in metres.
        n_rings:  Concentric rings to fill (0 = centre only).
        z:        Point height in metres — must be within
                  [min_obstacle_height, max_obstacle_height] in nav2_params.yaml.

    Returns:
        List of (x, y, z) tuples.
    """
    points = [(x_fwd, y_lat, z)]

    for ring in range(1, n_rings + 1):
        r = radius_m * (ring / n_rings)

        circumference = 2.0 * math.pi * r
        n_pts = max(8, int(circumference / 0.025))

        for i in range(n_pts):
            angle = 2.0 * math.pi * i / n_pts
            points.append((
                x_fwd + r * math.cos(angle),
                y_lat + r * math.sin(angle),
                z,
            ))

    return points


def _build_pointcloud2(points_xyz: list, stamp) -> PointCloud2:
    """
    Pack a list of (x, y, z) float tuples into a sensor_msgs/PointCloud2.

    Args:
        points_xyz: List of (x, y, z) tuples in metres (base_link frame).
        stamp:      ROS timestamp for the cloud header.

    Returns:
        A fully populated PointCloud2 message.
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
    header.frame_id = 'base_footprint'
    header.stamp    = stamp

    cloud              = PointCloud2()
    cloud.header       = header
    cloud.height       = 1
    cloud.width        = len(points_xyz)
    cloud.fields       = fields
    cloud.is_bigendian = False
    cloud.point_step   = 12
    cloud.row_step     = 12 * len(points_xyz)
    cloud.data         = bytes(data)
    cloud.is_dense     = True

    return cloud


# ═════════════════════════════════════════════════════════════════════════════
#  NODE
# ═════════════════════════════════════════════════════════════════════════════

class PotholePublisherNode(Node):
    """
    Converts (x, y, radius) pothole detections into a filled disk PointCloud2
    and publishes it for Nav2's costmap.

    Accepts detections from any upstream source via /pothole_detections as a
    flat Float32MultiArray [x1, y1, r1, x2, y2, r2, ...]. Publishes an empty
    cloud every frame when no detections arrive so the costmap raytrace clears
    any stale obstacle marks.
    """

    # Minimum distance (metres) a new detection must be from ALL known potholes
    # before it is treated as a new, distinct pothole rather than a re-detection.
    _DEDUP_DIST_M: float = 1.0

    def __init__(self):
        super().__init__('pothole_publisher_node')

        self.create_subscription(
            Float32MultiArray,
            RosTopics.POTHOLE_DETECTIONS,
            self._detections_cb,
            10,
        )

        self._cloud_pub = self.create_publisher(
            PointCloud2, RosTopics.POTHOLE_OBSTACLES, 10
        )

        # Stores (x_fwd, y_lat) of every pothole already published to the costmap.
        # A new detection is only forwarded if it is > _DEDUP_DIST_M away from
        # every entry in this list.
        self._known_potholes: list = []

        self.get_logger().info(
            'pothole_publisher_node ready\n'
            f'  input  : {RosTopics.POTHOLE_DETECTIONS}  [Float32MultiArray]\n'
            f'  output : {RosTopics.POTHOLE_OBSTACLES}   [PointCloud2]\n'
            f'  costmap gate : ≤ {PotholeDetection.COSTMAP_PUBLISH_M} m\n'
            f'  dedup radius : {PotholePublisherNode._DEDUP_DIST_M} m'
        )

    # =========================================================================
    #  CALLBACK
    # =========================================================================

    def _detections_cb(self, msg: Float32MultiArray) -> None:
        """
        Receive a flat list of (x, y, radius) triplets and publish a disk cloud.

        Silently ignores malformed messages (length not a multiple of 3).
        Publishes an empty cloud when the detection list is empty so the
        costmap raytrace clears stale marks.

        Args:
            msg: Flat Float32MultiArray [x1, y1, r1, x2, y2, r2, ...] in metres.
        """
        data = msg.data

        if len(data) % 3 != 0:
            self.get_logger().warn(
                f'Received malformed detection array (length {len(data)} is not '
                f'a multiple of 3). Message ignored.'
            )
            return

        triplets = [(data[i], data[i + 1], data[i + 2]) for i in range(0, len(data), 3)]

        disk_points: list = []
        for x_fwd, y_lat, radius_m in triplets:
            if not (PotholeDetection.MIN_DETECTION_M < x_fwd <= PotholeDetection.COSTMAP_PUBLISH_M):
                continue

            # Deduplicate — skip if this detection is close to an already-known pothole.
            too_close = any(
                math.hypot(x_fwd - kx, y_lat - ky) < self._DEDUP_DIST_M
                for kx, ky in self._known_potholes
            )
            if too_close:
                continue

            self._known_potholes.append((x_fwd, y_lat))
            self.get_logger().info(
                f'[pothole] NEW pothole registered at fwd={x_fwd:.2f}m  lat={y_lat:.2f}m  r={radius_m:.2f}m  '
                f'(total known: {len(self._known_potholes)})'
            )
            disk_points.extend(_pothole_disk(x_fwd, y_lat, radius_m))

        stamp = self.get_clock().now().to_msg()
        self._cloud_pub.publish(_build_pointcloud2(disk_points, stamp))


# ═════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = PotholePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('pothole_publisher_node shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()