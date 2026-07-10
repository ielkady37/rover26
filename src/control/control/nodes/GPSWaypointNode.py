#!/usr/bin/env python3
"""
GPSWaypointNode — converts lat/lon competition waypoints and the vehicle's
live GPS fix into a local ENU (x, y) frame anchored at an averaged GPS
origin fixed at boot.

Why this exists
────────────────
Nav2 / MissionManagerNode work in a flat local `map` frame of metres, not
lat/lon. GPS is only trustworthy to reach Waypoint 1 (the edge of the
lane-bounded track); after that, position must come from wheel encoders +
IMU dead reckoning (see ESPBridgeNode._publish_odom). This node performs
the one-time lat/lon -> x,y conversion for the 3 competition waypoints, and
continuously republishes the vehicle's *current* GPS position in that same
local frame, so it can be compared against Waypoint 1's target and fed to
Nav2 as a goal.

Frame convention
─────────────────
Local ENU tangent plane, origin = average of the first N valid /gps fixes
received at boot:
    x = East  (metres)
    y = North (metres)
This matches the `map` frame convention used for NavigateToPose goals in
MissionManagerNode.

Conversion method
──────────────────
Equirectangular (flat-earth) approximation around the origin latitude.
Error is on the order of centimetres over a few hundred metres, which is
well within the competition's 1.5 m waypoint tolerance. If you ever need
this to work over kilometres, switch to UTM or robot_localization's
navsat_transform_node instead.

Noise handling
───────────────
A bare GNSS module such as the Beitian BE-880 (u-blox based, no RTK) is
typically only accurate to ~2.5 m CEP standalone, so both the origin and
the live position benefit from averaging:
  - The origin is computed from the average of `origin_sample_count`
    consecutive fixes, rather than latching onto the very first fix
    (which is often the noisiest, before HDOP has settled).
  - The published /gps_xy position is smoothed with a simple moving
    average over `smoothing_window` fixes to reduce fix-to-fix jitter.
If you need to know why jitter is bad, log msg.status.status (SBAS vs
plain autonomous fix) and HDOP if your firmware exposes it — that will
tell you whether the noise is normal or a sign of a multipath/antenna
placement problem.

Published
─────────
    /waypoints_xy   (nav_msgs/Path)      the 3 competition waypoints converted to local x,y.
                                          Published once the origin is fixed, using a
                                          TRANSIENT_LOCAL QoS so late subscribers still get it.
    /gps_xy         (nav_msgs/Odometry)  the vehicle's live position in the same local x,y
                                          frame, derived purely from GPS and smoothed with a
                                          moving average. Use this ONLY to reach Waypoint 1 —
                                          after that, switch your position source to the
                                          encoder/IMU dead-reckoning odom.

Subscribed
──────────
    /gps            (sensor_msgs/NavSatFix)

Parameters
──────────
    waypoint_lats      (list[float], [])       latitudes  of WP1, WP2, WP3 in order
    waypoint_lons      (list[float], [])       longitudes of WP1, WP2, WP3 in order
    waypoint_ids       (list[str],   [])       optional labels, e.g. ["WP1","WP2","WP3"]
    map_frame          (str, 'map')
    base_frame         (str, 'base_link')
    wp1_tolerance_m    (float, 1.5)            logs a "within tolerance" message for WP1
    origin_sample_count (int, 10)              number of fixes averaged to fix the origin
    smoothing_window    (int, 5)               number of fixes averaged for /gps_xy output
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

# WGS84 mean earth radius, metres — fine for the equirectangular approximation
_R_EARTH = 6378137.0


def latlon_to_local_xy(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
    """Equirectangular projection of (lat, lon) onto a local ENU plane
    tangent at (lat0, lon0). Returns (x_east, y_north) in metres."""
    lat_r, lon_r = math.radians(lat), math.radians(lon)
    lat0_r, lon0_r = math.radians(lat0), math.radians(lon0)
    x = (lon_r - lon0_r) * math.cos(lat0_r) * _R_EARTH
    y = (lat_r - lat0_r) * _R_EARTH
    return x, y


class GPSWaypointNode(Node):
    def __init__(self) -> None:
        super().__init__("gps_waypoint_node")

        # parameters
        self.declare_parameter("waypoint_lats", [0.0])
        self.declare_parameter("waypoint_lons", [0.0])
        self.declare_parameter("waypoint_ids", [""])
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("wp1_tolerance_m", 1.5)
        self.declare_parameter("origin_sample_count",50)
        self.declare_parameter("smoothing_window", 20)

        self._lats = list(self.get_parameter("waypoint_lats").value)
        self._lons = list(self.get_parameter("waypoint_lons").value)
        ids = list(self.get_parameter("waypoint_ids").value)
        self._ids = ids if len(ids) == len(self._lats) else [
            f"WP{i + 1}" for i in range(len(self._lats))
        ]
        self._map_frame = self.get_parameter("map_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._wp1_tol = float(self.get_parameter("wp1_tolerance_m").value)
        self._origin_sample_count = max(1, int(self.get_parameter("origin_sample_count").value))
        self._smoothing_window = max(1, int(self.get_parameter("smoothing_window").value))

        if len(self._lats) != len(self._lons):
            self.get_logger().error(
                "waypoint_lats and waypoint_lons must be the same length — "
                f"got {len(self._lats)} lats and {len(self._lons)} lons."
            )

        # latched publisher: MissionManager (or anything else) can start after
        # this node and still receive the one-time waypoint conversion
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._waypoints_pub = self.create_publisher(Path, "/waypoints_xy", latched_qos)
        self._gps_xy_pub = self.create_publisher(Odometry, "/gps_xy", 10)

        self._gps_sub = self.create_subscription(NavSatFix, "/gps", self._gps_cb, 10)

        self._origin: tuple[float, float] | None = None
        self._origin_samples: list[tuple[float, float]] = []
        self._wp1_xy: tuple[float, float] | None = None
        self._wp1_notified = False

        # rolling window of recent (x, y) fixes for moving-average smoothing
        self._xy_window: deque[tuple[float, float]] = deque(maxlen=self._smoothing_window)

        self.get_logger().info(
            f"GPSWaypointNode up — averaging {self._origin_sample_count} fixes to fix the "
            f"local origin, smoothing /gps_xy over {self._smoothing_window} fixes "
            f"({len(self._lats)} waypoints loaded)."
        )

    # /gps

    def _gps_cb(self, msg: NavSatFix) -> None:
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        if self._origin is None:
            self._origin_samples.append((msg.latitude, msg.longitude))
            if len(self._origin_samples) < self._origin_sample_count:
                return  # keep collecting samples before locking the origin

            lat0 = sum(s[0] for s in self._origin_samples) / len(self._origin_samples)
            lon0 = sum(s[1] for s in self._origin_samples) / len(self._origin_samples)
            self._origin = (lat0, lon0)
            self.get_logger().info(
                f"GPS origin fixed at lat={lat0:.7f}, lon={lon0:.7f} "
                f"(averaged over {len(self._origin_samples)} fixes)"
            )
            self._origin_samples.clear()  # free the buffer, no longer needed
            self._publish_waypoints_xy()

        self._publish_gps_xy(msg)

    # one-time waypoint conversion

    def _publish_waypoints_xy(self) -> None:
        lat0, lon0 = self._origin
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self._map_frame

        for lat, lon, wp_id in zip(self._lats, self._lons, self._ids):
            x, y = latlon_to_local_xy(lat, lon, lat0, lon0)
            pose = PoseStamped()
            pose.header.stamp = path.header.stamp
            pose.header.frame_id = self._map_frame
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # no heading info from lat/lon alone
            path.poses.append(pose)
            self.get_logger().info(f"{wp_id}: lat={lat:.7f}, lon={lon:.7f} -> x={x:.2f} m, y={y:.2f} m")

        if path.poses:
            self._wp1_xy = (path.poses[0].pose.position.x, path.poses[0].pose.position.y)

        self._waypoints_pub.publish(path)

    # live GPS position, in the same local frame, smoothed with a moving average

    def _publish_gps_xy(self, fix: NavSatFix) -> None:
        lat0, lon0 = self._origin
        x_raw, y_raw = latlon_to_local_xy(fix.latitude, fix.longitude, lat0, lon0)

        self._xy_window.append((x_raw, y_raw))
        x = sum(p[0] for p in self._xy_window) / len(self._xy_window)
        y = sum(p[1] for p in self._xy_window) / len(self._xy_window)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self._map_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        # NOTE: a single GPS fix carries no heading — fuse yaw from /euler
        # (IMU) downstream rather than relying on this orientation field.
        odom.pose.pose.orientation.w = 1.0
        self._gps_xy_pub.publish(odom)

        if self._wp1_xy is not None and not self._wp1_notified:
            dist = math.hypot(x - self._wp1_xy[0], y - self._wp1_xy[1])
            if dist <= self._wp1_tol:
                self._wp1_notified = True
                self.get_logger().info(
                    f"Within WP1 tolerance ({dist:.2f} m <= {self._wp1_tol:.2f} m) — "
                    f"safe to switch position source to encoder/IMU dead reckoning."
                )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GPSWaypointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()