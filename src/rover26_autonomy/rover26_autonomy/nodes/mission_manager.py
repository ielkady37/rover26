#!/usr/bin/env python3
"""
═════════════════════════════════════════════════════════════════════════════════
mission_manager.py  —  rover26_autonomy
═════════════════════════════════════════════════════════════════════════════════

FUNCTIONALITY
─────────────
Top-level mission state machine that arbitrates between lane-following
(lane_goal_publisher) and GPS waypoint navigation, both of which drive the
rover through the SAME navigate_to_pose action server.

STATE MACHINE
──────────────
  LANE_FOLLOW
      lane_goal_publisher is active (mission/lane_enable = True).
      Every /gps/fix is checked against WAYPOINT_1's lat/lon.
      On arrival within wp1_arrival_radius_m  →  WAYPOINT_NAV

  WAYPOINT_NAV
      1. Publish mission/lane_enable = False.
         lane_goal_publisher cancels its in-flight goal and goes idle —
         this is what frees the navigate_to_pose action server for us.
      2. Convert WAYPOINT_2's lat/lon → map frame via /fromLL
         (provided by navsat_transform_node), send NavigateToPose, wait.
      3. Same for WAYPOINT_3.
      4. Publish mission/lane_enable = True  →  back to LANE_FOLLOW.

  A mission_complete flag prevents re-triggering once the waypoint loop has
  been run (set repeat_each_lap:=true to re-arm automatically once the
  rover is wp1_rearm_radius_m away from WAYPOINT_1 again).

TOPICS & SERVICES
──────────────────
Subscribed:
  • /gps/fix  [sensor_msgs/NavSatFix]  ← navsat_transform_node / GPS bridge

Published:
  • /mission/lane_enable  [std_msgs/Bool]  → lane_goal_publisher
  • /mission/state        [std_msgs/String] → diagnostics / RViz

Service client:
  • /fromLL  [robot_localization/srv/FromLL]  → navsat_transform_node

Action client:
  • navigate_to_pose  [nav2_msgs/action/NavigateToPose]  → bt_navigator

PARAMETERS
──────────
  wp1_lat, wp1_lon, wp2_lat, wp2_lon, wp3_lat, wp3_lon  (float, degrees)
      GPS coordinates for the three waypoints. Defaults below are SIM
      placeholders computed from competition_track_world.sdf's
      <spherical_coordinates> anchor — replace with the real competition
      values when given (no code changes needed, just override via YAML
      or launch arguments).
  wp1_arrival_radius_m   (float, default 2.5)
      Distance to WAYPOINT_1 at which lane-follow hands off to GPS nav.
  wp1_rearm_radius_m     (float, default 8.0)
      Distance the rover must move away from WAYPOINT_1 before the
      handoff can re-trigger (only matters if repeat_each_lap is True).
  repeat_each_lap        (bool, default False)
      If True, run the WP2→WP3 detour every time the rover passes near
      WP1 again (e.g. multi-lap missions). If False, run it once.
  goal_settle_s          (float, default 0.5)
      Pause after disabling lane-follow before sending the first GPS
      nav goal, giving lane_goal_publisher's cancel a moment to land.

═════════════════════════════════════════════════════════════════════════════════
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoint
from nav2_msgs.action import NavigateToPose
from robot_localization.srv import FromLL


EARTH_RADIUS_M = 6378137.0


def haversine_m(lat1, lon1, lat2, lon2) -> float:
    """Great-circle distance in metres — good enough at competition scale."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(dlmb / 2) ** 2)
    return 2 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


class MissionManager(Node):

    STATE_LANE_FOLLOW = 'LANE_FOLLOW'
    STATE_WAYPOINT_NAV = 'WAYPOINT_NAV'

    def __init__(self):
        super().__init__('mission_manager')

        # ── Parameters (SIM placeholders — see module docstring) ───────────────
        self.declare_parameter('wp1_lat', 31.2001992)
        self.declare_parameter('wp1_lon', 29.9187839)
        self.declare_parameter('wp2_lat', 31.2001902)
        self.declare_parameter('wp2_lon', 29.9187000)
        self.declare_parameter('wp3_lat', 31.2002263)
        self.declare_parameter('wp3_lon', 29.9186266)
        self.declare_parameter('wp1_arrival_radius_m', 2.5)
        self.declare_parameter('wp1_rearm_radius_m', 8.0)
        self.declare_parameter('repeat_each_lap', False)
        self.declare_parameter('goal_settle_s', 0.5)

        self._wp1 = (self.get_parameter('wp1_lat').value, self.get_parameter('wp1_lon').value)
        self._wp2 = (self.get_parameter('wp2_lat').value, self.get_parameter('wp2_lon').value)
        self._wp3 = (self.get_parameter('wp3_lat').value, self.get_parameter('wp3_lon').value)
        self._arrival_r = self.get_parameter('wp1_arrival_radius_m').value
        self._rearm_r = self.get_parameter('wp1_rearm_radius_m').value
        self._repeat = self.get_parameter('repeat_each_lap').value
        self._settle_s = self.get_parameter('goal_settle_s').value

        # ── State ────────────────────────────────────────────────────────────────
        self._state = self.STATE_LANE_FOLLOW
        self._mission_done_once = False
        self._armed = True          # False while inside the wp1 radius, until rearm
        self._busy = False          # guards against re-entering the detour thread

        # ── Pub/Sub ──────────────────────────────────────────────────────────────
        self._lane_enable_pub = self.create_publisher(Bool, '/mission/lane_enable', 10)
        self._state_pub = self.create_publisher(String, '/mission/state', 10)
        self.create_subscription(NavSatFix, '/gps/fix', self._gps_cb, 10)

        # ── fromLL service client ──────────────────────────────────────────────
        self._from_ll_client = self.create_client(FromLL, '/fromLL')

        # ── Nav2 action client (own instance — same server as lane_goal_publisher,
        #     used only while lane-follow is paused) ─────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._publish_state()
        self.get_logger().info(
            'mission_manager started\n'
            f'  WP1 (handoff trigger) : {self._wp1}\n'
            f'  WP2                   : {self._wp2}\n'
            f'  WP3                   : {self._wp3}\n'
            f'  arrival radius        : {self._arrival_r} m\n'
            f'  repeat_each_lap        : {self._repeat}'
        )

    # =========================================================================
    #  GPS / TRIGGER LOGIC
    # =========================================================================

    def _gps_cb(self, msg: NavSatFix) -> None:
        if self._busy or self._mission_done_once and not self._repeat:
            return
        if self._state != self.STATE_LANE_FOLLOW:
            return

        dist = haversine_m(msg.latitude, msg.longitude, *self._wp1)

        if not self._armed:
            if dist >= self._rearm_r:
                self._armed = True
            return

        if dist <= self._arrival_r:
            self.get_logger().info(f'[mission] Within {dist:.2f} m of WAYPOINT_1 — handing off')
            self._armed = False
            self._busy = True
            threading.Thread(target=self._run_waypoint_detour, daemon=True).start()

    # =========================================================================
    #  WAYPOINT DETOUR  (runs in a background thread — blocking is fine here)
    # =========================================================================

    def _run_waypoint_detour(self) -> None:
        self._set_state(self.STATE_WAYPOINT_NAV)
        self._lane_enable_pub.publish(Bool(data=False))
        time.sleep(self._settle_s)

        ok = True
        for name, latlon in (('WAYPOINT_2', self._wp2), ('WAYPOINT_3', self._wp3)):
            pose = self._latlon_to_map_pose(*latlon)
            if pose is None:
                self.get_logger().error(f'[mission] /fromLL failed for {name} — aborting detour')
                ok = False
                break
            self.get_logger().info(f'[mission] Navigating to {name} ({latlon})')
            if not self._navigate_blocking(pose):
                self.get_logger().error(f'[mission] Navigation to {name} failed')
                ok = False
                break

        if ok:
            self.get_logger().info('[mission] Waypoint detour complete — resuming lane-follow')
        else:
            self.get_logger().warn('[mission] Waypoint detour aborted — resuming lane-follow anyway')

        self._mission_done_once = True
        self._lane_enable_pub.publish(Bool(data=True))
        self._set_state(self.STATE_LANE_FOLLOW)
        self._busy = False

    def _latlon_to_map_pose(self, lat: float, lon: float) -> PoseStamped | None:
        """Convert a GPS coordinate to a map-frame PoseStamped via /fromLL."""
        if not self._from_ll_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('[mission] /fromLL service not available')
            return None

        req = FromLL.Request()
        req.ll_point = GeoPoint(latitude=lat, longitude=lon, altitude=0.0)

        future = self._from_ll_client.call_async(req)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done() or future.result() is None:
            return None

        result = future.result()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = result.map_point.x
        pose.pose.position.y = result.map_point.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0   # heading doesn't matter for a point goal
        return pose

    def _navigate_blocking(self, pose: PoseStamped) -> bool:
        """Send a NavigateToPose goal and block this thread until it finishes."""
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('[mission] navigate_to_pose server not available')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        send_future = self._nav_client.send_goal_async(goal_msg)
        deadline = time.monotonic() + 10.0
        while not send_future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not send_future.done():
            return False

        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('[mission] Goal rejected by Nav2')
            return False

        result_future = handle.get_result_async()
        # No fixed deadline here — let Nav2's own recovery/timeout behaviors run.
        while not result_future.done():
            time.sleep(0.1)

        STATUS_SUCCEEDED = 4
        try:
            status = result_future.result().status
        except Exception as e:
            self.get_logger().error(f'[mission] Result error: {e}')
            return False

        return status == STATUS_SUCCEEDED

    # =========================================================================
    #  MISC
    # =========================================================================

    def _set_state(self, state: str) -> None:
        self._state = state
        self._publish_state()

    def _publish_state(self) -> None:
        self._state_pub.publish(String(data=self._state))


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('mission_manager shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()