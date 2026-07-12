#!/usr/bin/env python3
"""
lane_goal_publisher.py  —  rover26_autonomy

Converts lane polynomial coefficients from lane_detection into a stream of
Nav2 NavigateToPose goals so the rover follows the lane.

Goal computation is a single continuous formula: curvature (from the fused
centre-lane polynomial) blends the look-ahead distance, goal-update spacing,
and lateral smoothing between "straight" and "sharp" behaviour — there's no
separate discrete mode switch.

If both lane sides are lost, the rover cancels its current goal and spins in
place to search for the lane again.
"""

import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node   import Node
from rclpy.action import ActionClient
from geometry_msgs.msg      import Pose, PoseStamped, Twist
from nav_msgs.msg           import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg           import ColorRGBA
from interfaces.msg         import LaneDetectionResult
from nav2_msgs.action       import NavigateToPose

from rover26_autonomy.config_params import IMG_W, IMG_H, LaneGoalPublisher as Cfg, Physical, RosTopics


class LaneGoalPublisher(Node):
    """Converts lane polynomials to Nav2 NavigateToPose goals."""

    def __init__(self):
        super().__init__('lane_goal_publisher')

        # ── Node state ────────────────────────────────────────────────────────
        self.last_goal_xy:      tuple | None = None
        self.current_odom                    = None
        self._nav_server_ready: bool         = False
        self._y_lat_smooth:     float | None = None

        self._nav_active:             bool = False
        self._goal_generation:        int  = 0
        self._intentional_cancel_gen: int  = -1
        self._current_goal_handle          = None
        self._current_goal_x:   float | None = None
        self._current_goal_y:   float | None = None
        self._current_goal_yaw: float | None = None

        # ── Lane-loss / rotation recovery ────────────────────────────────────
        self._last_valid_time_s:       float | None = None
        self._last_lane_status_time_s: float | None = None
        self._last_valid_curve_sign:   float        = 1.0
        self._is_rotating:             bool         = False
        self._lane_loss_start_s:       float | None = None

        self._marker_id: int = 0

        # ── Nav2 action client ───────────────────────────────────────────────
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── Publishers ────────────────────────────────────────────────────────
        self.viz_pub     = self.create_publisher(MarkerArray, RosTopics.LANE_GOALS_VIZ, 10)
        self.cmd_vel_pub = self.create_publisher(Twist,       RosTopics.CMD_VEL,        10)

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            LaneDetectionResult, RosTopics.LANE_DETECTION, self._lane_cb, 10
        )
        self.create_subscription(Odometry, RosTopics.ODOM, self._odom_cb, 10)

        # ── Watchdog timer ────────────────────────────────────────────────────
        self.create_timer(1.0 / Cfg.WATCHDOG_HZ, self._watchdog_cb)

        # ── Nav server ready-check (background thread) ───────────────────────
        if Cfg.DEBUG_STANDALONE:
            self._nav_server_ready = True
            self.get_logger().warn(
                '⚠ DEBUG_STANDALONE=True — goals are computed and published to '
                'RViz only, NOT dispatched to Nav2. Set False for real driving.'
            )
        else:
            threading.Thread(target=self._wait_for_nav_server, daemon=True).start()

        self.get_logger().info(
            'lane_goal_publisher started  '
            f'(curve_thr={Cfg.SHARP_THRESHOLD:.4f}  '
            f'look-ahead {Cfg.PY_FAR}..{Cfg.PY_SHARP}  '
            f'update {Cfg.UPDATE_M_STRAIGHT}..{Cfg.UPDATE_M_SHARP} m)'
        )

    # =========================================================================
    #  STARTUP / ODOMETRY
    # =========================================================================

    def _wait_for_nav_server(self) -> None:
        """Background thread: block until bt_navigator is ACTIVE by probing with a trivial goal."""
        self.get_logger().info('[startup] Waiting for action server...')
        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info('[startup] Probing for ACTIVE state...')

        while rclpy.ok():
            probe                    = PoseStamped()
            probe.header.frame_id    = RosTopics.ODOM_FRAME
            probe.header.stamp       = self.get_clock().now().to_msg()
            probe.pose.position.x    = 0.01
            probe.pose.orientation.w = 1.0

            goal_msg      = NavigateToPose.Goal()
            goal_msg.pose = probe
            future        = self.nav_to_pose_client.send_goal_async(goal_msg)

            deadline = time.monotonic() + 3.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)

            if not future.done():
                time.sleep(2.0)
                continue

            handle = future.result()
            if handle is None or not handle.accepted:
                time.sleep(2.0)
                continue

            self.get_logger().info('[startup] bt_navigator ACTIVE')
            handle.cancel_goal_async()
            self._nav_server_ready = True
            return

    def _odom_cb(self, msg: Odometry) -> None:
        """Cache the latest odometry pose (position + orientation quaternion)."""
        self.current_odom = msg.pose.pose

    # =========================================================================
    #  RVIZ MARKERS
    # =========================================================================

    def _publish_goal_marker(self, goal_x: float, goal_y: float, goal_yaw: float) -> None:
        """Publish an arrow + sphere marker pair at the computed goal position."""
        now     = self.get_clock().now().to_msg()
        markers = MarkerArray()

        qz = math.sin(goal_yaw / 2.0)
        qw = math.cos(goal_yaw / 2.0)

        arrow = Marker()
        arrow.header.frame_id    = RosTopics.ODOM_FRAME
        arrow.header.stamp       = now
        arrow.ns                 = 'lane_goal_arrows'
        arrow.id                 = self._marker_id
        arrow.type               = Marker.ARROW
        arrow.action             = Marker.ADD
        arrow.pose.position.x    = goal_x
        arrow.pose.position.y    = goal_y
        arrow.pose.position.z    = 0.05
        arrow.pose.orientation.z = qz
        arrow.pose.orientation.w = qw
        arrow.scale.x            = 0.55
        arrow.scale.y            = 0.07
        arrow.scale.z            = 0.13
        arrow.color              = ColorRGBA(r=0.0, g=0.95, b=0.1, a=0.95)
        arrow.lifetime.sec       = 8

        sphere = Marker()
        sphere.header          = arrow.header
        sphere.ns              = 'lane_goal_spheres'
        sphere.id              = self._marker_id
        sphere.type            = Marker.SPHERE
        sphere.action          = Marker.ADD
        sphere.pose.position.x = goal_x
        sphere.pose.position.y = goal_y
        sphere.pose.position.z = 0.05
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.11
        sphere.color           = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.9)
        sphere.lifetime.sec    = 8

        markers.markers = [arrow, sphere]
        self.viz_pub.publish(markers)

        self._marker_id = (self._marker_id + 1) % Cfg.MAX_MARKERS

    # =========================================================================
    #  MAIN CALLBACK — runs on every /lane_detection message
    # =========================================================================

    def _lane_cb(self, msg: LaneDetectionResult) -> None:
        """Receive a LaneDetectionResult message and compute + dispatch the next Nav2 goal."""
        now_s = self.get_clock().now().nanoseconds * 1e-9
        self._last_lane_status_time_s = now_s

        if Cfg.DEBUG_STANDALONE and self.current_odom is None:
            # No odometry source running — pretend the rover is at the origin.
            fake = Pose()
            fake.orientation.w = 1.0
            self.current_odom = fake

        if not self._nav_server_ready or self.current_odom is None:
            return

        left_ok  = len(msg.left_lane_coeffs)  == 3
        right_ok = len(msg.right_lane_coeffs) == 3

        if self._nav_active and self._is_rotating and left_ok and right_ok:
            self.get_logger().info('[recovery] lane recovered — interrupting rotation')
            self._goal_generation  += 1
            self._nav_active        = False
            self.last_goal_xy       = None
            self._is_rotating       = False
            self._lane_loss_start_s = None
            # Fall through to normal goal computation below.

        was_nav_active = self._nav_active

        if not (left_ok and right_ok):
            self._try_rotation_recovery()
            return

        # ── Lane geometry ────────────────────────────────────────────────────
        left_c   = np.array(msg.left_lane_coeffs,  dtype=float)
        right_c  = np.array(msg.right_lane_coeffs, dtype=float)
        centre_c = (left_c + right_c) / 2.0

        curvature = abs(centre_c[0])
        # 0 = straight, 1 = at/above the sharp-turn threshold. One continuous
        # blend drives look-ahead distance, update spacing, and smoothing —
        # no separate discrete mode.
        blend = min(1.0, curvature / Cfg.SHARP_THRESHOLD) if Cfg.SHARP_THRESHOLD > 0 else 0.0

        # ── Rover pose ───────────────────────────────────────────────────────
        q         = self.current_odom.orientation
        rover_yaw = 2.0 * math.atan2(q.z, q.w)
        rover_x   = self.current_odom.position.x
        rover_y   = self.current_odom.position.y

        # ── Adaptive look-ahead row ──────────────────────────────────────────
        py_frac = Cfg.PY_FAR + blend * (Cfg.PY_SHARP - Cfg.PY_FAR)
        py      = int(np.clip(IMG_H * py_frac, 0, IMG_H - 1))

        centre_px = np.polyval(centre_c, py)
        if not (0 <= centre_px < IMG_W):
            self.get_logger().warn(
                f'[lane] REJECT — centre_px={centre_px:.1f} off-image at py={py}',
                throttle_duration_sec=1.0,
            )
            return

        left_px_look  = np.polyval(left_c,  py)
        right_px_look = np.polyval(right_c, py)
        lane_width_px = right_px_look - left_px_look
        if lane_width_px < 10:
            self.get_logger().warn(
                f'[lane] REJECT — lane_width_px={lane_width_px:.1f} < 10 at py={py}',
                throttle_duration_sec=1.0,
            )
            return

        x_fwd         = (1.0 - py / IMG_H) * Physical.GROUND_HEIGHT_M * 0.92
        offset_px     = IMG_W / 2.0 - centre_px
        metres_per_px = Physical.LANE_WIDTH_M / lane_width_px
        y_lat         = offset_px * metres_per_px + Physical.LATERAL_BIAS_M

        # Lateral IIR smoother — heavier on straights, quicker in curves.
        alpha = Cfg.ALPHA_STRAIGHT + blend * (Cfg.ALPHA_CURVE - Cfg.ALPHA_STRAIGHT)
        if self._y_lat_smooth is None:
            self._y_lat_smooth = y_lat
        else:
            self._y_lat_smooth = alpha * self._y_lat_smooth + (1.0 - alpha) * y_lat
        y_lat_used = self._y_lat_smooth

        cos_yaw = math.cos(rover_yaw)
        sin_yaw = math.sin(rover_yaw)
        goal_x  = rover_x + x_fwd * cos_yaw - y_lat_used * sin_yaw
        goal_y  = rover_y + x_fwd * sin_yaw + y_lat_used * cos_yaw
        goal_yaw = rover_yaw + math.atan2(y_lat_used, x_fwd)

        min_update = Cfg.UPDATE_M_STRAIGHT + blend * (Cfg.UPDATE_M_SHARP - Cfg.UPDATE_M_STRAIGHT)

        # ── Goal throttle: only send if moved far enough and nothing active ──
        if self.last_goal_xy is not None:
            dist = math.hypot(goal_x - self.last_goal_xy[0], goal_y - self.last_goal_xy[1])
            if dist < min_update:
                return
        if was_nav_active:
            return

        self.last_goal_xy      = (goal_x, goal_y)
        self._last_valid_time_s = now_s
        if abs(y_lat_used) > 1e-3:
            self._last_valid_curve_sign = math.copysign(1.0, y_lat_used)

        self._is_rotating       = False
        self._lane_loss_start_s = None

        self._publish_goal_marker(goal_x, goal_y, goal_yaw)
        if Cfg.DEBUG_STANDALONE:
            # No Nav2: clear the anchor so every frame recomputes (the rover
            # never moves, so the throttle would otherwise HOLD forever).
            self.last_goal_xy = None
            self.get_logger().info(
                f'[STANDALONE] goal=({goal_x:.2f},{goal_y:.2f},'
                f'{math.degrees(goal_yaw):+.1f}°) — NOT dispatched',
                throttle_duration_sec=1.0,
            )
        else:
            self._send_action_goal(goal_x, goal_y, goal_yaw)

        self.get_logger().info(
            f'[goal] curv={curvature:.2e} blend={blend:.2f} py={py} '
            f'goal=({goal_x:.2f},{goal_y:.2f}) hdg={math.degrees(goal_yaw - rover_yaw):+.1f}°'
        )

    # =========================================================================
    #  LANE-LOSS RECOVERY
    # =========================================================================

    def _watchdog_cb(self) -> None:
        """Fires at WATCHDOG_HZ, independently of /lane_detection."""
        if not self._nav_server_ready or self.current_odom is None:
            return
        if self._last_lane_status_time_s is None:
            return

        now_s   = self.get_clock().now().nanoseconds * 1e-9
        silence = now_s - self._last_lane_status_time_s

        self._check_goal_proximity()

        # ── Trigger A: topic silence ─────────────────────────────────────────
        if silence >= Cfg.LANE_SILENCE_THRESH_S:
            if not self._nav_active:
                self.get_logger().warn(f'[watchdog] topic silent {silence:.1f}s', throttle_duration_sec=2.0)
                self._try_rotation_recovery()
            return

        if self._nav_active:
            return

        # ── Trigger B: detection loss (topic alive, no good lanes) ──────────
        detection_gap = (now_s - self._last_valid_time_s) if self._last_valid_time_s else silence
        if detection_gap >= Cfg.LANE_LOSS_DETECT_S:
            self.get_logger().warn(
                f'[watchdog] no good detection for {detection_gap:.1f}s — recovery',
                throttle_duration_sec=2.0,
            )
            self._try_rotation_recovery()

    def _check_goal_proximity(self) -> None:
        """Declare the current goal REACHED when xy_err ≤ XY_GOAL_TOL."""
        if not self._nav_active or self._current_goal_x is None or self.current_odom is None:
            return

        xy_err = math.hypot(self.current_odom.position.x - self._current_goal_x,
                             self.current_odom.position.y - self._current_goal_y)
        if xy_err > Cfg.XY_GOAL_TOL:
            return

        self._cancel_current_goal()
        self._goal_generation  += 1
        self._nav_active        = False
        self.last_goal_xy       = None
        self._current_goal_x    = None
        self._current_goal_y    = None
        self._current_goal_yaw  = None
        self.get_logger().info(f'[nav] goal reached (xy_err={xy_err:.2f}m) — next goal')

    def _cancel_current_goal(self) -> None:
        """Cancel the active Nav2 goal (marks the cancel as intentional)."""
        if self._current_goal_handle is not None:
            self._intentional_cancel_gen = self._goal_generation
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None

    def _try_rotation_recovery(self) -> None:
        """
        Lane-loss recovery: cancel any active goal and spin in place (toward
        the last known curve direction) until the lane is found again or
        LANE_LOSS_TIMEOUT_S elapses. Re-triggered by the watchdog every tick
        the lane stays lost, so it naturally keeps spinning until it works.
        """
        if Cfg.DEBUG_STANDALONE or self._is_rotating or self._nav_active or self.current_odom is None:
            return

        self._cancel_current_goal()
        now_s = self.get_clock().now().nanoseconds * 1e-9

        if self._lane_loss_start_s is None:
            self._lane_loss_start_s = now_s
            self.get_logger().warn('[recovery] lane lost — rotating to search')

        age = now_s - self._lane_loss_start_s
        if age > Cfg.LANE_LOSS_TIMEOUT_S:
            if age < Cfg.LANE_LOSS_TIMEOUT_S + 2.0:
                self.get_logger().error('[recovery] timeout — giving up on rotation search')
            return

        q         = self.current_odom.orientation
        start_yaw = 2.0 * math.atan2(q.z, q.w)

        self._is_rotating = True
        threading.Thread(
            target=self._spin_to_heading,
            args=(start_yaw, self._last_valid_curve_sign),
            daemon=True,
        ).start()

    def _spin_to_heading(self, start_yaw: float, rot_dir: float) -> None:
        """Background thread: spin until rotated RECOVERY_YAW_TARGET_RAD, then stop."""
        twist           = Twist()
        twist.angular.z = rot_dir * Cfg.RECOVERY_ROT_VEL
        stop            = Twist()

        try:
            while rclpy.ok():
                if not self._is_rotating:
                    self.cmd_vel_pub.publish(stop)
                    return

                if self.current_odom is None:
                    time.sleep(Cfg.RECOVERY_POLL_S)
                    continue

                q           = self.current_odom.orientation
                current_yaw = 2.0 * math.atan2(q.z, q.w)
                yaw_delta   = abs(math.atan2(math.sin(current_yaw - start_yaw), math.cos(current_yaw - start_yaw)))

                if yaw_delta >= Cfg.RECOVERY_YAW_TARGET_RAD:
                    self.cmd_vel_pub.publish(stop)
                    self._is_rotating = False
                    return

                self.cmd_vel_pub.publish(twist)
                time.sleep(Cfg.RECOVERY_POLL_S)

        except Exception as e:
            self.get_logger().error(f'[recovery] spin thread error: {e}')
            self._is_rotating = False
            self.cmd_vel_pub.publish(stop)

    # =========================================================================
    #  NAV2 ACTION DISPATCH
    # =========================================================================

    def _send_action_goal(self, x: float, y: float, yaw: float) -> None:
        """Send a NavigateToPose goal and wire up generation-tagged lifecycle callbacks."""
        self._current_goal_x   = x
        self._current_goal_y   = y
        self._current_goal_yaw = yaw

        self._goal_generation += 1
        gen = self._goal_generation

        goal_pose                    = PoseStamped()
        goal_pose.header.stamp       = self.get_clock().now().to_msg()
        goal_pose.header.frame_id    = RosTopics.ODOM_FRAME
        goal_pose.pose.position.x    = x
        goal_pose.pose.position.y    = y
        goal_pose.pose.position.z    = 0.0
        goal_pose.pose.orientation.z = math.sin(yaw / 2)
        goal_pose.pose.orientation.w = math.cos(yaw / 2)

        goal_msg      = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        send_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_future.add_done_callback(lambda f, g=gen: self._on_goal_response(f, g))

    def _on_goal_response(self, future, gen: int) -> None:
        """Called when Nav2 responds (accepted or rejected)."""
        if gen != self._goal_generation:
            return

        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('[nav] goal rejected by Nav2')
            self._nav_active = False
            return

        self._nav_active = True
        self._current_goal_handle = handle

        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda f, g=gen: self._on_nav_result(f, g))

    def _on_nav_result(self, future, gen: int) -> None:
        """Called when Nav2 reports SUCCEEDED / FAILED / CANCELLED."""
        if gen != self._goal_generation:
            return

        STATUS_SUCCEEDED = 4
        STATUS_CANCELED  = 5
        status = 0
        try:
            status = future.result().status
        except Exception as e:
            self.get_logger().warn(f'[nav] result callback error: {e}')

        self._current_goal_handle = None
        self._nav_active = False
        intentional = (gen == self._intentional_cancel_gen)

        if status not in (STATUS_SUCCEEDED, STATUS_CANCELED) and not intentional:
            self.get_logger().warn(f'[nav] goal FAILED (status={status}) — retrying fresh')

        # Whatever the outcome, clear the anchor so the next lane frame
        # recomputes a fresh goal from the rover's current position.
        self.last_goal_xy = None


# ═════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = LaneGoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('lane_goal_publisher shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
