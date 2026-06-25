#!/usr/bin/env python3
"""
═════════════════════════════════════════════════════════════════════════════════
lane_goal_publisher.py  —  rover26_autonomy
═════════════════════════════════════════════════════════════════════════════════

FUNCTIONALITY
─────────────
Converts lane polynomial coefficients from lane_detection_node into a stream of
Nav2 NavigateToPose goals, enabling the rover to follow the lane autonomously.

Implements three-tier goal computation based on road curvature:

  Tier 1 — STRAIGHT   |a| < CURVE_THRESHOLD
            Long look-ahead, sparse goal updates, heavy lateral smoothing.

  Tier 2 — CURVE      CURVE_THRESHOLD ≤ |a| < SHARP_THRESHOLD
            Shorter look-ahead, outer-wall bias to widen the arc, quicker
            lateral IIR response.

  Tier 3 — SHARP      |a| ≥ SHARP_THRESHOLD  (90° corners)
            Abandons lateral position entirely.  Projects a short fixed-distance
            waypoint in the lane-tangent heading direction, generating a rapid
            sequence of "stepping stones" through the turn.  This avoids the
            geometric unreliability of lane polynomial extrapolation at tight
            corners.

RECOVERY (lane-loss)
────────────────────
A 10 Hz watchdog detects when /lane_status goes completely silent (the lane
detector stops publishing entirely when the lane exits the BEV frame).
Two-phase direct /cmd_vel rotation recovery is then triggered:
  Phase 0 — rotate ~95° toward outside of the last known curve.
  Phase 1 — rotate ~185° in the opposite direction (full turnaround search).

TOPICS & SUBSCRIPTIONS
──────────────────────
Subscribed:
  • /lane_status  [rover26/LaneStatus]   ← lane_detection_node
  • /odom         [nav_msgs/Odometry]    ← odom_tf_broadcaster

Published:
  • /cmd_vel         [geometry_msgs/Twist]          → motor driver (recovery rotation only)
  • /lane_goals_viz  [visualization_msgs/MarkerArray] → RViz (goal arrows & spheres)

Nav2 Action Client:
  • navigate_to_pose  [nav2_msgs/action/NavigateToPose]  → bt_navigator

TF FRAMES
─────────
All goals published in the 'odom' frame.

PARAMETERS (from config_params.py)
──────────────────────────────────
LaneGoalPublisher.*  Curvature thresholds, look-ahead fractions, bias, smoothers,
                     update throttle, proximity tolerances, recovery parameters
Physical.*           Ground plane dimensions (GROUND_HEIGHT_M, GROUND_WIDTH_M,
                     LATERAL_BIAS_M)
RosTopics.*          Topic names

═════════════════════════════════════════════════════════════════════════════════
"""

import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node   import Node
from rclpy.action import ActionClient
from geometry_msgs.msg      import PoseStamped, Twist
from nav_msgs.msg           import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg           import ColorRGBA, Bool
from rover26.msg            import LaneStatus
from nav2_msgs.action       import NavigateToPose

from rover26_autonomy.config_params import IMG_W, IMG_H, LaneGoalPublisher as Cfg, Physical, RosTopics


# ═════════════════════════════════════════════════════════════════════════════
#  LANE GOAL PUBLISHER NODE
# ═════════════════════════════════════════════════════════════════════════════

class LaneGoalPublisher(Node):
    """
    Converts lane polynomials to Nav2 NavigateToPose goals in three tiers.

    All tunable parameters are in config_params.LaneGoalPublisher.
    All topic names are in config_params.RosTopics.

    Attributes:
        last_goal_xy (tuple | None):     (x, y) of last accepted goal (goal throttle anchor).
        current_odom (Pose | None):      Latest odometry pose (position + orientation).
        _nav_server_ready (bool):        True once bt_navigator has accepted its probe goal.
        _y_lat_smooth (float | None):    Lateral IIR state for STRAIGHT/CURVE mode.
        _goal_yaw_smooth (float | None): Heading IIR state for SHARP mode.
        _prev_sharp (bool):              Whether the previous frame was in SHARP mode.
        _sharp_exit_goals (int):         Dense-goal momentum counter after exiting SHARP mode.
        _nav_active (bool):              True from goal ACCEPTED until goal REACHED/FAILED.
        _goal_generation (int):          Counter to invalidate stale Nav2 result callbacks.
    """

    def __init__(self):
        super().__init__('lane_goal_publisher')

        # ── Node state ──────────────────────────────────────────────────────────
        self.last_goal_xy:          tuple | None  = None
        self.current_odom                         = None
        self._nav_server_ready: bool              = False
        self._y_lat_smooth:     float | None      = None
        self._goal_yaw_smooth:  float | None      = None
        self._prev_sharp:       bool              = False
        self._sharp_exit_goals: int               = 0

        self._nav_active:       bool              = False
        self._goal_generation:  int               = 0
        self._intentional_cancel_gen: int         = -1
        self._current_goal_handle                 = None

        self._current_goal_x:   float | None      = None
        self._current_goal_y:   float | None      = None
        self._current_goal_yaw: float | None      = None
        self._consecutive_failures: int           = 0

        # ── Mission-manager pause/resume ────────────────────────────────────────
        # When False, this node stops sending NEW NavigateToPose goals and
        # cancels whatever goal is currently in flight. Driven by
        # mission_manager so it can hand the navigate_to_pose action server
        # over to GPS waypoint navigation without the two nodes fighting
        # over the same goal slot.
        self._mission_enabled:  bool              = True

        # ── Rotation recovery state ─────────────────────────────────────────────
        self._last_valid_goal_yaw:     float | None  = None
        self._last_valid_time_s:       float | None  = None
        self._last_lane_status_time_s: float | None  = None
        self._last_valid_curve_sign:   float          = 0.0
        self._is_rotating:             bool           = False
        self._rotation_target_yaw:    float | None  = None
        self._rotation_accumulated:   float          = 0.0
        self._lane_loss_start_s:      float | None  = None
        self._recovery_phase:         int            = 0
        self._recovery_start_yaw:     float | None  = None

        # ── RViz marker cycling ─────────────────────────────────────────────────
        self._marker_id: int = 0

        # ── Nav2 action client ──────────────────────────────────────────────────
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── Publishers ──────────────────────────────────────────────────────────
        self.viz_pub     = self.create_publisher(MarkerArray, RosTopics.LANE_GOALS_VIZ, 10)
        self.cmd_vel_pub = self.create_publisher(Twist,       RosTopics.CMD_VEL,        10)

        # ── Subscriptions ────────────────────────────────────────────────────────
        self.create_subscription(LaneStatus, RosTopics.LANE_STATUS, self._lane_cb, 10)
        self.create_subscription(Odometry,   RosTopics.ODOM,        self._odom_cb, 10)
        self.create_subscription(Bool, '/mission/lane_enable', self._mission_enable_cb, 10)

        # ── Watchdog timer ───────────────────────────────────────────────────────
        self.create_timer(1.0 / Cfg.WATCHDOG_HZ, self._watchdog_cb)

        # ── Nav server ready-check (background thread) ───────────────────────────
        threading.Thread(target=self._wait_for_nav_server, daemon=True).start()

        self.get_logger().info(
            'lane_goal_publisher started — THREE-TIER MODE\n'
            f'  curve threshold : |a| > {Cfg.CURVE_THRESHOLD:.4f}\n'
            f'  sharp threshold : |a| > {Cfg.SHARP_THRESHOLD:.4f}\n'
            f'  look-ahead      : far={Cfg.PY_FAR}  near={Cfg.PY_NEAR}  sharp={Cfg.PY_SHARP}\n'
            f'  outer bias max  : {Cfg.OUTER_BIAS_MAX_M} m\n'
            f'  recovery vel    : {Cfg.RECOVERY_ROT_VEL} rad/s'
        )

    # =========================================================================
    #  STARTUP / ODOMETRY
    # =========================================================================

    def _wait_for_nav_server(self) -> None:
        """
        Background thread: block until bt_navigator is ACTIVE by probing with
        a trivial goal. A mere wait_for_server() call is insufficient — the
        action server may be up but the lifecycle node not yet ACTIVE.
        """
        self.get_logger().info('[startup] Waiting for action server...')
        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info('[startup] Probing for ACTIVE state...')

        while rclpy.ok():
            probe                    = PoseStamped()
            probe.header.frame_id    = RosTopics.ODOM_FRAME
            probe.header.stamp       = self.get_clock().now().to_msg()
            probe.pose.position.x    = 0.01
            probe.pose.orientation.w = 1.0

            goal_msg       = NavigateToPose.Goal()
            goal_msg.pose  = probe
            future         = self.nav_to_pose_client.send_goal_async(goal_msg)

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

    def _mission_enable_cb(self, msg: Bool) -> None:
        """
        React to mission_manager pausing/resuming lane-following.

        On disable: cancel any in-flight NavigateToPose goal, stop rotation
        recovery, and clear the goal-throttle anchor so the very first goal
        after re-enabling is computed fresh (not skipped as "too close").
        On enable: nothing extra needed — _lane_cb resumes on the next
        /lane_status message.
        """
        was_enabled = self._mission_enabled
        self._mission_enabled = msg.data

        if was_enabled and not msg.data:
            self.get_logger().info('[mission] Lane-following PAUSED — handing off to waypoint nav')
            self._goal_generation += 1  # invalidate any pending result callbacks
            if self._current_goal_handle is not None:
                self._current_goal_handle.cancel_goal_async()
                self._current_goal_handle = None
            self._nav_active   = False
            self._is_rotating  = False
            self.last_goal_xy  = None
        elif (not was_enabled) and msg.data:
            self.get_logger().info('[mission] Lane-following RESUMED')

    # =========================================================================
    #  CURVATURE BLEND HELPERS  (each returns 0 → 1)
    # =========================================================================

    def _curve_blend(self, curvature: float) -> float:
        """0 = dead straight, 1 = curvature at or above CURVE_THRESHOLD."""
        if Cfg.CURVE_THRESHOLD <= 0:
            return 0.0
        return min(1.0, curvature / Cfg.CURVE_THRESHOLD)

    def _sharp_blend(self, curvature: float) -> float:
        """0 = at CURVE_THRESHOLD (gentle curve), 1 = at/above SHARP_THRESHOLD."""
        if curvature <= Cfg.CURVE_THRESHOLD:
            return 0.0
        span = Cfg.SHARP_THRESHOLD - Cfg.CURVE_THRESHOLD
        if span <= 0:
            return 1.0
        return min(1.0, (curvature - Cfg.CURVE_THRESHOLD) / span)

    # =========================================================================
    #  GEOMETRY HELPERS
    # =========================================================================

    def _lane_tangent_yaw(self, centre_c: np.ndarray, py: int, rover_yaw: float) -> float:
        """
        Convert the polynomial slope at image row py into a world-frame yaw.

        Polynomial: x_pixel = a·py² + b·py + c
        Slope at py: dx/dpy = 2a·py + b

        The pixel slope is converted to a metric angle using the ground plane
        dimensions, then the rover's current yaw is added to get world-frame heading.

        Args:
            centre_c:  Centre-lane polynomial coefficients [a, b, c].
            py:        Image row (pixels) to evaluate the slope at.
            rover_yaw: Current rover heading in the odom frame (radians).

        Returns:
            goal_yaw: World-frame heading the lane is pointing at row py (radians).
        """
        a, b, _ = centre_c
        slope = 2.0 * a * py + b

        delta_x_fwd = Physical.GROUND_HEIGHT_M * 0.92 / IMG_H
        delta_y_lat = slope * Physical.GROUND_WIDTH_M / IMG_W

        theta_local = math.atan2(delta_y_lat, delta_x_fwd)
        return rover_yaw + theta_local

    @staticmethod
    def _wrap_angle(a: float) -> float:
        """Wrap an angle to [−π, π]."""
        return math.atan2(math.sin(a), math.cos(a))

    # =========================================================================
    #  RVIZ MARKERS
    # =========================================================================

    def _publish_goal_marker(self, goal_x: float, goal_y: float,
                              goal_yaw: float, sharp: bool = False) -> None:
        """
        Publish an arrow + sphere marker pair at the computed goal position.
        Arrow is RED in sharp-turn mode, GREEN in normal mode.
        """
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
        arrow.color              = (ColorRGBA(r=1.0, g=0.15, b=0.0, a=0.95) if sharp
                                    else ColorRGBA(r=0.0, g=0.95, b=0.1,  a=0.95))
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
    #  MAIN CALLBACK — runs on every /lane_status message
    # =========================================================================

    def _lane_cb(self, msg: LaneStatus) -> None:
        """
        Receive a LaneStatus message and compute + dispatch the next Nav2 goal.

        The method silently returns if:
          - Nav2 is not yet ready or odometry hasn't arrived
          - A goal is currently active (and the rover isn't recovering lane)
          - Either lane polynomial is missing (triggers rotation recovery instead)
          - The new goal is too close to the previous one (goal throttle)
        """
        self._last_lane_status_time_s = self.get_clock().now().nanoseconds * 1e-9

        if not self._mission_enabled:
            return

        if not self._nav_server_ready or self.current_odom is None:
            return

        left_ok  = msg.left_detected  and len(msg.left_coeffs)  == 3
        right_ok = msg.right_detected and len(msg.right_coeffs) == 3

        if self._nav_active:
            if self._is_rotating and left_ok and right_ok:
                self.get_logger().info('[ROTATE] Lane recovered — interrupting rotation')
                self._goal_generation    += 1
                self._nav_active          = False
                self.last_goal_xy         = None
                self._is_rotating         = False
                self._rotation_accumulated = 0.0
                self._rotation_target_yaw  = None
                self._lane_loss_start_s    = None
                self._recovery_phase       = 0
                self._recovery_start_yaw   = None
                # Fall through to normal goal computation below
            # else: fall through — heading-drift check in throttle block decides

        _was_nav_active = self._nav_active  # capture before geometry runs

        if not (left_ok and right_ok):
            self._try_rotation_recovery()
            return

        # ── 1. Lane geometry ─────────────────────────────────────────────────────
        left_c   = np.array(msg.left_coeffs,  dtype=float)
        right_c  = np.array(msg.right_coeffs, dtype=float)
        centre_c = (left_c + right_c) / 2.0

        curvature   = abs(centre_c[0])
        blend_curve = self._curve_blend(curvature)
        blend_sharp = self._sharp_blend(curvature)

        # ── 2. Rover pose ────────────────────────────────────────────────────────
        q         = self.current_odom.orientation
        rover_yaw = 2.0 * math.atan2(q.z, q.w)
        rover_x   = self.current_odom.position.x
        rover_y   = self.current_odom.position.y

        # ── 3. Adaptive look-ahead row ───────────────────────────────────────────
        py_frac = (Cfg.PY_FAR
                   + blend_curve * (Cfg.PY_NEAR  - Cfg.PY_FAR)
                   + blend_sharp * (Cfg.PY_SHARP - Cfg.PY_NEAR))
        py = int(np.clip(IMG_H * py_frac, 0, IMG_H - 1))

        # ── 4. Lane tangent heading ──────────────────────────────────────────────
        goal_yaw = self._lane_tangent_yaw(centre_c, py, rover_yaw)

        # ── 5. Mode decision ─────────────────────────────────────────────────────
        in_sharp_turn = blend_sharp > 0.10
        outer_bias    = 0.0

        if self._prev_sharp and not in_sharp_turn:
            self._sharp_exit_goals = 2
            self.last_goal_xy = None
        _sharp_exit_active = (not in_sharp_turn and self._sharp_exit_goals > 0)

        # ╔══════════════════════════════════════════════════════════════════════╗
        # ║  SHARP TURN MODE  (90° corners)                                    ║
        # ╚══════════════════════════════════════════════════════════════════════╝
        if in_sharp_turn:
            if self._goal_yaw_smooth is None or not self._prev_sharp:
                self._goal_yaw_smooth = goal_yaw
            else:
                delta = self._wrap_angle(goal_yaw - self._goal_yaw_smooth)
                self._goal_yaw_smooth += (1.0 - Cfg.ALPHA_SHARP_YAW) * delta

            heading = self._goal_yaw_smooth

            base_x, base_y = rover_x, rover_y

            goal_x = base_x + Cfg.GOAL_DIST_SHARP_M * math.cos(heading)
            goal_y = base_y + Cfg.GOAL_DIST_SHARP_M * math.sin(heading)

            self._y_lat_smooth = None
            min_update = Cfg.UPDATE_M_SHARP

        # ╔══════════════════════════════════════════════════════════════════════╗
        # ║  NORMAL MODE  (straight + gentle curve)                            ║
        # ╚══════════════════════════════════════════════════════════════════════╝
        else:
            self._goal_yaw_smooth = None

            centre_px = np.polyval(centre_c, py)
            if not (0 <= centre_px < IMG_W):
                return

            x_fwd = (1.0 - py / IMG_H) * Physical.GROUND_HEIGHT_M * 0.92
            if _sharp_exit_active:
                x_fwd = min(x_fwd, Cfg.GOAL_DIST_SHARP_M)

            # Lateral correction: rover is always at IMG_W/2 in BEV.
            # offset_px > 0 means lane centre is to the right → steer right.
            left_px_look  = np.polyval(left_c,  py)
            right_px_look = np.polyval(right_c, py)
            lane_width_px = right_px_look - left_px_look
            if lane_width_px < 10:
                return

            offset_px     =  IMG_W / 2.0 - centre_px
            metres_per_px = Physical.LANE_WIDTH_M / lane_width_px
            y_lat         = offset_px * metres_per_px
            y_lat        += Physical.LATERAL_BIAS_M

            # Outer-wall bias — widens the arc on curves.
            # Curve direction derived from heading offset (stable, no polynomial sign flip-flop).
            theta_local = self._wrap_angle(goal_yaw - rover_yaw)
            curve_sign  = -np.sign(theta_local) if abs(theta_local) > 1e-3 else 0.0
            bias_factor = blend_curve ** Cfg.BIAS_BLEND_POWER
            fwd_factor  = min(1.0, x_fwd / Cfg.FWD_FACTOR_DIV)
            outer_bias  = (Cfg.OUTER_BIAS_MAX_M
                           * bias_factor
                           * fwd_factor
                           * (1.0 - blend_sharp))
            y_lat_bias  = y_lat + curve_sign * outer_bias

            # Lateral IIR smoother — heavier on straight, lighter on curves
            alpha = (Cfg.ALPHA_STRAIGHT
                     + blend_curve * (Cfg.ALPHA_CURVE - Cfg.ALPHA_STRAIGHT))

            if self._y_lat_smooth is None:
                self._y_lat_smooth = y_lat_bias
            else:
                self._y_lat_smooth = (alpha * self._y_lat_smooth
                                      + (1.0 - alpha) * y_lat_bias)
            y_lat_used = self._y_lat_smooth

            cos_yaw = math.cos(rover_yaw)
            sin_yaw = math.sin(rover_yaw)
            goal_x = rover_x + x_fwd * cos_yaw - y_lat_used * sin_yaw
            goal_y = rover_y + x_fwd * sin_yaw + y_lat_used * cos_yaw

            min_update = (Cfg.UPDATE_M_STRAIGHT
                          + blend_curve * (Cfg.UPDATE_M_CURVE - Cfg.UPDATE_M_STRAIGHT))

            if _sharp_exit_active:
                min_update = Cfg.UPDATE_M_SHARP
                self._sharp_exit_goals -= 1
                self.get_logger().info(
                    f'[NORM] sharp-exit momentum — {self._sharp_exit_goals} goals remaining'
                )

    # ── 7. Goal throttle ─────────────────────────────────────────────────────
        on_straight   = not in_sharp_turn and blend_curve < 0.3
        heading_drift = False

        if self.last_goal_xy is not None:
            dist = math.hypot(goal_x - self.last_goal_xy[0],
                              goal_y - self.last_goal_xy[1])

            # On straights with <7 deg heading change AND the rover has travelled
            # at least min_update metres — send a fresh goal so it keeps tracking.
            # The dist >= min_update guard prevents the cancel-resend storm that
            # happens when the rover hasn't moved yet (dist=0.00m).
            heading_drift = (
                on_straight
                and dist >= min_update
                and self._last_valid_goal_yaw is not None
                and abs(self._wrap_angle(goal_yaw - self._last_valid_goal_yaw)) < math.radians(7.0)
            )

            if dist < min_update and not heading_drift:
                self._prev_sharp = in_sharp_turn
                return

            # If nav is active and this is not a straight refresh, wait for the
            # current goal to complete (curves and sharp turns are unaffected).
            if _was_nav_active and not heading_drift:
                self._prev_sharp = in_sharp_turn
                return

            # Straight refresh — rover has moved min_update, lane is still straight,
            # cancel and resend with the updated goal position.
            if heading_drift and _was_nav_active and self._current_goal_handle is not None:
                self.get_logger().info(
                    f'[NORM] straight refresh preempt — '
                    f'hdg={math.degrees(abs(self._wrap_angle(goal_yaw - self._last_valid_goal_yaw))):.1f}°  dist={dist:.2f}m'
                )
                self._intentional_cancel_gen = self._goal_generation
                self._current_goal_handle.cancel_goal_async()
                self._current_goal_handle    = None
                self._nav_active             = False

        # ── 8. Publish goal ───────────────────────────────────────────────────────
        # If nav is still active and we had no anchor (last_goal_xy was None),
        # don't send — wait for current goal to finish.
        if _was_nav_active:
            self._prev_sharp = in_sharp_turn
            return

        self.last_goal_xy = (goal_x, goal_y)
        self._prev_sharp  = in_sharp_turn

        self._last_valid_goal_yaw  = goal_yaw
        self._last_valid_time_s    = self.get_clock().now().nanoseconds * 1e-9
        if not in_sharp_turn and blend_curve > 0.2:
            self._last_valid_curve_sign = curve_sign

        self._is_rotating           = False
        self._rotation_accumulated  = 0.0
        self._rotation_target_yaw   = None
        self._lane_loss_start_s     = None
        self._recovery_phase        = 0
        self._recovery_start_yaw    = None

        self._publish_goal_marker(goal_x, goal_y, goal_yaw, sharp=in_sharp_turn)
        self._send_action_goal(goal_x, goal_y, goal_yaw)

        self.get_logger().info(
            f'[{"SHARP" if in_sharp_turn else "NORM "}] '
            f'curv={curvature:.2e}  bc={blend_curve:.2f}  bs={blend_sharp:.2f}  '
            f'py={py}  goal=({goal_x:.2f},{goal_y:.2f})  '
            f'hdg={math.degrees(goal_yaw - rover_yaw):+.1f}°'
            + (f'  drift_trigger={heading_drift}' if on_straight else '')
            + (f'  csign={curve_sign:+.0f} obias={curve_sign*outer_bias:+.2f}m'
               if not in_sharp_turn else '')
        )

    # =========================================================================
    #  LANE-LOSS RECOVERY
    # =========================================================================

    def _watchdog_cb(self) -> None:
        """
        Fires at WATCHDOG_HZ (10 Hz), independently of /lane_status.
        Detects topic silence and triggers rotation recovery.
        Also runs the goal proximity checker on every tick.
        """
        if not self._mission_enabled:
            return
        if not self._nav_server_ready or self.current_odom is None:
            return
        if self._last_lane_status_time_s is None:
            return

        now_s   = self.get_clock().now().nanoseconds * 1e-9
        silence = now_s - self._last_lane_status_time_s

        self._check_goal_proximity()

        # ── Trigger A: topic silence ──────────────────────────────────────────
        if silence >= Cfg.LANE_SILENCE_THRESH_S:
            if not self._nav_active:
                self.get_logger().warn(f'[WATCHDOG] Topic silent {silence:.1f}s')
                self._try_rotation_recovery()
            return

        if self._nav_active:
            return

        # ── Trigger B: detection loss (topic alive, no good lanes) ────────────
        if self._last_valid_time_s is None:
            detection_gap = silence
        else:
            detection_gap = now_s - self._last_valid_time_s

        if detection_gap >= Cfg.LANE_LOSS_DETECT_S:
            self.get_logger().warn(
                f'[WATCHDOG] No good detection for {detection_gap:.1f}s — recovery'
            )
            self._try_rotation_recovery()

    def _check_goal_proximity(self) -> None:
        """
        Declare the current goal REACHED when xy_err ≤ XY_GOAL_TOL.

        XY only — no yaw check. These are intermediate stepping-stone waypoints;
        the goal yaw is the lane tangent at send time and will naturally differ
        from the rover heading on arrival as the lane curves. Yaw correction is
        handled by the next goal. Nav2's own checker is intentionally bypassed
        here for a tighter update cycle.
        """
        if not self._nav_active:
            return
        if self._current_goal_x is None or self.current_odom is None:
            return

        rover_x = self.current_odom.position.x
        rover_y = self.current_odom.position.y

        xy_err = math.hypot(rover_x - self._current_goal_x,
                            rover_y - self._current_goal_y)

        if xy_err > Cfg.XY_GOAL_TOL:
            return

        self._cancel_current_goal()
        self._goal_generation += 1

        self._nav_active       = False
        self.last_goal_xy      = None
        self._current_goal_x   = None
        self._current_goal_y   = None
        self._current_goal_yaw = None

        self.get_logger().info(
            f'[nav] Proximity reached: xy={xy_err:.2f} m — next goal'
        )

    def _cancel_current_goal(self) -> None:
        """
        Cancel the active Nav2 goal so controller_server stops publishing cmd_vel.
        Marks _intentional_cancel_gen so _on_nav_result treats the resulting
        ABORTED status as intentional, not a real failure.
        """
        if self._current_goal_handle is not None:
            self._intentional_cancel_gen = self._goal_generation
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None

    def _try_rotation_recovery(self) -> None:
        """
        Two-phase lane-loss recovery via direct /cmd_vel spin.

        Phase 0: rotate ~RECOVERY_YAW_TARGET_RAD (≈95°) toward outside of curve.
        Phase 1: rotate ~185° in the opposite direction (full turnaround search).

        Direct /cmd_vel is used because Nav2/MPPI declares pure in-place yaw
        goals done in ~20 ms without the rover actually turning.
        """
        if self._is_rotating:
            return
        if self._nav_active:
            return
        if self.current_odom is None:
            return
        if self._last_valid_goal_yaw is None:
            q = self.current_odom.orientation
            self._last_valid_goal_yaw = 2.0 * math.atan2(q.z, q.w)
            self.get_logger().warn('[ROTATE] No lane heading — seeding from odom')

        self._cancel_current_goal()

        now_s = self.get_clock().now().nanoseconds * 1e-9

        if self._lane_loss_start_s is None:
            self._lane_loss_start_s    = now_s
            self._recovery_phase       = 0
            self._recovery_start_yaw   = None
            self._rotation_accumulated = 0.0
            self.get_logger().warn('[ROTATE] Lane lost — starting recovery Phase 0')

        age = now_s - self._lane_loss_start_s
        if age > Cfg.LANE_LOSS_TIMEOUT_S:
            if age < Cfg.LANE_LOSS_TIMEOUT_S + 2.0:
                self.get_logger().error('[ROTATE] Recovery timeout — aborting')
            return

        q         = self.current_odom.orientation
        start_yaw = 2.0 * math.atan2(q.z, q.w)

        if self._recovery_start_yaw is None:
            self._recovery_start_yaw = start_yaw

        if self._recovery_phase == 0:
            rot_dir      = -self._last_valid_curve_sign if self._last_valid_curve_sign != 0.0 else -1.0
            phase_name   = 'OUTSIDE (~95°)'
            target_delta = Cfg.RECOVERY_YAW_TARGET_RAD
        else:
            rot_dir      =  self._last_valid_curve_sign if self._last_valid_curve_sign != 0.0 else 1.0
            phase_name   = 'TURNAROUND (~185°)'
            target_delta = math.radians(185)

        self._is_rotating = True
        self.get_logger().warn(
            f'[ROTATE] Phase{self._recovery_phase}({phase_name}) '
            f'dir={rot_dir:+.0f}  target={math.degrees(target_delta):.0f}°  age={age:.1f}s'
        )

        threading.Thread(
            target=self._spin_to_heading,
            args=(start_yaw, rot_dir, target_delta),
            daemon=True,
        ).start()

    def _spin_to_heading(self, start_yaw: float, rot_dir: float, target_delta: float) -> None:
        """
        Background thread: publish /cmd_vel angular velocity until the rover has
        rotated target_delta radians from start_yaw, then stop.

        Exits early if the lane is recovered (_is_rotating cleared by _lane_cb).

        Args:
            start_yaw:    Yaw at the start of this rotation phase (radians).
            rot_dir:      +1 = CCW, -1 = CW.
            target_delta: Total rotation to complete in this phase (radians).
        """
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
                yaw_delta   = abs(self._wrap_angle(current_yaw - start_yaw))

                if yaw_delta >= target_delta:
                    self.cmd_vel_pub.publish(stop)
                    self.get_logger().warn(
                        f'[ROTATE] Phase{self._recovery_phase} spin done — '
                        f'turned {math.degrees(yaw_delta):.1f}°'
                    )
                    self._is_rotating = False
                    if self._recovery_phase == 0:
                        self._recovery_phase       = 1
                        self._recovery_start_yaw   = current_yaw
                        self._rotation_accumulated = 0.0
                    return

                self.cmd_vel_pub.publish(twist)
                time.sleep(Cfg.RECOVERY_POLL_S)

        except Exception as e:
            self.get_logger().error(f'[ROTATE] Spin thread error: {e}')
            self._is_rotating = False
            self.cmd_vel_pub.publish(stop)

    # =========================================================================
    #  NAV2 ACTION DISPATCH
    # =========================================================================

    def _send_action_goal(self, x: float, y: float, yaw: float) -> None:
        """
        Send a NavigateToPose goal, save its pose for proximity checking,
        and wire up generation-tagged lifecycle callbacks so stale results
        from preempted goals are silently ignored.
        """
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
            self.get_logger().warn('[nav] Goal rejected by Nav2 — unlocking')
            self._nav_active = False
            return

        self._nav_active = True
        self._current_goal_handle = handle
        self.get_logger().debug('[nav] Goal accepted — proximity checker active')

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
            self.get_logger().warn(f'[nav] Result callback error: {e}')

        self._current_goal_handle = None
        self._nav_active = False

        intentional = (gen == self._intentional_cancel_gen)

        if status == STATUS_SUCCEEDED or status == STATUS_CANCELED or intentional:
            self.get_logger().info(
                f'[nav] Goal done (status={status}, intentional={intentional})'
            )
            self._consecutive_failures = 0
            if not self._prev_sharp:
                self.last_goal_xy = None
        else:
            self._consecutive_failures += 1
            self.get_logger().warn(
                f'[nav] Goal FAILED (status={status}) — '
                f'failure #{self._consecutive_failures}, skipping forward'
            )
            # Push the anchor forward so next goal skips past the blocked spot.
            # After MAX_CONSECUTIVE_FAILURES give up and let lane recovery take over.
            if (self._consecutive_failures < Cfg.MAX_CONSECUTIVE_FAILURES
                    and self._current_goal_x is not None
                    and self.current_odom is not None):
                q         = self.current_odom.orientation
                rover_yaw = 2.0 * math.atan2(q.z, q.w)
                skip      = Cfg.OBSTACLE_SKIP_M * self._consecutive_failures
                self.last_goal_xy = (
                    self._current_goal_x + skip * math.cos(rover_yaw),
                    self._current_goal_y + skip * math.sin(rover_yaw),
                )
            else:
                self.get_logger().error(
                    f'[nav] {self._consecutive_failures} consecutive failures — '
                    'clearing anchor, lane recovery may trigger'
                )
                self.last_goal_xy = None
                self._consecutive_failures = 0
            self._prev_sharp = False


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