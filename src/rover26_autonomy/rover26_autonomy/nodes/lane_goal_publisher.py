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
from std_msgs.msg           import ColorRGBA
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
        self._y_lat_smooth:     float | None      = None   # Lateral IIR state (normal mode)
        self._goal_yaw_smooth:  float | None      = None   # Heading IIR state (sharp mode)
        self._prev_sharp:       bool              = False  # Previous-frame mode flag
        self._sharp_exit_goals: int               = 0      # Dense-goal momentum after sharp exit

        # Goal lifecycle: only one active Nav2 goal at a time.
        # _nav_active is True from goal ACCEPTED until _on_nav_result fires OR
        # _check_goal_proximity detects arrival. While True, _lane_cb skips computation.
        self._nav_active:       bool              = False
        self._goal_generation:  int               = 0      # Generation tag for stale callbacks
        self._intentional_cancel_gen: int         = -1     # Generation of intentional cancel
        self._current_goal_handle                 = None   # Active Nav2 goal handle for cancellation

        # Current goal pose — saved when sent, cleared on completion
        self._current_goal_x:   float | None      = None
        self._current_goal_y:   float | None      = None
        self._current_goal_yaw: float | None      = None

        # ── Rotation recovery state ─────────────────────────────────────────────
        self._last_valid_goal_yaw:     float | None  = None   # Heading at last good detection
        self._last_valid_time_s:       float | None  = None   # Wall-clock seconds of last good det.
        self._last_lane_status_time_s: float | None  = None   # When ANY LaneStatus msg last arrived
        self._last_valid_curve_sign:   float          = 0.0   # +1 = left curve, -1 = right
        self._is_rotating:             bool           = False  # True while spin thread is active
        self._rotation_target_yaw:    float | None  = None
        self._rotation_accumulated:   float          = 0.0
        self._lane_loss_start_s:      float | None  = None
        self._recovery_phase:         int            = 0      # 0 = Phase 0 (~95°), 1 = Phase 1 (~185°)
        self._recovery_start_yaw:     float | None  = None

        # ── RViz marker cycling ─────────────────────────────────────────────────
        self._marker_id: int = 0

        # ── Nav2 action client ──────────────────────────────────────────────────
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── Publishers ──────────────────────────────────────────────────────────
        # RViz goal visualisation (arrows + spheres, coloured by mode)
        self.viz_pub = self.create_publisher(MarkerArray, RosTopics.LANE_GOALS_VIZ, 10)

        # Direct velocity publisher for recovery rotation.
        # MPPI declares pure-yaw Nav2 goals done in ~20 ms without the rover
        # actually turning — we must publish /cmd_vel directly and poll odometry.
        self.cmd_vel_pub = self.create_publisher(Twist, RosTopics.CMD_VEL, 10)

        # ── Subscriptions ────────────────────────────────────────────────────────
        self.create_subscription(LaneStatus, RosTopics.LANE_STATUS, self._lane_cb, 10)
        self.create_subscription(Odometry,   RosTopics.ODOM,        self._odom_cb, 10)

        # ── Watchdog timer ───────────────────────────────────────────────────────
        # Fires at WATCHDOG_HZ regardless of /lane_status activity.
        # Detects when the topic goes completely silent and triggers recovery.
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

    # =========================================================================
    #  CURVATURE BLEND HELPERS  (each returns 0 → 1)
    # =========================================================================

    def _curve_blend(self, curvature: float) -> float:
        """
        0 = dead straight, 1 = curvature at or above CURVE_THRESHOLD.
        Drives the straight → gentle-curve transition.
        """
        if Cfg.CURVE_THRESHOLD <= 0:
            return 0.0
        return min(1.0, curvature / Cfg.CURVE_THRESHOLD)

    def _sharp_blend(self, curvature: float) -> float:
        """
        0 = at CURVE_THRESHOLD (gentle curve), 1 = at/above SHARP_THRESHOLD.
        Drives the gentle-curve → sharp-turn transition.
        """
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

        # Scale from pixel-ratio to metric ratio using ground plane dimensions
        delta_x_fwd = Physical.GROUND_HEIGHT_M * 0.92 / IMG_H  # Metres per pixel row
        delta_y_lat = slope * Physical.GROUND_WIDTH_M / IMG_W   # Metres lateral per row

        theta_local = math.atan2(delta_y_lat, delta_x_fwd)     # Local heading offset
        return rover_yaw + theta_local                           # World-frame yaw

    @staticmethod
    def _wrap_angle(a: float) -> float:
        """Wrap an angle to [−π, π] to avoid jumps in the IIR heading smoother."""
        return math.atan2(math.sin(a), math.cos(a))

    # =========================================================================
    #  RVIZ MARKERS
    # =========================================================================

    def _publish_goal_marker(self, goal_x: float, goal_y: float,
                              goal_yaw: float, sharp: bool = False) -> None:
        """
        Publish an arrow + sphere marker pair at the computed goal position.

        Arrow is RED in sharp-turn mode, GREEN in normal mode — instantly
        distinguishable in RViz which tier is active.

        Args:
            goal_x:   Goal X in odom frame (metres).
            goal_y:   Goal Y in odom frame (metres).
            goal_yaw: Goal heading in odom frame (radians).
            sharp:    True if the goal was computed in sharp-turn mode.
        """
        now     = self.get_clock().now().to_msg()
        markers = MarkerArray()

        qz = math.sin(goal_yaw / 2.0)
        qw = math.cos(goal_yaw / 2.0)

        # Arrow — shows both position and heading direction
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
        # Red = sharp turn (stepping-stone mode), Green = normal (position-based)
        arrow.color              = (ColorRGBA(r=1.0, g=0.15, b=0.0, a=0.95) if sharp
                                    else ColorRGBA(r=0.0, g=0.95, b=0.1,  a=0.95))
        arrow.lifetime.sec       = 8

        # Sphere — marks the exact goal position
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

        # Cycle marker ID to keep a rolling history of the last N goals
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

        # Stamp arrival of every LaneStatus message — even rejected ones.
        # The watchdog uses this to detect when the topic goes completely silent.
        self._last_lane_status_time_s = self.get_clock().now().nanoseconds * 1e-9

        # ── Guard: need Nav2 + odometry before anything else ────────────────────
        if not self._nav_server_ready or self.current_odom is None:
            return

        left_ok  = msg.left_detected  and len(msg.left_coeffs)  == 3
        right_ok = msg.right_detected and len(msg.right_coeffs) == 3

        # ── Wait for the active goal to finish before computing a new one ────────
        if self._nav_active:
            # Special case: rotation recovery was running but the lane just came back.
            # Interrupt the rotation and immediately compute a real goal.
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
            else:
                return  # Goal active — wait for proximity check to advance it

        # ── Any lane missing → trigger rotation recovery ─────────────────────────
        if not (left_ok and right_ok):
            self._try_rotation_recovery()
            return

        # ── 1. Lane geometry (both lanes confirmed) ──────────────────────────────
        left_c   = np.array(msg.left_coeffs,  dtype=float)
        right_c  = np.array(msg.right_coeffs, dtype=float)
        centre_c = (left_c + right_c) / 2.0   # True centre-lane polynomial

        # |a| of the quadratic is the curvature proxy (units: pixels⁻¹)
        curvature = abs(centre_c[0])

        # Compute blend factors (each is 0 → 1)
        blend_curve = self._curve_blend(curvature)   # Straight → gentle curve
        blend_sharp = self._sharp_blend(curvature)   # Gentle curve → sharp turn

        # ── 2. Rover pose ────────────────────────────────────────────────────────
        q         = self.current_odom.orientation
        rover_yaw = 2.0 * math.atan2(q.z, q.w)
        rover_x   = self.current_odom.position.x
        rover_y   = self.current_odom.position.y

        # ── 3. Adaptive look-ahead row ───────────────────────────────────────────
        # Blend through three anchor points as curvature increases:
        #   PY_FAR   (0.52) → x_fwd ≈ 4.4 m  on straight  (stable, long range)
        #   PY_NEAR  (0.85) → x_fwd ≈ 1.8 m  on curves    (goal stays on arc, not chord)
        #   PY_SHARP (0.85) → x_fwd ≈ 1.4 m  on sharp     (very close stepping stones)
        py_frac = (Cfg.PY_FAR
                   + blend_curve * (Cfg.PY_NEAR  - Cfg.PY_FAR)   # Curve pushes toward NEAR
                   + blend_sharp * (Cfg.PY_SHARP - Cfg.PY_NEAR)) # Sharp pushes to SHARP
        py = int(np.clip(IMG_H * py_frac, 0, IMG_H - 1))

        # ── 4. Lane tangent heading (used by both modes) ─────────────────────────
        goal_yaw = self._lane_tangent_yaw(centre_c, py, rover_yaw)

        # ── 5. Mode decision ─────────────────────────────────────────────────────
        # blend_sharp > 0.10 activates SHARP mode:
        #   threshold 0.10 → triggers at curvature ≈ 0.0019 (was 0.0028 at 0.30)
        #   This cuts the last long NORM goal short for tighter turn entries.
        in_sharp_turn = blend_sharp > 0.10
        outer_bias    = 0.0   # Initialised here; set inside NORMAL block; used in log

        # ── Sharp-exit momentum ───────────────────────────────────────────────────
        # After leaving sharp mode, keep the dense update throttle for 2 more
        # publications so the rover finishes the apex before returning to long
        # look-ahead.  We let normal mode compute a proper position-based goal —
        # only min_update is overridden to stay dense.
        if self._prev_sharp and not in_sharp_turn:
            self._sharp_exit_goals = 2
            self.last_goal_xy = None   # Re-anchor to rover position for exit goals
        _sharp_exit_active = (not in_sharp_turn and self._sharp_exit_goals > 0)

        # ╔══════════════════════════════════════════════════════════════════════╗
        # ║  SHARP TURN MODE  (90° corners)                                    ║
        # ║                                                                     ║
        # ║  Why: the quadratic lateral offset is unreliable when the lane      ║
        # ║  curves sharply — the fit degrades, the lane may exit the frame,    ║
        # ║  and a far goal causes cutting or oscillation.                      ║
        # ║                                                                     ║
        # ║  What: ignore lateral position, trust only the tangent heading at   ║
        # ║  a near row, then project a short (GOAL_DIST_SHARP_M) waypoint in  ║
        # ║  that direction.  Nav2 receives a rapid sequence of stepping stones  ║
        # ║  and follows the turn smoothly.                                     ║
        # ╚══════════════════════════════════════════════════════════════════════╝
        if in_sharp_turn:
            # Seed the heading smoother on first entry to sharp mode
            if self._goal_yaw_smooth is None or not self._prev_sharp:
                self._goal_yaw_smooth = goal_yaw
            else:
                delta = self._wrap_angle(goal_yaw - self._goal_yaw_smooth)
                self._goal_yaw_smooth += (1.0 - Cfg.ALPHA_SHARP_YAW) * delta

            heading = self._goal_yaw_smooth

            # Project from the furthest point reached (not the rover's odom position).
            # Using last_goal_xy ensures each new goal is GOAL_DIST_SHARP_M ahead of
            # the previous one — a true leapfrog chain through the turn.
            if self.last_goal_xy is not None:
                base_x, base_y = self.last_goal_xy
            else:
                base_x, base_y = rover_x, rover_y

            goal_x = base_x + Cfg.GOAL_DIST_SHARP_M * math.cos(heading)
            goal_y = base_y + Cfg.GOAL_DIST_SHARP_M * math.sin(heading)

            # Reset lateral smoother so normal mode restarts cleanly
            self._y_lat_smooth = None
            min_update = Cfg.UPDATE_M_SHARP

        # ╔══════════════════════════════════════════════════════════════════════╗
        # ║  NORMAL MODE  (straight + gentle curve)                            ║
        # ║  Position-based logic with outer-wall bias on curves.              ║
        # ╚══════════════════════════════════════════════════════════════════════╝
        else:
            # Reset sharp heading smoother so it seeds fresh on next entry
            self._goal_yaw_smooth = None

            # Lateral position of lane centre at look-ahead row
            centre_px = np.polyval(centre_c, py)
            if not (0 <= centre_px < IMG_W):
                return   # Lane centre out of image — skip this frame

            # Convert pixel coordinates to metric offsets from rover
            x_fwd = (1.0 - py / IMG_H) * Physical.GROUND_HEIGHT_M * 0.92
            # Sharp-exit momentum: clamp look-ahead to match sharp-turn step size
            if _sharp_exit_active:
                x_fwd = min(x_fwd, Cfg.GOAL_DIST_SHARP_M)

            # Compute y_lat as rover-relative lateral offset.
            #
            # OLD (wrong when rover is off-centre in the lane):
            #   y_lat = (0.5 - centre_px / IMG_W) * GROUND_WIDTH_M
            #   Measures lane centre relative to image centre, assuming the rover
            #   is always at IMG_W/2. If the rover drifts the goal drifts with it.
            #
            # NEW (correct):
            #   rover_px  = lane midpoint at the bottom row (where rover is now)
            #   offset_px = signed distance from rover to lane centre at lookahead
            #   Scaled by actual lane width in pixels -> metres, so the correction
            #   is accurate regardless of where the lanes sit in the image.
            left_px_bottom  = np.polyval(left_c,  IMG_H)
            right_px_bottom = np.polyval(right_c, IMG_H)
            rover_px        = (left_px_bottom + right_px_bottom) / 2.0

            left_px_look  = np.polyval(left_c,  py)
            right_px_look = np.polyval(right_c, py)
            lane_width_px = right_px_look - left_px_look
            if lane_width_px < 10:
                return  # Degenerate — skip

            # positive offset_px = lane centre is LEFT of rover → steer left
            offset_px = rover_px - centre_px
            metres_per_px = Physical.LANE_WIDTH_M / lane_width_px
            y_lat  = offset_px * metres_per_px
            y_lat += Physical.LATERAL_BIAS_M   # Per-rover calibration trim

            # ── Outer bias ────────────────────────────────────────────────────
            # Pushes the goal toward the OUTSIDE of the curve to widen the arc.
            #
            # BUG FIXED (was cutting inside every curve):
            #   Old code used np.sign(centre_c[0]) as curve direction.  The 'a'
            #   coefficient flip-flops sign between frames on the same physical curve
            #   (polynomial fit instability), so the bias alternated between inward
            #   and outward ~50% of the time.
            #
            # Fix: derive curve direction from goal_yaw - rover_yaw (heading offset).
            #   This is computed from atan2 and is stable — it never flip-flops.
            #   theta_local > 0 → turning left  → outside = -y_lat → curve_sign = -1
            #   theta_local < 0 → turning right → outside = +y_lat → curve_sign = +1
            theta_local = self._wrap_angle(goal_yaw - rover_yaw)
            curve_sign  = -np.sign(theta_local) if abs(theta_local) > 1e-3 else 0.0
            bias_factor = blend_curve ** Cfg.BIAS_BLEND_POWER
            fwd_factor  = min(1.0, x_fwd / Cfg.FWD_FACTOR_DIV)
            outer_bias  = (Cfg.OUTER_BIAS_MAX_M
                           * bias_factor
                           * fwd_factor
                           * (1.0 - blend_sharp))     # Fades to zero near sharp threshold
            y_lat_bias  = y_lat + curve_sign * outer_bias

            # Lateral IIR smoother — heavier on straight, lighter on curves
            alpha = (Cfg.ALPHA_STRAIGHT
                     + blend_curve * (Cfg.ALPHA_CURVE - Cfg.ALPHA_STRAIGHT))

            if self._y_lat_smooth is None:
                self._y_lat_smooth = y_lat_bias   # Seed on first frame
            else:
                self._y_lat_smooth = (alpha * self._y_lat_smooth
                                      + (1.0 - alpha) * y_lat_bias)
            y_lat_used = self._y_lat_smooth

            # Transform goal from rover-local frame to odom frame
            # x_fwd is along rover heading, y_lat_used is perpendicular (left = +)
            cos_yaw = math.cos(rover_yaw)
            sin_yaw = math.sin(rover_yaw)
            goal_x = rover_x + x_fwd * cos_yaw - y_lat_used * sin_yaw
            goal_y = rover_y + x_fwd * sin_yaw + y_lat_used * cos_yaw

            # Update interval: sparse on straight, denser on curves
            min_update = (Cfg.UPDATE_M_STRAIGHT
                          + blend_curve * (Cfg.UPDATE_M_CURVE - Cfg.UPDATE_M_STRAIGHT))

            # Sharp-exit momentum: keep dense throttle for 2 goals after sharp mode
            if _sharp_exit_active:
                min_update = Cfg.UPDATE_M_SHARP
                self._sharp_exit_goals -= 1
                self.get_logger().info(
                    f'[NORM] sharp-exit momentum — {self._sharp_exit_goals} goals remaining'
                )

        # ── 7. Goal throttle ─────────────────────────────────────────────────────
        # Skip if the new goal is too close to the previous one to avoid flooding Nav2.
        if self.last_goal_xy is not None:
            dist = math.hypot(goal_x - self.last_goal_xy[0],
                              goal_y - self.last_goal_xy[1])
            if dist < min_update:
                self._prev_sharp = in_sharp_turn
                return

        # ── 8. Publish goal ───────────────────────────────────────────────────────
        self.last_goal_xy = (goal_x, goal_y)
        self._prev_sharp  = in_sharp_turn

        # Record heading + curve direction for rotation recovery
        self._last_valid_goal_yaw  = goal_yaw
        self._last_valid_time_s    = self.get_clock().now().nanoseconds * 1e-9
        if not in_sharp_turn and blend_curve > 0.2:
            self._last_valid_curve_sign = curve_sign

        # Lane is visible → reset any in-progress rotation
        self._is_rotating           = False
        self._rotation_accumulated  = 0.0
        self._rotation_target_yaw   = None
        self._lane_loss_start_s     = None
        self._recovery_phase        = 0
        self._recovery_start_yaw    = None

        self._publish_goal_marker(goal_x, goal_y, goal_yaw, sharp=in_sharp_turn)
        self._send_action_goal(goal_x, goal_y, goal_yaw)

        # Log tier, curvature, blend values, and goal for debugging
        self.get_logger().info(
            f'[{"SHARP" if in_sharp_turn else "NORM "}] '
            f'curv={curvature:.2e}  bc={blend_curve:.2f}  bs={blend_sharp:.2f}  '
            f'py={py}  goal=({goal_x:.2f},{goal_y:.2f})  '
            f'hdg={math.degrees(goal_yaw - rover_yaw):+.1f}°'
            + (f'  csign={curve_sign:+.0f} obias={curve_sign*outer_bias:+.2f}m'
               if not in_sharp_turn else '')
        )

    # =========================================================================
    #  LANE-LOSS RECOVERY
    # =========================================================================

    def _watchdog_cb(self) -> None:
        """
        Fires at WATCHDOG_HZ (10 Hz), independently of /lane_status.

        ROOT CAUSE ADDRESSED: the lane detector stops publishing /lane_status
        entirely when the lane exits the BEV frame. _lane_cb can never catch this;
        only a separate timer can detect the silence.

        Also checks goal proximity at every tick so the rover advances to the
        next goal as soon as position + heading tolerances are met, without
        waiting for Nav2's slower built-in checker.
        """
        if not self._nav_server_ready or self.current_odom is None:
            return
        if self._last_lane_status_time_s is None:
            return   # No LaneStatus received yet — nothing to fall back to

        now_s   = self.get_clock().now().nanoseconds * 1e-9
        silence = now_s - self._last_lane_status_time_s

        # Always run proximity check — even while the lane is visible
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
        Declare the current goal REACHED when:
          xy_err ≤ XY_GOAL_TOL   (rover is near the waypoint position)

        WHY XY ONLY — NO YAW CHECK:
        These are intermediate lane-following stepping stones, not final
        parking positions. The goal yaw is the lane tangent computed 1.4–4.4 m
        ahead at goal-send time. By arrival the lane has curved, so the rover's
        heading can differ by 20–40° from _current_goal_yaw. A yaw check here
        can NEVER be satisfied, leaving _nav_active = True forever and causing
        MPPI to circle indefinitely.
        Yaw correction is handled naturally by the very next goal sent after
        this one fires. Nav2's yaml yaw_goal_tolerance is also irrelevant here:
        this function cancels the Nav2 goal before Nav2's checker can evaluate it.

        WHY NOT RELY ON NAV2'S CHECKER?
        Nav2's SimpleGoalChecker only fires after MPPI has settled. Here we
        advance as soon as XY tolerance is met — typically 0.5–1.0 s earlier,
        giving the lane-following loop a tighter update cycle.

        RACE-CONDITION SAFETY: _goal_generation is incremented so that any
        pending _on_nav_result callback for the preempted goal is a no-op.
        """
        if not self._nav_active:
            return
        if self._current_goal_x is None or self.current_odom is None:
            return

        rover_x   = self.current_odom.position.x
        rover_y   = self.current_odom.position.y
        q         = self.current_odom.orientation
        rover_yaw = 2.0 * math.atan2(q.z, q.w)

        xy_err = math.hypot(rover_x - self._current_goal_x,
                            rover_y - self._current_goal_y)

        # [FIX-CIRCLE] XY check only — no yaw check.
        # Nav2 yaw_goal_tolerance in the yaml is irrelevant here: this function
        # cancels the Nav2 goal before Nav2's checker can evaluate it. Yaw
        # correction is handled by the next goal sent after this fires.
        if xy_err > Cfg.XY_GOAL_TOL:
            return   # Not there yet

        # XY tolerance met — advance to next goal.
        # Cancel the Nav2 goal FIRST so controller_server stops publishing cmd_vel.
        # Without this, Nav2 keeps running the old goal and its MPPI cmd_vel
        # fights with the recovery spin on /cmd_vel.
        self._cancel_current_goal()
        self._goal_generation += 1   # Invalidate pending Nav2 result callback

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
        Cancel the current Nav2 goal so controller_server stops publishing
        cmd_vel. Called before starting recovery rotation and before proximity-
        based goal advancement, to ensure the spin thread has exclusive control
        of /cmd_vel without interference from the MPPI controller.
        Marks _intentional_cancel_gen so _on_nav_result treats the resulting
        ABORTED status as an intentional cancel, not a real failure.
        """
        if self._current_goal_handle is not None:
            # Record which generation we are intentionally cancelling so
            # _on_nav_result doesn't treat the resulting ABORTED as a failure.
            self._intentional_cancel_gen = self._goal_generation
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None

    def _try_rotation_recovery(self) -> None:
        """
        Two-phase lane-loss recovery via direct /cmd_vel spin.

        WHY /cmd_vel DIRECTLY (not Nav2)?
        Nav2/MPPI declares pure in-place yaw goals done in ~20 ms without the
        rover actually turning. We must publish angular velocity directly and
        poll odometry until the target heading delta is reached.

        Phase 0: rotate ~RECOVERY_YAW_TARGET_RAD (≈95°) toward outside of curve.
        Phase 1: rotate ~185° in the opposite direction (full turnaround search).
        """
        if self._is_rotating:
            return   # Spin thread already running
        if self._nav_active:
            return
        if self.current_odom is None:
            return
        if self._last_valid_goal_yaw is None:
            q = self.current_odom.orientation
            self._last_valid_goal_yaw = 2.0 * math.atan2(q.z, q.w)
            self.get_logger().warn('[ROTATE] No lane heading — seeding from odom')

        # Cancel any active Nav2 goal so controller_server stops publishing
        # cmd_vel. Without this, MPPI keeps outputting velocity commands that
        # fight the recovery spin on /cmd_vel, causing small oscillations
        # instead of the intended clean 95° + 185° rotation.
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

        # Phase parameters
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
        stop            = Twist()   # Zero velocity

        try:
            while rclpy.ok():
                # Exit if lane was recovered from the main thread
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
                        # Watchdog will call _try_rotation_recovery for Phase 1
                    # Phase 1 done — watchdog detects lane silence and stops
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

        Args:
            x:   Goal X in odom frame (metres).
            y:   Goal Y in odom frame (metres).
            yaw: Goal heading in odom frame (radians).
        """
        # Save goal pose so _check_goal_proximity can compare against odometry
        self._current_goal_x   = x
        self._current_goal_y   = y
        self._current_goal_yaw = yaw

        # Stamp this generation so stale callbacks from old goals are ignored
        self._goal_generation += 1
        gen = self._goal_generation
        # Mark that this generation was intentionally cancelled (preemption/proximity).
        # _on_nav_result uses this to distinguish intentional abort from real failure.
        self._intentional_cancel_gen: int = -1  # Reset on each new goal send

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
        # Capture gen in closure so each callback knows which generation it belongs to
        send_future.add_done_callback(lambda f, g=gen: self._on_goal_response(f, g))

    def _on_goal_response(self, future, gen: int) -> None:
        """
        Called when Nav2 responds (accepted or rejected).
        Tagged with generation so stale responses are silently dropped.
        """
        if gen != self._goal_generation:
            return   # A newer goal was already sent — ignore this response

        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('[nav] Goal rejected by Nav2 — unlocking')
            self._nav_active = False
            return

        self._nav_active = True
        self._current_goal_handle = handle   # Store so we can cancel if needed
        self.get_logger().debug('[nav] Goal accepted — proximity checker active')

        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda f, g=gen: self._on_nav_result(f, g))

    def _on_nav_result(self, future, gen: int) -> None:
        """
        Called when Nav2 reports SUCCEEDED / FAILED / CANCELLED.

        If _check_goal_proximity already advanced the generation, gen will
        not match and this callback is a no-op (avoids double-unlock).
        """
        if gen != self._goal_generation:
            return   # Stale result from a proximity-preempted goal — ignore

        STATUS_SUCCEEDED = 4
        STATUS_CANCELED  = 5
        status = 0
        try:
            status = future.result().status
        except Exception as e:
            self.get_logger().warn(f'[nav] Result callback error: {e}')

        self._current_goal_handle = None
        self._nav_active = False

        # Treat as intentional if this generation was explicitly cancelled
        # by preemption or proximity checker — even if Nav2 reports ABORTED.
        # Without this, a preemption cancel comes back as status=6 ABORTED,
        # triggers "Goal FAILED → force retry", resets last_goal_xy, and
        # breaks the throttle — causing a new goal to fire immediately after
        # the preemption already sent one, creating a 170ms update loop.
        intentional = (gen == self._intentional_cancel_gen)

        if status == STATUS_SUCCEEDED or status == STATUS_CANCELED or intentional:
            self.get_logger().info(
                f'[nav] Goal done (status={status}, intentional={intentional})'
            )
            if not self._prev_sharp:
                self.last_goal_xy = None
        else:
            # Genuine failure — retry immediately
            self.get_logger().warn(
                f'[nav] Goal FAILED (status={status}) — forcing immediate retry'
            )
            self.last_goal_xy = None
            self._prev_sharp  = False


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