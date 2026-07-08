#!/usr/bin/env python3
"""
═════════════════════════════════════════════════════════════════════════════════
lane_goal_publisher.py  —  rover26_autonomy          [INSTRUMENTED DIAG BUILD]
═════════════════════════════════════════════════════════════════════════════════

DIAGNOSTIC LOGGING
──────────────────
Every log line added by this build is prefixed with [DIAG] and can be located
or stripped with:   grep -n 'DIAG' lane_goal_publisher.py

What gets logged (throttled to DIAG_THROTTLE_S where high-rate):
  [DIAG:raw]    Raw left/right coefficient lists exactly as received (incl. lengths)
  [DIAG:poly]   Per-side a,b,c + fused centre polynomial
  [DIAG:shape]  Per-side x-pixel positions at 3 image rows + lane width at each
                → instantly shows if one side's polynomial is garbage
  [DIAG:slope]  Per-side and centre slope at look-ahead row, resulting local
                heading angle per side → shows which side poisons the tangent
  [DIAG:geom]   theta_local, delta components, py, blends, mode decision
  [DIAG:norm]   Normal-mode lateral pipeline: centre_px, offset_px, m/px,
                y_lat raw → biased → smoothed
  [DIAG:sharp]  Sharp-mode heading pipeline: raw yaw → smoothed yaw → goal
  [DIAG:thr]    Goal throttle decision (dist, min_update, heading_drift, verdict)
  [DIAG:nav]    Every action dispatch / response / result with generation number
  [DIAG:prox]   Proximity checker distance every tick while a goal is active
  [DIAG:wdog]   Watchdog silence / detection-gap values

Original functionality is unchanged — only logging added.
═════════════════════════════════════════════════════════════════════════════════

FUNCTIONALITY
─────────────
Converts lane polynomial coefficients from lane_detection_node into a stream of
Nav2 NavigateToPose goals, enabling the rover to follow the lane autonomously.

Implements three-tier goal computation based on road curvature:

  Tier 1 — STRAIGHT   |a| < CURVE_THRESHOLD
  Tier 2 — CURVE      CURVE_THRESHOLD ≤ |a| < SHARP_THRESHOLD
  Tier 3 — SHARP      |a| ≥ SHARP_THRESHOLD  (90° corners)

See original header for full documentation.
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
from interfaces.msg         import LaneDetectionResult
from nav2_msgs.action       import NavigateToPose

from rover26_autonomy.config_params import IMG_W, IMG_H, LaneGoalPublisher as Cfg, Physical, RosTopics

# ── DIAG configuration ────────────────────────────────────────────────────────
DIAG_THROTTLE_S: float = 0.5   # Throttle period for high-rate diag lines
DIAG_ROWS = (0.25, 0.50, 0.75) # Image-row fractions sampled for [DIAG:shape]


# ═════════════════════════════════════════════════════════════════════════════
#  LANE GOAL PUBLISHER NODE
# ═════════════════════════════════════════════════════════════════════════════

class LaneGoalPublisher(Node):
    """
    Converts lane polynomials to Nav2 NavigateToPose goals in three tiers.
    Instrumented diagnostic build — see module docstring.
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

        # ── DIAG state ──────────────────────────────────────────────────────────
        self._diag_frame:        int          = 0      # frames received
        self._diag_last_stamp_s: float | None = None   # for inter-frame period

        # ── RViz marker cycling ─────────────────────────────────────────────────
        self._marker_id: int = 0

        # ── Nav2 action client ──────────────────────────────────────────────────
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── Publishers ──────────────────────────────────────────────────────────
        self.viz_pub     = self.create_publisher(MarkerArray, RosTopics.LANE_GOALS_VIZ, 10)
        self.cmd_vel_pub = self.create_publisher(Twist,       RosTopics.CMD_VEL,        10)

        # ── Subscriptions ────────────────────────────────────────────────────────
        self.create_subscription(
            LaneDetectionResult, RosTopics.LANE_DETECTION, self._lane_cb, 10
        )
        self.create_subscription(Odometry, RosTopics.ODOM, self._odom_cb, 10)

        # ── Watchdog timer ───────────────────────────────────────────────────────
        self.create_timer(1.0 / Cfg.WATCHDOG_HZ, self._watchdog_cb)

        # ── Nav server ready-check (background thread) ───────────────────────────
        threading.Thread(target=self._wait_for_nav_server, daemon=True).start()

        self.get_logger().info(
            'lane_goal_publisher started — THREE-TIER MODE  [DIAG BUILD]\n'
            f'  curve threshold : |a| > {Cfg.CURVE_THRESHOLD:.4f}\n'
            f'  sharp threshold : |a| > {Cfg.SHARP_THRESHOLD:.4f}\n'
            f'  look-ahead      : far={Cfg.PY_FAR}  near={Cfg.PY_NEAR}  sharp={Cfg.PY_SHARP}\n'
            f'  outer bias max  : {Cfg.OUTER_BIAS_MAX_M} m\n'
            f'  recovery vel    : {Cfg.RECOVERY_ROT_VEL} rad/s\n'
            f'  DIAG throttle   : {DIAG_THROTTLE_S}s'
        )

    # =========================================================================
    #  STARTUP / ODOMETRY
    # =========================================================================

    def _wait_for_nav_server(self) -> None:
        """
        Background thread: block until bt_navigator is ACTIVE by probing with
        a trivial goal.
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
        Polynomial: x_pixel = a·py² + b·py + c ;  slope at py = 2a·py + b.
        """
        a, b, _ = centre_c
        slope = 2.0 * a * py + b

        delta_x_fwd = Physical.GROUND_HEIGHT_M * 0.92 / IMG_H
        delta_y_lat = slope * Physical.GROUND_WIDTH_M / IMG_W

        theta_local = math.atan2(delta_y_lat, delta_x_fwd)

        # ── [DIAG:slope] fused-tangent internals ──────────────────────────────
        self.get_logger().info(
            f'[DIAG:slope] centre: a={a:+.4e} b={b:+.4e} slope@py{py}={slope:+.3f}px/row  '
            f'dx_fwd={delta_x_fwd:.5f}m/row  dy_lat={delta_y_lat:+.5f}m/row  '
            f'theta_local={math.degrees(theta_local):+.1f}°',
            throttle_duration_sec=DIAG_THROTTLE_S,
        )

        return rover_yaw + theta_local

    @staticmethod
    def _wrap_angle(a: float) -> float:
        """Wrap an angle to [−π, π]."""
        return math.atan2(math.sin(a), math.cos(a))

    # =========================================================================
    #  DIAG HELPERS
    # =========================================================================

    def _side_theta_deg(self, coeffs: np.ndarray, py: int) -> float:
        """[DIAG] Local heading angle (deg) implied by a single side's polynomial."""
        a, b, _ = coeffs
        slope = 2.0 * a * py + b
        dx = Physical.GROUND_HEIGHT_M * 0.92 / IMG_H
        dy = slope * Physical.GROUND_WIDTH_M / IMG_W
        return math.degrees(math.atan2(dy, dx))

    def _log_lane_shape(self, left_c: np.ndarray, right_c: np.ndarray,
                        centre_c: np.ndarray, py: int) -> None:
        """
        [DIAG] Dump per-side polynomial coefficients, sampled x-positions at
        three image rows, lane width per row, and per-side implied heading.

        This is the single most useful block for the current bug: it shows
        immediately whether one side's fit is geometrically insane (points
        off-image, negative/huge lane width, wildly different per-side heading).
        """
        rows = [int(f * IMG_H) for f in DIAG_ROWS]

        def fmt_side(name, c):
            xs = [np.polyval(c, r) for r in rows]
            pos = '  '.join(f'r{r}:x={x:7.1f}px' for r, x in zip(rows, xs))
            return (f'{name}: a={c[0]:+.4e} b={c[1]:+.4e} c={c[2]:+.1f}  |  {pos}')

        widths = '  '.join(
            f'r{r}:w={np.polyval(right_c, r) - np.polyval(left_c, r):7.1f}px'
            for r in rows
        )

        th_l = self._side_theta_deg(left_c,  py)
        th_r = self._side_theta_deg(right_c, py)
        th_c = self._side_theta_deg(centre_c, py)

        self.get_logger().info(
            '[DIAG:poly]\n'
            f'  {fmt_side("LEFT ", left_c)}\n'
            f'  {fmt_side("RIGHT", right_c)}\n'
            f'  {fmt_side("CENTR", centre_c)}\n'
            f'  [DIAG:shape] lane width  {widths}   (expect ≈ constant, positive, '
            f'~{Physical.LANE_WIDTH_M / (Physical.GROUND_WIDTH_M / IMG_W):.0f}px for '
            f'{Physical.LANE_WIDTH_M}m lane)\n'
            f'  [DIAG:shape] heading@py{py}  left={th_l:+.1f}°  right={th_r:+.1f}°  '
            f'centre={th_c:+.1f}°   (per-side mismatch ⇒ one fit is garbage)',
            throttle_duration_sec=DIAG_THROTTLE_S,
        )

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
    #  MAIN CALLBACK — runs on every /lane_detection message
    # =========================================================================

    def _lane_cb(self, msg: LaneDetectionResult) -> None:
        """
        Receive a LaneDetectionResult message and compute + dispatch the next
        Nav2 goal.  (Instrumented — see [DIAG] blocks.)
        """
        now_s = self.get_clock().now().nanoseconds * 1e-9
        self._last_lane_status_time_s = now_s

        # ── [DIAG:raw] message arrival + raw payload ──────────────────────────
        self._diag_frame += 1
        period = (now_s - self._diag_last_stamp_s) if self._diag_last_stamp_s else 0.0
        self._diag_last_stamp_s = now_s
        self.get_logger().info(
            f'[DIAG:raw] frame#{self._diag_frame}  dt={period*1000:.0f}ms  '
            f'left(len={len(msg.left_lane_coeffs)})={[f"{c:.4e}" for c in msg.left_lane_coeffs]}  '
            f'right(len={len(msg.right_lane_coeffs)})={[f"{c:.4e}" for c in msg.right_lane_coeffs]}',
            throttle_duration_sec=DIAG_THROTTLE_S,
        )

        if not self._nav_server_ready or self.current_odom is None:
            self.get_logger().info(
                f'[DIAG:raw] gated: nav_ready={self._nav_server_ready} '
                f'odom={"yes" if self.current_odom else "no"}',
                throttle_duration_sec=2.0,
            )
            return

        left_ok  = len(msg.left_lane_coeffs)  == 3
        right_ok = len(msg.right_lane_coeffs) == 3

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

        _was_nav_active = self._nav_active  # capture before geometry runs

        if not (left_ok and right_ok):
            self.get_logger().warn(
                f'[DIAG:raw] side missing — left_ok={left_ok} right_ok={right_ok} '
                '→ rotation recovery path',
                throttle_duration_sec=DIAG_THROTTLE_S,
            )
            self._try_rotation_recovery()
            return

        # ── 1. Lane geometry ─────────────────────────────────────────────────────
        left_c   = np.array(msg.left_lane_coeffs,  dtype=float)
        right_c  = np.array(msg.right_lane_coeffs, dtype=float)
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

        # ── [DIAG:poly / DIAG:shape] full per-side geometry dump ──────────────
        self._log_lane_shape(left_c, right_c, centre_c, py)

        # ── 4. Lane tangent heading ──────────────────────────────────────────────
        goal_yaw = self._lane_tangent_yaw(centre_c, py, rover_yaw)

        # ── 5. Mode decision ─────────────────────────────────────────────────────
        in_sharp_turn = blend_sharp > 0.10
        outer_bias    = 0.0

        # ── [DIAG:geom] mode decision inputs ──────────────────────────────────
        self.get_logger().info(
            f'[DIAG:geom] |a_L|={abs(left_c[0]):.3e} |a_R|={abs(right_c[0]):.3e} '
            f'|a_C|={curvature:.3e}  thr(curve={Cfg.CURVE_THRESHOLD:.1e} '
            f'sharp={Cfg.SHARP_THRESHOLD:.1e})  bc={blend_curve:.2f} bs={blend_sharp:.2f}  '
            f'py_frac={py_frac:.2f} py={py}  mode={"SHARP" if in_sharp_turn else "NORM"}  '
            f'rover=({rover_x:.2f},{rover_y:.2f},{math.degrees(rover_yaw):.1f}°)',
            throttle_duration_sec=DIAG_THROTTLE_S,
        )

        if self._prev_sharp and not in_sharp_turn:
            self._sharp_exit_goals = 2
            self.last_goal_xy = None
        _sharp_exit_active = (not in_sharp_turn and self._sharp_exit_goals > 0)

        # ╔══════════════════════════════════════════════════════════════════════╗
        # ║  SHARP TURN MODE  (90° corners)                                    ║
        # ╚══════════════════════════════════════════════════════════════════════╝
        if in_sharp_turn:
            _yaw_raw = goal_yaw
            if self._goal_yaw_smooth is None or not self._prev_sharp:
                self._goal_yaw_smooth = goal_yaw
            else:
                delta = self._wrap_angle(goal_yaw - self._goal_yaw_smooth)
                self._goal_yaw_smooth += (1.0 - Cfg.ALPHA_SHARP_YAW) * delta

            heading = self._goal_yaw_smooth

            base_x, base_y = rover_x, rover_y

            goal_x = base_x + Cfg.GOAL_DIST_SHARP_M * math.cos(heading)
            goal_y = base_y + Cfg.GOAL_DIST_SHARP_M * math.sin(heading)

            # ── [DIAG:sharp] heading pipeline ────────────────────────────────
            self.get_logger().info(
                f'[DIAG:sharp] yaw_raw={math.degrees(_yaw_raw):+.1f}°  '
                f'yaw_smooth={math.degrees(heading):+.1f}°  '
                f'rel_hdg={math.degrees(self._wrap_angle(heading - rover_yaw)):+.1f}°  '
                f'proj={Cfg.GOAL_DIST_SHARP_M}m → goal=({goal_x:.2f},{goal_y:.2f})  '
                f'⚠ GOAL_DIST_SHARP_M={Cfg.GOAL_DIST_SHARP_M} vs XY_GOAL_TOL={Cfg.XY_GOAL_TOL}',
                throttle_duration_sec=DIAG_THROTTLE_S,
            )

            self._y_lat_smooth = None
            min_update = Cfg.UPDATE_M_SHARP

        # ╔══════════════════════════════════════════════════════════════════════╗
        # ║  NORMAL MODE  (straight + gentle curve)                            ║
        # ╚══════════════════════════════════════════════════════════════════════╝
        else:
            self._goal_yaw_smooth = None

            centre_px = np.polyval(centre_c, py)
            if not (0 <= centre_px < IMG_W):
                self.get_logger().warn(
                    f'[DIAG:norm] REJECT — centre_px={centre_px:.1f} off-image '
                    f'(0..{IMG_W}) at py={py}',
                    throttle_duration_sec=DIAG_THROTTLE_S,
                )
                return

            x_fwd = (1.0 - py / IMG_H) * Physical.GROUND_HEIGHT_M * 0.92
            if _sharp_exit_active:
                x_fwd = min(x_fwd, Cfg.GOAL_DIST_SHARP_M)

            left_px_look  = np.polyval(left_c,  py)
            right_px_look = np.polyval(right_c, py)
            lane_width_px = right_px_look - left_px_look
            if lane_width_px < 10:
                self.get_logger().warn(
                    f'[DIAG:norm] REJECT — lane_width_px={lane_width_px:.1f} < 10 '
                    f'(left_px={left_px_look:.1f} right_px={right_px_look:.1f}) at py={py}',
                    throttle_duration_sec=DIAG_THROTTLE_S,
                )
                return

            offset_px     =  IMG_W / 2.0 - centre_px
            metres_per_px = Physical.LANE_WIDTH_M / lane_width_px
            y_lat         = offset_px * metres_per_px
            y_lat_pre_bias = y_lat
            y_lat        += Physical.LATERAL_BIAS_M

            # Outer-wall bias
            theta_local = self._wrap_angle(goal_yaw - rover_yaw)
            curve_sign  = -np.sign(theta_local) if abs(theta_local) > 1e-3 else 0.0
            bias_factor = blend_curve ** Cfg.BIAS_BLEND_POWER
            fwd_factor  = min(1.0, x_fwd / Cfg.FWD_FACTOR_DIV)
            outer_bias  = (Cfg.OUTER_BIAS_MAX_M
                           * bias_factor
                           * fwd_factor
                           * (1.0 - blend_sharp))
            y_lat_bias  = y_lat + curve_sign * outer_bias

            # Lateral IIR smoother
            alpha = (Cfg.ALPHA_STRAIGHT
                     + blend_curve * (Cfg.ALPHA_CURVE - Cfg.ALPHA_STRAIGHT))

            _y_smooth_prev = self._y_lat_smooth
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

            # ── [DIAG:norm] lateral pipeline ─────────────────────────────────
            self.get_logger().info(
                f'[DIAG:norm] centre_px={centre_px:.1f} offset_px={offset_px:+.1f}  '
                f'lane_w_px={lane_width_px:.1f} m/px={metres_per_px:.5f}  '
                f'y_lat raw={y_lat_pre_bias:+.3f} +trim={Physical.LATERAL_BIAS_M:+.2f} '
                f'+outer({curve_sign:+.0f}×{outer_bias:.3f})={y_lat_bias:+.3f}  '
                f'IIR(a={alpha:.2f}) {_y_smooth_prev if _y_smooth_prev is None else f"{_y_smooth_prev:+.3f}"}'
                f'→{y_lat_used:+.3f}  x_fwd={x_fwd:.2f}m → goal=({goal_x:.2f},{goal_y:.2f})',
                throttle_duration_sec=DIAG_THROTTLE_S,
            )

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

            heading_drift = (
                on_straight
                and dist >= min_update
                and self._last_valid_goal_yaw is not None
                and abs(self._wrap_angle(goal_yaw - self._last_valid_goal_yaw)) < math.radians(7.0)
            )

            # ── [DIAG:thr] throttle decision ─────────────────────────────────
            verdict = ('SEND' if not (dist < min_update and not heading_drift)
                              and not (_was_nav_active and not heading_drift)
                       else 'HOLD')
            self.get_logger().info(
                f'[DIAG:thr] anchor=({self.last_goal_xy[0]:.2f},{self.last_goal_xy[1]:.2f})  '
                f'dist={dist:.3f}m  min_update={min_update:.2f}m  '
                f'on_straight={on_straight}  drift={heading_drift}  '
                f'nav_active={_was_nav_active}  → {verdict}',
                throttle_duration_sec=DIAG_THROTTLE_S,
            )

            if dist < min_update and not heading_drift:
                self._prev_sharp = in_sharp_turn
                return

            if _was_nav_active and not heading_drift:
                self._prev_sharp = in_sharp_turn
                return

            if heading_drift and _was_nav_active and self._current_goal_handle is not None:
                self.get_logger().info(
                    f'[NORM] straight refresh preempt — '
                    f'hdg={math.degrees(abs(self._wrap_angle(goal_yaw - self._last_valid_goal_yaw))):.1f}°  dist={dist:.2f}m'
                )
                self._intentional_cancel_gen = self._goal_generation
                self._current_goal_handle.cancel_goal_async()
                self._current_goal_handle    = None
                self._nav_active             = False
        else:
            self.get_logger().info(
                f'[DIAG:thr] no anchor — nav_active={_was_nav_active}  '
                f'→ {"HOLD (wait for active goal)" if _was_nav_active else "SEND (fresh)"}',
                throttle_duration_sec=DIAG_THROTTLE_S,
            )

        # ── 8. Publish goal ───────────────────────────────────────────────────────
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
        Fires at WATCHDOG_HZ (10 Hz), independently of /lane_detection.
        """
        if not self._nav_server_ready or self.current_odom is None:
            return
        if self._last_lane_status_time_s is None:
            return

        now_s   = self.get_clock().now().nanoseconds * 1e-9
        silence = now_s - self._last_lane_status_time_s

        # ── [DIAG:wdog] periodic state snapshot ───────────────────────────────
        gap = (now_s - self._last_valid_time_s) if self._last_valid_time_s else -1.0
        self.get_logger().info(
            f'[DIAG:wdog] silence={silence:.2f}s  valid_gap={gap:.2f}s  '
            f'nav_active={self._nav_active}  rotating={self._is_rotating}  '
            f'gen={self._goal_generation}  fails={self._consecutive_failures}',
            throttle_duration_sec=2.0,
        )

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
        """
        if not self._nav_active:
            return
        if self._current_goal_x is None or self.current_odom is None:
            return

        rover_x = self.current_odom.position.x
        rover_y = self.current_odom.position.y

        xy_err = math.hypot(rover_x - self._current_goal_x,
                            rover_y - self._current_goal_y)

        # ── [DIAG:prox] live distance to goal ─────────────────────────────────
        self.get_logger().info(
            f'[DIAG:prox] gen={self._goal_generation}  '
            f'rover=({rover_x:.2f},{rover_y:.2f})  '
            f'goal=({self._current_goal_x:.2f},{self._current_goal_y:.2f})  '
            f'xy_err={xy_err:.3f}m  tol={Cfg.XY_GOAL_TOL}m'
            + ('  ⚠ goal born inside tolerance!' if xy_err <= Cfg.XY_GOAL_TOL else ''),
            throttle_duration_sec=1.0,
        )

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
        """Cancel the active Nav2 goal (marks the cancel as intentional)."""
        if self._current_goal_handle is not None:
            self.get_logger().info(
                f'[DIAG:nav] intentional cancel — gen={self._goal_generation}'
            )
            self._intentional_cancel_gen = self._goal_generation
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None

    def _try_rotation_recovery(self) -> None:
        """
        Two-phase lane-loss recovery via direct /cmd_vel spin.
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
        and wire up generation-tagged lifecycle callbacks.
        """
        self._current_goal_x   = x
        self._current_goal_y   = y
        self._current_goal_yaw = yaw

        self._goal_generation += 1
        gen = self._goal_generation

        # ── [DIAG:nav] dispatch record ────────────────────────────────────────
        rx = self.current_odom.position.x if self.current_odom else float('nan')
        ry = self.current_odom.position.y if self.current_odom else float('nan')
        self.get_logger().info(
            f'[DIAG:nav] SEND gen={gen}  goal=({x:.2f},{y:.2f},{math.degrees(yaw):.1f}°)  '
            f'rover=({rx:.2f},{ry:.2f})  dist_to_goal={math.hypot(x-rx, y-ry):.3f}m  '
            f'tol={Cfg.XY_GOAL_TOL}m'
        )

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
            self.get_logger().info(
                f'[DIAG:nav] stale response gen={gen} (current={self._goal_generation}) — ignored'
            )
            return

        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('[nav] Goal rejected by Nav2 — unlocking')
            self._nav_active = False
            return

        self._nav_active = True
        self._current_goal_handle = handle
        self.get_logger().info(f'[DIAG:nav] ACCEPTED gen={gen} — proximity checker active')

        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda f, g=gen: self._on_nav_result(f, g))

    def _on_nav_result(self, future, gen: int) -> None:
        """Called when Nav2 reports SUCCEEDED / FAILED / CANCELLED."""
        if gen != self._goal_generation:
            self.get_logger().info(
                f'[DIAG:nav] stale result gen={gen} (current={self._goal_generation}) — ignored'
            )
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

        self.get_logger().info(
            f'[DIAG:nav] RESULT gen={gen}  status={status}  intentional={intentional}  '
            f'fails_so_far={self._consecutive_failures}'
        )

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
                self.get_logger().info(
                    f'[DIAG:nav] anchor pushed forward {skip:.2f}m → '
                    f'({self.last_goal_xy[0]:.2f},{self.last_goal_xy[1]:.2f})'
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