"""
═══════════════════════════════════════════════════════════════════════════════
config_params.py  —  rover26_autonomy
═══════════════════════════════════════════════════════════════════════════════

Master configuration file for the current lane-following pipeline:
    lane_detection_node.py  →  lane_goal_publisher.py  →  Nav2

CHANGELOG (this revision)
─────────────────────────
The old curvature proxy was `abs(centre_poly[0])` — the quadratic 'a'
coefficient of x_px = a·y_px² + b·y_px + c, fitted on a PERSPECTIVE frame.
That metric is broken:

  * On a hard turn the lane runs near-horizontal in the image, so x = f(y)
    degenerates and the fit gets FLATTER, not steeper.
  * Real logs from a 90° curve showed |a| = 8.1e-4, while this file's own
    comment records straights producing |a| up to 1.3e-3. The curve read as
    LESS curved than a straight. blend never rose above 0.12 → the sharp-turn
    branch was mathematically unreachable and the rover cut the corner.

Replaced with a ground-truth metric: the lane centre's BEND ANGLE, measured in
metres between a near anchor row and the far look-ahead row. This saturates
correctly in a 90° turn and is immune to the x=f(y) degeneracy.

Removed (were unused, or belonged to the dead three-tier logic):
    CURVE_THRESHOLD, SHARP_THRESHOLD, PY_NEAR, GOAL_DIST_SHARP_M,
    OUTER_BIAS_MAX_M, BIAS_BLEND_POWER, FWD_FACTOR_DIV, ALPHA_SHARP_YAW,
    UPDATE_M_CURVE, MAX_CONSECUTIVE_FAILURES, OBSTACLE_SKIP_M

Added:
    PY_ANCHOR, SHARP_ANGLE_RAD, TANGENT_ROWS, PREEMPT_BLEND

CLASS INDEX
───────────
LaneGoalPublisher  ACTIVE   — goal computation, recovery, Nav2 tolerances
RosTopics          ACTIVE   — central registry of every ROS topic name and TF frame
Physical           ACTIVE   — fixed calibration constants read by lane_goal_publisher
LanePublisher      DORMANT  — synthetic LaserScan params
PotholeDetection   DORMANT  — blob filter thresholds

═══════════════════════════════════════════════════════════════════════════════
"""

# ═══════════════════════════════════════════════════════════════════════════════
#  IMAGE DIMENSIONS (must match the frame size lane coefficients are fitted in)
# ═══════════════════════════════════════════════════════════════════════════════
# The cv-package LaneDetectionNode fits polynomials on the full undistorted
# camera frame (cameras.yaml: 1280x720), NOT a 640x480 BEV warp. These MUST
# track the camera resolution or every pixel→metre conversion is wrong.
#
# NOTE: with no BEV warp, the linear pixel→metre mapping via GROUND_*_M below
# is only an approximation on a perspective image — good near the bottom of the
# frame, increasingly wrong toward the horizon.

IMG_WIDTH:  int = 1280
IMG_HEIGHT: int = 720


# ═══════════════════════════════════════════════════════════════════════════════
#  POTHOLE DETECTION — DORMANT (no node currently imports this)
# ═══════════════════════════════════════════════════════════════════════════════

class PotholeDetection:
    """
    Pothole detection algorithm tunable parameters.

    DORMANT: kept for a future pothole_detection_node — nothing in the
    current lane_detection_node / lane_goal_publisher pipeline reads this.
    """

    # ── White blob area filtering (pixels²) ───────────────────────────────────
    MIN_AREA: int = 500       # Minimum pothole blob size
    MAX_AREA: int = 70000     # Reject oversized blobs (lane lines, rover reflections)

    # ── Shape descriptor thresholds ───────────────────────────────────────────
    MIN_CIRCULARITY:  float = 0.8
    MAX_CIRCULARITY:  float = 1.0
    MAX_ASPECT_RATIO: float = 3.0
    MIN_SOLIDITY:     float = 0.40

    # ── Detection range (metres) ──────────────────────────────────────────────
    MIN_DETECTION_M: float = 0.0
    MAX_DETECTION_M: float = 10.0

    # ── Costmap publish gate (metres) ─────────────────────────────────────────
    COSTMAP_PUBLISH_M: float = 9.0

    RADIUS_SCALE: float = 1.0
    MIN_RADIUS_M: float = 0.10
    MAX_RADIUS_M: float = 0.20


# ═══════════════════════════════════════════════════════════════════════════════
#  LANE GOAL PUBLISHER PARAMETERS
# ═══════════════════════════════════════════════════════════════════════════════

class LaneGoalPublisher:
    """
    Lane-following goal publisher parameters.

    CURVATURE MODEL
    ───────────────
    Every frame, the node evaluates the fused centre-lane polynomial at two
    image rows and converts both to ground metres:

        anchor = row PY_ANCHOR * IMG_H   (just in front of the rover)
        probe  = row PY_FAR    * IMG_H   (far look-ahead)

        bend  = atan2(y_probe - y_anchor, x_probe - x_anchor)   [signed radians]
        blend = min(1, |bend| / SHARP_ANGLE_RAD)                [0 = straight, 1 = sharp]

    `blend` then drives, continuously:
        * look-ahead row       PY_FAR .............. PY_SHARP
        * goal update spacing  UPDATE_M_STRAIGHT ... UPDATE_M_SHARP
        * lateral smoothing    ALPHA_STRAIGHT ...... ALPHA_CURVE
        * goal heading         aim-at-goal ......... lane tangent
        * goal preemption      off ................. on (above PREEMPT_BLEND)

    TUNING GUIDE — SHARP_ANGLE_RAD
    ──────────────────────────────
    Drive your 90° curve and watch the `bend=` field in the [goal] log line.
        Straight road    →  bend hovers around ±3..5°
        Real 90° curve   →  bend climbs to 25..50°
    Set SHARP_ANGLE_RAD to roughly 60% of the PEAK bend you observe in the
    curve. That keeps blend ≈ 1 through the whole corner instead of pinning
    it near zero the way the old |a| threshold did.
    """

    # ── Standalone debug mode ─────────────────────────────────────────────────
    # True → run WITHOUT Nav2 / odometry: rover assumed fixed at the origin,
    # goals computed, logged and published as RViz markers, but NO
    # NavigateToPose action dispatched and recovery spins disabled.
    # MUST be False for real driving.
    DEBUG_STANDALONE: bool = False

    # ── Curvature: lane bend angle over the look-ahead window ─────────────────
    PY_ANCHOR:       float = 0.92    # Near row ("right in front of the rover")
    SHARP_ANGLE_RAD: float = 0.45    # ≈26° of bend → blend saturates at 1.0
    TANGENT_ROWS:    int   = 25      # ± rows used to measure the lane tangent at the goal
    PREEMPT_BLEND:   float = 0.30    # Above this blend, a new goal cancels the active one

    # ── Look-ahead pixel row fraction of IMG_H ────────────────────────────────
    # Smaller fraction → higher row in the image → further ahead on the ground.
    #
    # IMPORTANT: the detection frame is a PERSPECTIVE image (no BEV warp), so
    # lane pixels only exist below the horizon (~row 300/720 ≈ 0.42). Sampling
    # above that extrapolates the polynomials outside their fitted range and
    # produces garbage (negative lane widths, ±10 m lateral offsets). Keep both
    # fractions comfortably below the horizon.
    #
    # PY_FAR   = 0.50 → x_fwd ≈ 4.6 m — far look-ahead on straights
    # PY_SHARP = 0.75 → x_fwd ≈ 2.3 m — short stepping stones in a 90° turn
    #                   (was 0.65 → 3.2 m, still far too long for a corner)
    PY_FAR:   float = 0.50
    PY_SHARP: float = 0.75

    # ── Lateral position IIR smoothers ────────────────────────────────────────
    # Lower alpha = faster response to new lane position.
    ALPHA_STRAIGHT: float = 0.50   # Heavy smoothing on straight segments
    ALPHA_CURVE:    float = 0.22   # Quicker reaction in curves

    # ── Goal update throttle (minimum displacement between consecutive goals) ─
    # Prevents flooding Nav2 with nearly identical goals. Each new
    # NavigateToPose goal triggers one full planning call.
    #
    # UPDATE_M_STRAIGHT was 3.00 — that alone guaranteed the corner-cut: once a
    # goal was committed the node would not recompute for 3 m of travel, which
    # is longer than the entire 90° bend.
    UPDATE_M_STRAIGHT: float = 1.00
    UPDATE_M_SHARP:    float = 0.20

    # Hard ceiling: never freeze the goal for more than this fraction of the
    # current look-ahead distance, regardless of the blend.
    UPDATE_FRAC_OF_LOOKAHEAD: float = 0.40

    # ── Goal proximity tolerance ──────────────────────────────────────────────
    # A goal is declared REACHED when xy_err ≤ XY_GOAL_TOL. XY only — no yaw
    # check (a yaw check caused circling).
    # Was 0.65 m: against a 2.3 m sharp-turn goal that let the rover coast
    # 1.65 m before asking for anything new.
    XY_GOAL_TOL: float = 0.35   # metres

    # ── Lane geometry sanity gates ────────────────────────────────────────────
    MIN_LANE_WIDTH_PX: float = 10.0   # Reject frames whose fitted lanes cross/invert

    # ── Lane-loss / rotation recovery ─────────────────────────────────────────
    LANE_SILENCE_THRESH_S:    float = 0.30   # Topic considered silent after this gap (s)
    LANE_LOSS_DETECT_S:       float = 1.50   # Good-detection gap before recovery (s)
    LANE_LOSS_TIMEOUT_S:      float = 18.0   # Abort recovery after this time (s)
    RECOVERY_YAW_TARGET_RAD:  float = 1.6580 # Rotation search target ≈ 95° (rad)
    RECOVERY_ROT_VEL:         float = 0.40   # In-place spin speed (rad/s)
    RECOVERY_POLL_S:          float = 0.05   # Spin thread yaw-check interval (s)

    # ── Watchdog timer rate ───────────────────────────────────────────────────
    WATCHDOG_HZ: float = 10.0   # How often _watchdog_cb fires (Hz)

    # ── RViz goal marker history ──────────────────────────────────────────────
    MAX_MARKERS: int = 25   # Rolling marker history kept in RViz


# ═══════════════════════════════════════════════════════════════════════════════
#  LANE PATH PUBLISHER (Generic Points) — DORMANT
# ═══════════════════════════════════════════════════════════════════════════════

class LanePublisher:
    """
    Parameters for converting lane polynomials to a synthetic LaserScan.

    DORMANT: kept for generic_points_publisher_node, in case lane edges are
    later fed into Nav2's local costmap as a second obstacle source.
    """

    # ── Sampling and visualization ────────────────────────────────────────────
    N_LANE_SAMPLES: int = 100        # Points sampled per lane for point cloud
    N_SCAN_BINS:    int = 720        # Rays in synthetic LaserScan (0.5° resolution)

    # ── Publishing rates (Hz) ─────────────────────────────────────────────────
    SCAN_HZ:   float = 20.0
    MARKER_HZ: float = 5.0

    # ── RViz marker appearance ────────────────────────────────────────────────
    MARKER_LINE_WIDTH_M: float = 0.08
    MARKER_Z_LIFT_M:     float = 0.05

    # ── Coordinate frames ─────────────────────────────────────────────────────
    ROBOT_FRAME: str = 'base_footprint'
    LIDAR_FRAME: str = 'lidar_link'


# ═══════════════════════════════════════════════════════════════════════════════
#  ROS TOPICS — CENTRAL TOPIC REGISTRY
# ═══════════════════════════════════════════════════════════════════════════════

class RosTopics:
    """
    Centralized ROS 2 topic names and TF frame IDs.

    ACTIVE:

        lane_detection_node  →  /lane_detection  [interfaces/LaneDetectionResult]
                                        ↓
                              lane_goal_publisher  →  navigate_to_pose  →  bt_navigator
                                        ↓                    ↑
                              /lane_goals_viz (RViz)    /odom (odometry)
                                        ↓
                              /cmd_vel  (recovery-spin only, direct — not via twist_mux)

    DORMANT (reserved, not currently published or subscribed):

        LANE_SCAN, LANE_MARKERS, POTHOLE_DETECTIONS, POTHOLE_OBSTACLES
    """

    # ── Active topics ─────────────────────────────────────────────────────────
    LANE_DETECTION:  str = '/lane_detection'   # Published by: lane_detection_node
    LANE_GOALS_VIZ:  str = '/lane_goals_viz'   # Published by: lane_goal_publisher (RViz)
    ODOM:            str = '/odom'             # Published by: odom_tf_broadcaster
    CMD_VEL:         str = '/cmd_vel'          # Recovery-spin only — bypasses twist_mux

    # ── Dormant topics ────────────────────────────────────────────────────────
    LANE_SCAN:          str = '/detection/scan'
    LANE_MARKERS:       str = '/detection/markers'
    POTHOLE_DETECTIONS: str = '/pothole_detections'
    POTHOLE_OBSTACLES:  str = '/pothole_obstacles'

    # ── TF frame IDs ──────────────────────────────────────────────────────────
    ODOM_FRAME: str = 'odom'


# ═══════════════════════════════════════════════════════════════════════════════
#  PHYSICAL & CALIBRATION CONSTANTS
# ═══════════════════════════════════════════════════════════════════════════════

class Physical:
    """
    Fixed physical constants read by lane_goal_publisher.py.
    These change only when hardware changes, not when tuning driving behaviour.
    """

    # ── Ground patch visible in the camera view ───────────────────────────────
    GROUND_WIDTH_M:  float = 15.0    # Left-right extent of the view (metres)

    # Forward extent: distance from the bottom image row to the top of the
    # visible road. Sets goal distance: x_fwd = (1 - py/IMG_H) * this * 0.92.
    # If goals land too close, raise it; too far, lower it.
    GROUND_HEIGHT_M: float = 10.0

    LANE_WIDTH_M:    float = 6.0     # Physical lane width — MEASURE on your track

    # ── Lateral bias trim ─────────────────────────────────────────────────────
    # ⚠ THIS VALUE IS UNVERIFIED AND MUST BE RE-CALIBRATED.
    #
    # The previous file shipped LATERAL_BIAS_M = 0.4 with a comment claiming it
    # had been calibrated to -1.271. Those contradict each other, so one of them
    # is stale and neither can be trusted. It is set to 0.0 here so the trim
    # contributes NOTHING until you actually measure it — a wrong trim is worse
    # than no trim, because it silently dominates y_lat on straights (in the
    # captured log, lane geometry contributed ≈ -0.08 m of lateral offset while
    # the 0.4 m trim contributed five times that).
    #
    # HOW TO CALIBRATE
    # ────────────────
    #   1. Drive the rover straight down a centred lane.
    #   2. Read "y_lat=" from the [goal] log line with LATERAL_BIAS_M = 0.0.
    #   3. Set LATERAL_BIAS_M to the NEGATIVE of the average value you see —
    #      the code does y_lat += LATERAL_BIAS_M, so cancelling a raw offset of
    #      +0.43 m requires a trim of -0.43, not +0.43.
    #   4. Rebuild. y_lat should now read ≈ 0.00 m when centred and straight.
    LATERAL_BIAS_M: float = 0.0


# ═══════════════════════════════════════════════════════════════════════════════
#  CONVENIENCE ALIASES
# ═══════════════════════════════════════════════════════════════════════════════

IMG_W: int = IMG_WIDTH    # Detection frame width (pixels)
IMG_H: int = IMG_HEIGHT   # Detection frame height (pixels)


# ═══════════════════════════════════════════════════════════════════════════════
#  PUBLIC API
# ═══════════════════════════════════════════════════════════════════════════════

__all__ = [
    'IMG_WIDTH', 'IMG_HEIGHT', 'IMG_W', 'IMG_H',
    # Active
    'LaneGoalPublisher',
    'RosTopics',
    'Physical',
    # Dormant
    'LanePublisher',
    'PotholeDetection',
]