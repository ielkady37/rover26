"""
═══════════════════════════════════════════════════════════════════════════════
config_params.py  —  rover26_autonomy
═══════════════════════════════════════════════════════════════════════════════

Master configuration file for the current lane-following pipeline:
    lane_detection_node.py  →  lane_goal_publisher.py  →  Nav2

Trimmed to what the active pipeline imports, PLUS two classes kept on standby
for planned future work: LanePublisher (costmap obstacle scan) and
PotholeDetection. Neither is imported by any node right now — see the
"DORMANT" marker on each for what would need to exist before they do anything.
lane_detection_node.py itself reads its own tuning from a separate YAML file
via Configurator, not this file.

CLASS INDEX
───────────
LaneGoalPublisher  ACTIVE   — three-tier goal computation, recovery, Nav2 tolerances
RosTopics          ACTIVE   — central registry of every ROS topic name and TF frame used
Physical           ACTIVE   — fixed calibration constants actually read by lane_goal_publisher
LanePublisher      DORMANT  — synthetic LaserScan params, for generic_points_publisher_node
                              if lane edges are later fed into Nav2's costmap as obstacles
PotholeDetection   DORMANT  — blob filter thresholds, for a pothole_detection_node
                              that does not exist yet in this package

═══════════════════════════════════════════════════════════════════════════════
"""

# ═══════════════════════════════════════════════════════════════════════════════
#  IMAGE DIMENSIONS (must match the frame size lane coefficients are fitted in)
# ═══════════════════════════════════════════════════════════════════════════════
# The cv-package LaneDetectionNode fits polynomials on the full undistorted
# camera frame (cameras.yaml: 1280x720), NOT a 640x480 BEV warp like the old
# rover26_autonomy lane_detection_node did. These MUST track the camera
# resolution or every pixel→metre conversion in lane_goal_publisher is wrong
# (image centre off by half a frame, look-ahead rows sampled in the top half).
#
# NOTE: with no BEV warp, the linear pixel→metre mapping via GROUND_*_M below
# is only an approximation on a perspective image — good enough near the
# bottom of the frame, increasingly wrong toward the horizon.

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
    MAX_AREA: int = 70000     # Reject oversized blobs (lane lines, rover body reflections)

    # ── Shape descriptor thresholds ───────────────────────────────────────────
    #   Lane lines:     circularity ≈ 0.05  →  easily rejected by MIN_CIRCULARITY
    #   Potholes:       circularity ≈ 0.4+  →  easily accepted
    MIN_CIRCULARITY: float = 0.8
    MAX_CIRCULARITY: float = 1.0
    MAX_ASPECT_RATIO: float = 3.0   # Lane lines are far wider than tall; potholes are not
    MIN_SOLIDITY: float = 0.40      # Lane-line fragments are jagged; potholes are compact

    # ── Detection range (metres) ───────────────────────────────────────────────
    MIN_DETECTION_M: float = 0.0
    MAX_DETECTION_M: float = 10.0

    # ── Costmap publish gate (metres) ─────────────────────────────────────────
    # Potholes farther than this would NOT enter Nav2's costmap, preventing
    # premature swerving. TUNING: raise if rover reaches pothole before
    # dodging; lower if rover swerves well before the pothole is a real threat.
    COSTMAP_PUBLISH_M: float = 9.0

    RADIUS_SCALE: float = 1.0    # tune until logged r matches real pothole size
    MIN_RADIUS_M: float = 0.10   # raise if close potholes look too small in costmap
    MAX_RADIUS_M: float = 0.20   # realistic pothole max


# ═══════════════════════════════════════════════════════════════════════════════
#  LANE GOAL PUBLISHER PARAMETERS
# ═══════════════════════════════════════════════════════════════════════════════

class LaneGoalPublisher:
    """
    Three-tier lane following goal publisher parameters.

    Tier detection is based on the quadratic polynomial coefficient 'a'
    (used as a curvature proxy, units: pixels⁻¹):

        |a| < CURVE_THRESHOLD   →  STRAIGHT  (position-based, long look-ahead)
        |a| < SHARP_THRESHOLD   →  CURVE     (position-based + outer bias)
        |a| >= SHARP_THRESHOLD  →  SHARP     (heading-projected short stepping stones)

    TUNING GUIDE — curvature thresholds
    ─────────────────────────────────────
    Check logs for real curvature values during a 90° turn, then set
    SHARP_THRESHOLD to a value between CURVE_THRESHOLD and that observed peak.
    """

    # ── Standalone debug mode ──────────────────────────────────────────────────
    # True → run WITHOUT Nav2 / odometry: the rover is assumed fixed at the
    # origin, goals are computed, logged and published as RViz markers, but
    # NO NavigateToPose action is dispatched and recovery spins are disabled.
    # Use when bench-testing with only lane_detection_node running.
    # MUST be False for real driving.
    DEBUG_STANDALONE: bool = False

    # ── Curvature thresholds ───────────────────────────────────────────────────
    # Measured on the perspective 1280x720 frame a STRAIGHT road already shows
    # |a_C| up to ~1.3e-3 (perspective + fisheye residual), so the thresholds
    # must sit above that. Drive a real curve/corner, read |a_C| from the
    # [DIAG:geom] log, and tune from there.
    CURVE_THRESHOLD: float = 0.0025   # Gentle-curve regime starts here (pixels⁻¹)
    SHARP_THRESHOLD: float = 0.0070   # Sharp / 90° turn regime starts here (pixels⁻¹)

    # ── Look-ahead pixel row fraction of IMG_H ─────────────────────────────────
    # Smaller fraction → higher row in the image → further ahead on the ground.
    #
    # IMPORTANT: the detection frame is a PERSPECTIVE image (no BEV warp), so
    # lane pixels only exist below the horizon (~row 300/720 ≈ 0.42). Sampling
    # above that extrapolates the polynomials outside their fitted range and
    # produces garbage (negative lane widths, ±10 m lateral offsets). Keep all
    # three fractions comfortably below the horizon.
    # PY_FAR  = 0.50 → far look-ahead on straights (stable, away from the
    #                  fisheye near-field where lane width is exaggerated)
    # PY_NEAR = 0.62 → closer on curves (goal stays on the arc, not the chord)
    # PY_SHARP= 0.65 → very close stepping stones on sharp turns
    # Still ~0.08 above the ~0.42 horizon floor documented above — if you push
    # these lower, watch the annotated lane overlay: py must stay within the
    # visibly-drawn fitted line, or you're extrapolating past real pixels.
    PY_FAR:   float = 0.50
    PY_NEAR:  float = 0.62
    PY_SHARP: float = 0.65

    # ── Sharp-turn goal projection ─────────────────────────────────────────────
    # Fixed forward distance for each stepping stone in sharp-turn mode.
    GOAL_DIST_SHARP_M: float = 0.3   # Metres

    # ── Outer bias (gentle curves only — fades to zero in sharp turns) ─────────
    # Pushes the goal toward the OUTSIDE of the curve to widen the arc.
    # Bias formula: outer_bias = OUTER_BIAS_MAX_M * blend_curve^BIAS_BLEND_POWER
    #                            * min(1, x_fwd / FWD_FACTOR_DIV) * (1 - blend_sharp)
    #
    # TUNING:
    #   Rover still cuts corners → increase OUTER_BIAS_MAX_M toward 1.0
    #   Rover clips the outer wall → decrease toward 0.3
    OUTER_BIAS_MAX_M:  float = 0.48
    BIAS_BLEND_POWER:  float = 1.20
    FWD_FACTOR_DIV:    float = 2.0

    # ── Lateral position IIR smoothers ────────────────────────────────────────
    # Lower alpha = faster response to new lane position.
    ALPHA_STRAIGHT: float = 0.50   # Heavy smoothing on straight segments
    ALPHA_CURVE:    float = 0.22   # Quicker reaction on gentle curves

    # ── Sharp-turn heading IIR smoother ───────────────────────────────────────
    ALPHA_SHARP_YAW: float = 0.45

    # ── Goal update throttle (minimum displacement between consecutive goals) ──
    # Prevents flooding Nav2 with nearly identical goals. Each new NavigateToPose
    # goal triggers one full planning call regardless of the BT.
    UPDATE_M_STRAIGHT: float = 3.00   # One planning call roughly every 1 m of travel
    UPDATE_M_CURVE:    float = 1.00   # Same cadence on gentle curves
    UPDATE_M_SHARP:    float = 0.10   # Very dense during 90° turns

    # ── Goal proximity tolerance ────────────────────────────────────────────────
    # A goal is declared REACHED when xy_err ≤ XY_GOAL_TOL. XY only — no yaw
    # check (a yaw check caused circling; see lane_goal_publisher.py comments).
    XY_GOAL_TOL: float = 0.65   # metres

    # ── Lane-loss handling ─────────────────────────────────────────────────────
    MAX_CONSECUTIVE_FAILURES: int   = 3      # failures before giving up and clearing anchor
    OBSTACLE_SKIP_M:          float = 0.40   # metres to skip forward per failure attempt

    # ── Lane-loss / rotation recovery ─────────────────────────────────────────
    LANE_SILENCE_THRESH_S:    float = 0.30   # Topic considered silent after this gap (s)
    LANE_LOSS_DETECT_S:       float = 1.50   # Good-detection gap before recovery (s)
    LANE_LOSS_TIMEOUT_S:      float = 18.0   # Abort recovery after this time (s)
    RECOVERY_YAW_TARGET_RAD:  float = 1.6580 # Phase 0 rotation target ≈ 95° (rad)
    RECOVERY_ROT_VEL:         float = 0.40   # In-place spin speed (rad/s)
    RECOVERY_POLL_S:          float = 0.05   # Spin thread yaw-check interval (s)

    # ── Watchdog timer rate ────────────────────────────────────────────────────
    WATCHDOG_HZ: float = 10.0   # How often _watchdog_cb fires (Hz)

    # ── RViz goal marker history ───────────────────────────────────────────────
    MAX_MARKERS: int = 25   # Rolling marker history kept in RViz


# ═══════════════════════════════════════════════════════════════════════════════
#  LANE PATH PUBLISHER (Generic Points) — DORMANT (no node currently imports this)
# ═══════════════════════════════════════════════════════════════════════════════

class LanePublisher:
    """
    Parameters for converting lane polynomials to a synthetic LaserScan.

    DORMANT: kept for generic_points_publisher_node, in case lane edges are
    later fed into Nav2's local costmap as a second obstacle source. Not used
    by the current lane_detection_node → lane_goal_publisher pipeline.
    """

    # ── Sampling and visualization ─────────────────────────────────────────────
    N_LANE_SAMPLES: int = 100        # Points sampled per lane for point cloud
    N_SCAN_BINS:    int = 720        # Rays in synthetic LaserScan (0.5° resolution)

    # ── Publishing rates (Hz) ──────────────────────────────────────────────────
    SCAN_HZ:   float = 20.0          # Should match whatever consumes /detection/scan
    MARKER_HZ: float = 5.0           # RViz visualization rate (lower for bandwidth)

    # ── RViz marker appearance ─────────────────────────────────────────────────
    MARKER_LINE_WIDTH_M: float = 0.08
    MARKER_Z_LIFT_M:     float = 0.05   # Slight elevation to prevent z-fighting with ground

    # ── Coordinate frames ──────────────────────────────────────────────────────
    ROBOT_FRAME: str = 'base_footprint'  # Frame for markers and point clouds
    LIDAR_FRAME: str = 'lidar_link'      # Frame for LaserScan — must match real /scan's frame


# ═══════════════════════════════════════════════════════════════════════════════
#  ROS TOPICS — CENTRAL TOPIC REGISTRY
# ═══════════════════════════════════════════════════════════════════════════════

class RosTopics:
    """
    Centralized ROS 2 topic names and TF frame IDs.

    ACTIVE (used by lane_detection_node.py / lane_goal_publisher.py today):

        lane_detection_node  →  /lane_detection  [interfaces/LaneDetectionResult]
                                        ↓
                              lane_goal_publisher  →  navigate_to_pose action  →  bt_navigator
                                        ↓                      ↑
                              /lane_goals_viz (RViz)      /odom (odometry)
                                        ↓
                              /cmd_vel  (recovery-spin only, direct — not through twist_mux)

    DORMANT (topic names reserved for LanePublisher / PotholeDetection above,
    not currently published or subscribed by anything):

        LANE_SCAN, LANE_MARKERS, POTHOLE_DETECTIONS, POTHOLE_OBSTACLES, POTHOLE_SPEED
    """

    # ── Active topics ───────────────────────────────────────────────────────────
    LANE_DETECTION:  str = '/lane_detection'   # Published by: lane_detection_node
    LANE_GOALS_VIZ:  str = '/lane_goals_viz'   # Published by: lane_goal_publisher (RViz)
    ODOM:            str = '/odom'             # Published by: odom_tf_broadcaster
    CMD_VEL:         str = '/cmd_vel'          # Recovery-spin only — bypasses twist_mux intentionally

    # ── Dormant topics (reserved for LanePublisher / PotholeDetection) ─────────
    LANE_SCAN:          str = '/detection/scan'        # Would be: generic_points_publisher_node
    LANE_MARKERS:       str = '/detection/markers'     # Would be: generic_points_publisher_node
    POTHOLE_DETECTIONS: str = '/pothole_detections'    # Would be: pothole_detection_node
    POTHOLE_OBSTACLES:  str = '/pothole_obstacles'     # Would be: pothole_detection_node → Nav2 costmap

    # ── TF frame IDs ────────────────────────────────────────────────────────────
    ODOM_FRAME: str = 'odom'


# ═══════════════════════════════════════════════════════════════════════════════
#  PHYSICAL & CALIBRATION CONSTANTS
# ═══════════════════════════════════════════════════════════════════════════════

class Physical:
    """
    Fixed physical constants actually read by lane_goal_publisher.py.
    These change only when hardware or the BEV warp changes, not when tuning
    day-to-day driving behavior.
    """

    # ── Ground patch visible in the camera view ────────────────────────────────
    GROUND_WIDTH_M:  float = 9.0    # Left-right extent of the view (metres)
    # Forward extent: distance from the bottom image row to the top of the
    # visible road. Sets goal distance: x_fwd = (1 - py/IMG_H) * this * 0.92.
    # If goals land too close, raise it; too far, lower it.
    GROUND_HEIGHT_M: float = 10.0
    LANE_WIDTH_M:    float = 4.0   # Physical lane width (metres) — measure on your track

    # ── Lateral bias trim ──────────────────────────────────────────────────────
    # HOW TO CALIBRATE
    # ────────────────
    #   1. Drive the rover straight down a centred lane.
    #   2. Read "y_lat raw=X.XXX" from the [DIAG:norm] line in
    #      lane_goal_publisher log output (this is the offset BEFORE trim
    #      is applied, so it's unaffected by the current LATERAL_BIAS_M).
    #   3. Set LATERAL_BIAS_M to the NEGATIVE of that value — the code does
    #      y_lat += LATERAL_BIAS_M, so cancelling a raw offset of +0.43 m
    #      requires a trim of -0.43, not +0.43.
    #   4. Rebuild. [DIAG:norm] "y_lat raw=... +trim=..." should now sum to
    #      ≈0.00 m when centred.
    #
    # Last calibrated 2026-07-11: raw offset measured at +1.271 m while
    # centred and driving straight → trim set to -1.271.
    LATERAL_BIAS_M: float = 1.271


# ═══════════════════════════════════════════════════════════════════════════════
#  CONVENIENCE ALIASES
# ═══════════════════════════════════════════════════════════════════════════════

IMG_W: int = IMG_WIDTH    # Detection frame width (pixels) — matches LaneDetectionNode's undistorted frame
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
    # Dormant — reserved for planned future nodes, not currently imported
    'LanePublisher',
    'PotholeDetection',
]