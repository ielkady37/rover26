"""
═══════════════════════════════════════════════════════════════════════════════
config_params.py  —  rover26_autonomy
═══════════════════════════════════════════════════════════════════════════════

Master configuration file for the rover26_autonomy package.

All tunable parameters for lane detection, pothole detection, goal publishing,
and odometry estimation are centralized here. Modify only this file when
transferring to real hardware or tuning behavior on a new platform.

TRANSFER TO REAL HARDWARE CHECKLIST
────────────────────────────────────
1.  Update CAMERA / ODOMETRY sections for your specific hardware.
2.  Run lane detection in manual override mode to verify white lane detection.
3.  Record statistics from lane_detection_node logs.
4.  Gradually increase MAX_DETECTION_M and COSTMAP_PUBLISH_M for potholes.
5.  Set Physical.LATERAL_BIAS_M by driving a straight lane and reading the log.
6.  Test on empty parking lot, then real track.

CLASS INDEX
───────────
  LaneDetection      Sliding-window tracker, steering, debug flags
  PotholeDetection   Blob area/shape/range filters, costmap gate, speed hint
  LanePublisher      Synthetic scan parameters, RViz visualization rates
  LaneGoalPublisher  Three-tier goal computation, recovery, Nav2 tolerances
  RosTopics          Central registry of every ROS topic name and TF frame
  Odometry           Wheel odometry physical geometry and joint names
  Physical           Fixed calibration constants (derived from URDF/camera)
  Debug              Global verbosity and OpenCV display flags

═══════════════════════════════════════════════════════════════════════════════
"""

# ═══════════════════════════════════════════════════════════════════════════════
#  CAMERA & IMAGE PROCESSING
# ═══════════════════════════════════════════════════════════════════════════════

# Image dimensions (must match camera driver and YUYV/BGR encoding)
IMG_WIDTH:  int = 640
IMG_HEIGHT: int = 480

# Bird's-eye view calibration (perspective warp)
#
# TUNING GUIDE
# ─────────────
# Set the perspective transform source points (SRC trapezoid) to match where
# the actual lane lines appear in the raw camera frame.
#   Still V-shape in bird's eye → SRC top points are too close together
#                                 → decrease LEFT_TOP / increase RIGHT_TOP
#   Still A-shape in bird's eye → over-corrected
#                                 → increase LEFT_TOP / decrease RIGHT_TOP
#
WARP_SRC_LEFT_TOP:  int = 200   # x-coordinate of left  lane at SRC top row
WARP_SRC_RIGHT_TOP: int = 440   # x-coordinate of right lane at SRC top row
WARP_SRC_TOP_Y:     int = 170   # y-row where we capture the lane tops

WARP_MARGIN: int = 80           # pixel margin on left/right of warped image

# Expected lane width in bird's-eye view (pixels)
# Used to validate lane separation and recover from frame drops
LANE_WIDTH_PX: int = 240


# ═══════════════════════════════════════════════════════════════════════════════
#  LANE DETECTION NODE PARAMETERS
# ═══════════════════════════════════════════════════════════════════════════════

class LaneDetection:
    """Lane detection algorithm tunable parameters."""

    # ── Sliding-window tracker ─────────────────────────────────────────────────
    N_WINDOWS:      int   = 30             # Number of vertical search windows
    MARGIN:         int   = 60             # Search margin around predicted lane position (px)
    MIN_PIX:        int   = 8              # Minimum white pixels in a window to detect lane
    MAX_GAP_WINDOWS:int   = 4             # Consecutive empty windows before tracker resets
    MAX_X_DRIFT:    int   = 80            # Max pixel jump between consecutive windows
    MIN_SEP_PX:    float  = LANE_WIDTH_PX * 0.55  # Minimum left-right lane separation
    MAX_MISS:       int   = 12            # Frames to keep lane model before giving up

    # ── Steering output ────────────────────────────────────────────────────────
    ANGLE_SMOOTHING:     float = 0.25   # IIR low-pass weight on steering angle (0=stiff, 1=passthrough)
    MAX_CURVATURE:       float = 0.09   # Clip polynomial curvature above this value
    MAX_STEERING_ANGLE:  float = 0.85   # Maximum absolute steering command (rad)

    # ── Sharp turn detection & boost ───────────────────────────────────────────
    SHARP_TURN_BOOST:     float = 2.4     # Multiply steering angle by this in sharp curves
    CURVATURE_THRESHOLD:  float = 0.005  # Curvature above this triggers sharp-turn logic

    # ── Look-ahead tuning (affects steering responsiveness) ───────────────────
    LOOKAHEAD_RATIO_NORMAL: float = 0.82   # Fraction of BEV image used as look-ahead on straight
    LOOKAHEAD_RATIO_SHARP:  float = 0.48  # Fraction used on sharp curves (closer = more reactive)

    # ── Debug visualization (disable all for headless deployment) ─────────────
    SHOW_BIRDSEYE:    bool = True
    SHOW_OVERLAY:     bool = False
    SHOW_WHITE_MASK:  bool = False
    SHOW_WARPED_MASK: bool = False
    SHOW_HISTOGRAM:   bool = False
    SHOW_SRC_OVERLAY: bool = False


# ═══════════════════════════════════════════════════════════════════════════════
#  POTHOLE DETECTION NODE PARAMETERS
# ═══════════════════════════════════════════════════════════════════════════════

class PotholeDetection:
    """Pothole detection algorithm tunable parameters."""

    # ── White blob area filtering (pixels²) ───────────────────────────────────
    # Adjust based on camera height and pothole size on your track.
    MIN_AREA: int = 80       # Minimum pothole blob size (lowered 300→80 for far detection)
    MAX_AREA: int = 15000     # Reject oversized blobs (lane lines, rover body reflections)

    # ── Shape descriptor thresholds ───────────────────────────────────────────
    # These filter out elongated lane-line fragments while accepting round potholes.
    #   Lane lines:     circularity ≈ 0.05  →  easily rejected by MIN_CIRCULARITY
    #   Potholes:       circularity ≈ 0.4+  →  easily accepted
    MIN_CIRCULARITY: float = 0.15   # Lane lines score ~0.05, easily rejected
    MAX_CIRCULARITY: float = 1.0
    MAX_ASPECT_RATIO: float = 4.0   # Lane lines are far wider than tall; potholes are not
    MIN_SOLIDITY: float = 0.40      # Lane-line fragments are jagged; potholes are compact

    # ── Detection range (metres) — split into two zones ───────────────────────
    MIN_DETECTION_M: float = 0.0    # Ignore very close detections (rover body noise)
    MAX_DETECTION_M: float = 7.0    # Detect potholes up to this distance (speed hint range)

    # ── Costmap publish gate (metres) ─────────────────────────────────────────
    # Potholes farther than this do NOT enter Nav2's costmap, preventing premature
    # MPPI swerving. The speed hint (/pothole_speed) is still published for ALL
    # detections up to MAX_DETECTION_M.
    #
    # At high rover speeds, raise this value so MPPI has enough lookahead to plan
    # an avoidance path. TUNING: raise if rover reaches pothole before dodging;
    # lower if rover swerves well before the pothole is a real threat.
    COSTMAP_PUBLISH_M: float = 5.0

    # ── Camera intrinsics (must match your camera hardware) ───────────────────
    CAM_HEIGHT_M:  float = 0.30    # Height of camera above ground (metres)
    CAM_HFOV_DEG:  float = 62.0   # Horizontal field of view (degrees)

    # ── Speed hint scaling ────────────────────────────────────────────────────
    # Speed factor = clip(closest_distance_m / MAX_DETECTION_M, MIN_FACTOR, MAX_FACTOR)
    # This gives the rover 2-3 seconds of planning time at typical speeds (2-4 m/s).
    SPEED_HINT_MIN_FACTOR: float = 0.15  # Minimum speed factor (pothole at rover's foot)
    SPEED_HINT_MAX_FACTOR: float = 1.0   # Maximum speed factor (pothole at MAX_DETECTION_M)


# ═══════════════════════════════════════════════════════════════════════════════
#  LANE PATH PUBLISHER (Generic Points) PARAMETERS
# ═══════════════════════════════════════════════════════════════════════════════

class LanePublisher:
    """Parameters for converting lane polynomials to a synthetic LaserScan."""

    # ── Sampling and visualization ─────────────────────────────────────────────
    N_LANE_SAMPLES: int = 100        # Points sampled per lane for point cloud
    N_SCAN_BINS:    int = 720        # Rays in synthetic LaserScan (0.5° resolution)

    # ── Publishing rates (Hz) ──────────────────────────────────────────────────
    SCAN_HZ:   float = 20.0          # Must match scan_merger's expected input rate
    MARKER_HZ: float = 5.0           # RViz visualization rate (lower for bandwidth)

    # ── RViz marker appearance ─────────────────────────────────────────────────
    MARKER_LINE_WIDTH_M: float = 0.08
    MARKER_Z_LIFT_M:     float = 0.05   # Slight elevation to prevent z-fighting with ground

    # ── Coordinate frames ──────────────────────────────────────────────────────
    ROBOT_FRAME: str = 'base_footprint'  # Frame for markers and point clouds
    LIDAR_FRAME: str = 'lidar_link'      # Frame for LaserScan (must match scan_merger config)


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

    # ── Curvature thresholds ───────────────────────────────────────────────────
    CURVE_THRESHOLD: float = 0.0014   # Gentle-curve regime starts here (pixels⁻¹)
    SHARP_THRESHOLD: float = 0.0060   # Sharp / 90° turn regime starts here (pixels⁻¹)

    # ── Look-ahead pixel row fraction of IMG_H ─────────────────────────────────
    # Smaller fraction → higher row in BEV image → further ahead on the ground.
    # PY_FAR  = 0.52 → x_fwd ≈ 4.4 m   (straight: stable, far)
    # PY_NEAR = 0.85 → x_fwd ≈ 1.8 m   (curves: goal stays ON the arc, not the chord)
    # PY_SHARP= 0.85 → x_fwd ≈ 1.4 m   (sharp: very close stepping stones)
    PY_FAR:   float = 0.52   # Look-ahead fraction for straight segments
    PY_NEAR:  float = 0.85   # Look-ahead fraction for gentle curves
    PY_SHARP: float = 0.85   # Look-ahead fraction for sharp turns

    # ── Sharp-turn goal projection ─────────────────────────────────────────────
    # Fixed forward distance for each stepping stone in sharp-turn mode.
    # Short enough to update rapidly through the turn; long enough for Nav2 to track.
    GOAL_DIST_SHARP_M: float = 0.50   # Metres

    # ── Outer bias (gentle curves only — fades to zero in sharp turns) ─────────
    # Pushes the goal toward the OUTSIDE of the curve to widen the arc.
    # Bias formula: outer_bias = OUTER_BIAS_MAX_M * blend_curve^BIAS_BLEND_POWER
    #                            * min(1, x_fwd / FWD_FACTOR_DIV) * (1 - blend_sharp)
    #
    # TUNING:
    #   Rover still cuts corners → increase OUTER_BIAS_MAX_M toward 1.0
    #   Rover clips the outer wall → decrease toward 0.3
    OUTER_BIAS_MAX_M:  float = 0.45   # Maximum lateral push (metres)
    BIAS_BLEND_POWER:  float = 1.30   # Exponent on blend factor; >1 delays bias onset
    FWD_FACTOR_DIV:    float = 1.50   # x_fwd divisor; saturates bias at short lookaheads

    # ── Lateral position IIR smoothers ────────────────────────────────────────
    # Lower alpha = faster response to new lane position.
    ALPHA_STRAIGHT: float = 0.70   # Heavy smoothing on straight segments
    ALPHA_CURVE:    float = 0.22   # Quicker reaction on gentle curves

    # ── Sharp-turn heading IIR smoother ───────────────────────────────────────
    # IIR keep-old weight on the lane-tangent yaw in sharp mode.
    #   0.40 → 60% weight on new heading (responsive, may overshoot apex)
    #   0.65 → 35% weight on new heading (stable, slight lag into curve)
    ALPHA_SHARP_YAW: float = 0.65

    # ── Goal update throttle (minimum displacement between consecutive goals) ──
    # Prevents flooding Nav2 with nearly identical goals.
    # Goal spacing = planning frequency. Every new NavigateToPose goal triggers
    # one full planning call regardless of the BT. Increasing spacing here
    # directly reduces how often the planner runs on each segment type.
    #
    # Was 1.40m — one plan every ~2s on straight. At 2.5m, one plan every ~5s.
    # Smoother planner output means you can afford longer spacing on straights.
    UPDATE_M_STRAIGHT: float = 2.50   # One planning call every ~5s at 0.5 m/s

    # Was 0.20m — very frequent on curves. At 0.40m still tracks curves well
    # while halving the number of planning calls per curve segment.
    UPDATE_M_CURVE:    float = 0.40   # Denser on curves but not excessive

    # Keep sharp turns dense — stepping-stone geometry requires it.
    UPDATE_M_SHARP:    float = 0.15   # Very dense during 90° turns (keep as-is)

    # ── Goal proximity tolerances ──────────────────────────────────────────────
    # A goal is declared REACHED when BOTH conditions are met.
    # This fires before Nav2's built-in checker, enabling early goal advance.
    # XY proximity only — no yaw check.
    # YAW_GOAL_TOL was removed: the publisher _check_goal_proximity() only
    # checks XY distance. A yaw check caused circles because the rover heading
    # at intermediate waypoints is always 20-40° off from the goal yaw.
    XY_GOAL_TOL: float = 0.65   # metres — only tolerance used

    # ── Lane-loss / rotation recovery ─────────────────────────────────────────
    LANE_SILENCE_THRESH_S:    float = 0.30           # Topic considered silent after this gap (s)
    LANE_LOSS_DETECT_S:       float = 1.50           # Good-detection gap before recovery (s)
    PREEMPT_MAX_CURVE_BLEND:  float = 0.15
    PREEMPT_MAX_ANGLE_RAD:    float = 0.1745  # ≈ 10 degrees in radians
    PREEMPT_MIN_GOAL_SHIFT_M: float = 0.80
    SHARP_EXIT_GOAL_COUNT:    int   = 6
    LANE_LOSS_TIMEOUT_S:      float = 18.0           # Abort recovery after this time (s)
    RECOVERY_YAW_TARGET_RAD:  float = 1.6580         # Phase 0 rotation target ≈ 95° (rad)
    RECOVERY_ROT_VEL:         float = 0.40           # In-place spin speed (rad/s)
    RECOVERY_POLL_S:          float = 0.05           # Spin thread yaw-check interval (s)

    # ── Watchdog timer rate ────────────────────────────────────────────────────
    WATCHDOG_HZ: float = 10.0   # How often _watchdog_cb fires (Hz)

    # ── RViz goal marker history ───────────────────────────────────────────────
    MAX_MARKERS: int = 25   # Rolling marker history kept in RViz


# ═══════════════════════════════════════════════════════════════════════════════
#  ROS TOPICS — CENTRAL TOPIC REGISTRY
# ═══════════════════════════════════════════════════════════════════════════════

class RosTopics:
    """
    Centralized ROS 2 topic names and TF frame IDs for all nodes.

    Every node subscribes and publishes using these constants — no hardcoded
    strings anywhere in the package. Rename a topic here and it propagates
    everywhere automatically.

    TOPIC GRAPH
    ───────────
    INPUT (Sensors):
      CAMERA_IMAGE    /camera/image          sensor_msgs/Image        ← camera driver
      IMU             /imu                   sensor_msgs/Imu          ← IMU driver
      JOINT_STATES    /joint_states          sensor_msgs/JointState   ← sim / HW bridge
      LIDAR_HW        /scan_hw               sensor_msgs/LaserScan    ← hardware LiDAR

    INTERNAL PIPELINE:
      LANE_STATUS     /lane_status           rover26/LaneStatus       lane → pothole & goals
      POTHOLE_OBSTACLES /pothole_obstacles   sensor_msgs/PointCloud2  → Nav2 costmap
      POTHOLE_SPEED   /pothole_speed         std_msgs/Float32         → speed controller
      LANE_SCAN       /lane_scan             sensor_msgs/LaserScan    → Nav2 (synthetic)
      LANE_MARKERS    /lane_markers          visualization_msgs/MarkerArray → RViz
      GOAL_POSE       /goal_pose             geometry_msgs/PoseStamped → Nav2
      GOAL_MARKERS    /goal_markers          visualization_msgs/MarkerArray → RViz
      LANE_GOALS_VIZ  /lane_goals_viz        visualization_msgs/MarkerArray → RViz
      ODOM            /odom                  nav_msgs/Odometry        → Nav2 / SLAM
      LIDAR_SCAN      /scan                  sensor_msgs/LaserScan    → Nav2 (re-timestamped)

    OUTPUT (Commands):
      CMD_VEL_STAMPED /cmd_vel_stamped       geometry_msgs/TwistStamped  ← twist_mux
      CMD_VEL         /cmd_vel               geometry_msgs/Twist         → motor driver

    TF TREE:
      map → odom → base_footprint → base_link → [sensor links]
      SLAM  odom_tf_broadcaster  robot_state_publisher
    """

    # ── Input (sensor) topics ──────────────────────────────────────────────────
    CAMERA_IMAGE: str = '/camera/image'
    IMU:          str = '/imu'
    JOINT_STATES: str = '/joint_states'
    LIDAR_HW:     str = '/scan_hw'          # Raw hardware LiDAR (wall-stamped)

    # ── Internal pipeline topics ───────────────────────────────────────────────
    LANE_STATUS:        str = '/lane_status'          # Published by: lane_detection_node
    POTHOLE_OBSTACLES:  str = '/pothole_obstacles'    # Published by: pothole_detection_node
    POTHOLE_SPEED:      str = '/pothole_speed'        # Published by: pothole_detection_node
    LANE_SCAN:          str = '/lane_scan'            # Published by: generic_points_publisher_node
    LANE_MARKERS:       str = '/lane_markers'         # Published by: generic_points_publisher_node
    GOAL_POSE:          str = '/goal_pose'            # Published by: lane_goal_publisher_node
    GOAL_MARKERS:       str = '/goal_markers'         # Published by: lane_goal_publisher_node
    LANE_GOALS_VIZ:     str = '/lane_goals_viz'      # Published by: lane_goal_publisher_node (RViz)
    ODOM:               str = '/odom'                # Published by: odom_tf_broadcaster
    LIDAR_SCAN:         str = '/scan'                # Published by: real_lidar_relay (sim-stamped)

    # ── Output (command) topics ────────────────────────────────────────────────
    CMD_VEL_STAMPED: str = '/cmd_vel_stamped'   # From twist_mux → twist_unstamper
    CMD_VEL:         str = '/cmd_vel'           # From twist_unstamper → motor driver
                                                # Also used by lane_goal_publisher for recovery spins

    # ── TF frame IDs ──────────────────────────────────────────────────────────
    ODOM_FRAME:  str = 'odom'
    MAP_FRAME:   str = 'map'
    BASE_FRAME:  str = 'base_footprint'
    LIDAR_FRAME: str = 'lidar_link'


# ═══════════════════════════════════════════════════════════════════════════════
#  ODOMETRY & TRANSFORM ESTIMATION
# ═══════════════════════════════════════════════════════════════════════════════

class Odometry:
    """
    Parameters for wheel-based odometry and odom → base_footprint TF broadcasting.

    All geometry values must match rover26_urdf.xacro exactly. If odometry
    drifts, check wheel radius first (most sensitive parameter).
    """

    # ── Physical geometry (from URDF inertia tensors) ─────────────────────────
    WHEEL_RADIUS_M:     float = 0.0825   # Metres — tune if wheel slips or odometry is scaled wrong
    WHEEL_SEPARATION_M: float = 0.6972  # Distance between left and right axles (2 × 0.3486 m)

    # ── Joint names (must match URDF <joint name="..."> exactly) ──────────────
    LEFT_JOINT:  str = 'leftwheel_joint'
    RIGHT_JOINT: str = 'rightwheel_joint'

    # ── Sign correction ────────────────────────────────────────────────────────
    # Flip to -1.0 if odometry moves backward when robot moves forward.
    LEFT_WHEEL_SIGN:  float = 1.0
    RIGHT_WHEEL_SIGN: float = 1.0

    # ── TF frame IDs (kept here for clarity; canonical source is RosTopics) ───
    ODOM_FRAME: str = 'odom'
    BASE_FRAME: str = 'base_footprint'

    # ── Covariance diagonal values (6×6 row-major, diagonal only) ─────────────
    # Increase if EKF / SLAM is over-trusting wheel odometry.
    POSE_COV_X:   float = 0.01   # x   variance (m²)
    POSE_COV_Y:   float = 0.01   # y   variance (m²)
    POSE_COV_YAW: float = 0.02   # yaw variance (rad²)
    TWIST_COV_VX: float = 0.005  # vx  variance ((m/s)²)
    TWIST_COV_WZ: float = 0.01   # wz  variance ((rad/s)²)


# ═══════════════════════════════════════════════════════════════════════════════
#  PHYSICAL & CALIBRATION CONSTANTS
# ═══════════════════════════════════════════════════════════════════════════════

class Physical:
    """
    Fixed physical constants derived from measurements, the URDF, or camera
    calibration. These change only when hardware changes, not when tuning behavior.
    """

    # ── Bird's-eye view pixel-to-metric mapping ────────────────────────────────
    PX_PER_METRE: float = 196.0           # Pixels per metre in warped image
    CTE_SCALE:    float = 1.0 / 196.0    # Metres per pixel (derived from PX_PER_METRE)

    # ── Camera mounting ────────────────────────────────────────────────────────
    CAMERA_HEIGHT_M: float = 0.45   # Height above ground (metres)

    # ── Ground patch visible in bird's-eye view ────────────────────────────────
    GROUND_WIDTH_M:       float = 9.0    # Left-right extent of warped view (metres)
    LANE_WIDTH_M:         float = 4.0    # Physical lane width (metres) — measure on your track
    GROUND_HEIGHT_M:      float = 10.0  # Forward extent of warped view (metres)
    GROUND_START_AHEAD_M: float = 0.5   # Distance to start of visible patch (metres)

    # ── Lateral bias trim ──────────────────────────────────────────────────────
    # HOW TO CALIBRATE
    # ────────────────
    # Problem: rover drives offset from lane centre even when lane_detection_node
    # reports the polynomial centre at the image midline. Caused by the bird's-eye
    # warp not being perfectly centred, or GROUND_WIDTH_M being slightly wrong.
    #
    # Steps:
    #   1. Set LATERAL_BIAS_M = 0.0 and rebuild.
    #   2. Drive the rover straight down a centred lane.
    #   3. Read "lateral=X.XXm" from lane_goal_publisher log output.
    #      e.g. "lateral=0.43m" means the formula thinks the rover is 0.43 m left of centre.
    #   4. Set LATERAL_BIAS_M to that value (e.g. 0.43) and rebuild.
    #   5. Log should now show "lateral≈0.00m" when centred.
    #
    # Note: if WARP_SRC_LEFT_TOP / WARP_SRC_RIGHT_TOP are retuned, recalibrate
    # LATERAL_BIAS_M too — they are tightly coupled.
    LATERAL_BIAS_M: float = 0.13   # Start at 0.0 — tune per steps above

    # ── Rover geometry ─────────────────────────────────────────────────────────
    WHEELBASE_M: float = 0.5   # Distance from rear to front axle (metres)

    # ── IMU ───────────────────────────────────────────────────────────────────
    IMU_UPDATE_HZ: float = 100.0   # Expected IMU publish rate (Hz)


# ═══════════════════════════════════════════════════════════════════════════════
#  DEBUGGING & MONITORING
# ═══════════════════════════════════════════════════════════════════════════════

class Debug:
    """Debug output and logging parameters. All safe to change without rebuild."""

    # Enable OpenCV display windows — MUST be False on headless systems
    ENABLE_DISPLAY: bool = True

    # Logging throttle intervals (seconds between repeated messages)
    THROTTLE_INFO: float = 0.5
    THROTTLE_WARN: float = 0.3

    # Per-node verbose modes (enable to see per-frame decision logs)
    LANE_DETECTION_VERBOSE:    bool = False
    POTHOLE_DETECTION_VERBOSE: bool = True
    ODOMETRY_VERBOSE:          bool = False


# ═══════════════════════════════════════════════════════════════════════════════
#  CONVENIENCE ALIASES FOR VISION UTILITIES
# ═══════════════════════════════════════════════════════════════════════════════
# Short-name aliases for commonly used constants in vision utilities.
# All values still live in their primary locations (IMG_WIDTH, Physical, etc.)
# but exported here with shorter names for math-heavy code readability.

IMG_W: int = IMG_WIDTH                           # Camera frame width (pixels)
IMG_H: int = IMG_HEIGHT                          # Camera frame height (pixels)
PX_PER_METRE: float = Physical.PX_PER_METRE      # Pixels per metre in bird's-eye view
CTE_SCALE: float = Physical.CTE_SCALE            # Metres per pixel (1 / PX_PER_METRE)
WHEELBASE: float = Physical.WHEELBASE_M          # Front-to-rear axle distance (m)
CAMERA_HEIGHT_M: float = Physical.CAMERA_HEIGHT_M  # Camera height above ground (m)
GROUND_WIDTH_M: float = Physical.GROUND_WIDTH_M     # Left-right extent of warp (m)
GROUND_HEIGHT_M: float = Physical.GROUND_HEIGHT_M   # Forward extent of warp (m)
LANE_Y_START: float = Physical.GROUND_START_AHEAD_M  # Distance to start of visible patch (m)


# ═══════════════════════════════════════════════════════════════════════════════
#  PUBLIC API
# ═══════════════════════════════════════════════════════════════════════════════

__all__ = [
    # Top-level constants
    'IMG_WIDTH', 'IMG_HEIGHT',
    'WARP_SRC_LEFT_TOP', 'WARP_SRC_RIGHT_TOP', 'WARP_SRC_TOP_Y', 'WARP_MARGIN',
    'LANE_WIDTH_PX',
    # Convenience aliases for vision utilities
    'IMG_W', 'IMG_H',
    'PX_PER_METRE', 'CTE_SCALE', 'WHEELBASE', 'CAMERA_HEIGHT_M',
    'GROUND_WIDTH_M', 'GROUND_HEIGHT_M', 'LANE_Y_START',
    # Configuration classes
    'LaneDetection',
    'PotholeDetection',
    'LanePublisher',
    'LaneGoalPublisher',
    'RosTopics',
    'Odometry',
    'Physical',
    'Debug',
]