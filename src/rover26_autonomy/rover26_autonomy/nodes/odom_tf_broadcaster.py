#!/usr/bin/env python3
"""
═════════════════════════════════════════════════════════════════════════════════
odom_tf_broadcaster.py  —  rover26_autonomy
═════════════════════════════════════════════════════════════════════════════════

FUNCTIONALITY
─────────────
Single node with two tightly coupled responsibilities:

  JOB 1 — Compute /odom from wheel joint states + IMU heading
  ────────────────────────────────────────────────────────────
  Subscribes /joint_states → extracts left/right wheel angular velocities
  Subscribes /imu          → extracts yaw (absolute heading)
  Publishes  /odom          → nav_msgs/Odometry (this node is the SOLE publisher)

  WHY IMU FOR HEADING (not wheel integration)?
  Integrating (v_right − v_left) / L drifts without bound on any real surface.
  The IMU gives absolute orientation with no cumulative error — heading is always
  correct regardless of how long the rover has been running.

  JOB 2 — Publish the odom → base_footprint TF
  ─────────────────────────────────────────────
  Reads the /odom just computed and immediately publishes the dynamic TF edge:
      odom  →  base_footprint
  This is the edge robot_state_publisher cannot produce on its own because it
  depends on live odometry, not the static URDF.

TF CHAIN COMPLETED BY THIS NODE
────────────────────────────────
  map → odom → base_footprint → base_link → [sensor links]
  ↑SLAM  ↑THIS NODE             ↑robot_state_publisher (URDF)

TOPICS & SUBSCRIPTIONS
──────────────────────
Subscribed:
  • /joint_states  [sensor_msgs/JointState]  ← Gazebo DiffDrive bridge (BEST_EFFORT)
  • /imu           [sensor_msgs/Imu]         ← IMU driver / Gazebo IMU (BEST_EFFORT)

Published:
  • /odom          [nav_msgs/Odometry]       → Nav2, slam_toolbox, EKF (RELIABLE)

TF Published:
  • odom → base_footprint  (dynamic, from live wheel odometry)

PARAMETERS (from config_params.py)
──────────────────────────────────
Odometry.*    Wheel radius, separation, joint names, sign correction, frame IDs,
              covariance diagonal values
RosTopics.*   Topic names and TF frame IDs

ONE-TIME PREREQUISITE CHANGES (outside this file)
─────────────────────────────────────────────────
1. gazebo.xacro  →  set <publish_odom>false</publish_odom>
                     and <publish_odom_tf>false</publish_odom_tf>
   (Gazebo's DiffDrive was also publishing to /odom and to the TF, causing
    "odom moves wrong way" and a TF race condition.)

2. gazebo_launch.py bridge  →  uncomment the joint_states bridge entry and its
   remapping so /joint_states reaches ROS.

LAUNCH FILE USAGE
─────────────────
    Node(
        package    = 'rover26_autonomy',
        executable = 'odom_tf_broadcaster',
        name       = 'odom_tf_broadcaster',
        parameters = [{'use_sim_time': True}],
        output     = 'screen',
    )

═════════════════════════════════════════════════════════════════════════════════
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

from rover26_autonomy.config_params import Odometry as OdomConfig, RosTopics


# ═════════════════════════════════════════════════════════════════════════════
#  QUATERNION HELPERS
# ═════════════════════════════════════════════════════════════════════════════

def quat_to_yaw(q) -> float:
    """
    Extract yaw (rotation about world Z) from a geometry_msgs/Quaternion.

    Formula: yaw = atan2(2·(w·z + x·y),  1 − 2·(y² + z²))
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw: float) -> Quaternion:
    """
    Build a geometry_msgs/Quaternion from a yaw angle only.

    Roll and pitch are assumed to be zero — this is a 2-D ground robot that
    operates on flat terrain.
    """
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


# ═════════════════════════════════════════════════════════════════════════════
#  ODOM TF BROADCASTER NODE
# ═════════════════════════════════════════════════════════════════════════════

class OdomTfBroadcaster(Node):
    """
    Computes wheel odometry and broadcasts the odom → base_footprint TF.

    Attributes:
        _r  (float): Wheel radius in metres (from Odometry config or ROS param).
        _L  (float): Wheel separation in metres.
        _lj (str):   Left joint name.
        _rj (str):   Right joint name.
        _ls (float): Left wheel sign correction (+1 or -1).
        _rs (float): Right wheel sign correction (+1 or -1).
        _odom_frame (str): Parent TF frame ('odom').
        _base_frame (str): Child TF frame ('base_footprint').
        _x   (float): Accumulated X position in odom frame (metres).
        _y   (float): Accumulated Y position in odom frame (metres).
        _yaw (float | None): Current heading (radians); None until first IMU msg.
        _imu_yaw (float | None): Latest yaw from IMU; None until first IMU msg.
        _last_stamp: rclpy.time.Time of the previous joint_states tick.
    """

    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        # ── ROS Parameters (defaults from OdomConfig) ──────────────────────────
        # These can be overridden from the launch file without recompiling.
        self.declare_parameter('wheel_radius',      OdomConfig.WHEEL_RADIUS_M)
        self.declare_parameter('wheel_separation',  OdomConfig.WHEEL_SEPARATION_M)
        self.declare_parameter('left_joint',        OdomConfig.LEFT_JOINT)
        self.declare_parameter('right_joint',       OdomConfig.RIGHT_JOINT)
        self.declare_parameter('left_wheel_sign',   OdomConfig.LEFT_WHEEL_SIGN)
        self.declare_parameter('right_wheel_sign',  OdomConfig.RIGHT_WHEEL_SIGN)
        self.declare_parameter('odom_frame',        RosTopics.ODOM_FRAME)
        self.declare_parameter('base_frame',        RosTopics.BASE_FRAME)
        self.declare_parameter('odom_topic',        RosTopics.ODOM)

        # Read parameters into instance variables
        self._r          = self.get_parameter('wheel_radius').value
        self._L          = self.get_parameter('wheel_separation').value
        self._lj         = self.get_parameter('left_joint').value
        self._rj         = self.get_parameter('right_joint').value
        self._ls         = self.get_parameter('left_wheel_sign').value
        self._rs         = self.get_parameter('right_wheel_sign').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        odom_topic       = self.get_parameter('odom_topic').value

        # ── Odometry state ─────────────────────────────────────────────────────
        self._x    = 0.0    # Accumulated X position in odom frame (metres)
        self._y    = 0.0    # Accumulated Y position in odom frame (metres)
        self._yaw  = None   # Heading (radians); initialised from first IMU message

        self._last_stamp = None   # rclpy.time.Time of previous joint_states tick
        self._imu_yaw    = None   # Latest yaw extracted from the IMU

        # ── TF broadcaster ─────────────────────────────────────────────────────
        # Publishes the dynamic odom → base_footprint edge to /tf.
        self._tf_broadcaster = TransformBroadcaster(self)

        # ── QoS — must match the Gazebo bridge publisher policy ────────────────
        # Gazebo bridge uses BEST_EFFORT.  A RELIABLE subscriber silently drops
        # every message from a BEST_EFFORT publisher in ROS 2 — use BEST_EFFORT.
        gz_qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            history     = HistoryPolicy.KEEP_LAST,
            durability  = DurabilityPolicy.VOLATILE,
            depth       = 10,
        )

        # ── Subscribers ────────────────────────────────────────────────────────
        # /joint_states — wheel angular velocities from Gazebo via bridge.
        # IMPORTANT: the joint_states bridge entry in gazebo_launch.py must be
        # uncommented before this subscriber receives any data.
        self._js_sub = self.create_subscription(
            JointState,
            RosTopics.JOINT_STATES,
            self._js_callback,
            gz_qos,
        )

        # /imu — absolute orientation; we extract yaw for drift-free heading.
        self._imu_sub = self.create_subscription(
            Imu,
            RosTopics.IMU,
            self._imu_callback,
            gz_qos,
        )

        # ── Publisher ──────────────────────────────────────────────────────────
        # /odom — RELIABLE so downstream nodes (Nav2, EKF) never miss a message.
        # This node is the SOLE publisher. Gazebo DiffDrive must have
        # <publish_odom>false</publish_odom> in gazebo.xacro.
        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)

        self.get_logger().info(
            f'odom_tf_broadcaster ready:\n'
            f'  wheel radius    : {self._r} m\n'
            f'  wheel separation: {self._L} m\n'
            f'  joints          : left="{self._lj}" ({self._ls:+.0f}×)  '
            f'right="{self._rj}" ({self._rs:+.0f}×)\n'
            f'  TF              : "{self._odom_frame}" → "{self._base_frame}"\n'
            f'  topic           : {odom_topic}'
        )
        self.get_logger().info(
            'odom_tf_broadcaster: waiting for first /imu message to set initial heading…'
        )

    # =========================================================================
    #  IMU CALLBACK
    # =========================================================================

    def _imu_callback(self, msg: Imu) -> None:
        """
        Runs on every /imu message (~100 Hz from Gazebo).

        Extracts yaw from the IMU quaternion and stores it so that the
        joint_states callback can read _imu_yaw on every wheel tick.
        """
        self._imu_yaw = quat_to_yaw(msg.orientation)

        # Initialise heading from the very first IMU message
        if self._yaw is None:
            self._yaw = self._imu_yaw
            self.get_logger().info(
                f'odom_tf_broadcaster: initial heading from IMU: '
                f'{math.degrees(self._yaw):.1f}°'
            )

    # =========================================================================
    #  JOINT STATES CALLBACK — MAIN ODOMETRY + TF LOOP
    # =========================================================================

    def _js_callback(self, msg: JointState) -> None:
        """
        Runs on every /joint_states message (50 Hz from Gazebo).

        Steps:
          1. Extract left/right wheel angular velocities by joint name.
          2. Compute robot linear and angular velocity (differential drive).
          3. Compute dt since the last message using simulation timestamps.
          4. Update heading — IMU (primary) or wheel integration (fallback).
          5. Integrate x, y position via dead reckoning.
          6. Publish /odom with full pose, twist, and covariance.
          7. Publish TF: odom → base_footprint.
        """

        # ── 1. Extract wheel velocities ───────────────────────────────────────
        try:
            li = msg.name.index(self._lj)   # Index of left  joint in this message
            ri = msg.name.index(self._rj)   # Index of right joint in this message
        except ValueError:
            # Joint not yet present — normal during startup, just wait
            return

        # Guard: velocity array may be empty during the first few ticks
        if not msg.velocity or len(msg.velocity) <= max(li, ri):
            return

        # Sign correction so positive velocity = forward motion for both wheels
        w_l = self._ls * msg.velocity[li]   # Left  wheel angular velocity (rad/s)
        w_r = self._rs * msg.velocity[ri]   # Right wheel angular velocity (rad/s)

        # Convert angular velocity → contact-point linear velocity
        v_l = w_l * self._r    # Left  wheel surface speed (m/s)
        v_r = w_r * self._r    # Right wheel surface speed (m/s)

        # Differential drive kinematics
        v     = (v_r + v_l) / 2.0      # Robot forward velocity (m/s)
        omega = (v_r - v_l) / self._L   # Robot angular velocity from wheels (rad/s)

        # ── 2. Compute dt ─────────────────────────────────────────────────────
        # Use the simulation timestamp from the message — NOT the wall clock.
        # With use_sim_time=True the ROS clock follows Gazebo, so msg.header.stamp
        # is the correct reference for TF timestamps and integration.
        stamp = msg.header.stamp
        now   = rclpy.time.Time.from_msg(stamp)

        if self._last_stamp is None:
            # First message — record time, skip integration (no previous stamp)
            self._last_stamp = now
            return

        dt = (now - self._last_stamp).nanoseconds * 1e-9   # nanoseconds → seconds
        self._last_stamp = now

        # Skip invalid dt (Gazebo clock not yet started, or a large sim-time jump)
        if dt <= 0.0 or dt > 1.0:
            return

        # ── 3. Update heading ─────────────────────────────────────────────────
        if self._imu_yaw is not None:
            # PRIMARY: direct IMU reading — no integration, no drift
            self._yaw = self._imu_yaw
        elif self._yaw is not None:
            # FALLBACK: integrate wheel angular velocity if IMU not yet available.
            # This drifts over time — only used until the first IMU message arrives.
            self._yaw += omega * dt
            self.get_logger().warn(
                'odom_tf_broadcaster: no IMU data — using wheel-integrated heading (drift expected)',
                throttle_duration_sec=5.0,
            )
        else:
            # Neither source available — cannot compute heading, skip this tick
            return

        # ── 4. Integrate position ─────────────────────────────────────────────
        # Dead reckoning: project current forward speed along current heading
        self._x += v * math.cos(self._yaw) * dt
        self._y += v * math.sin(self._yaw) * dt

        # ── 5. Build orientation quaternion from yaw ───────────────────────────
        orientation = yaw_to_quat(self._yaw)

        # ── 6. Publish /odom ──────────────────────────────────────────────────
        odom = Odometry()

        # Use simulation timestamp — keeps tf2 timestamps consistent across nodes
        odom.header.stamp    = stamp
        odom.header.frame_id = self._odom_frame   # Parent frame: 'odom'
        odom.child_frame_id  = self._base_frame   # Child frame:  'base_footprint'

        # Pose — accumulated position + current heading
        odom.pose.pose.position.x  = self._x
        odom.pose.pose.position.y  = self._y
        odom.pose.pose.position.z  = 0.0           # Ground robot — always flat
        odom.pose.pose.orientation = orientation

        # Twist — velocities in the robot's LOCAL (child) frame (ROS REP-105)
        odom.twist.twist.linear.x  = v        # Forward speed (m/s)
        odom.twist.twist.linear.y  = 0.0      # No sideways slip assumed
        odom.twist.twist.angular.z = omega    # Instantaneous yaw rate (rad/s)

        # Covariance — 6×6 row-major flattened to 36 elements (diagonal only)
        # Increase these if EKF / SLAM is over-trusting wheel odometry.
        odom.pose.covariance[0]  = OdomConfig.POSE_COV_X    # x   variance (m²)
        odom.pose.covariance[7]  = OdomConfig.POSE_COV_Y    # y   variance (m²)
        odom.pose.covariance[35] = OdomConfig.POSE_COV_YAW  # yaw variance (rad²)

        odom.twist.covariance[0]  = OdomConfig.TWIST_COV_VX  # vx variance ((m/s)²)
        odom.twist.covariance[35] = OdomConfig.TWIST_COV_WZ  # wz variance ((rad/s)²)

        self._odom_pub.publish(odom)

        # ── 7. Publish TF: odom → base_footprint ─────────────────────────────
        # Same data as the odometry message, repackaged as a TransformStamped.
        # All tf2 listeners (RViz, SLAM, Nav2, scan_merger) see this immediately.
        t = TransformStamped()

        # Use the same simulation timestamp — avoids TF extrapolation errors
        t.header.stamp    = stamp
        t.header.frame_id = self._odom_frame   # Parent: 'odom'
        t.child_frame_id  = self._base_frame   # Child:  'base_footprint'

        # Translation — same as odometry pose
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = 0.0

        # Rotation — same quaternion
        t.transform.rotation = orientation

        self._tf_broadcaster.sendTransform(t)


# ═════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('odom_tf_broadcaster shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()