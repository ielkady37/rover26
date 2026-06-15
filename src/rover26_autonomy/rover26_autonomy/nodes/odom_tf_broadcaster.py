#!/usr/bin/env python3
"""
═════════════════════════════════════════════════════════════════════════════════
odom_tf_broadcaster.py  —  rover26_autonomy
═════════════════════════════════════════════════════════════════════════════════

FUNCTIONALITY
─────────────
Single node with two tightly coupled responsibilities:

  JOB 1 — Compute /odom from wheel encoder joint states + IMU heading
  ────────────────────────────────────────────────────────────────────
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
  • /joint_states  [sensor_msgs/JointState]  ← hardware wheel encoders (BEST_EFFORT)
  • /imu           [sensor_msgs/Imu]         ← hardware IMU driver    (BEST_EFFORT)

Published:
  • /odom          [nav_msgs/Odometry]       → Nav2, SLAM toolbox     (RELIABLE)

TF Published:
  • odom → base_footprint  (dynamic, from live wheel odometry)

PARAMETERS (from config_params.py)
──────────────────────────────────
Odometry.*    Wheel radius, separation, joint names, sign correction, frame IDs
RosTopics.*   Topic names and TF frame IDs

PREREQUISITES
─────────────
control/run.launch.py must be running with:
  • Hardware encoder driver publishing /joint_states
  • Hardware IMU driver publishing /imu
  • robot_state_publisher providing URDF static TFs (base_link → lidar_link etc.)

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
    Computes wheel odometry from hardware encoders and broadcasts the
    odom → base_footprint TF.

    Attributes:
        _r  (float): Wheel radius in metres.
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
        self.declare_parameter('wheel_radius',      OdomConfig.WHEEL_RADIUS_M)
        self.declare_parameter('wheel_separation',  OdomConfig.WHEEL_SEPARATION_M)
        self.declare_parameter('left_joint',        OdomConfig.LEFT_JOINT)
        self.declare_parameter('right_joint',       OdomConfig.RIGHT_JOINT)
        self.declare_parameter('left_wheel_sign',   OdomConfig.LEFT_WHEEL_SIGN)
        self.declare_parameter('right_wheel_sign',  OdomConfig.RIGHT_WHEEL_SIGN)
        self.declare_parameter('odom_frame',        RosTopics.ODOM_FRAME)
        self.declare_parameter('base_frame',        RosTopics.BASE_FRAME)
        self.declare_parameter('odom_topic',        RosTopics.ODOM)

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
        self._x    = 0.0
        self._y    = 0.0
        self._yaw  = None       # Initialised from first IMU message

        self._last_stamp = None
        self._imu_yaw    = None

        # ── TF broadcaster ─────────────────────────────────────────────────────
        self._tf_broadcaster = TransformBroadcaster(self)

        # ── QoS — match hardware driver publisher policy ────────────────────────
        # Hardware drivers typically publish BEST_EFFORT (mirrors the old
        # Gazebo bridge policy). A RELIABLE subscriber silently drops every
        # message from a BEST_EFFORT publisher in ROS 2 — use BEST_EFFORT.
        hw_qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            history     = HistoryPolicy.KEEP_LAST,
            durability  = DurabilityPolicy.VOLATILE,
            depth       = 10,
        )

        # ── Subscribers ────────────────────────────────────────────────────────
        self._js_sub = self.create_subscription(
            JointState,
            RosTopics.JOINT_STATES,
            self._js_callback,
            hw_qos,
        )

        self._imu_sub = self.create_subscription(
            Imu,
            RosTopics.IMU,
            self._imu_callback,
            hw_qos,
        )

        # ── Publisher ──────────────────────────────────────────────────────────
        # RELIABLE so SLAM toolbox and Nav2 never miss an odometry message.
        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)

        self.get_logger().info(
            f'odom_tf_broadcaster ready:\n'
            f'  wheel radius    : {self._r} m\n'
            f'  wheel separation: {self._L} m\n'
            f'  joints          : left="{self._lj}" ({self._ls:+.0f}×)  '
            f'right="{self._rj}" ({self._rs:+.0f}×)\n'
            f'  TF              : "{self._odom_frame}" → "{self._base_frame}"\n'
            f'  odom topic      : {odom_topic}'
        )
        self.get_logger().info(
            'odom_tf_broadcaster: waiting for first /imu message to set initial heading…'
        )

    # =========================================================================
    #  IMU CALLBACK
    # =========================================================================

    def _imu_callback(self, msg: Imu) -> None:
        """
        Runs on every /imu message from the hardware IMU driver.

        Extracts yaw from the IMU quaternion and stores it so the
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
        Runs on every /joint_states message from the hardware encoder driver.

        Steps:
          1. Extract left/right wheel angular velocities by joint name.
          2. Compute robot linear and angular velocity (differential drive).
          3. Compute dt since the last message using message timestamps.
          4. Update heading — IMU (primary) or wheel integration (fallback).
          5. Integrate x, y position via dead reckoning.
          6. Publish /odom with full pose, twist, and covariance.
          7. Publish TF: odom → base_footprint.
        """

        # ── 1. Extract wheel velocities ───────────────────────────────────────
        try:
            li = msg.name.index(self._lj)
            ri = msg.name.index(self._rj)
        except ValueError:
            return  # Joint not yet present — normal at startup

        if not msg.velocity or len(msg.velocity) <= max(li, ri):
            return  # Velocity array not yet populated

        w_l = self._ls * msg.velocity[li]   # Left  wheel angular velocity (rad/s)
        w_r = self._rs * msg.velocity[ri]   # Right wheel angular velocity (rad/s)

        v_l = w_l * self._r
        v_r = w_r * self._r

        v     = (v_r + v_l) / 2.0       # Robot forward velocity (m/s)
        omega = (v_r - v_l) / self._L   # Robot angular velocity from wheels (rad/s)

        # ── 2. Compute dt ─────────────────────────────────────────────────────
        # Use the hardware timestamp from the message — NOT the wall clock.
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)

        if self._last_stamp is None:
            self._last_stamp = stamp
            return

        dt = (stamp - self._last_stamp).nanoseconds * 1e-9
        self._last_stamp = stamp

        if dt <= 0.0 or dt > 1.0:
            # Skip stale or future-stamped messages
            return

        # ── 3. Heading — IMU primary, wheel integration fallback ───────────────
        if self._imu_yaw is not None:
            self._yaw = self._imu_yaw
        elif self._yaw is not None:
            self._yaw += omega * dt
        else:
            # No IMU yet — cannot integrate reliably; wait
            return

        # ── 4. Position integration ───────────────────────────────────────────
        self._x += v * math.cos(self._yaw) * dt
        self._y += v * math.sin(self._yaw) * dt

        # ── 5. Build and publish /odom ────────────────────────────────────────
        orientation = yaw_to_quat(self._yaw)
        ros_stamp   = stamp.to_msg()

        odom = Odometry()
        odom.header.stamp    = ros_stamp
        odom.header.frame_id = self._odom_frame   # 'odom'
        odom.child_frame_id  = self._base_frame   # 'base_footprint'

        odom.pose.pose.position.x  = self._x
        odom.pose.pose.position.y  = self._y
        odom.pose.pose.position.z  = 0.0
        odom.pose.pose.orientation = orientation

        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = omega

        # Diagonal covariance — tune these to your hardware accuracy
        odom.pose.covariance[0]  = 0.01   # x
        odom.pose.covariance[7]  = 0.01   # y
        odom.pose.covariance[35] = 0.01   # yaw

        odom.twist.covariance[0]  = 0.01
        odom.twist.covariance[35] = 0.01

        self._odom_pub.publish(odom)

        # ── 6. Publish TF: odom → base_footprint ──────────────────────────────
        t = TransformStamped()
        t.header.stamp    = ros_stamp
        t.header.frame_id = self._odom_frame   # Parent: 'odom'
        t.child_frame_id  = self._base_frame   # Child:  'base_footprint'

        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = 0.0
        t.transform.rotation      = orientation

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
        node.get_logger().info('odom_tf_broadcaster shutting down…')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()