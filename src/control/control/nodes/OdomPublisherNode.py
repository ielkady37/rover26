#!/usr/bin/env python3
"""
OdomPublisherNode — builds /odom and the odom→base_link TF from two sources:

    • /imu               (from ESPBridgeNode)   → heading (yaw)
    • /encoders_speeds    (from hoverboard_node) → real drive wheel speed

Heading comes from the IMU rather than integrated wheel encoder deltas to
avoid accumulated drift during turns. Distance comes from the hoverboard's
actual wheel encoder feedback, since that is the real drivetrain — the
sensor ESP32's own encoders are on a separate board not attached to the
driving wheels.

CALIBRATION: _ENC_SPEED_SCALE converts whatever units /encoders_speeds
reports into wheel surface m/s. This depends on hoverboard firmware units
(raw ticks/sec, RPM, etc) and must be calibrated against real hardware:
drive a known distance, compare to what /odom reports, and adjust the
scale until they match.

ROS2 parameters
───────────────
  wheel_radius  (float, 0.05)    metres
  odom_frame    (str,   'odom')
  base_frame    (str,   'base_link')
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
from interfaces.msg import EncoderSpeeds


def _yaw_to_quat(yaw: float) -> Quaternion:
    """Convert a 2D yaw angle (radians) to a geometry_msgs/Quaternion."""
    half = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def _quat_to_yaw(q: Quaternion) -> float:
    """Extract yaw (radians) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_angle(angle: float) -> float:
    """Wrap *angle* (radians) to [-π, π]."""
    return math.atan2(math.sin(angle), math.cos(angle))


# Calibrate against real hardware: drive a known distance and adjust.
_ENC_SPEED_SCALE: float = 1.0 / 1000.0


class OdomPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("odom_publisher_node")

        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("odom_frame",   "odom")
        self.declare_parameter("base_frame",   "base_link")

        self._x:     float = 0.0
        self._y:     float = 0.0
        self._theta: float = 0.0
        self._imu_theta: float | None = None
        self._prev_time_sec: float | None = None

        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Imu, "/imu", self._imu_cb, 10)
        self.create_subscription(EncoderSpeeds, "/encoders_speeds", self._enc_speeds_cb, 10)

        self.get_logger().info("OdomPublisherNode ready — /imu + /encoders_speeds → /odom")

    # /imu → cache heading only

    def _imu_cb(self, msg: Imu) -> None:
        self._imu_theta = _quat_to_yaw(msg.orientation)

    # /encoders_speeds → integrate distance and publish /odom + TF

    def _enc_speeds_cb(self, msg: EncoderSpeeds) -> None:
        if self._imu_theta is None:
            # No heading yet — wait for the first /imu message before
            # publishing anything, so the first pose isn't built with a
            # garbage heading of 0.
            return

        wheel_radius: float = self.get_parameter("wheel_radius").value
        odom_frame:   str   = self.get_parameter("odom_frame").value
        base_frame:   str   = self.get_parameter("base_frame").value

        stamp = self.get_clock().now().to_msg()
        now_sec = stamp.sec + stamp.nanosec * 1e-9

        if self._prev_time_sec is None:
            self._prev_time_sec = now_sec
        else:
            dt = now_sec - self._prev_time_sec
            self._prev_time_sec = now_sec

            # Guard against startup glitches / clock jumps
            if 0.0 < dt < 1.0:
                speed_r = float(msg.motor1_speed)
                speed_l = float(msg.motor2_speed)

                v_left  = speed_l * 2.0 * math.pi * wheel_radius * _ENC_SPEED_SCALE
                v_right = speed_r * 2.0 * math.pi * wheel_radius * _ENC_SPEED_SCALE

                delta_dist = ((v_left + v_right) * 0.5) * dt

                # Integrate position using IMU heading at the midpoint of
                # the step, to reduce error during fast turns.
                mid_theta = _wrap_angle(
                    self._theta + _wrap_angle(self._imu_theta - self._theta) * 0.5
                )
                self._x += delta_dist * math.cos(mid_theta)
                self._y += delta_dist * math.sin(mid_theta)

        self._theta = self._imu_theta

        odom_quat = _yaw_to_quat(self._theta)

        odom_msg = Odometry()
        odom_msg.header.stamp    = stamp
        odom_msg.header.frame_id = odom_frame
        odom_msg.child_frame_id  = base_frame

        odom_msg.pose.pose.position.x  = self._x
        odom_msg.pose.pose.position.y  = self._y
        odom_msg.pose.pose.position.z  = 0.0
        odom_msg.pose.pose.orientation = odom_quat

        self._odom_pub.publish(odom_msg)

        tf = TransformStamped()
        tf.header.stamp            = stamp
        tf.header.frame_id         = odom_frame
        tf.child_frame_id          = base_frame
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.translation.z = 0.0
        tf.transform.rotation      = odom_quat
        self._tf_broadcaster.sendTransform(tf)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()