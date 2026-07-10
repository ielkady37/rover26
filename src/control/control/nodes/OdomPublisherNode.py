#!/usr/bin/env python3
"""
OdomPublisherNode — builds /odom from wheel encoder speeds only.

Source:
        • /encoders_speeds (from hoverboard_node) → wheel speeds

Pose integration uses a differential-drive kinematic model:
        v = (v_left + v_right) / 2
        w = (v_right - v_left) / wheel_base

CALIBRATION: _ENC_SPEED_SCALE converts whatever units /encoders_speeds
reports into wheel surface m/s. This depends on hoverboard firmware units
(raw ticks/sec, RPM, etc) and must be calibrated against real hardware:
drive a known distance, compare to what /odom reports, and adjust the
scale until they match.

ROS2 parameters
───────────────
    wheel_radius  (float, 0.075)   metres
    wheel_base    (float, 0.32)    metres
    odom_frame    (str,   'odom')
    base_frame    (str,   'base_link')
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from interfaces.msg import EncoderSpeeds

def _yaw_to_quat(yaw: float) -> Quaternion:
    """Convert a 2D yaw angle (radians) to a geometry_msgs/Quaternion."""
    half = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def _wrap_angle(angle: float) -> float:
    """Wrap *angle* (radians) to [-π, π]."""
    return math.atan2(math.sin(angle), math.cos(angle))


# Calibrate against real hardware: drive a known distance and adjust.
_ENC_SPEED_SCALE: float = 0.018


class OdomPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("odom_publisher_node")

        self.declare_parameter("wheel_radius", 0.075)
        self.declare_parameter("wheel_base", 0.32)
        self.declare_parameter("odom_frame",   "odom")
        self.declare_parameter("base_frame",   "base_link")

        self._x:     float = 0.0
        self._y:     float = 0.0
        self._theta: float = 0.0
        self._prev_time_sec: float | None = None

        self._odom_pub = self.create_publisher(Odometry, "/odom_wheels", 10)
        self.create_subscription(EncoderSpeeds, "/encoders_speeds", self._enc_speeds_cb, 10)

        self.get_logger().info("OdomPublisherNode ready — /encoders_speeds → /odom")
    # /encoders_speeds → integrate pose and publish /odom

    def _enc_speeds_cb(self, msg: EncoderSpeeds) -> None:
        wheel_radius: float = self.get_parameter("wheel_radius").value
        wheel_base:   float = self.get_parameter("wheel_base").value
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

                v = (v_left + v_right) * 0.5
                if wheel_base > 0.0:
                    w = (v_right - v_left) / wheel_base
                else:
                    w = 0.0

                # Midpoint integration is more stable during turns.
                mid_theta = self._theta + 0.5 * w * dt
                self._x += v * dt * math.cos(mid_theta)
                self._y += v * dt * math.sin(mid_theta)
                self._theta = _wrap_angle(self._theta + w * dt)

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

def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()