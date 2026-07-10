#!/usr/bin/env python3
"""
WheelJointStatePublisherNode — converts wheel encoder speeds into
sensor_msgs/JointState so RViz (via robot_state_publisher) actually
animates the driven wheels spinning.

Source:
    /encoders_speeds (interfaces/EncoderSpeeds) — motor1_speed / motor2_speed,
    published by esp_bridge_node / hoverboard driver.

Since /encoders_speeds reports SPEED (not absolute position), this node
integrates speed * dt over time to build up a running wheel angle, the
same pattern odom_publisher_node uses for x/y/theta.

CALIBRATION: _ENC_SPEED_SCALE mirrors odom_publisher_node's constant —
converts whatever units /encoders_speeds reports into wheel angular
velocity (rad/s) here. If your wheel visually spins at the wrong rate
in RViz, this is the value to adjust. Keep it consistent with whatever
odom_publisher_node uses so the two stay physically coherent.

ASSUMPTION (verify against esp_bridge_node / firmware mapping):
    motor1_speed -> rightwheel_joint
    motor2_speed -> leftwheel_joint
    This mirrors odom_publisher_node's existing motor1=right, motor2=left
    convention. If wheels spin the wrong direction in RViz relative to
    the real rover, swap RIGHT_/LEFT_ below.

Publishes:
    /wheel_joint_states (sensor_msgs/JointState) — meant to be merged by
    joint_state_publisher via its `source_list` parameter, NOT published
    directly to /joint_states (avoids two nodes fighting over one topic).

ROS2 parameters
───────────────
    right_wheel_joint  (str, 'rightwheel_joint')
    left_wheel_joint   (str, 'leftwheel_joint')
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from interfaces.msg import EncoderSpeeds
from control.services.value_safety import sanitize_float

# Must match odom_publisher_node's _ENC_SPEED_SCALE for the two to stay
# physically consistent — this converts raw /encoders_speeds units into
# wheel angular velocity (rad/s) here (rather than linear m/s there).
_ENC_SPEED_SCALE: float = 0.018
_WHEEL_RADIUS_M:  float = 0.075  # must match odom_publisher_node's wheel_radius


class WheelJointStatePublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("wheel_joint_state_publisher_node")

        self.declare_parameter("right_wheel_joint", "rightwheel_joint")
        self.declare_parameter("left_wheel_joint",  "leftwheel_joint")

        self._right_joint: str = self.get_parameter("right_wheel_joint").value
        self._left_joint:  str = self.get_parameter("left_wheel_joint").value

        self._right_angle: float = 0.0
        self._left_angle:  float = 0.0
        self._prev_time_sec: float | None = None

        self._joint_pub = self.create_publisher(JointState, "/wheel_joint_states", 10)
        self.create_subscription(
            EncoderSpeeds, "/encoders_speeds", self._enc_speeds_cb, 10
        )

        self.get_logger().info(
            "WheelJointStatePublisherNode ready — /encoders_speeds -> /wheel_joint_states "
            f"({self._right_joint}, {self._left_joint})"
        )

    def _enc_speeds_cb(self, msg: EncoderSpeeds) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        if self._prev_time_sec is None:
            self._prev_time_sec = now_sec
        else:
            dt = now_sec - self._prev_time_sec
            self._prev_time_sec = now_sec

            # Guard against startup glitches / clock jumps — same bound
            # odom_publisher_node uses.
            if 0.0 < dt < 1.0:
                speed_r = sanitize_float(msg.motor1_speed, 0.0)
                speed_l = sanitize_float(msg.motor2_speed, 0.0)

                # Linear wheel-surface speed (m/s), same conversion as
                # odom_publisher_node, then to angular speed (rad/s).
                v_right = speed_r * 2.0 * math.pi * _WHEEL_RADIUS_M * _ENC_SPEED_SCALE
                v_left  = speed_l * 2.0 * math.pi * _WHEEL_RADIUS_M * _ENC_SPEED_SCALE

                w_right = v_right / _WHEEL_RADIUS_M
                w_left  = v_left  / _WHEEL_RADIUS_M

                self._right_angle += w_right * dt
                self._left_angle  += w_left  * dt

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name     = [self._right_joint, self._left_joint]
        js.position = [self._right_angle, self._left_angle]

        self._joint_pub.publish(js)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheelJointStatePublisherNode()
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