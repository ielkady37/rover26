#!/usr/bin/env python3
"""DummyNode publishes stable sensor data for bench testing.

Published topics:
- /imu (sensor_msgs/Imu): stable orientation, zero angular velocity,
  and gravity-only acceleration.
- /encoders_speeds (interfaces/EncoderSpeeds): zero wheel speeds.
- /encoers_speeds (interfaces/EncoderSpeeds): typo-compatible mirror.
- /scan (sensor_msgs/LaserScan): synthetic fixed-range lidar sweep.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Quaternion
from interfaces.msg import EncoderSpeeds


class DummyNode(Node):
	"""Publishes deterministic dummy IMU, encoder, and lidar messages."""

	_GRAVITY = 9.80665

	def __init__(self) -> None:
		super().__init__("dummy_node")

		self.declare_parameter("imu_topic", "/imu")
		self.declare_parameter("encoders_topic", "/encoders_speeds")
		self.declare_parameter("encoders_topic_typo", "/encoers_speeds")
		self.declare_parameter("scan_topic", "/scan")
		self.declare_parameter("imu_frame", "imu_link")
		self.declare_parameter("lidar_frame", "lidar_link")
		self.declare_parameter("rate_hz", 20.0)

		imu_topic = str(self.get_parameter("imu_topic").value)
		enc_topic = str(self.get_parameter("encoders_topic").value)
		enc_typo_topic = str(self.get_parameter("encoders_topic_typo").value)
		scan_topic = str(self.get_parameter("scan_topic").value)
		self._imu_frame = str(self.get_parameter("imu_frame").value)
		self._lidar_frame = str(self.get_parameter("lidar_frame").value)

		self._imu_pub = self.create_publisher(Imu, imu_topic, 10)
		self._enc_pub = self.create_publisher(EncoderSpeeds, enc_topic, 10)
		self._enc_typo_pub = self.create_publisher(EncoderSpeeds, enc_typo_topic, 10)
		self._scan_pub = self.create_publisher(LaserScan, scan_topic, 10)

		rate_hz = float(self.get_parameter("rate_hz").value)
		if not math.isfinite(rate_hz) or rate_hz <= 0.0:
			rate_hz = 20.0

		self._timer = self.create_timer(1.0 / rate_hz, self._publish_all)

		self.get_logger().info(
			f"DummyNode publishing stable data at {rate_hz:.1f} Hz "
			f"to {imu_topic}, {enc_topic}, {enc_typo_topic}, {scan_topic}"
		)

	def _publish_all(self) -> None:
		"""Publish one sample for IMU, encoders, and lidar."""
		try:
			now = self.get_clock().now().to_msg()

			imu_msg = Imu()
			imu_msg.header.stamp = now
			imu_msg.header.frame_id = self._imu_frame
			imu_msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
			imu_msg.orientation_covariance = [
				0.0025,
				0.0,
				0.0,
				0.0,
				0.0025,
				0.0,
				0.0,
				0.0,
				0.0025,
			]
			imu_msg.angular_velocity.x = 0.0
			imu_msg.angular_velocity.y = 0.0
			imu_msg.angular_velocity.z = 0.0
			imu_msg.angular_velocity_covariance = [
				0.001,
				0.0,
				0.0,
				0.0,
				0.001,
				0.0,
				0.0,
				0.0,
				0.001,
			]
			imu_msg.linear_acceleration.x = 0.0
			imu_msg.linear_acceleration.y = 0.0
			imu_msg.linear_acceleration.z = self._GRAVITY
			imu_msg.linear_acceleration_covariance = [
				0.01,
				0.0,
				0.0,
				0.0,
				0.01,
				0.0,
				0.0,
				0.0,
				0.01,
			]

			enc_msg = EncoderSpeeds()
			enc_msg.header.stamp = now
			enc_msg.motor1_speed = 0.0
			enc_msg.motor2_speed = 0.0

			scan_msg = LaserScan()
			scan_msg.header.stamp = now
			scan_msg.header.frame_id = self._lidar_frame
			scan_msg.angle_min = -math.pi
			scan_msg.angle_max = math.pi
			scan_msg.angle_increment = math.radians(1.0)
			scan_msg.time_increment = 0.0
			scan_msg.scan_time = 0.1
			scan_msg.range_min = 0.12
			scan_msg.range_max = 8.0
			point_count = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1
			scan_msg.ranges = [3.0] * point_count
			scan_msg.intensities = [100.0] * point_count

			self._imu_pub.publish(imu_msg)
			self._enc_pub.publish(enc_msg)
			self._enc_typo_pub.publish(enc_msg)
			self._scan_pub.publish(scan_msg)
		except Exception as exc:
			self.get_logger().error(f"DummyNode publish error: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DummyNode()
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
