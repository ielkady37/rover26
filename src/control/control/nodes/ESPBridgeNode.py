#!/usr/bin/env python3
"""
ESPBridgeNode — ROS2 node that owns the SPI link to the ESP32.

On startup
──────────
  1. Opens SPI and reads the one-time ContractPacket from the ESP32.

Every timer tick (default 50 Hz)
─────────────────────────────────
  2. Reads one 62-byte SensorPacket.
  3. Publishes sensor_msgs/Imu  on /imu
  4. Publishes nav_msgs/Odometry on /odom  (dead-reckoning from encoders)

ROS2 parameters
───────────────
  bus           (int,   0)       SPI bus
  device        (int,   0)       SPI chip-select
  max_speed_hz  (int,   1000000) SPI clock
  spi_mode      (int,   0)       SPI mode 0-3
  wheel_radius  (float, 0.05)    metres — wheel radius
  wheel_base    (float, 0.30)    metres — centre-to-centre track width
  publish_rate  (float, 50.0)    Hz — read + publish frequency
  imu_frame     (str,   'imu_link')
  odom_frame    (str,   'odom')
  base_frame    (str,   'base_link')
"""
import math
import threading

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
from interfaces.msg import EulerAngles, EncoderRevolutions

from control.services.ESPDataController import ESPDataController
from control.DTOs.SensorData import SensorData
from control.exceptions.SensorInitializationError import SensorInitializationError
from control.exceptions.SensorReadError import SensorReadError

# Unknown covariance sentinel (ROS2 REP-145: first element = -1 means unknown)
_COV_UNKNOWN: list[float] = [-1.0] + [0.0] * 8


class ESPBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("esp_bridge_node")

        # declare parameters 
        self.declare_parameter("bus",          0)
        self.declare_parameter("device",       0)
        self.declare_parameter("max_speed_hz", 1_000_000)
        self.declare_parameter("spi_mode",     0)
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("wheel_base",   0.30)
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("imu_frame",    "imu_link")
        self.declare_parameter("odom_frame",   "odom")
        self.declare_parameter("base_frame",   "base_link")

        # publishers
        self._imu_pub     = self.create_publisher(Imu,                "/imu",      10)
        self._odom_pub    = self.create_publisher(Odometry,           "/odom",     10)
        self._euler_pub   = self.create_publisher(EulerAngles,        "/euler",    10)
        self._encoder_pub = self.create_publisher(EncoderRevolutions, "/encoders", 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        # odometry state
        self._x:     float = 0.0
        self._y:     float = 0.0
        self._theta: float = 0.0
        self._prev_enc1: float | None = None
        self._prev_enc2: float | None = None

        # initialise SPI / contract
        comm = {
            "bus":          self.get_parameter("bus").value,
            "device":       self.get_parameter("device").value,
            "max_speed_hz": self.get_parameter("max_speed_hz").value,
            "mode":         self.get_parameter("spi_mode").value,
        }
        self._ctrl = ESPDataController(comm)
        try:
            self._ctrl.initialize()
            self.get_logger().info(
                f"ESP32 contract received — "
                f"fields={self._ctrl.fields}, "
                f"ticks_per_rev={self._ctrl.ticks_per_rev}, "
                f"packet_size={self._ctrl._packet_size} bytes"
            )
        except SensorInitializationError as e:
            self.get_logger().fatal(f"Failed to initialise SPI link: {e}")
            raise

        # latest data cache — written by reader thread, read by timer
        self._latest_data: SensorData | None = None
        self._data_lock = threading.Lock()

        # background thread: reads SPI as fast as possible (~250 Hz at 125 kHz)
        # so the publish timer always gets data that is at most 1 SPI-transfer old
        self._running = True
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        # periodic publish timer (does NOT block on SPI)
        rate = self.get_parameter("publish_rate").value
        self._timer = self.create_timer(1.0 / rate, self._timer_cb)
        self.get_logger().info(f"ESPBridgeNode running at {rate:.1f} Hz")

    # background SPI reader

    def _reader_loop(self) -> None:
        while self._running:
            try:
                data = self._ctrl.read()
                with self._data_lock:
                    self._latest_data = data
            except SensorReadError as e:
                # self.get_logger().warn(
                #     f"SPI read error: {e}",
                #     throttle_duration_sec=5.0,
                # )
                pass

    # timer callback

    def _timer_cb(self) -> None:
        with self._data_lock:
            data = self._latest_data
        if data is None:
            return  # reader hasn't got first packet yet

        stamp = self.get_clock().now().to_msg()
        self._publish_imu(data, stamp)
        self._publish_odom(data, stamp)
        self._publish_euler(data, stamp)
        self._publish_encoders(data, stamp)

    # /imu

    def _publish_imu(self, data: SensorData, stamp: Time) -> None:
        msg = Imu()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.get_parameter("imu_frame").value

        msg.orientation = Quaternion(x=float(data.qx), y=float(data.qy),
                                     z=float(data.qz), w=float(data.qw))
        msg.orientation_covariance = _COV_UNKNOWN

        msg.angular_velocity.x = float(data.gx)
        msg.angular_velocity.y = float(data.gy)
        msg.angular_velocity.z = float(data.gz)
        msg.angular_velocity_covariance = _COV_UNKNOWN

        msg.linear_acceleration.x = float(data.ax)
        msg.linear_acceleration.y = float(data.ay)
        msg.linear_acceleration.z = float(data.az)
        msg.linear_acceleration_covariance = _COV_UNKNOWN

        self._imu_pub.publish(msg)

    # /encoders

    def _publish_encoders(self, data: SensorData, stamp: Time) -> None:
        msg = EncoderRevolutions()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.get_parameter("base_frame").value
        msg.enc1_net_rev    = float(data.enc1_net_rev)
        msg.enc2_net_rev    = float(data.enc2_net_rev)
        self._encoder_pub.publish(msg)

    # /euler

    def _publish_euler(self, data: SensorData, stamp: Time) -> None:
        msg = EulerAngles()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.get_parameter("imu_frame").value
        msg.yaw   = float(data.yaw)
        msg.pitch = float(data.pitch)
        msg.roll  = float(data.roll)
        self._euler_pub.publish(msg)

    # /odom 

    def _publish_odom(self, data: SensorData, stamp: Time) -> None:
        wheel_radius: float = self.get_parameter("wheel_radius").value
        wheel_base:   float = self.get_parameter("wheel_base").value
        odom_frame:   str   = self.get_parameter("odom_frame").value
        base_frame:   str   = self.get_parameter("base_frame").value

        #  dead-reckoning update
        if self._prev_enc1 is None:
            # first reading — seed without moving
            self._prev_enc1 = data.enc1_net_rev
            self._prev_enc2 = data.enc2_net_rev
        else:
            delta_left  = (data.enc1_net_rev - self._prev_enc1) * 2.0 * math.pi * wheel_radius
            delta_right = (data.enc2_net_rev - self._prev_enc2) * 2.0 * math.pi * wheel_radius
            self._prev_enc1 = data.enc1_net_rev
            self._prev_enc2 = data.enc2_net_rev

            delta_dist  = (delta_left + delta_right) * 0.5
            delta_theta = (delta_right - delta_left) / wheel_base

            self._theta += delta_theta
            self._x     += delta_dist * math.cos(self._theta)
            self._y     += delta_dist * math.sin(self._theta)

        # build Odometry message 
        odom_quat = _yaw_to_quat(self._theta)

        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = odom_frame
        msg.child_frame_id  = base_frame

        msg.pose.pose.position.x  = self._x
        msg.pose.pose.position.y  = self._y
        msg.pose.pose.position.z  = 0.0
        msg.pose.pose.orientation = odom_quat

        self._odom_pub.publish(msg)

        # broadcast TF odom → base_link 
        tf = TransformStamped()
        tf.header.stamp            = stamp
        tf.header.frame_id         = odom_frame
        tf.child_frame_id          = base_frame
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.translation.z = 0.0
        tf.transform.rotation      = odom_quat
        self._tf_broadcaster.sendTransform(tf)

    # cleanup

    def destroy_node(self) -> None:
        self._running = False
        self._reader_thread.join(timeout=1.0)
        self._ctrl.close()
        super().destroy_node()


# helpers

def _yaw_to_quat(yaw: float) -> Quaternion:
    """Convert a 2D yaw angle (radians) to a geometry_msgs/Quaternion."""
    half = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ESPBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
