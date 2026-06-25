#!/usr/bin/env python3
"""
ESPBridgeNode — ROS2 node that owns both ESP32 UART links.

    • UART(sensor)   → sensor ESP32   (read)   publishes /imu /odom /euler /encoders
    • UART(actuator) → actuator ESP32 (write)  subscribes /esp_tx

On startup
──────────
    1. Opens sensor UART, reads one-time ContractPacket from sensor ESP32.
    2. Opens actuator UART, sends one-time ContractPacket to actuator ESP32.

Every timer tick
─────────────────
  3. Reads one SensorPacket and publishes sensor topics.

On every /esp_tx message
─────────────────────────
    4. Sends one ActuatorPacket to the actuator ESP32 via UART.

ROS2 parameters
───────────────
    --- UART / sensor ---
    sensor_port      (str,   '/dev/ttyUSB0')  sensor ESP32 UART device
    sensor_baudrate  (int,   115200)          UART baud rate
    uart_timeout     (float, 0.2)             UART read/write timeout seconds
  wheel_radius  (float, 0.05)    metres
  wheel_base    (float, 0.30)    metres
  publish_rate  (float, 100.0)   Hz
  imu_frame     (str,   'imu_link')
  odom_frame    (str,   'odom')
  base_frame    (str,   'base_link')
    --- UART / actuator ---
    actuator_port     (str, '/dev/ttyUSB1')  actuator ESP32 UART device
    actuator_baudrate (int, 115200)          UART baud rate
"""
import math
import threading

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
from interfaces.msg import EulerAngles, EncoderRevolutions, ActuatorCommand as ActuatorCommandMsg

from control.services.ESPDataController import ESPDataController
from control.services.UARTService import UARTService
from control.DTOs.SensorData import SensorData
from control.DTOs.ActuatorCommand import ActuatorCommand, MotorCommand
from control.exceptions.SensorInitializationError import SensorInitializationError
from control.exceptions.SensorReadError import SensorReadError

# Unknown covariance sentinel (ROS2 REP-145: first element = -1 means unknown)
_COV_UNKNOWN: list[float] = [-1.0] + [0.0] * 8


class ESPBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("esp_bridge_node")

        # declare parameters 
        self.declare_parameter("sensor_port",      "/dev/ttyUSB0")
        self.declare_parameter("sensor_baudrate",  115200)
        self.declare_parameter("actuator_port",    "/dev/ttyUSB2")
        self.declare_parameter("actuator_baudrate", 115200)
        self.declare_parameter("uart_timeout",     0.2)
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
        self._encoder_pub = self.create_publisher(EncoderRevolutions, "/encoders_revs", 10)
        self._gps_pub     = self.create_publisher(NavSatFix,           "/gps",      10)
        self._tf_broadcaster = TransformBroadcaster(self)

        # odometry state
        self._x:     float = 0.0
        self._y:     float = 0.0
        self._theta: float = 0.0
        self._prev_enc1: float | None = None
        self._prev_enc2: float | None = None

        # initialise UART sensor + actuator links / contracts
        uart_sensor_comm = {
            "port": self.get_parameter("sensor_port").value,
            "baudrate": self.get_parameter("sensor_baudrate").value,
            "timeout": self.get_parameter("uart_timeout").value,
            "data_contract": 0xAB,
        }
        uart_actuator_comm = {
            "port": self.get_parameter("actuator_port").value,
            "baudrate": self.get_parameter("actuator_baudrate").value,
            "timeout": self.get_parameter("uart_timeout").value,
            "data_contract": 0xBC,
        }
        self._sensor_ctrl = ESPDataController(UARTService(uart_sensor_comm))
        self._actuator_ctrl = ESPDataController(UARTService(uart_actuator_comm))
        self._sensor_ready = False
        self._actuator_ready = False

        # sensor init (non-fatal — node continues without sensor data on failure)
        try:
            self._sensor_ctrl.initialize_with_retry(
                on_retry=lambda attempt, total, exc: self.get_logger().warn(
                    f"UART sensor init attempt {attempt}/{total} failed: {exc} — retrying..."
                ),
                on_success=lambda attempt, total: self.get_logger().info(
                    f"UART sensor initialized on attempt {attempt}/{total}"
                ),
            )
            self._sensor_ctrl.read_contract()
            self._sensor_ready = True
            self.get_logger().info(
                f"Sensor ESP32 ready — "
                f"fields={self._sensor_ctrl.fields}, "
                f"ticks_per_rev={self._sensor_ctrl.ticks_per_rev}, "
                f"packet_size={self._sensor_ctrl._packet_size} bytes"
            )
        except SensorInitializationError as e:
            self.get_logger().error(
                f"Sensor ESP32 unavailable — UART init failed: {e}. "
                f"Node will run without sensor data."
            )

        # actuator init (non-fatal — node continues without actuator on failure)
        try:
            self._actuator_ctrl.initialize_with_retry(
                on_retry=lambda attempt, total, exc: self.get_logger().warn(
                    f"UART actuator init attempt {attempt}/{total} failed: {exc} — retrying..."
                ),
                on_success=lambda attempt, total: self.get_logger().info(
                    f"UART actuator initialized on attempt {attempt}/{total}"
                ),
            )
            self._actuator_ready = True
            self.get_logger().info(
                f"Actuator ESP32 ready — UART {uart_actuator_comm['port']} ready"
            )
        except SensorInitializationError as e:
            self._actuator_ready = False
            self.get_logger().error(
                f"Actuator ESP32 unavailable — UART init/contract failed: {e}. "
                f"Node will run in sensor-only mode."
            )

        # /esp_tx subscriber
        self._esp_tx_sub = self.create_subscription(
            ActuatorCommandMsg,
            "/esp_tx",
            self._esp_tx_cb,
            10,
        )

        # latest data cache — written by reader thread, read by timer
        self._latest_data: SensorData | None = None
        self._data_lock = threading.Lock()

        # background thread: reads sensor UART continuously
        # so the publish timer always gets the latest complete packet
        self._running = True
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        # periodic publish timer (does NOT block on UART reads)
        rate = self.get_parameter("publish_rate").value
        self._timer = self.create_timer(1.0 / rate, self._timer_cb)
        self.get_logger().info(f"ESPBridgeNode running at {rate:.1f} Hz")

    # /esp_tx → actuator ESP32

    def _esp_tx_cb(self, msg: ActuatorCommandMsg) -> None:
        cmd = ActuatorCommand(
            motor1=MotorCommand(
                dir=int(msg.m1_dir),
                brake=int(msg.m1_brake),
                speed=float(msg.m1_speed),
            ),
            motor2=MotorCommand(
                dir=int(msg.m2_dir),
                brake=int(msg.m2_brake),
                speed=float(msg.m2_speed),
            ),
            laser=int(msg.laser),
            flash=int(msg.flash),
            servo=float(msg.servo),
        )
        if not self._actuator_ready:
            self.get_logger().warn(
                "Actuator write skipped — UART not available.",
                throttle_duration_sec=5.0,
            )
            return
        try:
            self._actuator_ctrl.write(cmd)
        except SensorReadError as e:
            self.get_logger().warn(
                f"Actuator write failed: {e}",
                throttle_duration_sec=5.0,
            )

    # background UART reader

    def _reader_loop(self) -> None:
        if not self._sensor_ready:
            return
        while self._running:
            try:
                data = self._sensor_ctrl.receive()
                if data is not None:
                    with self._data_lock:
                        self._latest_data = data
            except SensorReadError:
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
        self._publish_gps(data, stamp)

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

    # /gps

    def _publish_gps(self, data: SensorData, stamp: Time) -> None:
        msg = NavSatFix()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.get_parameter("base_frame").value

        status = NavSatStatus()
        status.status  = NavSatStatus.STATUS_FIX if data.gps_fix >= 1.0 \
                         else NavSatStatus.STATUS_NO_FIX
        status.service = NavSatStatus.SERVICE_GPS
        msg.status = status

        msg.latitude  = float(data.gps_lat)
        msg.longitude = float(data.gps_lon)
        msg.altitude  = float(data.gps_alt)
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self._gps_pub.publish(msg)

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
        odom_frame:   str   = self.get_parameter("odom_frame").value
        base_frame:   str   = self.get_parameter("base_frame").value

        # Use IMU yaw directly as heading (degrees → radians).
        # This avoids encoder-based heading drift.
        imu_theta = math.radians(data.yaw)   # data.yaw is in [-180, 180] °

        # Dead-reckoning: distance still comes from encoders, heading from IMU.
        if self._prev_enc1 is None:
            # first reading — seed without moving
            self._prev_enc1 = data.enc1_net_rev
            self._prev_enc2 = data.enc2_net_rev
        else:
            delta_left  = (data.enc1_net_rev - self._prev_enc1) * 2.0 * math.pi * wheel_radius
            delta_right = (data.enc2_net_rev - self._prev_enc2) * 2.0 * math.pi * wheel_radius
            self._prev_enc1 = data.enc1_net_rev
            self._prev_enc2 = data.enc2_net_rev

            delta_dist = (delta_left + delta_right) * 0.5

            # Integrate position using IMU heading at the midpoint of the step.
            # Use the average of previous and current IMU theta to reduce error
            # during fast turns.
            mid_theta   = _wrap_angle(self._theta + _wrap_angle(imu_theta - self._theta) * 0.5)
            self._x    += delta_dist * math.cos(mid_theta)
            self._y    += delta_dist * math.sin(mid_theta)

        # Update stored heading from IMU (not accumulated encoder delta).
        self._theta = imu_theta

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
        self._sensor_ctrl.close()
        self._actuator_ctrl.close()
        super().destroy_node()


# helpers

def _yaw_to_quat(yaw: float) -> Quaternion:
    """Convert a 2D yaw angle (radians) to a geometry_msgs/Quaternion."""
    half = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def _wrap_angle(angle: float) -> float:
    """Wrap *angle* (radians) to [-π, π]."""
    return math.atan2(math.sin(angle), math.cos(angle))


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
