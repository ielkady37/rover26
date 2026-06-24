#!/usr/bin/env python3
"""
HoverBoardNode — ROS2 node that bridges an ActuatorCommand subscription to a
hoverboard over UART, and publishes wheel feedback speeds on /encoders.

    • /esp_tx  (interfaces/msg/ActuatorCommand)  →  hoverboard UART TX
    • hoverboard UART RX                          →  /encoders (EncoderRevolutions)

TX frame  (8 bytes, little-endian):
    struct.pack('<HhhH', START_FRAME, steer, speed, checksum)
    checksum = (START_FRAME ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)) & 0xFFFF

RX frame (18 bytes, little-endian):
    struct.unpack('<HhhhhhhHH', data)
    → start, cmd1, cmd2, speedR, speedL, batV, boardTemp, cmdLed, chk
    chk = XOR of all preceding uint16 words (start through cmdLed, treating
          each signed int16 as a uint16 for the XOR).

Mapping: ActuatorCommand → hoverboard
    m1  →  steer channel  (scaled by max_speed, negated when dir=1, zeroed when brake=1)
    m2  →  speed channel  (scaled by max_speed, negated when dir=1, zeroed when brake=1)

Published speeds: enc1_net_rev = speedR (raw hoverboard RPM/ticks),
                  enc2_net_rev = speedL (raw hoverboard RPM/ticks)

ROS2 parameters
───────────────
    uart_port   (str)              UART device path (e.g. /dev/ttyUSB0) — REQUIRED
    baudrate    (int,   115200)    UART baud rate
    timeout     (float, 0.1)      serial read timeout in seconds
    max_speed   (int,   500)      duty-cycle 1.0 maps to this int16 value (range 0–1000)
    base_frame  (str,   'base_link')
"""
import struct
import threading
import time

import serial
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from interfaces.msg import ActuatorCommand as ActuatorCommandMsg, EncoderRevolutions


# Hoverboard protocol constants
_START_FRAME: int = 0xABCD
_TX_SIZE:     int = 8   # bytes
_RX_SIZE:     int = 18  # bytes
_INT16_MIN: int = -32768
_INT16_MAX: int =  32767


def _build_tx_frame(steer: int, speed: int) -> bytes:
    """Pack an 8-byte hoverboard command frame."""
    steer = max(_INT16_MIN, min(_INT16_MAX, steer))
    speed = max(_INT16_MIN, min(_INT16_MAX, speed))
    checksum = (_START_FRAME ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)) & 0xFFFF
    return struct.pack('<HhhH', _START_FRAME, steer, speed, checksum)


def _parse_rx_frame(data: bytes) -> tuple[int, int] | None:
    """
    Unpack an 18-byte hoverboard feedback frame.

    Returns (speedR, speedL) on success, or None if the frame is invalid
    (wrong start byte or checksum mismatch).
    """
    if len(data) != _RX_SIZE:
        return None

    fields = struct.unpack('<HhhhhhhHH', data)
    # fields: start, cmd1, cmd2, speedR, speedL, batV, boardTemp, cmdLed, chk
    start, cmd1, cmd2, speed_r, speed_l, bat_v, board_temp, cmd_led, chk = fields

    if start != _START_FRAME:
        return None

    # Checksum: XOR of every preceding field treated as uint16
    expected_chk = (
        start
        ^ (cmd1   & 0xFFFF)
        ^ (cmd2   & 0xFFFF)
        ^ (speed_r & 0xFFFF)
        ^ (speed_l & 0xFFFF)
        ^ (bat_v  & 0xFFFF)
        ^ (board_temp & 0xFFFF)
        ^ (cmd_led & 0xFFFF)
    ) & 0xFFFF

    if expected_chk != chk:
        return None

    return speed_r, speed_l


class HoverBoardNode(Node):
    def __init__(self) -> None:
        super().__init__("hoverboard_node")

        # ── parameters ──────────────────────────────────────────────────────
        self.declare_parameter("uart_port",  "")
        self.declare_parameter("baudrate",   115200)
        self.declare_parameter("timeout",    0.1)
        self.declare_parameter("max_speed",  500)
        self.declare_parameter("base_frame", "base_link")

        uart_port:  str   = self.get_parameter("uart_port").value
        baudrate:   int   = self.get_parameter("baudrate").value
        timeout:    float = self.get_parameter("timeout").value
        self._max_speed: int = int(self.get_parameter("max_speed").value)

        if not uart_port:
            self.get_logger().fatal(
                "Parameter 'uart_port' is required but was not set. "
                "Start with: --ros-args -p uart_port:=/dev/ttyUSBX"
            )
            raise RuntimeError("uart_port parameter is required")

        # ── open serial port ────────────────────────────────────────────────
        try:
            self._serial = serial.Serial(
                port=uart_port,
                baudrate=baudrate,
                timeout=timeout,
                write_timeout=timeout,
                dsrdtr=False,   # prevent DTR toggle from resetting the board on open
                rtscts=False,
            )
            # Allow the board to finish booting before we start exchanging frames.
            time.sleep(3.0)
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
        except Exception as e:
            self.get_logger().fatal(f"Failed to open {uart_port}: {e}")
            raise

        # ── thread safety for writer ─────────────────────────────────────────
        self._serial_lock = threading.Lock()

        # ── publisher ────────────────────────────────────────────────────────
        self._encoder_pub = self.create_publisher(EncoderRevolutions, "/encoders", 10)

        # ── subscriber ──────────────────────────────────────────────────────
        self._cmd_sub = self.create_subscription(
            ActuatorCommandMsg,
            "/esp_tx",
            self._cmd_cb,
            10,
        )

        # ── background reader thread ─────────────────────────────────────────
        self._running = True
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        self.get_logger().info(
            f"HoverBoardNode started — port={uart_port} baud={baudrate} max_speed={self._max_speed}"
        )

    # ── /esp_tx callback ─────────────────────────────────────────────────────

    def _cmd_cb(self, msg: ActuatorCommandMsg) -> None:
        """Convert ActuatorCommand → hoverboard TX frame and write to serial."""
        m1_speed = max(-1.0, min(1.0, float(msg.m1_speed)))
        m2_speed = max(-1.0, min(1.0, float(msg.m2_speed)))

        # m1 → steer
        steer = int(m1_speed * self._max_speed)
        if msg.m1_dir == 1:
            steer = -steer
        if msg.m1_brake == 1:
            steer = 0

        # m2 → speed
        speed = int(m2_speed * self._max_speed)
        if msg.m2_dir == 1:
            speed = -speed
        if msg.m2_brake == 1:
            speed = 0

        frame = _build_tx_frame(steer, speed)
        try:
            with self._serial_lock:
                self._serial.write(frame)
        except Exception as e:
            self.get_logger().warn(
                f"Hoverboard write failed: {e}",
                throttle_duration_sec=5.0,
            )

    # ── background reader loop ───────────────────────────────────────────────

    def _reader_loop(self) -> None:
        """Continuously read 18-byte feedback frames from the hoverboard."""
        while self._running:
            try:
                with self._serial_lock:
                    data = self._serial.read(_RX_SIZE)
            except Exception as e:
                if self._running:
                    self.get_logger().warn(
                        f"Hoverboard read error: {e}",
                        throttle_duration_sec=5.0,
                    )
                continue

            if len(data) != _RX_SIZE:
                # Timeout with no data — spin quietly
                continue

            result = _parse_rx_frame(data)
            if result is None:
                self.get_logger().warn(
                    "Hoverboard RX frame invalid (bad start byte or checksum) — re-syncing",
                    throttle_duration_sec=2.0,
                )
                # Discard one byte to re-sync the stream
                try:
                    with self._serial_lock:
                        self._serial.read(1)
                except Exception:
                    pass
                continue

            speed_r, speed_l = result
            self._publish_encoders(speed_r, speed_l)

    # ── publisher helper ─────────────────────────────────────────────────────

    def _publish_encoders(self, speed_r: int, speed_l: int) -> None:
        msg = EncoderRevolutions()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("base_frame").value
        msg.enc1_net_rev    = float(speed_r)
        msg.enc2_net_rev    = float(speed_l)
        self._encoder_pub.publish(msg)

    # ── cleanup ──────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        self._running = False
        self._reader_thread.join(timeout=2.0)
        try:
            with self._serial_lock:
                self._serial.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HoverBoardNode()
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
