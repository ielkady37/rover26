import struct
import time

import serial
import rclpy
from rclpy.node import Node
from interfaces.msg import ActuatorCommand as ActuatorCommandMsg, EncoderSpeeds
from utils.Logger import RoverLogger

_START_FRAME: int = 0xABCD
_TX_SIZE:     int = 8
_RX_SIZE:     int = 18
_INT16_MIN:   int = -32768
_INT16_MAX:   int =  32767


def _build_tx_frame(steer: int, speed: int) -> bytes:
    steer = max(_INT16_MIN, min(_INT16_MAX, steer))
    speed = max(_INT16_MIN, min(_INT16_MAX, speed))
    checksum = (_START_FRAME ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)) & 0xFFFF
    return struct.pack('<HhhH', _START_FRAME, steer, speed, checksum)


def _parse_rx_frame(data: bytes) -> tuple[int, int] | None:
    if len(data) != _RX_SIZE:
        return None
    fields = struct.unpack('<HhhhhhhHH', data)
    start, cmd1, cmd2, speed_r, speed_l, bat_v, board_temp, cmd_led, chk = fields
    if start != _START_FRAME:
        return None
    expected_chk = (
        start
        ^ (cmd1      & 0xFFFF)
        ^ (cmd2      & 0xFFFF)
        ^ (speed_r   & 0xFFFF)
        ^ (speed_l   & 0xFFFF)
        ^ (bat_v     & 0xFFFF)
        ^ (board_temp & 0xFFFF)
        ^ (cmd_led   & 0xFFFF)
    ) & 0xFFFF
    if expected_chk != chk:
        return None
    return speed_r, speed_l


class HoverBoardNode(Node):
    def __init__(self) -> None:
        super().__init__("hoverboard_node")

        self.declare_parameter("uart_port",   "")
        self.declare_parameter("baudrate",    115200)
        self.declare_parameter("max_speed",   500)
        self.declare_parameter("base_frame",  "base_link")
        self.declare_parameter("read_hz",     100)      # how often to poll serial RX

        uart_port  = self.get_parameter("uart_port").value
        baudrate   = self.get_parameter("baudrate").value
        self._max_speed  = int(self.get_parameter("max_speed").value)
        read_hz    = int(self.get_parameter("read_hz").value)
        self._logger = RoverLogger(self)

        if not uart_port:
            self._logger.err("Parameter 'uart_port' is required.")
            raise RuntimeError("uart_port parameter is required")

        # ── serial port: timeout=0 → non-blocking reads ──────────────────
        try:
            self._serial = serial.Serial(
                port=uart_port,
                baudrate=baudrate,
                timeout=0,          # non-blocking: returns immediately with whatever is in buffer
                write_timeout=0.05,
                dsrdtr=False,
                rtscts=False,
            )
            time.sleep(3.0)
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
        except Exception as e:
            self._logger.err(f"Failed to open {uart_port}: {e}")
            raise

        # ── accumulation buffer for partial frames ───────────────────────
        self._rx_buf = bytearray()

        # ── publisher + subscriber ───────────────────────────────────────
        self._encoder_pub = self.create_publisher(EncoderSpeeds, "/encoders_speeds", 10)
        self._cmd_sub = self.create_subscription(
            ActuatorCommandMsg, "/esp_tx", self._cmd_cb, 10
        )

        # ── single timer replaces the reader thread ──────────────────────
        self._read_timer = self.create_timer(1.0 / read_hz, self._read_tick)

        self._logger.info(
            f"HoverBoardNode started — port={uart_port} baud={baudrate} "
            f"max_speed={self._max_speed} read_hz={read_hz}"
        )

    # ── command callback: fires immediately on message receipt ───────────

    def _cmd_cb(self, msg: ActuatorCommandMsg) -> None:
        # self._logger.info(
        #     f"Received ActuatorCommand: m1_speed={msg.m1_speed} m1_dir={msg.m1_dir} "
        #     f"m1_brake={msg.m1_brake} m2_speed={msg.m2_speed} m2_dir={msg.m2_dir} "
        #     f"m2_brake={msg.m2_brake}"
        # )
        left  = max(0.0, min(1.0, float(msg.m1_speed) / 255.0))
        right = max(0.0, min(1.0, float(msg.m2_speed) / 255.0))

        if msg.m1_dir == 1:
            left = left
        if msg.m1_brake == 1:
            left = 0.0

        if msg.m2_dir == 1:
            right = right
        if msg.m2_brake == 1:
            right = 0.0

        speed = int(((left + right) / 2.0) * self._max_speed)
        steer = int(((left - right) / 2.0) * self._max_speed)
        # self._logger.info(f"Received command: left={left:.2f} right={right:.2f} → speed={speed} steer={steer}")

        try:
            self._serial.write(_build_tx_frame(steer, speed))
        except Exception as e:
            self._logger.warn(
                f"Hoverboard write failed: {e}"
            )

    # ── timer tick: drain whatever bytes arrived since last tick ─────────

    def _read_tick(self) -> None:
        # Non-blocking: read everything currently in the OS buffer
        waiting = self._serial.in_waiting
        if waiting == 0:
            return

        self._rx_buf.extend(self._serial.read(waiting))

        # Consume as many complete frames as are available
        while len(self._rx_buf) >= _RX_SIZE:
            # Find the start frame marker to stay in sync
            idx = self._rx_buf.find(
                _START_FRAME.to_bytes(2, byteorder='little')
            )

            if idx == -1:
                # No start marker anywhere — discard everything except last byte
                # (last byte might be the first byte of an incoming start marker)
                self._rx_buf = self._rx_buf[-1:]
                return

            if idx > 0:
                # Junk bytes before the marker — discard them
                self._logger.warn(
                    f"Discarding {idx} out-of-sync bytes"
                )
                self._rx_buf = self._rx_buf[idx:]

            if len(self._rx_buf) < _RX_SIZE:
                # Have the start marker but frame isn't complete yet — wait for next tick
                return

            frame_bytes = bytes(self._rx_buf[:_RX_SIZE])
            result = _parse_rx_frame(frame_bytes)

            if result is None:
                # Start marker found but checksum failed — skip one byte and re-sync
                self._rx_buf = self._rx_buf[1:]
                continue

            self._rx_buf = self._rx_buf[_RX_SIZE:]
            self._publish_encoders(*result)

    # ── publisher helper ─────────────────────────────────────────────────

    def _publish_encoders(self, speed_r: int, speed_l: int) -> None:
        msg = EncoderSpeeds()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("base_frame").value
        msg.motor1_speed    = float(speed_r)
        msg.motor2_speed    = float(speed_l)
        self._encoder_pub.publish(msg)

    # ── cleanup ──────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        try:
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