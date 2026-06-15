#!/usr/bin/env python3
"""
SerialService — replaces SPIService and I2CService when ESPs are connected via USB.

The same binary packet protocol (XOR checksum, start bytes) is kept exactly as-is.
Only the transport layer changes: instead of /dev/spidev0.0 or /dev/i2c-1,
we open a serial port like /dev/ttyUSB0 or /dev/ttyUSB1.

Usage in ESPBridgeNode:
    sensor_serial  = SerialService({"port": "/dev/ttyUSB0", "baudrate": 115200})
    actuator_serial = SerialService({"port": "/dev/ttyUSB1", "baudrate": 115200})
"""

import serial
from zope.interface import implementer
from control.interface.iCommunicationProtocol import iCommunicationProtocol
from control.exceptions.SensorInitializationError import SensorInitializationError
from control.exceptions.SensorReadError import SensorReadError


@implementer(iCommunicationProtocol)
class SerialService:
    # Start byte for contract packets (same as SPIService — the ESP firmware is unchanged)
    data_contract: int = 0xAB

    def __init__(self, comm_details: dict) -> None:
        """
        Setup serial communication details.

        Expected keys in comm_details:
            port     (str): Serial port path, e.g. '/dev/ttyUSB0'
            baudrate (int): Baud rate, e.g. 115200 (must match ESP firmware)
            timeout  (float, optional): Read timeout in seconds (default 2.0)
        """
        self._port: str = comm_details["port"]
        self._baudrate: int = comm_details.get("baudrate", 115200)
        self._timeout: float = comm_details.get("timeout", 2.0)
        self._serial: serial.Serial | None = None

    def initialize(self) -> None:
        """Open the serial port. Raises SensorInitializationError on failure."""
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=self._timeout
            )
            # Flush any leftover bytes from before we connected
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
        except Exception as e:
            raise SensorInitializationError(
                f"Failed to open serial port {self._port} at {self._baudrate} baud: {e}"
            )

    def send(self, data: bytes) -> None:
        """Append XOR checksum and write bytes to the serial port."""
        if self._serial is None:
            raise SensorInitializationError("Serial port not initialized. Call initialize() first.")
        try:
            # Compute and append checksum (same logic as I2CService)
            packet = data + bytes([self.compute_checksum(data)])
            self._serial.write(packet)
        except SensorInitializationError:
            raise
        except Exception as e:
            raise SensorReadError(f"Serial send failed on {self._port}: {e}")

    def receive(self, length: int = 1) -> bytes:
        """
        Read exactly `length` bytes from serial port and validate checksum.
        Blocks until data arrives (up to timeout seconds).
        """
        if self._serial is None:
            raise SensorInitializationError("Serial port not initialized. Call initialize() first.")
        try:
            raw = self._serial.read(length)
            if len(raw) < length:
                # Timeout expired before we got enough bytes
                raise SensorReadError(
                    f"Serial read timeout on {self._port}: expected {length} bytes, got {len(raw)}"
                )
        except SensorInitializationError:
            raise
        except SensorReadError:
            raise
        except Exception as e:
            raise SensorReadError(f"Serial receive failed on {self._port}: {e}")

        # Validate checksum (same logic as SPIService)
        self.validateData(raw)
        return raw

    def close(self) -> None:
        """Close the serial port."""
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def validateData(self, data: bytes) -> None:
        """
        Validate XOR checksum of a full packet.
        The last byte must equal XOR of all preceding bytes.
        Raises SensorReadError if checksum fails.
        """
        if len(data) < 2:
            raise SensorReadError("Packet too short to validate checksum.")
        expected = self.compute_checksum(data[:-1])
        if expected != data[-1]:
            raise SensorReadError(
                f"Serial checksum failed on {self._port}: "
                f"expected 0x{expected:02X}, got 0x{data[-1]:02X}"
            )

    @staticmethod
    def compute_checksum(data: bytes) -> int:
        """XOR all bytes together — matches the ESP32 firmware checksum."""
        cs = 0
        for b in data:
            cs ^= b
        return cs
