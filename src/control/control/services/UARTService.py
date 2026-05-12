#!/usr/bin/env python3
import time

import serial
from zope.interface import implementer

from control.exceptions.SensorInitializationError import SensorInitializationError
from control.exceptions.SensorReadError import SensorReadError
from control.interface.iCommunicationProtocol import iCommunicationProtocol


@implementer(iCommunicationProtocol)
class UARTService:
    data_contract: int = 0xAB  # start byte identifying a contract packet

    def __init__(self, comm_details: dict) -> None:
        """Setup the requirements to handle UART communication.

        Expected keys in comm_details:
            port (str): serial device path (e.g. /dev/ttyUSB0)
            baudrate (int, optional): UART baud rate (default 115200)
            timeout (float, optional): read timeout in seconds (default 0.2)
            write_timeout (float, optional): write timeout in seconds (default 0.2)
            data_contract (int, optional): contract start byte for this link
        """
        self._port: str = comm_details["port"]
        self._baudrate: int = comm_details.get("baudrate", 115200)
        self._timeout: float = comm_details.get("timeout", 0.2)
        self._write_timeout: float = comm_details.get("write_timeout", 0.2)
        self.data_contract = comm_details.get("data_contract", self.data_contract)
        self._serial: serial.Serial | None = None

    def initialize(self) -> None:
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=self._timeout,
                write_timeout=self._write_timeout,
            )
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
        except Exception as e:
            raise SensorInitializationError(
                f"Failed to open UART port {self._port} @ {self._baudrate}: {e}"
            )

    def send(self, data: bytes) -> None:
        if self._serial is None:
            raise SensorInitializationError("UART is not initialized. Call initialize() first.")
        try:
            packet = data + bytes([self.compute_checksum(data)])
            written = self._serial.write(packet)
            if written != len(packet):
                raise SensorReadError(
                    f"UART send incomplete on {self._port}: wrote {written}/{len(packet)} bytes"
                )
            self._serial.flush()
        except SensorReadError:
            raise
        except Exception as e:
            raise SensorReadError(f"UART send failed on {self._port}: {e}")

    def receive(self, length: int) -> bytes:
        if self._serial is None:
            raise SensorInitializationError("UART is not initialized. Call initialize() first.")

        try:
            deadline = time.monotonic() + max(self._timeout, 0.01)
            data = bytearray()
            while len(data) < length and time.monotonic() < deadline:
                chunk = self._serial.read(length - len(data))
                if chunk:
                    data.extend(chunk)
                    continue
                # Avoid tight spin while waiting for bytes.
                time.sleep(0.001)

            if len(data) != length:
                raise SensorReadError(
                    f"UART receive timeout on {self._port}: expected {length} bytes, got {len(data)}"
                )

            raw = bytes(data)
            self.validateData(raw)
            return raw
        except SensorReadError:
            raise
        except Exception as e:
            raise SensorReadError(f"UART receive failed on {self._port}: {e}")

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def validateData(self, data: bytes) -> None:
        """Validate XOR checksum of a full packet.

        The last byte of *data* must equal the XOR of all preceding bytes.
        Raises SensorReadError if the checksum does not match.
        """
        if len(data) < 2:
            raise SensorReadError("UART packet too short for checksum validation")
        expected = self.compute_checksum(data[:-1])
        if expected != data[-1]:
            raise SensorReadError(
                f"UART checksum failed: expected 0x{expected:02X}, got 0x{data[-1]:02X}"
            )

    @staticmethod
    def compute_checksum(data: bytes) -> int:
        """Compute XOR checksum over all bytes in data (for building outgoing packets)."""
        cs = 0
        for b in data:
            cs ^= b
        return cs