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
                dsrdtr=False,        # prevent DTR toggle from resetting the ESP32 on port open
                rtscts=False,        # prevent RTS toggle for the same reason
            )
            # Give the ESP32 time to finish booting in case it was power-cycled
            # or just programmed.  3 s covers the worst-case bootloader delay.
            time.sleep(3.0)
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

    def receive(self, length: int, start_byte: int | None = None) -> bytes:
        """Read exactly *length* bytes from the UART.

        If *start_byte* is given, bytes are discarded until the first byte of
        the read equals *start_byte*.  This re-synchronises the stream after
        any framing error without requiring an ESP32 reset.
        """
        if self._serial is None:
            raise SensorInitializationError("UART is not initialized. Call initialize() first.")

        try:
            # ── optional start-byte scan ──────────────────────────────────────
            if start_byte is not None:
                sync_deadline = time.monotonic() + self._timeout
                while True:
                    b = self._serial.read(1)
                    if not b:
                        if time.monotonic() >= sync_deadline:
                            raise SensorReadError(
                                f"UART sync timeout on {self._port}: "
                                f"start byte 0x{start_byte:02X} not found"
                            )
                        continue
                    if b[0] == start_byte:
                        # Found the start byte — read the remaining bytes below
                        prefix = b
                        break
                remaining = length - 1
            else:
                prefix = b""
                remaining = length

            # ── read remaining bytes ──────────────────────────────────────────
            deadline = time.monotonic() + max(self._timeout, 0.01)
            data = bytearray(prefix)
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

    def read_ack(self, timeout: float = 0.1) -> int | None:
        """Try to read one ACK byte with a short timeout. Returns the byte or None."""
        if self._serial is None:
            return None
        old_timeout = self._serial.timeout
        try:
            self._serial.timeout = timeout
            b = self._serial.read(1)
            return b[0] if b else None
        except Exception:
            return None
        finally:
            self._serial.timeout = old_timeout


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