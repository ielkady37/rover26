#!/usr/bin/env python3
import spidev
from zope.interface import implementer
from control.interface.iCommunicationProtocol import iCommunicationProtocol
from control.exceptions.SensorInitializationError import SensorInitializationError
from control.exceptions.SensorReadError import SensorReadError


@implementer(iCommunicationProtocol)
class SPIService:
    data_contract: int = 0xAB  # start byte identifying a contract packet

    def __init__(self, comm_details: dict) -> None:
        """Setup the requirements to handle SPI communication.

        Expected keys in comm_details:
            bus (int): SPI bus number (e.g. 0 for /dev/spidev0.x)
            device (int): chip-select line (e.g. 0 for /dev/spidev0.0)
            max_speed_hz (int, optional): clock speed in Hz (default 1_000_000)
            mode (int, optional): SPI mode 0-3 (default 0)
        """
        self._bus: int = comm_details["bus"]
        self._device: int = comm_details["device"]
        self._max_speed_hz: int = comm_details.get("max_speed_hz", 1_000_000)
        self._mode: int = comm_details.get("mode", 0)
        self._spi: spidev.SpiDev | None = None

    def initialize(self) -> None:
        try:
            self._spi = spidev.SpiDev()
            self._spi.open(self._bus, self._device)
            self._spi.max_speed_hz = self._max_speed_hz
            self._spi.mode = self._mode
        except Exception as e:
            raise SensorInitializationError(
                f"Failed to open SPI bus {self._bus}, device {self._device}: {e}"
            )

    def send(self, data: list) -> None:
        if self._spi is None:
            raise SensorInitializationError("SPI is not initialized. Call initialize() first.")
        try:
            self._spi.writebytes(data)
        except Exception as e:
            raise SensorReadError(f"SPI send failed: {e}")

    def receive(self, length: int = 1) -> bytes:
        if self._spi is None:
            raise SensorInitializationError("SPI is not initialized. Call initialize() first.")
        try:
            raw = bytes(self._spi.xfer2([0x00] * length))
        except Exception as e:
            raise SensorReadError(f"SPI receive failed: {e}")
        self.validateData(raw)
        return raw

    def close(self) -> None:
        if self._spi is not None:
            self._spi.close()
            self._spi = None

    def validateData(self, data: bytes) -> None:
        """Validate XOR checksum of a full packet.

        The last byte of *data* must equal the XOR of all preceding bytes.
        Raises SensorReadError if the checksum does not match.
        """
        expected = self.compute_checksum(data[:-1])
        if expected != data[-1]:
            raise SensorReadError(
                f"SPI checksum failed: expected 0x{expected:02X}, got 0x{data[-1]:02X}"
            )

    @staticmethod
    def compute_checksum(data: bytes) -> int:
        """Compute XOR checksum over all bytes in data (for building outgoing packets)."""
        cs = 0
        for b in data:
            cs ^= b
        return cs
