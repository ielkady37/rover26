#!/usr/bin/env python3
import spidev
from zope.interface import implementer
from control.interface.iCommunicationProtocol import iCommunicationProtocol
from control.exceptions.SensorInitializationError import SensorInitializationError
from control.exceptions.SensorReadError import SensorReadError


@implementer(iCommunicationProtocol)
class SPIService:
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

    def receive(self) -> list:
        if self._spi is None:
            raise SensorInitializationError("SPI is not initialized. Call initialize() first.")
        try:
            return self._spi.readbytes(1)
        except Exception as e:
            raise SensorReadError(f"SPI receive failed: {e}")

    def close(self) -> None:
        if self._spi is not None:
            self._spi.close()
            self._spi = None
