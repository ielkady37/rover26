#!/usr/bin/env python3
import smbus2
from zope.interface import implementer
from control.interface.iCommunicationProtocol import iCommunicationProtocol
from control.exceptions.SensorInitializationError import SensorInitializationError
from control.exceptions.SensorReadError import SensorReadError


@implementer(iCommunicationProtocol)
class I2CService:
    def __init__(self, comm_details: dict) -> None:
        """Setup the requirements to handle I2C communication.

        Expected keys in comm_details:
            bus (int): I2C bus number (e.g. 1 for /dev/i2c-1)
            address (int): 7-bit I2C device address
            register (int, optional): default register to read/write (default 0x00)
            read_length (int, optional): number of bytes to read per receive() call (default 1)
        """
        self._bus_number: int = comm_details["bus"]
        self._address: int = comm_details["address"]
        self._register: int = comm_details.get("register", 0x00)
        self._read_length: int = comm_details.get("read_length", 1)
        self._bus: smbus2.SMBus | None = None

    def initialize(self) -> None:
        try:
            self._bus = smbus2.SMBus(self._bus_number)
        except Exception as e:
            raise SensorInitializationError(
                f"Failed to open I2C bus {self._bus_number}: {e}"
            )

    def send(self, data: list) -> None:
        if self._bus is None:
            raise SensorInitializationError("I2C bus is not initialized. Call initialize() first.")
        try:
            self._bus.write_i2c_block_data(self._address, self._register, data)
        except Exception as e:
            raise SensorReadError(f"I2C send failed to address 0x{self._address:02X}: {e}")

    def receive(self) -> list:
        if self._bus is None:
            raise SensorInitializationError("I2C bus is not initialized. Call initialize() first.")
        try:
            return self._bus.read_i2c_block_data(self._address, self._register, self._read_length)
        except Exception as e:
            raise SensorReadError(f"I2C receive failed from address 0x{self._address:02X}: {e}")

    def close(self) -> None:
        if self._bus is not None:
            self._bus.close()
            self._bus = None
