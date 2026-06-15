#!/usr/bin/env python3
from zope.interface import Interface, Attribute

class iCommunicationProtocol(Interface):
    data_contract = Attribute("Start byte that identifies a contract packet for this transport")

    def initialize(self) -> None:
        """Initialize the communication protocol"""

    def send(self, data: bytes) -> None:
        """Append checksum and transmit data through the protocol"""

    def receive(self, length: int) -> bytes:
        """Read *length* bytes, validate checksum, and return the raw packet"""

    def close(self) -> None:
        """Close the communication protocol"""

    def validateData(self, data: bytes) -> None:
        """Validate XOR checksum of a full packet; raises SensorReadError on mismatch"""

    def compute_checksum(data: bytes) -> int:
        """Compute XOR checksum over all bytes in data"""