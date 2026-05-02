#!/usr/bin/env python3
from zope.interface import Interface

class iCommunicationProtocol(Interface):
    def __init__(self, comm_details: dict) -> None:
        """Setup the requirements to handle the communication
        """
    
    def initialize(self) -> None:
        """Initialize the communication protocol
        """
    
    def send(self, data: list) -> None:
        """Send the data through the communication protocol to the client
        """

    def receive(self) -> list:
        """Receive the data through the communication protocol from the client
        """
    def close(self) -> None:
        """Close the communication protocol
        """