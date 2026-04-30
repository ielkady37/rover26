#!/usr/bin/env python3
class SensorReadError(Exception):
    """Raised when a sensor fails to read data"""
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)