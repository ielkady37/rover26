#!/usr/bin/env python3
class SensorCalibrationError(Exception):
    """Raised for errors in the sensor calibration process."""
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)