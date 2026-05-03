#!/usr/bin/env python3
import math

class PWMMapper:
    """
    Maps normalized speed magnitudes to hardware-specific PWM integer bounds.
    """

    def __init__(self, max_pwm: int = 255):
        """
        Args:
            max_pwm: The maximum integer value accepted by the motor driver 
                     (e.g., 255 for 8-bit timers, 65535 for 16-bit).
        """
        if not isinstance(max_pwm, int) or max_pwm <= 0:
            raise ValueError("max_pwm must be a positive integer.")
        self.max_pwm = max_pwm

    def map_magnitude(self, magnitude: float) -> int:
        """
        Converts a normalized magnitude into a valid PWM duty cycle.

        Args:
            magnitude: Absolute speed [0.0, 1.0].

        Returns:
            int: PWM value from 0 to max_pwm.
            
        Raises:
            ValueError: If magnitude is NaN or Inf.
        """
        if math.isnan(magnitude) or math.isinf(magnitude):
            raise ValueError("Magnitude cannot be NaN or Inf.")
            
        mag_clamped = max(0.0, min(1.0, float(magnitude)))
        return int(mag_clamped * self.max_pwm)