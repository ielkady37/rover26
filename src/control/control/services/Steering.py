#!/usr/bin/env python3
import math
from typing import Tuple

class Steering:
    """
    Handles differential drive (skid-steer) kinematics mapping.
    Converts joystick/autonomous axes into separate left and right wheel speeds.
    """

    @staticmethod
    def calculate_tank_drive(throttle: float, yaw: float) -> Tuple[float, float]:
        """
        Mixes forward/backward throttle and left/right yaw into wheel speeds.

        Args:
            throttle: Forward/backward axis [-1.0, 1.0]. Positive is forward.
            yaw: Left/right rotational axis [-1.0, 1.0]. Positive is right turn.

        Returns:
            Tuple[float, float]: (left_speed, right_speed) normalized to [-1.0, 1.0].

        Raises:
            TypeError: If inputs are not numeric.
            ValueError: If inputs are NaN or Infinity.
        """
        if not isinstance(throttle, (int, float)) or not isinstance(yaw, (int, float)):
            raise TypeError("Steering inputs must be numeric.")
        
        if math.isnan(throttle) or math.isnan(yaw) or math.isinf(throttle) or math.isinf(yaw):
            raise ValueError("Steering inputs cannot be NaN or Inf.")

        # Clamp inputs defensively
        throttle_c = max(-1.0, min(1.0, float(throttle)))
        yaw_c = max(-1.0, min(1.0, float(yaw)))

        # Standard differential mixing
        left_speed = throttle_c + yaw_c
        right_speed = throttle_c - yaw_c

        # Normalize to keep within [-1.0, 1.0] while preserving the turn ratio
        max_magnitude = max(abs(left_speed), abs(right_speed))
        if max_magnitude > 1.0:
            left_speed /= max_magnitude
            right_speed /= max_magnitude

        return float(left_speed), float(right_speed)