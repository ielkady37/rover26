#!/usr/bin/env python3
import math
from typing import Tuple

class DirEvaluator:
    """
    Evaluates direction and brake logic states based on signed velocities.
    """

    def __init__(self, deadzone: float = 0.0):
        """
        Args:
            deadzone: Velocities closer to 0 than this value trigger the brake.
        """
        self.deadzone = max(0.0, float(deadzone))

    def evaluate(self, velocity: float) -> Tuple[int, int, float]:
        """
        Converts signed velocity into DIR pin, BRAKE pin, and absolute MAGNITUDE.

        Args:
            velocity: Normalized signed velocity [-1.0, 1.0].

        Returns:
            Tuple[int, int, float]: (dir_state, brake_state, magnitude)
            dir_state: 1 for forward, 0 for backward.
            brake_state: 1 for active brake, 0 for release.
            magnitude: Absolute speed [0.0, 1.0].
            
        Raises:
            ValueError: If velocity is NaN or Inf.
        """
        if math.isnan(velocity) or math.isinf(velocity):
            raise ValueError("Velocity cannot be NaN or Inf.")

        vel_clamped = max(-1.0, min(1.0, float(velocity)))

        # Engage brake if velocity is within the deadzone
        if abs(vel_clamped) < self.deadzone:
            return 0, 1, 0.0

        dir_state = 1 if vel_clamped >= 0 else 0
        brake_state = 0
        magnitude = abs(vel_clamped)

        return dir_state, brake_state, magnitude