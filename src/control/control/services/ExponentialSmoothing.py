#!/usr/bin/env python3
from utils.Logger import RoverLogger

class ExponentialSmoothing:
    """
    Applies a low-pass exponential filter to ramp values up or down smoothly.
    Prevents violent jerks from digital buttons or sudden autonomous commands.
    """
    def __init__(self, alpha: float = 0.05):
        self._log = RoverLogger()
        if not (0.0 < alpha <= 1.0):
            self._log.err(f"Alpha {alpha} must be between 0 and 1. Defaulting to 0.05")
            self.alpha = 0.05
        else:
            self.alpha = float(alpha)

    def smooth(self, current_value: float, target_value: float, tolerance: float = 0.01) -> float:
        """
        Calculates the next step in the smoothed curve.
        """
        smoothed_value = (1.0 - self.alpha) * current_value + self.alpha * target_value

        # Snap to target if we are close enough to avoid floating-point micro-adjustments
        if abs(smoothed_value - target_value) <= tolerance:
            return float(target_value)

        return float(smoothed_value)