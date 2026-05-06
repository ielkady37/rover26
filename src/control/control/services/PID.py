import math
from typing import Optional

def _clamp(value: float, lower: float, upper: float) -> float:
    """Clamp *value* to [lower, upper]."""
    return max(lower, min(value, upper))

class PIDController:
    """
    A PID controller specifically engineered for heading (yaw) stabilization.
    Safely handles -180 to 180 degree cyclic wrap-around and rejects NaN/Inf faults.
    """

    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        
        self.setpoint: float = 0.0
        self._h_integral: float = 0.0
        self._h_prev_error: Optional[float] = None 

    def update_setpoint(self, setpoint: float) -> None:
        """Updates the target angle [-180, 180]."""
        if math.isnan(setpoint) or math.isinf(setpoint):
            return # Defensively ignore garbage data
        self.setpoint = setpoint

    def update_constants(self, kp: float, ki: float, kd: float) -> None:
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)

    def stabilize(self, measured_value: float, dt: float) -> float:
        """
        Compute PID output on the bounded angular error.
        
        Args:
            measured_value: Current IMU yaw in degrees.
            dt: Time elapsed since last call in seconds (prevents I/D drift if loop rate varies).
            
        Returns:
            float: Control effort clamped to [-1.0, 1.0]. Returns 0.0 if inputs are invalid.
        """
        if dt <= 0.0 or math.isnan(measured_value) or math.isinf(measured_value):
            return 0.0

        error = self._angle_difference(measured_value, self.setpoint)

        # Proportional
        p = self.kp * error

        # Integral with anti-windup
        self._h_integral += self.ki * error * dt
        self._h_integral = _clamp(self._h_integral, -1.0, 1.0)

        # Derivative
        if self._h_prev_error is None:
            d = 0.0
        else:
            d = self.kd * (error - self._h_prev_error) / dt
            
        self._h_prev_error = error

        output = _clamp(p + self._h_integral + d, -1.0, 1.0)
        return float(output)

    def _angle_difference(self, current: float, target: float) -> float:
        """Calculates the shortest signed difference between two cyclic angles."""
        diff = (target - current + 180) % 360 - 180
        return diff