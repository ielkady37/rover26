import math
from typing import Optional
from utils.Logger import RoverLogger

def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))

class PIDController:
    """
    A PID controller specifically engineered for heading (yaw) stabilization.
    """

    def __init__(self, kp: float, ki: float, kd: float):
        self._log = RoverLogger()
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        
        self.setpoint: float = 0.0
        self._h_integral: float = 0.0
        self._h_prev_error: Optional[float] = None 
        
        self._log.info(f"PIDController initialized (kp={self.kp}, ki={self.ki}, kd={self.kd})")

    def update_setpoint(self, setpoint: float) -> None:
        if math.isnan(setpoint) or math.isinf(setpoint):
            self._log.warn("PID rejected invalid setpoint update (NaN/Inf).")
            return
        self.setpoint = setpoint

    def update_constants(self, kp: float, ki: float, kd: float) -> None:
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self._log.succ(f"PID constants updated to (kp={self.kp}, ki={self.ki}, kd={self.kd})")

    def stabilize(self, measured_value: float, dt: float) -> float:
        if dt <= 0.0 or math.isnan(measured_value) or math.isinf(measured_value):
            return 0.0

        error = self._angle_difference(measured_value, self.setpoint)

        p = self.kp * error
        self._h_integral += self.ki * error * dt
        self._h_integral = _clamp(self._h_integral, -1.0, 1.0)

        if self._h_prev_error is None:
            d = 0.0
        else:
            d = self.kd * (error - self._h_prev_error) / dt
            
        self._h_prev_error = error

        output = _clamp(p + self._h_integral + d, -1.0, 1.0)
        return float(output)

    def _angle_difference(self, current: float, target: float) -> float:
        diff = (target - current + 180) % 360 - 180
        return diff