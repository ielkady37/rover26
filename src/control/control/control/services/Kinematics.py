import math
from typing import Tuple
from utils.Logger import RoverLogger

class Kinematics:
    """
    Differential drive kinematics solver.
    Translates Twist commands (m/s, rad/s) into normalized wheel speeds [-1.0, 1.0].
    Dynamically scaled based on Nav2 configuration parameters.
    """

    def __init__(self, track_width: float, max_linear_vel: float, max_angular_vel: float):
        self._log = RoverLogger()
        
        if track_width <= 0.0 or max_linear_vel <= 0.0 or max_angular_vel <= 0.0:
            self._log.err("Kinematics parameters must be strictly positive.")
            raise ValueError("Kinematics parameters must be strictly positive.")
            
        self.track_width = float(track_width)
        self.max_linear_vel = float(max_linear_vel)
        self.max_angular_vel = float(max_angular_vel)

        self.max_wheel_speed = self.max_linear_vel + (self.max_angular_vel * self.track_width / 2.0)
        self._log.succ(f"Kinematics initialized. Max theoretical wheel speed: {self.max_wheel_speed:.2f} m/s")

    def calculate_wheel_speeds(self, linear_x: float, angular_z: float) -> Tuple[float, float]:
        if math.isnan(linear_x) or math.isnan(angular_z) or math.isinf(linear_x) or math.isinf(angular_z):
            self._log.err(f"Invalid Twist velocities received: linear={linear_x}, angular={angular_z}")
            raise ValueError("Velocity commands cannot be NaN or Inf.")

        v_left = linear_x - (angular_z * self.track_width / 2.0)
        v_right = linear_x + (angular_z * self.track_width / 2.0)

        left_norm = v_left / self.max_wheel_speed
        right_norm = v_right / self.max_wheel_speed

        left_norm = max(-1.0, min(1.0, left_norm))
        right_norm = max(-1.0, min(1.0, right_norm))

        return float(left_norm), float(right_norm)