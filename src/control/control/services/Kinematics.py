import math
from typing import Tuple

class Kinematics:
    """
    Differential drive kinematics solver.
    Translates Twist commands (m/s, rad/s) into normalized wheel speeds [-1.0, 1.0].
    """

    def __init__(self, track_width_meters: float, max_linear_speed_mps: float):
        """
        Args:
            track_width_meters: The distance between the left and right wheels in meters.
            max_linear_speed_mps: The physical maximum speed the rover can achieve in m/s.
                                  Used to normalize the output to the [-1.0, 1.0] range.
        Raises:
            ValueError: If physical constraints are zero or negative.
        """
        if track_width_meters <= 0.0 or max_linear_speed_mps <= 0.0:
            raise ValueError("Track width and max linear speed must be strictly positive.")
            
        self.track_width = float(track_width_meters)
        self.max_linear_speed = float(max_linear_speed_mps)

    def calculate_wheel_speeds(self, linear_x: float, angular_z: float) -> Tuple[float, float]:
        """
        Converts target rover velocities into normalized left and right wheel commands.

        Args:
            linear_x: Target forward velocity in meters/second.
            angular_z: Target rotational velocity in radians/second (positive = counter-clockwise/left turn).

        Returns:
            Tuple[float, float]: (left_speed, right_speed) normalized strictly to [-1.0, 1.0].
            
        Raises:
            TypeError: If inputs are not numeric.
            ValueError: If inputs are NaN or Infinity.
        """
        if not isinstance(linear_x, (int, float)) or not isinstance(angular_z, (int, float)):
            raise TypeError("Velocity commands must be numeric.")
            
        if math.isnan(linear_x) or math.isnan(angular_z) or math.isinf(linear_x) or math.isinf(angular_z):
            raise ValueError("Velocity commands cannot be NaN or Inf.")

        # 1. Standard Differential Drive Equations (m/s)
        # For a standard ROS frame, +angular_z turns left. 
        # Turning left means the right wheel spins faster than the left wheel.
        v_left = linear_x - (angular_z * self.track_width / 2.0)
        v_right = linear_x + (angular_z * self.track_width / 2.0)

        # 2. Normalize to [-1.0, 1.0] based on the rover's theoretical top speed
        left_norm = v_left / self.max_linear_speed
        right_norm = v_right / self.max_linear_speed

        # 3. Preserve the turn ratio if bounds are exceeded
        # If the local planner commands a combined move that exceeds hardware limits,
        # we scale both wheels down equally so the rover follows the exact same curve, just slower.
        max_magnitude = max(abs(left_norm), abs(right_norm))
        if max_magnitude > 1.0:
            left_norm /= max_magnitude
            right_norm /= max_magnitude
    
        return float(left_norm), float(right_norm)