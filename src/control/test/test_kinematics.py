import pytest
import math
from control.services.Kinematics import Kinematics

class TestKinematics:

    def setup_method(self):
        # A rover 0.5 meters wide, with a top speed of 2.0 m/s
        self.kinematics = Kinematics(track_width_meters=0.5, max_linear_speed_mps=2.0)

    def test_initialization_defensive_checks(self):
        with pytest.raises(ValueError):
            Kinematics(track_width_meters=-0.5, max_linear_speed_mps=2.0)
        with pytest.raises(ValueError):
            Kinematics(track_width_meters=0.5, max_linear_speed_mps=0.0)

    def test_pure_forward_motion(self):
        # Command: 1.0 m/s forward (half max speed)
        left, right = self.kinematics.calculate_wheel_speeds(linear_x=1.0, angular_z=0.0)
        assert left == 0.5 and right == 0.5

    def test_pure_rotation(self):
        # Command: 0 m/s forward, rotate left at 1.0 rad/s
        # v_left = 0 - (1.0 * 0.25) = -0.25 m/s. Normalized = -0.125
        # v_right = 0 + (1.0 * 0.25) = 0.25 m/s. Normalized = 0.125
        left, right = self.kinematics.calculate_wheel_speeds(linear_x=0.0, angular_z=1.0)
        assert left == -0.125 and right == 0.125

    def test_overspeed_normalization(self):
        # Command: 3.0 m/s forward (exceeds max speed of 2.0 m/s)
        # Should smoothly clamp to 1.0 without throwing an error
        left, right = self.kinematics.calculate_wheel_speeds(linear_x=3.0, angular_z=0.0)
        assert left == 1.0 and right == 1.0

    def test_overspeed_turn_preservation(self):
        # Command: Drive at 2.0 m/s AND turn left aggressively.
        # This exceeds the physical limits of the outer wheel.
        left, right = self.kinematics.calculate_wheel_speeds(linear_x=2.0, angular_z=4.0)
        # Math: v_left = 2.0 - 1.0 = 1.0 m/s (norm = 0.5)
        #       v_right = 2.0 + 1.0 = 3.0 m/s (norm = 1.5) -> Over limit!
        # Scaling by 1.5: left = 0.5/1.5 = 0.333, right = 1.5/1.5 = 1.0
        assert math.isclose(left, 0.333, rel_tol=1e-3)
        assert right == 1.0

    def test_defensive_execution(self):
        with pytest.raises(ValueError):
            self.kinematics.calculate_wheel_speeds(float('nan'), 0.0)
        with pytest.raises(ValueError):
            self.kinematics.calculate_wheel_speeds(1.0, float('inf'))
        with pytest.raises(TypeError):
            self.kinematics.calculate_wheel_speeds("1.0", 0.0) # type: ignore