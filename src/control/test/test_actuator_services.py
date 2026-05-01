#!/usr/bin/env python3
import pytest
import math
from control.services.Steering import Steering
from control.services.DirEvaluator import DirEvaluator
from control.services.PWMMapper import PWMMapper

class TestLocomotionServices:
    
    def test_steering_happy_path(self):
        left, right = Steering.calculate_tank_drive(throttle=0.5, yaw=0.0)
        assert left == 0.5 and right == 0.5
        
        left, right = Steering.calculate_tank_drive(throttle=0.0, yaw=0.5)
        assert left == 0.5 and right == -0.5
        
    def test_steering_normalization(self):
        # Throttle + Yaw > 1.0 should normalize smoothly
        left, right = Steering.calculate_tank_drive(throttle=1.0, yaw=1.0)
        assert left == 1.0
        assert right == 0.0

    def test_steering_defensive_checks(self):
        with pytest.raises(ValueError):
            Steering.calculate_tank_drive(float('nan'), 0.5)
        with pytest.raises(TypeError):
            Steering.calculate_tank_drive("0.5", 0.5) # type: ignore

    def test_dir_evaluator_forward_and_backward(self):
        evaluator = DirEvaluator(deadzone=0.05)
        dir_f, brake_f, mag_f = evaluator.evaluate(0.5)
        assert dir_f == 1 and brake_f == 0 and mag_f == 0.5
        
        dir_b, brake_b, mag_b = evaluator.evaluate(-0.5)
        assert dir_b == 0 and brake_b == 0 and mag_b == 0.5
        
    def test_dir_evaluator_deadzone_brake(self):
        evaluator = DirEvaluator(deadzone=0.05)
        dir_s, brake_s, mag_s = evaluator.evaluate(0.02)
        assert brake_s == 1 and mag_s == 0.0

    def test_pwm_mapper(self):
        mapper = PWMMapper(max_pwm=255)
        assert mapper.map_magnitude(0.0) == 0
        assert mapper.map_magnitude(1.0) == 255
        assert mapper.map_magnitude(0.5) == 127