#!/usr/bin/env python3
from typing import Optional
from control.services.Steering import Steering
from control.services.DirEvaluator import DirEvaluator
from control.services.PWMMapper import PWMMapper
from control.services.ExponentialSmoothing import ExponentialSmoothing
from control.DTOs.MotorCommandDTO import MotorCommandDTO
from utils.utils.Logger import RoverLogger

class Navigation:
    """
    Unified Facade service for locomotion.
    Takes normalized target efforts, applies hardware-level exponential smoothing 
    to the WHEELS, and maps them to hardware-ready MotorCommandDTOs.
    """

    def __init__(self, deadzone: float = 0.0, max_pwm: int = 255, alpha: float = 0.05, tolerance: float = 0.01):
        self._log = RoverLogger()
        self.dir_evaluator = DirEvaluator(deadzone=deadzone)
        self.pwm_mapper = PWMMapper(max_pwm=max_pwm)
        self.smoother = ExponentialSmoothing(alpha=alpha)
        
        # Facade maintains physical momentum state of the WHEELS, not the joystick
        self._current_left = 0.0
        self._current_right = 0.0
        self.tolerance = tolerance
        
        self._log.info(f"Navigation facade active (deadzone={deadzone}, max_pwm={max_pwm}, alpha={alpha})")

    def calculate_motor_commands(self, target_linear: float, target_angular: float) -> Optional[MotorCommandDTO]:
        """Used by Manual Navigation. Mixes joystick inputs, THEN smooths."""
        try:
            # 1. Kinematics (Tank Drive Mixing) using RAW abstract inputs
            target_left, target_right = Steering.calculate_tank_drive(throttle=target_linear, yaw=target_angular)
            
            # 2. Pass to wheel speed calculator for smoothing and mapping
            return self.calculate_from_wheel_speeds(target_left, target_right)
        except (ValueError, TypeError) as e:
            self._log.warn(f"Navigation facade mathematical fault during mixing: {e}")
            return None

    def calculate_from_wheel_speeds(self, target_left: float, target_right: float) -> Optional[MotorCommandDTO]:
        """Used by Autonomous Navigation. Smooths physical wheel targets."""
        try:
            # 1. Apply Hardware-Level Smoothing (Protect the ESCs)
            self._current_left = self.smoother.smooth(self._current_left, target_left, self.tolerance)
            self._current_right = self.smoother.smooth(self._current_right, target_right, self.tolerance)

            # 2. Evaluate Direction and Brake using SMOOTHED physical values
            l_dir, l_brake, l_mag = self.dir_evaluator.evaluate(self._current_left)
            r_dir, r_brake, r_mag = self.dir_evaluator.evaluate(self._current_right)

            # 3. Map to Hardware PWM
            l_pwm = self.pwm_mapper.map_magnitude(l_mag)
            r_pwm = self.pwm_mapper.map_magnitude(r_mag)

            return MotorCommandDTO(
                left_pwm=l_pwm, left_dir=l_dir, left_brake=l_brake,
                right_pwm=r_pwm, right_dir=r_dir, right_brake=r_brake
            )
        except (ValueError, TypeError) as e:
            self._log.warn(f"Navigation facade mathematical fault during mapping: {e}")
            return None
            
    def reset_momentum(self):
        """Emergency stop helper to clear residual momentum."""
        self._current_left = 0.0
        self._current_right = 0.0