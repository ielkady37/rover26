#!/usr/bin/env python3
from typing import Optional
from control.services.Steering import Steering
from control.services.DirEvaluator import DirEvaluator
from control.services.PWMMapper import PWMMapper
from control.DTOs.MotorCommandDTO import MotorCommandDTO

class Navigation:
    """
    Unified Facade service for locomotion.
    Takes normalized forward and rotational efforts [-1.0, 1.0] and maps 
    them to hardware-ready MotorCommandDTOs. Agnostic to whether the 
    input came from a joystick, PID, or autonomous kinematics.
    """

    def __init__(self, deadzone: float = 0.0, max_pwm: int = 255):
        self.dir_evaluator = DirEvaluator(deadzone=deadzone)
        self.pwm_mapper = PWMMapper(max_pwm=max_pwm)

    def calculate_motor_commands(self, linear_effort: float, angular_effort: float) -> Optional[MotorCommandDTO]:
        """
        Processes unified efforts into hardware bounds.

        Args:
            linear_effort: Forward/backward drive [-1.0, 1.0].
            angular_effort: Rotational drive [-1.0, 1.0].

        Returns:
            MotorCommandDTO if successful, None if math validation fails.
        """
        try:
            # 1. Kinematics (Tank Drive Mixing)
            left_speed, right_speed = Steering.calculate_tank_drive(throttle=linear_effort, yaw=angular_effort)

            # 2. Evaluate Direction and Brake
            l_dir, l_brake, l_mag = self.dir_evaluator.evaluate(left_speed)
            r_dir, r_brake, r_mag = self.dir_evaluator.evaluate(right_speed)

            # 3. Map to Hardware PWM
            l_pwm = self.pwm_mapper.map_magnitude(l_mag)
            r_pwm = self.pwm_mapper.map_magnitude(r_mag)

            return MotorCommandDTO(
                left_pwm=l_pwm, left_dir=l_dir, left_brake=l_brake,
                right_pwm=r_pwm, right_dir=r_dir, right_brake=r_brake
            )
        except (ValueError, TypeError):
            return None