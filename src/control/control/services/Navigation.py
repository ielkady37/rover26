#!/usr/bin/env python3
from typing import Optional
from control.services.Steering import Steering
from control.services.DirEvaluator import DirEvaluator
from control.services.PWMMapper import PWMMapper
from control.services.PID import PIDController
from control.DTOs.MotorCommandDTO import MotorCommandDTO

class Navigation:
    """
    Facade service orchestrating locomotion logic.
    Integrates PID stabilization to maintain straight-line driving.
    """

    def __init__(self, deadzone: float = 0.0, max_pwm: int = 255, kp: float = 0.0, ki: float = 0.0, kd: float = 0.0):
        self.dir_evaluator = DirEvaluator(deadzone=deadzone)
        self.pwm_mapper = PWMMapper(max_pwm=max_pwm)
        self.pid = PIDController(kp=kp, ki=ki, kd=kd)
        self.deadzone = deadzone

    def calculate_motor_commands(self, throttle_axis: float, yaw_axis: float, 
                                 current_yaw: float, dt: float) -> Optional[MotorCommandDTO]:
        """
        Processes axes and applies PID correction if driving straight.
        """
        try:
            active_yaw_command = yaw_axis

            # If driver is manually turning, bypass PID and sync setpoint.
            if abs(yaw_axis) > self.deadzone:
                self.pid.update_setpoint(current_yaw)
            
            # If driver is only pushing throttle, let PID maintain the heading.
            elif abs(throttle_axis) > self.deadzone:
                active_yaw_command = self.pid.stabilize(measured_value=current_yaw, dt=dt)
                
            # If idle (neither throttle nor yaw), just sync setpoint to avoid jumping when resuming
            else:
                self.pid.update_setpoint(current_yaw)

            # 1. Kinematics (Tank Drive Mixing) with the corrected yaw
            left_speed, right_speed = Steering.calculate_tank_drive(throttle=throttle_axis, yaw=active_yaw_command)

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