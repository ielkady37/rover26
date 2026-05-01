#!/usr/bin/env python3
from dataclasses import dataclass

@dataclass
class MotorCommandDTO:
    """
    Data Transfer Object holding the evaluated hardware states for the drivetrain.
    """
    left_pwm: int
    left_dir: int
    left_brake: int
    right_pwm: int
    right_dir: int
    right_brake: int