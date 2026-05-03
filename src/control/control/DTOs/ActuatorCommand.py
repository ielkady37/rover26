#!/usr/bin/env python3
from dataclasses import dataclass, field


@dataclass
class MotorCommand:
    dir: int    # 0 = forward, 1 = reverse
    brake: int  # 0 = off,     1 = on
    speed: float  # 0.0 – 1.0 duty cycle


@dataclass
class ActuatorCommand:
    motor1: MotorCommand = field(default_factory=lambda: MotorCommand(0, 0, 0.0))
    motor2: MotorCommand = field(default_factory=lambda: MotorCommand(0, 0, 0.0))
    laser: int = 0        # 0 = off, 1 = on
    servo: float = 0.0    # degrees
