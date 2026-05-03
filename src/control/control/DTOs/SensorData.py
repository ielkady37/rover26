#!/usr/bin/env python3
from dataclasses import dataclass


@dataclass
class SensorData:
    # Euler angles (degrees)
    yaw: float
    pitch: float
    roll: float
    # Orientation quaternion
    qx: float
    qy: float
    qz: float
    qw: float
    # Gyroscope (rad/s)
    gx: float
    gy: float
    gz: float
    # Linear acceleration (m/s²)
    ax: float
    ay: float
    az: float
    # Encoder net revolutions
    enc1_net_rev: float
    enc2_net_rev: float
