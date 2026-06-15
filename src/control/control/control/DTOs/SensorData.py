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
    # GPS (BE-880)
    gps_lat: float = 0.0   # latitude  degrees (+N / -S)
    gps_lon: float = 0.0   # longitude degrees (+E / -W)
    gps_alt: float = 0.0   # altitude  metres above sea level
    gps_fix: float = 0.0   # 0.0 = no fix,  1.0 = GPS fix
    gps_sats: float = 0.0  # number of tracked satellites
