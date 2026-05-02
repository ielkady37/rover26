#!/usr/bin/env python3
"""
Live SPI read tool — reads and parses SensorPacket frames from the ESP32.

Packet layout (22 bytes, little-endian, #pragma pack(1)):
    [0]      uint8  start      always 0xAA
    [1-4]    float  yaw        degrees
    [5-8]    float  pitch      degrees
    [9-12]   float  roll       degrees
    [13-16]  float  enc1       net revolutions
    [17-20]  float  enc2       net revolutions
    [21]     uint8  checksum   XOR of bytes [0..20]

Usage:
    python3 src/control/tools/spi_live_read.py [bus] [device] [speed_hz] [mode]

Defaults: bus=0 device=0 speed=1_000_000 mode=0
"""
import sys
import time
import struct

# Allow running without installing the package
sys.path.insert(0, "src/control")

from control.services.SPIService import SPIService

PACKET_START_BYTE = 0xAA
PACKET_SIZE       = 22          # must match ESP32 PACKET_SIZE
PACKET_FMT        = "<BfffffB"  # little-endian: start, yaw, pitch, roll, enc1, enc2, checksum

# ---------- config (override via CLI args) ----------
BUS      = int(sys.argv[1]) if len(sys.argv) > 1 else 0
DEVICE   = int(sys.argv[2]) if len(sys.argv) > 2 else 0
SPEED_HZ = int(sys.argv[3]) if len(sys.argv) > 3 else 1_000_000
MODE     = int(sys.argv[4]) if len(sys.argv) > 4 else 0
# ----------------------------------------------------


def verify_checksum(raw: bytes) -> bool:
    """XOR of bytes [0..20] must equal byte [21]."""
    expected = 0
    for b in raw[:-1]:
        expected ^= b
    return expected == raw[-1]


def main():
    print(f"Opening /dev/spidev{BUS}.{DEVICE}  speed={SPEED_HZ} Hz  mode={MODE}  "
          f"packet={PACKET_SIZE} bytes")
    svc = SPIService({"bus": BUS, "device": DEVICE, "max_speed_hz": SPEED_HZ, "mode": MODE})
    svc.initialize()
    print("SPI open — press Ctrl+C to stop\n")
    print(f"{'YAW':>10} {'PITCH':>10} {'ROLL':>10} {'ENC1':>10} {'ENC2':>10}  STATUS")
    print("-" * 65)

    try:
        while True:
            raw = bytes(svc._spi.xfer2([0x00] * PACKET_SIZE))

            if raw[0] != PACKET_START_BYTE:
                print(f"  [bad start byte: 0x{raw[0]:02X}]  raw: {raw.hex(' ').upper()}")
                time.sleep(0.1)
                continue

            if not verify_checksum(raw):
                expected = 0
                for b in raw[:-1]:
                    expected ^= b
                print(f"  [checksum fail: got 0x{raw[-1]:02X} expected 0x{expected:02X}]")
                time.sleep(0.1)
                continue

            _, yaw, pitch, roll, enc1, enc2, _ = struct.unpack(PACKET_FMT, raw)
            print(f"{yaw:>10.2f} {pitch:>10.2f} {roll:>10.2f} {enc1:>10.4f} {enc2:>10.4f}  OK")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        svc.close()


if __name__ == "__main__":
    main()
