#!/usr/bin/env python3
"""
ESPDataController — owns both ESP32 links in a single class.

════════════════════════════════════════════════════════════
SENSOR ESP32  (SPI)  — Pi ← ESP32  read-only
────────────────────────────────────────────────────────────
ESP32 → Pi communication sequence:
1. Pi reads 203 bytes → ContractPacket  (start=0xAB)
2. Pi reads  62 bytes → SensorPacket    (start=0xAA), repeated forever

ContractPacket layout (202 bytes, little-endian, #pragma pack(1)):
    [0]      uint8   start              0xAB
    [1]      uint8   field_count        15
    [2-3]    uint16  data_packet_size   bytes per sensor packet (62)
    [4-5]    uint16  ticks_per_rev      encoder ticks per full motor revolution
    [6-200]  15 × ContractField:
                 char[12]  name         null-padded ASCII
                 uint8     bit_width    32 = IEEE 754 float
    [201]    uint8   checksum           XOR of bytes [0..200]

SensorPacket layout (62 bytes, little-endian, #pragma pack(1)):
    [0]      uint8   start              0xAA
    [1-4]    float   yaw                degrees
    [5-8]    float   pitch              degrees
    [9-12]   float   roll               degrees
    [13-16]  float   qx                 quaternion x
    [17-20]  float   qy                 quaternion y
    [21-24]  float   qz                 quaternion z
    [25-28]  float   qw                 quaternion w
    [29-32]  float   gx                 gyroscope rad/s
    [33-36]  float   gy
    [37-40]  float   gz
    [41-44]  float   ax                 linear acceleration m/s²
    [45-48]  float   ay
    [49-52]  float   az
    [53-56]  float   enc1NetRev         motor 1 net revolutions
    [57-60]  float   enc2NetRev         motor 2 net revolutions
    [61]     uint8   checksum           XOR of bytes [0..60]

ACTUATOR ESP32  (I2C)  — Pi → ESP32  write-only
────────────────────────────────────────────────────────────
Pi → ESP32 communication sequence:
1. Pi writes 109 bytes → ContractPacket  (start=0xBC)
2. Pi writes  19 bytes → ActuatorPacket  (start=0xBB), repeated on demand

ContractPacket layout (109 bytes, little-endian, #pragma pack(1)):
    [0]      uint8   start              0xBC
    [1]      uint8   field_count        8
    [2-3]    uint16  data_packet_size   19
    [4-107]  8 × ContractField:
                 char[12]  name         null-padded ASCII
                 uint8     bit_width    8=uint8 / 32=float
    [108]    uint8   checksum           XOR of bytes [0..107]

ActuatorPacket layout (19 bytes, little-endian, #pragma pack(1)):
    [0]      uint8   start              0xBB
    [1]      uint8   m1_dir             0=forward  1=reverse
    [2]      uint8   m1_brake           0=off      1=on
    [3-6]    float   m1_speed           0.0–1.0
    [7]      uint8   m2_dir
    [8]      uint8   m2_brake
    [9-12]   float   m2_speed
    [13]     uint8   laser              0=off  1=on
    [14-17]  float   servo              degrees
    [18]     uint8   checksum           XOR of bytes [0..17]
"""
import struct
import time
import smbus2
from control.services.SPIService import SPIService
from control.services.I2CService import I2CService
from control.DTOs.SensorData import SensorData
from control.DTOs.ActuatorCommand import ActuatorCommand
from control.exceptions.SensorInitializationError import SensorInitializationError
from control.exceptions.SensorReadError import SensorReadError

# Contract constants
CONTRACT_START_BYTE      = 0xAB
CONTRACT_FIELD_NAME_LEN  = 12
CONTRACT_FIELD_COUNT     = 15
CONTRACT_PACKET_SIZE     = 203 
# Data-packet constants
PACKET_START_BYTE = 0xAA
PACKET_SIZE       = 62         

# Struct formats (little-endian, packed)
_CONTRACT_HEADER_FMT = "<BBHHB"                      # start, field_count, data_packet_size, ticks_per_rev, packet_start_byte
_CONTRACT_FIELD_FMT  = f"<{CONTRACT_FIELD_NAME_LEN}sB"  # name[12], bit_width
_DATA_PACKET_FMT     = "<B15fB"                      # start, 15 floats, checksum

# Actuator contract / packet constants
_ACT_CONTRACT_START      = 0xBC
_ACT_PACKET_START        = 0xBB
_ACT_FIELD_NAME_LEN      = 12
_ACT_CONTRACT_SIZE       = 109
_ACT_PACKET_SIZE         = 19
_ACT_CONTRACT_HEADER_FMT = "<BBH"                          # start, field_count, data_packet_size
_ACT_FIELD_FMT           = f"<{_ACT_FIELD_NAME_LEN}sB"    # name[12], bit_width

_ACT_CONTRACT_FIELDS: list[tuple[str, int]] = [
    ("m1_dir",   8),
    ("m1_brake", 8),
    ("m1_speed", 32),
    ("m2_dir",   8),
    ("m2_brake", 8),
    ("m2_speed", 32),
    ("laser",    8),
    ("servo",    32),
]


def _xor_checksum(data: bytes) -> int:
    cs = 0
    for b in data:
        cs ^= b
    return cs


class ESPDataController:
    """Owns both ESP32 links: SPI sensor reads and I2C actuator writes.

    Usage
    ─────
        ctrl = ESPDataController(
            spi_comm={"bus": 0, "device": 0},
            i2c_comm={"bus": 1, "address": 0x10},
        )
        ctrl.initialize()               # opens SPI + I2C, exchanges contracts
        data: SensorData = ctrl.read()  # call repeatedly in reader thread
        ctrl.write(ActuatorCommand())   # call on /esp_tx messages
        ctrl.close()
    """

    def __init__(self, spi_comm: dict, i2c_comm: dict) -> None:
        self._spi = SPIService(spi_comm)
        self._i2c = I2CService(i2c_comm)
        self._packet_size: int = PACKET_SIZE
        self._packet_start_byte: int = PACKET_START_BYTE
        self._fields: list[str] = []
        self._ticks_per_rev: int = 0

    # public API

    def initialize(self) -> None:
        """Open SPI + I2C and exchange the one-time contracts with both ESP32s."""
        self._spi.initialize()
        self._read_contract()
        self._i2c.initialize()
        self._send_act_contract()

    def read(self) -> SensorData:
        """Read and parse one sensor packet.

        Each call issues exactly one aligned xfer2 (one CS pulse = one ESP32
        transaction).  With the ESP32's double-buffered slave, a fresh 62-byte
        packet is always pre-queued.  If the start byte or checksum is wrong
        this tick is simply skipped — the caller retries on the next timer
        callback without issuing any extra CS pulses.
        """
        raw = bytes(self._spi._spi.xfer2([0x00] * self._packet_size))
        return self._parse_data_packet(raw)

    def write(self, cmd: ActuatorCommand) -> None:
        """Build and send one ActuatorPacket to the actuator ESP32 via I2C."""
        if self._i2c._bus is None:
            raise SensorInitializationError(
                "I2C bus not initialized. Call initialize() first."
            )
        body = struct.pack(
            "<BBBfBBfBf",
            _ACT_PACKET_START,
            int(cmd.motor1.dir)   & 0xFF,
            int(cmd.motor1.brake) & 0xFF,
            float(cmd.motor1.speed),
            int(cmd.motor2.dir)   & 0xFF,
            int(cmd.motor2.brake) & 0xFF,
            float(cmd.motor2.speed),
            int(cmd.laser)        & 0xFF,
            float(cmd.servo),
        )
        packet = body + bytes([_xor_checksum(body)])
        try:
            msg = smbus2.i2c_msg.write(self._i2c._address, list(packet))
            self._i2c._bus.i2c_rdwr(msg)
        except Exception as e:
            raise SensorReadError(f"I2C actuator write failed: {e}")

    def close(self) -> None:
        self._spi.close()
        self._i2c.close()

    @property
    def ticks_per_rev(self) -> int:
        """Encoder ticks per full motor revolution (from contract)."""
        return self._ticks_per_rev

    @property
    def fields(self) -> list[str]:
        """Ordered list of field names received in the contract."""
        return list(self._fields)

    # internal

    def _read_contract(self, max_attempts: int = 15, retry_delay: float = 0.3) -> None:
        """Read the one-time contract packet from the ESP32.

        Three scenarios are handled:
          A) Pi starts first  — raw[0] == 0xAB → valid contract, parse it.
          B) ESP32 was first  — raw[0] == 0xAA → contract was already sent.
               The 202-byte Pi transfer consumed exactly ONE 62-byte ESP32
               transaction (MISO holds its last bit for bytes 62-201).
               Transaction alignment is preserved; use hardcoded defaults.
          C) ESP32 not ready  — raw[0] == 0x00 or other garbage → retry.
               After max_attempts exhausted, fall back to defaults so the
               node never crashes on startup.
        """
        for attempt in range(max_attempts):
            raw = bytes(self._spi._spi.xfer2([0x00] * CONTRACT_PACKET_SIZE))

            if raw[0] == CONTRACT_START_BYTE:
                if _xor_checksum(raw[:-1]) != raw[-1]:
                    # Checksum mismatch — line noise during contract, retry
                    time.sleep(retry_delay)
                    continue
                self._parse_contract(raw)
                return

            if raw[0] == PACKET_START_BYTE:
                # Contract was already sent before we connected.
                # 202-byte transfer consumed one 62-byte ESP32 transaction;
                # MISO held the last MISO bit for bytes 62-201.
                # The next 62-byte read() call will get a fresh, aligned packet.
                self._apply_defaults()
                return

            # 0x00 or any other byte: ESP32 SPI slave not ready yet → wait and retry
            time.sleep(retry_delay)

        # Exhausted all retries — run with defaults so the node starts anyway
        self._apply_defaults()

    def _apply_defaults(self) -> None:
        """Populate fields with the hardcoded defaults matching Connection.h."""
        self._packet_size        = PACKET_SIZE
        self._packet_start_byte  = PACKET_START_BYTE
        self._ticks_per_rev      = 0
        self._fields = [
            "yaw", "pitch", "roll",
            "qx", "qy", "qz", "qw",
            "gx", "gy", "gz",
            "ax", "ay", "az",
            "enc1_rev", "enc2_rev",
        ]

    def _parse_contract(self, raw: bytes) -> None:
        """Extract and store all fields from a validated ContractPacket."""
        header_size = struct.calcsize(_CONTRACT_HEADER_FMT)
        _, field_count, data_packet_size, ticks_per_rev, packet_start_byte = struct.unpack_from(
            _CONTRACT_HEADER_FMT, raw, 0
        )

        field_size = struct.calcsize(_CONTRACT_FIELD_FMT)
        fields = []
        offset = header_size
        for _ in range(field_count):
            name_raw, _bit_width = struct.unpack_from(_CONTRACT_FIELD_FMT, raw, offset)
            name = name_raw.rstrip(b"\x00").decode("ascii")
            fields.append(name)
            offset += field_size

        self._packet_size        = data_packet_size
        self._packet_start_byte  = packet_start_byte
        self._ticks_per_rev      = ticks_per_rev
        self._fields             = fields

    def _parse_data_packet(self, raw: bytes) -> SensorData:
        if len(raw) != self._packet_size:
            raise SensorReadError(
                f"Expected {self._packet_size} bytes, got {len(raw)}"
            )

        if raw[0] != self._packet_start_byte:
            raise SensorReadError(
                f"Expected data start byte 0x{self._packet_start_byte:02X}, "
                f"got 0x{raw[0]:02X}"
            )

        if _xor_checksum(raw[:-1]) != raw[-1]:
            expected = _xor_checksum(raw[:-1])
            raise SensorReadError(
                f"Data packet checksum failed: expected 0x{expected:02X}, "
                f"got 0x{raw[-1]:02X}"
            )

        _, yaw, pitch, roll, qx, qy, qz, qw, gx, gy, gz, ax, ay, az, enc1, enc2, _ = \
            struct.unpack(_DATA_PACKET_FMT, raw)

        return SensorData(
            yaw=yaw, pitch=pitch, roll=roll,
            qx=qx, qy=qy, qz=qz, qw=qw,
            gx=gx, gy=gy, gz=gz,
            ax=ax, ay=ay, az=az,
            enc1_net_rev=enc1,
            enc2_net_rev=enc2,
        )

    def _send_act_contract(self) -> None:
        """Send the one-time 109-byte ContractPacket to the actuator ESP32."""
        if self._i2c._bus is None:
            raise SensorInitializationError(
                "I2C bus not initialized. Call initialize() first."
            )
        header = struct.pack(
            _ACT_CONTRACT_HEADER_FMT,
            _ACT_CONTRACT_START,
            len(_ACT_CONTRACT_FIELDS),
            _ACT_PACKET_SIZE,
        )
        fields_bytes = b""
        for name, bit_width in _ACT_CONTRACT_FIELDS:
            name_padded = name.encode("ascii").ljust(_ACT_FIELD_NAME_LEN, b"\x00")
            fields_bytes += struct.pack(_ACT_FIELD_FMT, name_padded, bit_width)
        payload  = header + fields_bytes
        contract = payload + bytes([_xor_checksum(payload)])
        try:
            msg = smbus2.i2c_msg.write(self._i2c._address, list(contract))
            self._i2c._bus.i2c_rdwr(msg)
        except Exception as e:
            raise SensorInitializationError(
                f"Failed to send actuator contract to 0x{self._i2c._address:02X}: {e}"
            )
