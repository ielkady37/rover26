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
from control.interface.iCommunicationProtocol import iCommunicationProtocol
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


class ESPDataController:
    """Single-channel ESP32 link controller.

    Instantiate once per transport; ESPBridgeNode creates two instances:

    Usage
    ─────
        from control.services.SPIService import SPIService
        from control.services.I2CService import I2CService

        spi_ctrl = ESPDataController(SPIService({"bus": 0, "device": 0}))
        i2c_ctrl = ESPDataController(I2CService({"bus": 1, "address": 0x10}))

        spi_ctrl.initialize()    # open SPI
        spi_ctrl.read_contract() # read one-time contract from sensor ESP32
        i2c_ctrl.initialize()    # open I2C
        i2c_ctrl.send_contract() # send one-time contract to actuator ESP32

        data = spi_ctrl.receive()          # call repeatedly in reader thread
        i2c_ctrl.write(ActuatorCommand())  # call on /esp_tx messages
    """

    def __init__(self, comm: iCommunicationProtocol) -> None:
        self._comm = comm
        self._packet_size: int = PACKET_SIZE
        self._packet_start_byte: int = PACKET_START_BYTE
        self._fields: list[str] = []
        self._ticks_per_rev: int = 0

    # public API

    def initialize(self) -> None:
        """Open the underlying transport."""
        self._comm.initialize()

    def initialize_with_retry(
        self,
        max_attempts: int = 10,
        retry_delay: float = 1.0,
        on_retry=None,
        on_success=None,
    ) -> None:
        """Open the transport with retry on failure.

        Args:
            max_attempts: Total number of attempts before giving up.
            retry_delay:  Seconds to wait between attempts.
            on_retry:     Optional callable(attempt, max_attempts, exc) called
                          before each retry (e.g. node.get_logger().warn).
            on_success:   Optional callable(attempt, max_attempts) called once
                          on success.

        Raises:
            SensorInitializationError: If all attempts are exhausted.
        """
        for attempt in range(1, max_attempts + 1):
            try:
                self._comm.initialize()
                if on_success:
                    on_success(attempt, max_attempts)
                return
            except Exception as exc:
                if attempt < max_attempts:
                    if on_retry:
                        on_retry(attempt, max_attempts, exc)
                    time.sleep(retry_delay)
                else:
                    raise SensorInitializationError(
                        f"Failed to initialize after {max_attempts} attempts: {exc}"
                    )

    def read_contract(self, max_attempts: int = 15, retry_delay: float = 0.3) -> None:
        """Read the one-time contract packet from the transport (SPI / sensor side).

        Three scenarios handled:
          A) Pi starts first  — raw[0] == data_contract → update_contract_struct().
          B) ESP32 was first  — raw[0] == PACKET_START_BYTE → apply defaults.
          C) ESP32 not ready  — other byte → retry; fall back to defaults on timeout.
        """
        for _ in range(max_attempts):
            try:
                raw = self._comm.receive(CONTRACT_PACKET_SIZE)
            except SensorReadError:
                time.sleep(retry_delay)
                continue

            if raw[0] == self._comm.data_contract:
                self.update_contract_struct(raw)
                return

            if raw[0] == PACKET_START_BYTE:
                # Contract was already sent before we connected.
                self._apply_defaults()
                return

            # 0x00 or other: not ready yet → retry
            time.sleep(retry_delay)

        self._apply_defaults()

    def send_contract(self) -> None:
        """Build and send the one-time contract packet (I2C / actuator side)."""
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
        payload = header + fields_bytes
        try:
            self.send(payload)
        except SensorReadError as e:
            raise SensorInitializationError(
                f"Failed to send actuator contract: {e}"
            )

    def receive(self) -> SensorData | None:
        """Read one packet from the transport.

        Returns SensorData for a normal data packet.
        Returns None if a contract packet arrived mid-stream and was applied
        via update_contract_struct() (rare after read_contract() is done).
        Raises SensorReadError on checksum / framing errors — caller retries.
        """
        raw = self._comm.receive(self._packet_size)
        if raw[0] == self._comm.data_contract:
            self.update_contract_struct(raw)
            return None
        return self._parse_data_packet(raw)

    def send(self, body: bytes) -> None:
        """Send *body* through the transport.
        """
        self._comm.send(body)

    def write(self, cmd: ActuatorCommand) -> None:
        """Build and send one ActuatorPacket to the actuator ESP32."""
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
        self.send(body)

    def close(self) -> None:
        self._comm.close()

    @property
    def get_ticks_per_rev(self) -> int:
        """Encoder ticks per full motor revolution (from contract)."""
        return self._ticks_per_rev

    @property
    def fields(self) -> list[str]:
        """Ordered list of field names received in the contract."""
        return list(self._fields)

    # internal

    def update_contract_struct(self, raw: bytes) -> None:
        """Parse a validated ContractPacket and update internal packet state."""
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
