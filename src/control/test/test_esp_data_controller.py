#!/usr/bin/env python3
import struct
import pytest
from unittest.mock import MagicMock, patch

from control.services.ESPDataController import (
    ESPDataController,
    CONTRACT_PACKET_SIZE,
    CONTRACT_START_BYTE,
    CONTRACT_FIELD_COUNT,
    CONTRACT_FIELD_NAME_LEN,
    PACKET_SIZE,
    PACKET_START_BYTE,
    _xor_checksum,
)
from control.DTOs.SensorData import SensorData
from control.exceptions.SensorInitializationError import SensorInitializationError
from control.exceptions.SensorReadError import SensorReadError

# ── Wire-format builders ──────────────────────────────────────────────────────

FIELD_NAMES = [
    "yaw", "pitch", "roll",
    "qx", "qy", "qz", "qw",
    "gx", "gy", "gz",
    "ax", "ay", "az",
    "enc1_rev", "enc2_rev",
]

TICKS_PER_REV = 1000


def _build_contract(
    field_names=None,
    ticks_per_rev=TICKS_PER_REV,
    corrupt_checksum=False,
    corrupt_start=False,
) -> bytes:
    """Build a valid (or intentionally broken) 202-byte ContractPacket."""
    names = field_names or FIELD_NAMES
    header = struct.pack(
        "<BBHH",
        0xFF if corrupt_start else CONTRACT_START_BYTE,
        len(names),
        PACKET_SIZE,
        ticks_per_rev,
    )
    fields_bytes = b""
    for name in names:
        name_bytes = name.encode("ascii").ljust(CONTRACT_FIELD_NAME_LEN, b"\x00")
        fields_bytes += struct.pack(f"<{CONTRACT_FIELD_NAME_LEN}sB", name_bytes, 32)

    body = header + fields_bytes
    cs = _xor_checksum(body)
    if corrupt_checksum:
        cs ^= 0xFF  # flip all bits → guaranteed wrong
    return body + bytes([cs])


def _build_data_packet(
    yaw=1.0, pitch=2.0, roll=3.0,
    qx=0.1, qy=0.2, qz=0.3, qw=0.9,
    gx=0.01, gy=0.02, gz=0.03,
    ax=0.5, ay=-0.5, az=9.81,
    enc1=4.25, enc2=-2.5,
    corrupt_checksum=False,
    corrupt_start=False,
) -> bytes:
    """Build a valid (or intentionally broken) 62-byte SensorPacket."""
    start = 0xFF if corrupt_start else PACKET_START_BYTE
    body = struct.pack(
        "<B15f",
        start,
        yaw, pitch, roll,
        qx, qy, qz, qw,
        gx, gy, gz,
        ax, ay, az,
        enc1, enc2,
    )
    cs = _xor_checksum(body)
    if corrupt_checksum:
        cs ^= 0xFF
    return body + bytes([cs])


# ── Fixtures ──────────────────────────────────────────────────────────────────

COMM_DETAILS = {"bus": 0, "device": 0}


@pytest.fixture
def mock_spi():
    """Return a MagicMock that replaces spidev.SpiDev inside SPIService."""
    return MagicMock()


@pytest.fixture
def initialized_controller(mock_spi):
    """ESPDataController with SPI mocked out and contract already exchanged."""
    with patch("control.services.SPIService.spidev.SpiDev") as mock_cls:
        mock_cls.return_value = mock_spi
        mock_spi.xfer2.return_value = list(_build_contract())
        ctrl = ESPDataController(COMM_DETAILS)
        ctrl.initialize()
    return ctrl, mock_spi


# ── _xor_checksum ─────────────────────────────────────────────────────────────

class TestXorChecksum:
    def test_empty(self):
        assert _xor_checksum(b"") == 0

    def test_single_byte(self):
        assert _xor_checksum(bytes([0xAB])) == 0xAB

    def test_known_value(self):
        assert _xor_checksum(bytes([0x01, 0x02, 0x03])) == 0x00

    def test_self_cancel(self):
        assert _xor_checksum(bytes([0xFF, 0xFF])) == 0x00


# ── ESPDataController.__init__ ────────────────────────────────────────────────

class TestESPDataControllerInit:
    def test_defaults(self):
        ctrl = ESPDataController(COMM_DETAILS)
        assert ctrl._packet_size == PACKET_SIZE
        assert ctrl._fields == []
        assert ctrl._ticks_per_rev == 0


# ── ESPDataController.initialize / contract parsing ───────────────────────────

class TestContractParsing:
    def test_parses_field_names(self, initialized_controller):
        ctrl, _ = initialized_controller
        assert ctrl.fields == FIELD_NAMES

    def test_parses_packet_size(self, initialized_controller):
        ctrl, _ = initialized_controller
        assert ctrl._packet_size == PACKET_SIZE

    def test_parses_ticks_per_rev(self, initialized_controller):
        ctrl, _ = initialized_controller
        assert ctrl.ticks_per_rev == TICKS_PER_REV

    def test_contract_requested_correct_size(self, mock_spi):
        with patch("control.services.SPIService.spidev.SpiDev") as mock_cls:
            mock_cls.return_value = mock_spi
            mock_spi.xfer2.return_value = list(_build_contract())
            ctrl = ESPDataController(COMM_DETAILS)
            ctrl.initialize()
        first_call_arg = mock_spi.xfer2.call_args_list[0][0][0]
        assert len(first_call_arg) == CONTRACT_PACKET_SIZE
        assert all(b == 0 for b in first_call_arg)

    def test_bad_start_byte_raises(self, mock_spi):
        with patch("control.services.SPIService.spidev.SpiDev") as mock_cls:
            mock_cls.return_value = mock_spi
            mock_spi.xfer2.return_value = list(_build_contract(corrupt_start=True))
            ctrl = ESPDataController(COMM_DETAILS)
            with pytest.raises(SensorInitializationError, match="contract start byte"):
                ctrl.initialize()

    def test_bad_checksum_raises(self, mock_spi):
        with patch("control.services.SPIService.spidev.SpiDev") as mock_cls:
            mock_cls.return_value = mock_spi
            mock_spi.xfer2.return_value = list(_build_contract(corrupt_checksum=True))
            ctrl = ESPDataController(COMM_DETAILS)
            with pytest.raises(SensorInitializationError, match="Contract checksum"):
                ctrl.initialize()


# ── ESPDataController.read / data-packet parsing ──────────────────────────────

class TestDataPacketParsing:
    def test_returns_sensor_data_type(self, initialized_controller):
        ctrl, mock_spi = initialized_controller
        mock_spi.xfer2.return_value = list(_build_data_packet())
        result = ctrl.read()
        assert isinstance(result, SensorData)

    def test_correct_euler_angles(self, initialized_controller):
        ctrl, mock_spi = initialized_controller
        mock_spi.xfer2.return_value = list(
            _build_data_packet(yaw=45.0, pitch=-10.0, roll=180.0)
        )
        data = ctrl.read()
        assert data.yaw   == pytest.approx(45.0)
        assert data.pitch == pytest.approx(-10.0)
        assert data.roll  == pytest.approx(180.0)

    def test_correct_quaternion(self, initialized_controller):
        ctrl, mock_spi = initialized_controller
        mock_spi.xfer2.return_value = list(
            _build_data_packet(qx=0.1, qy=0.2, qz=0.3, qw=0.9)
        )
        data = ctrl.read()
        assert data.qx == pytest.approx(0.1)
        assert data.qy == pytest.approx(0.2)
        assert data.qz == pytest.approx(0.3)
        assert data.qw == pytest.approx(0.9)

    def test_correct_gyro(self, initialized_controller):
        ctrl, mock_spi = initialized_controller
        mock_spi.xfer2.return_value = list(
            _build_data_packet(gx=1.1, gy=-2.2, gz=3.3)
        )
        data = ctrl.read()
        assert data.gx == pytest.approx(1.1)
        assert data.gy == pytest.approx(-2.2)
        assert data.gz == pytest.approx(3.3)

    def test_correct_accel(self, initialized_controller):
        ctrl, mock_spi = initialized_controller
        mock_spi.xfer2.return_value = list(
            _build_data_packet(ax=0.0, ay=0.0, az=9.81)
        )
        data = ctrl.read()
        assert data.ax == pytest.approx(0.0)
        assert data.ay == pytest.approx(0.0)
        assert data.az == pytest.approx(9.81)

    def test_correct_encoders(self, initialized_controller):
        ctrl, mock_spi = initialized_controller
        mock_spi.xfer2.return_value = list(
            _build_data_packet(enc1=12.5, enc2=-3.75)
        )
        data = ctrl.read()
        assert data.enc1_net_rev == pytest.approx(12.5)
        assert data.enc2_net_rev == pytest.approx(-3.75)

    def test_data_requested_correct_size(self, initialized_controller):
        ctrl, mock_spi = initialized_controller
        mock_spi.xfer2.return_value = list(_build_data_packet())
        ctrl.read()
        last_call_arg = mock_spi.xfer2.call_args_list[-1][0][0]
        assert len(last_call_arg) == PACKET_SIZE
        assert all(b == 0 for b in last_call_arg)

    def test_bad_start_byte_raises(self, initialized_controller):
        ctrl, mock_spi = initialized_controller
        mock_spi.xfer2.return_value = list(_build_data_packet(corrupt_start=True))
        with pytest.raises(SensorReadError, match="data start byte"):
            ctrl.read()

    def test_bad_checksum_raises(self, initialized_controller):
        ctrl, mock_spi = initialized_controller
        mock_spi.xfer2.return_value = list(_build_data_packet(corrupt_checksum=True))
        with pytest.raises(SensorReadError, match="checksum"):
            ctrl.read()

    def test_wrong_length_raises(self, initialized_controller):
        ctrl, mock_spi = initialized_controller
        mock_spi.xfer2.return_value = [0x00] * (PACKET_SIZE - 1)
        with pytest.raises(SensorReadError, match="Expected"):
            ctrl.read()


# ── ESPDataController.close ───────────────────────────────────────────────────

class TestClose:
    def test_close_delegates_to_spi(self, initialized_controller):
        ctrl, mock_spi = initialized_controller
        ctrl.close()
        mock_spi.close.assert_called_once()
