#!/usr/bin/env python3
import pytest
from unittest.mock import MagicMock, patch
from control.exceptions.SensorInitializationError import SensorInitializationError
from control.exceptions.SensorReadError import SensorReadError
from control.services.I2CService import I2CService


COMM_DETAILS = {
    "bus": 1,
    "address": 0x48,
    "register": 0x00,
    "read_length": 2,
}


@pytest.fixture
def service():
    return I2CService(COMM_DETAILS)


class TestI2CServiceInit:
    def test_defaults_applied(self):
        svc = I2CService({"bus": 1, "address": 0x10})
        assert svc._register == 0x00
        assert svc._read_length == 1
        assert svc._bus is None

    def test_custom_values_stored(self, service):
        assert service._bus_number == 1
        assert service._address == 0x48
        assert service._register == 0x00
        assert service._read_length == 2


class TestI2CServiceInitialize:
    def test_initialize_opens_bus(self, service):
        with patch("control.services.I2CService.smbus2.SMBus") as mock_smbus:
            mock_bus = MagicMock()
            mock_smbus.return_value = mock_bus
            service.initialize()
            mock_smbus.assert_called_once_with(1)
            assert service._bus is mock_bus

    def test_initialize_raises_on_failure(self, service):
        with patch(
            "control.services.I2CService.smbus2.SMBus", side_effect=OSError("no device")
        ):
            with pytest.raises(SensorInitializationError, match="Failed to open I2C bus"):
                service.initialize()


class TestI2CServiceSend:
    def test_send_writes_block_data(self, service):
        mock_bus = MagicMock()
        service._bus = mock_bus
        service.send([0x01, 0x02])
        mock_bus.write_i2c_block_data.assert_called_once_with(0x48, 0x00, [0x01, 0x02])

    def test_send_raises_when_not_initialized(self, service):
        with pytest.raises(SensorInitializationError, match="not initialized"):
            service.send([0x01])

    def test_send_raises_on_bus_error(self, service):
        mock_bus = MagicMock()
        mock_bus.write_i2c_block_data.side_effect = OSError("bus error")
        service._bus = mock_bus
        with pytest.raises(SensorReadError, match="I2C send failed"):
            service.send([0x01])


class TestI2CServiceReceive:
    def test_receive_reads_block_data(self, service):
        mock_bus = MagicMock()
        mock_bus.read_i2c_block_data.return_value = [0xAB, 0xCD]
        service._bus = mock_bus
        result = service.receive()
        mock_bus.read_i2c_block_data.assert_called_once_with(0x48, 0x00, 2)
        assert result == [0xAB, 0xCD]

    def test_receive_raises_when_not_initialized(self, service):
        with pytest.raises(SensorInitializationError, match="not initialized"):
            service.receive()

    def test_receive_raises_on_bus_error(self, service):
        mock_bus = MagicMock()
        mock_bus.read_i2c_block_data.side_effect = OSError("bus error")
        service._bus = mock_bus
        with pytest.raises(SensorReadError, match="I2C receive failed"):
            service.receive()


class TestI2CServiceClose:
    def test_close_closes_bus_and_sets_none(self, service):
        mock_bus = MagicMock()
        service._bus = mock_bus
        service.close()
        mock_bus.close.assert_called_once()
        assert service._bus is None

    def test_close_is_safe_when_not_initialized(self, service):
        service._bus = None
        service.close()  # should not raise
