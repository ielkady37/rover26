#!/usr/bin/env python3
import pytest
from unittest.mock import MagicMock, patch
from control.exceptions.SensorInitializationError import SensorInitializationError
from control.exceptions.SensorReadError import SensorReadError
from control.services.SPIService import SPIService


COMM_DETAILS = {
    "bus": 0,
    "device": 0,
    "max_speed_hz": 500_000,
    "mode": 1,
}


@pytest.fixture
def service():
    return SPIService(COMM_DETAILS)


class TestSPIServiceInit:
    def test_defaults_applied(self):
        svc = SPIService({"bus": 0, "device": 0})
        assert svc._max_speed_hz == 1_000_000
        assert svc._mode == 0
        assert svc._spi is None

    def test_custom_values_stored(self, service):
        assert service._bus == 0
        assert service._device == 0
        assert service._max_speed_hz == 500_000
        assert service._mode == 1


class TestSPIServiceInitialize:
    def test_initialize_opens_device(self, service):
        with patch("control.services.SPIService.spidev.SpiDev") as mock_spidev:
            mock_spi = MagicMock()
            mock_spidev.return_value = mock_spi
            service.initialize()
            mock_spi.open.assert_called_once_with(0, 0)
            assert mock_spi.max_speed_hz == 500_000
            assert mock_spi.mode == 1
            assert service._spi is mock_spi

    def test_initialize_raises_on_failure(self, service):
        with patch(
            "control.services.SPIService.spidev.SpiDev", side_effect=OSError("no device")
        ):
            with pytest.raises(SensorInitializationError, match="Failed to open SPI bus"):
                service.initialize()

    def test_initialize_raises_when_open_fails(self, service):
        with patch("control.services.SPIService.spidev.SpiDev") as mock_spidev:
            mock_spi = MagicMock()
            mock_spi.open.side_effect = OSError("permission denied")
            mock_spidev.return_value = mock_spi
            with pytest.raises(SensorInitializationError, match="Failed to open SPI bus"):
                service.initialize()


class TestSPIServiceSend:
    def test_send_writes_bytes(self, service):
        mock_spi = MagicMock()
        service._spi = mock_spi
        service.send([0xDE, 0xAD])
        mock_spi.writebytes.assert_called_once_with([0xDE, 0xAD])

    def test_send_raises_when_not_initialized(self, service):
        with pytest.raises(SensorInitializationError, match="not initialized"):
            service.send([0x01])

    def test_send_raises_on_spi_error(self, service):
        mock_spi = MagicMock()
        mock_spi.writebytes.side_effect = OSError("spi error")
        service._spi = mock_spi
        with pytest.raises(SensorReadError, match="SPI send failed"):
            service.send([0x01])


class TestSPIServiceReceive:
    def test_receive_reads_one_byte(self, service):
        mock_spi = MagicMock()
        mock_spi.readbytes.return_value = [0xFF]
        service._spi = mock_spi
        result = service.receive()
        mock_spi.readbytes.assert_called_once_with(1)
        assert result == [0xFF]

    def test_receive_raises_when_not_initialized(self, service):
        with pytest.raises(SensorInitializationError, match="not initialized"):
            service.receive()

    def test_receive_raises_on_spi_error(self, service):
        mock_spi = MagicMock()
        mock_spi.readbytes.side_effect = OSError("spi error")
        service._spi = mock_spi
        with pytest.raises(SensorReadError, match="SPI receive failed"):
            service.receive()


class TestSPIServiceClose:
    def test_close_closes_device_and_sets_none(self, service):
        mock_spi = MagicMock()
        service._spi = mock_spi
        service.close()
        mock_spi.close.assert_called_once()
        assert service._spi is None

    def test_close_is_safe_when_not_initialized(self, service):
        service._spi = None
        service.close()  # should not raise
