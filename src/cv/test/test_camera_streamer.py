#!/usr/bin/env python3
import numpy as np
import pytest
import cv2
from unittest.mock import MagicMock, patch

from cv.services.CameraStreamer import CameraStreamer


# ---------------------------------------------------------------------------
# Helpers

_BASE_CFG = {
    'index': '/dev/video0',
    'width': 320,
    'height': 240,
    'fps': 20,
    'format': 'MJPG',
    'calibration': None,
    'calibration_type': 'fisheye',
}


def _make(overrides=None):
    cfg = {**_BASE_CFG, **(overrides or {})}
    return CameraStreamer('test_cam', cfg)


def _fake_frame():
    return np.zeros((240, 320, 3), dtype=np.uint8)


# ---------------------------------------------------------------------------
# __init__

class TestInit:
    def test_stores_all_config_fields(self):
        s = _make()
        assert s.name == 'test_cam'
        assert s.fps == 20
        assert s._index == '/dev/video0'
        assert s._width == 320
        assert s._height == 240
        assert s._fourcc_str == 'MJPG'
        assert s._calibration_path is None
        assert s._calibration_type == 'fisheye'
        assert s._cap is None
        assert s._K is None
        assert s._D is None

    def test_defaults_applied_for_missing_optional_keys(self):
        s = CameraStreamer('cam', {'index': 0})
        assert s._width == 640
        assert s._height == 480
        assert s._fps == 30
        assert s._fourcc_str == 'MJPG'
        assert s._calibration_type == 'fisheye'

    def test_calibration_type_lowercased(self):
        s = _make({'calibration_type': 'STANDARD'})
        assert s._calibration_type == 'standard'


# ---------------------------------------------------------------------------
# is_opened()

class TestIsOpened:
    def test_false_before_open(self):
        assert not _make().is_opened()

    def test_true_when_cap_reports_open(self):
        s = _make()
        s._cap = MagicMock(**{'isOpened.return_value': True})
        assert s.is_opened()

    def test_false_when_cap_reports_closed(self):
        s = _make()
        s._cap = MagicMock(**{'isOpened.return_value': False})
        assert not s.is_opened()


# ---------------------------------------------------------------------------
# open()

class TestOpen:
    @patch('cv.services.CameraStreamer.cv2.VideoCapture')
    def test_open_success_sets_cap_properties(self, mock_vc_cls):
        mock_cap = MagicMock(**{'isOpened.return_value': True})
        mock_vc_cls.return_value = mock_cap

        result = _make({'index': '/dev/video0'}).open()

        assert result is True
        mock_vc_cls.assert_called_once_with('/dev/video0')
        mock_cap.set.assert_any_call(cv2.CAP_PROP_FRAME_WIDTH, 320)
        mock_cap.set.assert_any_call(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        mock_cap.set.assert_any_call(cv2.CAP_PROP_FPS, 20)

    @patch('cv.services.CameraStreamer.cv2.VideoCapture')
    def test_numeric_string_index_converted_to_int(self, mock_vc_cls):
        mock_cap = MagicMock(**{'isOpened.return_value': True})
        mock_vc_cls.return_value = mock_cap

        _make({'index': '3'}).open()

        mock_vc_cls.assert_called_once_with(3)

    @patch('cv.services.CameraStreamer.cv2.VideoCapture')
    def test_integer_index_passed_directly(self, mock_vc_cls):
        mock_cap = MagicMock(**{'isOpened.return_value': True})
        mock_vc_cls.return_value = mock_cap

        _make({'index': 0}).open()

        mock_vc_cls.assert_called_once_with(0)

    @patch('cv.services.CameraStreamer.cv2.VideoCapture')
    def test_open_returns_false_and_clears_cap_when_device_not_opened(self, mock_vc_cls):
        mock_cap = MagicMock(**{'isOpened.return_value': False})
        mock_vc_cls.return_value = mock_cap

        s = _make()
        assert s.open() is False
        assert s._cap is None

    @patch('cv.services.CameraStreamer.cv2.VideoCapture', side_effect=Exception('os error'))
    def test_open_returns_false_on_exception(self, _):
        s = _make()
        assert s.open() is False
        assert s._cap is None

    @patch('cv.services.CameraStreamer.cv2.VideoCapture')
    def test_open_loads_calibration_when_path_set(self, mock_vc_cls, tmp_path):
        mock_cap = MagicMock(**{'isOpened.return_value': True})
        mock_vc_cls.return_value = mock_cap

        npz = tmp_path / 'calib.npz'
        np.savez(str(npz), K=np.eye(3), D=np.zeros((4, 1)))

        s = _make({'calibration': str(npz), 'calibration_type': 'fisheye'})
        s.open()

        assert s._K is not None
        assert s._D is not None


# ---------------------------------------------------------------------------
# release()

class TestRelease:
    def test_calls_cap_release_and_clears_reference(self):
        s = _make()
        mock_cap = MagicMock()
        s._cap = mock_cap

        s.release()

        mock_cap.release.assert_called_once()
        assert s._cap is None

    def test_safe_when_already_released(self):
        _make().release()  # must not raise

    def test_is_opened_returns_false_after_release(self):
        s = _make()
        s._cap = MagicMock(**{'isOpened.return_value': True})
        s.release()
        assert not s.is_opened()


# ---------------------------------------------------------------------------
# get_frame()

class TestGetFrame:
    def test_returns_none_none_when_not_opened(self):
        raw, cal = _make().get_frame()
        assert raw is None and cal is None

    def test_returns_frame_with_none_calibrated_when_no_matrices(self):
        s = _make()
        frame = _fake_frame()
        s._cap = MagicMock(**{'isOpened.return_value': True, 'read.return_value': (True, frame)})

        raw, cal = s.get_frame()

        assert raw is frame
        assert cal is None

    def test_returns_calibrated_frame_with_standard_type(self):
        s = _make({'calibration_type': 'standard'})
        frame = _fake_frame()
        undistorted = np.ones_like(frame)
        s._cap = MagicMock(**{'isOpened.return_value': True, 'read.return_value': (True, frame)})
        s._K = np.eye(3)
        s._D = np.zeros((4, 1))

        with patch('cv.services.CameraStreamer.cv2.undistort', return_value=undistorted):
            raw, cal = s.get_frame()

        assert raw is frame
        assert cal is undistorted

    def test_returns_calibrated_frame_with_fisheye_type(self):
        s = _make({'calibration_type': 'fisheye'})
        frame = _fake_frame()
        undistorted = np.ones_like(frame)
        s._cap = MagicMock(**{'isOpened.return_value': True, 'read.return_value': (True, frame)})
        s._K = np.eye(3)
        s._D = np.zeros((4, 1))

        with patch('cv.services.CameraStreamer.cv2.fisheye') as mock_fisheye:
            mock_fisheye.undistortImage.return_value = undistorted
            raw, cal = s.get_frame()

        assert raw is frame
        assert cal is undistorted
        mock_fisheye.undistortImage.assert_called_once_with(frame, s._K, s._D, Knew=s._K)

    def test_returns_none_none_on_read_failure(self):
        s = _make()
        s._cap = MagicMock(**{'isOpened.return_value': True, 'read.return_value': (False, None)})

        raw, cal = s.get_frame()

        assert raw is None and cal is None

    def test_returns_none_none_on_read_exception(self):
        s = _make()
        s._cap = MagicMock(**{'isOpened.return_value': True})
        s._cap.read.side_effect = RuntimeError('camera disconnected')

        raw, cal = s.get_frame()

        assert raw is None and cal is None


# ---------------------------------------------------------------------------
# _load_calibration()

class TestLoadCalibration:
    def test_no_op_when_path_is_none(self):
        s = _make({'calibration': None})
        s._load_calibration()
        assert s._K is None and s._D is None

    def test_loads_K_and_D_shapes_from_valid_npz(self, tmp_path):
        K = np.eye(3, dtype=np.float64)
        D = np.zeros((4, 1), dtype=np.float64)
        npz = tmp_path / 'calib.npz'
        np.savez(str(npz), K=K, D=D)

        s = _make({'calibration': str(npz)})
        s._load_calibration()

        assert s._K.shape == (3, 3)
        assert s._D.shape == (4, 1)

    def test_K_values_match_saved_matrix(self, tmp_path):
        K = np.array([[500, 0, 160], [0, 500, 120], [0, 0, 1]], dtype=np.float64)
        npz = tmp_path / 'calib.npz'
        np.savez(str(npz), K=K, D=np.zeros((4, 1)))

        s = _make({'calibration': str(npz)})
        s._load_calibration()

        np.testing.assert_array_equal(s._K, K)

    def test_stays_none_when_file_missing(self):
        s = _make({'calibration': '/nonexistent/path.npz'})
        s._load_calibration()
        assert s._K is None and s._D is None

    def test_stays_none_when_npz_has_wrong_keys(self, tmp_path):
        npz = tmp_path / 'bad.npz'
        np.savez(str(npz), wrong_key=np.eye(3))

        s = _make({'calibration': str(npz)})
        s._load_calibration()

        assert s._K is None and s._D is None


# ---------------------------------------------------------------------------
# _undistort()

class TestUndistort:
    def setup_method(self):
        self.s = _make()
        self.s._K = np.eye(3)
        self.s._D = np.zeros((4, 1))
        self.frame = _fake_frame()

    def test_fisheye_delegates_to_cv2_fisheye(self):
        self.s._calibration_type = 'fisheye'
        expected = np.ones_like(self.frame)
        with patch('cv.services.CameraStreamer.cv2.fisheye') as mock_fisheye:
            mock_fisheye.undistortImage.return_value = expected
            result = self.s._undistort(self.frame)

        assert result is expected
        mock_fisheye.undistortImage.assert_called_once_with(
            self.frame, self.s._K, self.s._D, Knew=self.s._K
        )

    def test_standard_delegates_to_cv2_undistort(self):
        self.s._calibration_type = 'standard'
        expected = np.ones_like(self.frame)
        with patch('cv.services.CameraStreamer.cv2.undistort', return_value=expected) as mock_ud:
            result = self.s._undistort(self.frame)

        assert result is expected
        mock_ud.assert_called_once_with(self.frame, self.s._K, self.s._D)

    def test_returns_original_frame_on_undistort_exception(self):
        self.s._calibration_type = 'fisheye'
        with patch('cv.services.CameraStreamer.cv2.fisheye') as mock_fisheye:
            mock_fisheye.undistortImage.side_effect = Exception('bad matrix')
            result = self.s._undistort(self.frame)

        assert result is self.frame
