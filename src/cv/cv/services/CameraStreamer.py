#!/usr/bin/env python3
import os
import cv2
import numpy as np
from utils.Configurator import Configurator

class CameraStreamer:
    """Wraps a single camera capture with optional calibration undistortion.

    Parameters
    ----------
    name:
        Human-readable camera name (matches the key in cameras.yaml).
    config:
        Dict loaded from cameras.yaml for this camera. Expected keys:
        ``index``, ``width``, ``height``, ``fps``, ``format``,
        ``calibration`` (path or null), ``calibration_type`` ('fisheye' | 'standard').
    """

    def __init__(self, name: str, config: dict) -> None:
        self._name = name
        self._index = config['index']
        self._width = int(config.get('width', 640))
        self._height = int(config.get('height', 480))
        self._fps = int(config.get('fps', 30))
        self._fourcc_str = str(config.get('format', 'MJPG'))
        self._calibration_path = config.get('calibration')
        self._calibration_type = str(config.get('calibration_type', 'fisheye')).lower()

        self._cap: cv2.VideoCapture | None = None
        self._K: np.ndarray | None = None
        self._D: np.ndarray | None = None

    # ------------------------------------------------------------------
    # Lifecycle

    def open(self) -> bool:
        """Open the capture device and load calibration data if available.

        Returns True when the device opened successfully.
        """
        try:
            # Numeric string (e.g. "0") → int; device path stays as str
            index = self._index
            if isinstance(index, str) and index.lstrip('-').isdigit():
                index = int(index)

            self._cap = cv2.VideoCapture(index)
            if not self._cap.isOpened():
                self._cap = None
                return False

            fourcc = cv2.VideoWriter_fourcc(*self._fourcc_str)
            self._cap.set(cv2.CAP_PROP_FOURCC, fourcc)
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            self._cap.set(cv2.CAP_PROP_FPS, self._fps)

            self._load_calibration()
            return True
        except Exception:
            self._cap = None
            return False

    def release(self) -> None:
        """Release the underlying VideoCapture."""
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def is_opened(self) -> bool:
        """Return True when the capture device is open and ready."""
        return self._cap is not None and self._cap.isOpened()

    # ------------------------------------------------------------------
    # Frame capture

    def get_frame(self):
        """Capture one frame and return (raw, calibrated).

        Returns
        -------
        tuple[np.ndarray | None, np.ndarray | None]
            ``(raw_frame, calibrated_frame)``.
            *calibrated_frame* is ``None`` when no calibration data was loaded.
            Both are ``None`` on capture failure or if the device is not open.
        """
        if not self.is_opened():
            return None, None

        try:
            ret, frame = self._cap.read()
            if not ret or frame is None:
                return None, None

            calibrated = None
            if self._K is not None and self._D is not None:
                calibrated = self._undistort(frame)

            return frame, calibrated
        except Exception:
            return None, None

    # ------------------------------------------------------------------
    # Properties

    @property
    def name(self) -> str:
        return self._name

    @property
    def fps(self) -> int:
        return self._fps

    # ------------------------------------------------------------------
    # Internals

    def _load_calibration(self) -> None:
        """Load K and D matrices from the .npz file referenced in the config."""
        if not self._calibration_path:
            return
        try:
            path = self._calibration_path
            # Config stores repo-relative paths starting with '/' (e.g. /calibration/…).
            # Resolve against the project root when the absolute path doesn't exist on disk.
            if not os.path.exists(path):
                root = Configurator.getProjectRoot()
                path = os.path.join(root, path.lstrip('/'))

            data = np.load(path)
            self._K = data['K']
            self._D = data['D']
        except Exception:
            self._K = None
            self._D = None

    def _undistort(self, frame: np.ndarray) -> np.ndarray:
        """Return an undistorted copy of *frame* using the loaded calibration."""
        try:
            if self._calibration_type == 'fisheye':
                return cv2.fisheye.undistortImage(frame, self._K, self._D, Knew=self._K)
            else:
                return cv2.undistort(frame, self._K, self._D)
        except Exception:
            return frame
