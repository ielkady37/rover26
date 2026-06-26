import os
import cv2
import time
import numpy as np
from typing import Optional
from dataclasses import dataclass
from utils.Configurator import Configurator

@dataclass
class MonoFrame:
    raw: np.ndarray
    calibrated: np.ndarray

@dataclass
class StereoFrame:
    left_raw: np.ndarray
    left_calibrated: np.ndarray
    right_raw: np.ndarray
    right_calibrated: np.ndarray

class CameraStreamer:
    _OPEN_RETRY_INTERVAL = 0.5   # seconds between retries
    _OPEN_TIMEOUT = 10.0         # total seconds before giving up

    def __init__(self, name: str, config: dict):
        self._name = name
        self._config = config
        self._is_stereo: bool = config.get('is_stereo', False)

        self._cap: Optional[cv2.VideoCapture] = None

        # Mono calibration maps
        self._mono_map1: Optional[np.ndarray] = None
        self._mono_map2: Optional[np.ndarray] = None

        # Stereo calibration maps
        self._left_map1: Optional[np.ndarray] = None
        self._left_map2: Optional[np.ndarray] = None
        self._right_map1: Optional[np.ndarray] = None
        self._right_map2: Optional[np.ndarray] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def name(self) -> str:
        return self._name

    @property
    def is_stereo(self) -> bool:
        return self._is_stereo

    @property
    def has_calibration(self) -> bool:
        return str(self._config.get('calibration', 'NONE')).upper() != 'NONE'

    def open(self) -> bool:
        """
        Opens the camera capture, retrying every 0.5 s for up to 10 s.
        Applies codec, resolution and FPS from config.
        Returns True on success, False if the camera could not be opened
        within the timeout.
        """
        index = self._config.get('index', 0)
        deadline = time.time() + self._OPEN_TIMEOUT

        while time.time() < deadline:
            cap = cv2.VideoCapture(index)
            if cap.isOpened():
                self._cap = cap
                self._apply_config()
                self._load_calibration()
                return True
            cap.release()
            time.sleep(self._OPEN_RETRY_INTERVAL)

        return False

    def read(self) -> Optional[object]:
        """
        Reads the next frame(s).
        Returns a MonoFrame for normal cameras or a StereoFrame for stereo
        cameras. Returns None if the read fails.
        """
        if self._cap is None or not self._cap.isOpened():
            return None

        ret, frame = self._cap.read()
        if not ret or frame is None:
            return None

        if self._is_stereo:
            return self._process_stereo(frame)
        else:
            return self._process_mono(frame)

    def release(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _apply_config(self) -> None:
        if self._is_stereo:
            self._apply_stereo_config()
        else:
            fmt = self._config.get('format', 'MJPG')
            if len(fmt) == 4:
                fourcc = cv2.VideoWriter_fourcc(*fmt)
                self._cap.set(cv2.CAP_PROP_FOURCC, fourcc)

            width = self._config.get('width')
            height = self._config.get('height')
            fps = self._config.get('fps')

            if width:
                self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
            if height:
                self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
            if fps:
                self._cap.set(cv2.CAP_PROP_FPS, float(fps))

    def _apply_stereo_config(self) -> None:
        target_w = int(self._config.get('width', 3840))
        target_h = int(self._config.get('height', 1080))
        fps = float(self._config.get('fps', 30))

        # Step 1 – start at minimum resolution
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)
        self._cap.set(cv2.CAP_PROP_FPS, fps)

        # Step 2 – flush camera buffer
        for _ in range(5):
            self._cap.read()
            time.sleep(0.1)

        # Step 3 – force MJPG codec
        mjpg_fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
        self._cap.set(cv2.CAP_PROP_FOURCC, mjpg_fourcc)
        time.sleep(0.5)

        fourcc_val = int(self._cap.get(cv2.CAP_PROP_FOURCC))
        codec_str = ''.join([chr((fourcc_val >> 8 * i) & 0xFF) for i in range(4)])
        if codec_str != 'MJPG':
            self._cap.set(cv2.CAP_PROP_FOURCC, 1196444237)  # MJPG in decimal
            time.sleep(0.3)

        # Step 4 – build resolution ramp up to target
        standard_steps = [(640, 480), (1280, 720), (1920, 1080), (3840, 1080)]
        ramp = [s for s in standard_steps if s[0] <= target_w and s != (target_w, target_h)]
        ramp.append((target_w, target_h))

        # Step 5 – apply ramp one step at a time
        for w, h in ramp:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            self._cap.set(cv2.CAP_PROP_FPS, fps)
            self._cap.set(cv2.CAP_PROP_FOURCC, mjpg_fourcc)
            time.sleep(0.3)

            real_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            real_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

            if real_w != w or real_h != h:
                # Camera rejected this step; stop here
                break

        # Step 6 – apply exposure/focus settings if specified
        auto_exp = self._config.get('auto_exposure')
        exposure = self._config.get('exposure')
        autofocus = self._config.get('autofocus')

        if auto_exp is not None:
            self._cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, float(auto_exp))
        if exposure is not None:
            self._cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure))
        if autofocus is not None:
            self._cap.set(cv2.CAP_PROP_AUTOFOCUS, float(autofocus))

    def _load_calibration(self) -> None:
        cal_value = self._config.get('calibration', 'NONE')
        cal_type = self._config.get('calibration_type', 'STANDARD')

        if str(cal_value).upper() == 'NONE':
            return

        if cal_type != 'STANDARD':
            # Unsupported calibration type — passthrough mode
            return

        # Resolve path: absolute or relative to the calibration folder
        if os.path.isabs(str(cal_value)):
            cal_path = str(cal_value)
        else:
            try:
                cal_dir = os.path.join(Configurator.getProjectRoot(), 'calibration')
                cal_path = os.path.join(cal_dir, str(cal_value))
            except FileNotFoundError:
                return

        if not os.path.exists(cal_path):
            return

        try:
            data = np.load(cal_path)
            if self._is_stereo:
                self._load_stereo_calibration(data)
            else:
                self._load_mono_calibration(data)
        except Exception:
            pass

    def _load_mono_calibration(self, data: np.lib.npyio.NpzFile) -> None:
        required = {'K', 'D'}
        if not required.issubset(data.files):
            return

        K = data['K']
        D = data['D']
        img_size = self._get_image_size(data)
        if img_size is None:
            return

        self._mono_map1, self._mono_map2 = cv2.initUndistortRectifyMap(
            K, D, None, K, img_size, cv2.CV_32FC1
        )

    def _load_stereo_calibration(self, data: np.lib.npyio.NpzFile) -> None:
        required = {
            'K_left', 'D_left', 'K_right', 'D_right',
            'R_left', 'R_right', 'P_left', 'P_right',
        }
        if not required.issubset(data.files):
            return

        img_size = self._get_image_size(data)
        if img_size is None:
            return

        self._left_map1, self._left_map2 = cv2.initUndistortRectifyMap(
            data['K_left'], data['D_left'], data['R_left'],
            data['P_left'], img_size, cv2.CV_32FC1
        )
        self._right_map1, self._right_map2 = cv2.initUndistortRectifyMap(
            data['K_right'], data['D_right'], data['R_right'],
            data['P_right'], img_size, cv2.CV_32FC1
        )

    def _get_image_size(self, data: np.lib.npyio.NpzFile) -> Optional[tuple]:
        if 'image_size' in data.files:
            s = data['image_size']
            return (int(s[0]), int(s[1]))
        if self._cap is not None and self._cap.isOpened():
            w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if self._is_stereo:
                w = w // 2
            if w > 0 and h > 0:
                return (w, h)
        return None

    def _process_mono(self, frame: np.ndarray) -> MonoFrame:
        if self._mono_map1 is not None:
            calibrated = cv2.remap(frame, self._mono_map1, self._mono_map2,
                                   cv2.INTER_LINEAR)
        else:
            calibrated = frame
        return MonoFrame(raw=frame, calibrated=calibrated)

    def _process_stereo(self, frame: np.ndarray) -> StereoFrame:
        mid = frame.shape[1] // 2
        left_raw = frame[:, :mid]
        right_raw = frame[:, mid:]

        if self._left_map1 is not None:
            left_cal = cv2.remap(left_raw, self._left_map1, self._left_map2,
                                 cv2.INTER_LINEAR)
            right_cal = cv2.remap(right_raw, self._right_map1, self._right_map2,
                                  cv2.INTER_LINEAR)
        else:
            left_cal = left_raw
            right_cal = right_raw

        return StereoFrame(
            left_raw=left_raw,
            left_calibrated=left_cal,
            right_raw=right_raw,
            right_calibrated=right_cal,
        )
