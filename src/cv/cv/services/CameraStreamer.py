import os
import cv2
import time
import logging
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

        # One-shot log guards to avoid spamming on every frame.
        self._logged_mono_remap_status = False
        self._logged_stereo_remap_status = False
        self.roi: Optional[tuple] = None  # (x, y, w, h) for valid region of interest

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
        cal_type_norm = str(cal_type).upper()

        if str(cal_value).upper() == 'NONE':
            return

        if cal_type_norm not in {'STANDARD', 'FISHEYE'}:
            return

        # Resolve path: absolute or relative to the calibration folder
        if os.path.isabs(str(cal_value)):
            cal_path = str(cal_value)
            # Support project-root style absolute values like /calibration/file.npz.
            if not os.path.exists(cal_path):
                try:
                    cal_path_fallback = os.path.join(
                        Configurator.getProjectRoot(), str(cal_value).lstrip('/')
                    )
                    if os.path.exists(cal_path_fallback):
                        cal_path = cal_path_fallback
                except FileNotFoundError:
                    pass
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
                if cal_type_norm == 'FISHEYE':
                    self._log.warn(
                        '[%s] FISHEYE stereo calibration is not implemented; '
                        'passthrough mode is active.',
                        self._name
                    )
                    return
                self._load_stereo_calibration(data)
            else:
                if cal_type_norm == 'FISHEYE':
                    self._load_mono_fisheye_calibration(data)
                else:
                    self._load_mono_calibration(data)
        except Exception as exc:
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

    def _load_mono_fisheye_calibration(self, data: np.lib.npyio.NpzFile) -> None:
        required = {'K', 'D'}
        if not required.issubset(data.files):
            return

        K = data['K']
        D = data['D']
        runtime_size = self._get_capture_image_size()
        if runtime_size is None:
            # Fallback to the old behavior when capture size is unavailable.
            runtime_size = self._get_image_size(data)

        if runtime_size is None:
            return

        calib_size = None
        if 'image_size' in data.files:
            s = data['image_size']
            calib_size = (int(s[0]), int(s[1]))

        K_runtime = K.copy().astype(np.float64)
        if calib_size is not None and calib_size != runtime_size:
            sx = float(runtime_size[0]) / float(calib_size[0])
            sy = float(runtime_size[1]) / float(calib_size[1])
            K_runtime[0, 0] *= sx
            K_runtime[1, 1] *= sy
            K_runtime[0, 2] *= sx
            K_runtime[1, 2] *= sy

        # balance=1.0 preserves max FOV (less crop, more border). balance=0.0 crops more.
        balance = float(self._config.get('fisheye_balance', 1.0))
        fov_scale = float(self._config.get('fisheye_fov_scale', 1.0))
        Knew = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            K_runtime, D, runtime_size, np.eye(3), balance=balance
        )

        self._mono_map1, self._mono_map2 = cv2.fisheye.initUndistortRectifyMap(
            K_runtime, D, np.eye(3), Knew, runtime_size, cv2.CV_32FC1
        )

        self.roi = self._compute_valid_roi(self._mono_map1, self._mono_map2, runtime_size)

    def _get_capture_image_size(self) -> Optional[tuple]:
        if self._cap is None or not self._cap.isOpened():
            return None

        w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if self._is_stereo:
            w = w // 2
        if w > 0 and h > 0:
            return (w, h)
        return None

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
            if not self._logged_mono_remap_status:
                self._logged_mono_remap_status = True
            calibrated = cv2.remap(frame, self._mono_map1, self._mono_map2,
                                   cv2.INTER_LINEAR)
            if self.roi is not None:
                x, y, w, h = self.roi
                calibrated = calibrated[y : y + h, x : x + w]
        else:
            if not self._logged_mono_remap_status:
                self._logged_mono_remap_status = True
            calibrated = frame
        return MonoFrame(raw=frame, calibrated=calibrated)
    
    def _compute_valid_roi(self, map1_, map2_, size):
        w, h = size
        map1 = map1_
        map2 = map2_
        mask = (
            np.isfinite(map1)
            & np.isfinite(map2)
            & (map1 >= 0.0)
            & (map1 < w)
            & (map2 >= 0.0)
            & (map2 < h)
        )
        h, w = mask.shape
        heights = np.zeros(w, dtype=np.int32)
        best = None
        best_area = 0

        for y in range(h):
            row = mask[y]
            heights = np.where(row, heights + 1, 0)

            stack = []
            for x in range(w + 1):
                cur = heights[x] if x < w else 0
                start = x
                while stack and stack[-1][1] > cur:
                    idx, height = stack.pop()
                    area = height * (x - idx)
                    if area > best_area:
                        best_area = area
                        best = (idx, y - height + 1, x - idx, height)
                    start = idx
                if not stack or stack[-1][1] < cur:
                    stack.append((start, cur))

        if best_area == 0:
            return None
        return (int(best[0]), int(best[1]), int(best[2]), int(best[3]))

    def _process_stereo(self, frame: np.ndarray) -> StereoFrame:
        mid = frame.shape[1] // 2
        left_raw = frame[:, :mid]
        right_raw = frame[:, mid:]

        if self._left_map1 is not None and self._right_map1 is not None:
            if not self._logged_stereo_remap_status:
                self._logged_stereo_remap_status = True
            left_cal = cv2.remap(left_raw, self._left_map1, self._left_map2,
                                 cv2.INTER_LINEAR)
            right_cal = cv2.remap(right_raw, self._right_map1, self._right_map2,
                                  cv2.INTER_LINEAR)
        else:
            if not self._logged_stereo_remap_status:
                self._logged_stereo_remap_status = True
            left_cal = left_raw
            right_cal = right_raw

        return StereoFrame(
            left_raw=left_raw,
            left_calibrated=left_cal,
            right_raw=right_raw,
            right_calibrated=right_cal,
        )
