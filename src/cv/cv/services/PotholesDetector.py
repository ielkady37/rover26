#!/usr/bin/env python3
import os
import logging
import cv2
import numpy as np
from utils.Configurator import Configurator
from cv.helpers.CVUtilities import CVUtilities


logger = logging.getLogger(__name__)


class PotholesDetector:
    """Fisheye-aware white-target detector using the pipeline from ``test.py``.

    Parameters
    ----------
    pd_config:
        Dict loaded from ``potholes_detection.yaml``.
    camera_config:
        Dict loaded from ``cameras.yaml`` for the subscribed camera.  Used as a
        fallback for image size and calibration settings.
    """

    def __init__(self, pd_config: dict, camera_config: dict) -> None:
        self._use_fisheye = bool(pd_config.get('use_fisheye', False))
        self._balance = float(
            pd_config.get('balance', camera_config.get('fisheye_balance', 0.0))
        )
        self._width = int(pd_config.get('frame_width', camera_config.get('width', 640)))
        self._height = int(pd_config.get('frame_height', camera_config.get('height', 480)))
        self._calib_path = pd_config.get('calibration_file', camera_config.get('calibration'))

        # Thresholds mirrored from the standalone script.
        self._min_area = int(pd_config.get('min_area', 500))
        self._max_area = int(pd_config.get('max_area', 30000))
        self._min_circularity = float(pd_config.get('min_circularity', 0.65))
        self._hsv_lower = np.array(pd_config.get('hsv_lower', [0, 0, 200]), dtype=np.uint8)
        self._hsv_upper = np.array(pd_config.get('hsv_upper', [180, 40, 255]), dtype=np.uint8)
        self._gray_threshold = int(pd_config.get('gray_threshold', 220))
        self._open_kernel_size = int(pd_config.get('open_kernel_size', 3))
        self._close_kernel_size = int(pd_config.get('close_kernel_size', 5))
        self._blur_kernel_size = int(pd_config.get('blur_kernel_size', 5))
        self._blur_sigma = float(pd_config.get('blur_sigma', 0.0))

        self._map1 = None
        self._map2 = None
        self._ready = False

    # ------------------------------------------------------------------
    # Lifecycle

    def load(self) -> None:
        """Load optional fisheye maps and mark the detector ready.

        Raises
        ------
        RuntimeError
            When the calibration file is missing or cannot be loaded.
        """
        try:
            self._map1 = None
            self._map2 = None

            if self._use_fisheye:
                if not self._calib_path or str(self._calib_path).upper() == 'NONE':
                    raise RuntimeError('calibration path is not configured')

                root = Configurator.getProjectRoot()
                calib_abs = self._resolve_path(str(self._calib_path), root)

                if not os.path.exists(calib_abs):
                    raise RuntimeError(f'calibration file not found: {calib_abs}')

                self._map1, self._map2 = CVUtilities.build_maps(
                    calib_abs, self._balance, self._width, self._height
                )

                if self._map1 is None or self._map2 is None:
                    raise RuntimeError(
                        f'failed to build calibration maps from: {calib_abs}'
                    )

            self._ready = True
            if self._use_fisheye:
                logger.info(
                    'PotholesDetector calibration loaded successfully from %s '
                    '(size=%sx%s, balance=%.3f)',
                    calib_abs,
                    self._width,
                    self._height,
                    self._balance,
                )
            else:
                logger.info(
                    'PotholesDetector ready without fisheye remap (size=%sx%s)',
                    self._width,
                    self._height,
                )
        except Exception as exc:
            self._ready = False
            logger.error('PotholesDetector calibration load failed: %s', str(exc))
            raise RuntimeError(f'PotholesDetector.load failed: {exc}') from exc

    # ------------------------------------------------------------------
    # Detection

    def detect(self, raw_frame: np.ndarray):
        """Run white-target detection on one raw (uncalibrated) frame.

        Applies optional fisheye remap, then runs CLAHE enhancement, HSV and
        grayscale thresholding, morphological cleanup, and contour filtering by
        area and circularity.

        Parameters
        ----------
        raw_frame:
            BGR image array from the ROS uncalibrated topic.

        Returns
        -------
        tuple[np.ndarray, bool, list[float], list[float], list[float]]
            ``(annotated_frame, pothole_detected, xs, ys, radii)``.
            *xs*, *ys*, and *radii* are parallel lists — one entry per
            detected pothole.  All three are empty on any internal failure.
        """
        if not self._ready or raw_frame is None:
            return raw_frame, False, [], [], []

        try:
            frame = raw_frame.copy()

            # frame = cv2.resize(frame, (self._width, self._height))

            # --- Image enhancement ----------------------------------------
            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            l = clahe.apply(l)
            lab = cv2.merge((l, a, b))
            enhanced = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

            # --- Threshold ------------------------------------------------
            hsv = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)
            mask_hsv = cv2.inRange(hsv, self._hsv_lower, self._hsv_upper)

            gray = cv2.cvtColor(enhanced, cv2.COLOR_BGR2GRAY)
            _, mask_gray = cv2.threshold(
                gray,
                self._gray_threshold,
                255,
                cv2.THRESH_BINARY,
            )

            mask = cv2.bitwise_and(mask_hsv, mask_gray)

            # --- Morphology -----------------------------------------------
            open_kernel = np.ones((self._open_kernel_size, self._open_kernel_size), np.uint8)
            close_kernel = np.ones((self._close_kernel_size, self._close_kernel_size), np.uint8)

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)

            blur_kernel = self._blur_kernel_size if self._blur_kernel_size % 2 == 1 else self._blur_kernel_size + 1
            mask = cv2.GaussianBlur(mask, (blur_kernel, blur_kernel), self._blur_sigma)

            # --- Contour filtering ----------------------------------------
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            xs: list = []
            ys: list = []
            radii: list = []

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self._min_area or area > self._max_area:
                    continue

                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0:
                    continue

                circularity = (4 * np.pi * area) / (perimeter * perimeter)
                if circularity < self._min_circularity:
                    continue

                if len(cnt) < 5:
                    continue

                try:
                    ellipse = cv2.fitEllipse(cnt)
                except cv2.error:
                    continue

                (cx, cy), (major_axis, minor_axis), _ = ellipse
                if major_axis <= 0 or minor_axis <= 0:
                    continue

                center = (int(cx), int(cy))
                radius = max(major_axis, minor_axis) / 2.0

                cv2.ellipse(frame, ellipse, (0, 255, 0), 3)
                cv2.circle(frame, center, 4, (0, 0, 255), -1)
                cv2.putText(
                    frame,
                    'TARGET',
                    (center[0] - 40, center[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )

                xs.append(float(cx))
                ys.append(float(cy))
                radii.append(float(radius))

            return frame, len(xs) > 0, xs, ys, radii

        except Exception:
            return raw_frame, False, [], [], []

    # ------------------------------------------------------------------
    # Internals

    @staticmethod
    def _resolve_path(path: str, root: str) -> str:
        """Resolve a repo-relative path (starting with /) against the project root."""
        if os.path.isabs(path) and not os.path.exists(path):
            return os.path.join(root, path.lstrip('/'))
        return path