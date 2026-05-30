#!/usr/bin/env python3
import os
import cv2
import numpy as np
from utils.Configurator import Configurator
from cv.helpers.CVUtilities import CVUtilities


class PotholesDetector:
    """Fisheye-aware pothole detector using classical CV (CLAHE + threshold + morphology).

    Parameters
    ----------
    pd_config:
        Dict loaded from ``potholes_detection.yaml``.  Required key: ``balance``.
    camera_config:
        Dict loaded from ``cameras.yaml`` for the subscribed camera.  Used to
        obtain ``width``, ``height``, and ``calibration`` path for building
        the fisheye undistort maps.
    """

    def __init__(self, pd_config: dict, camera_config: dict) -> None:
        self._balance = float(pd_config.get('balance', 0.0))
        self._width = int(camera_config.get('width', 320))
        self._height = int(camera_config.get('height', 240))
        self._calib_path = camera_config.get('calibration')

        # Contour filter thresholds — tunable via potholes_detection.yaml
        self._min_area = int(pd_config.get('min_area', 150))
        self._max_area = int(pd_config.get('max_area', 60000))
        self._min_circularity = float(pd_config.get('min_circularity', 0.75))
        self._min_aspect_ratio = float(pd_config.get('min_aspect_ratio', 0.75))
        self._max_aspect_ratio = float(pd_config.get('max_aspect_ratio', 1.3))
        self._min_solidity = float(pd_config.get('min_solidity', 0.90))
        self._min_extent = float(pd_config.get('min_extent', 0.72))
        self._min_approx_vertices = int(pd_config.get('min_approx_vertices', 6))
        self._min_radius = float(pd_config.get('min_radius', 8.0))

        self._map1 = None
        self._map2 = None
        self._ready = False

    # ------------------------------------------------------------------
    # Lifecycle

    def load(self) -> None:
        """Build fisheye undistortion maps from the calibration file.

        Raises
        ------
        RuntimeError
            When the calibration file is missing or cannot be loaded.
        """
        try:
            if self._calib_path:
                root = Configurator.getProjectRoot()
                calib_abs = self._resolve_path(str(self._calib_path), root)
                self._map1, self._map2 = CVUtilities.build_maps(
                    calib_abs, self._balance, self._width, self._height
                )
            self._ready = True
        except Exception as exc:
            self._ready = False
            raise RuntimeError(f'PotholesDetector.load failed: {exc}') from exc

    # ------------------------------------------------------------------
    # Detection

    def detect(self, raw_frame: np.ndarray):
        """Run pothole detection on one raw (uncalibrated) frame.

        Applies fisheye remap when calibration maps are available, then runs
        CLAHE enhancement, binary thresholding, morphological cleanup, and
        contour filtering by circularity, solidity, and aspect ratio.

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
            frame = (
                cv2.remap(
                    raw_frame, self._map1, self._map2,
                    interpolation=cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT,
                )
                if self._map1 is not None
                else raw_frame.copy()
            )

            frame = cv2.resize(frame, (self._width, self._height))

            # --- Image enhancement ----------------------------------------
            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
            merged = cv2.merge((clahe.apply(l), a, b))
            enhanced = cv2.cvtColor(merged, cv2.COLOR_LAB2BGR)

            # --- Grayscale + blur -----------------------------------------
            blur = cv2.GaussianBlur(
                cv2.cvtColor(enhanced, cv2.COLOR_BGR2GRAY),
                (9, 9), 2,
            )

            # --- Threshold ------------------------------------------------
            _, mask = cv2.threshold(blur, 225, 255, cv2.THRESH_BINARY)

            # --- Morphology -----------------------------------------------
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)

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
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = w / float(h)
                extent = area / (w * h)
                hull = cv2.convexHull(cnt)
                hull_area = cv2.contourArea(hull)
                solidity = area / hull_area if hull_area > 0 else 0.0
                approx = cv2.approxPolyDP(cnt, 0.02 * perimeter, True)

                if (
                    circularity > self._min_circularity
                    and self._min_aspect_ratio < aspect_ratio < self._max_aspect_ratio
                    and solidity > self._min_solidity
                    and extent > self._min_extent
                    and len(approx) > self._min_approx_vertices
                ):
                    (cx, cy), radius = cv2.minEnclosingCircle(cnt)
                    if radius < self._min_radius:
                        continue

                    center = (int(cx), int(cy))
                    r = int(radius)

                    cv2.circle(frame, center, r, (0, 255, 0), 3)
                    cv2.circle(frame, center, 3, (0, 0, 255), -1)
                    cv2.putText(
                        frame, 'POTHOLE',
                        (center[0] - 40, center[1] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
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

