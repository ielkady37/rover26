#!/usr/bin/env python3
import os
import cv2
import numpy as np

class CVUtilities:
    """Static collection of reusable computer-vision geometry helpers."""

    @staticmethod
    def scale_intrinsics(K: np.ndarray, from_size: tuple, to_size: tuple) -> np.ndarray:
        """Scale a 3×3 camera matrix from one resolution to another.

        Parameters
        ----------
        K:          3×3 intrinsic matrix (float64).
        from_size:  (width, height) of the calibration image.
        to_size:    (width, height) of the target capture resolution.
        """
        from_w, from_h = from_size
        to_w, to_h = to_size
        sx = to_w / from_w
        sy = to_h / from_h
        K_scaled = K.copy()
        K_scaled[0, 0] *= sx   # fx
        K_scaled[0, 2] *= sx   # cx
        K_scaled[1, 1] *= sy   # fy
        K_scaled[1, 2] *= sy   # cy
        return K_scaled

    @staticmethod
    def build_maps(calib_path: str, balance: float, frame_width: int, frame_height: int):
        """Build fisheye undistortion/rectification maps for use with ``cv2.remap``.

        Uses ``cv2.fisheye.estimateNewCameraMatrixForUndistortRectify`` +
        ``cv2.fisheye.initUndistortRectifyMap``, giving high-quality results
        with a configurable *balance* factor:
        ``0.0`` crops away all black borders; ``1.0`` keeps the full FOV.

        Parameters
        ----------
        calib_path:     Absolute path to a .npz file with keys
                        ``K`` (3×3), ``D`` (4×1), ``image_size`` (2,).
        balance:        Float in [0, 1].
        frame_width:    Target frame width in pixels.
        frame_height:   Target frame height in pixels.

        Returns
        -------
        tuple[np.ndarray, np.ndarray] | tuple[None, None]
            ``(map1, map2)`` ready for ``cv2.remap``, or ``(None, None)``
            when the calibration file is absent or cannot be loaded.
        """
        if not calib_path or not os.path.exists(calib_path):
            return None, None

        try:
            data = np.load(calib_path)
            K_raw = data['K'].astype(np.float64)
            D = data['D'].astype(np.float64)
            calib_size = tuple(int(x) for x in data['image_size'])
            balance = float(np.clip(balance, 0.0, 1.0))

            K = CVUtilities.scale_intrinsics(K_raw, calib_size, (frame_width, frame_height))
            target_size = (frame_width, frame_height)

            new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                K, D, target_size, np.eye(3), balance=balance
            )
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(
                K, D, np.eye(3), new_K, target_size, cv2.CV_32FC1
            )
            return map1, map2
        except Exception:
            return None, None
