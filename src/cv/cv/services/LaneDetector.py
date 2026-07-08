#!/usr/bin/env python3
"""Lane detection service wrapping YOLOPv2.

Usage::

    detector = LaneDetector(ld_cfg, cam_cfg)
    detector.load()
    annotated, left_coeffs, right_coeffs = detector.detect(bgr_frame)
"""

import traceback
import warnings
import numpy as np
import torch
import cv2

from utils.Configurator import Configurator
from utils.Logger import RoverLogger
from cv.helpers.CVUtilities import CVUtilities
from cv.helpers.YOLOUtils import YOLOUtils

# [DIAG] Log the per-frame diagnostic line every Nth frame (rejections and
# errors are gated the same way; exceptions always log).
_DIAG_EVERY_N = 5


class LaneDetector:
    """Fisheye-aware lane detector using YOLOPv2.

    Parameters
    ----------
    ld_config:
        Dict loaded from ``lane_detection.yaml``.
    camera_config:
        Dict loaded from ``cameras.yaml`` for the subscribed camera.
    """

    def __init__(self, ld_config: dict, camera_config: dict) -> None:
        self._weights = str(ld_config.get('weights', './models/yolopv2.pt'))
        self._img_size = int(ld_config.get('img_size', 640))
        self._conf_thres = float(ld_config.get('conf_thres', 0.3))
        self._iou_thres = float(ld_config.get('iou_thres', 0.45))
        self._device_str = str(ld_config.get('device', 'cpu'))
        self._lane_ema_alpha = float(ld_config.get('lane_ema_alpha', 0.0))
        self._lane_ema_thresh = float(ld_config.get('lane_ema_thresh', 0.0))
        self._poly_degree = int(ld_config.get('poly_degree', 2))

        # Fit sanity gates — a lane line must run predominantly *vertically*
        # through the image. Horizontal blobs (undistortion border, stop
        # lines, wall edges) otherwise fit with enormous curvature and poison
        # everything downstream.
        #   min_y_spread_frac: min vertical extent of lane pixels, as a
        #     fraction of frame height (0.15 * 720 = 108 rows).
        #   max_horizontal_ratio: reject when x-extent / y-extent exceeds
        #     this (a real lane stays well under ~2 even in sharp turns).
        self._min_y_spread_frac = float(ld_config.get('min_y_spread_frac', 0.15))
        self._max_horizontal_ratio = float(ld_config.get('max_horizontal_ratio', 5.0))

        self._width = int(camera_config.get('width', 640))
        self._height = int(camera_config.get('height', 480))
        self._balance = float(ld_config.get('balance', 0.0))
        self._calib_path = camera_config.get('calibration')

        self._device = None
        self._model = None
        self._half = False
        self._map1 = None
        self._map2 = None
        self._lane_ema = None
        self._ready = False

        # [DIAG] state
        self._log = RoverLogger()
        self._diag_frame = 0

    # ------------------------------------------------------------------
    # Lifecycle

    def load(self) -> None:
        """Load YOLOPv2 model and build fisheye undistortion maps.

        Raises
        ------
        RuntimeError
            When the model file is missing or fails to load.
        """
        try:
            self._device = YOLOUtils.select_device(self._device_str)
            self._model = torch.jit.load(self._weights, map_location=self._device)
            self._half = self._device.type != 'cpu'
            if self._half:
                self._model.half()
            self._model.eval()

            # Warm-up pass
            dummy = torch.zeros(1, 3, self._img_size, self._img_size)
            dummy = dummy.to(self._device).type_as(next(self._model.parameters()))
            if self._device.type != 'cpu':
                self._model(dummy)

            if self._calib_path:
                root = Configurator.getProjectRoot()
                calib_abs = self._resolve_path(str(self._calib_path), root)
                self._map1, self._map2 = CVUtilities.build_maps(
                    calib_abs, self._balance, self._width, self._height
                )

            self._ready = True
        except Exception as exc:
            self._ready = False
            raise RuntimeError(f'LaneDetector.load failed: {exc}') from exc

    # ------------------------------------------------------------------
    # Detection

    def detect(self, raw_frame: np.ndarray):
        """Run lane detection on one raw (uncalibrated) BGR frame.

        Parameters
        ----------
        raw_frame:
            BGR image array from the ROS uncalibrated topic.

        Returns
        -------
        tuple[np.ndarray, list[float], list[float]]
            ``(annotated_frame, left_lane_coeffs, right_lane_coeffs)``
            where each coeffs list is ``[a, b, c]`` for the polynomial
            ``x = a*y^2 + b*y + c`` fitted in the lane mask, or ``[]``
            when too few lane pixels were found.
        """
        self._diag_frame += 1
        diag = (self._diag_frame % _DIAG_EVERY_N) == 1

        if not self._ready:
            if diag:
                self._log.warn('[DIAG] detect() called but detector not ready — returning empty coeffs')
            return raw_frame, [], []

        try:
            im0 = raw_frame.copy()

            # --- Fisheye undistortion ---
            if self._map1 is not None and self._map2 is not None:
                im0 = cv2.remap(
                    im0, self._map1, self._map2,
                    interpolation=cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT,
                )

            # --- Letterbox + tensor ---
            img, _, _ = CVUtilities.letterbox(im0, self._img_size, stride=32)
            img = img[:, :, ::-1].transpose(2, 0, 1)
            img = np.ascontiguousarray(img)
            img_t = torch.from_numpy(img).to(self._device)
            img_t = img_t.half() if self._half else img_t.float()
            img_t /= 255.0
            if img_t.ndimension() == 3:
                img_t = img_t.unsqueeze(0)

            # --- Inference ---
            with torch.no_grad():
                [pred, anchor_grid], seg, ll = self._model(img_t)

            pred = YOLOUtils.split_for_trace_model(pred, anchor_grid)
            pred = YOLOUtils.non_max_suppression(
                pred,
                self._conf_thres,
                self._iou_thres,
            )

            da_seg_mask = YOLOUtils.driving_area_mask(seg)
            ll_seg_mask = YOLOUtils.lane_line_mask(ll)

            # The model outputs masks in its own feature-map coordinate space
            # (typically ~720x1280 for a 640-stride input), which is unrelated
            # to the actual frame size. Resize both to match im0 so that
            # show_seg_result overlay and polyfit coordinates are consistent.
            h_frame, w_frame = im0.shape[:2]
            da_seg_mask = cv2.resize(
                da_seg_mask.astype(np.uint8), (w_frame, h_frame),
                interpolation=cv2.INTER_NEAREST,
            )
            ll_seg_mask = cv2.resize(
                ll_seg_mask.astype(np.uint8), (w_frame, h_frame),
                interpolation=cv2.INTER_NEAREST,
            )

            # --- EMA smoothing ---
            if self._lane_ema_alpha > 0.0:
                alpha = float(np.clip(self._lane_ema_alpha, 0.0, 1.0))
                if self._lane_ema is None:
                    self._lane_ema = ll_seg_mask.astype(np.float32)
                else:
                    self._lane_ema = (1.0 - alpha) * self._lane_ema + alpha * ll_seg_mask.astype(np.float32)
                ll_seg_mask = (self._lane_ema >= self._lane_ema_thresh).astype(np.uint8)

            # --- Split left / right by vertical midline ---
            mid = w_frame // 2
            left_mask = ll_seg_mask[:, :mid]
            right_mask = ll_seg_mask[:, mid:]

            left_coeffs = self._fit_lane(left_mask, x_offset=0,
                                         side='LEFT ' if diag else None)
            right_coeffs = self._fit_lane(right_mask, x_offset=mid,
                                          side='RIGHT' if diag else None)

            # [DIAG] per-frame summary: mask pixel budget per side + fit outcome
            if diag:
                left_px = int(left_mask.sum())
                right_px = int(right_mask.sum())
                da_px = int(da_seg_mask.sum())

                def fmt(coeffs):
                    if not coeffs:
                        return 'NONE'
                    return '[' + ' '.join(f'{c:+.4e}' for c in coeffs) + ']'

                self._log.info(
                    f'[DIAG] frame#{self._diag_frame} {w_frame}x{h_frame}  '
                    f'lane_px L={left_px} R={right_px}  drivable_px={da_px}  '
                    f'L={fmt(left_coeffs)}  R={fmt(right_coeffs)}'
                )

            # --- Annotate ---
            YOLOUtils.show_seg_result(im0, (da_seg_mask, ll_seg_mask), is_demo=True)
            for det in pred:
                if len(det):
                    det[:, :4] = CVUtilities.scale_coords(
                        img_t.shape[2:], det[:, :4], im0.shape
                    ).round()
                    for *xyxy, _conf, _cls in reversed(det):
                        YOLOUtils.plot_one_box(xyxy, im0, line_thickness=3)

            return im0, left_coeffs, right_coeffs

        except Exception:
            # [DIAG] previously swallowed silently — an inference/undistort crash
            # looked identical to "no lanes found" downstream. Always log.
            self._log.err(
                f'[DIAG] detect() crashed on frame#{self._diag_frame} — returning '
                f'empty coeffs:\n{traceback.format_exc()}'
            )
            return raw_frame, [], []

    # ------------------------------------------------------------------
    # Internal

    def _fit_lane(self, mask: np.ndarray, x_offset: int = 0, side: str | None = None):
        """Fit a polynomial to lit pixels in *mask*.

        Parameters
        ----------
        mask:
            Binary 2-D array (H × W_half) where 1 = lane pixel.
        x_offset:
            Column offset to add to x-coordinates (non-zero for right lane).
        side:
            [DIAG] Side label ('LEFT '/'RIGHT') to log fit outcome under,
            or None to stay silent (used to throttle logging per frame).

        Returns
        -------
        list[float]
            Polynomial coefficients ``[a, b, ..., c]`` of degree
            ``self._poly_degree`` for ``x = f(y)``, or ``[]`` when fewer
            than ``poly_degree + 1`` pixels are available.
        """
        ys, xs = np.where(mask == 1)
        # Require enough points and sufficient vertical spread for a stable fit
        min_pts = max(self._poly_degree + 1, 10)
        if len(ys) < min_pts:
            if side:
                self._log.warn(
                    f'[DIAG:fit] {side} REJECT — only {len(ys)} lane px '
                    f'(need ≥{min_pts})'
                )
            return []

        y_spread = float(np.ptp(ys))
        x_spread = float(np.ptp(xs))

        # A lane line must span a real vertical stretch of the image.
        min_y_spread = self._min_y_spread_frac * mask.shape[0]
        if y_spread < min_y_spread:
            if side:
                self._log.warn(
                    f'[DIAG:fit] {side} REJECT — y-spread {int(y_spread)}px '
                    f'< {int(min_y_spread)}px ({self._min_y_spread_frac:.2f} of '
                    f'{mask.shape[0]} rows) — horizontal blob, not a lane line'
                )
            return []

        # Near-horizontal features fit x=f(y) with insane curvature.
        ratio = x_spread / max(y_spread, 1.0)
        if ratio > self._max_horizontal_ratio:
            if side:
                self._log.warn(
                    f'[DIAG:fit] {side} REJECT — x/y extent ratio {ratio:.1f} '
                    f'> {self._max_horizontal_ratio:.1f} '
                    f'(x_spread={int(x_spread)}px y_spread={int(y_spread)}px) '
                    f'— feature is near-horizontal'
                )
            return []
        try:
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', np.RankWarning)
                coeffs = np.polyfit(ys, xs + x_offset, self._poly_degree)
            if side:
                self._log.info(
                    f'[DIAG:fit] {side} OK — {len(ys)} px  '
                    f'y=[{int(ys.min())}..{int(ys.max())}]  '
                    f'x=[{int(xs.min()) + x_offset}..{int(xs.max()) + x_offset}]  '
                    f'coeffs=[{" ".join(f"{c:+.4e}" for c in coeffs)}]'
                )
            return coeffs.tolist()
        except (np.linalg.LinAlgError, ValueError) as exc:
            if side:
                self._log.warn(f'[DIAG:fit] {side} REJECT — polyfit failed: {exc}')
            return []

    @staticmethod
    def _resolve_path(path: str, root: str) -> str:
        """Return an absolute path, resolving relative paths from *root*."""
        import os
        if os.path.isabs(path):
            return path
        stripped = path.lstrip('/')
        return os.path.join(root, stripped)



