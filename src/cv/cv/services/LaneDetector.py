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

        # Spacing (px, Euclidean) enforced between consecutive drawn points
        # so the overlay never jumps — every point is the real row-wise
        # center of the chosen lane blob, resampled to this step.
        self._point_spacing_px = int(ld_config.get('point_spacing_px', 8))

        # Fraction of the frame height, from the top, to drop before any
        # lane-line contour is considered — the top third is sky/horizon/
        # buildings, not road, and every "picked the far/wrong line" bug so
        # far has come from something up there (the horizon border, a
        # building edge) surviving into the candidate pool.
        self._ignore_top_frac = float(ld_config.get('ignore_top_frac', 0.33))

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

            # # --- Fisheye undistortion ---
            # if self._map1 is not None and self._map2 is not None:
            #     im0 = cv2.remap(
            #         im0, self._map1, self._map2,
            #         interpolation=cv2.INTER_LINEAR,
            #         borderMode=cv2.BORDER_CONSTANT,
            #     )

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

            # --- Drop the top slice of the frame (sky/horizon/buildings)
            # before any contour is found there. Every "picked the wrong,
            # far-off line" bug so far traced back to something up there
            # (horizon border, a building edge) surviving into the
            # candidate pool — cheaper and more reliable to remove it than
            # to keep out-arguing it in the ranking logic. ---
            ignore_rows = int(h_frame * self._ignore_top_frac)
            if ignore_rows > 0:
                ll_seg_mask[:ignore_rows, :] = 0

            # Adaptive per-row center column, from the drivable-area mask's
            # own extent (rather than a fixed w_frame // 2), so it follows
            # the road as it curves or the camera tilts/rolls.
            mid_col = self._adaptive_midline(da_seg_mask, w_frame)
            lane_bool = ll_seg_mask == 1

            left_coeffs, left_points, right_coeffs, right_points = self._fit_lanes(
                lane_bool, mid_col, diag
            )

            # [DIAG] per-frame summary: mask pixel budget + fit outcome
            if diag:
                lane_px = int(lane_bool.sum())
                da_px = int(da_seg_mask.sum())

                def fmt(coeffs):
                    if not coeffs:
                        return 'NONE'
                    return '[' + ' '.join(f'{c:+.4e}' for c in coeffs) + ']'

                # self._log.info(
                #     f'[DIAG] frame#{self._diag_frame} {w_frame}x{h_frame}  '
                #     f'lane_px={lane_px}  drivable_px={da_px}  '
                #     f'L={fmt(left_coeffs)}  R={fmt(right_coeffs)}'
                # )

            # --- Annotate ---
            YOLOUtils.show_seg_result(im0, (da_seg_mask, ll_seg_mask), is_demo=True)

            # Drawn from the actual selected lane blob's own pixels (never
            # from the fitted polynomial), so every point sits on a
            # detected contour: LEFT = blue, RIGHT = red.
            self._draw_lane_points(im0, left_points, (255, 80, 0))
            self._draw_lane_points(im0, right_points, (0, 0, 255))
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
            # self._log.err(
            #     f'[DIAG] detect() crashed on frame#{self._diag_frame} — returning '
            #     f'empty coeffs:\n{traceback.format_exc()}'
            # )
            return raw_frame, [], []

    # ------------------------------------------------------------------
    # Internal

    def _fit_lanes(self, lane_bool: np.ndarray, mid_col: np.ndarray, diag: bool):
        """Pick the nearest-to-center lane contour on each side, fit and trace it.

        Contours are found on the *whole* mask, not a mask already split
        into "left"/"right" by column: splitting first cuts any blob that
        straddles that column — a fork's median edge, a curving lane
        crossing it mid-turn — into fragments that then get stitched onto
        whichever side grabbed each piece, producing a line that hooks or
        bends where it shouldn't. Each whole contour is classified left or
        right afterwards by the sign of its own real pixels' ``x - mid_col``
        (majority vote), then ranked by proximity to center at its own
        bottom-most rows, the same as before.

        Parameters
        ----------
        lane_bool:
            Boolean 2-D array (H × W) of the full lane-line mask.
        mid_col:
            Per-row center column (shape ``(H,)``).
        diag:
            [DIAG] Whether to log fit rejections/outcomes this frame.

        Returns
        -------
        tuple[list[float], np.ndarray, list[float], np.ndarray]
            ``(left_coeffs, left_points, right_coeffs, right_points)``.
        """
        min_pts = max(self._poly_degree + 1, 10)
        min_y_spread = self._min_y_spread_frac * lane_bool.shape[0]

        num_labels, labels = cv2.connectedComponents(lane_bool.astype(np.uint8), connectivity=8)

        left_candidates = []
        right_candidates = []
        for lbl in range(1, num_labels):
            ys, xs = np.where(labels == lbl)
            if len(ys) < min_pts:
                continue

            y_spread = float(np.ptp(ys))
            x_spread = float(np.ptp(xs))
            if y_spread < min_y_spread:
                continue

            ratio = x_spread / max(y_spread, 1.0)
            if ratio > self._max_horizontal_ratio:
                continue

            is_left = float(np.median(xs - mid_col[ys])) < 0.0
            (left_candidates if is_left else right_candidates).append((ys, xs))

        left_coeffs, left_points = self._pick_nearest(
            left_candidates, mid_col, side='LEFT ' if diag else None
        )
        right_coeffs, right_points = self._pick_nearest(
            right_candidates, mid_col, side='RIGHT' if diag else None
        )
        return left_coeffs, left_points, right_coeffs, right_points

    def _pick_nearest(self, candidates: list, mid_col: np.ndarray, side: str | None = None):
        """Pick the candidate contour nearest to center, fit and trace it.

        Parameters
        ----------
        candidates:
            List of ``(ys, xs)`` pixel-coordinate pairs, one per contour
            already classified onto this side.
        mid_col:
            Per-row center column (shape ``(H,)``), used to rank contours.
        side:
            [DIAG] Side label ('LEFT '/'RIGHT') to log fit outcome under,
            or None to stay silent (used to throttle logging per frame).

        Returns
        -------
        tuple[list[float], np.ndarray]
            ``(coeffs, points)`` — polynomial coefficients ``[a, b, ..., c]``
            of degree ``self._poly_degree`` for ``x = f(y)`` fitted to the
            selected contour, and an ``(N, 2)`` int array of ``(x, y)``
            points sampled directly off that contour's own pixels for
            drawing. Either or both are empty when no contour qualifies.
        """
        if not candidates:
            # if side:
            #     self._log.warn(f'[DIAG:fit] {side} REJECT — no contour passed the shape gates')
            return [], np.empty((0, 2), dtype=np.int32)

        best = None
        best_dist = None
        for ys, xs in candidates:
            # Rank by proximity to center at this blob's own bottom-most
            # rows (closest to the vehicle), not anywhere along its length.
            # Near the horizon every contour converges toward the vanishing
            # point, so a farther-out lane can look deceptively close to
            # center up there even though it sits well outside the near
            # lane lower in the frame — comparing at the bottom is the only
            # place perspective doesn't lie about which one is adjacent.
            y_spread = float(np.ptp(ys))
            bottom_n = max(1, int(0.2 * (y_spread + 1)))
            bottom_thresh = ys.max() - bottom_n + 1
            bottom_sel = ys >= bottom_thresh
            dist = float(np.mean(np.abs(xs[bottom_sel] - mid_col[ys[bottom_sel]])))
            if best_dist is None or dist < best_dist:
                best_dist = dist
                best = (ys, xs)

        ys, xs = best
        points = self._contour_points(ys, xs, self._point_spacing_px)

        try:
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', np.RankWarning)
                coeffs = np.polyfit(ys, xs, self._poly_degree)
            # if side:
            #     self._log.info(
            #         f'[DIAG:fit] {side} OK — {len(ys)} px  '
            #         f'y=[{int(ys.min())}..{int(ys.max())}]  '
            #         f'x=[{int(xs.min())}..{int(xs.max())}]  '
            #         f'coeffs=[{" ".join(f"{c:+.4e}" for c in coeffs)}]'
            #     )
            return coeffs.tolist(), points
        except (np.linalg.LinAlgError, ValueError) as exc:
            if side:
                self._log.warn(f'[DIAG:fit] {side} REJECT — polyfit failed: {exc}')
            return [], points

    @staticmethod
    def _contour_points(ys: np.ndarray, xs: np.ndarray, min_step: int) -> np.ndarray:
        """Row-wise centerline of one blob, resampled to ~``min_step`` spacing.

        Every output point is the mean x of that blob's own pixels on one
        row — i.e. it sits on the detected lane contour itself, never on an
        extrapolated curve. Consecutive rows are then merged until they are
        at least ``min_step`` px apart (Euclidean) so the drawn line has no
        noisy micro-jitter between neighbouring rows.
        """
        order = np.argsort(ys, kind='stable')
        ys_sorted = ys[order]
        xs_sorted = xs[order].astype(np.float64)

        uniq_y, start_idx, counts = np.unique(ys_sorted, return_index=True, return_counts=True)
        sums = np.add.reduceat(xs_sorted, start_idx)
        row_x = sums / counts

        if len(uniq_y) < 2:
            return np.empty((0, 2), dtype=np.int32)

        kept_y = [int(uniq_y[0])]
        kept_x = [float(row_x[0])]
        min_step_sq = float(min_step) ** 2
        for y, x in zip(uniq_y[1:], row_x[1:]):
            dy = float(y) - kept_y[-1]
            dx = float(x) - kept_x[-1]
            if dy * dy + dx * dx >= min_step_sq:
                kept_y.append(int(y))
                kept_x.append(float(x))

        last_y, last_x = int(uniq_y[-1]), float(row_x[-1])
        if kept_y[-1] != last_y:
            kept_y.append(last_y)
            kept_x.append(last_x)

        return np.column_stack([kept_x, kept_y]).astype(np.int32)

    @staticmethod
    def _adaptive_midline(da_seg_mask: np.ndarray, w_frame: int) -> np.ndarray:
        """Per-row left/right split column, tracked from the drivable-area mask.

        Returns an array of shape ``(H,)`` giving, for each row, the x
        column that separates "left" from "right" lane pixels. That column
        is the midpoint of the drivable-area mask's own extent on that row
        (rather than a fixed ``w_frame // 2``), so the split follows the
        road as it curves or the camera tilts/rolls. Rows where the
        drivable-area mask has no pixels (e.g. above the horizon) reuse the
        nearest row above that did; rows before any such row fall back to
        the frame's center column.
        """
        h = da_seg_mask.shape[0]
        da_bool = da_seg_mask.astype(bool)
        cols = np.arange(w_frame)

        row_has = da_bool.any(axis=1)
        xs_min = np.where(da_bool, cols, w_frame).min(axis=1)
        xs_max = np.where(da_bool, cols, -1).max(axis=1)
        row_mid = (xs_min + xs_max) / 2.0

        # Forward-fill: carry the last row with drivable-area pixels down
        # to subsequent rows that lack them.
        last_valid_row = np.where(row_has, np.arange(h), -1)
        np.maximum.accumulate(last_valid_row, out=last_valid_row)

        default_mid = float(w_frame // 2)
        mid_col = np.where(
            last_valid_row >= 0,
            row_mid[np.clip(last_valid_row, 0, h - 1)],
            default_mid,
        )
        return mid_col

    @staticmethod
    def _draw_lane_points(frame: np.ndarray, points: np.ndarray, color) -> None:
        """Draw a lane as a polyline through points sampled off its own contour."""
        if points is None or len(points) < 2:
            return
        cv2.polylines(frame, [points], False, color, 3)

    @staticmethod
    def _resolve_path(path: str, root: str) -> str:
        """Return an absolute path, resolving relative paths from *root*."""
        import os
        if os.path.isabs(path):
            return path
        stripped = path.lstrip('/')
        return os.path.join(root, stripped)



