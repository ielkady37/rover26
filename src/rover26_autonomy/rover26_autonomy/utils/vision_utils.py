"""
═════════════════════════════════════════════════════════════════════════════════
vision_utils.py  —  rover26_autonomy/utils
═════════════════════════════════════════════════════════════════════════════════

FUNCTIONALITY
─────────────
Shared computer-vision helpers used by lane_detection_node and
pothole_detection_node.  Centralises all camera-specific geometry so that
changing the camera mount, FOV, or resolution requires edits to only this
file (and config_params.py).

Provides three capabilities:

  1. White-pixel mask extraction
       Detects white lane markings in a BGR frame using a two-strategy pipeline:
         • HLS lightness + low-saturation threshold  →  robust to shadows
         • Canny edges constrained to white regions  →  sharp boundary capture
       A morphological open+close cleans noise and closes small gaps.

  2. Perspective warp  (camera frame → bird's-eye view)
       A homography (cv2.getPerspectiveTransform) maps the forward-camera
       trapezoid that covers the road ahead to a top-down rectangle.
       After warping, lane lines appear vertical and parallel, making
       polynomial fitting straightforward.

  3. Coordinate transforms
       unwarp_pts  — map pixel coordinates FROM bird's-eye BACK to camera frame
       birdseye_to_ground — convert a bird's-eye pixel to real-world ground
                            coordinates (metres ahead, metres lateral)

CALIBRATION ASSUMPTIONS
────────────────────────
  • Camera mounted at Physical.CAMERA_HEIGHT_M above ground, pointing ahead.
  • Image resolution must match IMG_WIDTH × IMG_HEIGHT in config_params.py.
  • SRC trapezoid is tuned to match where the left/right lane lines actually
    appear in the raw camera image at y = TOP_Y.

TUNING GUIDE (warp calibration)
────────────────────────────────
  Open the 'Bird's-eye' debug window in lane_detection_node and look at the
  warped white mask.  Adjust ONLY the TOP-edge X coordinates:

    Still V-shape (lanes converge)
      → tops too close together → decrease LEFT_TOP / increase RIGHT_TOP

    Still A-shape (lanes diverge)
      → over-corrected           → increase LEFT_TOP / decrease RIGHT_TOP

    Vertical and parallel ✓  →  calibration correct

PARAMETERS (from config_params.py)
──────────────────────────────────
Physical.*         PX_PER_METRE, GROUND_WIDTH_M, GROUND_HEIGHT_M
IMG_WIDTH          Camera frame width  (pixels)
IMG_HEIGHT         Camera frame height (pixels)

═════════════════════════════════════════════════════════════════════════════════
"""

import cv2
import numpy as np

# Image dimensions and physical calibration from the central config.
from rover26_autonomy.config_params import (
    IMG_W,
    IMG_H,
    GROUND_HEIGHT_M,
    GROUND_WIDTH_M,
)


# ═════════════════════════════════════════════════════════════════════════════
#  VISION UTILS
# ═════════════════════════════════════════════════════════════════════════════

class VisionUtils:
    """
    Perspective warp and white-mask helpers shared across all vision nodes.

    Instantiate once per node — the constructor pre-computes the homography
    matrices and morphological kernels so they are not rebuilt on every frame.

    Attributes:
        M       (np.ndarray): 3×3 forward homography  (camera → bird's-eye).
        M_inv   (np.ndarray): 3×3 inverse homography  (bird's-eye → camera).
        SRC     (np.ndarray): Source trapezoid corners in the camera frame.
        DST     (np.ndarray): Destination rectangle corners in bird's-eye view.
        LEFT_TOP  (int): X-coordinate of the left  lane at TOP_Y in camera frame.
        RIGHT_TOP (int): X-coordinate of the right lane at TOP_Y in camera frame.
        TOP_Y     (int): Y-row of the top edge of the ROI trapezoid.
        MARGIN    (int): Left/right pixel margin in the warped (bird's-eye) image.
    """

    # =========================================================================
    #  WARP CALIBRATION POINTS
    # =========================================================================

    # ── Top-edge X coordinates of the lane lines in the RAW camera image ──────
    #
    # These must be measured from the actual live camera stream.
    # Find the Y row where both lane lines are clearly visible (TOP_Y below),
    # then read off the pixel X coordinate of each line at that row.
    #
    # TUNING: adjust only LEFT_TOP and RIGHT_TOP (not the bottom corners).
    # The bottom corners span the full image width to include the entire ground
    # patch at the closest visible distance.
    LEFT_TOP:  int = 200   # x of left  lane at y = TOP_Y  (raw camera frame)
    RIGHT_TOP: int = 440   # x of right lane at y = TOP_Y  (raw camera frame)
    TOP_Y:     int = 170   # y-row of the ROI top edge

    # ── Source trapezoid (4 corners, in order: BL, BR, TR, TL) ───────────────
    # Covers the road corridor from the closest visible ground (bottom) to the
    # vanishing-point region (top).
    SRC: np.ndarray = np.float32([
        [0,         IMG_H - 220],   # Bottom-left  — full image width at ground
        [IMG_W,     IMG_H - 220],   # Bottom-right
        [RIGHT_TOP, TOP_Y      ],   # Top-right    — where the right lane is
        [LEFT_TOP,  TOP_Y      ],   # Top-left     — where the left  lane is
    ])

    # ── Destination rectangle (same corner order as SRC) ─────────────────────
    # A perfect rectangle so warped lane lines become vertical and parallel.
    MARGIN: int = 80
    DST: np.ndarray = np.float32([
        [MARGIN,          IMG_H],   # Bottom-left
        [IMG_W - MARGIN,  IMG_H],   # Bottom-right
        [IMG_W - MARGIN,  0    ],   # Top-right
        [MARGIN,          0    ],   # Top-left
    ])

    # =========================================================================
    #  CONSTRUCTOR
    # =========================================================================

    def __init__(self) -> None:
        """
        Pre-compute all invariant objects so per-frame methods stay fast.

        Computes:
          - Forward and inverse perspective transform matrices (M, M_inv).
          - Three morphological kernels: open (noise removal), close (gap fill),
            dilate (edge expansion for white-edge detection strategy).
          - CLAHE contrast enhancer for robust lightness normalisation.
          - Initial ROI mask derived from the SRC trapezoid.
        """

        # ── Perspective homographies ───────────────────────────────────────────
        # M     : maps SRC corners → DST corners  (camera → bird's-eye)
        # M_inv : maps DST corners → SRC corners  (bird's-eye → camera)
        self.M:     np.ndarray = cv2.getPerspectiveTransform(self.SRC, self.DST)
        self.M_inv: np.ndarray = cv2.getPerspectiveTransform(self.DST, self.SRC)

        # ── Morphological kernels ─────────────────────────────────────────────
        # open_kernel  — small ellipse removes isolated noise pixels
        # close_kernel — larger ellipse closes small intra-lane gaps
        # dilate_kernel — rectangle expands Canny edges before ANDing with mask
        self._open_kernel   = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self._close_kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        self._dilate_kernel = cv2.getStructuringElement(cv2.MORPH_RECT,    (3, 3))

        # ── CLAHE contrast enhancer ───────────────────────────────────────────
        # Contrast Limited Adaptive Histogram Equalisation normalises the
        # lightness channel across tiles, making white-lane detection robust
        # to shadows, glare, and uneven illumination.
        # clipLimit=3.0  — prevents over-amplification of noise
        # tileGridSize   — 8×8 tiles balance local vs global contrast
        self._clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))

        # ── ROI mask ─────────────────────────────────────────────────────────
        # Binary mask shaped like the SRC trapezoid. Applied after the white-
        # pixel detection to discard detections outside the road corridor.
        self._roi_mask: np.ndarray | None = None
        self._update_roi_mask()

    # =========================================================================
    #  ROI MASK
    # =========================================================================

    def _update_roi_mask(self) -> None:
        """
        (Re)build the ROI binary mask from the SRC trapezoid.

        The mask is a filled polygon on a black background the same size as
        the input camera image.  Pixels inside the trapezoid are white (255);
        everything outside is black (0).  Applied via bitwise_and after the
        white-pixel detection step to reject any detections outside the road
        area (e.g. sky, rover body reflections).

        Called once in __init__.  If SRC is ever updated at runtime, call this
        method again to refresh the mask.
        """
        self._roi_mask = np.zeros((IMG_H, IMG_W), dtype=np.uint8)
        cv2.fillPoly(self._roi_mask, [self.SRC.astype(np.int32)], 255)

    # =========================================================================
    #  WHITE-PIXEL MASK
    # =========================================================================

    def _normalize_brightness(self, l_ch: np.ndarray) -> np.ndarray:
        """Normalize brightness using CLAHE and Gaussian blur."""
        l_eq   = self._clahe.apply(l_ch)
        l_blur = cv2.GaussianBlur(l_eq, (5, 5), 0)
        return l_blur

    def _extract_hls_white(self, l_blur: np.ndarray, s_ch: np.ndarray) -> np.ndarray:
        """Extract white regions using HLS thresholding."""
        _, white_lightness = cv2.threshold(l_blur, 180, 255, cv2.THRESH_BINARY)
        _, low_saturation  = cv2.threshold(s_ch, 150, 255, cv2.THRESH_BINARY_INV)
        return cv2.bitwise_and(white_lightness, low_saturation)

    def _extract_canny_edges(self, l_blur: np.ndarray, white_mask: np.ndarray) -> np.ndarray:
        """Extract Canny edges constrained to white regions."""
        edges          = cv2.Canny(l_blur, 50, 150)
        edges_dilated  = cv2.dilate(edges, self._dilate_kernel, iterations=1)
        return cv2.bitwise_and(edges_dilated, white_mask)

    def get_white_mask(self, img: np.ndarray, apply_roi: bool = True) -> np.ndarray:
        """
        Extract a binary mask of white lane markings from a BGR camera frame.

        Uses two complementary strategies and combines them with a bitwise OR:

        Strategy A — HLS thresholding (bulk white-region detection)
          High lightness (>180) AND low saturation (<150).

        Strategy B — Canny edge detection (sharp boundary capture)
          Canny edges constrained to white regions.

        Post-processing:
          Morphological OPEN (3×3) removes noise, CLOSE (7×7) fills gaps.

        Args:
            img:       BGR camera frame, any resolution. Resized to match IMG_W × IMG_H.
            apply_roi: When True, applies ROI trapezoid mask (lane detection).
                       When False, searches full frame (pothole detection).

        Returns:
            Binary mask (uint8, 0 or 255), same size as IMG_H × IMG_W.
        """
        # Resize if necessary
        if img.shape[:2] != (IMG_H, IMG_W):
            img = cv2.resize(img, (IMG_W, IMG_H))

        # Convert to HLS and normalize brightness
        hls    = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        l_ch   = hls[:, :, 1]
        s_ch   = hls[:, :, 2]
        l_blur = self._normalize_brightness(l_ch)

        # Extract white regions using two strategies
        white_mask  = self._extract_hls_white(l_blur, s_ch)
        white_edges = self._extract_canny_edges(l_blur, white_mask)

        # Combine both strategies
        mask = cv2.bitwise_or(white_mask, white_edges)

        # Apply ROI mask if needed
        if apply_roi:
            self._update_roi_mask()
            mask = cv2.bitwise_and(mask, self._roi_mask)

        # Morphological cleanup: OPEN removes noise, CLOSE fills gaps
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._open_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._close_kernel)

        return mask

    # =========================================================================
    #  PERSPECTIVE TRANSFORMS
    # =========================================================================

    def warp(self, img: np.ndarray) -> np.ndarray:
        """
        Apply the forward perspective transform to produce a bird's-eye view.

        The SRC trapezoid (road corridor in the camera frame) is mapped to the
        DST rectangle (top-down view with vertical, parallel lane lines).

        After warping:
          • Left lane line  → vertical line near x = MARGIN
          • Right lane line → vertical line near x = IMG_W - MARGIN
          • Rows closer to y = IMG_H correspond to ground closer to the rover.
          • Rows closer to y = 0       correspond to ground further ahead.

        Args:
            img: BGR or binary (uint8) image in camera frame coordinates.
                 Any resolution — the output is always (img.shape[1], img.shape[0]).

        Returns:
            Bird's-eye view image, same width and height as the input.
        """
        h, w = img.shape[:2]
        return cv2.warpPerspective(img, self.M, (w, h))

    def unwarp_pts(self, pts: np.ndarray | list, img_shape: tuple) -> np.ndarray:
        """
        Map 2-D pixel coordinates from bird's-eye view back to the camera frame.

        Useful for drawing lane polynomial curves on the original camera image
        or for validating perspective calibration visually.

        Args:
            pts:       Array-like of (x, y) pixel coordinates in bird's-eye space.
                       Shape can be (N, 2) or any form accepted by np.float32().
            img_shape: Not used in computation but kept for API symmetry with
                       callers that pass img.shape[:2].

        Returns:
            Integer (x, y) pixel coordinates in the original camera frame,
            shape (N, 2).  Returns an empty (0, 2) array if pts is empty.
        """
        if len(pts) == 0:
            return np.array([], dtype=np.int32).reshape(0, 2)

        # perspectiveTransform requires shape (N, 1, 2) with float32
        pts_f  = np.float32(pts).reshape(-1, 1, 2)
        warped = cv2.perspectiveTransform(pts_f, self.M_inv)
        return warped.reshape(-1, 2).astype(int)

    # =========================================================================
    #  GROUND-PLANE COORDINATE PROJECTION
    # =========================================================================

    def birdseye_to_ground(
        self,
        px: float,
        py: float,
        rover_offset: float = 2.0,
    ) -> tuple[float, float]:
        """
        Convert a bird's-eye view pixel to real-world ground coordinates.

        The conversion goes through the inverse homography (bird's-eye → camera
        frame) and then applies a linear pixel-to-metric mapping based on the
        physical camera calibration in Physical / constants.py.

        This method inherits the perspective calibration from the SRC/DST
        trapezoid directly, so it is consistent with the lane polynomial
        fits produced by lane_detection_node.

        Coordinate convention (rover body frame):
          +x_forward → straight ahead  (metres)
          +y_left    → to the rover's left (metres)

        Args:
            px:           Bird's-eye pixel X coordinate (horizontal).
            py:           Bird's-eye pixel Y coordinate (vertical).
                          py = 0 → furthest ahead visible;
                          py = IMG_H → closest ground to the rover.
            rover_offset: Lateral correction applied after the metric
                          conversion (metres). Compensates for the camera
                          not being perfectly centred on the rover body.
                          Matches Physical.LATERAL_BIAS_M in typical usage.

        Returns:
            (x_forward_m, y_left_m) — ground-plane position in metres,
            or (0.0, 0.0) if the inverse transform fails.
        """

        # ── Step 1: Inverse warp — bird's-eye pixel → camera-frame pixel ──────
        pts      = np.array([[[px, py]]], dtype=np.float32)
        orig_pts = cv2.perspectiveTransform(pts, self.M_inv)

        if orig_pts is None or len(orig_pts) == 0:
            return 0.0, 0.0   # Guard: transform failed

        orig_x, orig_y = orig_pts[0][0]   # Camera-frame pixel coordinates

        # ── Step 2: Camera-frame pixel → ground-plane metric ──────────────────
        # Linear mapping derived from the camera calibration:
        #   orig_y = 0       → top of image  → furthest ahead  → x_fwd = GROUND_HEIGHT_M
        #   orig_y = IMG_H   → bottom of img → rover position  → x_fwd = 0
        #   orig_x = 0       → left edge     → y_left = +GROUND_WIDTH_M / 2
        #   orig_x = IMG_W   → right edge    → y_left = -GROUND_WIDTH_M / 2
        x_forward = (1.0 - orig_y / IMG_H) * GROUND_HEIGHT_M
        y_left    = (0.5 - orig_x / IMG_W) * GROUND_WIDTH_M

        # ── Step 3: Apply rover lateral offset correction ─────────────────────
        # If the camera is mounted slightly off-centre, this shifts the
        # coordinate origin to the rover's actual centre line.
        y_left += rover_offset

        return x_forward, y_left