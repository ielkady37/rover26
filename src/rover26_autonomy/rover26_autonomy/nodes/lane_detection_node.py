"""
═════════════════════════════════════════════════════════════════════════════════
lane_detection_node.py  —  rover26_autonomy
═════════════════════════════════════════════════════════════════════════════════

FUNCTIONALITY
─────────────
Detects white lane markings in camera images and computes steering commands
for lane following. Uses sliding-window tracking on bird's-eye-view transformed
images to fit lane-center polynomials, then computes lookahead-based steering
angles with curvature-dependent boost for sharp turns.

PIPELINE (per frame):
  1. Convert camera image (ROS Image) → BGR ndarray
  2. White mask extraction using HLS-based thresholding
  3. Perspective warp to bird's-eye view
  4. Sliding-window detector on binary lane mask
  5. Polynomial fitting (2nd-degree) to lane points
  6. Sharp-turn detection via curvature
  7. Compute steering angle via lookahead projection
  8. Publish LaneStatus message with polynomials + steering

TOPICS & SUBSCRIPTIONS
──────────────────────
Subscribed:
  • /camera/image    [sensor_msgs/Image]           ← camera driver
  • /imu             [sensor_msgs/Imu]             ← IMU driver (yaw_rate only)

Published:
  • /lane_status     [rover26/LaneStatus]          → lane_goal_publisher, pothole_detection

TF FRAMES
─────────
Input frame:  base_footprint (camera origin in URDF)
Output frame: (N/A — LaneStatus contains relative polynomials)

PARAMETERS (from config_params.py)
──────────────────────────────────
LaneDetection.*         Lane algorithm tuning (sliding window, steering boost, lookahead)
Physical.*              Physical constants (camera height, ground extent, calibration trim)
Debug.*                 Debug output (OpenCV windows, logging verbosity)

ALGORITHM NOTES
───────────────
• Sliding windows search for white pixels in the bird's-eye mask
• Lane separation is enforced to reject ghost detections
• Velocity-based extrapolation fills gaps when lanes are briefly lost
• Polynomial curvature is clipped to avoid unrealistic steering
• Lookahead distance adapts: shorter on sharp curves, longer on straights
• Steering angle is smoothed with IIR filter for jitter reduction
• Last-known polynomials are retained for up to MAX_MISS frames

═════════════════════════════════════════════════════════════════════════════════
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Imu
from rover26.msg import LaneStatus

from rover26_autonomy.utils.vision_utils import VisionUtils
from rover26_autonomy.config_params import IMG_W, IMG_H, LaneDetection, Debug, RosTopics


class LaneDetectionNode(Node):
    """
    Lane Detection Node — Detects lanes via vision and publishes steering commands.
    
    Attributes:
        bridge (CvBridge): ROS image message ↔ OpenCV conversion
        vision (VisionUtils): Bird's-eye warping, white-mask extraction
        last_left_coeffs (np.ndarray): Cached left-lane polynomial (a, b, c)
        last_right_coeffs (np.ndarray): Cached right-lane polynomial (a, b, c)
        left_miss, right_miss (int): Frame counter for lane dropout detection
        last_angle (float): Previous steering angle (for IIR smoothing)
        imu_yaw_rate (float): Latest yaw rate from IMU (for steering blending)
        consecutive_sharp (int): Hysteresis counter for sharp-turn detection
    """

    def __init__(self):
        super().__init__('lane_detection_node')
        self.bridge = CvBridge()
        self.vision = VisionUtils()
        self.last_left_coeffs = None
        self.last_right_coeffs = None
        self.left_miss = 0
        self.right_miss = 0
        self.last_angle = 0.0
        self.imu_yaw_rate = 0.0
        self.consecutive_sharp = 0

        # ── Pre-compute integer half-separation once ──────────────────────────
        self._half_sep = int(LaneDetection.MIN_SEP_PX // 2)

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(Image, RosTopics.CAMERA_IMAGE, self._image_cb, 10)
        self.create_subscription(Imu, RosTopics.IMU, self._imu_cb, 10)
        
        # ── Publishers ────────────────────────────────────────────────────────
        self._lane_pub = self.create_publisher(LaneStatus, RosTopics.LANE_STATUS, 10)

        self._create_debug_windows()
        self.get_logger().info("Lane Detection started")

    def _create_debug_windows(self):
        """Open OpenCV display windows when debug display is enabled."""
        if not Debug.ENABLE_DISPLAY:
            return
        if LaneDetection.SHOW_BIRDSEYE:
            cv2.namedWindow("Bird's Eye View", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Bird's Eye View", 900, 600)
        if LaneDetection.SHOW_OVERLAY:
            cv2.namedWindow("Lane Detection Live", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Lane Detection Live", 900, 600)

    def _imu_cb(self, msg: Imu):
        """Cache the latest yaw rate from the IMU for steering blending."""
        self.imu_yaw_rate = msg.angular_velocity.z

    def _image_cb(self, msg: Image):
        """Convert ROS Image message to BGR ndarray and kick off processing."""
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if img is not None:
                self._process(img)
        except Exception as e:
            self.get_logger().error(f"Image callback failed: {e}")

    def _process(self, img: np.ndarray):
        """Main per-frame pipeline: warp → sliding window → fit → steer → publish."""
        try:
            img = cv2.resize(img, (IMG_W, IMG_H))
            h, w = img.shape[:2]

            white_mask = self.vision.get_white_mask(img)
            bird_mask = self.vision.warp(white_mask)
            bird_color = cv2.cvtColor(bird_mask, cv2.COLOR_GRAY2BGR)

            left_pts, right_pts = self._sliding_window(bird_mask)
            left_pts = self._prune_points(left_pts)
            right_pts = self._prune_points(right_pts)

            left_coeffs = self._fit_lane(left_pts)
            right_coeffs = self._fit_lane(right_pts)

            is_sharp_turn = self._detect_sharp_turn(left_coeffs, right_coeffs)
            self._update_lane(left_coeffs, right_coeffs)

            if left_coeffs is not None and right_coeffs is not None:
                angle, magnitude = self._get_lookahead(h, w, is_sharp_turn)
                if angle is not None:
                    angle = self._apply_sharp_boost(angle, is_sharp_turn)
                    # IIR low-pass filter to smooth out jitter in the steering angle
                    angle = (1 - LaneDetection.ANGLE_SMOOTHING) * self.last_angle + \
                            LaneDetection.ANGLE_SMOOTHING * angle
                    self.last_angle = angle
            else:
                angle = None
                magnitude = 0.0

            self._publish(angle, magnitude, left_coeffs, right_coeffs)

            if Debug.ENABLE_DISPLAY:
                bird_view = self._draw_birdseye(bird_color, h, w, left_pts, right_pts, angle, is_sharp_turn)
                overlay = self._draw_overlay(img.copy(), h, w, angle, is_sharp_turn)
                if angle is not None:
                    overlay = self._draw_steering_indicator(overlay, angle, is_sharp_turn)
                    bird_view = self._draw_steering_indicator(bird_view, angle, is_sharp_turn)
                self._show_debug_windows(bird_view, overlay)

        except Exception as e:
            self.get_logger().error(f"Process failed: {e}")

    def _sliding_window(self, bird_mask: np.ndarray):
        """
        Sliding-window lane tracker operating on the bird's-eye binary mask.

        Returns two lists of (x, y) integer pixel coordinates — one per lane side.
        """
        h, w = bird_mask.shape

        # Height of each search window in pixels (always an integer)
        win_h = h // LaneDetection.N_WINDOWS

        # Column histogram of the bottom 2/3 of the image to seed left/right peaks
        hist = np.sum(bird_mask[h // 3:, :], axis=0)
        hist = cv2.GaussianBlur(hist.astype(np.float32), (15, 1), 0)

        if np.max(hist) <= 0:
            # No white pixels at all — return empty point lists
            return [], []

        # ── Seed X positions from previous polynomial or histogram peak ───────
        # int() ensures left_x / right_x are always Python ints (not numpy types)
        left_x = int(np.polyval(self.last_left_coeffs, h - win_h // 2)) \
            if self.last_left_coeffs is not None else int(np.argmax(hist[:w // 2]))
        right_x = int(np.polyval(self.last_right_coeffs, h - win_h // 2)) \
            if self.last_right_coeffs is not None else int(np.argmax(hist[w // 2:]) + w // 2)

        left_x = np.clip(left_x, 0, w - 1)
        right_x = np.clip(right_x, 0, w - 1)

        # ── Enforce minimum lane separation ────────────────────────────────────
        if abs(right_x - left_x) < LaneDetection.MIN_SEP_PX:
            center = int((left_x + right_x) // 2)
            left_x = max(0, center - self._half_sep)      # guaranteed int
            right_x = min(w - 1, center + self._half_sep)  # guaranteed int

        left_pts = []
        right_pts = []
        left_gap = right_gap = 0
        left_vx = right_vx = 0.0

        for win in range(LaneDetection.N_WINDOWS):
            # Pixel row bounds for this search window
            y_low = h - (win + 1) * win_h
            y_high = h - win * win_h
            y_mid = (y_low + y_high) // 2

            # Widen the search margin slightly for lower windows (farther ahead)
            margin = LaneDetection.MARGIN + (LaneDetection.N_WINDOWS - win) * 2

            # ── Left lane window ──────────────────────────────────────────────
            xl1, xl2 = max(0, left_x - margin), min(w, left_x + margin)
            ln = np.where(bird_mask[y_low:y_high, xl1:xl2] > 0)
            if len(ln[1]) >= LaneDetection.MIN_PIX:
                new_x = int(np.mean(ln[1])) + xl1
                if abs(new_x - right_x) > LaneDetection.MIN_SEP_PX:
                    # Exponential moving average on the velocity for smoother tracking
                    left_vx = (new_x - left_x) * 0.7 + left_vx * 0.3
                    left_x = new_x
                    left_gap = 0
                    left_pts.append((left_x, y_mid))
            else:
                left_gap += 1
                if left_gap <= LaneDetection.MAX_GAP_WINDOWS:
                    # Extrapolate position using last known velocity
                    left_x = int(np.clip(left_x + left_vx, 0, w - 1))

            # ── Right lane window ─────────────────────────────────────────────
            xr1, xr2 = max(0, right_x - margin), min(w, right_x + margin)
            rn = np.where(bird_mask[y_low:y_high, xr1:xr2] > 0)
            if len(rn[1]) >= LaneDetection.MIN_PIX:
                new_x = int(np.mean(rn[1])) + xr1
                if abs(new_x - left_x) > LaneDetection.MIN_SEP_PX:
                    right_vx = (new_x - right_x) * 0.7 + right_vx * 0.3
                    right_x = new_x
                    right_gap = 0
                    right_pts.append((right_x, y_mid))
            else:
                right_gap += 1
                if right_gap <= LaneDetection.MAX_GAP_WINDOWS:
                    right_x = int(np.clip(right_x + right_vx, 0, w - 1))

        return left_pts, right_pts

    def _prune_points(self, points):
        """
        Remove or interpolate points where the X jump exceeds MAX_X_DRIFT.

        Prevents wild outlier detections from corrupting the polynomial fit.
        """
        if len(points) < 3:
            return points
        pruned = [points[0]]
        for p in points[1:]:
            if abs(p[0] - pruned[-1][0]) < LaneDetection.MAX_X_DRIFT:
                pruned.append(p)
            else:
                # Bridge the gap with a midpoint instead of dropping the point
                interp_x = (pruned[-1][0] + p[0]) // 2
                pruned.append((interp_x, p[1]))
        return pruned

    def _fit_lane(self, points):
        """
        Fit a 2nd-degree polynomial to detected lane points (x = f(y)).

        Returns None if there are fewer than 6 valid points.
        Falls back to a linear fit if the quadratic curvature exceeds MAX_CURVATURE.
        """
        if len(points) < 6:
            return None
        xs = np.array([p[0] for p in points])
        ys = np.array([p[1] for p in points])
        try:
            coeffs = np.polyfit(ys, xs, 2)
            if abs(coeffs[0]) > LaneDetection.MAX_CURVATURE:
                # Curvature too high → degenerate detection; fall back to linear
                coeffs_lin = np.polyfit(ys, xs, 1)
                coeffs = np.array([0.0, coeffs_lin[0], coeffs_lin[1]])
            return coeffs
        except Exception:
            return None

    def _update_lane(self, l_c, r_c):
        """
        Update cached lane polynomials.

        If a lane is not detected for MAX_MISS consecutive frames, it is cleared
        so stale data does not mislead the steering controller.
        """
        if l_c is not None:
            self.last_left_coeffs = l_c
            self.left_miss = 0
        else:
            self.left_miss += 1
            if self.left_miss > LaneDetection.MAX_MISS:
                self.last_left_coeffs = None

        if r_c is not None:
            self.last_right_coeffs = r_c
            self.right_miss = 0
        else:
            self.right_miss += 1
            if self.right_miss > LaneDetection.MAX_MISS:
                self.last_right_coeffs = None

    def _get_lookahead(self, h: int, w: int, is_sharp_turn: bool):
        """
        Compute the steering angle by projecting a lookahead point along the
        lane centre-line.

        Lookahead distance adapts to curvature: farther ahead on straight
        segments, closer on sharp curves for tighter correction.

        Returns (angle_rad, magnitude) where magnitude is 0..1.
        """
        left = self.last_left_coeffs
        right = self.last_right_coeffs

        # Determine the dominant curvature from whichever lane is detected
        curv = 0.0
        if left is not None:
            curv = max(curv, abs(left[0]))
        if right is not None:
            curv = max(curv, abs(right[0]))

        # Choose lookahead ratio based on turn severity
        if is_sharp_turn or curv > 0.045:
            lookahead_ratio = LaneDetection.LOOKAHEAD_RATIO_SHARP
        elif curv > 0.025:
            lookahead_ratio = 0.65
        else:
            lookahead_ratio = LaneDetection.LOOKAHEAD_RATIO_NORMAL

        y_look = int(h * lookahead_ratio)
        x_target = (np.polyval(left, y_look) + np.polyval(right, y_look)) / 2.0

        # arctan2(lateral_error, longitudinal_distance) → steering angle in radians
        angle = np.arctan2(x_target - w / 2.0, h - y_look)
        magnitude = float(np.clip(abs(angle) / (np.pi / 3.0), 0.0, 1.0))

        return float(angle), magnitude

    def _detect_sharp_turn(self, left_coeffs, right_coeffs):
        """
        Return True if the rover is in or approaching a sharp curve.

        Uses a hysteresis counter so the flag stays True for a few frames after
        the curvature drops (prevents rapid on/off switching of the boost).
        """
        if left_coeffs is not None and abs(left_coeffs[0]) > LaneDetection.CURVATURE_THRESHOLD:
            self.consecutive_sharp = min(self.consecutive_sharp + 1, 6)
            return True
        if right_coeffs is not None and abs(right_coeffs[0]) > LaneDetection.CURVATURE_THRESHOLD:
            self.consecutive_sharp = min(self.consecutive_sharp + 1, 6)
            return True
        self.consecutive_sharp = max(self.consecutive_sharp - 1, 0)
        return self.consecutive_sharp >= 3

    def _apply_sharp_boost(self, angle: float, is_sharp: bool):
        """
        Multiply the steering angle by SHARP_TURN_BOOST on tight curves and
        clip the result to MAX_STEERING_ANGLE.
        """
        if not is_sharp:
            return angle
        boosted = angle * LaneDetection.SHARP_TURN_BOOST
        return float(np.clip(boosted, -LaneDetection.MAX_STEERING_ANGLE,
                             LaneDetection.MAX_STEERING_ANGLE))

    def _publish(self, angle, magnitude, left_c, right_c):
        """Publish a LaneStatus message when both lane polynomials are valid."""
        if left_c is None or right_c is None:
            return

        msg = LaneStatus()
        msg.left_detected = True
        msg.right_detected = True
        msg.imu_yaw_rate = float(self.imu_yaw_rate)
        msg.left_coeffs = [float(c) for c in left_c]
        msg.right_coeffs = [float(c) for c in right_c]

        if angle is not None:
            msg.steering_angle = float(angle)
            msg.steering_magnitude = float(magnitude)

        self._lane_pub.publish(msg)

    def _draw_birdseye(self, bird_color, h, w, left_pts, right_pts, angle, is_sharp_turn):
        """
        Overlay lane detection results on the bird's-eye colour image for
        debug visualisation.
        """
        # Raw detected window points
        for (x, y) in left_pts:
            cv2.circle(bird_color, (int(x), int(y)), 5, (0, 255, 0), -1)
        for (x, y) in right_pts:
            cv2.circle(bird_color, (int(x), int(y)), 5, (0, 0, 255), -1)

        y_vals = np.linspace(0, h - 1, 100).astype(int)

        # Draw left polynomial curve (green)
        if self.last_left_coeffs is not None:
            lx = np.polyval(self.last_left_coeffs, y_vals).astype(int)
            for i in range(len(y_vals) - 1):
                if 0 <= lx[i] < w and 0 <= lx[i + 1] < w:
                    cv2.line(bird_color, (lx[i], y_vals[i]), (lx[i + 1], y_vals[i + 1]), (0, 255, 0), 4)

        # Draw right polynomial curve (red)
        if self.last_right_coeffs is not None:
            rx = np.polyval(self.last_right_coeffs, y_vals).astype(int)
            for i in range(len(y_vals) - 1):
                if 0 <= rx[i] < w and 0 <= rx[i + 1] < w:
                    cv2.line(bird_color, (rx[i], y_vals[i]), (rx[i + 1], y_vals[i + 1]), (0, 0, 255), 4)

        # Draw centre-line (cyan)
        if self.last_left_coeffs is not None and self.last_right_coeffs is not None:
            cx = ((np.polyval(self.last_left_coeffs, y_vals) +
                   np.polyval(self.last_right_coeffs, y_vals)) / 2).astype(int)
            for i in range(len(y_vals) - 1):
                if 0 <= cx[i] < w and 0 <= cx[i + 1] < w:
                    cv2.line(bird_color, (cx[i], y_vals[i]), (cx[i + 1], y_vals[i + 1]), (0, 255, 255), 3)

        # ── HUD overlay ───────────────────────────────────────────────────────
        cv2.rectangle(bird_color, (5, 5), (400, 130), (0, 0, 0), -1)
        cv2.putText(bird_color, "BIRD'S EYE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)

        left_ok  = 'ok' if self.last_left_coeffs  is not None else 'No'
        right_ok = 'ok' if self.last_right_coeffs is not None else 'No'
        cv2.putText(bird_color, f"L: {left_ok}",  (10, 60),  cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        cv2.putText(bird_color, f"R: {right_ok}", (10, 85),  cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
        if angle is not None:
            cv2.putText(bird_color, f"Angle: {np.degrees(angle):+.1f}°", (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        return bird_color

    def _draw_overlay(self, img, h, w, angle, is_sharp_turn):
        """
        Project the fitted lane polynomials back onto the original camera frame
        for an intuitive driver-view overlay.
        """
        left  = self.last_left_coeffs
        right = self.last_right_coeffs

        def bird_to_orig(coeffs):
            """Inverse-warp a set of polynomial-sampled points to camera space."""
            if coeffs is None:
                return np.array([])
            y_range = np.linspace(0, h - 1, 40)
            xs = np.polyval(coeffs, y_range).astype(int)
            pts = np.array([[x, int(y)] for x, y in zip(xs, y_range) if 0 <= x < w])
            if len(pts) < 2:
                return np.array([])
            return self.vision.unwarp_pts(pts, img.shape)

        if left is not None:
            pts = bird_to_orig(left)
            if len(pts) > 1:
                cv2.polylines(img, [pts.reshape(-1, 1, 2)], False, (0, 255, 0), 4)
        if right is not None:
            pts = bird_to_orig(right)
            if len(pts) > 1:
                cv2.polylines(img, [pts.reshape(-1, 1, 2)], False, (0, 0, 255), 4)

        # BUG FIX: same as _draw_birdseye — use `is not None` to check numpy arrays.
        left_ok  = '✓' if left  is not None else '✗'
        right_ok = '✓' if right is not None else '✗'
        cv2.rectangle(img, (5, 5), (400, 90), (0, 0, 0), -1)
        cv2.putText(img, f"L: {left_ok}  R: {right_ok}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        return img

    def _draw_steering_indicator(self, img, angle, is_sharp_turn):
        """Draw an arrow at the bottom of the frame showing the steering direction."""
        h, w = img.shape[:2]
        cx = w // 2
        tx = int(cx + angle * 170)
        tx = max(40, min(w - 40, tx))
        # Red arrow = sharp turn; cyan arrow = normal steering
        color = (0, 0, 255) if is_sharp_turn else (0, 255, 255)
        cv2.arrowedLine(img, (cx, h - 80), (tx, h - 80), color, 6, tipLength=0.4)
        return img

    def _show_debug_windows(self, bird_view, overlay):
        """Push frames to the OpenCV debug windows and pump the GUI event loop."""
        if LaneDetection.SHOW_BIRDSEYE:
            cv2.imshow("Bird's Eye View", bird_view)
        if LaneDetection.SHOW_OVERLAY:
            cv2.imshow("Lane Detection Live", overlay)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        if Debug.ENABLE_DISPLAY:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()