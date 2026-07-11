#!/usr/bin/env python3
"""
simple_lane_goal.py  —  rover26_autonomy

Ultra-simple straight-path lane centering. On every /lane_detection message:

  1. Fuse left + right lane polynomials into a centre line.
  2. Sample that centre line at py = IMG_H / 2 (the vertical centre of the
     frame) — a fixed row, no curvature-based look-ahead.
  3. Compare that pixel position to IMG_W / 2 (the horizontal centre of the
     frame) to get how far off-centre the rover is.
  4. Convert to a Nav2 goal straight ahead of the rover, print it, send it.

No sharp-turn handling, no smoothing, no outer bias, no recovery spin — just
"keep the lane centre at the frame centre."
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from interfaces.msg import LaneDetectionResult
from nav2_msgs.action import NavigateToPose

from rover26_autonomy.config_params import IMG_W, IMG_H, Physical, RosTopics

# Where this node publishes the annotated_frame it receives, with the goal
# point drawn on top of the already-drawn lane lines. View with e.g.:
#   ros2 run rqt_image_view rqt_image_view /simple_lane_goal/annotated
ANNOTATED_GOAL_TOPIC = '/simple_lane_goal/annotated'


class SimpleLaneGoal(Node):
    def __init__(self):
        super().__init__('simple_lane_goal')

        self.current_odom = None
        self._nav_active = False
        self._goal_generation = 0
        self._bridge = CvBridge()

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_subscription(LaneDetectionResult, RosTopics.LANE_DETECTION, self._lane_cb, 10)
        self.create_subscription(Odometry, RosTopics.ODOM, self._odom_cb, 10)
        self._annotated_pub = self.create_publisher(Image, ANNOTATED_GOAL_TOPIC, 10)

        self.get_logger().info(
            f'simple_lane_goal started — centring at py={IMG_H // 2} (frame centre), '
            f'no curve/sharp handling. Annotated goal image on {ANNOTATED_GOAL_TOPIC}'
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self.current_odom = msg.pose.pose

    def _draw_and_publish(self, msg: LaneDetectionResult, py: int,
                           centre_px: float | None, stamp) -> None:
        """Draw the frame-centre reference lines + goal point on top of the
        already-annotated lane frame, and republish it."""
        try:
            frame = self._bridge.imgmsg_to_cv2(msg.annotated_frame, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'[simple] annotated_frame decode failed: {exc}', throttle_duration_sec=2.0)
            return

        # Reference crosshair: horizontal centre column, and the fixed row
        # we sample the lane centre at.
        cv2.line(frame, (IMG_W // 2, 0), (IMG_W // 2, IMG_H), (255, 255, 255), 1)
        cv2.line(frame, (0, py), (IMG_W, py), (255, 255, 255), 1)

        if centre_px is not None:
            goal_pt = (int(round(centre_px)), py)
            cv2.circle(frame, goal_pt, 10, (0, 255, 255), -1)
            cv2.putText(frame, 'GOAL', (goal_pt[0] + 12, goal_pt[1] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        out = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out.header.stamp = stamp
        out.header.frame_id = msg.header.frame_id
        self._annotated_pub.publish(out)

    def _lane_cb(self, msg: LaneDetectionResult) -> None:
        self.get_logger().info(
            f'[simple] raw left={list(msg.left_lane_coeffs)} right={list(msg.right_lane_coeffs)}'
        )

        py = IMG_H // 2
        stamp = self.get_clock().now().to_msg()

        if len(msg.left_lane_coeffs) != 3 or len(msg.right_lane_coeffs) != 3:
            self.get_logger().warn('[simple] lane side missing — skipping')
            self._draw_and_publish(msg, py, None, stamp)
            return

        left_c   = np.array(msg.left_lane_coeffs,  dtype=float)
        right_c  = np.array(msg.right_lane_coeffs, dtype=float)
        centre_c = (left_c + right_c) / 2.0

        # Fixed row: vertical centre of the frame.
        centre_px     = np.polyval(centre_c, py)
        lane_width_px = np.polyval(right_c, py) - np.polyval(left_c, py)

        if lane_width_px < 10:
            self.get_logger().warn(f'[simple] lane_width_px={lane_width_px:.1f} too small — skipping')
            self._draw_and_publish(msg, py, None, stamp)
            return

        self._draw_and_publish(msg, py, centre_px, stamp)

        if self.current_odom is None:
            self.get_logger().warn('[simple] no /odom yet — skipping goal', throttle_duration_sec=2.0)
            return

        # Horizontal centre of the frame vs. where the lane centre actually is.
        offset_px     = IMG_W / 2.0 - centre_px
        metres_per_px = Physical.LANE_WIDTH_M / lane_width_px
        y_lat         = offset_px * metres_per_px + Physical.LATERAL_BIAS_M

        # Vertical centre of the frame maps to the midpoint of the visible
        # ground patch ahead of the rover.
        x_fwd = 0.5 * Physical.GROUND_HEIGHT_M

        q         = self.current_odom.orientation
        rover_yaw = 2.0 * math.atan2(q.z, q.w)
        rover_x   = self.current_odom.position.x
        rover_y   = self.current_odom.position.y

        cos_yaw = math.cos(rover_yaw)
        sin_yaw = math.sin(rover_yaw)
        goal_x   = rover_x + x_fwd * cos_yaw - y_lat * sin_yaw
        goal_y   = rover_y + x_fwd * sin_yaw + y_lat * cos_yaw
        goal_yaw = rover_yaw + math.atan2(y_lat, x_fwd)

        self.get_logger().info(
            f'[simple] py={py} centre_px={centre_px:.1f} offset_px={offset_px:+.1f} '
            f'lane_width_px={lane_width_px:.1f} m/px={metres_per_px:.5f} y_lat={y_lat:+.2f}m '
            f'x_fwd={x_fwd:.2f}m rover=({rover_x:.2f},{rover_y:.2f},{math.degrees(rover_yaw):+.1f}°) -> '
            f'goal=({goal_x:.2f}, {goal_y:.2f}, {math.degrees(goal_yaw):+.1f}°)'
        )

        if self._nav_active:
            self.get_logger().info('[simple] HOLD — a goal is already active')
            return

        self._send_goal(goal_x, goal_y, goal_yaw)

    def _send_goal(self, x: float, y: float, yaw: float) -> None:
        if not self.nav_to_pose_client.server_is_ready():
            self.get_logger().warn('[simple] Nav2 action server not ready — skipping', throttle_duration_sec=2.0)
            return

        self._goal_generation += 1
        gen = self._goal_generation

        goal_pose                    = PoseStamped()
        goal_pose.header.stamp       = self.get_clock().now().to_msg()
        goal_pose.header.frame_id    = RosTopics.ODOM_FRAME
        goal_pose.pose.position.x    = x
        goal_pose.pose.position.y    = y
        goal_pose.pose.orientation.z = math.sin(yaw / 2)
        goal_pose.pose.orientation.w = math.cos(yaw / 2)

        goal_msg      = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self._nav_active = True
        self.get_logger().info(f'[simple] SEND gen={gen} goal=({x:.2f},{y:.2f},{math.degrees(yaw):+.1f}°)')
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f, g=gen: self._on_response(f, g))

    def _on_response(self, future, gen: int) -> None:
        if gen != self._goal_generation:
            return
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn(f'[simple] gen={gen} REJECTED by Nav2')
            self._nav_active = False
            return
        self.get_logger().info(f'[simple] gen={gen} ACCEPTED')
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda f, g=gen: self._on_result(f, g))

    def _on_result(self, future, gen: int) -> None:
        if gen != self._goal_generation:
            return
        STATUS_NAMES = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
                        4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
        status = 0
        try:
            status = future.result().status
        except Exception as e:
            self.get_logger().warn(f'[simple] gen={gen} result error: {e}')
        self.get_logger().info(f'[simple] gen={gen} RESULT status={STATUS_NAMES.get(status, status)}')
        self._nav_active = False


def main(args=None):
    rclpy.init(args=args)
    node = SimpleLaneGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
