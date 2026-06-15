#!/usr/bin/env python3
"""
═════════════════════════════════════════════════════════════════════════════════
real_lidar_relay.py  —  rover26_autonomy
═════════════════════════════════════════════════════════════════════════════════

FUNCTIONALITY
─────────────
Bridges a physical (hardware) LiDAR into the Gazebo simulation pipeline by
re-stamping every incoming scan with the current Gazebo sim time and
republishing it on the topic the simulation stack expects.

WHY THIS NODE EXISTS
────────────────────
The entire simulation stack (slam_toolbox, odom_tf_broadcaster, scan_merger,
Nav2) runs with use_sim_time: True — all clocks follow Gazebo's /clock topic,
NOT the wall clock.

A real LiDAR driver always publishes with WALL-CLOCK timestamps (it knows
nothing about Gazebo). If those wall-stamped scans reach slam_toolbox or tf2
directly you get:

    [tf2_ros]      Lookup would require extrapolation into the future ...
    [slam_toolbox] Scan is too old to process ...

PIPELINE (per scan)
────────────────────
  1. Receive scan from hardware driver on RosTopics.LIDAR_HW  (/scan_hw)
  2. Replace header.stamp  with current Gazebo sim time (self.get_clock().now())
  3. Replace header.frame_id with RosTopics.LIDAR_FRAME ('lidar_link')
  4. Optionally clip readings outside [min_range_clip, max_range_clip] to inf
  5. Publish the corrected scan on RosTopics.LIDAR_SCAN  (/scan)

TOPICS & SUBSCRIPTIONS
──────────────────────
Subscribed:
  • /scan_hw  [sensor_msgs/LaserScan]  ← hardware LiDAR driver (wall-stamped)

Published:
  • /scan     [sensor_msgs/LaserScan]  → scan_merger / slam_toolbox / Nav2

TF FRAMES
─────────
Input  frame: any (set by hardware driver, e.g. 'laser')
Output frame: lidar_link  (overwritten here - must match lidar.xacro gz_frame_id)

PARAMETERS (from config_params.py)
──────────────────────────────────
RosTopics.LIDAR_HW     Default input topic  (/scan_hw)
RosTopics.LIDAR_SCAN   Default output topic (/scan)
RosTopics.LIDAR_FRAME  Default output frame (lidar_link)

All values are also overridable from the launch file via ROS parameters,
so you can test with a different topic/frame without editing this file.

PREREQUISITE — disable Gazebo's /scan bridge
────────────────────────────────────────────
In gazebo_launch.py comment out or remap the Gazebo /scan bridge so it does
not fight the hardware scan on the same topic:
    # ('ros_gz_bridge', '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'),

═════════════════════════════════════════════════════════════════════════════════
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from sensor_msgs.msg import LaserScan

# All topic and frame names come from the central registry — no hardcoded strings.
from rover26_autonomy.config_params import RosTopics


# ═════════════════════════════════════════════════════════════════════════════
#  REAL LIDAR RELAY NODE
# ═════════════════════════════════════════════════════════════════════════════

class RealLidarRelay(Node):
    """
    Re-stamps and re-frames a hardware LiDAR scan so the Gazebo simulation
    pipeline accepts it as if it came from Gazebo's own sensor.

    Because this node is launched with use_sim_time: True, calling
    self.get_clock().now() already returns Gazebo sim time — the whole
    re-stamping logic is a single line inside the callback.

    Attributes:
        _frame       (str):   Output frame_id written into every scan header.
        _max_r       (float): Ranges above this are clipped to inf (-1 = off).
        _min_r       (float): Ranges below this are clipped to inf (-1 = off).
        _relay_count (int):   Scans forwarded in the current 5-second window.
        _drop_count  (int):   Ranges clipped in the current 5-second window.
        _pub         (Publisher): Output publisher on RosTopics.LIDAR_SCAN.
    """

    def __init__(self):
        super().__init__('real_lidar_relay')

        # ── Declare ROS parameters (overridable from launch file) ──────────────
        # Default values come from the central RosTopics registry so changing a
        # topic name in config_params.py propagates here automatically.
        self.declare_parameter('hw_scan_topic',   RosTopics.LIDAR_HW)      # Input  topic
        self.declare_parameter('output_topic',    RosTopics.LIDAR_SCAN)    # Output topic
        self.declare_parameter('output_frame_id', RosTopics.LIDAR_FRAME)   # Output frame_id
        self.declare_parameter('max_range_clip',  25.0)   # metres; -1 = disable clipping
        self.declare_parameter('min_range_clip',   0.08)  # metres; -1 = disable clipping
        self.declare_parameter('log_hz',          True)   # Log relay Hz every 5 s

        # Read all parameters into instance variables once at startup
        hw_topic      = self.get_parameter('hw_scan_topic').value
        out_topic     = self.get_parameter('output_topic').value
        self._frame   = self.get_parameter('output_frame_id').value
        self._max_r   = self.get_parameter('max_range_clip').value
        self._min_r   = self.get_parameter('min_range_clip').value
        log_hz        = self.get_parameter('log_hz').value

        # ── Diagnostic counters (reset every 5 s by the timer) ────────────────
        self._relay_count: int = 0   # Scans relayed since last log
        self._drop_count:  int = 0   # Individual ranges clipped since last log

        # ── Publisher on /scan (RELIABLE — sim stack must not miss a frame) ────
        pub_qos = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE,
            history     = QoSHistoryPolicy.KEEP_LAST,
            depth       = 20,
        )
        self._pub = self.create_publisher(LaserScan, out_topic, pub_qos)

        # ── Subscriber on /scan_hw (BEST_EFFORT — matches most HW LiDAR drivers)
        # Most drivers (rplidar_ros, ydlidar_ros2_driver, urg_node2) publish with
        # BEST_EFFORT. A BEST_EFFORT subscriber also accepts RELIABLE publishers,
        # so this works regardless of which driver is running.
        hw_qos = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            history     = QoSHistoryPolicy.KEEP_LAST,
            depth       = 10,
        )
        self.create_subscription(LaserScan, hw_topic, self._relay_cb, hw_qos)

        # ── Optional diagnostic timer — logs actual relay Hz every 5 s ─────────
        if log_hz:
            self.create_timer(5.0, self._log_rates)

        # Startup summary so the operator can verify topics at a glance
        self.get_logger().info(
            'real_lidar_relay ready:\n'
            f'  in  (hw, wall-time) : {hw_topic}\n'
            f'  out (sim-time)      : {out_topic}  frame={self._frame}\n'
            f'  range clip          : [{self._min_r}, {self._max_r}] m'
        )
        self.get_logger().warn(
            f'Waiting for first scan from hardware LiDAR on {hw_topic} ...\n'
            f'  Verify with: ros2 topic hz {hw_topic}'
        )

    # =========================================================================
    #  RELAY CALLBACK
    # =========================================================================

    def _relay_cb(self, msg: LaserScan) -> None:
        """
        Called for every LaserScan from the hardware driver.

        All the heavy lifting happens here in three small steps:

        Step 1 — Re-stamp with Gazebo sim time
            self.get_clock().now() returns sim time because this node is
            launched with use_sim_time: True.  This is the core purpose of
            the relay — one line fixes the timestamp problem entirely.

        Step 2 — Overwrite frame_id
            lidar_link matches the URDF and the scan_merger config so that
            tf2 can transform the scan into odom / map without errors.

        Step 3 — Range clipping (optional)
            Hardware LiDARs sometimes return 0.0 for missed or saturated
            returns instead of inf or range_max.  Clipping those to inf
            prevents slam_toolbox from seeing phantom obstacles at the
            rover's centre.  The slam_toolbox ignores inf readings by design.

        Args:
            msg: Incoming LaserScan from the hardware driver (wall-stamped).
                 The message is mutated in-place and then republished.
        """

        # ── Step 1: Replace wall-clock stamp with Gazebo sim time ─────────────
        # This is the entire reason this relay node exists.
        msg.header.stamp    = self.get_clock().now().to_msg()

        # ── Step 2: Set frame_id to the URDF-defined LiDAR link ───────────────
        # If the driver already uses 'lidar_link', this is a no-op.
        # If it uses 'laser' or 'base_scan', this corrects it here without
        # needing a static_transform_publisher workaround.
        msg.header.frame_id = self._frame

        # ── Step 3: Optional range clipping ───────────────────────────────────
        # Skip entirely if both clip thresholds are disabled (-1).
        if self._min_r >= 0.0 or self._max_r >= 0.0:
            ranges = list(msg.ranges)   # Copy — msg.ranges is a tuple in ROS 2
            for i, r in enumerate(ranges):
                if not math.isfinite(r):
                    continue   # Already inf / nan — leave as-is

                # Clip too-close returns (driver hardware noise, rover body reflex)
                if self._min_r >= 0.0 and r < self._min_r:
                    ranges[i] = float('inf')
                    self._drop_count += 1

                # Clip too-far returns (beyond sensor spec, or open environment)
                elif self._max_r >= 0.0 and r > self._max_r:
                    ranges[i] = float('inf')
                    self._drop_count += 1

            msg.ranges = ranges

        # ── Publish the corrected scan ─────────────────────────────────────────
        self._pub.publish(msg)
        self._relay_count += 1

    # =========================================================================
    #  DIAGNOSTIC TIMER
    # =========================================================================

    def _log_rates(self) -> None:
        """
        Log relay statistics every 5 seconds so the operator can verify
        the hardware LiDAR is running at the expected rate.

        Expected values:
          - RPLIDAR A1/A2  →  ~7–10 Hz
          - RPLIDAR A3/S1  → ~15 Hz
          - YDLIDAR X4     →  ~8 Hz
        """
        hz = self._relay_count / 5.0   # Average Hz over the last window
        self.get_logger().info(
            f'real_lidar_relay: {hz:.1f} Hz relayed  |  '
            f'{self._drop_count} range values clipped  (last 5 s)'
        )
        # Reset counters for the next 5-second window
        self._relay_count = 0
        self._drop_count  = 0


# ═════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = RealLidarRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('real_lidar_relay shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()