"""
real_lidar_test.launch.py  —  rover26
══════════════════════════════════════════════════════════════════════════════
Plugs a physical LiDAR into the Gazebo simulation stack in place of the
simulated gpu_lidar sensor.

USAGE
─────
  # 1. Start your normal Gazebo simulation launch FIRST (without /scan bridge).
  #    See "PREREQUISITE" section below.

  # 2. Then, in a second terminal:
  ros2 launch rover26_autonomy real_lidar_test.launch.py

  # Override LiDAR driver or port:
  ros2 launch rover26_autonomy real_lidar_test.launch.py \
      lidar_driver:=ydlidar_ros2_driver \
      serial_port:=/dev/ttyUSB1 \
      serial_baudrate:=512000

PREREQUISITE — disable Gazebo's /scan bridge
─────────────────────────────────────────────
In your gazebo_launch.py (or equivalent), find the line that bridges /scan
and either:

  OPTION A — Remove/comment it out entirely:
    # ('ros_gz_bridge', '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'),

  OPTION B — Remap it to a dead topic so it doesn't fight the real LiDAR:
    ('ros_gz_bridge', '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'),
    # and add this remap in the bridge Node(...) arguments:
    remappings=[('/scan', '/gz/scan_sim')],

If you skip this step, both the Gazebo bridge and the real hardware will
publish to /scan simultaneously, causing duplicate or interleaved scan frames
that will confuse slam_toolbox.

SUPPORTED DRIVERS (set via lidar_driver argument)
──────────────────────────────────────────────────
  rplidar_ros          RPLIDAR A1 / A2 / A3 / S1 / S2 / S3    (default)
  ydlidar_ros2_driver  YDLIDAR X2 / X4 / G2 / TG / T15
  urg_node2            Hokuyo UST / UTM / UBG series
  sllidar_ros2         Slamtec SLLIDAR (newer RPLIDAR firmware)

WHAT THIS LAUNCH DOES
─────────────────────
  1. Starts the hardware LiDAR driver with use_sim_time: False
     (hardware nodes must use wall time — they don't know about Gazebo)

  2. Starts real_lidar_relay with use_sim_time: True
     (re-stamps hardware scans with Gazebo sim time so tf2/SLAM accepts them)

  3. Publishes a static TF: driver_frame → lidar_link
     (only needed if your driver uses a different frame_id than 'lidar_link')

VERIFICATION CHECKLIST (run after launch)
──────────────────────────────────────────
  □  ros2 topic hz /scan_hw          → should match LiDAR update rate (e.g. 10-15 Hz)
  □  ros2 topic hz /scan             → same rate, sim-stamped
  □  ros2 topic echo /scan --once    → frame_id='lidar_link', stamp follows sim time
  □  ros2 run tf2_ros tf2_echo odom lidar_link  → no extrapolation errors
  □  RViz: Fixed Frame = 'odom', add LaserScan on /scan  → points visible in world

══════════════════════════════════════════════════════════════════════════════
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────────

    lidar_driver_arg = DeclareLaunchArgument(
        'lidar_driver',
        default_value='rplidar_ros',
        description='ROS 2 package name for your LiDAR driver '
                    '(rplidar_ros | ydlidar_ros2_driver | urg_node2 | sllidar_ros2)'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial device the LiDAR is connected to'
    )

    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='460800',
        description='Baud rate for serial LiDAR communication '
                    '(RPLIDAR A1/A2=115200, A3/S1/S2=256000, YDLIDAR X4=128000)'
    )

    # The frame_id your hardware driver publishes in.
    # Most drivers let you set this; some hardcode it.
    # Common defaults: rplidar_ros='laser', ydlidar='laser_frame', urg_node2='laser'
    # Set to 'lidar_link' here if your driver already uses the right frame.
    driver_frame_arg = DeclareLaunchArgument(
        'driver_frame_id',
        default_value='laser',
        description='frame_id the hardware driver publishes in. '
                    'A static TF will be published from this frame to lidar_link.'
    )

    hw_scan_topic_arg = DeclareLaunchArgument(
        'hw_scan_topic',
        default_value='/scan_hw',
        description='Topic the hardware driver will publish on (remapped from /scan)'
    )

    lidar_driver    = LaunchConfiguration('lidar_driver')
    serial_port     = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    driver_frame_id = LaunchConfiguration('driver_frame_id')
    hw_scan_topic   = LaunchConfiguration('hw_scan_topic')

    # ── 1. Hardware LiDAR driver ──────────────────────────────────────────────
    #
    # Runs with use_sim_time: False — hardware nodes MUST use wall time.
    # The driver publishes to /scan (its default); we remap that to /scan_hw
    # so it doesn't fight the Gazebo bridge (if bridge is still running).
    #
    # NOTE: Parameter names differ between drivers.  The parameters below work
    # for rplidar_ros.  For other drivers, replace with their equivalents:
    #
    #   ydlidar_ros2_driver:
    #     port      → serial_port
    #     baudrate  → serial_baudrate
    #     frame_id  → frame_id
    #     executable: ydlidar_ros2_driver_node
    #
    #   urg_node2:
    #     ip_address / serial_port depending on model
    #     frame_id   → laser_frame_id
    #     executable: urg_node2
    #
    #   sllidar_ros2:
    #     same as rplidar_ros but executable = sllidar_ros2_node
    #
    lidar_driver_node = Node(
        package    = lidar_driver,
        executable = 'rplidar_composition',   # ← change for non-RPLIDAR drivers
        name       = 'lidar_hw_driver',
        output     = 'screen',
        parameters = [
            {
                'use_sim_time':     False,   # CRITICAL: hardware uses wall clock
                'serial_port':      serial_port,
                'serial_baudrate':  serial_baudrate,
                'frame_id':         driver_frame_id,
                'angle_compensate': True,    # RPLIDAR-specific: even angle spacing
                'scan_mode':        'Standard',
            }
        ],
        remappings = [
            # Redirect driver's /scan → /scan_hw so Gazebo bridge can't conflict
            ('/scan', hw_scan_topic),
        ],
    )

    # ── 2. Real-LiDAR relay ───────────────────────────────────────────────────
    #
    # Runs with use_sim_time: True so self.get_clock().now() returns Gazebo
    # sim time.  Re-stamps hardware scans and republishes to /scan.
    #
    relay_node = Node(
        package    = 'rover26_autonomy',
        executable = 'real_lidar_relay',
        name       = 'real_lidar_relay',
        output     = 'screen',
        parameters = [
            {
                'use_sim_time':     True,    # CRITICAL: must follow Gazebo /clock
                'hw_scan_topic':    hw_scan_topic,
                'output_topic':     '/scan',
                'output_frame_id':  'lidar_link',
                'max_range_clip':   25.0,    # matches Gazebo sensor range_max
                'min_range_clip':    0.08,   # matches Gazebo sensor range_min (0.3 in xacro;
                                             # 0.08 is safer for real hardware which reports
                                             # ~0.1 m for near returns instead of 0)
                'log_hz':           True,
            }
        ],
    )

    # ── 3. Static TF: driver_frame → lidar_link ───────────────────────────────
    #
    # If your driver's frame_id already IS 'lidar_link', this TF is an identity
    # and does no harm.  If it's 'laser' or 'base_scan', this makes TF lookups
    # work without modifying the driver.
    #
    # The sensor is physically at (0.27365, -0.0014, 0.04) from base_link
    # (from lidar.xacro), but that TF is already published by robot_state_publisher.
    # This static TF only spans from driver_frame_id → lidar_link (zero offset
    # assuming the driver frame coincides with the lidar_link origin).
    #
    static_tf_node = Node(
        package    = 'tf2_ros',
        executable = 'static_transform_publisher',
        name       = 'lidar_frame_bridge',
        output     = 'screen',
        arguments  = [
            # x  y  z  qx  qy  qz  qw
            '0', '0', '0', '0', '0', '0', '1',
            # parent_frame    child_frame
            'lidar_link',     driver_frame_id,
        ],
        parameters = [{'use_sim_time': True}],
    )

    # ── 4. Health-check log ───────────────────────────────────────────────────
    health_info = LogInfo(
        msg=(
            '\n'
            '╔══════════════════════════════════════════════════════════════╗\n'
            '║       REAL LIDAR TEST — rover26                              ║\n'
            '╠══════════════════════════════════════════════════════════════╣\n'
            '║  VERIFY (in another terminal):                               ║\n'
            '║  1. ros2 topic hz /scan_hw   → LiDAR is publishing          ║\n'
            '║  2. ros2 topic hz /scan      → relay is forwarding           ║\n'
            '║  3. ros2 topic echo /scan --once                             ║\n'
            '║       frame_id should be: lidar_link                         ║\n'
            '║       stamp  should be:   Gazebo sim time                    ║\n'
            '║  4. rviz2 → add LaserScan on /scan                           ║\n'
            '║       Fixed Frame = odom  (or map once SLAM converges)       ║\n'
            '║                                                              ║\n'
            '║  If /scan_hw is empty:                                       ║\n'
            '║    → check serial port / driver executable name              ║\n'
            '║    → try: ros2 run rplidar_ros rplidar_ros_node              ║\n'
            '║           --ros-args -p serial_port:=/dev/ttyUSB0           ║\n'
            '╚══════════════════════════════════════════════════════════════╝\n'
        )
    )

    return LaunchDescription([
        lidar_driver_arg,
        serial_port_arg,
        serial_baudrate_arg,
        driver_frame_arg,
        hw_scan_topic_arg,
        health_info,
        lidar_driver_node,
        relay_node,
        static_tf_node,
    ])