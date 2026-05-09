"""
localization.launch.py
Starts the robot_localization EKF node that fuses wheel odometry and IMU
into /odometry/filtered and broadcasts the odom → base_footprint TF.

Usage:
  ros2 launch rover_nav localization.launch.py
  ros2 launch rover_nav localization.launch.py use_sim_time:=true
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_rover_nav = get_package_share_directory('rover_nav')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    ekf_config = os.path.join(pkg_rover_nav, 'config', 'ekf.yaml')

    # ── robot_state_publisher ──────────────────────────────────────────────
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_nav, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ── EKF odom-frame filter ─────────────────────────────────────────────
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
            # If your wheel odometry node publishes on a different topic, remap here:
            # ('/odom', '/wheel/odom'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation clock'),
        rsp_launch,
        ekf_node,
    ])
