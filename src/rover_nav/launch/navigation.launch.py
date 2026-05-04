"""
navigation.launch.py
Starts the full nav2 stack (planner, controller, costmaps, behaviours).
Expects an existing map → odom → base_footprint TF tree to already be running
(provided by slam.launch.py or localization.launch.py + a pre-built map).

Usage – mapping mode (SLAM provides the map):
  ros2 launch rover_nav slam.launch.py          # terminal 1
  ros2 launch rover_nav navigation.launch.py    # terminal 2

Usage – localisation-only mode (use a saved map):
  ros2 launch rover_nav navigation.launch.py map:=/path/to/map.yaml
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg_rover_nav = get_package_share_directory('rover_nav')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml = LaunchConfiguration('map', default='')
    nav2_params_file = os.path.join(pkg_rover_nav, 'config', 'nav2_params.yaml')

    # ── nav2 bringup – navigation-only (no SLAM, no AMCL when map is empty) ──
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation clock'),
        DeclareLaunchArgument('map', default_value='',
                              description='Full path to map yaml file (leave empty when using SLAM)'),
        nav2_launch,
    ])
