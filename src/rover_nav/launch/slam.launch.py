"""
slam.launch.py
Starts slam_toolbox in online async mapping mode together with
robot_state_publisher and the EKF localizer.

Usage:
  ros2 launch rover_nav slam.launch.py
  ros2 launch rover_nav slam.launch.py use_sim_time:=true
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Resolve maps directory: ROBOT_MAPS_DIR env var → ~/maps fallback
_maps_dir = os.environ.get('ROBOT_MAPS_DIR', os.path.expanduser('~/maps'))
os.makedirs(_maps_dir, exist_ok=True)


def generate_launch_description():
    pkg_rover_nav = get_package_share_directory('rover_nav')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_params = os.path.join(pkg_rover_nav, 'config', 'slam_toolbox_params.yaml')
    map_filename = os.path.join(_maps_dir, 'my_map')

    # ── Localization stack (RSP + EKF) ────────────────────────────────────
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_nav, 'launch', 'localization.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ── slam_toolbox async online mapper ─────────────────────────────────
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': use_sim_time, 'filename': map_filename}],
        remappings=[
            ('/scan', '/scan'),
            ('pose', '/slam_pose'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation clock'),
        localization_launch,
        slam_node,
    ])
