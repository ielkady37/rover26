"""
bringup.launch.py
One-shot launch that starts the full autonomous navigation stack:
  1. robot_state_publisher  (TF from URDF + sensors xacro)
  2. robot_localization EKF (odom → base_footprint, /odometry/filtered)
  3. slam_toolbox           (online async mapping,  map → odom TF)
  4. nav2                   (planner + controller + costmaps + behaviours)

Usage:
  ros2 launch rover_nav bringup.launch.py
  ros2 launch rover_nav bringup.launch.py use_sim_time:=true

  # Localisation-only with a saved map (skip SLAM):
  ros2 launch rover_nav bringup.launch.py use_slam:=false map:=/path/to/map.yaml
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Resolve maps directory: ROBOT_MAPS_DIR env var → ~/maps fallback
_maps_dir = os.environ.get('ROBOT_MAPS_DIR', os.path.expanduser('~/maps'))
os.makedirs(_maps_dir, exist_ok=True)


def generate_launch_description():
    pkg_rover_nav = get_package_share_directory('rover_nav')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # ── Arguments ────────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_slam     = LaunchConfiguration('use_slam',     default='true')
    map_yaml     = LaunchConfiguration('map',          default='')
    nav2_params  = os.path.join(pkg_rover_nav, 'config', 'nav2_params.yaml')
    slam_params  = os.path.join(pkg_rover_nav, 'config', 'slam_toolbox_params.yaml')
    ekf_config   = os.path.join(pkg_rover_nav, 'config', 'ekf.yaml')
    map_filename = os.path.join(_maps_dir, 'my_map')

    # ── 1. robot_state_publisher ─────────────────────────────────────────
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_nav, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ── 2. robot_localization EKF ────────────────────────────────────────
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odometry/filtered')],
    )

    # ── 3a. slam_toolbox (mapping mode) ──────────────────────────────────
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': use_sim_time, 'filename': map_filename}],
        remappings=[('/scan', '/scan')],
        condition=IfCondition(use_slam),
    )

    # ── 3b. AMCL localisation (localisation-only mode, needs a map) ──────
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map':          map_yaml,
            'params_file':  nav2_params,
            'autostart':    'true',
        }.items(),
        condition=UnlessCondition(use_slam),
    )

    # ── 4. nav2 navigation stack ─────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params,
            'autostart':    'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation (Gazebo) clock'),
        DeclareLaunchArgument('use_slam', default_value='true',
                              description='Run slam_toolbox for online mapping (false = AMCL with saved map)'),
        DeclareLaunchArgument('map', default_value='',
                              description='Path to map yaml (used only when use_slam:=false)'),
        rsp_launch,
        ekf_node,
        slam_node,
        amcl_launch,
        nav2_launch,
    ])
