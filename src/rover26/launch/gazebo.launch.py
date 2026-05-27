"""
gazebo.launch.py  –  Rover 26 full simulation + Nav2 autonomy stack
====================================================================

Starts:
  1. Gazebo (competition world)
  2. Robot State Publisher (URDF/xacro)
  3. rover26_autonomy nodes
       • pothole_detection_node   (publishes PointCloud2 → Nav2 costmap)
       • lane_path_publisher_node (publishes Path → Nav2 controller)
  4. Nav2 bringup (controller_server + costmap + planner)
  5. ros_gz_bridge
  6. Robot spawn (delayed 4 s)
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # ── package paths ─────────────────────────────────────────────────────────
    rover26_pkg   = get_package_share_directory('rover26')
    nav2_bringup  = get_package_share_directory('nav2_bringup')
    autonomy_pkg  = get_package_share_directory('rover26_autonomy')

    xacro_file  = os.path.join(rover26_pkg, 'description', 'rover26.urdf.xacro')
    world_file  = os.path.join(rover26_pkg, 'worlds', 'competition_track_world.sdf')
    nav2_params = os.path.join(autonomy_pkg, 'config', 'nav2_params.yaml')

    robot_desc = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([

        # ── environment ───────────────────────────────────────────────────────
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.join(rover26_pkg, '..')
        ),

        # ── Gazebo ────────────────────────────────────────────────────────────
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen'
        ),

        # ── Robot State Publisher ─────────────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
        ),

        # ── Gazebo Bridge ─────────────────────────────────────────────────────
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/gps/location@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
                '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            ],
            output='screen'
        ),

        # ── Autonomy nodes ────────────────────────────────────────────────────
        Node(
            package='rover26_autonomy',
            executable='lane_detection',
            name='lane_detection',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='rover26_autonomy',
            executable='pothole_detection',
            name='pothole_detection',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='rover26_autonomy',
            executable='lane_path_publisher',
            name='lane_path_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='rover26_autonomy',
            executable='lane_follow_client',
            name='lane_follow_client',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # ── Nav2 ──────────────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file':  nav2_params,
                'use_lifecycle_mgr': 'true',
                'map_subscribe_transient_local': 'true',
            }.items(),
        ),

        # ── Spawn robot (delayed) ─────────────────────────────────────────────
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name',  'rover26',
                        '-topic', '/robot_description',
                        '-y',     '-2',
                        '-z',     '0.3',
                        '-Y',     '1.57',
                    ],
                    output='screen'
                ),
            ]
        ),
    ])