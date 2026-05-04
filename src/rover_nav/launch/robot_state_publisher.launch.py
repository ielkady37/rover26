"""
robot_state_publisher.launch.py
Publishes TF tree from the rover URDF with sensor links (lidar, IMU).
This launch file is included by all other nav launch files.
"""
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_rover_nav = get_package_share_directory('rover_nav')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    xacro_file = os.path.join(pkg_rover_nav, 'urdf', 'rover_with_sensors.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation (Gazebo) clock'),
        robot_state_publisher_node,
    ])
