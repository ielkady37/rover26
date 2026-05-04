from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    manual_navigation_node = Node(
        package='control',
        executable='manual_navigation_node',
        output="screen",
        name="manual_navigation_node",
    )

    return LaunchDescription([
        manual_navigation_node,
    ])