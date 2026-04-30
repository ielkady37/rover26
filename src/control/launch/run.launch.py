from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # example_node = Node(
    #     package='example_package',
    #     executable='example_executable',
    #     output="screen",
    #     name="example_node",
    # )

    return LaunchDescription([
        # example_node,
    ])