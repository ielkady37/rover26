from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # example_node = Node(
    #     package='example_package',
    #     executable='example_executable',
    #     output="screen",
    #     name="example_node",
    # )
    esp_bridge_node = Node(
        package='control',
        executable='esp_bridge_node',
        output="screen",
        name="esp_bridge_node"
    )
    return LaunchDescription([
        esp_bridge_node,
    manual_navigation_node = Node(
        package='control',
        executable='manual_navigation_node',
        output="screen",
        name="manual_navigation_node",
    ),
    autonomous_navigation_node = Node(
        package='control',
        executable='autonomous_navigation_node',
        output="screen",
        name="autonomous_navigation_node",
    ),

    return LaunchDescription([
        manual_navigation_node,
        autonomous_navigation_node,
    ])