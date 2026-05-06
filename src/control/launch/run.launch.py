from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Joystick Node
    joystick_node = Node(
        package='control',
        executable='joystick_node', 
        output="screen",
        name="joystick_node"
    )
    
    # ESP Bridge Node
    esp_bridge_node = Node(
        package='control',
        executable='esp_bridge_node',
        output="screen",
        name="esp_bridge_node"
    )

    # Manual Navigation Node
    manual_navigation_node = Node(
        package='control',
        executable='manual_navigation_node',
        output="screen",
        emulate_tty=True,
        name="manual_navigation_node"
    )

    # Autonomous Navigation Node
    autonomous_navigation_node = Node(
        package='control',
        executable='autonomous_navigation_node',
        output="screen",
        name="autonomous_navigation_node"
    )

    return LaunchDescription([
        joystick_node,
        esp_bridge_node,
        manual_navigation_node,
        autonomous_navigation_node
    ])