from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

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
        emulate_tty=True,
        name="autonomous_navigation_node"
    )

    # ROSBridge WebSocket Server
    rosbridge_server_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        output='screen',
        name='rosbridge_websocket',
        parameters=[{'port': 9090}]
    )

    return LaunchDescription([
        joystick_node,
        # esp_bridge_node,
        manual_navigation_node,
        autonomous_navigation_node,
        rosbridge_server_node
    ])