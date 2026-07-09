from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    rover26_pkg = get_package_share_directory('rover26')
    xacro_file  = os.path.join(rover26_pkg, 'description', 'rover26.urdf.xacro')
    robot_desc  = xacro.process_file(xacro_file).toxml()

    # Robot State Publisher — publishes URDF transforms to /tf_static
    # Required by SLAM, Nav2, and RViz to know sensor positions on the robot
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],
        output='screen',
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # LiDAR — RPLidar C1
    # Publishes /scan (LaserScan) in lidar_link frame at 10Hz
    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'serial_port': '/dev/rover_lidar',
            'serial_baudrate': 460800,
            'frame_id': 'lidar_link',
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'use_sim_time': False,
        }],
        output='screen',
    )

    # Joystick Node
    joystick_node = Node(
        package='control',
        executable='joystick_node',
        output="screen",
        name="joystick_node"
    )

    # ESP Bridge Node
    # base_frame set to base_footprint to match URDF TF chain
    esp_bridge_node = Node(
        package='control',
        executable='esp_bridge_node',
        output="screen",
        name="esp_bridge_node",
        parameters=[{
            'base_frame': 'base_footprint',
            'sensor_port': '/dev/rover_esp_sensor',
            'actuator_port': '/dev/ttyUSB3',
        }]
    )

    # HoverBoard Node
    hoverboard_node = Node(
        package='control',
        executable='hoverboard_node',
        output="screen",
        name="hoverboard_node",
        parameters=[{
            'uart_port': '/dev/rover_hoverboard',
            'baudrate': 115200,
            'timeout': 0.1,
            'max_speed': 200,
            'base_frame': 'base_link',
        }]
    )

    # Odometry Publisher Node
    odom_publisher_node = Node(
        package='control',
        executable='odom_publisher_node',
        output="screen",
        name="odom_publisher_node",
        parameters=[{
            'wheel_radius': 0.075,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
        }]
    )

    # GPS Waypoint Node
    # Converts lat/lon competition waypoints + live /gps fix into local ENU x,y.
    # Publishes /waypoints_xy (latched) and /gps_xy — use gps_xy only to reach WP1,
    # then switch position source to encoder/IMU dead reckoning (see odom_publisher_node).
    # NOTE: this node subscribes to /gps (sensor_msgs/NavSatFix) but does not publish it —
    # make sure your GPS driver node (e.g. a u-blox/NMEA driver) is included somewhere
    # in your bringup, or this node will just sit waiting for a fix.
    gps_waypoint_node = Node(
        package='control',
        executable='gps_waypoint_node',
        output="screen",
        name="gps_waypoint_node",
        parameters=[{
            'waypoint_lats': [0.0, 0.0, 0.0],
            'waypoint_lons': [0.0, 0.0, 0.0],
            'waypoint_ids': ['WP1', 'WP2', 'WP3'],
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'wp1_tolerance_m': 1.5,
        }]
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

    # Mission Manager Node
    mission_manager_node = Node(
        package='control',
        executable='mission_manager_node',
        output="screen",
        emulate_tty=True,
        name="mission_manager_node"
    )

    camera_control_server = Node(
        package="control",
        executable="camera_control_server",
        name="camera_control_server",
        output="screen",
    )

    # ROSBridge WebSocket Server
    rosbridge_server_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        output='screen',
        name='rosbridge_websocket',
        parameters=[{'port': 9090}]
    )

    # cv nodes
    manual_camera_streaming_node = Node(
        package='cv',
        executable='manual_camera_streaming_node',
        output="screen",
        name="manual_camera_streaming_node"
    )

    autonomous_camera_streaming_node = Node(
        package='cv',
        executable='autonomous_camera_streaming_node',
        output="screen",
        name="autonomous_camera_streaming_node"
    )

    lane_detection_node = Node(
        package='cv',
        executable='lane_detection_node',
        output="screen",
        name="lane_detection_node"
    )

    face_recognition_node = Node(
        package='cv',
        executable='face_recognition_node',
        output="screen",
        name="face_recognition_node"
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        odom_publisher_node,
        sllidar_node,
        gps_waypoint_node,
        joystick_node,
        esp_bridge_node,
        hoverboard_node,
        manual_navigation_node,
        autonomous_navigation_node,
        mission_manager_node,
        rosbridge_server_node,
        camera_control_server,
        manual_camera_streaming_node,
        autonomous_camera_streaming_node,
        lane_detection_node,
        face_recognition_node
    ])