import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_share_path

def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('rover_description'), 'urdf', 'rover.urdf.xacro')
    rviz_config_path = os.path.join(get_package_share_path('rover_description'), 'rviz', 'rover_config.rviz')
    world_path = os.path.join(get_package_share_path('rover_description'), 'worlds', 'competition_track_world.sdf')
    share_root = os.path.dirname(get_package_share_directory('rover_description'))  
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description, "use_sim_time": True}]
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments=[
                    ("gz_args", [" -v 4", " -r", f" {world_path}"]
                    )
                ]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "rover",
                   "-y", "-2",
                   "-z", "0.3"],
    )

    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/right_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/left_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/gps/location@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        output='screen',
    )

    return LaunchDescription([
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=share_root),
        AppendEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=share_root),
        robot_state_publisher_node,
        # joint_state_publisher_gui_node,
        rviz2_node,
        gazebo,
        gz_spawn_entity,
        ros_gz_bridge_node,
    ])

