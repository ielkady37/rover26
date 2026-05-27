"""
gazebo_launch.py  —  rover26 (TF FIXED)
──────────────────────────────────────────────────────────────────────────────
TF ARCHITECTURE (who publishes what)
──────────────────────────────────────────────────────────────────────────────

  robot_state_publisher  →  ALL URDF-defined transforms
  ┌─────────────────────────────────────────────────────────────────────┐
  │  Fixed joints  →  /tf_static  (published once at startup)           │
  │    base_footprint  →  base_link                                     │
  │    base_link  →  lidar_link                                         │
  │    base_link  →  camera_link                                        │
  │    base_link  →  imu_link                                           │
  │    base_link  →  gps_link                                           │
  │    base_link  →  FL/BL/FR/BR castor and wheel links (fixed parts)   │
  │                                                                     │
  │  Continuous/revolute joints  →  /tf  (published on /joint_states)   │
  │    base_link  →  rightwheel_link   (from Gazebo joint states)       │
  │    base_link  →  leftwheel_link    (from Gazebo joint states)       │
  │    base_link  →  castor swivel/wheel joints                         │
  └─────────────────────────────────────────────────────────────────────┘

  odom_tf_broadcaster  →  THE DYNAMIC ODOMETRY EDGE
  ┌─────────────────────────────────────────────────────────────────────┐
  │  odom  →  base_footprint   (published on every /odom message)      │
  │  This is the ONE transform RSP cannot publish — it reads live       │
  │  odometry data from Gazebo, not the static URDF.                    │
  │  WITHOUT THIS NODE the entire TF chain from "odom" down is broken.  │
  └─────────────────────────────────────────────────────────────────────┘

  slam_toolbox  →  THE LOCALISATION EDGE
  ┌─────────────────────────────────────────────────────────────────────┐
  │  map  →  odom   (published once SLAM is activated)                  │
  └─────────────────────────────────────────────────────────────────────┘

Full TF tree when everything is running:
  map → odom → base_footprint → base_link → lidar_link
                                           → camera_link
                                           → imu_link
                                           → gps_link
                                           → rightwheel_link
                                           → leftwheel_link
                                           → FL/BL/FR/BR castor/wheel links

──────────────────────────────────────────────────────────────────────────────
TIMELINE
──────────────────────────────────────────────────────────────────────────────
  t= 0s  Gazebo + robot_state_publisher + bridge
           → /tf_static is fully populated immediately
  t= 5s  Spawn rover
           → Gazebo starts publishing /odom + /joint_states
  t= 7s  odom_tf_broadcaster
           → odom → base_footprint TF starts flowing
           → RSP starts publishing wheel/castor dynamic TFs (needs /joint_states)
  t=12s  All autonomy nodes (full TF chain is guaranteed complete by now)
  t=16s  slam_toolbox (unconfigured)
  t=21s  lifecycle_manager activates SLAM → map → odom TF appears
"""

import math
import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    rover26_pkg  = get_package_share_directory('rover26')
    autonomy_pkg = get_package_share_directory('rover26_autonomy')

    # ── Config file paths ──────────────────────────────────────────────────
    twist_mux_params = os.path.join(autonomy_pkg, 'config', 'twist_mux.yaml')
    xacro_file       = os.path.join(rover26_pkg,  'description', 'rover26.urdf.xacro')
    world_file       = os.path.join(rover26_pkg,  'worlds', 'competition_track_world.sdf')
    nav2_params      = os.path.join(autonomy_pkg, 'config', 'nav2_params.yaml')
    mapper_params    = os.path.join(autonomy_pkg, 'config', 'mapper_params_online_async.yaml')


    # Process the xacro file into a full URDF XML string.
    # robot_state_publisher needs this to know the robot's joint structure.
    robot_desc = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([

        # ══════════════════════════════════════════════════════════════════════
        #  Environment variables
        # ══════════════════════════════════════════════════════════════════════
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(rover26_pkg, '..')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),

        # ══════════════════════════════════════════════════════════════════════
        #  t = 0 s — Gazebo + robot_state_publisher + ROS-GZ bridge
        # ══════════════════════════════════════════════════════════════════════

        # Gazebo simulator
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen',
        ),

        # ── robot_state_publisher ─────────────────────────────────────────────
        # Responsible for ALL URDF-defined transforms.
        #
        # What it publishes:
        #   → /tf_static  for every FIXED joint in the URDF
        #       base_footprint → base_link
        #       base_link → lidar_link / camera_link / imu_link / gps_link
        #       base_link → castor mount links (fixed parts of each castor)
        #
        #   → /tf  for every CONTINUOUS or REVOLUTE joint whenever it receives
        #     a /joint_states message from the bridge:
        #       base_link → rightwheel_link
        #       base_link → leftwheel_link
        #       base_link → FL/BL/FR/BR castor swivel + wheel joints
        #
        # What it does NOT publish:
        #   → odom → base_footprint  (that's odom_tf_broadcaster's job)
        #   → map  → odom            (that's slam_toolbox's job)
        # ── robot_state_publisher ─────────────────────────────────────────────────────
        # Publishes /robot_description (needed by the spawn node at t=5s)
        # and ALL URDF-defined transforms to /tf_static and /tf.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True,
            }],
            output='screen',
        ),

        # ── ROS ↔ Gazebo bridge ───────────────────────────────────────────────────────
        # Single bridge node — do NOT add a second one.
        # /joint_states feeds both RSP (wheel TFs) and odom_tf_broadcaster (odometry).
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # Simulation clock — all nodes with use_sim_time=True need this
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                # Velocity commands: Nav2/teleop → Gazebo
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                # LiDAR scan: Gazebo → ROS
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                # Camera image: Gazebo → ROS
                '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                # IMU: Gazebo → ROS (heading source for odom_tf_broadcaster)
                '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                # Joint states: Gazebo → ROS
                # Published directly at /joint_states by gz-sim-joint-state-publisher-system.
                # No remapping needed — the topic name is already correct.
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
            # No parameters= here — nav2_params does not belong on the bridge node
            output='screen',
        ),

        # ══════════════════════════════════════════════════════════════════════
        #  t = 5 s — Spawn rover into Gazebo
        #  After this: Gazebo starts publishing /odom and /joint_states
        # ══════════════════════════════════════════════════════════════════════
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'rover26', '-topic', '/robot_description',
                        '-x', '0.0', '-y', '-2.0', '-z', '0.05', '-Y', '0.0',
                    ],
                    output='screen',
                ),
            ],
        ),

        # ══════════════════════════════════════════════════════════════════════
        #  t = 7 s — odom_tf_broadcaster (THE TF FIX)
        # ══════════════════════════════════════════════════════════════════════
        # WHY ITS OWN TIMER?
        #   The rover must be spawned (t=5s) before Gazebo publishes /odom.
        #   We give 2 extra seconds for Gazebo to fully initialise the model
        #   and start streaming odometry before this node starts listening.
        #
        # WHAT THIS NODE DOES:
        #   Subscribes to /odom  →  publishes TF: odom → base_footprint
        #   This is the missing dynamic edge that connects the SLAM-produced
        #   "odom" frame to the URDF's "base_footprint" frame.
        #
        # WHY NOT robot_state_publisher?
        #   RSP only reads the static URDF description + /joint_states.
        #   It has no concept of where the robot IS in the world — that is
        #   entirely the odometry broadcaster's responsibility.
        #
        # NOTE: odom_relay has been REMOVED.
        #   It only republished the /odom topic without publishing any TF,
        #   which left the odom → base_footprint edge permanently missing.
        # ─────────────────────────────────────────────────────────────────────
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='rover26_autonomy',
                    executable='odom_tf_broadcaster',
                    name='odom_tf_broadcaster',
                    # use_sim_time=True ensures the TF timestamps match the
                    # Gazebo clock — critical to avoid TF extrapolation errors
                    parameters=[{'use_sim_time': True}],
                    output='screen',
                ),
                # Node(
                #     package='joint_state_publisher',
                #     executable='joint_state_publisher',
                #     name='joint_state_publisher',
                #     parameters=[{'use_sim_time': True}],
                #     output='screen',
                # ),
            ],
        ),

        # ══════════════════════════════════════════════════════════════════════
        #  t = 12 s — All other autonomy nodes
        #  By t=12s the full TF chain is established:
        #    odom → base_footprint  ✓  (odom_tf_broadcaster, running 5s)
        #    base_footprint → base_link → sensors  ✓  (RSP static, since t=0)
        #    base_link → wheels  ✓  (RSP dynamic via /joint_states)
        # ══════════════════════════════════════════════════════════════════════
        TimerAction(
            period=18.0,
            actions=[

                # ── Velocity arbitration ───────────────────────────────────────
                # Receives cmd_vel from multiple sources (Nav2, teleop, etc.),
                # applies priority rules, outputs the winning command.
                Node(
                    package='twist_mux',
                    executable='twist_mux',
                    name='twist_mux',
                    parameters=[twist_mux_params, {'use_sim_time': True}],
                    remappings=[('/cmd_vel_out', '/cmd_vel_stamped')],
                    output='screen',
                ),

                # ── TwistStamped → Twist ───────────────────────────────────────
                # Nav2 outputs TwistStamped; Gazebo expects plain Twist.
                # This node strips the header timestamp.
                Node(
                    package='rover26_autonomy',
                    executable='twist_unstamper',
                    name='twist_unstamper',
                    parameters=[{'use_sim_time': True}],
                    output='screen',
                ),

                # ── Dual LiDAR scan merger ─────────────────────────────────────
                # Merges two LaserScan sources into /scan_merged for SLAM/Nav2:
                #   /scan           — physical LiDAR on the rover
                #   /detection/scan — synthetic scan from lane detection camera
                #
                # Both input scans must have frame_id = 'lidar_link'.
                # If /scan arrives with 'rover26/lidar_link' (Gazebo namespace),
                # add a static_transform_publisher aliasing the frames.
                # Check with: ros2 topic echo /scan --field header.frame_id
                ComposableNodeContainer(
                    name='scan_merger_container',
                    namespace='',
                    package='rclcpp_components',
                    executable='component_container',
                    composable_node_descriptions=[
                        ComposableNode(
                            package='dual_laser_merger',
                            plugin='merger_node::MergerNode',
                            name='scan_merger',
                            parameters=[
                                # QoS for /scan_merged — must match SLAM toolbox
                                {'merged_scan_topic_qos': 'reliable'},
                                # Input topics
                                {'laser_1_topic':     '/detection/scan'},  # synthetic from camera
                                {'laser_2_topic':     '/scan'},            # physical LiDAR
                                # Output topic consumed by SLAM + Nav2
                                {'merged_scan_topic': '/scan_merged'},
                                # Transform all scans into this frame before merging
                                # The TF lookup requires odom_tf_broadcaster to be running
                                {'target_frame':      'lidar_link'},
                                {'publish_rate':       20.0},
                                # Range filter — must match SLAM's min/max_laser_range
                                {'range_min':          0.1},
                                {'range_max':          12.0},
                                # Angular coverage — full 360°
                                {'angle_min':         -math.pi},
                                {'angle_max':          math.pi},
                                # ~720 bins → ~0.5° resolution
                                {'angle_increment':    0.004363},
                                {'scan_time':          0.067},
                                # Treat inf returns as "no obstacle" (not a wall)
                                {'use_inf':            False},
                                {'inf_epsilon':        1.0},
                                # Height filter for 3D → 2D projection
                                {'min_height':        -0.5},
                                {'max_height':         0.5},
                                {'tolerance':          0.01},
                                {'queue_size':         10},
                                # No physical offsets — both sources share lidar_link
                                {'laser_1_x_offset':   0.0},
                                {'laser_1_y_offset':   0.0},
                                {'laser_1_yaw_offset': 0.0},
                                {'laser_2_x_offset':   0.0},
                                {'laser_2_y_offset':   0.0},
                                {'laser_2_yaw_offset': 0.0},
                                {'allowed_radius':     0.6},
                                {'enable_shadow_filter':  False},
                                {'enable_average_filter': False},
                            ],
                        ),
                    ],
                    output='screen',
                ),

                # ── Lane detection ─────────────────────────────────────────────
                # Bird's-eye perspective warp on /camera/image,
                # polynomial lane fitting, publishes LaneStatus → /lane_status
                Node(
                    package='rover26_autonomy',
                    executable='lane_detection',
                    name='lane_detection',
                    parameters=[{'use_sim_time': True}],
                    output='screen',
                ),

                #── Pothole detection ─────────────────────────────────────────────
                Node(
                    package='rover26_autonomy',
                    executable='pothole_detection',
                    name='pothole_detection',
                    parameters=[{'use_sim_time': True}],
                    output='screen',
                ),
                # ── Generic Points Publisher ───────────────────────────────────
                # Converts /lane_status polynomial coefficients into a synthetic
                # LaserScan on /detection/scan so SLAM treats lane edges as
                # virtual obstacles.
                Node(
                    package='rover26_autonomy',
                    executable='generic_points_publisher',
                    name='generic_points_publisher',
                    parameters=[{'use_sim_time': True}],
                    output='screen',
                ),
                Node(
                    package='rover26_autonomy',
                    executable='lane_goal_publisher',
                    name='lane_goal_publisher',
                    parameters=[{'use_sim_time': True}],
                    output='screen',
                ),
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[nav2_params, {'use_sim_time': True}],
                ),
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[nav2_params, {'use_sim_time': True}],
                ),
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    parameters=[nav2_params, {'use_sim_time': True}],
                ),
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[nav2_params, {'use_sim_time': True}],
                ),
                Node(
                    package='nav2_costmap_2d',
                    executable='nav2_costmap_2d',
                    name='local_costmap',
                    output='screen',
                    parameters=[nav2_params, {'use_sim_time': True}],
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_nav2',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'autostart': True,
                        'node_names': [
                            'controller_server',
                            'planner_server',
                            'behavior_server',
                            'bt_navigator',
                            'local_costmap',
                        ],
                        'bond_timeout': 0.0,
                    }],
                ),
            ],
        ),

        # ══════════════════════════════════════════════════════════════════════
        #  t = 16 s — SLAM toolbox (starts unconfigured)
        # ══════════════════════════════════════════════════════════════════════
        # WHY t=16s?
        #   By now lane_detection + generic_points_publisher have been running
        #   4 seconds → /detection/scan has real data → scan_merger is
        #   producing a populated /scan_merged → SLAM's first scan match will
        #   succeed immediately when it's activated at t=21s.
        # ══════════════════════════════════════════════════════════════════════
        TimerAction(
            period=16.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    parameters=[mapper_params, {'use_sim_time': True}],
                    output='screen',
                ),
            ],
        ),

        # ══════════════════════════════════════════════════════════════════════
        #  t = 21 s — SLAM lifecycle manager (configure + activate)
        # ══════════════════════════════════════════════════════════════════════
        # Activating SLAM now means its FIRST scan match uses a fully populated
        # /scan_merged → good initial map → odom transform → no snap-back.
        # After activation, SLAM publishes the final TF edge: map → odom.
        # ══════════════════════════════════════════════════════════════════════
        TimerAction(
            period=28.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_slam',
                    parameters=[{
                        'use_sim_time': True,
                        'autostart': True,
                        'node_names': ['slam_toolbox'],
                        'bond_timeout': 0.0,
                    }],
                    output='screen',
                ),
            ],
        ),
    ])