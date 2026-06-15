"""
autonomy.launch.py  —  Autonomous Navigation Stack (Hardware, No Vision)
──────────────────────────────────────────────────────────────────────────────
Pure autonomy stack for real hardware. No Gazebo. No vision pipeline.
Vision (lane_detection, pothole_detection, scan_merger) is integrated later.

Launches:
  • odom_tf_broadcaster  — computes /odom from /joint_states + /imu, broadcasts
                           odom → base_footprint TF
  • twist_mux            — velocity arbitration (Nav2 priority)
  • twist_unstamper      — TwistStamped → Twist for AutonomousNavigationNode
  • Nav2 stack           — path planning + navigation (reads /scan directly)
  • SLAM toolbox         — localization + mapping (reads /scan directly)
  • RViz2                — visualize LiDAR, robot, map; send 2D Nav Goals

ASSUMES control/run.launch.py is already running with:
  • robot_state_publisher  (URDF → /tf_static + wheel /tf from /joint_states)
  • AutonomousNavigationNode  (subscribes /cmd_vel → publishes /esp_tx to motors)
  • Hardware drivers:
      LiDAR   → /scan          (LaserScan,  lidar_link frame)
      IMU     → /imu           (Imu)
      Encoders→ /joint_states  (JointState)

Topic / TF flow:
  /joint_states + /imu
        ↓
  odom_tf_broadcaster  →  /odom  +  TF: odom → base_footprint
        ↓
  SLAM toolbox  ←  /scan  →  TF: map → odom  +  /map
        ↓
  Nav2 (planner + controller)  →  /cmd_vel_nav  (TwistStamped)
        ↓
  twist_mux  →  /cmd_vel_stamped  (TwistStamped, highest-priority winner)
        ↓
  twist_unstamper  →  /cmd_vel  (plain Twist)
        ↓
  AutonomousNavigationNode  →  /esp_tx  →  motors turn

Startup timeline:
  t= 0s  odom_tf_broadcaster  — TF chain: odom → base_footprint available
  t= 2s  twist_mux + twist_unstamper  — velocity pipeline ready
  t= 3s  Nav2 stack  — waiting for /scan + TF; will plan once SLAM activates
  t= 5s  SLAM toolbox (unconfigured)
  t= 9s  SLAM lifecycle_manager → activates SLAM → map → odom TF published
         Nav2 can now plan paths; set goals with "2D Goal Pose" in RViz
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    autonomy_pkg = get_package_share_directory('rover26_autonomy')

    # ── Config file paths ──────────────────────────────────────────────────
    twist_mux_params = os.path.join(autonomy_pkg, 'config', 'twist_mux.yaml')
    nav2_params      = os.path.join(autonomy_pkg, 'config', 'nav2_params.yaml')
    mapper_params    = os.path.join(autonomy_pkg, 'config', 'mapper_params_online_async.yaml')
    rviz_config      = os.path.join(autonomy_pkg, 'config', 'autonomy.rviz')

    return LaunchDescription([

        # ══════════════════════════════════════════════════════════════════════
        #  t = 0 s — Odometry TF broadcaster
        # ══════════════════════════════════════════════════════════════════════
        # Reads /joint_states (wheel encoders) + /imu (heading) from hardware.
        # Computes wheel odometry and publishes:
        #   • /odom          (nav_msgs/Odometry)
        #   • TF: odom → base_footprint  (the dynamic edge RSP cannot produce)
        #
        # MUST start first — SLAM, Nav2, and scan tf-lookups all need this TF.
        # Node(
        #     package='rover26_autonomy',
        #     executable='odom_tf_broadcaster',
        #     name='odom_tf_broadcaster',
        #     output='screen',
        # ),

        # ══════════════════════════════════════════════════════════════════════
        #  t = 0 s — RViz2
        # ══════════════════════════════════════════════════════════════════════
        # Visualize: robot model, /scan (LiDAR), /map (SLAM), TF tree.
        # Use the "2D Goal Pose" tool to send goals to Nav2.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),

        # ══════════════════════════════════════════════════════════════════════
        #  t = 2 s — Velocity pipeline
        # ══════════════════════════════════════════════════════════════════════
        # odom_tf_broadcaster has had 2s to establish the TF chain.
        # twist_mux arbitrates between Nav2 velocity commands (and any future
        # sources) and outputs the winner as TwistStamped on /cmd_vel_stamped.
        # twist_unstamper strips the header → plain Twist on /cmd_vel so
        # AutonomousNavigationNode can consume it directly.
        TimerAction(
            period=2.0,
            actions=[

                # ── Velocity arbitration ───────────────────────────────────
                Node(
                    package='twist_mux',
                    executable='twist_mux',
                    name='twist_mux',
                    parameters=[twist_mux_params],
                    remappings=[('/cmd_vel_out', '/cmd_vel_stamped')],
                    output='screen',
                ),

                # ── TwistStamped → Twist ───────────────────────────────────
                Node(
                    package='rover26_autonomy',
                    executable='twist_unstamper',
                    name='twist_unstamper',
                    output='screen',
                ),
            ],
        ),

        # ══════════════════════════════════════════════════════════════════════
        #  t = 3 s — Nav2 stack
        # ══════════════════════════════════════════════════════════════════════
        # controller_server, planner_server, behavior_server, bt_navigator all
        # read /scan directly (no scan_merger needed without vision pipeline).
        # They wait for a valid map from SLAM before planning; the lifecycle
        # manager below handles that sequencing automatically.
        TimerAction(
            period=3.0,
            actions=[

                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[nav2_params],
                ),

                Node(
                    package='nav2_velocity_smoother',
                    executable='velocity_smoother',
                    name='velocity_smoother',
                    output='screen',
                    parameters=[nav2_params],
                    remappings=[
                        ('cmd_vel',          'cmd_vel_smoothed'),
                        ('cmd_vel_smoothed', 'cmd_vel_nav'),
                    ],
                ),

                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[nav2_params],
                ),

                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    parameters=[nav2_params],
                ),

                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[nav2_params],
                ),

                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_nav2',
                    output='screen',
                    parameters=[{
                        'autostart': True,
                        'node_names': [
                            'controller_server',
                            'velocity_smoother',
                            'planner_server',
                            'behavior_server',
                            'bt_navigator',
                        ],
                        'bond_timeout': 0.0,
                    }],
                ),
            ],
        ),

        # ══════════════════════════════════════════════════════════════════════
        #  t = 5 s — SLAM toolbox (starts unconfigured)
        # ══════════════════════════════════════════════════════════════════════
        # Reads /scan directly. odom_tf_broadcaster has been running 5s so the
        # TF chain (odom → base_footprint → base_link → lidar_link) is solid.
        # Starts unconfigured; lifecycle_manager activates it at t=9s.
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    parameters=[mapper_params],
                    output='screen',
                ),
            ],
        ),

        # ══════════════════════════════════════════════════════════════════════
        #  t = 9 s — SLAM lifecycle manager (configure + activate)
        # ══════════════════════════════════════════════════════════════════════
        # Activating after 4s of SLAM running means the first scan match has
        # real /scan data → good initial map → no snap-back on first TF publish.
        # After activation, SLAM publishes the final TF edge: map → odom.
        # Nav2 can now plan paths. Use "2D Goal Pose" in RViz to set a goal.
        TimerAction(
            period=9.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_slam',
                    parameters=[{
                        'autostart': True,
                        'node_names': ['slam_toolbox'],
                        'bond_timeout': 0.0,
                    }],
                    output='screen',
                ),
            ],
        ),
    ])