#!/usr/bin/env python3
"""health_monitoring_test.launch.py

Launches the full health-monitoring and logging pipeline:

    1. Loads the correct .env file into the process environment
       (picks .env.rasp when ENV=REALTIME is already set, otherwise .env.local).
    2. Starts the Redis server (redis-server) if it is not already running.
    3. Waits 1 s for Redis to be ready, then starts:
           LoggerNode          – reads rover:logs Redis stream → /rover/logs
           HealthMonitorNode   – reads rover:health:* Redis keys → /rover/health
    4. Waits a further 1.5 s, then starts:
           MockSensorNode      – drives mock IMU / GPS / Ultrasonic sensors

Prerequisites
-------------
- redis-server must be installed:   sudo apt install redis-server
- ROS 2 workspace must be sourced:  source install/setup.bash
"""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, TimerAction
from launch_ros.actions import Node

from utils.EnvParams import EnvParams


# ---------------------------------------------------------------------------
# Load env (auto-selects .env.rasp → .env.local)
# ---------------------------------------------------------------------------

_ep = EnvParams()
_ENV_FILE = EnvParams.env_path or '(none – using built-in defaults)'

# Merge loaded env ON TOP of current process environment so that
# PYTHONPATH, LD_LIBRARY_PATH, ROS_* set by source install/setup.bash
# are preserved.
_MERGED_ENV = {**os.environ}


# ---------------------------------------------------------------------------
# Launch description
# ---------------------------------------------------------------------------

def generate_launch_description() -> LaunchDescription:
    """Build the launch description for the health monitoring test."""

    # --- 1. No extra SetEnvironmentVariable needed – EnvParams already
    #        injected all values into os.environ (and _MERGED_ENV reflects that) ---

    # --- 2. Start Redis (skip silently if already running) ---
    redis_host = _ep.get('ROVER_REDIS_HOST', 'localhost')
    redis_port = _ep.get('ROVER_REDIS_PORT', '6379')

    start_redis = ExecuteProcess(
        cmd=[
            'bash', '-c',
            # Check if redis-server is already listening on the configured port;
            # if not, start it in the foreground (the launch system manages its life).
            f'redis-cli -h {redis_host} -p {redis_port} ping 2>/dev/null | grep -q PONG'
            f' && echo "[launch] Redis already running on {redis_host}:{redis_port}"'
            f' || redis-server --bind {redis_host} --port {redis_port} --loglevel notice',
        ],
        output='screen',
        name='redis_server',
    )

    # --- 3. ROS nodes (delayed so Redis is ready) ---
    logger_node = Node(
        package='control',
        executable='logger_node',
        name='logger_node',
        output='screen',
        emulate_tty=True,
        env=_MERGED_ENV,
    )

    health_monitor_node = Node(
        package='control',
        executable='health_monitor_node',
        name='health_monitor_node',
        output='screen',
        emulate_tty=True,
        env=_MERGED_ENV,
    )

    # Give logger and health monitor a moment to connect to Redis
    mock_sensor_node = Node(
        package='control',
        executable='mock_sensor_node',
        name='mock_sensor_node',
        output='screen',
        emulate_tty=True,
        env=_MERGED_ENV,
    )

    return LaunchDescription([
        LogInfo(msg=f'[launch] Loading env from: {_ENV_FILE}'),

        LogInfo(msg=f'[launch] Starting Redis on {redis_host}:{redis_port}'),
        start_redis,

        # Wait 1 s for Redis, then start logger + health monitor
        TimerAction(period=1.0, actions=[
            LogInfo(msg='[launch] Starting LoggerNode and HealthMonitorNode'),
            logger_node,
            health_monitor_node,
        ]),

        # Wait a further 1.5 s, then start mock sensors
        TimerAction(period=2.5, actions=[
            LogInfo(msg='[launch] Starting MockSensorNode'),
            mock_sensor_node,
        ]),
    ])

