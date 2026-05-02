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
import pathlib

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


# ---------------------------------------------------------------------------
# Locate project root and resolve the correct .env file
# ---------------------------------------------------------------------------

def _find_project_root() -> pathlib.Path:
    """Walk upward from this file until a /config directory is found."""
    current = pathlib.Path(__file__).resolve()
    for parent in current.parents:
        if (parent / 'config').is_dir():
            return parent
    return current.parents[3]   # reasonable fallback


def _load_env_file(path: pathlib.Path) -> dict:
    """Parse a .env file and return a {key: value} dict (skips comments)."""
    env = {}
    if not path.exists():
        return env
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            key, _, value = line.partition('=')
            env[key.strip()] = value.strip()
    return env


_PROJECT_ROOT = _find_project_root()

# Decide which .env to use: prefer .env.rasp in REALTIME, else .env.local
_ENV_MODE = os.environ.get('ENV', 'DEV')
_ENV_FILE = (
    _PROJECT_ROOT / '.env.rasp'
    if _ENV_MODE == 'REALTIME'
    else _PROJECT_ROOT / '.env.local'
)

# Load values now so Node env_overrides and SetEnvironmentVariable can use them
_ENV_VARS = _load_env_file(_ENV_FILE)

# Merge .env vars ON TOP of the current process environment so that
# PYTHONPATH, LD_LIBRARY_PATH, ROS_* etc. set by source install/setup.bash
# are preserved.  .env values take priority for any key they define.
_MERGED_ENV = {**os.environ, **_ENV_VARS}


# ---------------------------------------------------------------------------
# Launch description
# ---------------------------------------------------------------------------

def generate_launch_description() -> LaunchDescription:
    """Build the launch description for the health monitoring test."""

    # --- 1. Inject every .env variable into the launch process environment ---
    set_env_actions = [
        SetEnvironmentVariable(name=k, value=v)
        for k, v in _ENV_VARS.items()
    ]

    # --- 2. Start Redis (skip silently if already running) ---
    redis_host = _ENV_VARS.get('ROVER_REDIS_HOST', 'localhost')
    redis_port = _ENV_VARS.get('ROVER_REDIS_PORT', '6379')

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
        *set_env_actions,

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

