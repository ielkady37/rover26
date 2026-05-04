# rover26 – Setup Guide

> **Assumptions**: ROS 2 Humble is already installed at `/opt/ros/humble`.
> You have just cloned the repository.

---

## 1. Install system packages

```bash
sudo apt-get update
sudo apt-get install -y redis-server python3-pip python3-colcon-common-extensions
```

## 2. Install Python dependencies

```bash
pip install -r requirements.txt
```

## 3. Create your environment file

```bash
cp .env.example .env.local
```

Open `.env.local` and adjust any values you need (thresholds, Hz rates, Redis
connection, etc.). The defaults work for a dev laptop without changes.

## 4. Prepare the log directory

```bash
mkdir -p log/services log/nodes log/mocks log/helpers log/general
chown -R $USER:$USER log/
```

## 5. Set up the maps directory

Navigation maps saved by SLAM Toolbox are written to `$ROBOT_MAPS_DIR`. Add
this to your shell profile so the path is always resolved correctly regardless
of username or machine:

```bash
echo 'export ROBOT_MAPS_DIR="$HOME/maps"' >> ~/.bashrc
source ~/.bashrc
mkdir -p ~/maps
```

> The launch files fall back to `~/maps` if the variable is not set, but
> setting it explicitly is recommended.

## 6. Source ROS and build `interfaces` first

`interfaces` generates the custom ROS message types that `control` depends on,
so it must be built before the other packages.

```bash
source /opt/ros/humble/setup.bash

colcon build --packages-select interfaces
source install/setup.bash
```

## 7. Build `utils` and `control`

```bash
colcon build --packages-select utils control --symlink-install
source install/setup.bash
```

## 8. Fix Python package metadata (one-time, Python 3.10+)

`colcon --symlink-install` creates `.egg-link` files that Python 3.10's
`importlib.metadata` cannot read. Running `pip install -e` fixes this without
re-building anything:

```bash
pip install -e src/utils --no-build-isolation
pip install -e src/control --no-build-isolation
```

## 9. Verify the installation

```bash
python3 -c "from utils import RoverLogger, RedisClient, IHealthCheckable; print('utils  OK')"
python3 -c "from control.services.SystemHealthService import SystemHealthService; print('control OK')"
```

Both lines should print `OK` with no errors.

## 10. Launch the health monitoring stack

```bash
ros2 launch control health_monitoring_test.launch.py
```

The launch file will:
- Start `redis-server` automatically if it is not already running
- Load your `.env.local`
- Start `LoggerNode` + `HealthMonitorNode` after 1 s
- Start `MockSensorNode` (3 simulated sensors) after 2.5 s

## 11. Verify live data (separate terminals)

Source the workspace in every new terminal:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
```

Then:

```bash
# Live ROS log entries
ros2 topic echo /rover/logs

# Live health reports
ros2 topic echo /rover/health

# Log files written to disk
tail -f log/mocks/MockSensorService.log
tail -f log/nodes/HealthMonitorNode.log
tail -f log/services/SystemHealthService.log

# Inspect Redis directly
redis-cli XLEN rover:logs
redis-cli SMEMBERS rover:health:__registry__
```

---

## Order of operations — cheat sheet

| Step | Command | Why |
|------|---------|-----|
| System deps | `apt install redis-server …` | Redis must exist before any node starts |
| Python deps | `pip install -r requirements.txt` | psutil, redis-py, PyYAML |
| `.env.local` | `cp .env.example .env.local` | Launch file crashes without it |
| Log dirs | `mkdir -p log/{services,nodes,mocks,helpers,general}` | RoverLogger writes here |
| Maps dir | `echo 'export ROBOT_MAPS_DIR="$HOME/maps"' >> ~/.bashrc && mkdir -p ~/maps` | SLAM map output location |
| Build interfaces | `colcon build --packages-select interfaces` | Must precede control/utils |
| Source after build | `source install/setup.bash` | Registers generated types |
| Build rest | `colcon build … --symlink-install` | |
| Metadata fix | `pip install -e src/{utils,control} …` | Python 3.10 importlib compat |
