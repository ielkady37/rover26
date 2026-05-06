# Copilot Instructions — rover26

## Project Overview
ROS 2 (ament_python) rover control stack with three packages:
- **`interfaces`** (ament_cmake): Defines custom ROS messages (e.g., `MotorCommands.msg`). Must be built before other packages.
- **`utils`**: Shared utilities — `Configurator` (YAML config loader) and `EnvParams` (.env loader).
- **`control`**: Main control logic — nodes, services, DTOs, and exceptions.

## Architecture & Data Flow
```
/joystick (ROS topic) → JoystickNode → SharedMemory ("joystick_data")
                                              ↓
                                   ManualNavigationNode (polls shared memory)
                                              ↓
                          Navigation facade → Steering → DirEvaluator → PWMMapper
                                         ↘ PIDController (yaw stabilization)
                                              ↓
                                   /motor_commands (MotorCommands msg)
```
- Joystick data is passed between processes via **POSIX shared memory** (pickle-serialized). `JoystickNode` is `is_writer=True`; `ManualNavigationNode` is `is_writer=False`.
- `CJoystick` is a **singleton** — only one instance per process.
- `Navigation` is a **facade** wrapping `Steering`, `DirEvaluator`, `PWMMapper`, and `PIDController`.

## Key Conventions
- **MotorCommandDTO** (`control/DTOs/`) is the internal data object; `MotorCommands` (ROS msg) is only used at the publish boundary in the node.
- **Defensive coding is mandatory**: every callback and service method must handle `NaN`/`Inf` inputs and wrap logic in `try/except`. Faults fall back to safe-stop, not crashes.
- **PID yaw stabilization**: active only during straight-line throttle. When yaw stick is moved, PID setpoint syncs to current heading. When idle, setpoint also syncs.
- `control_loop_callback` and the timer in `ManualNavigationNode` are **currently commented out** (WIP). Re-enable both together when wiring up the loop.

## Configuration
All YAML configs live in `/config/` at the project root:
- `hardware_pins.yaml` → `Configurator.PINS`
- `joystick_buttons.yaml` → `Configurator.BUTTONS`
- `pid_ks.yaml` → `Configurator.PID_PARAMS`

`Configurator.getProjectRoot()` walks up the directory tree to find the `/config` folder — do not move it.

Button constants are dynamically assigned as class attributes on `CJoystick` at init time from `joystick_buttons.yaml`. Access via `CJoystick.BUTTON_NAME` or `CJoystick._<number>`.

## Build & Run
```bash
# From workspace root
colcon build --symlink-install
source install/setup.bash

# Run launch file
ros2 launch control run.launch.py

# Run tests (per package)
colcon test --packages-select control
colcon test --packages-select utils
```
> `interfaces` must be built first since `control` and `utils` depend on it.

## Testing
- Unit tests are in `src/<pkg>/test/`. Service-level tests are in `test_actuator_services.py`.
- Linter tests (`flake8`, `pep257`, `copyright`) run automatically via `colcon test`.
- `test_copyright.py` is skipped until copyright headers are added to source files.
- To run a single test file directly: `python3 -m pytest src/control/test/test_actuator_services.py -v`

## Adding New Services
1. Place in `src/control/control/services/` and export from `__init__.py`.
2. If hardware-facing, add pin entries to `config/hardware_pins.yaml` and read via `Configurator.PINS`.
3. Wire into `Navigation` facade rather than directly into the node.
4. Add a corresponding `SensorXxxError` in `control/exceptions/` if it wraps hardware I/O.
