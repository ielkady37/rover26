#!/usr/bin/env python3
"""MockSensorNode – ROS 2 node that drives MockSensorService and logs via Redis.

This node demonstrates the full logger + health-monitor pipeline:

1. Instantiates :class:`control.mocks.MockSensorService` for an arbitrary set
   of named sensors.
2. Calls ``tick()`` on each sensor at :data:`_TICK_HZ` Hz.
3. Every log call inside the service is pushed to the Redis stream
   ``rover:logs`` (consumed by :class:`control.nodes.LoggerNode`
   which publishes to ``/rover/logs``).
4. Every ``tick()`` calls ``report_health()`` which writes to
   ``rover:health:<ClassName>`` and registers in
   ``rover:health:__registry__`` (consumed by
   :class:`control.nodes.HealthMonitorNode` which publishes to
   ``/rover/health``).

No direct ROS publisher is needed here – the Redis pipeline is the
thread-safe transport shared with the rest of the stack.

Topics published (indirectly via Redis → ROS nodes)
-----------------------------------------------------
``/rover/logs``    – populated by LoggerNode reading ``rover:logs``
``/rover/health``  – populated by HealthMonitorNode reading Redis health keys

Launch
------
Add to your launch file::

    Node(package='control', executable='mock_sensor_node',
         name='mock_sensor_node')

Or run standalone::

    ros2 run control mock_sensor_node
"""

import os

import rclpy
from rclpy.node import Node

from control.mocks.MockSensorService import MockSensorService
from utils.Logger import RoverLogger

_TICK_HZ = float(os.getenv('ROVER_MOCK_TICK_HZ', '2'))


class MockSensorNode(Node):
    """Drives one or more MockSensorService instances and logs every reading.

    Sensors are defined in :meth:`_init_sensors`.  Add or remove entries
    there to control what is simulated.
    """

    def __init__(self) -> None:
        super().__init__('mock_sensor_node')
        self._log = RoverLogger(file_logging=True)

        self._sensors = self._init_sensors()
        self._timer = self.create_timer(1.0 / _TICK_HZ, self._tick_all)

        self._log.succ(
            f'MockSensorNode ready – {len(self._sensors)} sensor(s) '
            f'at {_TICK_HZ} Hz'
        )

    # ------------------------------------------------------------------
    # Graceful shutdown
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        """Cancel timer and log shutdown before ROS teardown."""
        try:
            self._timer.cancel()
        except Exception:
            pass
        try:
            self._log.info('MockSensorNode shutting down')
        except Exception:
            pass
        super().destroy_node()

    # ------------------------------------------------------------------
    # Sensor definitions – edit here to add/remove mock sensors
    # ------------------------------------------------------------------

    def _init_sensors(self) -> list:
        """Instantiate all mock sensors."""
        sensors = [
            MockSensorService(
                sensor_name='IMU',
                fault_rate=0.0,
                value_range=(-10.0, 10.0),
            ),
            MockSensorService(
                sensor_name='GPS',
                fault_rate=0.02,    # 2 % spontaneous fault rate
                value_range=(0.0, 180.0),
            ),
            MockSensorService(
                sensor_name='Ultrasonic',
                fault_rate=0.0,
                value_range=(2.0, 400.0),
            ),
        ]
        for svc in sensors:
            self._log.info(f'Registered mock sensor: {svc._sensor_name}')
        return sensors

    # ------------------------------------------------------------------
    # Timer callback
    # ------------------------------------------------------------------

    def _tick_all(self) -> None:
        """Tick every sensor once; each tick logs and reports health."""
        for svc in self._sensors:
            try:
                svc.tick()
            except Exception as exc:
                self._log.err(
                    f'MockSensorNode: unhandled error in '
                    f'{svc._sensor_name}.tick(): {exc}'
                )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    """Spin the MockSensorNode."""
    rclpy.init(args=args)
    node = MockSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
