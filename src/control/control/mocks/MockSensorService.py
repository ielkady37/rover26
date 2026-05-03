#!/usr/bin/env python3
"""MockSensorService – concrete mock of a sensor service.

Simulates a generic sensor that:
- Produces periodic fake readings (configurable via :meth:`tick`).
- Implements :meth:`are_you_ok` through :class:`MockHealthCheckableBase`.
- Writes every reading and state change to its log file
  (``log/services/MockSensorService.log``) via :class:`utils.Logger.RoverLogger`.
- Pushes health state to Redis on every tick so
  :class:`control.nodes.HealthMonitorNode` sees it automatically.

Intended use
------------
Use this service directly in integration tests, launch-file demos, or as a
blueprint when writing real sensor services::

    svc = MockSensorService(sensor_name="IMU", fault_rate=0.05)

    # Manually inject a fault:
    svc.set_mock_state(False, "I2C bus timeout")

    # Drive it from a timer:
    svc.tick()

    # Restore health:
    svc.set_mock_state(True)
"""

import random
import threading
from typing import Optional

from control.mocks.MockHealthCheckableBase import MockHealthCheckableBase


class MockSensorService(MockHealthCheckableBase):
    """Simulates a named sensor service with randomised readings.

    Parameters
    ----------
    sensor_name:
        Human-friendly name shown in log output (e.g. ``"IMU"``, ``"GPS"``).
    fault_rate:
        Spontaneous fault probability per tick (0.0 – 1.0). Default 0.0.
    value_range:
        ``(min, max)`` range for the simulated float reading.
    """

    def __init__(
        self,
        sensor_name: str = 'GenericSensor',
        fault_rate: float = 0.0,
        value_range: tuple = (0.0, 100.0),
    ) -> None:
        super().__init__(fault_rate=fault_rate)
        self._sensor_name = sensor_name
        self._value_range = value_range
        self._last_value: Optional[float] = None
        self._tick_count: int = 0
        self._tick_lock = threading.Lock()

        self._log.succ(
            f'MockSensorService ready – sensor="{sensor_name}" '
            f'range={value_range} fault_rate={fault_rate:.0%}'
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def tick(self) -> float:
        """Simulate one sensor reading cycle.

        - Generates a random value within ``value_range``.
        - Logs the reading at INFO level (or WARN if a fault is active).
        - Calls :meth:`report_health` to push state to Redis.

        Returns
        -------
        float
            The simulated sensor reading.
        """
        with self._tick_lock:
            self._tick_count += 1
            count = self._tick_count

        value = round(random.uniform(*self._value_range), 3)

        with self._lock:
            self._last_value = value

        ok, reason = self.are_you_ok()

        if ok:
            self._log.info(
                f'[{self._sensor_name}] tick={count:06d}  value={value}'
            )
        else:
            self._log.warn(
                f'[{self._sensor_name}] tick={count:06d}  value={value}  '
                f'FAULT: {reason}'
            )

        # Push to Redis health registry (thread-safe via pipeline)
        self.report_health()

        self._simulate_work()
        return value

    @property
    def last_value(self) -> Optional[float]:
        """Most recent simulated reading, or None before the first tick."""
        return self._last_value

    @property
    def tick_count(self) -> int:
        """Total number of ticks executed."""
        return self._tick_count

    # ------------------------------------------------------------------
    # MockHealthCheckableBase override
    # ------------------------------------------------------------------

    def _simulate_work(self) -> None:
        """Log additional domain-specific detail every 10 ticks."""
        if self._tick_count % 10 == 0:
            self._log.info(
                f'[{self._sensor_name}] --- 10-tick summary: '
                f'last={self._last_value}  total_ticks={self._tick_count}'
            )
