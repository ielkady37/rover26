#!/usr/bin/env python3
"""MockHealthCheckableBase – abstract base for mock IHealthCheckable services.

Extends :class:`utils.interface.IHealthCheckable` with test-oriented helpers:

- Inject a fake health state at any time via :meth:`set_mock_state`.
- Override fault injection rate with :meth:`set_fault_rate` for probabilistic
  failure simulation.
- All health transitions are logged through :class:`utils.Logger.RoverLogger`
  and reported to Redis via :meth:`report_health`.

Concrete mocks should subclass this and optionally override
:meth:`_simulate_work` to add domain-specific behaviour.

Example::

    class MockIMUService(MockHealthCheckableBase):
        def _simulate_work(self):
            self._log.info("IMU reading: ax=0.01 ay=0.00 az=9.81")
"""

import random
import threading
from typing import Optional, Tuple

from utils.interface.IHealthCheckable import IHealthCheckable
from utils.Logger import RoverLogger


class MockHealthCheckableBase(IHealthCheckable):
    """Abstract mock base for any IHealthCheckable service.

    Parameters
    ----------
    fault_rate:
        Probability (0.0 – 1.0) that :meth:`are_you_ok` spontaneously
        returns False to simulate flapping.  Default 0.0 (always healthy).
    """

    def __init__(self, fault_rate: float = 0.0) -> None:
        self._log = RoverLogger(file_logging=True)
        self._lock = threading.Lock()
        self._fault_rate = max(0.0, min(1.0, fault_rate))

        # Injected state: (ok, reason).  None means use probabilistic logic.
        self._injected: Optional[Tuple[bool, Optional[str]]] = None

        self._log.info(
            f'{type(self).__name__} mock created '
            f'(fault_rate={self._fault_rate:.0%})'
        )

    # ------------------------------------------------------------------
    # IHealthCheckable
    # ------------------------------------------------------------------

    def are_you_ok(self) -> Tuple[bool, Optional[str]]:
        """Return injected state or probabilistically simulate a fault."""
        with self._lock:
            if self._injected is not None:
                return self._injected

            if self._fault_rate > 0.0 and random.random() < self._fault_rate:
                reason = f'{type(self).__name__}: simulated fault triggered'
                return False, reason

            return True, None

    # ------------------------------------------------------------------
    # Mock control API
    # ------------------------------------------------------------------

    def set_mock_state(self, ok: bool, reason: Optional[str] = None) -> None:
        """Inject a fixed health state that overrides probabilistic logic.

        Parameters
        ----------
        ok:
            True → healthy, False → unhealthy.
        reason:
            Human-readable fault description (ignored when ok=True).
        """
        with self._lock:
            self._injected = (ok, reason if not ok else None)
        level = 'succ' if ok else 'warn'
        getattr(self._log, level)(
            f'{type(self).__name__} mock state → '
            f'{"OK" if ok else f"FAULT: {reason}"}'
        )
        self.report_health()

    def clear_mock_state(self) -> None:
        """Remove injected state and revert to probabilistic logic."""
        with self._lock:
            self._injected = None
        self._log.info(f'{type(self).__name__} mock state cleared – using fault_rate')

    def set_fault_rate(self, rate: float) -> None:
        """Change the spontaneous fault probability (0.0 – 1.0)."""
        with self._lock:
            self._fault_rate = max(0.0, min(1.0, rate))
        self._log.info(
            f'{type(self).__name__} fault_rate updated → {self._fault_rate:.0%}'
        )

    # ------------------------------------------------------------------
    # Optional override for domain-specific simulation
    # ------------------------------------------------------------------

    def _simulate_work(self) -> None:
        """Override in concrete mocks to emit domain-specific log lines."""
        pass
