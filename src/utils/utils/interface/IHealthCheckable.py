#!/usr/bin/env python3
"""IHealthCheckable – interface for services that expose a health-check endpoint.

Any service that wants to participate in the health-monitoring system must
subclass this ABC and implement :meth:`are_you_ok`.

Self-reporting via Redis
------------------------
The concrete :meth:`report_health` method (no need to override) handles
publishing the service's health state to Redis so the
:class:`control.nodes.HealthMonitorNode` can aggregate it without requiring
manual registration:

Redis keys used:
    ``rover:health:<ClassName>`` – Hash with fields:
        ok          – '1' (healthy) or '0' (unhealthy)
        reason      – human-readable problem description, or empty string
        timestamp   – ISO-8601 string of the last report time
    ``rover:health:__registry__`` – Set of all class names that have ever
        called :meth:`report_health`.

Entries in the hash expire after :data:`_HEALTH_TTL_SECONDS` seconds so
stale data does not mislead the monitor node.

Example::

    from utils.interface.IHealthCheckable import IHealthCheckable

    class IMUService(IHealthCheckable):
        def are_you_ok(self) -> tuple[bool, str | None]:
            if not self._initialized:
                return False, "IMU not initialized"
            if self._last_reading is None:
                return False, "No readings received yet"
            return True, None

        def _some_periodic_method(self):
            # push latest health state to Redis so HealthMonitorNode sees it
            self.report_health()
"""

from abc import ABC, abstractmethod
from datetime import datetime
from typing import Optional, Tuple
import os

# How long (seconds) a health entry lives in Redis before expiring.
# Services should call report_health() more frequently than this.
_HEALTH_TTL_SECONDS = int(os.getenv('ROVER_HEALTH_TTL_S', '30'))
_REGISTRY_KEY = os.getenv('ROVER_HEALTH_REGISTRY_KEY', 'rover:health:__registry__')


class IHealthCheckable(ABC):
    """Interface for services that can report their operational health.

    Implement :meth:`are_you_ok` to describe the service's internal state.
    Call :meth:`report_health` from your own timer or after significant state
    changes so that :class:`HealthMonitorNode` can pick up the latest status
    automatically via Redis — no manual registration required.
    """

    @abstractmethod
    def are_you_ok(self) -> Tuple[bool, Optional[str]]:
        """Check whether this service is healthy.

        Returns
        -------
        (True, None)
            When the service is operating normally.
        (False, "human-readable reason")
            When the service has detected a problem.
        """
        ...

    def report_health(self) -> None:
        """Push the current health state to Redis.

        Calls :meth:`are_you_ok`, stores the result in
        ``rover:health:<ClassName>`` and registers this class in
        ``rover:health:__registry__``.

        Safe to call from any thread.  Silently does nothing if Redis is
        unavailable.
        """
        try:
            from utils.RedisClient import RedisClient
            rc = RedisClient()
            if not rc.available:
                return

            class_name = type(self).__name__
            hash_key = f'rover:health:{class_name}'

            try:
                ok, reason = self.are_you_ok()
            except Exception as exc:
                ok, reason = False, f'are_you_ok() raised: {exc}'

            pipe = rc.client.pipeline()
            pipe.hset(hash_key, mapping={
                'ok': '1' if ok else '0',
                'reason': reason or '',
                'timestamp': datetime.now().isoformat(timespec='milliseconds'),
            })
            pipe.expire(hash_key, _HEALTH_TTL_SECONDS)
            pipe.sadd(_REGISTRY_KEY, class_name)
            pipe.execute()
        except Exception:
            # Never crash the caller due to Redis issues
            pass

