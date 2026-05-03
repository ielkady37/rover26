#!/usr/bin/env python3
"""HealthMonitorNode – ROS 2 node that publishes system health on ``/rover/health``.

Aggregates two sources of health data and publishes them as a
:class:`interfaces.msg.HealthReport` every second:

1. **System metrics** – CPU, RAM, temperature, disk, WiFi latency/bandwidth,
   battery via :class:`control.services.SystemHealthService`.

2. **Service health states** – Any class that implements
   :class:`utils.interface.IHealthCheckable` and calls
   ``self.report_health()`` writes its state to Redis automatically.  This
   node reads the ``rover:health:__registry__`` set to discover all such
   services and fetches their latest ``rover:health:<ClassName>`` hash
   without requiring manual registration.

Redis keys consumed:
    ``rover:health:__registry__``   – Set of service class names
    ``rover:health:<ClassName>``    – Hash: ok / reason / timestamp

Topic
-----
``/rover/health``  (``interfaces/msg/HealthReport``, queue depth 10)

Launch
------
Add to your launch file::

    Node(package='control', executable='health_monitor_node',
         name='health_monitor_node')

Or run standalone::

    ros2 run control health_monitor_node
"""

import os

import rclpy
from rclpy.node import Node

from interfaces.msg import HealthReport
from control.services.SystemHealthService import SystemHealthService
from utils.RedisClient import RedisClient
from utils.Logger import RoverLogger

_REGISTRY_KEY = os.getenv('ROVER_HEALTH_REGISTRY_KEY', 'rover:health:__registry__')
_STALE_THRESHOLD_S = int(os.getenv('ROVER_HEALTH_STALE_S', '15'))


class HealthMonitorNode(Node):
    """Collects system metrics and all Redis-reported service health states.

    Services automatically appear in the health report as soon as they
    call ``self.report_health()`` — no wiring needed here.
    """

    _PUBLISH_HZ = float(os.getenv('ROVER_HEALTH_PUBLISH_HZ', '1'))
    _QOS_DEPTH = int(os.getenv('ROVER_HEALTH_QOS_DEPTH', '10'))

    def __init__(self) -> None:
        super().__init__('health_monitor_node')

        self._log = RoverLogger(file_logging=True)
        self._health_svc = SystemHealthService()
        self._rc = RedisClient()

        if not self._rc.available:
            self._log.warn(
                'HealthMonitorNode: Redis unavailable – service health checks '
                'will be empty. Start Redis and restart this node.'
            )

        self._pub = self.create_publisher(HealthReport, '/rover/health', self._QOS_DEPTH)
        self._timer = self.create_timer(1.0 / self._PUBLISH_HZ, self._publish_health)

        self._log.info('HealthMonitorNode ready – publishing /rover/health at 1 Hz')

    # ------------------------------------------------------------------
    # Graceful shutdown
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        """Cancel timer, close Redis, flush logger before ROS teardown."""
        try:
            self._timer.cancel()
        except Exception:
            pass
        try:
            self._rc.close()
        except Exception:
            pass
        try:
            self._log.info('HealthMonitorNode shutting down')
        except Exception:
            pass
        super().destroy_node()

    # ------------------------------------------------------------------
    # Timer callback
    # ------------------------------------------------------------------

    def _publish_health(self) -> None:
        try:
            # --- System metrics ---
            metrics = self._health_svc.get_metrics()

            # SystemHealthService self-reports its own state to Redis
            self._health_svc.report_health()

            # --- Service health states from Redis ---
            svc_statuses = self._read_service_health()

            # --- Build ROS message ---
            msg = HealthReport()
            msg.cpu_percent = float(metrics['cpu_percent'])
            msg.ram_percent = float(metrics['ram_percent'])
            msg.cpu_temp_celsius = float(metrics['cpu_temp_celsius'])
            msg.wifi_latency_ms = float(metrics['wifi_latency_ms'])
            msg.wifi_bandwidth_mbps = float(metrics['wifi_bandwidth_mbps'])
            msg.disk_percent = float(metrics['disk_percent'])
            msg.battery_percent = float(metrics['battery_percent'])
            msg.battery_plugged = bool(metrics['battery_plugged'])
            msg.gateway_ip = str(metrics['gateway_ip'])
            msg.health_checks = svc_statuses
            msg.stamp = self.get_clock().now().to_msg()

            self._pub.publish(msg)

            # Warn via logger for any degraded state
            ok, reason = self._health_svc.are_you_ok()
            if not ok:
                self._log.warn(f'System health degraded: {reason}')
            for status in svc_statuses:
                if ':error:' in status or ':exception:' in status or ':stale' in status:
                    self._log.warn(f'Service health issue – {status}')

        except Exception as exc:
            self._log.err(f'HealthMonitorNode publish error: {exc}')

    # ------------------------------------------------------------------
    # Redis health reader
    # ------------------------------------------------------------------

    def _read_service_health(self) -> list:
        """Read all service health entries from Redis.

        Returns
        -------
        list of str
            Each entry is one of:
            - ``"ClassName:ok"``
            - ``"ClassName:error:<reason>"``
            - ``"ClassName:stale"``   (entry TTL expired / very old)
        """
        statuses = []
        if not self._rc.available:
            return statuses
        try:
            from datetime import datetime, timezone

            members = self._rc.client.smembers(_REGISTRY_KEY)
            for class_name in members:
                hash_key = f'rover:health:{class_name}'
                data = self._rc.client.hgetall(hash_key)
                if not data:
                    # Key expired – entry is stale
                    statuses.append(f'{class_name}:stale')
                    continue

                ok = data.get('ok', '0') == '1'
                reason = data.get('reason', '')

                if ok:
                    statuses.append(f'{class_name}:ok')
                else:
                    statuses.append(f'{class_name}:error:{reason or "unknown"}')
        except Exception as exc:
            self.get_logger().error(f'HealthMonitorNode: Redis read error: {exc}')
        return statuses


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    """Spin the HealthMonitorNode."""
    rclpy.init(args=args)
    node = HealthMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
