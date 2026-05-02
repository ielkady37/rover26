#!/usr/bin/env python3
"""LoggerNode – ROS 2 node that publishes rover log entries on ``/rover/logs``.

Reads log entries from the Redis Stream ``rover:logs`` (written by every
:class:`utils.Logger.RoverLogger` instance across all nodes/services) and
publishes them as :class:`interfaces.msg.LogEntry` ROS messages.

Using Redis as the transport decouples producers from the ROS layer: any
process, thread, or future microservice can push logs without a direct
dependency on rclpy.

Redis Stream key: ``rover:logs``

Topic
-----
``/rover/logs``  (``interfaces/msg/LogEntry``, queue depth 100)

Launch
------
Add to your launch file::

    Node(package='control', executable='logger_node', name='logger_node')

Or run standalone::

    ros2 run control logger_node
"""

from datetime import datetime
import os

import rclpy
from rclpy.node import Node

from interfaces.msg import LogEntry
from utils.RedisClient import RedisClient

_STREAM_KEY = os.getenv('ROVER_LOG_STREAM_KEY', 'rover:logs')


class LoggerNode(Node):
    """Reads ``rover:logs`` Redis stream and publishes to ``/rover/logs``.

    Uses ``XREAD`` with a short blocking timeout so the ROS executor is
    not starved.  Tracks the last consumed stream ID so entries are never
    re-published after a restart (starts from the current tail).
    """

    _QOS_DEPTH = int(os.getenv('ROVER_LOG_QOS_DEPTH', '100'))
    _BLOCK_MS = int(os.getenv('ROVER_LOGGER_BLOCK_MS', '50'))
    _BATCH_SIZE = int(os.getenv('ROVER_LOGGER_BATCH', '200'))
    _DRAIN_HZ = float(os.getenv('ROVER_LOGGER_DRAIN_HZ', '20'))

    def __init__(self) -> None:
        super().__init__('logger_node')

        self._pub = self.create_publisher(LogEntry, '/rover/logs', self._QOS_DEPTH)
        self._rc = RedisClient()

        if not self._rc.available:
            self.get_logger().warn(
                'LoggerNode: Redis unavailable – /rover/logs will not receive entries. '
                'Start Redis and restart this node.'
            )

        # '$' means "only entries added after this moment" on first connect.
        # We store the last-seen stream ID so we never replay old entries.
        self._last_id = '$'

        # Use a wall-clock timer driven by ROVER_LOGGER_DRAIN_HZ
        self._timer = self.create_timer(1.0 / self._DRAIN_HZ, self._drain)
        self.get_logger().info('LoggerNode ready – publishing /rover/logs')

    # ------------------------------------------------------------------
    # Timer callback
    # ------------------------------------------------------------------

    def _drain(self) -> None:
        if not self._rc.available:
            return
        try:
            # XREAD returns [(stream_name, [(id, {fields}), ...])]
            results = self._rc.client.xread(
                {_STREAM_KEY: self._last_id},
                count=self._BATCH_SIZE,
                block=self._BLOCK_MS,
            )
            if not results:
                return
            for _stream_name, messages in results:
                for msg_id, fields in messages:
                    self._publish_entry(fields)
                    self._last_id = msg_id   # advance cursor
        except Exception as exc:
            self.get_logger().error(f'LoggerNode: XREAD error: {exc}')

    def _publish_entry(self, fields: dict) -> None:
        try:
            msg = LogEntry()
            msg.level = fields.get('level', '')
            msg.source_class = fields.get('source_class', '')
            msg.category = fields.get('category', '')
            msg.message = fields.get('message', '')

            ts_str = fields.get('timestamp', '')
            if ts_str:
                dt = datetime.fromisoformat(ts_str)
                epoch = dt.timestamp()
                msg.stamp.sec = int(epoch)
                msg.stamp.nanosec = int((epoch % 1) * 1_000_000_000)
            else:
                msg.stamp = self.get_clock().now().to_msg()

            self._pub.publish(msg)
        except Exception as exc:
            self.get_logger().error(f'LoggerNode: failed to publish entry: {exc}')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    """Spin the LoggerNode."""
    rclpy.init(args=args)
    node = LoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
