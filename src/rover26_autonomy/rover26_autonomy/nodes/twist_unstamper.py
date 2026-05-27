#!/usr/bin/env python3
"""
═════════════════════════════════════════════════════════════════════════════════
twist_unstamper.py  —  rover26_autonomy
═════════════════════════════════════════════════════════════════════════════════

FUNCTIONALITY
─────────────
Converts TwistStamped messages into plain Twist messages, bridging the gap
between twist_mux and the ROS–Gazebo bridge (ros_gz_bridge).

WHY THIS NODE EXISTS
────────────────────
In ROS 2 Jazzy, twist_mux outputs TwistStamped on /cmd_vel_stamped because
the stamped variant carries priority/timeout metadata needed by the mux.
However, ros_gz_bridge expects a plain Twist on /cmd_vel to drive the
Gazebo DiffDrive plugin.

This tiny relay strips the header and forwards only the velocity payload.

PIPELINE
────────
  twist_mux → /cmd_vel_stamped (TwistStamped)
                      │
              [twist_unstamper]        ← this node
                      │
               /cmd_vel (Twist)
                      │
              ros_gz_bridge → Gazebo DiffDrive

TOPICS & SUBSCRIPTIONS
──────────────────────
Subscribed:
  • /cmd_vel_stamped  [geometry_msgs/TwistStamped]  ← twist_mux

Published:
  • /cmd_vel          [geometry_msgs/Twist]         → ros_gz_bridge / motor driver

PARAMETERS (from config_params.py)
──────────────────────────────────
RosTopics.CMD_VEL_STAMPED   Input  topic name (/cmd_vel_stamped)
RosTopics.CMD_VEL           Output topic name (/cmd_vel)

═════════════════════════════════════════════════════════════════════════════════
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

# Topic names come from the central registry — no hardcoded strings.
from rover26_autonomy.config_params import RosTopics


# ═════════════════════════════════════════════════════════════════════════════
#  TWIST UNSTAMPER NODE
# ═════════════════════════════════════════════════════════════════════════════

class TwistUnstamper(Node):
    """
    Strips the header from TwistStamped messages and republishes the
    bare Twist payload so ros_gz_bridge can forward it to Gazebo.

    Attributes:
        _pub (Publisher): Output publisher on RosTopics.CMD_VEL (/cmd_vel).
    """

    def __init__(self):
        super().__init__('twist_unstamper')

        # ── Publisher: plain Twist on /cmd_vel ─────────────────────────────────
        # Depth 10 matches the typical QoS used by ros_gz_bridge.
        self._pub = self.create_publisher(
            Twist,
            RosTopics.CMD_VEL,   # '/cmd_vel' from the central topic registry
            10,
        )

        # ── Subscriber: TwistStamped from twist_mux on /cmd_vel_stamped ────────
        self.create_subscription(
            TwistStamped,
            RosTopics.CMD_VEL_STAMPED,   # '/cmd_vel_stamped' from the central registry
            self._cb,
            10,
        )

        self.get_logger().info(
            f'twist_unstamper ready: '
            f'{RosTopics.CMD_VEL_STAMPED} → {RosTopics.CMD_VEL}'
        )

    # =========================================================================
    #  CALLBACK
    # =========================================================================

    def _cb(self, msg: TwistStamped) -> None:
        """
        Called for every TwistStamped from twist_mux.

        Extracts msg.twist (the bare Twist payload — linear + angular
        velocities) and forwards it immediately.  The header (stamp,
        frame_id, sequence) is intentionally discarded — ros_gz_bridge
        does not need it and the Gazebo DiffDrive plugin uses the plain
        Twist interface.

        Args:
            msg: TwistStamped from twist_mux containing velocity commands.
        """
        # Forward only the velocity payload — strip the header entirely.
        self._pub.publish(msg.twist)


# ═════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = TwistUnstamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('twist_unstamper shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()