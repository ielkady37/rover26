#!/usr/bin/env python3
"""
═════════════════════════════════════════════════════════════════════════════════
twist_unstamper.py  —  rover26_autonomy
═════════════════════════════════════════════════════════════════════════════════

FUNCTIONALITY
─────────────
Converts TwistStamped messages into plain Twist messages, bridging the gap
between twist_mux and AutonomousNavigationNode.

WHY THIS NODE EXISTS
────────────────────
In ROS 2 Jazzy, twist_mux outputs TwistStamped on /cmd_vel_stamped because
the stamped variant carries the priority/timeout metadata needed by the mux.
However, AutonomousNavigationNode subscribes to plain geometry_msgs/Twist on
/cmd_vel (it has no use for the header).

This tiny relay strips the header and forwards only the velocity payload.

PIPELINE
────────
  Nav2 controller_server → /cmd_vel_nav (TwistStamped)
                │
           twist_mux  (arbitrates priority)
                │
       /cmd_vel_stamped (TwistStamped)
                │
        [twist_unstamper]    ← this node
                │
          /cmd_vel (Twist)
                │
  AutonomousNavigationNode → /esp_tx → motors

TOPICS & SUBSCRIPTIONS
──────────────────────
Subscribed:
  • /cmd_vel_stamped  [geometry_msgs/TwistStamped]  ← twist_mux

Published:
  • /cmd_vel          [geometry_msgs/Twist]         → AutonomousNavigationNode

PARAMETERS (from config_params.py)
──────────────────────────────────
RosTopics.CMD_VEL_STAMPED   Input  topic name (/cmd_vel_stamped)
RosTopics.CMD_VEL           Output topic name (/cmd_vel)

═════════════════════════════════════════════════════════════════════════════════
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

from rover26_autonomy.config_params import RosTopics


# ═════════════════════════════════════════════════════════════════════════════
#  TWIST UNSTAMPER NODE
# ═════════════════════════════════════════════════════════════════════════════

class TwistUnstamper(Node):
    """
    Strips the header from TwistStamped messages and republishes the
    bare Twist payload so AutonomousNavigationNode can consume it directly.

    Attributes:
        _pub (Publisher): Output publisher on RosTopics.CMD_VEL (/cmd_vel).
    """

    def __init__(self):
        super().__init__('twist_unstamper')

        # ── Publisher: plain Twist on /cmd_vel ─────────────────────────────────
        self._pub = self.create_publisher(
            Twist,
            RosTopics.CMD_VEL,
            10,
        )

        # ── Subscriber: TwistStamped from twist_mux on /cmd_vel_stamped ────────
        self.create_subscription(
            TwistStamped,
            RosTopics.CMD_VEL_STAMPED,
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

        Extracts msg.twist (linear + angular velocities) and publishes it
        immediately as a plain Twist. The header (stamp, frame_id) is
        intentionally discarded — AutonomousNavigationNode does not need it.

        Args:
            msg: TwistStamped from twist_mux containing velocity commands.
        """
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
        node.get_logger().info('twist_unstamper shutting down…')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()