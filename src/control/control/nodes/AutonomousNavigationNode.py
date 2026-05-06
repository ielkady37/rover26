#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist

import time
from interfaces.msg import ActuatorCommand
from control.services.Navigation import Navigation
from control.services.Kinematics import Kinematics
from utils.Logger import RoverLogger
from utils.Configurator import Configurator

class AutonomousNavigationNode(LifecycleNode):
    """
    Lifecycle Node for autonomous navigation.
    Subscribes to Nav2's /cmd_vel, calculates precise physical kinematics, 
    and incorporates a safety watchdog timer.
    """

    def __init__(self):
        super().__init__('autonomous_navigation_node')
        self._log = RoverLogger()
        self.watchdog_timer = None
        self.last_cmd_time = time.time()
        self.watchdog_timeout = 0.5 # Default fallback

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info("Configuring AutonomousNavigationNode...")
        try:
            # 1. Fetch parameters via Configurator
            conf = Configurator()
            kin_data = conf.fetchData(Configurator.KINEMATICS)
            
            if not kin_data:
                self._log.warn("kinematics.yaml not found or empty. Using defensive fallbacks.")
                kin_data = {}

            track_width = float(kin_data.get('track_width_meters', 0.3))
            max_linear_vel = float(kin_data.get('max_linear_vel_mps', 0.3))
            max_angular_vel = float(kin_data.get('max_angular_vel_radps', 1.0))
            deadzone = float(kin_data.get('deadzone', 0.0))
            self.watchdog_timeout = float(kin_data.get('watchdog_timeout_sec', 0.5))
            max_pwm = int(kin_data.get('max_pwm', 255))

            # 2. Initialize Services
            self.kinematics = Kinematics(
                track_width=track_width, 
                max_linear_vel=max_linear_vel, 
                max_angular_vel=max_angular_vel
            )
            self.navigation_service = Navigation(deadzone=deadzone, max_pwm=max_pwm)

            # 3. Setup Publishers and Subscribers
            motor_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE)
            self.motor_pub = self.create_publisher(ActuatorCommand, '/esp_tx', motor_qos)

            cmd_vel_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, cmd_vel_qos)

            # 4. Setup Watchdog Timer (runs at 10Hz)
            self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)
            self.watchdog_timer.cancel() # Keep dormant until active

            self._log.succ("AutonomousNavigationNode configured successfully from YAML.")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self._log.err(f"Configuration failed: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info("Activating Autonomous Navigation...")
        super().on_activate(state)
        self.last_cmd_time = time.time()
        self.watchdog_timer.reset()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info("Deactivating Autonomous Navigation...")
        super().on_deactivate(state)
        self.watchdog_timer.cancel()
        self.publish_safe_stop()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info("Cleaning up resources...")
        self.destroy_timer(self.watchdog_timer)
        self.destroy_subscription(self.cmd_vel_sub)
        self.destroy_publisher(self.motor_pub)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info("Shutting down...")
        if self.watchdog_timer:
            self.watchdog_timer.cancel()
        self.publish_safe_stop()
        return TransitionCallbackReturn.SUCCESS

    def cmd_vel_callback(self, msg: Twist):
        try:
            self.last_cmd_time = time.time()

            linear_x = msg.linear.x
            angular_z = msg.angular.z

            left_norm, right_norm = self.kinematics.calculate_wheel_speeds(linear_x, angular_z)
            cmd_dto = self.navigation_service.calculate_from_wheel_speeds(left_norm, right_norm)

            if cmd_dto is None:
                self._log.err("cmd_dto is None. Skipping publish.")
                return

            act_msg = ActuatorCommand()
            act_msg.m2_speed = cmd_dto.left_pwm
            act_msg.m2_dir = cmd_dto.left_dir
            act_msg.m2_brake = cmd_dto.left_brake
            act_msg.m1_speed = cmd_dto.right_pwm
            act_msg.m1_dir = cmd_dto.right_dir
            act_msg.m1_brake = cmd_dto.right_brake

            self.motor_pub.publish(act_msg)

        except Exception as e:
            self._log.err(f"Exception in /cmd_vel callback: {e}")
            self.publish_safe_stop()

    def watchdog_callback(self):
        try:
            time_since_last_cmd = time.time() - self.last_cmd_time
            if time_since_last_cmd > self.watchdog_timeout:
                self.publish_safe_stop()
        except Exception as e:
            self._log.err(f"Watchdog error: {e}")

    def publish_safe_stop(self):
        try:
            msg = ActuatorCommand()
            msg.m1_brake = 1
            msg.m2_brake = 1
            self.motor_pub.publish(msg)
        except Exception as e:
            self._log.err(f"Failed to publish safe stop: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = AutonomousNavigationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Fatal node error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()