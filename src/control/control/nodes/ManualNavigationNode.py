#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time

from interfaces.msg import ActuatorCommand
from interfaces.msg import EulerAngles
from control.services.Navigation import Navigation
from control.services.PID import PIDController
from control.services.Joystick import CJoystick
from utils.Configurator import Configurator
from utils.Logger import RoverLogger

class ManualNavigationNode(LifecycleNode):

    def __init__(self):
        super().__init__('manual_navigation_node')
        self._log = RoverLogger()
        
        self.control_timer = None
        self.latest_yaw = 0.0 
        self.last_time = time.time()
        self.joystick = None
        self.deadzone = 0.0
        
        # Debug Counter to prevent terminal flooding
        self.debug_counter = 0

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info("Configuring ManualNavigationNode...")
        try:
            conf = Configurator()
            
            locomotion_data = conf.fetchData(Configurator.LOCOMOTION)
            if not locomotion_data:
                self._log.warn("locomotion.yaml not found! Using defensive fallbacks.")
                locomotion_data = {}

            loop_rate = float(locomotion_data.get('control_loop_rate_hz', 100.0))
            max_pwm = int(locomotion_data.get('max_pwm', 255))
            self.deadzone = float(locomotion_data.get('deadzone', 0.0))

            self.fwd_axis = locomotion_data.get('throttle_forward_axis', 'r2_axis')
            self.rev_axis = locomotion_data.get('throttle_reverse_axis', 'l2_axis')
            self.steer_axis = locomotion_data.get('steering_axis', 'left_x_axis')

            alpha = float(locomotion_data.get('smoothing_alpha', 0.05))
            tolerance = float(locomotion_data.get('smoothing_tolerance', 0.01))

            pid_data = conf.fetchData(Configurator.PID_PARAMS)
            if not pid_data:
                pid_data = {}
                
            kp = float(pid_data.get('yaw_KP', 0.0))
            ki = float(pid_data.get('yaw_KI', 0.0))
            kd = float(pid_data.get('yaw_KD', 0.0))

            # Initialize Services
            self.pid = PIDController(kp=kp, ki=ki, kd=kd)
            self.navigation_service = Navigation(
                deadzone=self.deadzone, max_pwm=max_pwm, alpha=alpha, tolerance=tolerance
            )
            self.joystick = CJoystick(is_writer=False) 

            # Setup Publishers & Subscribers
            motor_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE)
            self.motor_pub = self.create_publisher(ActuatorCommand, '/esp_tx', motor_qos)

            euler_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.euler_sub = self.create_subscription(EulerAngles, '/euler', self.euler_callback, euler_qos)

            # Setup Timer
            timer_period = 1.0 / loop_rate
            self.control_timer = self.create_timer(timer_period, self.control_loop_callback)
            self.control_timer.cancel() 

            self._log.succ("ManualNavigationNode configured successfully from YAML.")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self._log.err(f"Configuration failed: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info("Activating Manual Navigation...")
        super().on_activate(state)
        self.last_time = time.time()
        self.control_timer.reset() 
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info("Deactivating Manual Navigation...")
        super().on_deactivate(state)
        self.control_timer.cancel() 
        self.publish_safe_stop()    
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info("Cleaning up resources...")
        self.destroy_timer(self.control_timer)
        self.destroy_subscription(self.euler_sub)
        self.destroy_publisher(self.motor_pub)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info("Shutting down...")
        self.control_timer.cancel()
        self.publish_safe_stop()
        return TransitionCallbackReturn.SUCCESS

    def euler_callback(self, msg: EulerAngles):
        try:
            if msg.yaw != msg.yaw: 
                return
            self.latest_yaw = msg.yaw
        except Exception as e:
            self._log.err(f"Error processing /euler callback: {e}")

    def control_loop_callback(self):
        try:
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time

            axes = self.joystick.getAxis()
            
            fwd_val = axes.get(self.fwd_axis, 0.0) 
            rev_val = axes.get(self.rev_axis, 0.0)
            throttle = fwd_val - rev_val
            yaw = axes.get(self.steer_axis, 0.0)

            active_yaw_effort = yaw

            if abs(yaw) > self.deadzone:
                self.pid.update_setpoint(self.latest_yaw)
            elif abs(throttle) > self.deadzone:
                active_yaw_effort = self.pid.stabilize(measured_value=self.latest_yaw, dt=dt)
            else:
                self.pid.update_setpoint(self.latest_yaw)

            cmd_dto = self.navigation_service.calculate_motor_commands(
                target_linear=throttle,
                target_angular=active_yaw_effort
            )

            if cmd_dto is None:
                return
            
            # ------------------------------------------
            # Throttled Data Flow Logging
            # self.debug_counter += 1
            # if self.debug_counter >= 20:  # Triggers 5 times a second (assuming 100Hz loop)
            #     # Only print if we are actually commanding movement to keep idle logs clean
            #     if abs(throttle) > 0.01 or abs(yaw) > 0.01:
            #         print(f"\n--- DATA FLOW ---")
            #         print(f"INPUTS   -> Target Throttle: {throttle:+.2f} | Target Yaw: {active_yaw_effort:+.2f}")
            #         print(f"OUT LEFT -> PWM: {cmd_dto.left_pwm:3.0f} | Dir: {cmd_dto.left_dir} | Brake: {cmd_dto.left_brake}")
            #         print(f"OUT RGHT -> PWM: {cmd_dto.right_pwm:3.0f} | Dir: {cmd_dto.right_dir} | Brake: {cmd_dto.right_brake}")
            #     self.debug_counter = 0
            # ------------------------------------------

            msg = ActuatorCommand()
            msg.m1_speed = float(cmd_dto.right_pwm)
            msg.m1_dir = int(cmd_dto.right_dir)
            msg.m1_brake = int(cmd_dto.right_brake)
            msg.m2_speed = float(cmd_dto.left_pwm)
            msg.m2_dir = int(cmd_dto.left_dir)
            msg.m2_brake = int(cmd_dto.left_brake)

            self.motor_pub.publish(msg)

        except Exception as e:
            self._log.err(f"Exception in control loop: {e}")
            self.publish_safe_stop()

    def publish_safe_stop(self):
        try:
            msg = ActuatorCommand()
            msg.m1_brake = 1
            msg.m2_brake = 1
            msg.m1_speed = 0.0
            msg.m2_speed = 0.0
            self.motor_pub.publish(msg)

            if hasattr(self, 'navigation_service'):
                self.navigation_service.reset_momentum()
        except Exception as e:
            self._log.err(f"Failed to publish safe stop: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ManualNavigationNode()
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