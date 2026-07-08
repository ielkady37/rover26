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
        self._pid_heading_locked = False
        self.imu_initialized = False  
        self.max_pwm = 255  # Dynamic fallback bound
        
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
            self.max_pwm = int(locomotion_data.get('max_pwm', 255))
            self.deadzone = float(locomotion_data.get('deadzone', 0.05))

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
                deadzone=self.deadzone, max_pwm=self.max_pwm, alpha=alpha, tolerance=tolerance
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
        self._pid_heading_locked = False
        self.imu_initialized = False  
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
            self.imu_initialized = True  
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

            # Handle user steering input via joystick
            if abs(yaw) > self.deadzone:
                self._pid_heading_locked = False
                if self.imu_initialized:
                    self.pid.update_setpoint(self.latest_yaw)
                active_yaw_effort = yaw
            else:
                # Handle automatic PID stabilizing holding lock
                if not self._pid_heading_locked and self.imu_initialized:
                    self.pid.update_setpoint(self.latest_yaw)
                    self._pid_heading_locked = True

                # Calculate shortest angular path error
                error = self.pid.setpoint - self.latest_yaw
                while error > 180.0:  error -= 360.0
                while error < -180.0: error += 360.0
                
                # Set a tight hold tolerance threshold (in degrees)
                HEADING_TOLERANCE_DEG = 1.5
                
                if abs(error) <= HEADING_TOLERANCE_DEG or not self.imu_initialized:
                    active_yaw_effort = 0.0
                else:
                    corrected_yaw = self.pid.setpoint - error
                    # Output is normalized (-1.0 to 1.0) for the locomotion engine
                    active_yaw_effort = -self.pid.stabilize(measured_value=corrected_yaw, dt=dt)

            # --- BYPASS NAVIGATION DEADZONE FOR PID ADJUSTMENTS ---
            if abs(yaw) <= self.deadzone and abs(active_yaw_effort) > 0.001:
                if abs(active_yaw_effort) < 0.1:
                    active_yaw_effort = 0.1 if active_yaw_effort > 0 else -0.1

            # Compute standard mix values
            cmd_dto = self.navigation_service.calculate_motor_commands(
                target_linear=throttle,
                target_angular=active_yaw_effort
            )

            if cmd_dto is None:
                return

            # --- DYNAMIC HARDWARE SPEED FLOORS & CEILINGS ---
            MIN_PWM_FLOOR = 40.0  # Dynamic starting floor threshold
            
            # Scale PWM outputs using both floor constraints and your loaded max_pwm configuration boundaries
            if abs(yaw) <= self.deadzone and abs(active_yaw_effort) > 0.001:
                # Handle Right Side Channel
                if abs(cmd_dto.right_pwm) > 0.1:
                    sign_r = 1.0 if cmd_dto.right_pwm >= 0 else -1.0
                    scaled_r = MIN_PWM_FLOOR + (abs(cmd_dto.right_pwm) / self.max_pwm) * (self.max_pwm - MIN_PWM_FLOOR)
                    cmd_dto.right_pwm = (sign_r * min(max(scaled_r, MIN_PWM_FLOOR), self.max_pwm)) * 1.5
                
                # Handle Left Side Channel
                if abs(cmd_dto.left_pwm) > 0.1:
                    sign_l = 1.0 if cmd_dto.left_pwm >= 0 else -1.0
                    scaled_l = MIN_PWM_FLOOR + (abs(cmd_dto.left_pwm) / self.max_pwm) * (self.max_pwm - MIN_PWM_FLOOR)
                    cmd_dto.left_pwm = (sign_l * min(max(scaled_l, MIN_PWM_FLOOR), self.max_pwm)) * 1.5

            # Populate outbound Actuator message
            msg = ActuatorCommand()
            msg.m1_speed = float(abs(cmd_dto.right_pwm))
            msg.m1_dir = int(cmd_dto.right_dir)
            msg.m1_brake = int(cmd_dto.right_brake)
            msg.m2_speed = float(abs(cmd_dto.left_pwm))
            msg.m2_dir = int(cmd_dto.left_dir)
            msg.m2_brake = int(cmd_dto.left_brake)

            # Override direction bits if speed mapping changed polarity
            if cmd_dto.right_pwm < 0: msg.m1_dir = 1 if msg.m1_dir == 0 else 0
            if cmd_dto.left_pwm < 0:  msg.m2_dir = 1 if msg.m2_dir == 0 else 0

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
    node = None
    try:
        node = ManualNavigationNode()

        configure_result = node.on_configure(None)
        if configure_result != TransitionCallbackReturn.SUCCESS:
            raise RuntimeError("ManualNavigationNode configuration failed")

        activate_result = node.on_activate(None)
        if activate_result != TransitionCallbackReturn.SUCCESS:
            raise RuntimeError("ManualNavigationNode activation failed")

        while rclpy.ok():
            try:
                rclpy.spin(node)
                break
            except KeyboardInterrupt:
                break
            except Exception as exc:
                # An invalid lifecycle request raises out of the executor and
                # would kill the process — log and keep spinning instead.
                node.get_logger().error(f'spin error (continuing): {exc}')
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Fatal node error: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()