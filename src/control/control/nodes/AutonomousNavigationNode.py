#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist

import time
from interfaces.msg import ActuatorCommand
from interfaces.msg import EulerAngles
from control.services.Navigation import Navigation
from control.services.Kinematics import Kinematics
from control.services.PID import PIDController
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

        # --- heading-assist PID state (trim only, never overrides throttle) ---
        self.latest_yaw = 0.0
        self.imu_initialized = False
        self._heading_locked = False
        self._last_heading_pid_time = time.time()
        self.max_heading_correction = 0.3  # rad/s, overwritten from YAML in on_configure
        self._straight_threshold = 0.05    # rad/s — below this, Nav2 is considered "going straight"

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info("Configuring AutonomousNavigationNode...")
        try:
            # 1. Fetch parameters via Configurator
            conf = Configurator()
            kin_data = conf.fetchData(Configurator.LOCOMOTION)
            
            if not kin_data:
                self._log.warn("kinematics.yaml not found or empty. Using defensive fallbacks.")
                kin_data = {}

            track_width = float(kin_data.get('track_width_meters', 0.3))
            max_linear_vel = float(kin_data.get('max_linear_vel_mps', 0.3))
            max_angular_vel = float(kin_data.get('max_angular_vel_radps', 1.0))
            deadzone = float(kin_data.get('deadzone', 0.0))
            self.watchdog_timeout = float(kin_data.get('watchdog_timeout_sec', 0.5))
            max_pwm = int(kin_data.get('max_pwm', 255))
            self.rotation_gain = float(kin_data.get('rotation_gain', 2.0))

            # Heading-assist trim bounds (kept small on purpose — this is a
            # correction on top of Nav2's own angular command, not a substitute
            # for it, so it should never be able to dominate a forward drive).
            self.max_heading_correction = float(kin_data.get('max_heading_correction_radps', 0.3))
            self._straight_threshold = float(kin_data.get('heading_straight_threshold_radps', 0.05))

            # 2. Initialize Services
            self.kinematics = Kinematics(
                track_width=track_width, 
                max_linear_vel=max_linear_vel, 
                max_angular_vel=max_angular_vel
            )
            self.navigation_service = Navigation(deadzone=deadzone, max_pwm=max_pwm)

            # Heading-assist PID: corrects drift only while Nav2 is commanding
            # near-straight motion. Reuses the PID_PARAMS config block with its
            # own dedicated gain keys so it doesn't fight ManualNavigationNode's
            # heading-hold gains.
            pid_data = conf.fetchData(Configurator.PID_PARAMS)
            if not pid_data:
                pid_data = {}
            hp_kp = float(pid_data.get('auto_heading_KP', 0.0))
            hp_ki = float(pid_data.get('auto_heading_KI', 0.0))
            hp_kd = float(pid_data.get('auto_heading_KD', 0.0))
            self.heading_pid = PIDController(kp=hp_kp, ki=hp_ki, kd=hp_kd)

            # 3. Setup Publishers and Subscribers
            motor_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE)
            self.motor_pub = self.create_publisher(ActuatorCommand, '/esp_tx', motor_qos)

            cmd_vel_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, cmd_vel_qos)

            euler_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.euler_sub = self.create_subscription(EulerAngles, '/euler', self.euler_callback, euler_qos)

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
        self._last_heading_pid_time = time.time()
        self._heading_locked = False  # never resume a stale heading lock from a previous run
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
        self.destroy_subscription(self.euler_sub)
        self.destroy_publisher(self.motor_pub)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._log.info("Shutting down...")
        if self.watchdog_timer:
            self.watchdog_timer.cancel()
        self.publish_safe_stop()
        return TransitionCallbackReturn.SUCCESS

    def euler_callback(self, msg: EulerAngles):
        try:
            if msg.yaw != msg.yaw:  # NaN guard
                return
            self.latest_yaw = msg.yaw
            self.imu_initialized = True
        except Exception as e:
            self._log.err(f"Error processing /euler callback: {e}")

    def cmd_vel_callback(self, msg: Twist):
        try:
            now = time.time()
            heading_dt = now - self._last_heading_pid_time
            self._last_heading_pid_time = now
            self.last_cmd_time = now

            linear_x = msg.linear.x * 0.8
            commanded_angular_z = msg.angular.z * self.rotation_gain  # Apply rotation gain for responsiveness
            angular_z = commanded_angular_z

            # --- Heading-assist trim ---
            # Only engages while Nav2 is itself commanding near-zero angular
            # velocity (i.e. it already believes it's driving straight). The
            # output is scaled by max_heading_correction and *added* to
            # Nav2's own command — it can never zero out or replace linear_x,
            # and it can never cancel a real turn Nav2 is asking for, unlike
            # ManualNavigationNode's heading-hold which fully substitutes the
            # yaw effort. If Nav2 starts turning again, the lock resets so the
            # trim doesn't fight the next real maneuver.
            if self.imu_initialized:
                if abs(commanded_angular_z) < self._straight_threshold:
                    if not self._heading_locked:
                        self.heading_pid.update_setpoint(self.latest_yaw)
                        self._heading_locked = True

                    error = self.heading_pid.setpoint - self.latest_yaw
                    while error > 180.0: error -= 360.0
                    while error < -180.0: error += 360.0
                    corrected_yaw = self.heading_pid.setpoint - error

                    correction_normalized = self.heading_pid.stabilize(measured_value=corrected_yaw, dt=max(heading_dt, 1e-3))
                    # stabilize() returns a normalized value in [-1, 1] (see
                    # PIDController's error/180 scaling) — it's a fraction of
                    # authority, not rad/s. Scale into physical units here
                    # rather than clamping the raw normalized value, or small
                    # errors get under-corrected and large errors all
                    # collapse to the same clipped output.
                    correction = correction_normalized * self.max_heading_correction

                    angular_z = commanded_angular_z + correction
                else:
                    self._heading_locked = False

            left_norm, right_norm = self.kinematics.calculate_wheel_speeds(linear_x, angular_z)
            cmd_dto = self.navigation_service.calculate_from_wheel_speeds(left_norm, right_norm)

            if cmd_dto is None:
                self._log.err("cmd_dto is None. Skipping publish.")
                return

            act_msg = ActuatorCommand()
            act_msg.m2_speed = 4.0 * float(cmd_dto.left_pwm)
            act_msg.m2_dir   = int(cmd_dto.left_dir)
            act_msg.m2_brake = int(cmd_dto.left_brake)
            act_msg.m1_speed = 4.0 * float(cmd_dto.right_pwm)
            act_msg.m1_dir   = int(cmd_dto.right_dir)
            act_msg.m1_brake = int(cmd_dto.right_brake)
            act_msg.flash    = int(1)
            act_msg.laser    = int(0)
            act_msg.servo    = float(0.0)

            self.motor_pub.publish(act_msg)

        except Exception as e:
            self._log.err(f"Exception in /cmd_vel callback: {e}")
            self.publish_safe_stop()

    def watchdog_callback(self):
        try:
            time_since_last_cmd = time.time() - self.last_cmd_time
            if time_since_last_cmd > self.watchdog_timeout:
                # Only log on the transition into "tripped" so this doesn't
                # spam the console every 0.1s while cmd_vel stays quiet — but
                # it tells you exactly how big the real gap was, which is the
                # number you actually need, not the nav2-configured target.
                if not getattr(self, '_watchdog_tripped', False):
                    self._log.warn(
                        f"Watchdog tripped: {time_since_last_cmd:.3f}s since last /cmd_vel "
                        f"(timeout={self.watchdog_timeout:.3f}s)"
                    )
                    self._watchdog_tripped = True
                self.publish_safe_stop()
            else:
                self._watchdog_tripped = False
        except Exception as e:
            self._log.err(f"Watchdog error: {e}")

    def publish_safe_stop(self):
        try:
            msg = ActuatorCommand()  # ← ADD THIS LINE
            msg.m1_speed = float(0.0)
            msg.m1_dir   = int(0)
            msg.m1_brake = int(1)
            msg.m2_speed = float(0.0)
            msg.m2_dir   = int(0)
            msg.m2_brake = int(1)
            msg.flash    = int(1)
            msg.laser    = int(0)
            msg.servo    = float(0.0)
            self.motor_pub.publish(msg)
        except Exception as e:
            self._log.err(f"Failed to publish safe stop: {e}")
def main(args=None):
    rclpy.init(args=args)
    try:
        node = AutonomousNavigationNode()
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
        if 'node' in locals():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()