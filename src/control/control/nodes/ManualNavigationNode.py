#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

# Import our custom messages and services
from interfaces.msg import MotorCommands
from control.services.Navigation import Navigation
from control.services.Joystick import CJoystick

import time
from utils.services.Configurator import Configurator

class ManualNavigationNode(Node):
    """
    Node responsible for polling joystick shared memory, calculating kinematics,
    and publishing motor commands. Operates defensively to prevent crashes.
    """

    def __init__(self):
        super().__init__('manual_navigation_node')
        self.logger = self.get_logger()

        # 1. Load Parameters (ROS 2 Parameters for node-specific tuning)
        self.declare_parameter('control_loop_rate_hz', 20.0)
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('deadzone', 0.0)
        
        loop_rate = self.get_parameter('control_loop_rate_hz').value
        max_pwm = self.get_parameter('max_pwm').value
        deadzone = self.get_parameter('deadzone').value

        # Load PID configurations
        try:
            conf = Configurator()
            pid_data = conf.fetchData(Configurator.PID_PARAMS)
            kp = float(pid_data.get('yaw_KP', 0.0))
            ki = float(pid_data.get('yaw_KI', 0.0))
            kd = float(pid_data.get('yaw_KD', 0.0))
        except Exception as e:
            self.logger.warn(f"Failed to load PID configs, defaulting to 0: {e}")
            kp, ki, kd = 0.0, 0.0, 0.0

        # 2. Initialize Services
        self.logger.info(f"Initializing Navigation Facade (PID: {kp}, {ki}, {kd})...")
        self.navigation_service = Navigation(deadzone=deadzone, max_pwm=max_pwm, kp=kp, ki=ki, kd=kd)
        
        try:
            # Initialize as reader (is_writer=False)
            self.joystick = CJoystick(is_writer=False) 
        except Exception as e:
            self.logger.fatal(f"Failed to connect to CJoystick shared memory: {e}")
            raise SystemExit(1)

        # 3. Setup Publishers
        # QoS: Keep Last 1, Reliable for motor commands to ensure ESCs get latest state immediately
        # motor_qos = QoSProfile(
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=1,
        #     reliability=ReliabilityPolicy.RELIABLE
        # )
        # self.motor_pub = self.create_publisher(MotorCommands, '/motor_commands', motor_qos)
        self.motor_pub = self.create_publisher(MotorCommands, '/motor_commands')

        # 4. Setup Subscribers
        # QoS: Sensor data profile (Best Effort, small queue) to prevent lag build-up
        # imu_qos = QoSProfile(
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=5,
        #     reliability=ReliabilityPolicy.BEST_EFFORT
        # )
        # self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, imu_qos)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback)

        self.latest_yaw = 0.0 
        self.last_time = time.time() # Track dt for accurate PID math

        # 5. Setup Control Loop Timer
        # timer_period = 1.0 / loop_rate
        # self.control_timer = self.create_timer(timer_period, self.control_loop_callback)
        
        # self.logger.info(f"ManualNavigationNode started. Loop rate: {loop_rate}Hz")

    def imu_callback(self, msg: Imu):
        """
        Defensively handles incoming IMU data for future Yaw PID integration.
        """
        try:
            # Extract Z-axis angular velocity or orientation
            if msg.angular_velocity.z != msg.angular_velocity.z: # NaN check 
                self.logger.warn("Received NaN from /imu. Ignoring.")
                return
            
            self.latest_yaw = msg.angular_velocity.z
        except Exception as e:
            self.logger.error(f"Error processing /imu callback: {e}")

    def control_loop_callback(self):
        """
        Main polling loop: Reads joystick, calculates kinematics, publishes commands.
        Wrapped in try/except to guarantee zero-crash execution.
        """
        try:
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time

            axes = self.joystick.getAxis()
            throttle = axes.get("left_y_axis", 0.0)
            yaw = axes.get("right_x_axis", 0.0)

            cmd_dto = self.navigation_service.calculate_motor_commands(
                throttle_axis=throttle, 
                yaw_axis=yaw,
                current_yaw=self.latest_yaw,
                dt=dt
            )

            if cmd_dto is None:
                return

            # 3. Publish to ROS network
            msg = MotorCommands()
            msg.left_pwm = cmd_dto.left_pwm
            msg.left_dir = cmd_dto.left_dir
            msg.left_brake = cmd_dto.left_brake
            msg.right_pwm = cmd_dto.right_pwm
            msg.right_dir = cmd_dto.right_dir
            msg.right_brake = cmd_dto.right_brake

            self.motor_pub.publish(msg)

        except Exception as e:
            self.logger.error(f"Exception in control loop: {e}")
            self.publish_safe_stop()

    def publish_safe_stop(self):
        """Emergency stop command if node logic faults."""
        try:
            msg = MotorCommands()
            msg.left_brake = 1
            msg.right_brake = 1
            self.motor_pub.publish(msg)
            self.logger.warn("Safe stop commands published.")
        except Exception as e:
            self.logger.error(f"Failed to publish safe stop: {e}")

    def destroy_node(self):
        """Graceful shutdown hook to stop motors before dying."""
        self.logger.info("Initiating graceful shutdown...")
        # self.control_timer.cancel()
        self.publish_safe_stop()
        super().destroy_node()

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