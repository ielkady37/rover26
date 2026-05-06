#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from msgs.msg import Joystick
from control.services.Joystick import CJoystick
import signal
from multiprocessing.shared_memory import SharedMemory

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')
        self.logger = self.get_logger()
        self.joystick = CJoystick(is_writer=True)
        signal.signal(signal.SIGINT, self.handle_exit)
        signal.signal(signal.SIGTERM, self.handle_exit)
        self.subscriber = self.create_subscription(Joystick, '/joystick', self.callback, 10)

    def handle_exit(self, signum, frame):
        self.logger.info(f"Received termination signal {signum}. Cleaning up shared memory.")
        self.joystick.cleanup()
        try:
            shm = SharedMemory(name="joystick_data")
            shm.unlink()
        except FileNotFoundError:
            pass
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        
    def callback(self, data):
        # Extract button data (keys containing "button")
        button_data = {key: getattr(data, key) for key in dir(data) if "button" in key}
        # print(button_data)
        # Extract axis data (keys containing "axis")
        axis_data = {key: getattr(data, key) for key in dir(data) if "axis" in key}
        # print(axis_data)
        # Update shared memory with extracted data
        self.joystick.updateData(button_data, axis_data)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    try:
        rclpy.spin(node)
    finally:
        node.joystick.cleanup()  # Explicit cleanup if needed
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()