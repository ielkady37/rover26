import pytest
import time
from unittest.mock import MagicMock, patch
from geometry_msgs.msg import Twist
from interfaces.msg import ActuatorCommand
from rclpy.lifecycle import TransitionCallbackReturn
import rclpy

from control.nodes.AutonomousNavigationNode import AutonomousNavigationNode

@pytest.fixture(scope="module")
def ros_init():
    """Initializes the ROS 2 context for the entire test module."""
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def mocked_dependencies():
    """Yields mocked dependencies that stay active for the duration of the test."""
    with patch('control.nodes.AutonomousNavigationNode.Configurator') as mock_conf:
         
         conf_instance = MagicMock()
         def mock_fetch(config_type):
             return {
                 'track_width_meters': 0.3,
                 'max_linear_vel_mps': 0.3,
                 'max_angular_vel_radps': 1.0,
                 'deadzone': 0.05, 
                 'max_pwm': 255, 
                 'watchdog_timeout_sec': 0.5,
                 'smoothing_alpha': 1.0, # Bypasses smoothing delay for tests
                 'smoothing_tolerance': 0.01
             }
             
         conf_instance.fetchData.side_effect = mock_fetch
         
         # THE MISSING LINK: Connect the class mock to the instance mock!
         mock_conf.return_value = conf_instance
         
         yield mock_conf

class TestAutonomousNavigationNode:

    def setup_method(self):
        self.node = AutonomousNavigationNode()
        
    def teardown_method(self):
        self.node.destroy_node()

    # ==========================================
    # 🔴 1. LIFECYCLE & FAILURE PATHS
    # ==========================================

    def test_on_configure_failure(self, ros_init, mocked_dependencies):
        """Verifies node handles Configurator exceptions gracefully."""
        mock_conf = mocked_dependencies
        mock_conf.return_value.fetchData.side_effect = Exception("Missing YAML!")
        
        assert self.node.on_configure(None) == TransitionCallbackReturn.FAILURE

    def test_on_shutdown_and_cleanup(self, ros_init, mocked_dependencies):
        """Verifies shutdown/cleanup destroys watchdog and stops motors."""
        self.node.on_configure(None)
        self.node.on_activate(None)
        
        self.node.motor_pub.publish = MagicMock()

        assert self.node.on_shutdown(None) == TransitionCallbackReturn.SUCCESS
        assert self.node.watchdog_timer.is_canceled() 
        self.node.motor_pub.publish.assert_called_once() # Safe stop

        with patch.object(self.node, 'destroy_timer', wraps=self.node.destroy_timer) as spy_destroy:
            assert self.node.on_cleanup(None) == TransitionCallbackReturn.SUCCESS
            spy_destroy.assert_called_once_with(self.node.watchdog_timer)

    # ==========================================
    # 🔴 2. TWIST COMMAND PROCESSING
    # ==========================================

    def test_cmd_vel_callback_happy_path(self, ros_init, mocked_dependencies):
        """Valid Twist command maps properly through kinematics and publishes ActuatorCommand."""
        self.node.on_configure(None)
        self.node.on_activate(None)
        self.node.motor_pub.publish = MagicMock()
        
        msg = Twist()
        # To hit 100% PWM, we must command the theoretical max_wheel_speed.
        # max_wheel_speed = max_linear (0.3) + (max_angular (1.0) * track_width (0.3) / 2.0) = 0.45
        msg.linear.x = 0.45 
        msg.angular.z = 0.0

        self.node.cmd_vel_callback(msg)
        
        self.node.motor_pub.publish.assert_called_once()
        pub_msg = self.node.motor_pub.publish.call_args[0][0]
        
        assert isinstance(pub_msg, ActuatorCommand)
        assert pub_msg.m1_dir == 1
        assert pub_msg.m2_dir == 1
        assert pub_msg.m1_speed == 255.0
        assert pub_msg.m2_speed == 255.0

    def test_cmd_vel_dto_none_early_exit(self, ros_init, mocked_dependencies):
        """If Facade returns None (math validation fails), skip publishing."""
        self.node.on_configure(None)
        self.node.motor_pub.publish = MagicMock()
        
        self.node.navigation_service.calculate_from_wheel_speeds = MagicMock(return_value=None)
        
        msg = Twist()
        self.node.cmd_vel_callback(msg)
        self.node.motor_pub.publish.assert_not_called()

    def test_cmd_vel_exception_triggers_safe_stop(self, ros_init, mocked_dependencies):
        """Corrupted data like NaN throws ValueError in Kinematics, catching and triggering brakes."""
        self.node.on_configure(None)
        self.node.publish_safe_stop = MagicMock()
        
        msg = Twist()
        msg.linear.x = float('nan') # This causes Kinematics solver to throw ValueError
        
        self.node.cmd_vel_callback(msg)
        self.node.publish_safe_stop.assert_called_once()

    # ==========================================
    # 🟡 3. WATCHDOG TIMER LOGIC
    # ==========================================

    def test_watchdog_healthy(self, ros_init, mocked_dependencies):
        """If time since last command is under timeout, do nothing."""
        self.node.on_configure(None)
        self.node.publish_safe_stop = MagicMock()
        
        # Simulate receiving a command exactly NOW
        self.node.last_cmd_time = time.time()
        
        self.node.watchdog_callback()
        self.node.publish_safe_stop.assert_not_called()

    def test_watchdog_timeout_triggers_safe_stop(self, ros_init, mocked_dependencies):
        """If time since last command exceeds timeout, apply hardware brakes."""
        self.node.on_configure(None)
        self.node.publish_safe_stop = MagicMock()
        
        # Simulate the last command arriving 5 seconds ago (timeout is 0.5s)
        self.node.last_cmd_time = time.time() - 5.0
        
        self.node.watchdog_callback()
        self.node.publish_safe_stop.assert_called_once()

    # ==========================================
    # 🟡 4. SAFE STOP DETAILS
    # ==========================================

    def test_publish_safe_stop_details(self, ros_init, mocked_dependencies):
        """Verifies safe stop zeroes speed, sets brakes, and clears Facade momentum."""
        self.node.on_configure(None)
        self.node.motor_pub.publish = MagicMock()
        self.node.navigation_service.reset_momentum = MagicMock()
        
        self.node.publish_safe_stop()
        
        msg = self.node.motor_pub.publish.call_args[0][0]
        assert msg.m1_brake == 1
        assert msg.m2_brake == 1
        assert msg.m1_speed == 0.0
        assert msg.m2_speed == 0.0
        
        self.node.navigation_service.reset_momentum.assert_called_once()