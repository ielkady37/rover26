import pytest
import time
from unittest.mock import MagicMock, patch
from interfaces.msg import EulerAngles
from rclpy.lifecycle import TransitionCallbackReturn
import rclpy

from control.nodes.ManualNavigationNode import ManualNavigationNode

@pytest.fixture(scope="module")
def ros_init():
    """Initializes the ROS 2 context for the entire test module."""
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def mocked_dependencies():
    """Yields mocked dependencies that stay active for the duration of the test."""
    with patch('control.nodes.ManualNavigationNode.Configurator') as mock_conf, \
         patch('control.nodes.ManualNavigationNode.CJoystick') as mock_joy:
         
         # Setup Configurator with a non-zero deadzone to test boundary logic
         conf_instance = MagicMock()
         conf_instance.fetchData.return_value = {'yaw_KP': 1.0, 'yaw_KI': 0.1, 'yaw_KD': 0.05}
         mock_conf.return_value = conf_instance
         
         # Setup Joystick
         joy_instance = MagicMock()
         joy_instance.getAxis.return_value = {"left_y_axis": 0.0, "right_x_axis": 0.0}
         mock_joy.return_value = joy_instance
         
         yield mock_conf, mock_joy

class TestManualNavigationNode:

    def setup_method(self):
        self.node = ManualNavigationNode()
        # Override deadzone parameter for testing before configuration
        self.node.set_parameters([rclpy.parameter.Parameter('deadzone', rclpy.Parameter.Type.DOUBLE, 0.05)])
        
    def teardown_method(self):
        self.node.destroy_node()

    # ==========================================
    # 🔴 1. LIFECYCLE & FAILURE PATHS
    # ==========================================

    def test_on_configure_failure(self, ros_init, mocked_dependencies):
        """[Gap 1] Verifies node handles Configurator exceptions gracefully."""
        mock_conf, _ = mocked_dependencies
        mock_conf.return_value.fetchData.side_effect = Exception("Missing YAML!")
        
        assert self.node.on_configure(None) == TransitionCallbackReturn.FAILURE

    def test_on_shutdown_and_cleanup(self, ros_init, mocked_dependencies):
        """[Gap 2, 12] Verifies shutdown/cleanup destroys resources and stops motors."""
        self.node.on_configure(None)
        self.node.on_activate(None)
        
        # We can safely mock publish because it doesn't hold memory pointers
        self.node.motor_pub.publish = MagicMock()

        assert self.node.on_shutdown(None) == TransitionCallbackReturn.SUCCESS
        
        # Verify natively that the real timer was canceled
        assert self.node.control_timer.is_canceled() 
        self.node.motor_pub.publish.assert_called_once() # Safe stop

        # Use a "Spy" (wraps) to check if destroy_timer is called WITHOUT breaking the C++ cleanup
        with patch.object(self.node, 'destroy_timer', wraps=self.node.destroy_timer) as spy_destroy:
            assert self.node.on_cleanup(None) == TransitionCallbackReturn.SUCCESS
            spy_destroy.assert_called_once_with(self.node.control_timer)
    # ==========================================
    # 🔴 2. CONTROL LOOP BRANCHING LOGIC
    # ==========================================

    def test_control_loop_idle(self, ros_init, mocked_dependencies):
        """[Gap 3] Throttle=0, Yaw=0 -> Should update PID setpoint to latest yaw."""
        mock_conf, mock_joy = mocked_dependencies
        self.node.on_configure(None)
        self.node.latest_yaw = 90.0
        
        self.node.pid.update_setpoint = MagicMock()
        mock_joy.return_value.getAxis.return_value = {"left_y_axis": 0.0, "right_x_axis": 0.0}
        
        self.node.control_loop_callback()
        self.node.pid.update_setpoint.assert_called_with(90.0)

    def test_control_loop_manual_yaw_bypasses_pid(self, ros_init, mocked_dependencies):
        """[Gap 3] Yaw > deadzone -> Updates setpoint, uses raw yaw, bypasses stabilize."""
        mock_conf, mock_joy = mocked_dependencies
        self.node.on_configure(None)
        self.node.latest_yaw = 45.0
        
        self.node.pid.update_setpoint = MagicMock()
        self.node.pid.stabilize = MagicMock()
        
        # Yaw of 0.5 is > deadzone of 0.05
        mock_joy.return_value.getAxis.return_value = {"left_y_axis": 0.0, "right_x_axis": 0.5}
        
        self.node.control_loop_callback()
        self.node.pid.update_setpoint.assert_called_with(45.0)
        self.node.pid.stabilize.assert_not_called()

    def test_control_loop_throttle_uses_pid(self, ros_init, mocked_dependencies):
        """[Gap 3, 10, 11] Throttle > deadzone, Yaw=0 -> Should call stabilize with dt."""
        mock_conf, mock_joy = mocked_dependencies
        self.node.on_configure(None)
        self.node.latest_yaw = 10.0
        
        self.node.pid.stabilize = MagicMock(return_value=0.2) # Simulate PID correcting a drift
        mock_joy.return_value.getAxis.return_value = {"left_y_axis": 1.0, "right_x_axis": 0.0}
        
        # Manually manipulate time to ensure dt > 0
        self.node.last_time = time.time() - 0.1 
        
        self.node.control_loop_callback()
        
        # Verify stabilize was called
        assert self.node.pid.stabilize.called
        args, kwargs = self.node.pid.stabilize.call_args
        assert kwargs['measured_value'] == 10.0
        assert kwargs['dt'] > 0.0

    def test_motor_mapping_asymmetric_turn(self, ros_init, mocked_dependencies):
        """[Gap 3, 16] Pure right turn -> Left wheel (M2) forwards, Right wheel (M1) backwards."""
        mock_conf, mock_joy = mocked_dependencies
        self.node.on_configure(None)
        self.node.motor_pub.publish = MagicMock()
        
        # Right turn (positive yaw)
        mock_joy.return_value.getAxis.return_value = {"left_y_axis": 0.0, "right_x_axis": 1.0}
        self.node.control_loop_callback()
        
        msg = self.node.motor_pub.publish.call_args[0][0]
        
        # Left wheel (M2) should drive forward to push the chassis right
        assert msg.m2_dir == 1
        assert msg.m2_speed > 0.0
        # Right wheel (M1) should drive backward to pull the chassis right
        assert msg.m1_dir == 0
        assert msg.m1_speed > 0.0

    # ==========================================
    # 🔴 3. EXCEPTIONS & DEFENSIVE EXITS
    # ==========================================

    def test_control_loop_dto_none_early_exit(self, ros_init, mocked_dependencies):
        """[Gap 4] If Facade returns None (math fault), do not publish."""
        self.node.on_configure(None)
        self.node.motor_pub.publish = MagicMock()
        
        # Force facade to fail
        self.node.navigation_service.calculate_motor_commands = MagicMock(return_value=None)
        
        self.node.control_loop_callback()
        self.node.motor_pub.publish.assert_not_called()

    def test_control_loop_exception_triggers_safe_stop(self, ros_init, mocked_dependencies):
        """[Gap 5] Any unexpected exception in the main loop triggers hardware brakes."""
        mock_conf, mock_joy = mocked_dependencies
        self.node.on_configure(None)
        self.node.publish_safe_stop = MagicMock()
        
        # Force an exception during axis retrieval
        mock_joy.return_value.getAxis.side_effect = Exception("Shared Memory Corrupted!")
        
        self.node.control_loop_callback()
        self.node.publish_safe_stop.assert_called_once()

    def test_publish_safe_stop_details(self, ros_init, mocked_dependencies):
        """[Gap 9] Verifies safe stop explicitly zeroes speed and sets brake flags."""
        self.node.on_configure(None)
        self.node.motor_pub.publish = MagicMock()
        
        self.node.publish_safe_stop()
        
        msg = self.node.motor_pub.publish.call_args[0][0]
        assert msg.m1_brake == 1
        assert msg.m2_brake == 1
        assert msg.m1_speed == 0.0
        assert msg.m2_speed == 0.0

    # ==========================================
    # 🟡 4. SENSOR ANOMALIES
    # ==========================================

    def test_euler_valid_update(self, ros_init):
        """[Gap 6] Verifies valid IMU data updates the internal state."""
        self.node.latest_yaw = 0.0
        msg = EulerAngles()
        msg.yaw = -15.5
        
        self.node.euler_callback(msg)
        assert self.node.latest_yaw == -15.5

    def test_joystick_missing_keys(self, ros_init, mocked_dependencies):
        """[Gap 19] Verifies .get() defaults safely if CJoystick dictionary is incomplete."""
        mock_conf, mock_joy = mocked_dependencies
        self.node.on_configure(None)
        self.node.motor_pub.publish = MagicMock()
        
        # Return empty dict instead of axis keys
        mock_joy.return_value.getAxis.return_value = {}
        
        self.node.control_loop_callback()
        
        # Should default to 0.0, resulting in idle logic and 0.0 speeds
        msg = self.node.motor_pub.publish.call_args[0][0]
        assert msg.m1_speed == 0.0
        assert msg.m2_speed == 0.0