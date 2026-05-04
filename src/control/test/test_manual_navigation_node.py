import pytest
from unittest.mock import MagicMock, patch
from interfaces.msg import ActuatorCommand
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
         
         # Setup Configurator
         conf_instance = MagicMock()
         conf_instance.fetchData.return_value = {'yaw_KP': 1.0, 'yaw_KI': 0.0, 'yaw_KD': 0.0}
         mock_conf.return_value = conf_instance
         
         # Setup Joystick
         joy_instance = MagicMock()
         joy_instance.getAxis.return_value = {"left_y_axis": 0.0, "right_x_axis": 0.0}
         mock_joy.return_value = joy_instance
         
         yield mock_conf, mock_joy

class TestManualNavigationNode:

    def setup_method(self):
        self.node = ManualNavigationNode()
        
    def teardown_method(self):
        self.node.destroy_node()

    def test_lifecycle_transitions(self, ros_init, mocked_dependencies):
        assert self.node.on_configure(None) == TransitionCallbackReturn.SUCCESS
        assert self.node.pid.kp == 1.0 
        
        assert self.node.on_activate(None) == TransitionCallbackReturn.SUCCESS
        assert not self.node.control_timer.is_canceled()
        
        assert self.node.on_deactivate(None) == TransitionCallbackReturn.SUCCESS
        assert self.node.control_timer.is_canceled() 
        
        assert self.node.on_cleanup(None) == TransitionCallbackReturn.SUCCESS

    def test_control_loop_publishing(self, ros_init, mocked_dependencies):
        mock_conf, mock_joy = mocked_dependencies
        self.node.on_configure(None)
        self.node.on_activate(None)

        # Mock the publisher AFTER on_configure creates it
        self.node.motor_pub.publish = MagicMock()

        # Inject forward throttle
        mock_joy.return_value.getAxis.return_value = {"left_y_axis": 1.0, "right_x_axis": 0.0}

        self.node.control_loop_callback()

        self.node.motor_pub.publish.assert_called_once()
        published_msg = self.node.motor_pub.publish.call_args[0][0]
        
        assert isinstance(published_msg, ActuatorCommand)
        assert published_msg.m1_dir == 1 
        assert published_msg.m2_dir == 1 
        assert published_msg.m1_speed == 255.0  # Assert float
        assert published_msg.m2_speed == 255.0  # Assert float

    def test_defensive_imu_nan_handling(self, ros_init, mocked_dependencies):
        self.node.on_configure(None)
        self.node.latest_yaw = 45.0 

        bad_msg = EulerAngles()
        bad_msg.yaw = float('nan') 

        self.node.euler_callback(bad_msg)
        assert self.node.latest_yaw == 45.0

    def test_safe_stop_on_deactivate(self, ros_init, mocked_dependencies):
        self.node.on_configure(None)
        self.node.on_activate(None)
        
        # Mock the publisher AFTER on_configure creates it
        self.node.motor_pub.publish = MagicMock()

        self.node.on_deactivate(None)

        self.node.motor_pub.publish.assert_called_once()
        published_msg = self.node.motor_pub.publish.call_args[0][0]
        
        assert published_msg.m1_brake == 1
        assert published_msg.m2_brake == 1