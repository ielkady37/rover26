import pytest
import time
from unittest.mock import MagicMock, patch
import rclpy
from std_msgs.msg import String

from control.nodes.MissionManagerNode import MissionManagerNode
from control.services.MissionManager import Phase, VisionState
from lifecycle_msgs.msg import Transition

@pytest.fixture(scope="module")
def ros_init():
    """Initializes the ROS 2 context for the entire test module."""
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def mocked_dependencies():
    """Yields mocked dependencies to isolate the node from the filesystem and network."""
    with patch('control.nodes.MissionManagerNode.Configurator') as mock_conf, \
         patch('control.nodes.MissionManagerNode.ActionClient') as mock_action_client, \
         patch.object(MissionManagerNode, 'create_client') as mock_create_client:
         
         conf_instance = MagicMock()
         conf_instance.fetchData.return_value = {
             'vision_timeout_sec': 0.5,
             'min_confidence_score': 0.8,
             'waypoints': [
                 {'id': 'WP1', 'x': 5.0, 'y': 2.0, 'yaw': 0.0},
                 {'id': 'WP2', 'x': 10.0, 'y': 10.0, 'yaw': 1.57},
                 {'id': 'WP3', 'x': 15.0, 'y': -5.0, 'yaw': 3.14}
             ]
         }
         mock_conf.return_value = conf_instance
         
         mock_manual_client = MagicMock()
         mock_manual_client.srv_name = '/manual_navigation_node/change_state'
         mock_auto_client = MagicMock()
         mock_auto_client.srv_name = '/autonomous_navigation_node/change_state'
         
         mock_create_client.side_effect = [mock_manual_client, mock_auto_client]

         yield mock_conf, mock_manual_client, mock_auto_client

class TestMissionManagerNode:

    def setup_method(self):
        with patch.object(MissionManagerNode, '_execute_boot_sequence'):
            self.node = MissionManagerNode()
            
        # Force test variables to override real mission.yaml
        self.node.manager.vision_timeout_sec = 0.5
        self.node.manager.min_confidence = 0.8
        self.node.manager.waypoints = [
             {'id': 'WP1', 'x': 5.0, 'y': 2.0, 'yaw': 0.0},
             {'id': 'WP2', 'x': 10.0, 'y': 10.0, 'yaw': 1.57},
             {'id': 'WP3', 'x': 15.0, 'y': -5.0, 'yaw': 3.14}
        ]
            
        self.node.manual_client = MagicMock()
        self.node.auto_client = MagicMock()
        self.node.nav_client = MagicMock()
        
    def teardown_method(self):
        with patch.object(self.node, '_execute_teardown_sequence'):
            self.node.destroy_node()

    # ==========================================
    # 🔴 TIER 1: CRITICAL PATHS & SAFETY
    # ==========================================

    def test_vision_task_low_confidence_rejection(self, ros_init, mocked_dependencies):
        """[Gap 5] Verifies CV targets below threshold are rejected."""
        self.node.manager.start_vision_task(current_time=time.time())
        
        mock_msg = MagicMock()
        mock_msg.is_found = True
        mock_msg.confidence_score = 0.50 # Below 0.8 threshold
        
        self.node.target_status_callback(mock_msg)
        
        # State should NOT resolve
        assert self.node.manager.vision_state == VisionState.SEARCHING

    def test_end_to_end_mission_flow(self, ros_init, mocked_dependencies):
        """[Gap 8, 10, 20] Verifies WP1 -> WP2(Vision) -> WP3 -> Complete."""
        self.node._dispatch_next_waypoint = MagicMock(wraps=self.node._dispatch_next_waypoint)
        self.node.nav_client.send_goal_async = MagicMock()
        
        # 1. Dispatch WP1
        self.node._dispatch_next_waypoint()
        assert self.node.manager.peek_current_waypoint_id() == "WP2"

        # 2. Arrive at WP1 -> Should auto-dispatch WP2
        self.node._goal_result_callback(MagicMock(), "WP1")
        assert self.node._dispatch_next_waypoint.call_count == 2
        assert self.node.manager.peek_current_waypoint_id() == "WP3"

        # 3. Arrive at WP2 -> Should trigger vision, NOT dispatch WP3
        self.node._goal_result_callback(MagicMock(), "WP2")
        assert self.node.manager.vision_state == VisionState.SEARCHING
        assert self.node._dispatch_next_waypoint.call_count == 2

        # 4. Vision Success -> Should dispatch WP3
        mock_msg = MagicMock(is_found=True, confidence_score=0.99)
        self.node.target_status_callback(mock_msg)
        assert self.node.manager.vision_state == VisionState.RESOLVED
        assert self.node._dispatch_next_waypoint.call_count == 3

        # 5. Arrive at WP3 -> Should trigger completion
        self.node._goal_result_callback(MagicMock(), "WP3")
        assert self.node._dispatch_next_waypoint.call_count == 4
        assert self.node.manager.current_phase == Phase.COMPLETE

    def test_goal_response_rejection(self, ros_init, mocked_dependencies):
        """[Gap 7] Verifies node handles Nav2 actively rejecting a goal."""
        mock_future = MagicMock()
        mock_future.result.return_value.accepted = False
        
        self.node._goal_response_callback(mock_future, "WP1")
        
        # Ensure it does not attempt to track progress of a rejected goal
        mock_future.result.return_value.get_result_async.assert_not_called()

    def test_lifecycle_result_callback_failure(self, ros_init, mocked_dependencies):
        """[Gap 2] Verifies node gracefully handles a lifecycle transition failure."""
        mock_future = MagicMock()
        mock_future.result.return_value.success = False
        
        # Simply verifying it doesn't crash the executor
        self.node._lifecycle_result_cb(mock_future, "ManualNav", Transition.TRANSITION_ACTIVATE)

    # ==========================================
    # 🟡 TIER 2: TRANSITIONS & EXCEPTIONS
    # ==========================================

    def test_transition_to_manual(self, ros_init, mocked_dependencies):
        """[Gap 3] Verifies GUI 'manual' command drops Auto and raises Manual."""
        self.node.manager.current_phase = Phase.AUTONOMOUS # Pre-condition
        self.node._change_lifecycle_state = MagicMock()

        self.node.mission_state_callback(String(data=Phase.MANUAL.value))

        assert self.node.manager.current_phase == Phase.MANUAL
        self.node._change_lifecycle_state.assert_any_call(
            self.node.auto_client, Transition.TRANSITION_DEACTIVATE, "AutoNav"
        )
        self.node._change_lifecycle_state.assert_any_call(
            self.node.manual_client, Transition.TRANSITION_ACTIVATE, "ManualNav"
        )

    def test_nav2_server_unavailable(self, ros_init, mocked_dependencies):
        """[Gap 13] Verifies Auto transition aborts if Nav2 is offline."""
        self.node.nav_client.wait_for_server.return_value = False
        self.node._dispatch_next_waypoint = MagicMock()

        self.node._execute_transition_to_auto()

        # Should log an error and abort before dispatching
        self.node._dispatch_next_waypoint.assert_not_called()

    def test_callback_exception_handling(self, ros_init, mocked_dependencies):
        """[Gap 4, 6, 9] Verifies unexpected None/Malformed types don't crash callbacks."""
        try:
            self.node.mission_state_callback(None) # Missing .data
            self.node.target_status_callback(None) # Missing .is_found
            self.node.tick_callback() # (Should survive)
            self.node._goal_response_callback(None, "WP1") 
            self.node._goal_result_callback(None, "WP1")
            self.node._lifecycle_result_cb(None, "Test", 1)
        except Exception:
            pytest.fail("A callback allowed an exception to crash the ROS node!")

    def test_configurator_failure_on_init(self, ros_init):
        """[Gap 21] Verifies node enforces SystemExit if YAML is missing at boot."""
        with patch('control.nodes.MissionManagerNode.Configurator') as mock_conf:
            mock_conf.side_effect = Exception("File missing!")
            
            with pytest.raises(SystemExit):
                MissionManagerNode()
                
    # ==========================================
    # PREVIOUSLY ESTABLISHED TESTS
    # ==========================================
    
    def test_boot_sequence_orchestration(self, ros_init, mocked_dependencies):
        self.node._change_lifecycle_state = MagicMock()
        self.node._execute_boot_sequence()
        self.node._change_lifecycle_state.assert_any_call(self.node.manual_client, Transition.TRANSITION_ACTIVATE, "ManualNav")

    def test_dispatch_waypoint_formatting(self, ros_init, mocked_dependencies):
        """[Gap 11 included] Verifies Waypoint fields map correctly to PoseStamped."""
        self.node.nav_client.send_goal_async = MagicMock()
        self.node._dispatch_next_waypoint()
        goal_msg = self.node.nav_client.send_goal_async.call_args[0][0]
        assert goal_msg.pose.pose.position.x == 5.0
        assert goal_msg.pose.pose.position.y == 2.0
        assert goal_msg.pose.pose.orientation.z == 0.0

    def test_vision_task_timeout_watchdog(self, ros_init, mocked_dependencies):
        self.node._dispatch_next_waypoint = MagicMock()
        self.node.manager.start_vision_task(current_time=time.time())
        self.node.tick_callback()
        self.node._dispatch_next_waypoint.assert_not_called()
        self.node.manager.vision_start_time -= 1.0
        self.node.tick_callback()
        assert self.node.manager.vision_state == VisionState.RESOLVED
        self.node._dispatch_next_waypoint.assert_called_once()