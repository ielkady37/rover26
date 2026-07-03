#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future

from std_msgs.msg import String
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from nav2_msgs.action import NavigateToPose

try:
    from interfaces.msg import TargetStatus
except ImportError:
    TargetStatus = None 

import time
from control.services.MissionManager import MissionManager, Phase, VisionState
from utils.Configurator import Configurator
from utils.Logger import RoverLogger

def _yaw_to_quat(yaw: float):
    half = yaw * 0.5
    return math.sin(half), math.cos(half)  # (z, w)

class MissionManagerNode(Node):
    """
    The Supreme Orchestrator.
    Handles Lifecycle Management (Configure, Activate, Deactivate, Cleanup), 
    Nav2 Action goals, and Vision timeouts.
    """

    def __init__(self):
        super().__init__('mission_manager_node')
        self._log = RoverLogger()

        # 1. Mission Configurations
        try:
            conf = Configurator()
            mission_data = conf.fetchData(Configurator.MISSION) or {}
            
            waypoints = mission_data.get('waypoints', [])
            vision_timeout = float(mission_data.get('vision_timeout_sec', 120.0))
            min_confidence = float(mission_data.get('min_confidence_score', 0.80))
            
            self.manager = MissionManager(
                waypoints=waypoints,
                vision_timeout_sec=vision_timeout,
                min_confidence=min_confidence
            )
        except Exception as e:
            self._log.err(f"Fatal: Failed to load mission.yaml. {e}")
            raise SystemExit(1)

        # 2. Setup Lifecycle Service Clients
        self.manual_client = self.create_client(ChangeState, '/manual_navigation_node/change_state')
        self.manual_camera_client = self.create_client(ChangeState, '/manual_camera_streaming_node/change_state')
        self.auto_camera_client = self.create_client(ChangeState, '/autonomous_camera_streaming_node/change_state')
        self.auto_client = self.create_client(ChangeState, '/autonomous_navigation_node/change_state')

        # 3. Setup Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 4. Setup Subscribers
        self.state_sub = self.create_subscription(String, '/mission_state', self.mission_state_callback, 10)
        if TargetStatus:
            self.target_sub = self.create_subscription(TargetStatus, '/target_status', self.target_status_callback, 10)

        # 5. Setup Watchdog/Tick Timer (runs at 10Hz)
        self.tick_timer = self.create_timer(0.1, self.tick_callback)
        
        # 6. Execute Boot Sequence
        self._log.info("Waiting for Navigation nodes to come online...")
        self._execute_boot_sequence()

    # LIFECYCLE ORCHESTRATION
    def _execute_boot_sequence(self):
        """
        Brings nodes from Unconfigured -> Inactive -> Active.
        """
        # Ensure services are available before sending requests
        self.manual_client.wait_for_service(timeout_sec=5.0)
        self.auto_client.wait_for_service(timeout_sec=5.0)
        self.manual_camera_client.wait_for_service(timeout_sec=5.0)
        self.auto_camera_client.wait_for_service(timeout_sec=5.0)
        self._log.info("Configuring Navigation Nodes (Loading YAMLs, Shared Memory, etc.)...")
        
        # 1. Send CONFIGURE to both (Moves them to 'Inactive')
        self._change_lifecycle_state(self.manual_client, Transition.TRANSITION_CONFIGURE, "ManualNav")
        self._change_lifecycle_state(self.auto_client, Transition.TRANSITION_CONFIGURE, "AutoNav")
        self._change_lifecycle_state(self.manual_camera_client, Transition.TRANSITION_CONFIGURE, "ManualCamera")
        self._change_lifecycle_state(self.auto_camera_client, Transition.TRANSITION_CONFIGURE, "AutoCamera")

        # 2. Automatically Activate Phase 1 (Manual)
        self._log.info("Boot sequence complete. Activating default Phase 1 (Manual).")
        self._change_lifecycle_state(self.manual_client, Transition.TRANSITION_ACTIVATE, "ManualNav")
        self._change_lifecycle_state(self.manual_camera_client, Transition.TRANSITION_ACTIVATE, "ManualCamera")

    def _execute_transition_to_manual(self):
        """Safely pauses Auto and spins up Manual."""
        self._log.info("Transitioning stack to MANUAL control...")
        
        # 1. Deactivate Autonomous Node (Active -> Inactive)
        self._change_lifecycle_state(self.auto_client, Transition.TRANSITION_DEACTIVATE, "AutoNav")
        self._change_lifecycle_state(self.auto_camera_client, Transition.TRANSITION_DEACTIVATE, "AutoCamera")
        
        # 2. Activate Manual Node (Inactive -> Active)
        self._change_lifecycle_state(self.manual_client, Transition.TRANSITION_ACTIVATE, "ManualNav")
        self._change_lifecycle_state(self.manual_camera_client, Transition.TRANSITION_ACTIVATE, "ManualCamera")

    def _execute_transition_to_auto(self):
        """Safely pauses Manual, spins up Auto, and triggers Waypoint 1."""
        self._log.info("Transitioning stack to AUTONOMOUS control...")
        
        # 1. Deactivate Manual Node (Active -> Inactive)
        self._change_lifecycle_state(self.manual_client, Transition.TRANSITION_DEACTIVATE, "ManualNav")
        self._change_lifecycle_state(self.manual_camera_client, Transition.TRANSITION_DEACTIVATE, "ManualCamera")
        
        # 2. Activate Autonomous Node (Inactive -> Active)
        self._change_lifecycle_state(self.auto_client, Transition.TRANSITION_ACTIVATE, "AutoNav")
        self._change_lifecycle_state(self.auto_camera_client, Transition.TRANSITION_ACTIVATE, "AutoCamera")

        # 3. Ensure Nav2 is ready, then dispatch the first waypoint
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self._log.err("Nav2 Action Server not available! Cannot proceed with mission.")
            return
            
        self._dispatch_next_waypoint()

    def _execute_teardown_sequence(self):
        """
        Called on shutdown to completely free resources.
        Moves nodes from Active/Inactive -> Unconfigured.
        """
        self._log.info("Unconfiguring nodes to free shared memory and hardware locks...")
        
        # First ensure they are deactivated
        self._change_lifecycle_state(self.manual_client, Transition.TRANSITION_DEACTIVATE, "ManualNav")
        self._change_lifecycle_state(self.manual_camera_client, Transition.TRANSITION_DEACTIVATE, "ManualCamera")
        self._change_lifecycle_state(self.auto_client, Transition.TRANSITION_DEACTIVATE, "AutoNav")
        self._change_lifecycle_state(self.auto_camera_client, Transition.TRANSITION_DEACTIVATE, "AutoCamera")
        
        # Then Unconfigure (Cleanup)
        self._change_lifecycle_state(self.manual_client, Transition.TRANSITION_CLEANUP, "ManualNav")
        self._change_lifecycle_state(self.manual_camera_client, Transition.TRANSITION_CLEANUP, "ManualCamera")
        self._change_lifecycle_state(self.auto_client, Transition.TRANSITION_CLEANUP, "AutoNav")
        self._change_lifecycle_state(self.auto_camera_client, Transition.TRANSITION_CLEANUP, "AutoCamera")

    def _change_lifecycle_state(self, client: rclpy.client.Client, transition_id: int, node_label: str):
        """Helper to send standard ROS 2 lifecycle transition requests."""
        if not client.wait_for_service(timeout_sec=2.0):
            self._log.warn(f"Lifecycle service {client.srv_name} not available.")
            return

        req = ChangeState.Request()
        req.transition.id = transition_id
        
        # Send async so we don't block the orchestrator
        future = client.call_async(req)
        future.add_done_callback(lambda f: self._lifecycle_result_cb(f, node_label, transition_id))

    def _lifecycle_result_cb(self, future: Future, node_label: str, transition_id: int):
        """Handles the response from the lifecycle nodes."""
        try:
            response = future.result()
            transition_map = {
                Transition.TRANSITION_CONFIGURE: "Configured (Inactive)",
                Transition.TRANSITION_ACTIVATE: "Activated",
                Transition.TRANSITION_DEACTIVATE: "Deactivated (Inactive)",
                Transition.TRANSITION_CLEANUP: "Unconfigured (Cleaned Up)"
            }
            state_str = transition_map.get(transition_id, "Unknown State")

            if response.success:
                self._log.succ(f"{node_label} successfully transitioned to: {state_str}.")
            else:
                self._log.err(f"{node_label} FAILED to transition to: {state_str}.")
        except Exception as e:
            self._log.err(f"Lifecycle service call failed for {node_label}: {e}")

    # CALLBACKS
    def mission_state_callback(self, msg: String):
        """Triggered by the GUI to change the operational phase."""
        try:
            requested_phase = msg.data.strip().lower()
            changed = self.manager.set_phase(requested_phase)
            
            if not changed:
                return # Ignore duplicate state commands

            if requested_phase == Phase.MANUAL.value:
                self._execute_transition_to_manual()
            elif requested_phase == Phase.AUTONOMOUS.value:
                self._execute_transition_to_auto()

        except Exception as e:
            self._log.err(f"Failed to process mission state request: {e}")

    def target_status_callback(self, msg):
        """Receives data from the CV node if we are actively searching at WP2."""
        try:
            resolved, reason = self.manager.evaluate_target_status(
                is_found=msg.is_found, 
                confidence=msg.confidence_score, 
                current_time=time.time()
            )

            if resolved:
                self._log.info(f"Vision Task Concluded: {reason}")
                self._dispatch_next_waypoint() # Proceed to WP3
        except Exception as e:
            self._log.err(f"Error evaluating target status: {e}")

    def tick_callback(self):
        """Background loop to check for timeouts when blocking on vision tasks."""
        try:
            if self.manager.vision_state == VisionState.SEARCHING:
                resolved, reason = self.manager.evaluate_target_status(False, 0.0, time.time())
                if resolved:
                    self._log.warn(f"Vision Task Concluded via Timeout: {reason}")
                    self._dispatch_next_waypoint()
        except Exception as e:
            self._log.err(f"Exception in mission tick loop: {e}")

    # NAV2 ACTION CLIENT LOGIC
    def _dispatch_next_waypoint(self):
        """Fetches the next waypoint from the manager and sends it to Nav2."""
        wp = self.manager.get_next_waypoint()
        
        if wp is None:
            self._log.succ("Rover has reached the final destination. Mission complete.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(wp.get('x', 0.0))
        goal_msg.pose.pose.position.y = float(wp.get('y', 0.0))
        
        yaw = float(wp.get('yaw', 0.0))
        qz, qw = _yaw_to_quat(yaw)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self._log.info(f"Dispatching Nav2 Goal -> {wp.get('id')}: X:{goal_msg.pose.pose.position.x}, Y:{goal_msg.pose.pose.position.y}")
        
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(lambda f: self._goal_response_callback(f, wp.get('id')))

    def _goal_response_callback(self, future: Future, wp_id: str):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self._log.warn(f"Nav2 rejected waypoint {wp_id}!")
                return

            self._log.info(f"Nav2 accepted waypoint {wp_id}. Tracking progress...")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda f: self._goal_result_callback(f, wp_id))
        except Exception as e:
            self._log.err(f"Error receiving goal response: {e}")

    def _goal_result_callback(self, future: Future, wp_id: str):
        """Called when Nav2 finishes driving to a waypoint."""
        try:
            self._log.succ(f"Arrived at waypoint: {wp_id}")
            
            if wp_id == "WP2":
                self._log.info("Waypoint 2 Reached. Triggering Vision Task Search...")
                self.manager.start_vision_task(time.time())
            else:
                self._dispatch_next_waypoint()
                
        except Exception as e:
            self._log.err(f"Error handling waypoint result: {e}")

    def destroy_node(self):
        """Intercepts node destruction to Unconfigure the worker nodes."""
        # We do not send Lifecycle network calls here because during Ctrl+C, 
        # the ROS context is invalid and the target nodes are already dead.
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MissionManagerNode()
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