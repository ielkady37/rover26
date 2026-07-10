#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

from collections import deque

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
from nav2_msgs.action import NavigateToPose
from interfaces.msg import LaneDetectionResult

try:
    from interfaces.msg import TargetStatus
except ImportError:
    TargetStatus = None

import time
from control.services.MissionManager import MissionManager, Phase, VisionState, SubState
from utils.Configurator import Configurator
from utils.Logger import RoverLogger

def _yaw_to_quat(yaw: float):
    half = yaw * 0.5
    return math.sin(half), math.cos(half)  # (z, w)

# Latched QoS — matches GPSWaypointNode's /waypoints_xy publisher so this
# node gets the one-time waypoint conversion even if it starts up later.
_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
)

class MissionManagerNode(Node):
    """
    The Supreme Orchestrator.
    Handles Lifecycle Management (Configure, Activate, Deactivate, Cleanup),
    Nav2 Action goals, GPS-waypoint navigation, and the autonomous sub-state
    machine (LANE / GPS_WAYPOINT / FACE_DETECTION / LANE_SCAN).
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

            # GPS-driven sub-state machine params
            self._wp_arrival_tol_m = float(mission_data.get('wp_arrival_tolerance_m', 3.0))
            self._spin_vel = float(mission_data.get('mission_spin_vel', 0.6))
            self._nav_goal_retry_s = float(mission_data.get('nav_goal_retry_sec', 4.0))

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
        self.face_recognition_client = self.create_client(ChangeState, '/face_recognition_node/change_state')
        self.lane_detection_client = self.create_client(ChangeState, '/lane_detection_node/change_state')

        # Lifecycle transition machinery: per-node FIFO queues so transitions
        # to the same node run strictly one at a time, plus GetState clients
        # to verify a transition is valid before sending it (see
        # _change_lifecycle_state for why this matters).
        self._get_state_clients: dict = {}
        self._transition_queues: dict = {}
        self._transition_busy: set = set()

        # 3. Setup Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._gps_nav_active: bool = False
        self._gps_goal_handle = None
        self._gps_goal_generation: int = 0
        self._last_gps_nav_send_s: float | None = None

        # 4. Setup Subscribers
        self.state_sub = self.create_subscription(String, '/mission_state', self.mission_state_callback, 10)
        if TargetStatus:
            self.target_sub = self.create_subscription(TargetStatus, '/target_status', self.target_status_callback, 10)

        # GPS-waypoint + lane-status inputs for the sub-state machine.
        self._waypoints_xy: list[tuple[float, float]] | None = None
        self._gps_xy: tuple[float, float] | None = None
        self._lane_valid: bool = False
        self.waypoints_xy_sub = self.create_subscription(
            Path, '/waypoints_xy', self._waypoints_xy_cb, _LATCHED_QOS
        )
        self.gps_xy_sub = self.create_subscription(Odometry, '/gps_xy', self._gps_xy_cb, 10)
        self.lane_status_sub = self.create_subscription(
            LaneDetectionResult, '/lane_detection', self._lane_status_cb, 10
        )

        # Sub-state broadcast — LaneGoalPublisher gates on this (only drives
        # while it reads "lane") and stays fully dormant otherwise.
        self.substate_pub = self.create_publisher(String, '/mission_substate', _LATCHED_QOS)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._publish_substate('manual')

        # 5. Setup Watchdog/Tick Timer (runs at 10Hz)
        self.tick_timer = self.create_timer(0.1, self.tick_callback)

        # 6. Execute Boot Sequence
        self._log.info("Waiting for Navigation, Camera, and Perception nodes to come online...")
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
        self.face_recognition_client.wait_for_service(timeout_sec=5.0)
        self.lane_detection_client.wait_for_service(timeout_sec=5.0)
        self._log.info("Configuring Navigation, Camera, and Perception Nodes (Loading YAMLs, Shared Memory, etc.)...")

        # 1. Send CONFIGURE to both (Moves them to 'Inactive')
        self._change_lifecycle_state(self.manual_client, Transition.TRANSITION_CONFIGURE, "ManualNav")
        self._change_lifecycle_state(self.auto_client, Transition.TRANSITION_CONFIGURE, "AutoNav")
        self._change_lifecycle_state(self.manual_camera_client, Transition.TRANSITION_CONFIGURE, "ManualCamera")
        self._change_lifecycle_state(self.auto_camera_client, Transition.TRANSITION_CONFIGURE, "AutoCamera")
        self._change_lifecycle_state(self.face_recognition_client, Transition.TRANSITION_CONFIGURE, "FaceRecognition")
        self._change_lifecycle_state(self.lane_detection_client, Transition.TRANSITION_CONFIGURE, "LaneDetection")

        # 2. Automatically Activate Phase 1 (Manual)
        self._log.info("Boot sequence complete. Activating default Phase 1 (Manual).")
        self._change_lifecycle_state(self.manual_client, Transition.TRANSITION_ACTIVATE, "ManualNav")
        self._change_lifecycle_state(self.manual_camera_client, Transition.TRANSITION_ACTIVATE, "ManualCamera")

    def _execute_transition_to_manual(self):
        """Safely pauses Auto and spins up Manual."""
        self._log.info("Transitioning stack to MANUAL control...")

        self._stop_gps_waypoint_nav()
        self._publish_twist(0.0)
        self._publish_substate('manual')

        # 1. Deactivate Autonomous Node (Active -> Inactive)
        self._change_lifecycle_state(self.auto_client, Transition.TRANSITION_DEACTIVATE, "AutoNav")
        self._change_lifecycle_state(self.auto_camera_client, Transition.TRANSITION_DEACTIVATE, "AutoCamera")
        self._change_lifecycle_state(self.face_recognition_client, Transition.TRANSITION_DEACTIVATE, "FaceRecognition")
        self._change_lifecycle_state(self.lane_detection_client, Transition.TRANSITION_DEACTIVATE, "LaneDetection")

        # 2. Activate Manual Node (Inactive -> Active)
        self._change_lifecycle_state(self.manual_client, Transition.TRANSITION_ACTIVATE, "ManualNav")
        self._change_lifecycle_state(self.manual_camera_client, Transition.TRANSITION_ACTIVATE, "ManualCamera")

    def _execute_transition_to_auto(self):
        """Safely pauses Manual, spins up Auto+LaneDetection, and starts on LANE."""
        self._log.info("Transitioning stack to AUTONOMOUS control...")

        # 1. Deactivate Manual Node (Active -> Inactive)
        self._change_lifecycle_state(self.manual_client, Transition.TRANSITION_DEACTIVATE, "ManualNav")
        self._change_lifecycle_state(self.manual_camera_client, Transition.TRANSITION_DEACTIVATE, "ManualCamera")

        # 2. Activate Autonomous Node + LaneDetection (Inactive -> Active)
        # FaceRecognition stays Inactive here - it is only activated upon reaching WP2.
        self._change_lifecycle_state(self.auto_client, Transition.TRANSITION_ACTIVATE, "AutoNav")
        self._change_lifecycle_state(self.auto_camera_client, Transition.TRANSITION_ACTIVATE, "AutoCamera")
        self._change_lifecycle_state(self.lane_detection_client, Transition.TRANSITION_ACTIVATE, "LaneDetection")

        # 3. Ensure Nav2 is ready before we might need it for GPS_WAYPOINT later
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self._log.err("Nav2 Action Server not available! Cannot proceed with mission.")
            return

        # 4. Enter the sub-state machine on LANE — LaneGoalPublisher takes
        # over driving as soon as it sees /mission_substate == "lane".
        self.manager.enter_autonomous_driving()
        self._publish_substate('lane')

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
        self._change_lifecycle_state(self.face_recognition_client, Transition.TRANSITION_DEACTIVATE, "FaceRecognition")
        self._change_lifecycle_state(self.lane_detection_client, Transition.TRANSITION_DEACTIVATE, "LaneDetection")

        # Then Unconfigure (Cleanup)
        self._change_lifecycle_state(self.manual_client, Transition.TRANSITION_CLEANUP, "ManualNav")
        self._change_lifecycle_state(self.manual_camera_client, Transition.TRANSITION_CLEANUP, "ManualCamera")
        self._change_lifecycle_state(self.auto_client, Transition.TRANSITION_CLEANUP, "AutoNav")
        self._change_lifecycle_state(self.auto_camera_client, Transition.TRANSITION_CLEANUP, "AutoCamera")
        self._change_lifecycle_state(self.face_recognition_client, Transition.TRANSITION_CLEANUP, "FaceRecognition")
        self._change_lifecycle_state(self.lane_detection_client, Transition.TRANSITION_CLEANUP, "LaneDetection")

    # Which primary state each transition may legally start from
    # (rcl_lifecycle state machine rules).
    _TRANSITION_VALID_FROM = {
        Transition.TRANSITION_CONFIGURE:  (State.PRIMARY_STATE_UNCONFIGURED,),
        Transition.TRANSITION_ACTIVATE:   (State.PRIMARY_STATE_INACTIVE,),
        Transition.TRANSITION_DEACTIVATE: (State.PRIMARY_STATE_ACTIVE,),
        Transition.TRANSITION_CLEANUP:    (State.PRIMARY_STATE_INACTIVE,),
    }

    def _change_lifecycle_state(self, client: rclpy.client.Client, transition_id: int, node_label: str):
        """Queue a lifecycle transition for *client*'s node.

        Transitions to the same node run strictly one at a time, and each is
        checked against the node's live state (via /get_state) before being
        sent. Requesting a transition that is invalid for the current state
        (e.g. ACTIVATE on an already-active node) raises an unhandled
        RCLError inside the TARGET node on Jazzy and kills its process, so
        those requests are skipped with a log line instead.
        """
        key = client.srv_name
        self._transition_queues.setdefault(key, deque()).append(
            (client, transition_id, node_label)
        )
        self._pump_transition_queue(key)

    def _pump_transition_queue(self, key: str):
        """Start the next queued transition for a node, if none is in flight."""
        if key in self._transition_busy:
            return
        queue = self._transition_queues.get(key)
        if not queue:
            return

        client, transition_id, node_label = queue.popleft()
        self._transition_busy.add(key)

        if not client.wait_for_service(timeout_sec=2.0):
            self._log.warn(f"Lifecycle service {client.srv_name} not available.")
            self._finish_transition(key)
            return

        gs_client = self._get_state_client(client)
        if gs_client is None or not gs_client.wait_for_service(timeout_sec=1.0):
            # Cannot verify the node's state — send unchecked (old behaviour).
            self._send_transition(key, client, transition_id, node_label)
            return

        future = gs_client.call_async(GetState.Request())
        future.add_done_callback(
            lambda f: self._on_state_checked(f, key, client, transition_id, node_label)
        )

    def _finish_transition(self, key: str):
        self._transition_busy.discard(key)
        self._pump_transition_queue(key)

    def _get_state_client(self, change_client: rclpy.client.Client):
        """Return (and cache) the GetState client matching a ChangeState client."""
        srv_name = change_client.srv_name
        if not srv_name.endswith('/change_state'):
            return None
        gs_name = srv_name[: -len('/change_state')] + '/get_state'
        if gs_name not in self._get_state_clients:
            self._get_state_clients[gs_name] = self.create_client(GetState, gs_name)
        return self._get_state_clients[gs_name]

    def _on_state_checked(self, future: Future, key: str, client: rclpy.client.Client,
                          transition_id: int, node_label: str):
        try:
            current = future.result().current_state
        except Exception as e:
            self._log.warn(f"{node_label}: get_state failed ({e}) — sending transition unchecked.")
            self._send_transition(key, client, transition_id, node_label)
            return

        valid_from = self._TRANSITION_VALID_FROM.get(transition_id)
        if valid_from is not None and current.id not in valid_from:
            self._log.info(
                f'{node_label}: skipping transition {transition_id} — '
                f'not valid from current state "{current.label}".'
            )
            self._finish_transition(key)
            return

        self._send_transition(key, client, transition_id, node_label)

    def _send_transition(self, key: str, client: rclpy.client.Client,
                         transition_id: int, node_label: str):
        req = ChangeState.Request()
        req.transition.id = transition_id

        # Send async so we don't block the orchestrator
        future = client.call_async(req)
        future.add_done_callback(lambda f: self._lifecycle_result_cb(f, key, node_label, transition_id))

    def _lifecycle_result_cb(self, future: Future, key: str, node_label: str, transition_id: int):
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
        finally:
            self._finish_transition(key)

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
        """Receives data from the CV pipeline (via TargetAcquisitionNode) if
        we are actively searching at WP2."""
        try:
            resolved, reason = self.manager.evaluate_target_status(
                is_found=msg.is_found,
                confidence=msg.confidence_score,
                current_time=time.time()
            )

            if resolved:
                self._log.info(f"Vision Task Concluded: {reason}")
                self._conclude_vision_task() # Proceed to WP3

        except Exception as e:
            self._log.err(f"Error evaluating target status: {e}")

    def _waypoints_xy_cb(self, msg: Path) -> None:
        self._waypoints_xy = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._log.info(f"GPS waypoints received: {self._waypoints_xy}")

    def _gps_xy_cb(self, msg: Odometry) -> None:
        self._gps_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _lane_status_cb(self, msg: LaneDetectionResult) -> None:
        self._lane_valid = (len(msg.left_lane_coeffs) == 3 and len(msg.right_lane_coeffs) == 3)

    def tick_callback(self):
        """Background loop: vision timeout watchdog + autonomous sub-state machine."""
        try:
            if self.manager.vision_state == VisionState.SEARCHING:
                resolved, reason = self.manager.evaluate_target_status(False, 0.0, time.time())
                if resolved:
                    self._log.warn(f"Vision Task Concluded via Timeout: {reason}")
                    self._conclude_vision_task()

            if self.manager.is_autonomous():
                self._tick_autonomous_substate()
        except Exception as e:
            self._log.err(f"Exception in mission tick loop: {e}")

    # AUTONOMOUS SUB-STATE MACHINE
    def _publish_substate(self, value: str) -> None:
        self.substate_pub.publish(String(data=value))

    def _publish_twist(self, angular_z: float) -> None:
        t = Twist()
        t.angular.z = angular_z
        self.cmd_vel_pub.publish(t)

    def _tick_autonomous_substate(self) -> None:
        state = self.manager.sub_state
        if state == SubState.LANE:
            self._tick_lane_substate()
        elif state == SubState.GPS_WAYPOINT:
            self._tick_gps_waypoint_substate()
        elif state == SubState.FACE_DETECTION:
            self._tick_face_detection_substate()
        elif state == SubState.LANE_SCAN:
            self._tick_lane_scan_substate()

    def _tick_lane_substate(self) -> None:
        """LANE: LaneGoalPublisher is driving. Just watch for WP1 arrival."""
        if self._waypoints_xy is None or self._gps_xy is None or len(self._waypoints_xy) < 1:
            return
        wp1 = self._waypoints_xy[0]
        dist = math.hypot(self._gps_xy[0] - wp1[0], self._gps_xy[1] - wp1[1])
        if dist <= self._wp_arrival_tol_m:
            self._log.succ(f"Waypoint 1 reached (gps dist={dist:.2f}m) — switching to GPS_WAYPOINT for WP2")
            self.manager.set_sub_state(SubState.GPS_WAYPOINT)
            self.manager.target_wp_index = 1  # WP2
            self._publish_substate('gps_waypoint')
            self._dispatch_gps_goal(1)

    def _tick_gps_waypoint_substate(self) -> None:
        """GPS_WAYPOINT: dispatch/retry a NavigateToPose goal toward target_wp_index."""
        if self._gps_nav_active or self.manager.target_wp_index is None:
            return
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if self._last_gps_nav_send_s is None or (now_s - self._last_gps_nav_send_s) >= self._nav_goal_retry_s:
            self._dispatch_gps_goal(self.manager.target_wp_index)

    def _tick_face_detection_substate(self) -> None:
        """FACE_DETECTION: spin in place while searching, stop once resolved."""
        if self.manager.vision_state == VisionState.SEARCHING:
            self._publish_twist(self._spin_vel)
        else:
            self._publish_twist(0.0)

    def _tick_lane_scan_substate(self) -> None:
        """LANE_SCAN: spin in place until lane_detection reports valid lanes."""
        if self._lane_valid:
            self._publish_twist(0.0)
            self._log.succ("Lanes redetected — switching back to LANE")
            self.manager.set_sub_state(SubState.LANE)
            self._publish_substate('lane')
        else:
            self._publish_twist(self._spin_vel)

    def _conclude_vision_task(self):
        """Deactivates FaceRecognition (only needed at WP2), stops the spin,
        and resumes the mission toward WP3."""
        self._publish_twist(0.0)
        self._change_lifecycle_state(self.face_recognition_client, Transition.TRANSITION_DEACTIVATE, "FaceRecognition")
        self.manager.set_sub_state(SubState.GPS_WAYPOINT)
        self.manager.target_wp_index = 2  # WP3
        self._publish_substate('gps_waypoint')
        self._dispatch_gps_goal(2)

    # NAV2 ACTION CLIENT LOGIC — GPS waypoints
    def _dispatch_gps_goal(self, index: int) -> None:
        """Sends a NavigateToPose goal at GPSWaypointNode's local x,y for waypoint `index`."""
        if self._waypoints_xy is None or index >= len(self._waypoints_xy):
            self._log.warn(f"GPS_WAYPOINT: waypoint {index} not available yet — will retry")
            return

        tx, ty = self._waypoints_xy[index]
        yaw = 0.0
        if self._gps_xy is not None:
            yaw = math.atan2(ty - self._gps_xy[1], tx - self._gps_xy[0])

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = tx
        goal_msg.pose.pose.position.y = ty
        qz, qw = _yaw_to_quat(yaw)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self._gps_goal_generation += 1
        gen = self._gps_goal_generation
        self._last_gps_nav_send_s = self.get_clock().now().nanoseconds * 1e-9

        label = self.manager.get_waypoint_label(index)
        self._log.info(f"Dispatching GPS_WAYPOINT goal gen={gen} -> {label}: X:{tx:.2f}, Y:{ty:.2f}")

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(lambda f, g=gen, i=index: self._gps_goal_response_callback(f, g, i))

    def _gps_goal_response_callback(self, future: Future, gen: int, index: int):
        if gen != self._gps_goal_generation:
            return  # stale — a newer goal has since been dispatched
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self._log.warn(f"Nav2 rejected GPS_WAYPOINT goal (index={index}) — will retry")
                return

            self._log.info(f"Nav2 accepted GPS_WAYPOINT goal (index={index}). Tracking progress...")
            self._gps_nav_active = True
            self._gps_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda f, g=gen, i=index: self._gps_goal_result_callback(f, g, i))
        except Exception as e:
            self._log.err(f"Error receiving GPS_WAYPOINT goal response: {e}")

    def _gps_goal_result_callback(self, future: Future, gen: int, index: int):
        """Called when Nav2 finishes driving to a GPS waypoint."""
        if gen != self._gps_goal_generation:
            return  # stale
        self._gps_nav_active = False
        self._gps_goal_handle = None

        STATUS_SUCCEEDED = 4
        try:
            status = future.result().status
        except Exception as e:
            self._log.err(f"Error handling GPS_WAYPOINT result: {e}")
            return

        if status != STATUS_SUCCEEDED:
            self._log.warn(f"GPS_WAYPOINT goal (index={index}) did not succeed (status={status}) — retrying")
            return  # _tick_gps_waypoint_substate will redispatch after nav_goal_retry_s

        label = self.manager.get_waypoint_label(index)
        self._log.succ(f"Arrived at {label}")

        if index == 1:  # WP2 — start the face-detection search
            self._log.info("Waypoint 2 reached. Activating FaceRecognition and starting FACE_DETECTION...")
            self.manager.set_sub_state(SubState.FACE_DETECTION)
            self._publish_substate('face_detection')
            self._change_lifecycle_state(self.face_recognition_client, Transition.TRANSITION_ACTIVATE, "FaceRecognition")
            self.manager.start_vision_task(time.time())
        elif index == 2:  # WP3 — start the lane-reacquisition spin
            self._log.info("Waypoint 3 reached. Starting LANE_SCAN...")
            self.manager.set_sub_state(SubState.LANE_SCAN)
            self._publish_substate('lane_scan')

    def _stop_gps_waypoint_nav(self) -> None:
        """Cancel any in-flight GPS_WAYPOINT Nav2 goal (e.g. on return to MANUAL)."""
        if self._gps_goal_handle is not None:
            self._gps_goal_handle.cancel_goal_async()
            self._gps_goal_handle = None
        self._gps_nav_active = False
        self._gps_goal_generation += 1  # invalidate any in-flight callbacks

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