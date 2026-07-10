#!/usr/bin/env python3
from enum import Enum
from typing import List, Dict, Optional, Tuple
from utils.Logger import RoverLogger

class Phase(Enum):
    MANUAL = "manual"
    AUTONOMOUS = "autonomous"
    COMPLETE = "complete"

class VisionState(Enum):
    IDLE = 1
    SEARCHING = 2
    RESOLVED = 3

class SubState(Enum):
    """
    Sub-states that only apply while Phase.AUTONOMOUS is active. None of
    these are meaningful in MANUAL or COMPLETE.

        LANE            -> LaneGoalPublisher drives, watching for WP1 arrival
        GPS_WAYPOINT    -> MissionManagerNode drives Nav2 straight at a
                            waypoint from GPSWaypointNode's /waypoints_xy
        FACE_DETECTION  -> spin-in-place search + vision lock, at WP2
        LANE_SCAN       -> spin-in-place until lanes are redetected, at WP3
    """
    LANE = "lane"
    GPS_WAYPOINT = "gps_waypoint"
    FACE_DETECTION = "face_detection"
    LANE_SCAN = "lane_scan"

class MissionManager:
    """
    Manages phase transitions, the autonomous sub-state machine, and vision
    task timeouts.
    """

    def __init__(self, waypoints: List[Dict], vision_timeout_sec: float, min_confidence: float):
        self._log = RoverLogger()

        # Constraints check
        if not isinstance(waypoints, list):
            self._log.err("Waypoints must be a list of dictionaries.")
            raise TypeError("Waypoints must be a list of dictionaries.")
        if vision_timeout_sec <= 0.0 or min_confidence < 0.0 or min_confidence > 1.0:
            self._log.err("Invalid vision parameters in config.")
            raise ValueError("Invalid vision parameters in config.")

        # NOTE: waypoints (x/y/yaw from mission.yaml) are no longer used to
        # drive Nav2 directly during AUTONOMOUS — GPSWaypointNode's
        # /waypoints_xy (converted from lat/lon at runtime) is the source of
        # truth for goal coordinates now. This list is kept only for its
        # ids/labels via get_waypoint_label(), and for backwards
        # compatibility with anything else still reading it.
        self.waypoints = waypoints
        self.vision_timeout_sec = float(vision_timeout_sec)
        self.min_confidence = float(min_confidence)

        # Phase state
        self.current_phase = Phase.MANUAL
        self.vision_state = VisionState.IDLE
        self.vision_start_time = 0.0

        # Autonomous sub-state — only meaningful while current_phase ==
        # AUTONOMOUS. None while MANUAL/COMPLETE.
        self.sub_state: Optional[SubState] = None
        # 0-based index into GPSWaypointNode's /waypoints_xy array
        # (0=WP1, 1=WP2, 2=WP3) that GPS_WAYPOINT is currently driving to.
        self.target_wp_index: Optional[int] = None

        self._log.succ(f"MissionManager init: {len(self.waypoints)} waypoints loaded.")

    def set_phase(self, new_phase_str: str) -> bool:
        """
        Safely attempts to transition the mission state based on string input.
        Returns True if the state changed, False if ignored.
        """
        target = new_phase_str.strip().lower()

        if target == Phase.MANUAL.value and self.current_phase != Phase.MANUAL:
            self.current_phase = Phase.MANUAL
            self.reset_sub_state()
            self._log.info("State Machine transitioned to MANUAL phase.")
            return True

        elif target == Phase.AUTONOMOUS.value and self.current_phase != Phase.AUTONOMOUS:
            self.current_phase = Phase.AUTONOMOUS
            self._log.info("State Machine transitioned to AUTONOMOUS phase.")
            return True

        return False

    def is_autonomous(self) -> bool:
        return self.current_phase == Phase.AUTONOMOUS

    # ------------------------------------------------------------------
    # Autonomous sub-state machine

    def enter_autonomous_driving(self) -> None:
        """Call once, right after Phase flips to AUTONOMOUS — begin on LANE."""
        self.sub_state = SubState.LANE
        self.target_wp_index = None
        self._log.info("Sub-state machine entering LANE (autonomous driving start).")

    def set_sub_state(self, new_state: SubState) -> None:
        if new_state == self.sub_state:
            return
        self._log.info(
            f"[SUBSTATE] {self.sub_state.value.upper() if self.sub_state else 'NONE'} "
            f"-> {new_state.value.upper()}"
        )
        self.sub_state = new_state

    def reset_sub_state(self) -> None:
        self.sub_state = None
        self.target_wp_index = None

    def get_waypoint_label(self, index: int) -> str:
        """Best-effort id/label lookup for logging only — falls back to WP<n>."""
        if 0 <= index < len(self.waypoints):
            return self.waypoints[index].get('id', f'WP{index + 1}')
        return f'WP{index + 1}'

    # ------------------------------------------------------------------
    # Vision task (unchanged — still driven by /target_status regardless of
    # where the underlying detection data comes from)

    def start_vision_task(self, current_time: float) -> None:
        """Triggers the search phase and starts the safety timeout clock."""
        self.vision_state = VisionState.SEARCHING
        self.vision_start_time = current_time
        self._log.info(f"Vision Task initiated. Timeout set for {self.vision_timeout_sec}s.")

    def evaluate_target_status(self, is_found: bool, confidence: float, current_time: float) -> Tuple[bool, str]:
        """
        Evaluates incoming CV data against constraints.
        Returns: (True/False if resolved, Reason string)
        """
        if self.vision_state != VisionState.SEARCHING:
            return False, "Not searching"

        # Check for success
        if is_found and confidence >= self.min_confidence:
            self.vision_state = VisionState.RESOLVED
            msg = f"Target FOUND with {confidence*100:.1f}% confidence."
            self._log.succ(msg)
            return True, msg

        # Check for timeout (Safety constraint)
        elapsed = current_time - self.vision_start_time
        if elapsed > self.vision_timeout_sec:
            self.vision_state = VisionState.RESOLVED
            msg = f"Vision Task TIMED OUT after {elapsed:.1f}s."
            self._log.warn(msg)
            return True, msg

        return False, "Still searching"