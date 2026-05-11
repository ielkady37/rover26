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

class MissionManager:
    """
    Manages phase transitions, waypoint queues, and vision task timeouts.
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

        self.waypoints = waypoints
        self.vision_timeout_sec = float(vision_timeout_sec)
        self.min_confidence = float(min_confidence)

        # Initial State
        self.current_phase = Phase.MANUAL
        self.vision_state = VisionState.IDLE
        self.vision_start_time = 0.0
        
        self._log.succ(f"MissionManager init: {len(self.waypoints)} waypoints loaded.")

    def set_phase(self, new_phase_str: str) -> bool:
        """
        Safely attempts to transition the mission state based on string input.
        Returns True if the state changed, False if ignored.
        """
        target = new_phase_str.strip().lower()
        
        if target == Phase.MANUAL.value and self.current_phase != Phase.MANUAL:
            self.current_phase = Phase.MANUAL
            self._log.info("State Machine transitioned to MANUAL phase.")
            return True
            
        elif target == Phase.AUTONOMOUS.value and self.current_phase != Phase.AUTONOMOUS:
            self.current_phase = Phase.AUTONOMOUS
            self._log.info("State Machine transitioned to AUTONOMOUS phase.")
            return True
            
        return False

    def is_autonomous(self) -> bool:
        return self.current_phase == Phase.AUTONOMOUS

    def get_next_waypoint(self) -> Optional[Dict]:
        """Pops the next waypoint from the queue. Returns None if mission complete."""
        if len(self.waypoints) > 0:
            wp = self.waypoints.pop(0)
            self._log.info(f"Targeting Waypoint: {wp.get('id', 'Unknown')}")
            return wp
        else:
            self.current_phase = Phase.COMPLETE
            self._log.succ("Waypoint queue empty. Mission Complete!")
            return None

    def peek_current_waypoint_id(self) -> str:
        """Looks at the next waypoint without removing it."""
        if len(self.waypoints) > 0:
            return self.waypoints[0].get('id', 'Unknown')
        return "NONE"

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