import time
from drillpulse_autonomous.utils import log_colored

class SafetyManager:
    """
    Highest priority safety evaluation node.
    Monitors ultrasonic sensors and camera connection state, overriding navigation
    instantly if critical danger or hardware failure is detected.
    """
    def __init__(self, node, ultrasonic_manager, critical_stop=15.0, warning_dist=25.0, safe_dist=35.0):
        self.node = node
        self.ultrasonic_mgr = ultrasonic_manager
        self.critical_stop = critical_stop
        self.warning_dist = warning_dist
        self.safe_dist = safe_dist
        
        self.in_recovery = False
        self.recovery_start_time = 0.0
        self.recovery_phase = "NONE"  # "REVERSE", "ROTATE", "NONE"

    def evaluate_safety(self, vision_rec_dir="CENTER", camera_ok=True):
        """
        Evaluates current sensor inputs against safety rules.
        Returns:
            override_active (bool): True if SafetyManager overrides normal navigation.
            action_command (str): Override action ('STOP', 'REVERSE', 'TURN_LEFT', 'TURN_RIGHT', 'SLOW', 'EMERGENCY_STOP', None)
            target_state (str): Associated state machine state ('OBSTACLE_AVOIDANCE', 'RECOVERY', 'EMERGENCY_STOP', None)
        """
        left_dist, right_dist, sensors_ok = self.ultrasonic_mgr.get_distances()

        # Failsafe Rule 1: Ultrasonic sensors disconnected or unavailable
        if not sensors_ok:
            log_colored("SAFETY", "CRITICAL FAILSAFE: Ultrasonic sensors unavailable! Triggering EMERGENCY_STOP.", getattr(self.node, 'get_logger', lambda: None)())
            return True, "STOP", "EMERGENCY_STOP"

        # Failsafe Rule 2: Camera disconnected (Vision failsafe: rely on ultrasonics)
        if not camera_ok:
            log_colored("SAFETY", "WARNING FAILSAFE: Camera disconnected! Continuing on Ultrasonic safety mode.", getattr(self.node, 'get_logger', lambda: None)())

        # Handle active recovery sequence (STOP -> REVERSE -> ROTATE)
        if self.in_recovery:
            elapsed = time.monotonic() - self.recovery_start_time
            if elapsed < 1.5:
                # Phase 1: Reverse away from obstacle
                return True, "REVERSE", "RECOVERY"
            elif elapsed < 2.5:
                # Phase 2: Rotate to search for open space
                return True, "TURN_RIGHT", "RECOVERY"
            else:
                # Recovery completed
                self.in_recovery = False
                self.recovery_phase = "NONE"
                log_colored("SAFETY", "Recovery maneuver completed. Resuming autonomous navigation.", getattr(self.node, 'get_logger', lambda: None)())

        # Rule 1: IF both sensors less than 15 cm -> Critical Stop & Recovery
        if left_dist < self.critical_stop or right_dist < self.critical_stop:
            log_colored("SAFETY", f"CRITICAL DISTANCE DETECTED! Left: {left_dist:.1f} cm, Right: {right_dist:.1f} cm. Initiating Emergency Recovery!", getattr(self.node, 'get_logger', lambda: None)())
            self.in_recovery = True
            self.recovery_start_time = time.monotonic()
            self.recovery_phase = "REVERSE"
            return True, "REVERSE", "RECOVERY"

        # Rule 2: IF left less than warning (25 cm) -> Turn Right
        if left_dist < self.warning_dist:
            log_colored("SAFETY", f"Warning: Obstacle near Left ({left_dist:.1f} cm). Steering RIGHT.", getattr(self.node, 'get_logger', lambda: None)())
            return True, "TURN_RIGHT", "OBSTACLE_AVOIDANCE"

        # Rule 3: IF right less than warning (25 cm) -> Turn Left
        if right_dist < self.warning_dist:
            log_colored("SAFETY", f"Warning: Obstacle near Right ({right_dist:.1f} cm). Steering LEFT.", getattr(self.node, 'get_logger', lambda: None)())
            return True, "TURN_LEFT", "OBSTACLE_AVOIDANCE"

        # Rule 4: IF both between warning (25 cm) and safe (35 cm) -> Slow down, find opening
        if (self.warning_dist <= left_dist < self.safe_dist) or (self.warning_dist <= right_dist < self.safe_dist):
            if vision_rec_dir == "LEFT":
                log_colored("SAFETY", "Caution zone: Vision recommends turning LEFT.", getattr(self.node, 'get_logger', lambda: None)())
                return True, "TURN_LEFT", "OBSTACLE_AVOIDANCE"
            elif vision_rec_dir == "RIGHT":
                log_colored("SAFETY", "Caution zone: Vision recommends turning RIGHT.", getattr(self.node, 'get_logger', lambda: None)())
                return True, "TURN_RIGHT", "OBSTACLE_AVOIDANCE"
            else:
                log_colored("SAFETY", "Caution zone: Slowing down to navigate narrow corridor.", getattr(self.node, 'get_logger', lambda: None)())
                return True, "SLOW", "OBSTACLE_AVOIDANCE"

        # Rule 5: Clear pathway
        return False, None, None
