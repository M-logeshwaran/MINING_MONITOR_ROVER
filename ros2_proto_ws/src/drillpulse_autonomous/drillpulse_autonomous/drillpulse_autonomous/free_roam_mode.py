import time
import random
from drillpulse_autonomous.utils import log_colored

class FreeRoamMode:
    """
    Mode 1: Autonomous Free Roam / Swarm Mode controller.
    Drives the rover continuously while exploring, avoiding obstacles, escaping corners/dead-ends,
    and seeking open corridors using random bias and turn memory.
    """
    def __init__(self, node=None):
        self.node = node
        self.last_turn_dir = "RIGHT"
        self.consecutive_turns = 0
        self.last_recovery_time = 0.0

    def compute_step(self, left_dist, right_dist, vision_rec_dir, corridor_info):
        """
        Computes the next exploration movement action based on sensor and vision inputs.
        Returns:
            action (str): 'FORWARD', 'TURN_LEFT', 'TURN_RIGHT', 'REVERSE', 'SEARCH_OPENING'
            sub_state (str): Internal state string for visualization HUD
        """
        left_blocked = corridor_info.get("left_blocked", False)
        center_blocked = corridor_info.get("center_blocked", False)
        right_blocked = corridor_info.get("right_blocked", False)

        # 1. Dead-end & Corner Escape Recovery
        if (left_dist < 20.0 and right_dist < 20.0) or (left_blocked and center_blocked and right_blocked):
            log_colored("AUTO", "Free Roam: Corner / Dead-end detected! Initiating Reverse & Turn Recovery.", getattr(self.node, 'get_logger', lambda: None)())
            self.consecutive_turns += 1
            return "REVERSE", "RECOVERY"

        # 2. Clear Path Ahead -> Immediately Straighten and Move FORWARD
        if not center_blocked and left_dist >= 25.0 and right_dist >= 25.0:
            self.consecutive_turns = 0
            return "FORWARD", "FORWARD"

        # 3. Vision-guided Corridor Navigation
        if vision_rec_dir in ["LEFT", "RIGHT"]:
            self.last_turn_dir = vision_rec_dir
            return f"TURN_{vision_rec_dir}", f"TURN_{vision_rec_dir}"

        # 4. Sensor-guided Wall Avoidance
        if left_dist < right_dist:
            self.last_turn_dir = "RIGHT"
            return "TURN_RIGHT", "TURN_RIGHT"
        elif right_dist < left_dist:
            self.last_turn_dir = "LEFT"
            return "TURN_LEFT", "TURN_LEFT"

        # 5. Loop Prevention & Random Exploration Bias when Ambiguous
        if self.consecutive_turns > 3:
            # Alternate turn direction to prevent spinning in tight loops
            bias_dir = "RIGHT" if self.last_turn_dir == "LEFT" else "LEFT"
            self.consecutive_turns = 0
            log_colored("AUTO", f"Free Roam: Loop prevention active. Applying random bias turn: {bias_dir}", getattr(self.node, 'get_logger', lambda: None)())
            return f"TURN_{bias_dir}", "SEARCH_OPENING"

        # Default fallback: Turn toward wider free space
        choice = random.choice(["LEFT", "RIGHT"])
        return f"TURN_{choice}", "SEARCH_OPENING"
