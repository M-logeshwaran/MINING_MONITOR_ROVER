from drillpulse_autonomous.utils import log_colored

class StateMachine:
    """
    Finite State Machine governing overall rover operational state and safety state transitions.
    States: IDLE, FREE_ROAM, POINT_NAVIGATION, OBSTACLE_AVOIDANCE, RECOVERY, GOAL_REACHED, EMERGENCY_STOP
    """
    VALID_STATES = {
        "IDLE",
        "FREE_ROAM",
        "POINT_NAVIGATION",
        "OBSTACLE_AVOIDANCE",
        "RECOVERY",
        "GOAL_REACHED",
        "EMERGENCY_STOP"
    }

    def __init__(self, node=None, initial_state="IDLE"):
        self.node = node
        self._current_state = initial_state if initial_state in self.VALID_STATES else "IDLE"
        self.previous_state = self._current_state
        self.state_change_callbacks = []

    @property
    def current_state(self) -> str:
        return self._current_state

    def set_state(self, new_state: str) -> bool:
        """
        Transitions to a new state if valid. Logs transition and notifies listeners.
        """
        new_state = new_state.upper()
        if new_state not in self.VALID_STATES:
            log_colored("AUTO", f"Invalid state transition attempted: {new_state}", getattr(self.node, 'get_logger', lambda: None)())
            return False

        if new_state != self._current_state:
            self.previous_state = self._current_state
            self._current_state = new_state
            log_colored("AUTO", f"STATE TRANSITION: [{self.previous_state}] -> [{self._current_state}]", getattr(self.node, 'get_logger', lambda: None)())
            
            for cb in self.state_change_callbacks:
                try:
                    cb(self.previous_state, self._current_state)
                except Exception as e:
                    log_colored("AUTO", f"State callback exception: {e}", getattr(self.node, 'get_logger', lambda: None)())
            return True
        return False

    def add_callback(self, callback_func):
        """Registers a callback function to be called on state transition."""
        self.state_change_callbacks.append(callback_func)
