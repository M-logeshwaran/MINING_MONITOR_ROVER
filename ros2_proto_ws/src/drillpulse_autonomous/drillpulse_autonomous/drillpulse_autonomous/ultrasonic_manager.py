import time
import collections
import statistics
from std_msgs.msg import Float32
from drillpulse_autonomous.utils import log_colored

class UltrasonicManager:
    """
    Manages left and right ultrasonic sensor streams, applying median filtering,
    noise rejection, threshold evaluation, and sensor connection watchdog.
    """
    def __init__(self, node, left_topic="/left_distance", right_topic="/right_distance",
                 critical_stop=15.0, warning_dist=25.0, safe_dist=35.0,
                 window_size=5, simulation=False):
        self.node = node
        self.left_topic = left_topic
        self.right_topic = right_topic
        
        self.critical_stop = critical_stop
        self.warning_dist = warning_dist
        self.safe_dist = safe_dist
        self.window_size = window_size
        self.simulation = simulation

        self.left_buffer = collections.deque(maxlen=window_size)
        self.right_buffer = collections.deque(maxlen=window_size)
        
        self.latest_left_stable = 100.0  # Default clear distance in cm
        self.latest_right_stable = 100.0
        
        self.last_left_time = 0.0
        self.last_right_time = 0.0
        
        self.left_connected = False
        self.right_connected = False

        if self.node is not None:
            self.left_sub = self.node.create_subscription(
                Float32,
                self.left_topic,
                self._left_callback,
                10
            )
            self.right_sub = self.node.create_subscription(
                Float32,
                self.right_topic,
                self._right_callback,
                10
            )
            log_colored("ULTRASONIC", f"Subscribed to ultrasonic topics: {self.left_topic}, {self.right_topic}", self.node.get_logger())

    def _left_callback(self, msg: Float32):
        val = float(msg.data)
        if self._is_valid_reading(val):
            self.left_buffer.append(val)
            self.latest_left_stable = statistics.median(self.left_buffer)
            self.last_left_time = time.monotonic()
            self.left_connected = True

    def _right_callback(self, msg: Float32):
        val = float(msg.data)
        if self._is_valid_reading(val):
            self.right_buffer.append(val)
            self.latest_right_stable = statistics.median(self.right_buffer)
            self.last_right_time = time.monotonic()
            self.right_connected = True

    def _is_valid_reading(self, val: float) -> bool:
        """Filters out invalid sensor spikes, zeros, NaNs, and out-of-range values."""
        if val is None or val != val:  # NaN check
            return False
        if val <= 0.0 or val > 400.0:  # HC-SR04 valid range is 2cm to 400cm
            return False
        return True

    def get_distances(self):
        """
        Returns (left_dist, right_dist, sensors_ok)
        Applies watchdog check for sensor timeouts.
        """
        now = time.monotonic()
        
        if not self.simulation:
            # Watchdog timeout check (1.0 second threshold)
            if now - self.last_left_time > 1.0:
                self.left_connected = False
            if now - self.last_right_time > 1.0:
                self.right_connected = False
            
            sensors_ok = self.left_connected and self.right_connected
        else:
            # In simulation, if ROS topics are not active, provide safe mock clear values
            if now - self.last_left_time > 1.0:
                self.latest_left_stable = 100.0
                self.left_connected = True
            if now - self.last_right_time > 1.0:
                self.latest_right_stable = 100.0
                self.right_connected = True
            sensors_ok = True

        return self.latest_left_stable, self.latest_right_stable, sensors_ok

    def evaluate_safety_status(self):
        """
        Evaluates current ultrasonic distances against thresholds.
        Returns status string: 'CRITICAL', 'WARNING_LEFT', 'WARNING_RIGHT', 'WARNING_BOTH', 'CLEAR'
        """
        left, right, ok = self.get_distances()
        if not ok and not self.simulation:
            return "UNAVAILABLE"

        if left < self.critical_stop or right < self.critical_stop:
            return "CRITICAL"
        elif left < self.warning_dist and right < self.warning_dist:
            return "WARNING_BOTH"
        elif left < self.warning_dist:
            return "WARNING_LEFT"
        elif right < self.warning_dist:
            return "WARNING_RIGHT"
        elif left < self.safe_dist or right < self.safe_dist:
            return "CAUTION"
        else:
            return "CLEAR"
