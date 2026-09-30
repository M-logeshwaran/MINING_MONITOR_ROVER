import math
import os
import yaml
from drillpulse_autonomous.utils import normalize_angle, euclidean_distance, log_colored

class CoordinateManager:
    """
    Manages spatial coordinates, goal coordinates, waypoint loading,
    and dead-reckoning pose tracking for point-to-point navigation.
    """
    def __init__(self, node=None, nav_points_file=None):
        self.node = node
        
        self.start_x = 0.0
        self.start_y = 0.0
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0  # Heading in radians (0 = Facing East/Forward)
        
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.has_goal = False

        self.saved_waypoints = {}
        if nav_points_file and os.path.exists(nav_points_file):
            self.load_waypoints_from_yaml(nav_points_file)
        else:
            # Default built-in navigation points
            self.saved_waypoints = {
                "Home": {"x": 0.0, "y": 0.0},
                "Lab": {"x": 5.0, "y": 8.0},
                "Desk": {"x": 2.0, "y": 3.0},
                "Door": {"x": 4.0, "y": -2.0}
            }

    def load_waypoints_from_yaml(self, file_path: str):
        """Loads pre-configured named waypoints from navigation_points.yaml."""
        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)
                if data and "navigation_points" in data:
                    self.saved_waypoints = data["navigation_points"]
                    log_colored("NAV", f"Loaded {len(self.saved_waypoints)} waypoints from YAML.", getattr(self.node, 'get_logger', lambda: None)())
        except Exception as e:
            log_colored("NAV", f"Failed to load navigation_points.yaml: {e}", getattr(self.node, 'get_logger', lambda: None)())

    def set_goal(self, x: float, y: float):
        """Sets target goal destination coordinates."""
        self.goal_x = float(x)
        self.goal_y = float(y)
        self.has_goal = True
        log_colored("NAV", f"New Target Goal Set: X={self.goal_x:.2f}, Y={self.goal_y:.2f}", getattr(self.node, 'get_logger', lambda: None)())

    def set_goal_by_name(self, name: str) -> bool:
        """Sets target goal from a named saved waypoint (e.g. 'Lab', 'Desk')."""
        if name in self.saved_waypoints:
            wp = self.saved_waypoints[name]
            self.set_goal(wp["x"], wp["y"])
            return True
        log_colored("NAV", f"Waypoint '{name}' not found in configuration!", getattr(self.node, 'get_logger', lambda: None)())
        return False

    def update_dead_reckoning(self, last_command: str, dt: float, linear_speed: float = 0.3, angular_speed: float = 0.8):
        """
        Predicts position changes based on motor movement commands when physical wheel odometry is unavailable.
        """
        cmd = last_command.upper() if last_command else "STOP"
        
        if "FORWARD" in cmd or "CMD,0,1" in cmd:
            self.current_x += linear_speed * math.cos(self.current_theta) * dt
            self.current_y += linear_speed * math.sin(self.current_theta) * dt
        elif "BACKWARD" in cmd or "REVERSE" in cmd or "CMD,0,-1" in cmd:
            self.current_x -= linear_speed * math.cos(self.current_theta) * dt
            self.current_y -= linear_speed * math.sin(self.current_theta) * dt
        elif "LEFT" in cmd or "CMD,-1,0" in cmd:
            self.current_theta = normalize_angle(self.current_theta + angular_speed * dt)
        elif "RIGHT" in cmd or "CMD,1,0" in cmd:
            self.current_theta = normalize_angle(self.current_theta - angular_speed * dt)

    def get_distance_to_goal(self) -> float:
        """Returns distance from current position to goal."""
        if not self.has_goal:
            return 0.0
        return euclidean_distance(self.current_x, self.current_y, self.goal_x, self.goal_y)

    def get_bearing_to_goal(self) -> float:
        """Returns target bearing angle (in radians) toward goal relative to current pose."""
        if not self.has_goal:
            return 0.0
        target_angle = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        heading_error = normalize_angle(target_angle - self.current_theta)
        return heading_error
