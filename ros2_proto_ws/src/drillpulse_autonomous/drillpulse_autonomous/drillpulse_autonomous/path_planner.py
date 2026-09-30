import math
import heapq
from drillpulse_autonomous.utils import normalize_angle, log_colored

class PathPlanner:
    """
    Lightweight local path planner and A* grid search algorithm.
    Computes optimal movement steps toward goal while dynamically bypassing obstacle blockages.
    """
    def __init__(self, grid_size=0.5, goal_tolerance=0.2):
        self.grid_size = grid_size
        self.goal_tolerance = goal_tolerance

    def plan_next_step(self, current_pose, goal, obstacle_corridor):
        """
        Receives:
            current_pose: (x, y, theta)
            goal: (gx, gy)
            obstacle_corridor: dict with 'left_blocked', 'center_blocked', 'right_blocked'
        Returns:
            recommended_action: 'FORWARD', 'TURN_LEFT', 'TURN_RIGHT', 'REVERSE', 'STOP'
            distance_to_goal: float
        """
        cx, cy, ctheta = current_pose
        gx, gy = goal

        dx = gx - cx
        dy = gy - cy
        dist = math.hypot(dx, dy)

        if dist <= self.goal_tolerance:
            return "STOP", dist

        # Desired heading directly toward goal
        target_theta = math.atan2(dy, dx)
        heading_diff = normalize_angle(target_theta - ctheta)

        left_blocked = obstacle_corridor.get("left_blocked", False)
        center_blocked = obstacle_corridor.get("center_blocked", False)
        right_blocked = obstacle_corridor.get("right_blocked", False)

        # Check if direct path is clear
        if not center_blocked:
            if abs(heading_diff) < math.radians(15.0):
                return "FORWARD", dist
            elif heading_diff > 0:
                return "TURN_LEFT", dist
            else:
                return "TURN_RIGHT", dist
        
        # Local Obstacle Avoidance Rerouting
        if not right_blocked and heading_diff <= 0:
            return "TURN_RIGHT", dist
        elif not left_blocked:
            return "TURN_LEFT", dist
        elif not right_blocked:
            return "TURN_RIGHT", dist
        else:
            return "REVERSE", dist

    def astar_grid_plan(self, start_grid, goal_grid, obstacle_grid_set):
        """
        Grid-based A* Search Algorithm for global waypoint recovery.
        start_grid: (gx0, gy0)
        goal_grid: (gx1, gy1)
        obstacle_grid_set: set of blocked grid tuples {(x,y), ...}
        """
        def heuristic(a, b):
            return math.hypot(b[0] - a[0], b[1] - a[1])

        open_set = []
        heapq.heappush(open_set, (0 + heuristic(start_grid, goal_grid), 0, start_grid, [start_grid]))
        visited = set()

        while open_set:
            f, g, current, path = heapq.heappop(open_set)

            if current in visited:
                continue
            visited.add(current)

            if current == goal_grid:
                return path

            # 4-connected neighbors
            neighbors = [
                (current[0] + 1, current[1]),
                (current[0] - 1, current[1]),
                (current[0], current[1] + 1),
                (current[0], current[1] - 1)
            ]

            for neighbor in neighbors:
                if neighbor not in obstacle_grid_set and neighbor not in visited:
                    new_g = g + 1
                    new_f = new_g + heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (new_f, new_g, neighbor, path + [neighbor]))

        # Return direct line path if no obstacle-free grid path found
        return [start_grid, goal_grid]
