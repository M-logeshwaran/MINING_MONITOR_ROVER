import sys
import time
import argparse
from drillpulse_autonomous.utils import log_colored

class PointNavigationMode:
    """
    Mode 2: Point-to-Point Autonomous Navigation controller.
    Drives rover to specified (goal_x, goal_y) target coordinates while dynamically avoiding obstacles.
    """
    def __init__(self, coordinate_manager, path_planner, node=None, goal_tolerance=0.2):
        self.coord_mgr = coordinate_manager
        self.planner = path_planner
        self.node = node
        self.goal_tolerance = goal_tolerance

    def set_goal(self, gx: float, gy: float):
        """Sets the destination goal coordinates."""
        self.coord_mgr.set_goal(gx, gy)

    def compute_step(self, left_dist, right_dist, vision_rec_dir, corridor_info):
        """
        Computes the next movement command toward target goal with local obstacle avoidance.
        Returns:
            action (str): 'FORWARD', 'TURN_LEFT', 'TURN_RIGHT', 'REVERSE', 'STOP'
            goal_reached (bool): True if arrived at target destination
            dist_to_goal (float): Remaining distance to goal
        """
        if not self.coord_mgr.has_goal:
            return "STOP", False, 0.0

        current_pose = (self.coord_mgr.current_x, self.coord_mgr.current_y, self.coord_mgr.current_theta)
        goal = (self.coord_mgr.goal_x, self.coord_mgr.goal_y)

        # Plan next step via local PathPlanner
        planned_action, dist_to_goal = self.planner.plan_next_step(current_pose, goal, corridor_info)

        # Check if destination reached
        if dist_to_goal <= self.goal_tolerance:
            log_colored("NAV", f"==================================================", getattr(self.node, 'get_logger', lambda: None)())
            log_colored("NAV", f"  GOAL REACHED! Destination (X={goal[0]:.2f}, Y={goal[1]:.2f}) Arrival Verified!", getattr(self.node, 'get_logger', lambda: None)())
            log_colored("NAV", f"==================================================", getattr(self.node, 'get_logger', lambda: None)())
            return "STOP", True, dist_to_goal

        return planned_action, False, dist_to_goal

def main_cli():
    """
    CLI Entry Point for terminal execution:
    ros2 run drillpulse_autonomous point_navigation --goal 5 8
    """
    parser = argparse.ArgumentParser(description="DrillPulse Point to Point Autonomous CLI")
    parser.add_argument("--goal", nargs=2, type=float, metavar=("X", "Y"), help="Goal coordinates: --goal X Y (e.g. --goal 5 8)")
    parser.add_argument("--name", type=str, help="Saved goal waypoint name (e.g. --name Lab)")
    
    args, unknown = parser.parse_known_args()

    import rclpy
    from drillpulse_autonomous.autonomous_controller import AutonomousControllerNode

    rclpy.init()

    gx = args.goal[0] if args.goal else 0.0
    gy = args.goal[1] if args.goal else 0.0

    # Initialize CoordinateManager to resolve --name waypoint if provided
    from drillpulse_autonomous.coordinate_manager import CoordinateManager
    temp_cm = CoordinateManager()
    if args.name and temp_cm.set_goal_by_name(args.name):
        gx = temp_cm.goal_x
        gy = temp_cm.goal_y

    node = AutonomousControllerNode(initial_mode="point", goal_x=gx, goal_y=gy)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main_cli()
