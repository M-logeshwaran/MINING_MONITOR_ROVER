import time
from drillpulse_autonomous.utils import log_colored

class NavigationManager:
    """
    High-level central coordinator for autonomous rover operations.
    Integrates SafetyManager, FreeRoamMode, PointNavigationMode, VisionManager,
    UltrasonicManager, MotorManager, and StateMachine.
    """
    def __init__(self, node, mode_str="free", goal_x=0.0, goal_y=0.0,
                 vision_manager=None, ultrasonic_manager=None, safety_manager=None,
                 motor_manager=None, coordinate_manager=None, path_planner=None,
                 state_machine=None, free_roam_mode=None, point_nav_mode=None):
        self.node = node
        self.mode = mode_str.lower()
        self.goal_x = goal_x
        self.goal_y = goal_y
        
        self.vision_mgr = vision_manager
        self.ultrasonic_mgr = ultrasonic_manager
        self.safety_mgr = safety_manager
        self.motor_mgr = motor_manager
        self.coord_mgr = coordinate_manager
        self.planner = path_planner
        self.state_machine = state_machine
        self.free_roam = free_roam_mode
        self.point_nav = point_nav_mode
        
        self.last_tick_time = time.monotonic()

        if self.mode == "point":
            self.coord_mgr.set_goal(self.goal_x, self.goal_y)
            self.state_machine.set_state("POINT_NAVIGATION")
        else:
            self.state_machine.set_state("FREE_ROAM")

    def tick(self):
        """
        Executes one iteration of the autonomous control loop.
        Priority Hierarchy:
        1. Failsafe & Emergency Stop
        2. Ultrasonic Safety Manager (Critical Stop, Left/Right Warning)
        3. Active Navigation Mode (Free Roam or Point-to-Point)
        4. Motor Bridge Command Output
        """
        now = time.monotonic()
        dt = max(0.01, now - self.last_tick_time)
        self.last_tick_time = now

        # Get Ultrasonic distances
        left_dist, right_dist, sensors_ok = self.ultrasonic_mgr.get_distances()

        # Update Dead-Reckoning Pose
        self.coord_mgr.update_dead_reckoning(self.motor_mgr.last_command or "STOP", dt)

        # Prepare Goal Info String for HUD
        goal_info_str = ""
        if self.mode == "point" and self.coord_mgr.has_goal:
            dist_g = self.coord_mgr.get_distance_to_goal()
            goal_info_str = f"Target: ({self.coord_mgr.goal_x:.1f}, {self.coord_mgr.goal_y:.1f}) | Dist: {dist_g:.2f}m"

        # Process Camera & OpenCV Vision
        _, obstacles, vision_rec_dir, corridor_info, camera_ok = self.vision_mgr.process_latest_frame(
            left_dist=left_dist,
            right_dist=right_dist,
            state_str=self.state_machine.current_state,
            mode_str=self.mode.upper(),
            speed_val=self.motor_mgr.current_speed_mode,
            goal_info=goal_info_str
        )

        # -------------------------------------------------------------
        # STEP 1: SAFETY MANAGER EVALUATION (HIGHEST PRIORITY OVERRIDE)
        # -------------------------------------------------------------
        override_active, safety_action, safety_state = self.safety_mgr.evaluate_safety(
            vision_rec_dir=vision_rec_dir,
            camera_ok=camera_ok
        )

        if override_active:
            self.state_machine.set_state(safety_state)
            self.motor_mgr.send_movement(safety_action)
            return

        # Check if Goal Reached state
        if self.state_machine.current_state == "GOAL_REACHED":
            self.motor_mgr.send_movement("STOP")
            return

        # -------------------------------------------------------------
        # STEP 2: MODE-SPECIFIC NAVIGATION EXECUTION
        # -------------------------------------------------------------
        if self.mode == "point":
            self.state_machine.set_state("POINT_NAVIGATION")
            action, goal_reached, dist_g = self.point_nav.compute_step(left_dist, right_dist, vision_rec_dir, corridor_info)
            
            if goal_reached:
                self.state_machine.set_state("GOAL_REACHED")
                self.motor_mgr.send_movement("STOP", force=True)
            else:
                self.motor_mgr.send_movement(action)

        else: # Default: Free Roam Mode
            self.state_machine.set_state("FREE_ROAM")
            action, sub_state = self.free_roam.compute_step(left_dist, right_dist, vision_rec_dir, corridor_info)
            self.motor_mgr.send_movement(action)
