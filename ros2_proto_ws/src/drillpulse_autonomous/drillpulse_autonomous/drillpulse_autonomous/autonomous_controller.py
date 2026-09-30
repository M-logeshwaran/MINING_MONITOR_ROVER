#!/usr/bin/env python3
"""
DRILLPULSE AUTONOMOUS CONTROLLER NODE
=====================================
Central ROS2 node initializing all sub-managers and running the main control loop.
"""

import sys
import rclpy
from rclpy.node import Node

from drillpulse_autonomous.utils import log_colored
from drillpulse_autonomous.camera_stream import CameraStream
from drillpulse_autonomous.ultrasonic_manager import UltrasonicManager
from drillpulse_autonomous.obstacle_detector import ObstacleDetector
from drillpulse_autonomous.vision_manager import VisionManager
from drillpulse_autonomous.safety_manager import SafetyManager
from drillpulse_autonomous.motor_manager import MotorManager
from drillpulse_autonomous.coordinate_manager import CoordinateManager
from drillpulse_autonomous.path_planner import PathPlanner
from drillpulse_autonomous.state_machine import StateMachine
from drillpulse_autonomous.free_roam_mode import FreeRoamMode
from drillpulse_autonomous.point_navigation_mode import PointNavigationMode
from drillpulse_autonomous.navigation_manager import NavigationManager

class AutonomousControllerNode(Node):
    def __init__(self, initial_mode="free", goal_x=0.0, goal_y=0.0):
        super().__init__("drillpulse_autonomous_node")

        log_colored("AUTO", "==================================================", self.get_logger())
        log_colored("AUTO", "   DRILLPULSE AUTONOMOUS SYSTEM INITIALIZING", self.get_logger())
        log_colored("AUTO", "==================================================", self.get_logger())

        # Declare ROS 2 Parameters with Defaults
        self.declare_parameter("mode", initial_mode)
        self.declare_parameter("goal_x", float(goal_x))
        self.declare_parameter("goal_y", float(goal_y))
        self.declare_parameter("simulation", True)
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("left_ultrasonic_topic", "/left_distance")
        self.declare_parameter("right_ultrasonic_topic", "/right_distance")
        self.declare_parameter("motor_command_topic", "/rover/command")
        self.declare_parameter("speed_mode_topic", "/speed_mode")
        self.declare_parameter("critical_stop_distance", 15.0)
        self.declare_parameter("warning_distance", 25.0)
        self.declare_parameter("safe_distance", 35.0)
        self.declare_parameter("canny_threshold_low", 50)
        self.declare_parameter("canny_threshold_high", 150)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("fps", 20)
        self.declare_parameter("webcam_id", 0)
        self.declare_parameter("navigation_points_file", "")

        # Retrieve Parameter Values
        mode_val = self.get_parameter("mode").value
        gx_val = float(self.get_parameter("goal_x").value)
        gy_val = float(self.get_parameter("goal_y").value)
        simulation_val = bool(self.get_parameter("simulation").value)
        cam_topic = self.get_parameter("camera_topic").value
        left_topic = self.get_parameter("left_ultrasonic_topic").value
        right_topic = self.get_parameter("right_ultrasonic_topic").value
        motor_topic = self.get_parameter("motor_command_topic").value
        speed_topic = self.get_parameter("speed_mode_topic").value
        nav_points_file = self.get_parameter("navigation_points_file").value

        crit_stop = float(self.get_parameter("critical_stop_distance").value)
        warn_dist = float(self.get_parameter("warning_distance").value)
        safe_dist = float(self.get_parameter("safe_distance").value)
        c_low = int(self.get_parameter("canny_threshold_low").value)
        c_high = int(self.get_parameter("canny_threshold_high").value)
        width = int(self.get_parameter("frame_width").value)
        height = int(self.get_parameter("frame_height").value)
        fps_val = int(self.get_parameter("fps").value)
        webcam = int(self.get_parameter("webcam_id").value)

        log_colored("AUTO", f"Configuration: Mode={mode_val.upper()}, Goal=({gx_val}, {gy_val}), Simulation={simulation_val}", self.get_logger())

        # Initialize Managers
        self.ultrasonic_mgr = UltrasonicManager(
            node=self,
            left_topic=left_topic,
            right_topic=right_topic,
            critical_stop=crit_stop,
            warning_dist=warn_dist,
            safe_dist=safe_dist,
            simulation=simulation_val
        )

        self.vision_mgr = VisionManager(
            node=self,
            topic=cam_topic,
            width=width,
            height=height,
            fps=fps_val,
            canny_low=c_low,
            canny_high=c_high,
            simulation=simulation_val,
            webcam_id=webcam
        )

        self.safety_mgr = SafetyManager(
            node=self,
            ultrasonic_manager=self.ultrasonic_mgr,
            critical_stop=crit_stop,
            warning_dist=warn_dist,
            safe_dist=safe_dist
        )

        self.motor_mgr = MotorManager(
            node=self,
            topic=motor_topic,
            speed_topic=speed_topic
        )

        self.coord_mgr = CoordinateManager(node=self, nav_points_file=nav_points_file)
        self.planner = PathPlanner()
        self.state_machine = StateMachine(node=self)
        self.free_roam_mode = FreeRoamMode(node=self)
        self.point_nav_mode = PointNavigationMode(self.coord_mgr, self.planner, node=self)

        self.nav_mgr = NavigationManager(
            node=self,
            mode_str=mode_val,
            goal_x=gx_val,
            goal_y=gy_val,
            vision_manager=self.vision_mgr,
            ultrasonic_manager=self.ultrasonic_mgr,
            safety_manager=self.safety_mgr,
            motor_manager=self.motor_mgr,
            coordinate_manager=self.coord_mgr,
            path_planner=self.planner,
            state_machine=self.state_machine,
            free_roam_mode=self.free_roam_mode,
            point_nav_mode=self.point_nav_mode
        )

        # Main Autonomous Loop Timer (20 Hz)
        timer_period = 1.0 / fps_val
        self.timer = self.create_timer(timer_period, self._control_loop_callback)
        log_colored("AUTO", "DrillPulse Autonomous Node fully initialized & READY!", self.get_logger())

    def _control_loop_callback(self):
        try:
            self.nav_mgr.tick()
        except Exception as e:
            log_colored("AUTO", f"Control Loop Exception: {e}", self.get_logger())

    def destroy_node(self):
        log_colored("AUTO", "Shutting down DrillPulse Autonomous Node...", self.get_logger())
        if hasattr(self, "motor_mgr"):
            self.motor_mgr.emergency_stop()
        if hasattr(self, "vision_mgr"):
            self.vision_mgr.destroy_windows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
