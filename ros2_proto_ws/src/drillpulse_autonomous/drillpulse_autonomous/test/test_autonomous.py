import sys
import os
import time
import unittest
import numpy as np

# Mock ROS modules if running in standard python without ROS2 environment sourced
try:
    from std_msgs.msg import Float32, String, Int32
except ImportError:
    import types
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    class Float32: data = 0.0
    class String: data = ""
    class Int32: data = 0
    std_msgs_msg.Float32 = Float32
    std_msgs_msg.String = String
    std_msgs_msg.Int32 = Int32
    std_msgs.msg = std_msgs_msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg

from drillpulse_autonomous.utils import normalize_angle, euclidean_distance
from drillpulse_autonomous.safety_manager import SafetyManager
from drillpulse_autonomous.ultrasonic_manager import UltrasonicManager
from drillpulse_autonomous.motor_manager import MotorManager
from drillpulse_autonomous.coordinate_manager import CoordinateManager
from drillpulse_autonomous.path_planner import PathPlanner
from drillpulse_autonomous.state_machine import StateMachine
from drillpulse_autonomous.obstacle_detector import ObstacleDetector

class MockNode:
    """Mock ROS2 Node for testing managers without spinning ROS."""
    def get_logger(self):
        return self
    def info(self, msg): pass
    def warn(self, msg): pass
    def warning(self, msg): pass
    def create_publisher(self, msg_type, topic, qos):
        return MockPublisher()

class MockPublisher:
    def publish(self, msg): pass

class TestDrillPulseAutonomous(unittest.TestCase):

    def test_safety_manager_critical_stop(self):
        mock_node = MockNode()
        us_mgr = UltrasonicManager(node=None, simulation=True)
        us_mgr.latest_left_stable = 10.0  # Critical < 15cm
        us_mgr.latest_right_stable = 50.0
        us_mgr.left_connected = True
        us_mgr.right_connected = True
        us_mgr.last_left_time = time.monotonic()
        us_mgr.last_right_time = time.monotonic()
        
        safety_mgr = SafetyManager(node=mock_node, ultrasonic_manager=us_mgr)
        override, action, state = safety_mgr.evaluate_safety(vision_rec_dir="CENTER", camera_ok=True)
        
        self.assertTrue(override)
        self.assertEqual(action, "REVERSE")
        self.assertEqual(state, "RECOVERY")

    def test_safety_manager_warning_steer(self):
        mock_node = MockNode()
        us_mgr = UltrasonicManager(node=None, simulation=True)
        us_mgr.latest_left_stable = 20.0  # Warning < 25cm
        us_mgr.latest_right_stable = 50.0
        us_mgr.left_connected = True
        us_mgr.right_connected = True
        us_mgr.last_left_time = time.monotonic()
        us_mgr.last_right_time = time.monotonic()
        
        safety_mgr = SafetyManager(node=mock_node, ultrasonic_manager=us_mgr)
        override, action, state = safety_mgr.evaluate_safety(vision_rec_dir="CENTER", camera_ok=True)
        
        self.assertTrue(override)
        self.assertEqual(action, "TURN_RIGHT")
        self.assertEqual(state, "OBSTACLE_AVOIDANCE")

    def test_motor_manager_slow_command(self):
        mock_node = MockNode()
        motor_mgr = MotorManager(node=mock_node)
        
        motor_mgr.send_movement("SLOW")
        self.assertEqual(motor_mgr.current_speed_mode, 1)
        self.assertEqual(motor_mgr.last_command, "CMD,0,1")

    def test_coordinate_manager_waypoints(self):
        coord_mgr = CoordinateManager()
        self.assertIn("Lab", coord_mgr.saved_waypoints)
        
        success = coord_mgr.set_goal_by_name("Lab")
        self.assertTrue(success)
        self.assertEqual(coord_mgr.goal_x, 5.0)
        self.assertEqual(coord_mgr.goal_y, 8.0)
        self.assertGreater(coord_mgr.get_distance_to_goal(), 0)

    def test_path_planner(self):
        planner = PathPlanner()
        corridor = {"left_blocked": False, "center_blocked": False, "right_blocked": False}
        
        action, dist = planner.plan_next_step((0, 0, 0), (5, 0), corridor)
        self.assertEqual(action, "FORWARD")
        self.assertEqual(dist, 5.0)

    def test_state_machine_transitions(self):
        sm = StateMachine()
        self.assertEqual(sm.current_state, "IDLE")
        
        self.assertTrue(sm.set_state("FREE_ROAM"))
        self.assertEqual(sm.current_state, "FREE_ROAM")
        
        self.assertFalse(sm.set_state("INVALID_STATE"))
        self.assertEqual(sm.current_state, "FREE_ROAM")

    def test_obstacle_detector(self):
        detector = ObstacleDetector()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        
        annotated, obstacles, rec_dir, corridor = detector.process_frame(
            frame=frame,
            left_dist=50.0,
            right_dist=50.0,
            state_str="FREE_ROAM",
            mode_str="FREE",
            speed_val=1
        )
        
        self.assertIsNotNone(annotated)
        self.assertEqual(rec_dir, "CENTER")
        self.assertEqual(corridor["confidence"], 1.0)

if __name__ == "__main__":
    unittest.main()
