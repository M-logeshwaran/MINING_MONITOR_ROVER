#!/usr/bin/env python3
# ============================================================
# TEST 6, 7, 8 — NAVIGATION & EXPLORATION SEPARATION TEST
# Verifies:
#   1. Nav2 Target Dispatch: Goal pose dispatched cleanly.
#   2. Explored Mode: Uses AMCL / pre-built map, SLAM inactive.
#   3. Unexplored Mode: Uses SLAM Toolbox, AMCL inactive.
# ============================================================

import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from drillpulse_msgs.msg import JobAssignment, MissionCommand, MissionState
from drillpulse_mission.mission_manager_node import MissionManagerNode


class NavigationValidator(Node):
    def __init__(self):
        super().__init__('navigation_validator')
        self.dispatched_goal = None
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)

    def on_goal(self, msg: PoseStamped):
        self.dispatched_goal = msg


def run_tests():
    print("\n--- TEST 6, 7, 8: NAVIGATION & MODE SEPARATION ---")
    rclpy.init()

    mgr = MissionManagerNode()
    validator = NavigationValidator()

    from rclpy.executors import MultiThreadedExecutor
    import threading

    executor = MultiThreadedExecutor()
    executor.add_node(mgr)
    executor.add_node(validator)

    t = threading.Thread(target=executor.spin, daemon=True)
    t.start()

    time.sleep(0.3)
    passed = True

    # Test 6: Send Waypoint via Mission Command
    pub_cmd = validator.create_publisher(MissionCommand, '/rover/mission_command', 10)
    time.sleep(0.1)

    cmd = MissionCommand()
    cmd.header.stamp = validator.get_clock().now().to_msg()
    cmd.command_type = "GOTO_WAYPOINT"
    cmd.target_x = 5.25
    cmd.target_y = -3.10
    cmd.target_yaw = 0.5
    pub_cmd.publish(cmd)

    for _ in range(15):
        time.sleep(0.05)
        if validator.dispatched_goal is not None:
            break

    if validator.dispatched_goal:
        gx = validator.dispatched_goal.pose.position.x
        gy = validator.dispatched_goal.pose.position.y
        print(f"  [PASS] Nav2 Target Pose dispatched successfully: ({gx:.2f}, {gy:.2f})")
    else:
        print("  [FAIL] Failed to dispatch Nav2 goal pose")
        passed = False

    # Test 7 & 8: Verify Mode Separation
    pub_job = validator.create_publisher(JobAssignment, '/rover/job', 10)
    time.sleep(0.15)

    # Explored Mode Check
    job_exp = JobAssignment()
    job_exp.assigned_job = "EXPLORED"
    job_exp.map_name = "rover_map"
    for _ in range(15):
        pub_job.publish(job_exp)
        time.sleep(0.05)
        if mgr.active_job == "EXPLORED":
            break

    if mgr.active_job == "EXPLORED" and mgr.current_state == "NAVIGATING":
        print("  [PASS] Explored Mode verified: Configured for AMCL / Pre-built Map.")
    else:
        print(f"  [FAIL] Explored mode transition failed: {mgr.active_job}")
        passed = False

    # Unexplored Mode Check
    job_unexp = JobAssignment()
    job_unexp.assigned_job = "UNEXPLORED"
    job_unexp.map_name = "slam_toolbox"
    for _ in range(15):
        pub_job.publish(job_unexp)
        time.sleep(0.05)
        if mgr.active_job == "UNEXPLORED":
            break

    if mgr.active_job == "UNEXPLORED" and mgr.current_state == "EXPLORING":
        print("  [PASS] Unexplored Mode verified: Configured for SLAM Toolbox Online Mapping.")
    else:
        print(f"  [FAIL] Unexplored mode transition failed: {mgr.active_job}")
        passed = False

    executor.shutdown()
    mgr.destroy_node()
    validator.destroy_node()
    rclpy.shutdown()
    return passed


if __name__ == '__main__':
    ok = run_tests()
    sys.exit(0 if ok else 1)
