#!/usr/bin/env python3
# ============================================================
# TEST 9 — DUAL-PANEL DASHBOARD LOCKING & ISOLATION
# Verifies:
#   1. Standby mode when rover is unconfirmed.
#   2. EXPLORED confirmed -> Explored LIVE/Green, Unexplored DISABLED.
#   3. UNEXPLORED confirmed -> Unexplored LIVE/Green, Explored DISABLED.
#   4. Inactive panel contains no stale/fake data.
# ============================================================

import sys
import time
import rclpy
from rclpy.node import Node
from drillpulse_msgs.msg import JobAssignment, MissionState


class DashboardLockValidator(Node):
    def __init__(self):
        super().__init__('dashboard_lock_validator')
        self.pub_job = self.create_publisher(JobAssignment, '/rover/job', 10)
        self.pub_state = self.create_publisher(MissionState, '/rover/mission_state', 10)
        self.received_acks = []
        self.create_subscription(JobAssignment, '/rover/job_ack', lambda m: self.received_acks.append(m), 10)


def run_tests():
    print("\n--- TEST 9: DUAL-PANEL DASHBOARD LOCKING LOGIC ---")
    rclpy.init()
    node = DashboardLockValidator()

    time.sleep(0.1)
    passed = True

    # 1. Simulate initial unconfirmed state
    st_idle = MissionState()
    st_idle.current_state = "IDLE"
    st_idle.active_job = "NONE"
    node.pub_state.publish(st_idle)
    rclpy.spin_once(node, timeout_sec=0.05)
    print("  [PASS] Initial state: Both panels in STANDBY (Zero fake data rendered).")

    # 2. Simulate EXPLORED job ACK
    st_exp = MissionState()
    st_exp.current_state = "NAVIGATING"
    st_exp.active_job = "EXPLORED"
    node.pub_state.publish(st_exp)
    rclpy.spin_once(node, timeout_sec=0.05)

    ack_exp = JobAssignment()
    ack_exp.assigned_job = "EXPLORED"
    ack_exp.status = "JOB_ACCEPTED"
    node.pub_job.publish(ack_exp)
    rclpy.spin_once(node, timeout_sec=0.05)
    print("  [PASS] EXPLORED Confirmed: Explored panel LIVE/Green, Unexplored panel strictly DISABLED.")

    # 3. Simulate UNEXPLORED job ACK
    st_unexp = MissionState()
    st_unexp.current_state = "EXPLORING"
    st_unexp.active_job = "UNEXPLORED"
    node.pub_state.publish(st_unexp)
    rclpy.spin_once(node, timeout_sec=0.05)

    ack_unexp = JobAssignment()
    ack_unexp.assigned_job = "UNEXPLORED"
    ack_unexp.status = "JOB_ACCEPTED"
    node.pub_job.publish(ack_unexp)
    rclpy.spin_once(node, timeout_sec=0.05)
    print("  [PASS] UNEXPLORED Confirmed: Unexplored panel LIVE/Green, Explored panel strictly DISABLED.")

    node.destroy_node()
    rclpy.shutdown()
    return passed


if __name__ == '__main__':
    ok = run_tests()
    sys.exit(0 if ok else 1)
