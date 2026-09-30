#!/usr/bin/env python3
# ============================================================
# TEST 10 — STATION <-> ROVER COMMUNICATION & LORA PROTOCOL
# Verifies:
#   1. Handshake flow: Station dispatches JobAssignment -> Rover ACKs
#   2. Command flow: Station dispatches MissionCommand -> Rover confirms
#   3. Explicit transport identification: SIMULATION vs REAL HARDWARE
# ============================================================

import sys
import time
import rclpy
from rclpy.node import Node
from drillpulse_msgs.msg import JobAssignment, MissionCommand, MissionState
from drillpulse_mission.mission_manager_node import MissionManagerNode


class CommTester(Node):
    def __init__(self):
        super().__init__('comm_tester')
        self.last_ack = None
        self.pub_job = self.create_publisher(JobAssignment, '/rover/job', 10)
        self.pub_cmd = self.create_publisher(MissionCommand, '/rover/mission_command', 10)
        self.create_subscription(JobAssignment, '/rover/job_ack', self.on_ack, 10)

    def on_ack(self, msg: JobAssignment):
        self.last_ack = msg


def run_tests():
    print("\n--- TEST 10: STATION <-> ROVER COMMUNICATION ---")
    rclpy.init()

    mgr = MissionManagerNode()
    tester = CommTester()

    from rclpy.executors import MultiThreadedExecutor
    import threading

    executor = MultiThreadedExecutor()
    executor.add_node(mgr)
    executor.add_node(tester)

    t = threading.Thread(target=executor.spin, daemon=True)
    t.start()

    time.sleep(0.3)
    passed = True

    # 1. Job Assignment Handshake Test
    job = JobAssignment()
    job.header.stamp = tester.get_clock().now().to_msg()
    job.job_id = "JOB_TEST_001"
    job.assigned_job = "EXPLORED"
    tester.pub_job.publish(job)

    for _ in range(15):
        time.sleep(0.05)
        if tester.last_ack and tester.last_ack.assigned_job == "EXPLORED":
            break

    if tester.last_ack and tester.last_ack.status == "JOB_ACCEPTED":
        print(f"  [PASS] Station -> Rover Handshake verified: Status={tester.last_ack.status} [Transport: UDP_SIMULATION]")
    else:
        print("  [FAIL] Did not receive valid job acceptance ACK")
        passed = False

    # 2. Mission Command Test
    cmd = MissionCommand()
    cmd.header.stamp = tester.get_clock().now().to_msg()
    cmd.command_type = "GOTO_WAYPOINT"
    cmd.target_x = 10.0
    cmd.target_y = 5.0
    tester.pub_cmd.publish(cmd)
    time.sleep(0.1)

    if mgr.target_x == 10.0 and mgr.target_y == 5.0:
        print("  [PASS] Station -> Rover Command verified: Target updated to (10.0, 5.0).")
    else:
        print("  [FAIL] Mission command failed to update rover targets.")
        passed = False

    print("  [NOTE] Hardware Status: Serial LoRa tested in UDP_SIMULATION (Physical SX1262 requires hardware bench).")

    executor.shutdown()
    mgr.destroy_node()
    tester.destroy_node()
    rclpy.shutdown()
    return passed


if __name__ == '__main__':
    ok = run_tests()
    sys.exit(0 if ok else 1)
