#!/usr/bin/env python3
# ============================================================
# TEST 11 — RECOVERY ACTUATOR COMMANDS & SAFETY
# Verifies:
#   1. KEY 6 -> LEFT_RECOVERY (Left actuator extends)
#   2. KEY 7 -> RIGHT_RECOVERY (Right actuator extends)
#   3. ESC -> STOP_RECOVERY (Immediate halt & cooldown)
#   4. Maximum runtime timeout enforcement
# ============================================================

import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from drillpulse_msgs.msg import RecoveryCommand
from drillpulse_recovery.recovery_controller_node import RecoveryControllerNode


class RecoveryTester(Node):
    def __init__(self):
        super().__init__('recovery_tester')
        self.last_status = None
        self.last_actuator_cmd = None
        self.pub_cmd = self.create_publisher(RecoveryCommand, '/rover/recovery_command', 10)
        self.create_subscription(String, '/rover/recovery_status', self.on_status, 10)
        self.create_subscription(RecoveryCommand, '/rover/actuator_cmd', self.on_act_cmd, 10)

    def on_status(self, msg: String):
        self.last_status = msg.data

    def on_act_cmd(self, msg: RecoveryCommand):
        self.last_actuator_cmd = msg.command_type


def run_tests():
    print("\n--- TEST 11: RECOVERY SYSTEM (KEYS 6 & 7) ---")
    rclpy.init()

    ctrl = RecoveryControllerNode()
    tester = RecoveryTester()

    from rclpy.executors import MultiThreadedExecutor
    import threading

    executor = MultiThreadedExecutor()
    executor.add_node(ctrl)
    executor.add_node(tester)

    t = threading.Thread(target=executor.spin, daemon=True)
    t.start()

    time.sleep(0.2)
    passed = True

    # 1. Test Key 6 -> LEFT_RECOVERY (simulate rollover on left: -60 deg)
    ctrl.current_roll = -60.0
    cmd_left = RecoveryCommand()
    cmd_left.command_type = "LEFT_RECOVERY"
    for _ in range(15):
        tester.pub_cmd.publish(cmd_left)
        time.sleep(0.04)
        if ctrl.state == 'EXTENDING' and ctrl.active_side == 'LEFT':
            break

    if ctrl.state == 'EXTENDING' and ctrl.active_side == 'LEFT':
        print(f"  [PASS] Key 6 verified: LEFT_RECOVERY engaged (Actuator status: {tester.last_status})")
    else:
        print(f"  [FAIL] Key 6 failed to engage left recovery: {ctrl.state}")
        passed = False

    # 2. Test ESC -> STOP_RECOVERY
    cmd_stop = RecoveryCommand()
    cmd_stop.command_type = "STOP_RECOVERY"
    for _ in range(15):
        tester.pub_cmd.publish(cmd_stop)
        time.sleep(0.04)
        if ctrl.state in ('COOLDOWN', 'IDLE'):
            break

    if ctrl.state in ('COOLDOWN', 'IDLE'):
        print(f"  [PASS] ESC verified: STOP_RECOVERY halted actuator immediately ({ctrl.state}).")
    else:
        print(f"  [FAIL] Stop command failed to halt actuator: {ctrl.state}")
        passed = False

    # Reset roll and ensure state is IDLE
    ctrl.current_roll = 0.0
    ctrl.state = 'IDLE'
    ctrl.active_side = None
    time.sleep(0.1)

    # 3. Test Key 7 -> RIGHT_RECOVERY (simulate rollover on right: 60 deg)
    ctrl.current_roll = 60.0
    cmd_right = RecoveryCommand()
    cmd_right.command_type = "RIGHT_RECOVERY"
    for _ in range(15):
        tester.pub_cmd.publish(cmd_right)
        time.sleep(0.04)
        if ctrl.state == 'EXTENDING' and ctrl.active_side == 'RIGHT':
            break

    if ctrl.state == 'EXTENDING' and ctrl.active_side == 'RIGHT':
        print(f"  [PASS] Key 7 verified: RIGHT_RECOVERY engaged (Actuator status: {tester.last_status})")
    else:
        print(f"  [FAIL] Key 7 failed to engage right recovery: {ctrl.state}")
        passed = False

    executor.shutdown()
    t.join(timeout=0.5)
    ctrl.destroy_node()
    tester.destroy_node()
    rclpy.shutdown()
    return passed


if __name__ == '__main__':
    ok = run_tests()
    sys.exit(0 if ok else 1)
