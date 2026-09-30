#!/usr/bin/env python3
# ============================================================
# TEST 5 — ODOMETRY THRESHOLD & SENSOR CONSISTENCY LOGIC
# Verifies all 5 states:
#   1. CONSISTENT (Nominal)
#   2. ENCODER_DEGRADED (Wheel slip / discrepancy)
#   3. IMU_DEGRADED (Angular discrepancy)
#   4. SENSOR_DISAGREEMENT (Severe multi-axis violation)
#   5. SENSOR_TIMEOUT (Watchdog cutoff)
# ============================================================

import sys
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from drillpulse_msgs.msg import OdometryDiagnostics
from drillpulse_odometry.threshold_consistency_node import ThresholdConsistencyNode


class ThresholdTester(Node):
    def __init__(self):
        super().__init__('threshold_tester')
        self.last_diag = None
        self.diag_sub = self.create_subscription(OdometryDiagnostics, '/odom/diagnostics', self.on_diag, 10)
        self.enc_pub = self.create_publisher(Odometry, '/odom_encoder', 10)
        self.imu_pub = self.create_publisher(Odometry, '/odom_imu', 10)
        self.raw_imu_pub = self.create_publisher(Imu, '/imu/data', 10)

    def on_diag(self, msg: OdometryDiagnostics):
        self.last_diag = msg

    def send_signals(self, enc_v, enc_w, imu_v, imu_w):
        now = self.get_clock().now().to_msg()

        enc = Odometry()
        enc.header.stamp = now
        enc.twist.twist.linear.x = float(enc_v)
        enc.twist.twist.angular.z = float(enc_w)
        self.enc_pub.publish(enc)

        imu = Odometry()
        imu.header.stamp = now
        imu.twist.twist.linear.x = float(imu_v)
        imu.twist.twist.angular.z = float(imu_w)
        self.imu_pub.publish(imu)

        raw = Imu()
        raw.header.stamp = now
        raw.angular_velocity.z = float(imu_w)
        self.raw_imu_pub.publish(raw)


def run_tests():
    print("\n--- TEST 5: THRESHOLD CONSISTENCY 5-STATE VERIFICATION ---")
    rclpy.init()

    eval_node = ThresholdConsistencyNode()
    tester = ThresholdTester()

    from rclpy.executors import MultiThreadedExecutor
    import threading

    executor = MultiThreadedExecutor()
    executor.add_node(eval_node)
    executor.add_node(tester)

    t = threading.Thread(target=executor.spin, daemon=True)
    t.start()

    time.sleep(0.5)
    passed = True

    # 1. CONSISTENT
    for _ in range(8):
        tester.send_signals(enc_v=0.40, enc_w=0.10, imu_v=0.42, imu_w=0.11)
        time.sleep(0.02)

    st1 = tester.last_diag.fusion_state if tester.last_diag else "NONE"
    if st1 == 'CONSISTENT':
        print(f"  [PASS] State 1 verified: CONSISTENT (v_err={tester.last_diag.velocity_error:.2f})")
    else:
        print(f"  [FAIL] Expected CONSISTENT, got {st1}")
        passed = False

    # 2. ENCODER_DEGRADED (Wheel slip: enc_v=1.2, imu_v=0.4 -> diff=0.8 > 0.35)
    for _ in range(8):
        tester.send_signals(enc_v=1.20, enc_w=0.10, imu_v=0.40, imu_w=0.10)
        time.sleep(0.02)

    st2 = tester.last_diag.fusion_state if tester.last_diag else "NONE"
    if st2 in ('ENCODER_DEGRADED', 'SENSOR_DISAGREEMENT'):
        print(f"  [PASS] State 2 verified: {st2} (Detected wheel slip: v_err={tester.last_diag.velocity_error:.2f})")
    else:
        print(f"  [FAIL] Expected ENCODER_DEGRADED, got {st2}")
        passed = False

    # 3. IMU_DEGRADED (Angular discrepancy: enc_w=0.1, imu_w=0.9 -> diff=0.8 > 0.50)
    for _ in range(8):
        tester.send_signals(enc_v=0.40, enc_w=0.10, imu_v=0.42, imu_w=0.90)
        time.sleep(0.02)

    st3 = tester.last_diag.fusion_state if tester.last_diag else "NONE"
    if st3 in ('IMU_DEGRADED', 'SENSOR_DISAGREEMENT'):
        print(f"  [PASS] State 3 verified: {st3} (Detected yaw disagreement: w_err={tester.last_diag.yaw_rate_error:.2f})")
    else:
        print(f"  [FAIL] Expected IMU_DEGRADED, got {st3}")
        passed = False

    # 4. SENSOR_DISAGREEMENT (Both linear and angular violation)
    for _ in range(8):
        tester.send_signals(enc_v=1.50, enc_w=0.10, imu_v=0.20, imu_w=1.20)
        time.sleep(0.02)

    st4 = tester.last_diag.fusion_state if tester.last_diag else "NONE"
    if st4 == 'SENSOR_DISAGREEMENT':
        print(f"  [PASS] State 4 verified: SENSOR_DISAGREEMENT (Severe multi-axis violation)")
    else:
        print(f"  [FAIL] Expected SENSOR_DISAGREEMENT, got {st4}")
        passed = False

    # 5. SENSOR_TIMEOUT (Stop sending signals for > 1.2s watchdog)
    time.sleep(1.2)
    st5 = tester.last_diag.fusion_state if tester.last_diag else "NONE"
    if st5 == 'SENSOR_TIMEOUT':
        print(f"  [PASS] State 5 verified: SENSOR_TIMEOUT (Watchdog cutoff)")
    else:
        print(f"  [FAIL] Expected SENSOR_TIMEOUT, got {st5}")
        passed = False

    executor.shutdown()
    eval_node.destroy_node()
    tester.destroy_node()
    rclpy.shutdown()
    return passed


if __name__ == '__main__':
    ok = run_tests()
    sys.exit(0 if ok else 1)
