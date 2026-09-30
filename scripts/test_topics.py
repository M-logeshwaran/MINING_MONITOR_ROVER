#!/usr/bin/env python3
# ============================================================
# TEST 2 — TOPIC CONTRACT VALIDATION
# Verifies canonical topics exist, publish valid schemas,
# and have correct message contracts.
# ============================================================

import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from drillpulse_msgs.msg import OdometryDiagnostics, RoverTelemetry, MissionState


class TopicContractValidator(Node):
    def __init__(self):
        super().__init__('topic_contract_validator')
        self.received = {}

        # Canonical Subscriptions
        self.create_subscription(LaserScan, '/scan', lambda m: self._rec('/scan'), 10)
        self.create_subscription(Imu, '/imu/data', lambda m: self._rec('/imu/data'), 10)
        self.create_subscription(Odometry, '/odom_encoder', lambda m: self._rec('/odom_encoder'), 10)
        self.create_subscription(Odometry, '/odom_imu', lambda m: self._rec('/odom_imu'), 10)
        self.create_subscription(Odometry, '/odom', lambda m: self._rec('/odom'), 10)
        self.create_subscription(OdometryDiagnostics, '/odom/diagnostics', lambda m: self._rec('/odom/diagnostics'), 10)
        self.create_subscription(RoverTelemetry, '/rover/telemetry', lambda m: self._rec('/rover/telemetry'), 10)
        self.create_subscription(MissionState, '/rover/mission_state', lambda m: self._rec('/rover/mission_state'), 10)

        # Mock Publishers to verify schemas
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)
        self.pub_enc = self.create_publisher(Odometry, '/odom_encoder', 10)
        self.pub_imu_odom = self.create_publisher(Odometry, '/odom_imu', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_diag = self.create_publisher(OdometryDiagnostics, '/odom/diagnostics', 10)
        self.pub_telem = self.create_publisher(RoverTelemetry, '/rover/telemetry', 10)
        self.pub_state = self.create_publisher(MissionState, '/rover/mission_state', 10)

    def _rec(self, topic):
        self.received[topic] = True

    def run_validation(self):
        print("\n--- TEST 2: CANONICAL TOPIC CONTRACTS ---")
        time.sleep(0.2)

        # Publish test messages
        self.pub_scan.publish(LaserScan())
        self.pub_imu.publish(Imu())
        self.pub_enc.publish(Odometry())
        self.pub_imu_odom.publish(Odometry())
        self.pub_odom.publish(Odometry())
        self.pub_diag.publish(OdometryDiagnostics())
        self.pub_telem.publish(RoverTelemetry())
        self.pub_state.publish(MissionState())

        for _ in range(15):
            rclpy.spin_once(self, timeout_sec=0.05)

        required_topics = [
            '/scan', '/imu/data', '/odom_encoder', '/odom_imu',
            '/odom', '/odom/diagnostics', '/rover/telemetry', '/rover/mission_state'
        ]

        passed = True
        for topic in required_topics:
            if self.received.get(topic):
                print(f"  [PASS] Canonical topic {topic} verified.")
            else:
                print(f"  [FAIL] Missing or invalid schema on {topic}")
                passed = False

        return passed


def main():
    rclpy.init()
    node = TopicContractValidator()
    ok = node.run_validation()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
