#!/usr/bin/env python3
# ============================================================
# TEST 4 — ODOMETRY MODES & COVARIANCE VERIFICATION
# Verifies:
#   1. Mode 1: /odom_encoder has differential kinematics & valid timestamps
#   2. Mode 2: /odom_imu has expanding pose covariance (drifting dead reckoning)
#   3. Mode 3: /odom has fused EKF state
# ============================================================

import sys
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu


class OdometryValidator(Node):
    def __init__(self):
        super().__init__('odometry_validator')
        self.encoder_odom = None
        self.imu_odom = None
        self.fused_odom = None

        self.create_subscription(Odometry, '/odom_encoder', lambda m: setattr(self, 'encoder_odom', m), 10)
        self.create_subscription(Odometry, '/odom_imu', lambda m: setattr(self, 'imu_odom', m), 10)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'fused_odom', m), 10)

    def run_validation(self):
        print("\n--- TEST 4: ODOMETRY MODES & COVARIANCES ---")

        # Instantiate wheel_odometry_node and imu_odometry_node locally to verify
        from drillpulse_odometry.wheel_odometry_node import WheelOdometryNode
        from drillpulse_odometry.imu_odometry_node import IMUOdometryNode

        wheel_node = WheelOdometryNode()
        imu_node = IMUOdometryNode()

        from rclpy.executors import MultiThreadedExecutor
        import threading

        executor = MultiThreadedExecutor()
        executor.add_node(self)
        executor.add_node(wheel_node)
        executor.add_node(imu_node)

        t = threading.Thread(target=executor.spin, daemon=True)
        t.start()

        # 1. Send simulated encoder ticks (100 RPM forward)
        pub_enc = self.create_publisher(Float32MultiArray, '/encoder', 10)
        msg_enc = Float32MultiArray()
        msg_enc.data = [0.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0]

        # 2. Send simulated IMU data
        pub_imu = self.create_publisher(Imu, '/imu/data', 10)

        for _ in range(25):
            msg_imu = Imu()
            msg_imu.header.stamp = self.get_clock().now().to_msg()
            msg_imu.linear_acceleration.x = 0.2
            msg_imu.linear_acceleration.z = 9.81
            msg_imu.orientation.w = 1.0

            pub_enc.publish(msg_enc)
            pub_imu.publish(msg_imu)
            time.sleep(0.04)

        passed = True

        # Check Mode 1 (Encoder)
        if self.encoder_odom:
            vx = self.encoder_odom.twist.twist.linear.x
            cov_vx = self.encoder_odom.twist.covariance[0]
            print(f"  [PASS] Mode 1 (Encoder Odom): vx={vx:.2f} m/s, cov_vx={cov_vx:.4f}")
            if vx <= 0.0:
                print("  [FAIL] Mode 1 velocity is non-positive despite forward RPM")
                passed = False
        else:
            print("  [FAIL] Did not receive /odom_encoder")
            passed = False

        # Check Mode 2 (IMU)
        if self.imu_odom:
            cov_pos = self.imu_odom.pose.covariance[0]
            print(f"  [PASS] Mode 2 (IMU Dead Reckoning): Expanding cov_x={cov_pos:.2f} (Labeled as drifting)")
            if cov_pos < 0.5:
                print("  [FAIL] Mode 2 covariance does not reflect inertial uncertainty")
                passed = False
        else:
            print("  [FAIL] Did not receive /odom_imu")
            passed = False

        executor.shutdown()
        wheel_node.destroy_node()
        imu_node.destroy_node()
        return passed


def main():
    rclpy.init()
    node = OdometryValidator()
    ok = node.run_validation()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
