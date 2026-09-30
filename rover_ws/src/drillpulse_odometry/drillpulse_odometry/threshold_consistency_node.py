#!/usr/bin/env python3
# ============================================================
# DRILLPULSE
#
# FILE:
#     threshold_consistency_node.py
#
# PURPOSE:
#     Cross-checks kinematic wheel odometry against inertial IMU
#     measurements. Calculates velocity, yaw rate, and yaw errors.
#     Monitors sensor freshness and asserts fusion gating states
#     to ensure corrupted or slipping sensor streams do not pollute
#     localization.
#
# INPUT:
#     /odom_encoder (nav_msgs/Odometry)
#     /odom_imu (nav_msgs/Odometry)
#     /sparkfun/imu/data (sensor_msgs/Imu)
#
# OUTPUT:
#     /odom/diagnostics (drillpulse_msgs/OdometryDiagnostics)
#
# TF OWNER:
#     None
#
# USER PARAMETERS:
#     See USER CONFIGURATION section below.
# ============================================================

# =========================================================
# USER CONFIGURATION
# =========================================================
DEFAULT_VELOCITY_DIFF_THRESHOLD = 0.35    # m/s
DEFAULT_YAW_RATE_DIFF_THRESHOLD = 0.30    # rad/s
DEFAULT_YAW_DIFF_THRESHOLD = 0.40         # rad (~23 degrees)
DEFAULT_IMU_TIMEOUT = 0.50                # seconds
DEFAULT_ENCODER_TIMEOUT = 0.50            # seconds
DEFAULT_MAX_ACCELERATION = 2.50           # m/s^2
DEFAULT_MAX_YAW_RATE = 2.00               # rad/s
DEFAULT_DIAGNOSTICS_RATE_HZ = 20.0
# =========================================================

import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from drillpulse_msgs.msg import OdometryDiagnostics


def extract_yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ThresholdConsistencyNode(Node):
    """Consistency checker evaluating sensor health and cross-disagreement."""

    def __init__(self):
        super().__init__('threshold_consistency_node')

        # Declare ROS Parameters
        self.declare_parameter('velocity_difference_threshold', DEFAULT_VELOCITY_DIFF_THRESHOLD)
        self.declare_parameter('yaw_rate_difference_threshold', DEFAULT_YAW_RATE_DIFF_THRESHOLD)
        self.declare_parameter('yaw_difference_threshold', DEFAULT_YAW_DIFF_THRESHOLD)
        self.declare_parameter('imu_timeout', DEFAULT_IMU_TIMEOUT)
        self.declare_parameter('encoder_timeout', DEFAULT_ENCODER_TIMEOUT)
        self.declare_parameter('maximum_acceleration', DEFAULT_MAX_ACCELERATION)
        self.declare_parameter('maximum_yaw_rate', DEFAULT_MAX_YAW_RATE)
        self.declare_parameter('diagnostics_rate', DEFAULT_DIAGNOSTICS_RATE_HZ)

        self.vel_diff_thresh = float(self.get_parameter('velocity_difference_threshold').value)
        self.yaw_rate_diff_thresh = float(self.get_parameter('yaw_rate_difference_threshold').value)
        self.yaw_diff_thresh = float(self.get_parameter('yaw_difference_threshold').value)
        self.imu_timeout = float(self.get_parameter('imu_timeout').value)
        self.encoder_timeout = float(self.get_parameter('encoder_timeout').value)
        self.max_accel = float(self.get_parameter('maximum_acceleration').value)
        self.max_yaw_rate = float(self.get_parameter('maximum_yaw_rate').value)
        rate = float(self.get_parameter('diagnostics_rate').value)

        # Cached sensor measurements
        self.last_encoder_msg = None
        self.last_encoder_time = 0.0

        self.last_imu_odom_msg = None
        self.last_imu_odom_time = 0.0

        self.last_raw_imu_msg = None
        self.last_raw_imu_time = 0.0

        # ROS Interfaces
        self.diag_pub = self.create_publisher(OdometryDiagnostics, '/odom/diagnostics', 10)
        self.encoder_sub = self.create_subscription(Odometry, '/odom_encoder', self.encoder_callback, 10)
        self.imu_odom_sub = self.create_subscription(Odometry, '/odom_imu', self.imu_odom_callback, 10)
        self.raw_imu_sub = self.create_subscription(Imu, '/sparkfun/imu/data', self.raw_imu_callback, 20)

        timer_period = 1.0 / max(1.0, rate)
        self.timer = self.create_timer(timer_period, self.evaluate_consistency)
        self.get_logger().info('Threshold Consistency Node active.')

    def encoder_callback(self, msg: Odometry):
        self.last_encoder_msg = msg
        self.last_encoder_time = time.time()

    def imu_odom_callback(self, msg: Odometry):
        self.last_imu_odom_msg = msg
        self.last_imu_odom_time = time.time()

    def raw_imu_callback(self, msg: Imu):
        self.last_raw_imu_msg = msg
        self.last_raw_imu_time = time.time()

    def evaluate_consistency(self):
        now = time.time()
        diag = OdometryDiagnostics()
        diag.header.stamp = self.get_clock().now().to_msg()
        diag.header.frame_id = 'base_link'

        # Check freshness
        encoder_fresh = (now - self.last_encoder_time) <= self.encoder_timeout and (self.last_encoder_msg is not None)
        imu_fresh = ((now - self.last_raw_imu_time) <= self.imu_timeout and self.last_raw_imu_msg is not None) or \
                    ((now - self.last_imu_odom_time) <= self.imu_timeout and self.last_imu_odom_msg is not None)

        diag.encoder_valid = bool(encoder_fresh)
        diag.imu_valid = bool(imu_fresh)

        # Baseline confidence
        enc_conf = 1.0 if encoder_fresh else 0.0
        imu_conf = 1.0 if imu_fresh else 0.0

        # Sensor Timeout Check
        if not encoder_fresh and not imu_fresh:
            diag.velocity_error = 0.0
            diag.yaw_rate_error = 0.0
            diag.yaw_error = 0.0
            diag.encoder_confidence = 0.0
            diag.imu_confidence = 0.0
            diag.selected_source = 'NONE'
            diag.fusion_state = 'SENSOR_TIMEOUT'
            self.diag_pub.publish(diag)
            return

        if not encoder_fresh:
            diag.velocity_error = 0.0
            diag.yaw_rate_error = 0.0
            diag.yaw_error = 0.0
            diag.encoder_confidence = 0.0
            diag.imu_confidence = imu_conf
            diag.selected_source = 'IMU_ONLY'
            diag.fusion_state = 'FALLBACK'
            self.diag_pub.publish(diag)
            return

        if not imu_fresh:
            diag.velocity_error = 0.0
            diag.yaw_rate_error = 0.0
            diag.yaw_error = 0.0
            diag.encoder_confidence = enc_conf
            diag.imu_confidence = 0.0
            diag.selected_source = 'ENCODER_ONLY'
            diag.fusion_state = 'FALLBACK'
            self.diag_pub.publish(diag)
            return

        # Both fresh: Compare measurements
        enc_vx = self.last_encoder_msg.twist.twist.linear.x
        enc_wz = self.last_encoder_msg.twist.twist.angular.z
        enc_yaw = extract_yaw_from_quaternion(self.last_encoder_msg.pose.pose.orientation)

        if self.last_raw_imu_msg:
            imu_wz = self.last_raw_imu_msg.angular_velocity.z
        elif self.last_imu_odom_msg:
            imu_wz = self.last_imu_odom_msg.twist.twist.angular.z
        else:
            imu_wz = 0.0

        if self.last_imu_odom_msg:
            imu_vx = self.last_imu_odom_msg.twist.twist.linear.x
            imu_yaw = extract_yaw_from_quaternion(self.last_imu_odom_msg.pose.pose.orientation)
        else:
            imu_vx = enc_vx
            imu_yaw = enc_yaw

        # Errors
        vel_err = abs(enc_vx - imu_vx)
        yaw_rate_err = abs(enc_wz - imu_wz)
        yaw_err = abs(math.atan2(math.sin(enc_yaw - imu_yaw), math.cos(enc_yaw - imu_yaw)))

        diag.velocity_error = float(vel_err)
        diag.yaw_rate_error = float(yaw_rate_err)
        diag.yaw_error = float(yaw_err)

        # Dynamic confidence penalties based on discrepancies
        if yaw_rate_err > self.yaw_rate_diff_thresh:
            enc_conf *= 0.6
            imu_conf *= 0.7

        if vel_err > self.vel_diff_thresh:
            enc_conf *= 0.7  # Likely wheel slippage
            imu_conf *= 0.8

        diag.encoder_confidence = float(enc_conf)
        diag.imu_confidence = float(imu_conf)

        # Gated fusion state classification
        if vel_err > (self.vel_diff_thresh * 1.8) and yaw_rate_err > (self.yaw_rate_diff_thresh * 1.8):
            diag.fusion_state = 'SENSOR_DISAGREEMENT'
            diag.selected_source = 'FALLBACK'
        elif vel_err > self.vel_diff_thresh:
            diag.fusion_state = 'ENCODER_DEGRADED'
            diag.selected_source = 'FUSED'
        elif yaw_rate_err > self.yaw_rate_diff_thresh:
            diag.fusion_state = 'IMU_DEGRADED'
            diag.selected_source = 'FUSED'
        else:
            diag.fusion_state = 'CONSISTENT'
            diag.selected_source = 'FUSED'

        self.diag_pub.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = ThresholdConsistencyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
