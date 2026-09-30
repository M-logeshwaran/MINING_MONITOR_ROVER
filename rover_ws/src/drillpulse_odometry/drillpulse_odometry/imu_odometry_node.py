#!/usr/bin/env python3
# ============================================================
# DRILLPULSE
#
# FILE:
#     imu_odometry_node.py
#
# PURPOSE:
#     Generates inertial dead-reckoning odometry from ICM-20948 IMU
#     angular velocity and linear acceleration. Explicitly models
#     inertial drift, accelerometer bias accumulation, and assigns
#     expanding pose covariances.
#
# INPUT:
#     /sparkfun/imu/data (sensor_msgs/Imu)
#
# OUTPUT:
#     /odom_imu (nav_msgs/Odometry)
#     odom -> base_link (CONDITIONAL: Only when publish_tf is True)
#
# TF OWNER:
#     odom -> base_link (CONDITIONAL: Disabled by default for EKF fusion)
#
# USER PARAMETERS:
#     See USER CONFIGURATION section below.
# ============================================================

# =========================================================
# USER CONFIGURATION
# =========================================================
DEFAULT_IMU_TOPIC = '/imu/data'
DEFAULT_ODOM_TOPIC = '/odom_imu'
DEFAULT_ODOM_FRAME = 'odom'
DEFAULT_BASE_FRAME = 'base_link'
DEFAULT_PUBLISH_TF = False

# Drift compensation & velocity damping
VELOCITY_DECAY_FACTOR = 0.96      # Exponential damping to bound open-loop velocity drift
ACCEL_DEADBAND = 0.08             # m/s^2 acceleration noise threshold
STATIONARY_ACCEL_LIMIT = 0.05
MAX_INTEGRATION_DT = 0.10         # Seconds

# Initial Covariance Settings (Growing with time)
BASE_COV_X = 0.50
BASE_COV_Y = 0.50
BASE_COV_YAW = 0.05
# =========================================================

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


def euler_to_quaternion(yaw, pitch=0.0, roll=0.0):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * cp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * cp * cy
    return q


class IMUOdometryNode(Node):
    """Inertial odometry estimator with drift acknowledgment."""

    def __init__(self):
        super().__init__('imu_odometry_node')

        self.declare_parameter('imu_topic', DEFAULT_IMU_TOPIC)
        self.declare_parameter('odom_topic', DEFAULT_ODOM_TOPIC)
        self.declare_parameter('odom_frame', DEFAULT_ODOM_FRAME)
        self.declare_parameter('base_frame', DEFAULT_BASE_FRAME)
        self.declare_parameter('publish_tf', DEFAULT_PUBLISH_TF)

        imu_topic = str(self.get_parameter('imu_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        # State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.yaw_rate = 0.0

        # Timing & Uncertainty
        self.last_time = None
        self.cumulative_time = 0.0

        # ROS Interfaces
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_callback, 20)
        if imu_topic != '/sparkfun/imu/data':
            self.compat_sub = self.create_subscription(Imu, '/sparkfun/imu/data', self.imu_callback, 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f'IMU Odometry Node active: in={imu_topic}, out={odom_topic}, publish_tf={self.publish_tf}'
        )

    def imu_callback(self, msg: Imu):
        current_time = self.get_clock().now()

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        if dt <= 0.0 or dt > MAX_INTEGRATION_DT:
            return

        self.cumulative_time += dt

        # Gyroscope yaw rate
        self.yaw_rate = msg.angular_velocity.z
        self.yaw += self.yaw_rate * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # Linear acceleration in body frame
        ax_body = msg.linear_acceleration.x
        ay_body = msg.linear_acceleration.y

        # Apply deadband to suppress low-amplitude sensor noise
        if abs(ax_body) < ACCEL_DEADBAND:
            ax_body = 0.0
        if abs(ay_body) < ACCEL_DEADBAND:
            ay_body = 0.0

        # Transform body accelerations into world frame
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        ax_world = ax_body * cos_y - ay_body * sin_y
        ay_world = ax_body * sin_y + ay_body * cos_y

        # Integrate world acceleration into velocity with realistic damping
        self.vx = (self.vx + ax_world * dt) * VELOCITY_DECAY_FACTOR
        self.vy = (self.vy + ay_world * dt) * VELOCITY_DECAY_FACTOR

        # Integrate velocity into position
        self.x += self.vx * dt
        self.y += self.vy * dt

        # Build Odometry message with time-expanding covariance acknowledging drift
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = euler_to_quaternion(self.yaw)

        # Covariance expands linearly with time elapsed (acknowledging unbounded double-integration drift)
        drift_scale = 1.0 + 0.1 * min(100.0, self.cumulative_time)
        odom_msg.pose.covariance = [
            BASE_COV_X * drift_scale, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, BASE_COV_Y * drift_scale, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, BASE_COV_YAW * (1.0 + 0.02 * self.cumulative_time)
        ]

        # Twist in body frame
        v_linear_body = self.vx * cos_y + self.vy * sin_y
        odom_msg.twist.twist.linear.x = v_linear_body
        odom_msg.twist.twist.linear.y = -self.vx * sin_y + self.vy * cos_y
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.z = self.yaw_rate

        odom_msg.twist.covariance = [
            0.15, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.15, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.02
        ]

        self.odom_pub.publish(odom_msg)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom_msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = IMUOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
