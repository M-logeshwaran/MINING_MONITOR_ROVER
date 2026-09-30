#!/usr/bin/env python3
# ============================================================
# DRILLPULSE
#
# FILE:
#     wheel_odometry_node.py
#
# PURPOSE:
#     Calculates differential-drive dead-reckoning odometry from
#     6-wheel RPM feedback using side median filtering and
#     constant-curvature arc kinematics.
#
# INPUT:
#     /encoder (std_msgs/Float32MultiArray)
#     Contract: [timestamp_ms, LF_RPM, RF_RPM, LM_RPM, RM_RPM, LB_RPM, RB_RPM]
#
# OUTPUT:
#     /odom_encoder (nav_msgs/Odometry)
#     odom -> base_link (tf2_ros, ONLY when publish_tf is True)
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
DEFAULT_WHEEL_DIAMETER = 0.22      # meters (measured 22cm)
DEFAULT_TRACK_WIDTH = 0.90         # meters (effective center-to-center)
DEFAULT_FILTER_ALPHA = 0.35        # Low-pass alpha (0.0 to 1.0)
DEFAULT_USE_MEDIAN_FILTER = True   # Reject single-wheel outliers
DEFAULT_USE_ARC_INTEGRATION = True # Constant-curvature exact update
DEFAULT_NORMALIZE_YAW = True
DEFAULT_STALE_TIMEOUT = 1.0        # Seconds before flagging stale data
DEFAULT_PUBLISH_TF = False         # MUST be False when EKF is active

DEFAULT_ODOM_FRAME = 'odom'
DEFAULT_BASE_FRAME = 'base_link'
DEFAULT_ENCODER_TOPIC = '/encoder'
DEFAULT_OUTPUT_TOPIC = '/odom_encoder'

# Covariances for /odom_encoder
COV_VX = 0.02
COV_VY = 1e6
COV_WZ = 0.05
COV_X = 0.05
COV_Y = 0.05
COV_YAW = 0.10
# =========================================================

import math
import statistics
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
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
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class WheelOdometryNode(Node):
    """Calculates dead-reckoning odometry from 6-wheel encoder telemetry."""

    def __init__(self):
        super().__init__('wheel_odometry_node')

        # Declare ROS Parameters
        self.declare_parameter('wheel_diameter', DEFAULT_WHEEL_DIAMETER)
        self.declare_parameter('track_width', DEFAULT_TRACK_WIDTH)
        self.declare_parameter('filter_alpha', DEFAULT_FILTER_ALPHA)
        self.declare_parameter('use_median_filter', DEFAULT_USE_MEDIAN_FILTER)
        self.declare_parameter('use_arc_integration', DEFAULT_USE_ARC_INTEGRATION)
        self.declare_parameter('normalize_yaw', DEFAULT_NORMALIZE_YAW)
        self.declare_parameter('stale_timeout', DEFAULT_STALE_TIMEOUT)
        self.declare_parameter('publish_tf', DEFAULT_PUBLISH_TF)
        self.declare_parameter('odom_frame', DEFAULT_ODOM_FRAME)
        self.declare_parameter('base_frame', DEFAULT_BASE_FRAME)
        self.declare_parameter('encoder_topic', DEFAULT_ENCODER_TOPIC)
        self.declare_parameter('odom_topic', DEFAULT_OUTPUT_TOPIC)

        self.wheel_diameter = float(self.get_parameter('wheel_diameter').value)
        self.track_width = float(self.get_parameter('track_width').value)
        self.filter_alpha = float(self.get_parameter('filter_alpha').value)
        self.use_median = bool(self.get_parameter('use_median_filter').value)
        self.use_arc = bool(self.get_parameter('use_arc_integration').value)
        self.normalize_yaw = bool(self.get_parameter('normalize_yaw').value)
        self.stale_timeout = float(self.get_parameter('stale_timeout').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        encoder_topic = str(self.get_parameter('encoder_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)

        # Pose state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Filtered RPM state
        self.filtered_left_rpm = 0.0
        self.filtered_right_rpm = 0.0

        # Timing
        self.last_time = None
        self.last_ts_ms = None

        # ROS Interfaces
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.encoder_sub = self.create_subscription(
            Float32MultiArray,
            encoder_topic,
            self.encoder_callback,
            10
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f'Wheel Odometry Node active: output={odom_topic}, publish_tf={self.publish_tf}'
        )

    def encoder_callback(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) < 7:
            return

        ts_ms = data[0]
        lf, rf = data[1], data[2]
        lm, rm = data[3], data[4]
        lb, rb = data[5], data[6]

        # 1. Median filtering across 3 wheels per side to reject single wheel slippage
        if self.use_median:
            raw_left_rpm = statistics.median([lf, lm, lb])
            raw_right_rpm = statistics.median([rf, rm, rb])
        else:
            raw_left_rpm = (lf + lm + lb) / 3.0
            raw_right_rpm = (rf + rm + rb) / 3.0

        # 2. First-order low pass filter
        self.filtered_left_rpm = (
            self.filter_alpha * raw_left_rpm + (1.0 - self.filter_alpha) * self.filtered_left_rpm
        )
        self.filtered_right_rpm = (
            self.filter_alpha * raw_right_rpm + (1.0 - self.filter_alpha) * self.filtered_right_rpm
        )

        current_time = self.get_clock().now()

        if self.last_time is None:
            self.last_time = current_time
            self.last_ts_ms = ts_ms
            return

        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        if dt <= 0.0 or dt > self.stale_timeout:
            self.get_logger().warn(f'Wheel odom stale sample or time step jump (dt={dt:.4f}s), skipping integration.')
            return

        # 3. Kinematics calculation
        wheel_circ = math.pi * self.wheel_diameter
        v_left = (self.filtered_left_rpm / 60.0) * wheel_circ
        v_right = (self.filtered_right_rpm / 60.0) * wheel_circ

        v_linear = (v_right + v_left) * 0.5
        v_angular = (v_right - v_left) / max(0.01, self.track_width)

        # 4. Pose integration (Arc integration vs Euler)
        if self.use_arc and abs(v_angular) > 1e-4:
            radius = v_linear / v_angular
            d_yaw = v_angular * dt
            dx = radius * (math.sin(self.yaw + d_yaw) - math.sin(self.yaw))
            dy = -radius * (math.cos(self.yaw + d_yaw) - math.cos(self.yaw))
            self.x += dx
            self.y += dy
            self.yaw += d_yaw
        else:
            d_dist = v_linear * dt
            d_yaw = v_angular * dt
            avg_yaw = self.yaw + 0.5 * d_yaw
            self.x += d_dist * math.cos(avg_yaw)
            self.y += d_dist * math.sin(avg_yaw)
            self.yaw += d_yaw

        if self.normalize_yaw:
            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # 5. Populate Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = euler_to_quaternion(self.yaw)

        # Pose Covariance
        odom_msg.pose.covariance = [
            COV_X, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, COV_Y, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, COV_YAW
        ]

        odom_msg.twist.twist.linear.x = v_linear
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = v_angular

        # Twist Covariance
        odom_msg.twist.covariance = [
            COV_VX, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, COV_VY, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, COV_WZ
        ]

        self.odom_pub.publish(odom_msg)

        # 6. Publish Dynamic TF ONLY when explicitly configured
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
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
