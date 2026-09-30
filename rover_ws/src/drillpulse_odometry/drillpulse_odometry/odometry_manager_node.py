#!/usr/bin/env python3
# ============================================================
# DRILLPULSE
#
# FILE:
#     odometry_manager_node.py
#
# PURPOSE:
#     Coordinates the 3 selectable odometry modes (ENCODER_ONLY,
#     IMU_ONLY, FUSED / AUTO). Relays the selected primary odometry
#     to /odom and guarantees single TF ownership of odom -> base_link.
#
# INPUT:
#     /odom_encoder (nav_msgs/Odometry)
#     /odom_imu (nav_msgs/Odometry)
#     /odometry/filtered (nav_msgs/Odometry - from EKF)
#     /odom/diagnostics (drillpulse_msgs/OdometryDiagnostics)
#
# OUTPUT:
#     /odom (nav_msgs/Odometry)
#     odom -> base_link (tf2_ros, ONLY when EKF is disabled)
#
# TF OWNER:
#     odom -> base_link (CONDITIONAL based on active mode)
#
# USER PARAMETERS:
#     See USER CONFIGURATION section below.
# ============================================================

# =========================================================
# USER CONFIGURATION
# =========================================================
DEFAULT_MODE = 'FUSED'       # 'FUSED', 'ENCODER_ONLY', 'IMU_ONLY', 'AUTO'
DEFAULT_ODOM_FRAME = 'odom'
DEFAULT_BASE_FRAME = 'base_link'
# =========================================================

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from drillpulse_msgs.msg import OdometryDiagnostics


class OdometryManagerNode(Node):
    """Supervises odometry modes and prevents conflicting TF broadcasts."""

    def __init__(self):
        super().__init__('odometry_manager_node')

        self.declare_parameter('mode', DEFAULT_MODE)
        self.declare_parameter('odom_frame', DEFAULT_ODOM_FRAME)
        self.declare_parameter('base_frame', DEFAULT_BASE_FRAME)

        self.current_mode = str(self.get_parameter('mode').value).upper()
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        # Mode command subscriber
        self.mode_sub = self.create_subscription(String, '/odometry/set_mode', self.mode_callback, 10)
        self.diag_sub = self.create_subscription(OdometryDiagnostics, '/odom/diagnostics', self.diag_callback, 10)

        # Odometry inputs
        self.encoder_sub = self.create_subscription(Odometry, '/odom_encoder', self.encoder_callback, 10)
        self.imu_sub = self.create_subscription(Odometry, '/odom_imu', self.imu_callback, 10)
        self.ekf_sub = self.create_subscription(Odometry, '/odometry/filtered', self.ekf_callback, 10)

        # Primary Output
        self.primary_odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.latest_encoder_odom = None
        self.latest_imu_odom = None
        self.latest_ekf_odom = None
        self.latest_diagnostics = None

        self.get_logger().info(f'Odometry Manager active. Initial Mode: {self.current_mode}')

    def mode_callback(self, msg: String):
        req_mode = msg.data.strip().upper()
        if req_mode in ('FUSED', 'ENCODER_ONLY', 'IMU_ONLY', 'AUTO'):
            self.current_mode = req_mode
            self.get_logger().info(f'Switched Odometry Mode to: {self.current_mode}')

    def diag_callback(self, msg: OdometryDiagnostics):
        self.latest_diagnostics = msg

    def encoder_callback(self, msg: Odometry):
        self.latest_encoder_odom = msg
        if self.current_mode == 'ENCODER_ONLY':
            self._publish_primary_and_tf(msg)

    def imu_callback(self, msg: Odometry):
        self.latest_imu_odom = msg
        if self.current_mode == 'IMU_ONLY':
            self._publish_primary_and_tf(msg)

    def ekf_callback(self, msg: Odometry):
        self.latest_ekf_odom = msg
        if self.current_mode in ('FUSED', 'AUTO'):
            # In FUSED mode, EKF produces /odometry/filtered, we mirror to /odom if not remapped.
            # EKF itself publishes TF odom -> base_link, so we DO NOT publish TF here!
            self.primary_odom_pub.publish(msg)

    def _publish_primary_and_tf(self, odom_msg: Odometry):
        # Publish to /odom
        self.primary_odom_pub.publish(odom_msg)

        # Publish TF odom -> base_link only in manual non-fused modes
        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        t.transform.rotation = odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
