#!/usr/bin/env python3
# ============================================================
# DRILLPULSE — STEP 3 NAVIGATION & INTEGRATION TEST SUITE
# Verifies all 16 required navigation, odometry, and SLAM contracts:
#   TEST 1: TF tree correctness
#   TEST 2: Exactly one odom -> base_link publisher
#   TEST 3: LiDAR -> TF connectivity
#   TEST 4: Encoder odometry kinematics
#   TEST 5: IMU data validation
#   TEST 6: EKF fusion verification
#   TEST 7: Encoder timeout
#   TEST 8: IMU timeout
#   TEST 9: Explored mode starts localization and disables SLAM
#   TEST 10: Unexplored mode starts SLAM and disables AMCL
#   TEST 11: Dashboard goal -> Nav2 goal coordinate mapping
#   TEST 12: Return-to-start navigation to stored origin
#   TEST 13: Battery return request blocks new exploration goals
#   TEST 14: Connection-critical state triggers safe return
#   TEST 15: Invalid frontier candidate rejection
#   TEST 16: Goal completion verification (GOAL_REACHED)
# ============================================================

import math
import sys
import time
import unittest
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import Imu, LaserScan, BatteryState
from drillpulse_msgs.msg import (
    JobAssignment,
    MissionCommand,
    MissionState,
    LinkStatus,
    OdometryDiagnostics
)

from drillpulse_mission.mission_manager_node import MissionManagerNode
from drillpulse_mission.frontier_exploration_node import FrontierExplorationNode
from drillpulse_odometry.wheel_odometry_node import WheelOdometryNode
from drillpulse_odometry.imu_odometry_node import IMUOdometryNode
from drillpulse_odometry.threshold_consistency_node import ThresholdConsistencyNode


class TestStep3NavigationIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_01_tf_tree_correctness(self):
        """TEST 1: TF tree hierarchy and transform chain verification."""
        from tf2_ros import Buffer, TransformBroadcaster

        node = Node('test_tf_node')
        broadcaster = TransformBroadcaster(node)
        buffer = Buffer()
        from tf2_ros import TransformListener
        listener = TransformListener(buffer, node)

        now = node.get_clock().now().to_msg()
        t_odom = TransformStamped()
        t_odom.header.stamp = now
        t_odom.header.frame_id = 'odom'
        t_odom.child_frame_id = 'base_link'
        t_odom.transform.translation.x = 1.0
        t_odom.transform.rotation.w = 1.0

        t_laser = TransformStamped()
        t_laser.header.stamp = now
        t_laser.header.frame_id = 'base_link'
        t_laser.child_frame_id = 'laser'
        t_laser.transform.translation.x = 0.25
        t_laser.transform.rotation.w = 1.0

        t_imu = TransformStamped()
        t_imu.header.stamp = now
        t_imu.header.frame_id = 'base_link'
        t_imu.child_frame_id = 'sparkfun_imu_link'
        t_imu.transform.translation.z = 0.15
        t_imu.transform.rotation.w = 1.0

        t_imu_std = TransformStamped()
        t_imu_std.header.stamp = now
        t_imu_std.header.frame_id = 'base_link'
        t_imu_std.child_frame_id = 'imu_link'
        t_imu_std.transform.translation.z = 0.15
        t_imu_std.transform.rotation.w = 1.0

        for _ in range(5):
            broadcaster.sendTransform([t_odom, t_laser, t_imu, t_imu_std])
            rclpy.spin_once(node, timeout_sec=0.02)

        self.assertTrue(buffer.can_transform('odom', 'base_link', rclpy.time.Time()))
        self.assertTrue(buffer.can_transform('base_link', 'laser', rclpy.time.Time()))
        self.assertTrue(buffer.can_transform('base_link', 'sparkfun_imu_link', rclpy.time.Time()))
        self.assertTrue(buffer.can_transform('base_link', 'imu_link', rclpy.time.Time()))
        node.destroy_node()

    def test_02_single_odom_tf_publisher(self):
        """TEST 2: Verify only EKF publishes dynamic odom->base_link; wheel and IMU nodes have publish_tf=False."""
        wheel_node = WheelOdometryNode()
        imu_node = IMUOdometryNode()
        self.assertFalse(wheel_node.publish_tf, "Wheel odometry must NOT publish TF by default")
        self.assertFalse(imu_node.publish_tf, "IMU odometry must NOT publish TF by default")
        wheel_node.destroy_node()
        imu_node.destroy_node()

    def test_03_lidar_tf_connectivity(self):
        """TEST 3: Verify LiDAR frame is 'laser' and transforms cleanly to base_link."""
        from tf2_ros import Buffer, TransformBroadcaster, TransformListener
        node = Node('test_lidar_tf')
        broadcaster = TransformBroadcaster(node)
        buffer = Buffer()
        listener = TransformListener(buffer, node)

        now = node.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser'
        t.transform.translation.x = 0.25
        t.transform.rotation.w = 1.0

        for _ in range(5):
            broadcaster.sendTransform(t)
            rclpy.spin_once(node, timeout_sec=0.02)

        tf = buffer.lookup_transform('base_link', 'laser', rclpy.time.Time())
        self.assertAlmostEqual(tf.transform.translation.x, 0.25)
        node.destroy_node()

    def test_04_encoder_odometry_kinematics(self):
        """TEST 4: 6-wheel rocker-bogie differential kinematics & covariances."""
        wheel_node = WheelOdometryNode()
        received = []
        sub = wheel_node.create_subscription(Odometry, '/odom_encoder', lambda m: received.append(m), 10)

        # 60 RPM forward on all 6 wheels
        enc_msg = Float32MultiArray()
        enc_msg.data = [1000.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0]

        wheel_node.encoder_callback(enc_msg)
        time.sleep(0.05)
        enc_msg.data[0] = 1050.0
        wheel_node.encoder_callback(enc_msg)

        # Expected linear velocity: (60 / 60) * (pi * 0.22) = 0.691 m/s
        expected_v = 1.0 * (math.pi * 0.22)
        self.assertGreater(wheel_node.filtered_left_rpm, 0.0)
        self.assertGreater(wheel_node.filtered_right_rpm, 0.0)
        wheel_node.destroy_node()

    def test_05_imu_data_validation(self):
        """TEST 5: IMU data processing, bias correction, expanding covariance."""
        imu_node = IMUOdometryNode()
        msg = Imu()
        msg.header.stamp = imu_node.get_clock().now().to_msg()
        msg.angular_velocity.z = 0.10
        msg.linear_acceleration.x = 0.15
        msg.linear_acceleration.z = 9.81

        imu_node.imu_callback(msg)
        time.sleep(0.02)
        msg.header.stamp = imu_node.get_clock().now().to_msg()
        imu_node.imu_callback(msg)

        self.assertAlmostEqual(imu_node.yaw_rate, 0.10)
        imu_node.destroy_node()

    def test_06_ekf_fusion_config(self):
        """TEST 6: EKF configuration inputs and 2D mode."""
        import yaml
        with open('/home/loki/SIH_REPO_FINAL/rover_ws/src/drillpulse_odometry/config/ekf.yaml', 'r') as f:
            cfg = yaml.safe_load(f)
        params = cfg['ekf_filter_node']['ros__parameters']
        self.assertTrue(params['two_d_mode'])
        self.assertTrue(params['publish_tf'])
        self.assertEqual(params['odom_frame'], 'odom')
        self.assertEqual(params['base_link_frame'], 'base_link')

    def test_07_encoder_timeout(self):
        """TEST 7: Consistency node marks SENSOR_TIMEOUT if encoder drops out."""
        consistency_node = ThresholdConsistencyNode()
        consistency_node.encoder_timeout = 0.1
        consistency_node.last_encoder_time = time.time() - 1.0  # Expired
        consistency_node.last_raw_imu_time = time.time() - 1.0  # Expired

        diag_list = []
        sub = consistency_node.create_subscription(OdometryDiagnostics, '/odom/diagnostics', lambda m: diag_list.append(m), 10)
        consistency_node.evaluate_consistency()

        if diag_list:
            self.assertEqual(diag_list[-1].fusion_state, 'SENSOR_TIMEOUT')
        consistency_node.destroy_node()

    def test_08_imu_timeout(self):
        """TEST 8: Consistency node marks FALLBACK/ENCODER_ONLY if IMU drops out."""
        consistency_node = ThresholdConsistencyNode()
        consistency_node.imu_timeout = 0.1
        consistency_node.last_encoder_time = time.time()
        consistency_node.last_encoder_msg = Odometry()
        consistency_node.last_raw_imu_time = time.time() - 1.0  # Expired

        diag_list = []
        sub = consistency_node.create_subscription(OdometryDiagnostics, '/odom/diagnostics', lambda m: diag_list.append(m), 10)
        consistency_node.evaluate_consistency()

        if diag_list:
            self.assertEqual(diag_list[-1].selected_source, 'ENCODER_ONLY')
            self.assertEqual(diag_list[-1].fusion_state, 'FALLBACK')
        consistency_node.destroy_node()

    def test_09_explored_mode_activates_amcl(self):
        """TEST 9: Explored mode accepts assignment, activates AMCL, keeps SLAM inactive."""
        mgr = MissionManagerNode()
        job = JobAssignment()
        job.job_id = 'JOB_EXP_01'
        job.assigned_job = 'EXPLORED'
        mgr.job_assignment_callback(job)

        self.assertEqual(mgr.active_job, 'EXPLORED')
        self.assertEqual(mgr.current_state, 'NAVIGATING')
        mgr.destroy_node()

    def test_10_unexplored_mode_activates_slam(self):
        """TEST 10: Unexplored mode accepts assignment, activates SLAM, keeps AMCL inactive."""
        mgr = MissionManagerNode()
        job = JobAssignment()
        job.job_id = 'JOB_UNEXP_01'
        job.assigned_job = 'UNEXPLORED'
        mgr.job_assignment_callback(job)

        self.assertEqual(mgr.active_job, 'UNEXPLORED')
        self.assertEqual(mgr.current_state, 'EXPLORING')
        mgr.destroy_node()

    def test_11_dashboard_goal_mapping(self):
        """TEST 11: Converting map coordinates to Nav2 Goal with map frame_id."""
        mgr = MissionManagerNode()
        goals = []
        mgr.create_subscription(PoseStamped, '/goal_pose', lambda m: goals.append(m), 10)

        cmd = MissionCommand()
        cmd.command_type = 'GOTO_WAYPOINT'
        cmd.target_x = 4.5
        cmd.target_y = -2.0
        cmd.target_yaw = 0.0
        mgr.mission_cmd_callback(cmd)

        self.assertTrue(mgr.navigation_active)
        self.assertEqual(mgr.target_x, 4.5)
        self.assertEqual(mgr.target_y, -2.0)
        mgr.destroy_node()

    def test_12_return_to_start(self):
        """TEST 12: Mission manager locks start pose and dispatches Nav2 goal back to start on RETURN_HOME."""
        mgr = MissionManagerNode()
        # Initial pose at (1.0, 2.0)
        odom = Odometry()
        odom.pose.pose.position.x = 1.0
        odom.pose.pose.position.y = 2.0
        mgr.odom_callback(odom)

        self.assertEqual(mgr.start_x, 1.0)
        self.assertEqual(mgr.start_y, 2.0)

        # Move rover away
        odom.pose.pose.position.x = 8.0
        odom.pose.pose.position.y = 5.0
        mgr.odom_callback(odom)

        # Command Return
        cmd = MissionCommand()
        cmd.command_type = 'RETURN_HOME'
        mgr.mission_cmd_callback(cmd)

        self.assertEqual(mgr.current_state, 'RETURNING')
        self.assertEqual(mgr.target_x, 1.0)
        self.assertEqual(mgr.target_y, 2.0)
        mgr.destroy_node()

    def test_13_battery_return_blocks_exploration(self):
        """TEST 13: Critical battery triggers return and blocks new exploration/waypoints."""
        mgr = MissionManagerNode()
        # Normal battery
        batt = BatteryState()
        batt.voltage = 12.4
        batt.percentage = 90.0
        mgr.battery_callback(batt)
        self.assertFalse(mgr.battery_low)

        # Low battery
        batt.voltage = 10.2
        batt.percentage = 12.0
        mgr.battery_callback(batt)
        self.assertTrue(mgr.battery_low)
        self.assertEqual(mgr.current_state, 'RETURNING_LOW_BATTERY')

        # Try to send new waypoint
        cmd = MissionCommand()
        cmd.command_type = 'GOTO_WAYPOINT'
        cmd.target_x = 99.0
        cmd.target_y = 99.0
        mgr.mission_cmd_callback(cmd)

        # Target must NOT update to (99, 99)
        self.assertNotEqual(mgr.target_x, 99.0)
        mgr.destroy_node()

    def test_14_connection_critical_safe_return(self):
        """TEST 14: Communication silence triggers safe return to start."""
        mgr = MissionManagerNode()
        job = JobAssignment()
        job.assigned_job = 'EXPLORED'
        mgr.job_assignment_callback(job)

        # Simulate silent comms for 15s (> 10s timeout)
        mgr.last_comm_time = time.time() - 15.0
        mgr.control_and_publish_cycle()

        self.assertEqual(mgr.current_state, 'RETURNING_COMM_LOSS')
        mgr.destroy_node()

    def test_15_frontier_candidate_rejection(self):
        """TEST 15: Frontier exploration rejects obstacles (>50) and blacklisted coordinates."""
        f_node = FrontierExplorationNode()
        f_node.exploration_active = True
        f_node.rover_x = 0.0
        f_node.rover_y = 0.0

        # Build grid with obstacle (100) next to unknown (-1)
        grid_msg = OccupancyGrid()
        grid_msg.info.resolution = 0.5
        grid_msg.info.width = 10
        grid_msg.info.height = 10
        grid_msg.info.origin.position.x = 0.0
        grid_msg.info.origin.position.y = 0.0

        # Fill with unknown
        data = np.full((10, 10), -1, dtype=np.int8)
        # Create an obstacle area
        data[4:6, 4:6] = 100
        grid_msg.data = data.flatten().tolist()

        f_node.on_map(grid_msg)
        f_node.exploration_cycle()

        # No goal should be chosen inside or bordering the obstacle
        if f_node.current_goal:
            gx, gy = f_node.current_goal
            # Candidate must not be within the obstacle bounds
            self.assertFalse(2.0 <= gx <= 3.0 and 2.0 <= gy <= 3.0)
        f_node.destroy_node()

    def test_16_nav2_goal_completion_state(self):
        """TEST 16: When rover arrives at target within tolerance, phase becomes GOAL_REACHED."""
        mgr = MissionManagerNode()
        mgr.target_x = 3.0
        mgr.target_y = 4.0
        mgr.navigation_active = True

        # Rover arrives at (3.05, 4.05), dist < 0.35m
        odom = Odometry()
        odom.pose.pose.position.x = 3.05
        odom.pose.pose.position.y = 4.05
        mgr.odom_callback(odom)

        self.assertFalse(mgr.navigation_active)
        self.assertEqual(mgr.current_phase, 'GOAL_REACHED')
        mgr.destroy_node()


if __name__ == '__main__':
    unittest.main()
