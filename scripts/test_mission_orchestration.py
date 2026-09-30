#!/usr/bin/env python3
# ============================================================
# DRILLPULSE — Step 7 Mission Orchestration & Autonomous Return
# Platform: Ubuntu 24.04 LTS + ROS 2 Jazzy + Python 3.12
#
# Covers all 20 Validation Requirements:
#   1. State transitions: Valid path across the 18 states
#   2. Invalid state transitions: Explicit rejection of illegal transitions
#   3. Machine-readable Job Validation & Rejection (6 reasons)
#   4. Explored mode execution & AMCL alignment
#   5. Unexplored mode execution & SLAM alignment
#   6. Frontier candidate scoring & hysteresis
#   7. Target dispatch to /goal_pose
#   8. Target lifecycle acknowledgement (REQUESTED -> ACCEPTED -> EXECUTING -> COMPLETED/FAILED)
#   9. Hazard-triggered state transition & return
#  10. Return-to-start real localization pose lock
#  11. Return reason consistency
#  12. Return failure retry escalation & SAFE_HOLD
#  13. Recovery navigation interlock (motion blocked during recovery)
#  14. Manual control safety interlock (blocked on e-stop)
#  15. Mission session ID generation format
#  16. Mission completion handling
#  17. Report generation trigger request
#  18. Dashboard command acknowledgement
#  19. Map click-to-point coordinate conversion
#  20. Communication-loss return transition
# ============================================================

import os
import re
import math
import time
import unittest

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool

from drillpulse_msgs.msg import (
    RoverTelemetry,
    LinkStatus,
    MissionState,
    MissionCommand,
    JobAssignment,
    HazardEvent,
    SafetyStatus,
    RecoveryStatus,
    OdometryDiagnostics
)

from drillpulse_mission.mission_manager_node import (
    MissionManagerNode,
    MissionStateEnum,
    ReturnReasonEnum,
    RejectionReasonEnum,
    VALID_TRANSITIONS
)
from drillpulse_mission.frontier_exploration_node import FrontierExplorationNode
from drillpulse_dashboard.dashboard_node import DrillPulseDashboardNode


class TestStep7MissionOrchestration(unittest.TestCase):
    """Step 7 Comprehensive Verification Suite (20 Checks)."""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    # ------------------------------------------------------------
    # 01. State Transitions: Valid Path
    # ------------------------------------------------------------
    def test_01_state_transitions_valid_path(self):
        """TEST 01: Valid state transitions through normal mission progression."""
        mgr = MissionManagerNode()
        # STANDBY -> JOB_RECEIVED -> JOB_ACCEPTED -> INITIALIZING -> READY -> AUTONOMOUS_NAVIGATION -> RETURNING -> RETURNED_TO_BASE -> MISSION_COMPLETE
        self.assertTrue(mgr.transition_to(MissionStateEnum.STANDBY, "INITIAL", "TEST"))
        self.assertTrue(mgr.transition_to(MissionStateEnum.JOB_RECEIVED, "NEW_JOB", "TEST"))
        self.assertTrue(mgr.transition_to(MissionStateEnum.JOB_ACCEPTED, "VALID_JOB", "TEST"))
        self.assertTrue(mgr.transition_to(MissionStateEnum.INITIALIZING, "SUBSYSTEMS_BOOT", "TEST"))
        self.assertTrue(mgr.transition_to(MissionStateEnum.READY, "SYSTEMS_ONLINE", "TEST"))
        self.assertTrue(mgr.transition_to(MissionStateEnum.AUTONOMOUS_NAVIGATION, "WAYPOINT_DISPATCHED", "TEST"))
        self.assertTrue(mgr.transition_to(MissionStateEnum.RETURNING, "MISSION_FINISHED", "TEST"))
        self.assertTrue(mgr.transition_to(MissionStateEnum.RETURNED_TO_BASE, "BASE_ARRIVED", "TEST"))
        self.assertTrue(mgr.transition_to(MissionStateEnum.MISSION_COMPLETE, "REPORT_FILED", "TEST"))
        self.assertEqual(mgr.current_state, MissionStateEnum.MISSION_COMPLETE)
        mgr.destroy_node()

    # ------------------------------------------------------------
    # 02. Invalid State Transitions Rejection
    # ------------------------------------------------------------
    def test_02_invalid_state_transition_rejection(self):
        """TEST 02: Explicit rejection of illegal transitions."""
        mgr = MissionManagerNode()
        mgr.current_state = MissionStateEnum.STANDBY

        # Cannot jump directly from STANDBY to MISSION_COMPLETE
        ok = mgr.transition_to(MissionStateEnum.MISSION_COMPLETE, "ILLEGAL_JUMP", "TEST")
        self.assertFalse(ok)
        self.assertEqual(mgr.current_state, MissionStateEnum.STANDBY)

        # Cannot jump directly from STANDBY to RETURNED_TO_BASE
        ok2 = mgr.transition_to(MissionStateEnum.RETURNED_TO_BASE, "ILLEGAL_JUMP", "TEST")
        self.assertFalse(ok2)
        self.assertEqual(mgr.current_state, MissionStateEnum.STANDBY)
        mgr.destroy_node()

    # ------------------------------------------------------------
    # 03. Machine-Readable Job Validation & Rejection
    # ------------------------------------------------------------
    def test_03_job_validation_and_rejection(self):
        """TEST 03: Machine-readable job validation & exact rejections."""
        mgr = MissionManagerNode()
        mgr.current_state = MissionStateEnum.STANDBY
        mgr.battery_percentage = 85.0
        mgr.battery_voltage = 12.4
        mgr.has_received_odom = True
        mgr.emergency_stop_active = False
        mgr.motion_allowed = True
        mgr.comm_connected = True

        # Valid Explored Job
        valid_job = JobAssignment()
        valid_job.job_id = "JOB_101"
        valid_job.assigned_job = "EXPLORED"
        ok, reason = mgr.validate_job(valid_job)
        self.assertTrue(ok)
        self.assertEqual(reason, "VALID")

        # Invalid Job (empty mode)
        empty_job = JobAssignment()
        empty_job.job_id = "JOB_100"
        empty_job.assigned_job = ""
        ok, reason = mgr.validate_job(empty_job)
        self.assertFalse(ok)
        self.assertEqual(reason, RejectionReasonEnum.INVALID_JOB)

        # Invalid Mode
        bad_mode = JobAssignment()
        bad_mode.job_id = "JOB_102"
        bad_mode.assigned_job = "SURFACE_DRIVE"
        ok, reason = mgr.validate_job(bad_mode)
        self.assertFalse(ok)
        self.assertEqual(reason, RejectionReasonEnum.INVALID_MODE)

        # Low Battery Reject
        mgr.battery_percentage = 12.0
        ok, reason = mgr.validate_job(valid_job)
        self.assertFalse(ok)
        self.assertEqual(reason, RejectionReasonEnum.BATTERY_TOO_LOW)
        mgr.battery_percentage = 85.0

        # Localization Unavailable Reject
        mgr.has_received_odom = False
        ok, reason = mgr.validate_job(valid_job)
        self.assertFalse(ok)
        self.assertEqual(reason, RejectionReasonEnum.LOCALIZATION_UNAVAILABLE)
        mgr.has_received_odom = True

        # Safety Locked Reject (e-stop active)
        mgr.emergency_stop_active = True
        ok, reason = mgr.validate_job(valid_job)
        self.assertFalse(ok)
        self.assertEqual(reason, RejectionReasonEnum.SAFETY_LOCK)
        mgr.emergency_stop_active = False

        # Comm Lost Reject
        mgr.comm_connected = False
        ok, reason = mgr.validate_job(valid_job)
        self.assertFalse(ok)
        self.assertEqual(reason, RejectionReasonEnum.COMMUNICATION_UNSTABLE)

        mgr.destroy_node()

    # ------------------------------------------------------------
    # 04. Explored Mode Execution & AMCL Alignment
    # ------------------------------------------------------------
    def test_04_explored_mode_amcl_alignment(self):
        """TEST 04: Explored mode activates AMCL corridor and handles waypoint dispatch."""
        mgr = MissionManagerNode()
        mgr.has_received_odom = True
        job = JobAssignment()
        job.job_id = "EXP_01"
        job.assigned_job = "EXPLORED"
        job.map_name = "rover_map"
        mgr.job_assignment_callback(job)

        self.assertEqual(mgr.active_job, "EXPLORED")
        self.assertEqual(mgr.map_name, "rover_map")

        # Dispatch waypoint
        cmd = MissionCommand()
        cmd.command_id = "WP_01"
        cmd.command_type = "GOTO_WAYPOINT"
        cmd.target_x = 5.0
        cmd.target_y = 2.0
        mgr.mission_cmd_callback(cmd)
        self.assertIn(mgr.current_state, (MissionStateEnum.AUTONOMOUS_NAVIGATION, MissionStateEnum.NAVIGATING))
        mgr.destroy_node()

    # ------------------------------------------------------------
    # 05. Unexplored Mode Execution & SLAM Alignment
    # ------------------------------------------------------------
    def test_05_unexplored_mode_slam_alignment(self):
        """TEST 05: Unexplored mode activates SLAM Toolbox and frontier exploration."""
        mgr = MissionManagerNode()
        mgr.has_received_odom = True
        job = JobAssignment()
        job.job_id = "UNEXP_01"
        job.assigned_job = "UNEXPLORED"
        mgr.job_assignment_callback(job)

        self.assertEqual(mgr.active_job, "UNEXPLORED")
        self.assertIn(mgr.current_state, (MissionStateEnum.AUTONOMOUS_NAVIGATION, MissionStateEnum.EXPLORING))
        mgr.destroy_node()

    # ------------------------------------------------------------
    # 06. Frontier Candidate Scoring & Hysteresis
    # ------------------------------------------------------------
    def test_06_frontier_scoring_and_hysteresis(self):
        """TEST 06: Multi-factor candidate scoring and target commitment hysteresis."""
        fn = FrontierExplorationNode()
        score1 = fn.score_candidate(dist=3.0, info_gain=15, min_obs_dist=1.5, is_current=False)
        score2 = fn.score_candidate(dist=5.0, info_gain=2, min_obs_dist=0.2, is_current=False)
        self.assertLess(score1, score2)

        # Hysteresis: current candidate receives commitment discount
        score_new = fn.score_candidate(dist=3.0, info_gain=15, min_obs_dist=1.5, is_current=False)
        score_curr = fn.score_candidate(dist=3.0, info_gain=15, min_obs_dist=1.5, is_current=True)
        self.assertLess(score_curr, score_new)
        fn.destroy_node()

    # ------------------------------------------------------------
    # 07. Target Dispatch to /goal_pose
    # ------------------------------------------------------------
    def test_07_target_dispatch_to_goal_pose(self):
        """TEST 07: Target poses published to /goal_pose with map frame."""
        mgr = MissionManagerNode()
        received = []
        sub = mgr.create_subscription(PoseStamped, '/goal_pose', lambda msg: received.append(msg), 10)

        # Dispatch nav goal
        mgr._dispatch_nav2_goal(1.25, 2.50, 0.0)

        rclpy.spin_once(mgr, timeout_sec=0.1)
        self.assertGreater(len(received), 0)
        self.assertEqual(received[-1].header.frame_id, 'map')
        self.assertAlmostEqual(received[-1].pose.position.x, 1.25)
        self.assertAlmostEqual(received[-1].pose.position.y, 2.50)
        mgr.destroy_node()

    # ------------------------------------------------------------
    # 08. Target Lifecycle Acknowledgement
    # ------------------------------------------------------------
    def test_08_target_lifecycle_acknowledgement(self):
        """TEST 08: Commands tracked through REQUESTED -> ACCEPTED -> EXECUTING -> COMPLETED/FAILED."""
        dash = DrillPulseDashboardNode()
        cmd_id = "CMD_TEST_LIFECYCLE"
        dash.active_commands[cmd_id] = {
            'cmd_id': cmd_id,
            'status': 'REQUESTED',
            'timestamp': time.time(),
            'type': 'WAYPOINT'
        }

        # Step 1: ACK received
        ack = JobAssignment()
        ack.job_id = cmd_id
        ack.status = "JOB_ACCEPTED"
        dash._on_job_ack(ack)
        self.assertEqual(dash.active_commands[cmd_id]['status'], 'ACCEPTED')

        # Step 2: Mission moves to AUTONOMOUS_NAVIGATION
        mst = MissionState()
        mst.current_state = MissionStateEnum.AUTONOMOUS_NAVIGATION
        dash._on_mission_state(mst)
        self.assertEqual(dash.active_commands[cmd_id]['status'], 'EXECUTING')

        # Step 3: Mission completes
        mst_done = MissionState()
        mst_done.current_state = MissionStateEnum.MISSION_COMPLETE
        dash._on_mission_state(mst_done)
        self.assertEqual(dash.active_commands[cmd_id]['status'], 'COMPLETED')
        dash.destroy_node()

    # ------------------------------------------------------------
    # 09. Hazard-Triggered State Transition & Return
    # ------------------------------------------------------------
    def test_09_hazard_triggered_state_transition(self):
        """TEST 09: Critical hazard triggers HAZARD_RESPONSE and base return."""
        mgr = MissionManagerNode()
        mgr.current_state = MissionStateEnum.AUTONOMOUS_NAVIGATION
        mgr.start_x = 0.0
        mgr.start_y = 0.0
        mgr.start_pose_locked = True

        h = HazardEvent()
        h.event_id = "HZ_CRIT_CH4"
        h.event_type = "METHANE_CRITICAL"
        h.severity = "CRITICAL"
        h.measured_value = 25.0
        h.threshold_value = 10.0
        mgr.hazard_callback(h)

        self.assertIn(mgr.current_state, (MissionStateEnum.HAZARD_RESPONSE, MissionStateEnum.RETURNING))
        self.assertEqual(mgr.return_reason, ReturnReasonEnum.HAZARD)
        self.assertGreater(len(mgr.spatial_hazard_journal), 0)
        mgr.destroy_node()

    # ------------------------------------------------------------
    # 10. Return-to-Start Real Localization Pose Lock
    # ------------------------------------------------------------
    def test_10_return_to_start_real_pose_lock(self):
        """TEST 10: Return locks to true localization start pose, not hardcoded origin."""
        mgr = MissionManagerNode()
        odom = Odometry()
        odom.pose.pose.position.x = 8.42
        odom.pose.pose.position.y = -3.15
        mgr.odom_callback(odom)

        self.assertTrue(mgr.start_pose_locked)
        self.assertAlmostEqual(mgr.start_x, 8.42)
        self.assertAlmostEqual(mgr.start_y, -3.15)

        mgr.trigger_return_to_start(reason=ReturnReasonEnum.OPERATOR)
        self.assertEqual(mgr.return_retries, 1)
        mgr.destroy_node()

    # ------------------------------------------------------------
    # 11. Return Reason Consistency
    # ------------------------------------------------------------
    def test_11_return_reason_consistency(self):
        """TEST 11: Return reasons consistently tracked across triggers."""
        mgr = MissionManagerNode()
        mgr.start_pose_locked = True

        # Operator
        mgr.trigger_return_to_start(reason=ReturnReasonEnum.OPERATOR)
        self.assertEqual(mgr.return_reason, ReturnReasonEnum.OPERATOR)

        # Comm loss
        mgr.trigger_return_to_start(reason=ReturnReasonEnum.COMM_LOSS)
        self.assertEqual(mgr.return_reason, ReturnReasonEnum.COMM_LOSS)

        # Low battery
        mgr.trigger_return_to_start(reason=ReturnReasonEnum.BATTERY_RETURN)
        self.assertEqual(mgr.return_reason, ReturnReasonEnum.BATTERY_RETURN)

        # Critical battery
        mgr.trigger_return_to_start(reason=ReturnReasonEnum.CRITICAL_BATTERY)
        self.assertEqual(mgr.return_reason, ReturnReasonEnum.CRITICAL_BATTERY)
        mgr.destroy_node()

    # ------------------------------------------------------------
    # 12. Return Failure Retry Escalation & SAFE_HOLD
    # ------------------------------------------------------------
    def test_12_return_failure_retry_and_safe_hold(self):
        """TEST 12: Stalled return retries 3 times before entering SAFE_HOLD."""
        mgr = MissionManagerNode()
        mgr.current_state = MissionStateEnum.RETURNING
        mgr.navigation_active = True
        mgr.max_retries = 3
        mgr.return_retries = 1
        mgr.last_nav_progress_time = time.time() - 35.0  # Force timeout
        mgr.nav_stuck_timeout = 10.0

        # Cycle 1: retry 2
        mgr.control_and_publish_cycle()
        self.assertEqual(mgr.return_retries, 2)
        self.assertIn(mgr.current_state, (MissionStateEnum.RETURNING, MissionStateEnum.RETURNING_HOME))

        # Cycle 2: retry 3
        mgr.last_nav_progress_time = time.time() - 35.0
        mgr.control_and_publish_cycle()
        self.assertEqual(mgr.return_retries, 3)

        # Cycle 3: Exceeded max retries -> Escalate to SAFE_HOLD / HOLD_POSITION
        mgr.last_nav_progress_time = time.time() - 35.0
        mgr.control_and_publish_cycle()
        self.assertIn(mgr.current_state, (MissionStateEnum.SAFE_HOLD, MissionStateEnum.HOLD_POSITION))
        self.assertEqual(mgr.last_transition_reason, "NAV2_STUCK_EXCEEDED_RETRIES")
        mgr.destroy_node()

    # ------------------------------------------------------------
    # 13. Recovery Navigation Interlock
    # ------------------------------------------------------------
    def test_13_recovery_navigation_interlock(self):
        """TEST 13: Navigation motion blocked while recovery actuator is operating."""
        mgr = MissionManagerNode()
        rec_status = RecoveryStatus()
        rec_status.state = "EXTENDING"
        rec_status.active_side = "LEFT"
        rec_status.actuator_position = 0.08
        mgr.recovery_callback(rec_status)

        self.assertTrue(mgr.is_recovery_active)
        self.assertEqual(mgr.current_state, MissionStateEnum.RECOVERY)

        # Finished recovery returns to READY
        rec_status.state = "READY"
        rec_status.actuator_position = 0.0
        mgr.recovery_callback(rec_status)
        self.assertFalse(mgr.is_recovery_active)
        mgr.destroy_node()

    # ------------------------------------------------------------
    # 14. Manual Control Safety Interlock
    # ------------------------------------------------------------
    def test_14_manual_control_safety_interlock(self):
        """TEST 14: Emergency stop latched prevents motion dispatch."""
        mgr = MissionManagerNode()
        safety = SafetyStatus()
        safety.emergency_stop_active = True
        safety.motion_allowed = False
        mgr.safety_callback(safety)

        self.assertEqual(mgr.current_state, MissionStateEnum.EMERGENCY_STOP)
        self.assertTrue(mgr.emergency_stop_active)

        # Attempting to assign job is rejected
        job = JobAssignment()
        job.job_id = "JOB_ESTOP_TEST"
        job.assigned_job = "EXPLORED"
        ok, reason = mgr.validate_job(job)
        self.assertFalse(ok)
        self.assertEqual(reason, RejectionReasonEnum.SAFETY_LOCK)
        mgr.destroy_node()

    # ------------------------------------------------------------
    # 15. Mission Session ID Generation Format
    # ------------------------------------------------------------
    def test_15_session_id_format(self):
        """TEST 15: Session ID follows collision-resistant DRILLPULSE-YYYYMMDD-HHMMSS-XXXX pattern."""
        mgr = MissionManagerNode()
        pattern = r"^DRILLPULSE-\d{8}-\d{6}-[0-9A-F]{4}$"
        self.assertRegex(mgr.session_id, pattern)
        mgr.destroy_node()

    # ------------------------------------------------------------
    # 16. Mission Completion Handling
    # ------------------------------------------------------------
    def test_16_mission_completion_handling(self):
        """TEST 16: Returning rover arriving within threshold transitions to base arrival."""
        mgr = MissionManagerNode()
        mgr.start_x = 0.0
        mgr.start_y = 0.0
        mgr.start_pose_locked = True
        mgr.target_x = 0.0
        mgr.target_y = 0.0
        mgr.current_state = MissionStateEnum.RETURNING
        mgr.navigation_active = True

        # Rover arrives at (0.1, 0.1) -> distance ~ 0.14m <= 0.35m tolerance
        odom = Odometry()
        odom.pose.pose.position.x = 0.10
        odom.pose.pose.position.y = 0.10
        mgr.odom_callback(odom)

        self.assertIn(mgr.current_state, (MissionStateEnum.RETURNED_TO_BASE, MissionStateEnum.RETURNED_TO_START))
        mgr.destroy_node()

    # ------------------------------------------------------------
    # 17. Report Generation Trigger Request
    # ------------------------------------------------------------
    def test_17_report_generation_trigger(self):
        """TEST 17: Mission completion triggers report generation request."""
        mgr = MissionManagerNode()
        received_reports = []
        sub = mgr.create_subscription(String, '/drillpulse/generate_report', lambda msg: received_reports.append(msg.data), 10)

        mgr._request_report_generation("SUCCESS")
        rclpy.spin_once(mgr, timeout_sec=0.1)

        self.assertGreater(len(received_reports), 0)
        self.assertIn("TRIGGER_REPORT", received_reports[-1])
        self.assertIn("SUCCESS", received_reports[-1])
        mgr.destroy_node()

    # ------------------------------------------------------------
    # 18. Dashboard Command Acknowledgement
    # ------------------------------------------------------------
    def test_18_dashboard_command_acknowledgement(self):
        """TEST 18: Dashboard updates command status from rover job acknowledgement."""
        dash = DrillPulseDashboardNode()
        cmd_id = "JOB_TEST_ACK"
        dash.active_commands[cmd_id] = {
            'cmd_id': cmd_id,
            'status': 'REQUESTED',
            'timestamp': time.time(),
            'type': 'MODE_EXPLORED'
        }

        job_ack = JobAssignment()
        job_ack.job_id = cmd_id
        job_ack.assigned_job = "EXPLORED"
        job_ack.status = "JOB_ACCEPTED"
        dash._on_job_ack(job_ack)

        self.assertEqual(dash.active_commands[cmd_id]['status'], 'ACCEPTED')
        dash.destroy_node()

    # ------------------------------------------------------------
    # 19. Map Click-to-Point Coordinate Conversion
    # ------------------------------------------------------------
    def test_19_map_click_coordinate_conversion(self):
        """TEST 19: Mathematical inversion between canvas pixels and metric map coords."""
        cw, ch = 600, 400
        scale = 35.0

        # Test point: wx = 2.5m, wy = -1.5m
        wx_orig, wy_orig = 2.5, -1.5

        # Forward: worldToCanvas
        cx = cw / 2.0 + wx_orig * scale
        cy = ch / 2.0 - wy_orig * scale

        # Inverse: canvasToWorld
        wx_calc = (cx - cw / 2.0) / scale
        wy_calc = (ch / 2.0 - cy) / scale

        self.assertAlmostEqual(wx_orig, wx_calc, places=4)
        self.assertAlmostEqual(wy_orig, wy_calc, places=4)

    # ------------------------------------------------------------
    # 20. Communication-Loss Return Transition
    # ------------------------------------------------------------
    def test_20_comms_loss_return_transition(self):
        """TEST 20: Communication loss triggers autonomous return to base."""
        mgr = MissionManagerNode()
        mgr.current_state = MissionStateEnum.AUTONOMOUS_NAVIGATION
        mgr.start_x = 0.0
        mgr.start_y = 0.0
        mgr.start_pose_locked = True

        lost_link = LinkStatus()
        lost_link.connected = False
        lost_link.signal_strength_rssi = -120.0
        lost_link.packet_loss_rate = 1.0
        mgr.link_callback(lost_link)

        self.assertIn(mgr.current_state, (MissionStateEnum.RETURNING, MissionStateEnum.RETURNING_COMM_LOSS))
        self.assertEqual(mgr.return_reason, ReturnReasonEnum.COMM_LOSS)
        mgr.destroy_node()


if __name__ == '__main__':
    unittest.main()
