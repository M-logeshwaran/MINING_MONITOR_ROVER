#!/usr/bin/env python3
# ============================================================
# DRILLPULSE — Step 8 Robust Communication & Adaptive Telemetry Test Suite
# Platform: Ubuntu 24.04 LTS + ROS 2 Jazzy + Python 3.12
#
# Covers all 20 Step 8 Verification Requirements:
#   1. GOOD communication state
#   2. DEGRADED communication state
#   3. WEAK communication state
#   4. INTERMITTENT communication state
#   5. DISCONNECTED communication state
#   6. Recovery after communication loss
#   7. Priority queue behavior (Priority 0-3 ordering)
#   8. Critical message preservation (Priority 0 never dropped by lower)
#   9. Low-priority message dropping on queue saturation
#  10. Heartbeat timeout detection
#  11. Duplicate command protection (idempotency guard)
#  12. Command acknowledgement lifecycle
#  13. Return-to-base command & acknowledgement
#  14. Communication-loss safety behavior (onboard autonomous return)
#  15. Video degradation & suspension (zero interference with LoRa)
#  16. Telemetry recovery & queue burst synchronization
#  17. Local buffering during disconnection
#  18. Session communication log generation & rotation
#  19. Dashboard zero-fake-data policy (explicit None / "--")
#  20. Rover safe autonomous operation without dashboard
# ============================================================

import os
import json
import time
import math
import unittest

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool

from drillpulse_msgs.msg import (
    RoverTelemetry,
    LinkStatus,
    MissionState,
    MissionCommand,
    JobAssignment,
    HazardEvent,
    RecoveryStatus,
    OdometryDiagnostics
)

from drillpulse_transport.priority_queue import (
    PriorityTelemetryQueue,
    QueueItem,
    TelemetryPriority
)
from drillpulse_transport.connection_state import (
    ConnectionQualityEvaluator,
    LinkStateEnum,
    OverallLinkStateEnum
)
from drillpulse_transport.acknowledgement_manager import (
    AcknowledgementManager,
    AckStatusEnum,
    ErrorCodeEnum
)
from drillpulse_transport.heartbeat_manager import HeartbeatManager
from drillpulse_transport.session_logger import SessionCommunicationLogger
from drillpulse_transport.lora_transceiver_node import LoRaTransceiverNode
from drillpulse_camera.video_streamer_node import VideoStreamerNode
from drillpulse_mission.mission_manager_node import MissionManagerNode
from drillpulse_dashboard.dashboard_node import DrillPulseDashboardNode


class TestStep8RobustCommunication(unittest.TestCase):
    """Step 8 Comprehensive Verification Suite (20 Checks)."""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    # ------------------------------------------------------------
    # 01. GOOD Communication State
    # ------------------------------------------------------------
    def test_01_good_communication_state(self):
        """TEST 01: RSSI >= -75 dBm, loss < 15%, latency <= 120ms -> CONNECTED_GOOD (10 Hz)."""
        evaluator = ConnectionQualityEvaluator()
        state = evaluator.evaluate(
            rssi=-68.0,
            packet_loss=0.03,
            latency_ms=45.0,
            last_recv_time=time.time()
        )
        rate = evaluator.get_transmission_rate(state)
        self.assertEqual(state, LinkStateEnum.CONNECTED_GOOD)
        self.assertEqual(rate, 10.0)

    # ------------------------------------------------------------
    # 02. DEGRADED Communication State
    # ------------------------------------------------------------
    def test_02_degraded_communication_state(self):
        """TEST 02: RSSI >= -90 dBm, loss < 40%, latency <= 350ms -> CONNECTED_DEGRADED (3 Hz)."""
        evaluator = ConnectionQualityEvaluator()
        state = evaluator.evaluate(
            rssi=-84.0,
            packet_loss=0.22,
            latency_ms=180.0,
            last_recv_time=time.time()
        )
        rate = evaluator.get_transmission_rate(state)
        self.assertEqual(state, LinkStateEnum.CONNECTED_DEGRADED)
        self.assertEqual(rate, 3.0)

    # ------------------------------------------------------------
    # 03. WEAK Communication State
    # ------------------------------------------------------------
    def test_03_weak_communication_state(self):
        """TEST 03: RSSI >= -105 dBm, loss < 70%, latency <= 800ms -> CONNECTED_WEAK (1 Hz)."""
        evaluator = ConnectionQualityEvaluator()
        state = evaluator.evaluate(
            rssi=-98.0,
            packet_loss=0.55,
            latency_ms=450.0,
            last_recv_time=time.time()
        )
        rate = evaluator.get_transmission_rate(state)
        self.assertEqual(state, LinkStateEnum.CONNECTED_WEAK)
        self.assertEqual(rate, 1.0)

    # ------------------------------------------------------------
    # 04. INTERMITTENT Communication State
    # ------------------------------------------------------------
    def test_04_intermittent_communication_state(self):
        """TEST 04: Frequent timeouts / loss >= 70% or retries >= 3 -> INTERMITTENT (0.5 Hz)."""
        evaluator = ConnectionQualityEvaluator()
        state = evaluator.evaluate(
            rssi=-108.0,
            packet_loss=0.75,
            latency_ms=850.0,
            last_recv_time=time.time(),
            retry_count=4
        )
        rate = evaluator.get_transmission_rate(state)
        self.assertEqual(state, LinkStateEnum.INTERMITTENT)
        self.assertEqual(rate, 0.5)

    # ------------------------------------------------------------
    # 05. DISCONNECTED Communication State
    # ------------------------------------------------------------
    def test_05_disconnected_communication_state(self):
        """TEST 05: Heartbeat silence > 4.0s -> DISCONNECTED (0.2 Hz probe rate)."""
        evaluator = ConnectionQualityEvaluator(heartbeat_timeout_sec=4.0)
        state = evaluator.evaluate(
            rssi=-70.0,
            packet_loss=0.0,
            latency_ms=30.0,
            last_recv_time=time.time() - 5.5
        )
        rate = evaluator.get_transmission_rate(state)
        self.assertEqual(state, LinkStateEnum.DISCONNECTED)
        self.assertEqual(rate, 0.2)

    # ------------------------------------------------------------
    # 06. Recovery After Communication Loss
    # ------------------------------------------------------------
    def test_06_recovery_after_communication_loss(self):
        """TEST 06: Communication resuming after disconnect transitions to RECOVERING."""
        evaluator = ConnectionQualityEvaluator(recovery_window_sec=2.0)
        # 1. First trigger disconnect
        state1 = evaluator.evaluate(rssi=-120.0, packet_loss=1.0, latency_ms=999.0, last_recv_time=time.time() - 6.0)
        self.assertEqual(state1, LinkStateEnum.DISCONNECTED)

        # 2. Packets resume
        state2 = evaluator.evaluate(rssi=-72.0, packet_loss=0.02, latency_ms=40.0, last_recv_time=time.time())
        self.assertEqual(state2, LinkStateEnum.RECOVERING)
        rate = evaluator.get_transmission_rate(state2)
        self.assertEqual(rate, 2.0)

    # ------------------------------------------------------------
    # 07. Priority Queue Behavior
    # ------------------------------------------------------------
    def test_07_priority_queue_behavior(self):
        """TEST 07: Packets are dequeued in strict priority order (0 -> 1 -> 2 -> 3)."""
        q = PriorityTelemetryQueue(max_size=100)
        q.push(QueueItem("item_bulk", TelemetryPriority.BULK, "MAP", {}))
        q.push(QueueItem("item_norm", TelemetryPriority.NORMAL, "TELEM", {}))
        q.push(QueueItem("item_safe", TelemetryPriority.SAFETY, "STATE", {}))
        q.push(QueueItem("item_emerg", TelemetryPriority.EMERGENCY, "HAZARD", {}))

        popped = [q.pop_highest().priority for _ in range(4)]
        self.assertEqual(popped, [0, 1, 2, 3])

    # ------------------------------------------------------------
    # 08. Critical Message Preservation
    # ------------------------------------------------------------
    def test_08_critical_message_preservation(self):
        """TEST 08: When queue capacity is reached, Priority 0 (Critical) is NEVER discarded."""
        q = PriorityTelemetryQueue(max_size=50)
        # Fill with Priority 2 and 3
        for i in range(25):
            q.push(QueueItem(f"bulk_{i}", TelemetryPriority.BULK, "BULK", {}))
            q.push(QueueItem(f"norm_{i}", TelemetryPriority.NORMAL, "TELEM", {}))

        self.assertEqual(q.size(), 50)

        # Push 10 Priority 0 critical items
        for i in range(10):
            pushed = q.push(QueueItem(f"crit_{i}", TelemetryPriority.EMERGENCY, "CRITICAL", {}))
            self.assertTrue(pushed)

        self.assertEqual(q.size(), 50)
        # Verify all 10 critical items are present
        counts = q.size_by_priority()
        self.assertEqual(counts[TelemetryPriority.EMERGENCY], 10)
        self.assertGreater(q.total_dropped_low_pri, 0)

    # ------------------------------------------------------------
    # 09. Low-Priority Message Dropping
    # ------------------------------------------------------------
    def test_09_low_priority_message_dropping(self):
        """TEST 09: Saturation discards lowest-priority items (Priority 3 first)."""
        q = PriorityTelemetryQueue(max_size=10)
        for i in range(10):
            q.push(QueueItem(f"bulk_{i}", TelemetryPriority.BULK, "BULK", {}))

        # Push higher-priority item
        q.push(QueueItem("high_pri", TelemetryPriority.SAFETY, "STATE", {}))
        counts = q.size_by_priority()
        self.assertEqual(counts[TelemetryPriority.SAFETY], 1)
        self.assertEqual(counts[TelemetryPriority.BULK], 9)
        self.assertEqual(q.total_dropped_low_pri, 1)

    # ------------------------------------------------------------
    # 10. Heartbeat Timeout Detection
    # ------------------------------------------------------------
    def test_10_heartbeat_timeout(self):
        """TEST 10: HeartbeatManager flags timeout after inactivity period."""
        hbm = HeartbeatManager(timeout_sec=1.5)
        hbm.process_incoming_heartbeat({"rid": "ROVER_01"})
        self.assertFalse(hbm.is_timed_out())
        time.sleep(1.6)
        self.assertTrue(hbm.is_timed_out())

    # ------------------------------------------------------------
    # 11. Duplicate Command Protection (Idempotency)
    # ------------------------------------------------------------
    def test_11_duplicate_command_protection(self):
        """TEST 11: Duplicate command reception is detected and suppresses re-execution."""
        ack_mgr = AcknowledgementManager()
        cmd_id = "CMD_TEST_UNIQUE_01"

        # First arrival: new command
        is_dup1 = ack_mgr.is_duplicate_incoming(cmd_id)
        self.assertFalse(is_dup1)

        # Second arrival: duplicate command
        is_dup2 = ack_mgr.is_duplicate_incoming(cmd_id)
        self.assertTrue(is_dup2)

    # ------------------------------------------------------------
    # 12. Command Acknowledgement Lifecycle
    # ------------------------------------------------------------
    def test_12_command_acknowledgement(self):
        """TEST 12: Outgoing command transitions REQUESTED -> ACCEPTED -> COMPLETED on ACK."""
        ack_mgr = AcknowledgementManager(default_timeout_sec=2.0)
        cmd_id = "CMD_DISPATCH_88"

        entry = ack_mgr.register_outgoing(cmd_id, "GOTO_WAYPOINT", {"x": 5.0, "y": 2.0})
        self.assertEqual(entry.status, AckStatusEnum.REQUESTED)

        ack_mgr.process_incoming_ack(cmd_id, AckStatusEnum.ACCEPTED)
        self.assertEqual(ack_mgr.get_command_status(cmd_id), AckStatusEnum.ACCEPTED)

        ack_mgr.process_incoming_ack(cmd_id, AckStatusEnum.COMPLETED)
        self.assertEqual(ack_mgr.get_command_status(cmd_id), AckStatusEnum.COMPLETED)

    # ------------------------------------------------------------
    # 13. Return-to-Base Command
    # ------------------------------------------------------------
    def test_13_return_to_base_command(self):
        """TEST 13: RETURN_HOME / RETURN_TO_BASE command transitions rover to return state."""
        mgr = MissionManagerNode()
        mgr.current_x = 12.0
        mgr.current_y = 4.5
        mgr.start_x = 0.0
        mgr.start_y = 0.0
        mgr.start_pose_locked = True
        mgr.current_state = 'NAVIGATING'

        cmd = MissionCommand()
        cmd.command_id = "CMD_RTH_01"
        cmd.command_type = "RETURN_HOME"
        mgr.mission_cmd_callback(cmd)

        self.assertEqual(mgr.current_state, 'RETURNING')
        self.assertEqual(mgr.target_x, 0.0)
        self.assertEqual(mgr.target_y, 0.0)
        mgr.destroy_node()

    # ------------------------------------------------------------
    # 14. Communication-Loss Safety Behavior
    # ------------------------------------------------------------
    def test_14_communication_loss_safety_behavior(self):
        """TEST 14: Sustained communication loss in autonomy triggers autonomous return."""
        mgr = MissionManagerNode()
        mgr.current_state = 'NAVIGATING'
        mgr.active_job = 'EXPLORED'
        mgr.start_pose_locked = True

        link = LinkStatus()
        link.connected = False
        link.signal_strength_rssi = -125.0
        link.packet_loss_rate = 1.0

        # Trigger link loss
        mgr.link_callback(link)

        self.assertIn(mgr.current_state, ('RETURNING_COMM_LOSS', 'RETURNING'))
        mgr.destroy_node()

    # ------------------------------------------------------------
    # 15. Video Degradation & Suspension
    # ------------------------------------------------------------
    def test_15_video_degradation_and_suspension(self):
        """TEST 15: Wi-Fi degradation throttles FPS/resolution; disconnect suspends stream."""
        cam = VideoStreamerNode()
        self.assertEqual(cam.current_fps, 20.0)

        # 1. Degraded link -> 5 FPS, low quality
        cam._update_link_profile(connected=True, rssi=-92.0, loss=0.45)
        self.assertEqual(cam.link_state, "WEAK")
        self.assertEqual(cam.current_fps, 5.0)
        self.assertEqual(cam.jpeg_quality, 30)

        # 2. Lost link -> Suspended (0 FPS, zero interference with LoRa)
        cam._update_link_profile(connected=False, rssi=-125.0, loss=1.0)
        self.assertTrue(cam.video_suspended)
        self.assertEqual(cam.current_fps, 0.0)

        # 3. Restored link -> 20 FPS, high quality
        cam._update_link_profile(connected=True, rssi=-65.0, loss=0.05)
        self.assertFalse(cam.video_suspended)
        self.assertEqual(cam.current_fps, 20.0)
        self.assertEqual(cam.jpeg_quality, 75)
        cam.destroy_node()

    # ------------------------------------------------------------
    # 16. Telemetry Recovery & Burst Synchronization
    # ------------------------------------------------------------
    def test_16_telemetry_recovery_sync(self):
        """TEST 16: Reconnected link drains buffered priority packets in bursts."""
        q = PriorityTelemetryQueue(max_size=100)
        for i in range(15):
            q.push(QueueItem(f"item_{i}", TelemetryPriority.SAFETY if i < 5 else TelemetryPriority.NORMAL, "TELEM", {}))

        self.assertEqual(q.size(), 15)

        # Drain burst of 5 items
        burst = q.drain_batch(max_items=5)
        self.assertEqual(len(burst), 5)
        # Highest priority (SAFETY = 1) must be drained first
        self.assertTrue(all(item.priority == TelemetryPriority.SAFETY for item in burst))
        self.assertEqual(q.size(), 10)

    # ------------------------------------------------------------
    # 17. Local Buffering During Disconnection
    # ------------------------------------------------------------
    def test_17_local_buffering(self):
        """TEST 17: LoRa node buffers frames in PriorityQueue when link is LOST."""
        node = LoRaTransceiverNode()
        node.last_recv_time = time.time() - 10.0  # Force LOST state
        self.assertEqual(node.evaluate_link_state(), "LOST")

        node.send_frame("TELEMETRY", TelemetryPriority.NORMAL, {"test": "buffer"})
        self.assertEqual(node.priority_queue.size(), 1)
        item = node.priority_queue.peek_highest()
        self.assertIsNotNone(item)
        self.assertEqual(item.msg_type, "TELEMETRY")
        node.destroy_node()

    # ------------------------------------------------------------
    # 18. Session Communication Log Generation & Rotation
    # ------------------------------------------------------------
    def test_18_session_log_generation(self):
        """TEST 18: Communication events and connection changes are written to JSONL session log."""
        session_id = f"TEST-COMM-{int(time.time()*1000)}"
        logger = SessionCommunicationLogger(session_id=session_id)
        logger.log_connection_change("CONNECTED_GOOD", "CONNECTED_DEGRADED", -85.0, 0.25)
        logger.log_command_ack("CMD_99", "RETURN_HOME", "COMPLETED")
        logger.log_critical_alert("ROLLOVER_ALERT", "Rover rollover detected", (1.0, 2.0))
        logger.close()

        self.assertTrue(os.path.exists(logger.log_path))
        with open(logger.log_path, "r", encoding="utf-8") as f:
            lines = [json.loads(line) for line in f]

        event_types = [l.get("event_type") for l in lines]
        self.assertIn("SESSION_START", event_types)
        self.assertIn("CONNECTION_CHANGE", event_types)
        self.assertIn("COMMAND_ACK", event_types)
        self.assertIn("CRITICAL_ALERT", event_types)
        self.assertIn("SESSION_END", event_types)

    # ------------------------------------------------------------
    # 19. Dashboard Zero Fake Data Behavior
    # ------------------------------------------------------------
    def test_19_dashboard_zero_fake_data(self):
        """TEST 19: Disconnected or missing data yields explicit None, avoiding fake defaults."""
        dash = DrillPulseDashboardNode()
        # Force stale state
        dash.last_rover_seen = 0.0
        dash._emit_full_state()

        link = dash.latest_link
        self.assertEqual(link["link_state"], "LOST")
        self.assertEqual(link["overall_state"], "DISCONNECTED")
        self.assertIsNone(link["rssi"])
        self.assertIsNone(link["packet_loss"])
        self.assertIsNone(link["latency"])
        dash.destroy_node()

    # ------------------------------------------------------------
    # 20. Rover Autonomous Operation Without Dashboard
    # ------------------------------------------------------------
    def test_20_rover_operation_without_dashboard(self):
        """TEST 20: Rover mission manager and odometry operate safely without dashboard active."""
        mgr = MissionManagerNode()
        # Ingest odometry without any dashboard or station running
        odom = Odometry()
        odom.pose.pose.position.x = 3.5
        odom.pose.pose.position.y = 1.2
        mgr.odom_callback(odom)

        self.assertTrue(mgr.has_received_odom)
        self.assertTrue(mgr.start_pose_locked)
        self.assertEqual(mgr.start_x, 3.5)
        self.assertEqual(mgr.start_y, 1.2)
        # Rover remains in valid state, zero crash
        self.assertNotIn(mgr.current_state, ('ERROR', 'FAILED'))
        mgr.destroy_node()


if __name__ == '__main__':
    unittest.main()
