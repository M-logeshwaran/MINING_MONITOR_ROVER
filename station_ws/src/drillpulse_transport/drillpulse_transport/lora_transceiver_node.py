#!/usr/bin/env python3
# ============================================================
# DRILLPULSE — Unified Canonical LoRa Transceiver Node
# Platform: Ubuntu 24.04 LTS + ROS 2 Jazzy + Python 3.12
#
# Step 8 Robust Communication & Adaptive Telemetry Implementation:
#   - 4-Tier Priority Telemetry System (Priority 0-3)
#   - Priority-Aware Bounded Queue with Graceful Degradation Eviction
#   - Multi-Metric Connection Quality Evaluation (GOOD, DEGRADED, WEAK, INTERMITTENT, DISCONNECTED, RECOVERING)
#   - Reliable Command Acknowledgement & Idempotency Duplicate Suppression
#   - Compact Under-100-byte Heartbeat Protocol & Liveness Watchdog
#   - Bounded / Rotated Local SD Session Communication Logging
#   - Hardware Abstraction (Serial Hardware, UDP Simulation, Loopback)
#   - Decoupled Failure Resilience (Zero crashes to Nav/Odom/Sensors/Safety)
# ============================================================

import json
import math
import socket
import sys
import threading
import time
import zlib
from typing import Dict, Any, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from drillpulse_msgs.msg import (
    HazardEvent,
    JobAssignment,
    LinkStatus,
    MissionCommand,
    MissionState,
    RecoveryCommand,
    RecoveryStatus,
    RoverTelemetry,
    SafetyStatus
)

try:
    import serial
except ImportError:
    serial = None

try:
    from .priority_queue import PriorityTelemetryQueue, QueueItem, TelemetryPriority
    from .connection_state import ConnectionQualityEvaluator, LinkStateEnum, OverallLinkStateEnum
    from .acknowledgement_manager import AcknowledgementManager, AckStatusEnum, ErrorCodeEnum
    from .heartbeat_manager import HeartbeatManager
    from .session_logger import SessionCommunicationLogger
except ImportError:
    from drillpulse_transport.priority_queue import PriorityTelemetryQueue, QueueItem, TelemetryPriority
    from drillpulse_transport.connection_state import ConnectionQualityEvaluator, LinkStateEnum, OverallLinkStateEnum
    from drillpulse_transport.acknowledgement_manager import AcknowledgementManager, AckStatusEnum, ErrorCodeEnum
    from drillpulse_transport.heartbeat_manager import HeartbeatManager
    from drillpulse_transport.session_logger import SessionCommunicationLogger

PROTOCOL_VERSION = 2
MAX_PAYLOAD_BYTES = 220
DEFAULT_BATTERY_CAPACITY_AH = 10.0


class LoRaTransceiverNode(Node):
    """Canonical LoRa mesh transceiver adapter shared by Rover and Station."""

    def __init__(self):
        super().__init__('lora_transceiver_node')

        # Core Parameters
        self.declare_parameter('role', 'rover')
        self.declare_parameter('mode', 'UDP_SIM')
        self.declare_parameter('transport_mode', '')
        self.declare_parameter('serial_port', '/dev/ttyUSB2')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('local_port', 0)
        self.declare_parameter('local_udp_port', 0)
        self.declare_parameter('remote_port', 0)
        self.declare_parameter('remote_udp_port', 0)
        self.declare_parameter('remote_ip', '127.0.0.1')

        # Adaptive Telemetry Parameters
        self.declare_parameter('rssi_good_threshold', -75.0)
        self.declare_parameter('rssi_degraded_threshold', -90.0)
        self.declare_parameter('rssi_weak_threshold', -105.0)
        self.declare_parameter('rssi_critical_threshold', -115.0)
        self.declare_parameter('packet_loss_degraded', 0.15)
        self.declare_parameter('packet_loss_weak', 0.40)
        self.declare_parameter('packet_loss_critical', 0.70)
        self.declare_parameter('link_timeout_lost', 4.0)
        self.declare_parameter('rate_good', 10.0)
        self.declare_parameter('rate_degraded', 3.0)
        self.declare_parameter('rate_weak', 1.0)
        self.declare_parameter('rate_critical', 0.5)
        self.declare_parameter('rate_lost_probe', 0.2)
        self.declare_parameter('battery_capacity_ah', DEFAULT_BATTERY_CAPACITY_AH)
        self.declare_parameter('queue_max_size', 500)

        self.role = str(self.get_parameter('role').value).lower()
        mode_val = str(self.get_parameter('mode').value).strip()
        transport_mode_val = str(self.get_parameter('transport_mode').value).strip()
        self.mode = (transport_mode_val if transport_mode_val else mode_val).upper()
        if 'SIM' in self.mode or 'UDP' in self.mode:
            self.mode = 'UDP_SIM'
        elif 'SERIAL' in self.mode or 'UART' in self.mode:
            self.mode = 'SERIAL'

        self.serial_port = str(self.get_parameter('serial_port').value)
        self.serial_baud = int(self.get_parameter('serial_baud').value)

        loc_p = int(self.get_parameter('local_port').value)
        loc_udp_p = int(self.get_parameter('local_udp_port').value)
        rem_p = int(self.get_parameter('remote_port').value)
        rem_udp_p = int(self.get_parameter('remote_udp_port').value)

        if loc_p != 0:
            self.local_port = loc_p
        elif loc_udp_p != 0:
            self.local_port = loc_udp_p
        else:
            self.local_port = 9001 if self.role == 'rover' else 9002

        if rem_p != 0:
            self.remote_port = rem_p
        elif rem_udp_p != 0:
            self.remote_port = rem_udp_p
        else:
            self.remote_port = 9002 if self.role == 'rover' else 9001

        self.remote_ip = str(self.get_parameter('remote_ip').value)

        self.rssi_good_th = float(self.get_parameter('rssi_good_threshold').value)
        self.rssi_degraded_th = float(self.get_parameter('rssi_degraded_threshold').value)
        self.rssi_weak_th = float(self.get_parameter('rssi_weak_threshold').value)
        self.rssi_critical_th = float(self.get_parameter('rssi_critical_threshold').value)
        self.loss_degraded_th = float(self.get_parameter('packet_loss_degraded').value)
        self.loss_weak_th = float(self.get_parameter('packet_loss_weak').value)
        self.loss_critical_th = float(self.get_parameter('packet_loss_critical').value)
        self.link_timeout_lost = float(self.get_parameter('link_timeout_lost').value)
        self.rate_good = float(self.get_parameter('rate_good').value)
        self.rate_degraded = float(self.get_parameter('rate_degraded').value)
        self.rate_weak = float(self.get_parameter('rate_weak').value)
        self.rate_critical = float(self.get_parameter('rate_critical').value)
        self.rate_lost_probe = float(self.get_parameter('rate_lost_probe').value)
        self.battery_capacity_ah = float(self.get_parameter('battery_capacity_ah').value)
        queue_max = int(self.get_parameter('queue_max_size').value)

        # Connection Quality Evaluator & Priority Queue
        self.connection_evaluator = ConnectionQualityEvaluator(
            rssi_good=self.rssi_good_th,
            rssi_degraded=self.rssi_degraded_th,
            rssi_weak=self.rssi_weak_th,
            loss_degraded=self.loss_degraded_th,
            loss_weak=self.loss_weak_th,
            loss_critical=self.loss_critical_th,
            heartbeat_timeout_sec=self.link_timeout_lost
        )
        self.priority_queue = PriorityTelemetryQueue(max_size=queue_max)
        self.ack_mgr = AcknowledgementManager(default_timeout_sec=2.0, max_retries=3)
        self.heartbeat_mgr = HeartbeatManager(
            rover_id="ROVER_01" if self.role == 'rover' else "STATION_01",
            timeout_sec=self.link_timeout_lost
        )
        self.session_id = f"DRILLPULSE-{time.strftime('%Y%m%d-%H%M%S')}"
        self.session_logger = SessionCommunicationLogger(session_id=self.session_id)

        self.seq_num = 0
        self.seen_sequences = set()
        self.packets_sent = 0
        self.packets_recv = 0
        self.corrupt_packets = 0
        self.last_recv_time = time.time()
        self.last_telemetry_tx_time = 0.0
        self.last_probe_tx_time = 0.0
        self.simulated_rssi = -65.0
        self.simulated_snr = 9.5
        self.simulated_loss = 0.0
        self.simulated_latency = 45.0
        self.current_link_state = "GOOD"

        # Latest cache for compact heartbeats
        self.last_mode = "IDLE"
        self.last_fsm_state = "IDLE"
        self.last_battery_pct = 100.0
        self.last_runtime_min = 180.0
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0

        self.command_tracker = {}
        self.serial_conn = None
        self.udp_sock = None
        self.hardware_active = False

        if self.mode == 'SERIAL':
            if serial is None:
                self.get_logger().warn('pyserial not available, falling back to UDP_SIM')
                self.mode = 'UDP_SIM'
            else:
                try:
                    self.serial_conn = serial.Serial(self.serial_port, self.serial_baud, timeout=0.05)
                    self.hardware_active = True
                    self.get_logger().info(f'LoRa hardware serial connected on {self.serial_port} @ {self.serial_baud}')
                except Exception as ex:
                    self.get_logger().warn(f'Could not open {self.serial_port}: {ex}. Using UDP_SIM.')
                    self.mode = 'UDP_SIM'

        if self.mode == 'UDP_SIM':
            try:
                self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.udp_sock.bind(('0.0.0.0', self.local_port))
                self.udp_sock.settimeout(0.1)
                self.get_logger().info(f'LoRa UDP Simulation listening on {self.local_port}, target {self.remote_ip}:{self.remote_port}')
            except Exception as ex:
                self.get_logger().error(f'UDP socket bind failed on {self.local_port}: {ex}')

        # Publishers
        self.pub_telemetry = self.create_publisher(RoverTelemetry, '/rover/telemetry', 10)
        self.pub_link = self.create_publisher(LinkStatus, '/rover/link_status', 10)
        self.pub_link_compat = self.create_publisher(LinkStatus, '/drillpulse/link_status', 10)
        self.pub_state = self.create_publisher(MissionState, '/rover/mission_state', 10)
        self.pub_hazard = self.create_publisher(HazardEvent, '/rover/hazard_event', 10)
        self.pub_rec_cmd = self.create_publisher(RecoveryCommand, '/rover/recovery_command', 10)
        self.pub_rec_status = self.create_publisher(RecoveryStatus, '/rover/recovery_status', 10)
        self.pub_cmd = self.create_publisher(MissionCommand, '/rover/mission_command', 10)
        self.pub_job = self.create_publisher(JobAssignment, '/rover/job_assignment', 10)
        self.pub_estop = self.create_publisher(Bool, '/rover/emergency_stop', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/rover/cmd_vel', 10)

        # Legacy aliases
        self.pub_cmd_compat = self.create_publisher(MissionCommand, '/rover/mission_cmd', 10)
        self.pub_job_compat = self.create_publisher(JobAssignment, '/rover/job', 10)
        self.pub_job_ack_compat = self.create_publisher(JobAssignment, '/rover/job_ack', 10)
        self.pub_telemetry_compat = self.create_publisher(RoverTelemetry, '/drillpulse/telemetry', 10)

        if self.role == 'rover':
            self.create_subscription(RoverTelemetry, '/rover/telemetry', self.on_rover_telemetry, 10)
            self.create_subscription(RoverTelemetry, '/drillpulse/telemetry', self.on_rover_telemetry, 10)
            self.create_subscription(RoverTelemetry, '/drillpulse/rover_telemetry', self.on_rover_telemetry, 10)
            self.create_subscription(MissionState, '/rover/mission_state', self.on_rover_state, 10)
            self.create_subscription(HazardEvent, '/rover/hazard_event', self.on_rover_hazard, 10)
            self.create_subscription(HazardEvent, '/hazards/events', self.on_rover_hazard, 10)
            self.create_subscription(JobAssignment, '/rover/job_assignment_ack', self.on_rover_job_ack, 10)
            self.create_subscription(JobAssignment, '/rover/job_ack', self.on_rover_job_ack, 10)
            self.create_subscription(RecoveryStatus, '/rover/recovery_status', self.on_rover_rec_status, 10)
            self.create_subscription(Bool, '/rover/emergency_stop', self.on_rover_estop, 10)
        else:
            self.create_subscription(JobAssignment, '/rover/job_assignment', self.on_station_job, 10)
            self.create_subscription(JobAssignment, '/station/job_assignment', self.on_station_job, 10)
            self.create_subscription(MissionCommand, '/rover/mission_command', self.on_station_cmd, 10)
            self.create_subscription(MissionCommand, '/rover/mission_cmd', self.on_station_cmd, 10)
            self.create_subscription(MissionCommand, '/drillpulse/mission_command', self.on_station_cmd, 10)
            self.create_subscription(RecoveryCommand, '/rover/recovery_command', self.on_station_rec, 10)
            self.create_subscription(RecoveryCommand, '/recovery/command', self.on_station_rec, 10)
            self.create_subscription(Bool, '/rover/emergency_stop', self.on_station_estop, 10)
            self.create_subscription(Twist, '/rover/cmd_vel', self.on_station_teleop, 10)

        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

        self.heartbeat_timer = self.create_timer(1.0, self._publish_link_status_and_watchdog)
        self.get_logger().info(f'Canonical LoRa Transceiver active [Role: {self.role}, Transport: {self.mode}]')

    def evaluate_link_state(self) -> str:
        """Returns standard link quality state (GOOD, DEGRADED, WEAK, CRITICAL, LOST)."""
        now = time.time()
        elapsed_since_recv = now - self.last_recv_time

        if elapsed_since_recv > self.link_timeout_lost:
            return "LOST"

        rssi = self.simulated_rssi
        loss = self.simulated_loss

        if rssi >= self.rssi_good_th and loss < self.loss_degraded_th:
            return "GOOD"
        elif rssi >= self.rssi_degraded_th and loss < self.loss_weak_th:
            return "DEGRADED"
        elif rssi >= self.rssi_weak_th and loss < self.loss_critical_th:
            return "WEAK"
        else:
            return "CRITICAL"

    def evaluate_detailed_state(self) -> str:
        """Returns canonical Step 8 communication state."""
        return self.connection_evaluator.evaluate(
            rssi=self.simulated_rssi,
            packet_loss=self.simulated_loss,
            latency_ms=self.simulated_latency,
            last_recv_time=self.last_recv_time,
            retry_count=0,
            queue_backlog=self.priority_queue.size(),
            link_hardware_ok=True
        )

    def get_adaptive_rate(self, state: str) -> float:
        """Maps any state string (standard or detailed) to transmission frequency."""
        if state in ("GOOD", LinkStateEnum.CONNECTED_GOOD):
            return self.rate_good
        elif state in ("DEGRADED", LinkStateEnum.CONNECTED_DEGRADED):
            return self.rate_degraded
        elif state in ("WEAK", LinkStateEnum.CONNECTED_WEAK):
            return self.rate_weak
        elif state in ("CRITICAL", LinkStateEnum.INTERMITTENT):
            return self.rate_critical
        elif state == LinkStateEnum.RECOVERING:
            return 2.0
        else:
            return self.rate_lost_probe

    def send_frame(self, msg_type: str, priority: int, payload: dict):
        """Transmits a frame over LoRa wire, or buffers in PriorityQueue when disconnected."""
        link_state = self.evaluate_link_state()
        item_id = str(payload.get('id', f"{msg_type}_{int(time.time()*1000)}"))

        # If completely disconnected, buffer into local priority queue
        if link_state == "LOST":
            q_item = QueueItem(item_id, priority, msg_type, payload, self.session_id)
            self.priority_queue.push(q_item)
            return

        self._transmit_wire_frame(msg_type, priority, payload)

    def _transmit_wire_frame(self, msg_type: str, priority: int, payload: dict):
        self.seq_num = (self.seq_num + 1) & 0xFFFF
        source_id = "ROVER_01" if self.role == 'rover' else "STATION_01"
        dest_id = "STATION_01" if self.role == 'rover' else "ROVER_01"

        payload_str = json.dumps(payload, separators=(',', ':'))
        if len(payload_str.encode('utf-8')) > MAX_PAYLOAD_BYTES:
            essential_keys = ['st', 'bat', 'x', 'y', 'cmd', 'id', 'estop', 'job', 'stat', 'type', 'sev']
            trimmed = {k: payload[k] for k in essential_keys if k in payload}
            payload = trimmed
            payload_str = json.dumps(payload, separators=(',', ':'))

        crc = zlib.crc32(payload_str.encode('utf-8')) & 0xFFFFFFFF
        frame = {
            "v": PROTOCOL_VERSION,
            "type": msg_type,
            "src": source_id,
            "dst": dest_id,
            "seq": self.seq_num,
            "pri": priority,
            "ts": round(time.time(), 3),
            "data": payload,
            "crc": crc
        }

        wire_bytes = json.dumps(frame, separators=(',', ':')).encode('utf-8') + b'\n'

        if self.mode == 'SERIAL' and self.serial_conn:
            try:
                self.serial_conn.write(wire_bytes)
                self.packets_sent += 1
            except Exception as ex:
                self.get_logger().warn(f'LoRa serial TX error: {ex}')
        elif self.udp_sock:
            try:
                self.udp_sock.sendto(wire_bytes, (self.remote_ip, self.remote_port))
                self.packets_sent += 1
            except Exception as ex:
                self.get_logger().warn(f'LoRa UDP TX error: {ex}')

    def on_rover_telemetry(self, msg: RoverTelemetry):
        now = time.time()
        link_state = self.evaluate_link_state()
        max_rate = self.get_adaptive_rate(link_state)
        min_interval = 1.0 / max(0.1, max_rate)

        if (now - self.last_telemetry_tx_time) < min_interval:
            return
        self.last_telemetry_tx_time = now

        soc = max(0.0, min(1.0, msg.battery_percentage / 100.0))
        effective_curr = max(0.5, float(msg.current_draw))
        runtime_min = round((soc * self.battery_capacity_ah) / effective_curr * 60.0, 1)

        self.last_battery_pct = float(msg.battery_percentage)
        self.last_runtime_min = runtime_min
        self.last_pose_x = float(msg.pose_x)
        self.last_pose_y = float(msg.pose_y)
        self.last_fsm_state = msg.current_state
        self.last_mode = msg.active_job

        if link_state == 'CRITICAL':
            payload = {
                'bat': round(msg.battery_percentage, 1),
                'runtime': runtime_min,
                'x': round(msg.pose_x, 2),
                'y': round(msg.pose_y, 2),
                'st': msg.current_state,
                'estop': msg.emergency_stopped,
                'state_link': link_state
            }
        elif link_state == 'WEAK':
            payload = {
                'bat': round(msg.battery_percentage, 1),
                'volt': round(msg.battery_voltage, 1),
                'curr': round(msg.current_draw, 1),
                'runtime': runtime_min,
                'vx': round(msg.linear_velocity, 2),
                'pitch': round(msg.pitch_angle, 1),
                'roll': round(msg.roll_angle, 1),
                'yaw': round(msg.yaw_angle, 2),
                'x': round(msg.pose_x, 2),
                'y': round(msg.pose_y, 2),
                'st': msg.current_state,
                'job': msg.active_job,
                'estop': msg.emergency_stopped,
                'state_link': link_state
            }
        else:
            payload = {
                'bat': round(msg.battery_percentage, 1),
                'volt': round(msg.battery_voltage, 1),
                'curr': round(msg.current_draw, 1),
                'runtime': runtime_min,
                'vx': round(msg.linear_velocity, 2),
                'wz': round(msg.angular_velocity, 2),
                'pitch': round(msg.pitch_angle, 1),
                'roll': round(msg.roll_angle, 1),
                'yaw': round(msg.yaw_angle, 2),
                'x': round(msg.pose_x, 2),
                'y': round(msg.pose_y, 2),
                'temp': round(msg.temperature, 1),
                'hum': round(msg.humidity, 1),
                'gas': round(msg.gas_ppm, 1),
                'ch4': round(msg.methane_ppm, 2),
                'co': round(msg.carbon_monoxide_ppm, 2),
                'o2': round(msg.oxygen_percentage, 1),
                'dust': round(msg.dust_concentration, 2),
                'st': msg.current_state,
                'job': msg.active_job,
                'estop': msg.emergency_stopped,
                'state_link': link_state
            }

        self.send_frame('TELEMETRY', TelemetryPriority.NORMAL, payload)

    def on_rover_state(self, msg: MissionState):
        self.last_fsm_state = msg.current_state
        self.last_mode = msg.active_job
        self.send_frame('MISSION_STATE', TelemetryPriority.SAFETY, {
            'id': msg.mission_id,
            'st': msg.current_state,
            'job': msg.active_job,
            'phase': msg.current_phase,
            'pct': round(msg.progress_percentage, 1),
            'x': round(msg.current_x, 2),
            'y': round(msg.current_y, 2),
            'dist': round(msg.distance_to_target, 2),
            'frontiers': msg.discovered_frontiers_count
        })

    def on_rover_hazard(self, msg: HazardEvent):
        # Priority 0: Critical Hazard Event
        self.send_frame('HAZARD_EVENT', TelemetryPriority.EMERGENCY, {
            'id': msg.event_id,
            'type': msg.event_type,
            'sev': msg.severity,
            'desc': msg.description,
            'val': round(msg.measured_value, 2),
            'th': round(msg.threshold_value, 2),
            'x': round(msg.location_x, 2),
            'y': round(msg.location_y, 2)
        })
        self.session_logger.log_critical_alert(msg.event_type, msg.description, (msg.location_x, msg.location_y))

    def on_rover_job_ack(self, msg: JobAssignment):
        self.send_frame('JOB_ACK', TelemetryPriority.SAFETY, {
            'id': msg.job_id,
            'job': msg.assigned_job,
            'stat': msg.status,
            'map': msg.map_name
        })

    def on_rover_rec_status(self, msg: RecoveryStatus):
        self.send_frame('RECOVERY_STATUS', TelemetryPriority.EMERGENCY, {
            'st': msg.state,
            'side': msg.active_side,
            'roll': round(msg.current_roll, 1),
            'pos': round(msg.actuator_position, 2),
            'fault': msg.fault,
            'cool': round(msg.cooldown_remaining, 1)
        })

    def on_rover_estop(self, msg: Bool):
        if msg.data:
            self.send_frame('SAFETY_EVENT', TelemetryPriority.EMERGENCY, {'estop': True, 'src': 'ROVER'})
            self.session_logger.log_critical_alert("EMERGENCY_STOP", "Rover emergency stop engaged", (self.last_pose_x, self.last_pose_y))

    def on_station_job(self, msg: JobAssignment):
        cmd_id = msg.job_id if msg.job_id else f"JOB_{int(time.time()*1000)}"
        payload = {
            'id': cmd_id,
            'job': msg.assigned_job,
            'stat': msg.status if msg.status else 'ASSIGNED',
            'map': msg.map_name
        }
        self.ack_mgr.register_outgoing(cmd_id, 'JOB_ASSIGN', payload)
        self.command_tracker[cmd_id] = {'status': 'SENT', 'timestamp': time.time(), 'type': 'JOB_ASSIGN'}
        self.send_frame('JOB_ASSIGN', TelemetryPriority.SAFETY, payload)
        self.session_logger.log_event("COMMAND_SENT", payload)

    def on_station_cmd(self, msg: MissionCommand):
        cmd_id = msg.command_id if msg.command_id else f"CMD_{int(time.time()*1000)}"
        payload = {
            'id': cmd_id,
            'cmd': msg.command_type,
            'x': round(msg.target_x, 2),
            'y': round(msg.target_y, 2),
            'yaw': round(msg.target_yaw, 2),
            'estop': msg.emergency_stop
        }
        pri = TelemetryPriority.EMERGENCY if msg.command_type in ("EMERGENCY_STOP", "RETURN_HOME") or msg.emergency_stop else TelemetryPriority.SAFETY
        self.ack_mgr.register_outgoing(cmd_id, msg.command_type, payload)
        self.command_tracker[cmd_id] = {'status': 'SENT', 'timestamp': time.time(), 'type': msg.command_type}
        self.send_frame('COMMAND', pri, payload)
        self.session_logger.log_event("COMMAND_SENT", payload)

    def on_station_rec(self, msg: RecoveryCommand):
        cmd_id = f"REC_{int(time.time()*1000)}"
        payload = {
            'id': cmd_id,
            'cmd': msg.command_type,
            'ext': round(msg.actuator_extension, 2),
            'ovr': msg.force_override
        }
        self.ack_mgr.register_outgoing(cmd_id, msg.command_type, payload)
        self.command_tracker[cmd_id] = {'status': 'SENT', 'timestamp': time.time(), 'type': msg.command_type}
        self.send_frame('RECOVERY_COMMAND', TelemetryPriority.EMERGENCY, payload)

    def on_station_estop(self, msg: Bool):
        if msg.data:
            self.send_frame('SAFETY_EVENT', TelemetryPriority.EMERGENCY, {'estop': True, 'src': 'STATION'})

    def on_station_teleop(self, msg: Twist):
        self.send_frame('TELEOP', TelemetryPriority.NORMAL, {
            'vx': round(msg.linear.x, 2),
            'wz': round(msg.angular.z, 2)
        })

    def _rx_loop(self):
        while self.running:
            raw = None
            if self.mode == 'SERIAL' and self.serial_conn:
                try:
                    if self.serial_conn.in_waiting:
                        raw = self.serial_conn.readline()
                except Exception:
                    pass
            elif self.udp_sock:
                try:
                    data, _ = self.udp_sock.recvfrom(2048)
                    raw = data
                except socket.timeout:
                    continue
                except Exception:
                    continue

            if not raw:
                continue

            try:
                line = raw.decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                frame = json.loads(line)

                if frame.get('v') != PROTOCOL_VERSION:
                    continue

                data = frame.get('data', {})
                data_str = json.dumps(data, separators=(',', ':'))
                expected_crc = zlib.crc32(data_str.encode('utf-8')) & 0xFFFFFFFF
                if frame.get('crc') != expected_crc:
                    self.corrupt_packets += 1
                    continue

                seq = frame.get('seq', 0)
                if seq in self.seen_sequences:
                    continue
                self.seen_sequences.add(seq)
                if len(self.seen_sequences) > 1000:
                    self.seen_sequences.clear()

                ts = frame.get('ts', 0.0)
                now = time.time()
                if (now - ts) > 5.0:
                    continue

                self.packets_recv += 1
                self.last_recv_time = now
                self._dispatch_rx_frame(frame.get('type'), data)
            except Exception:
                self.corrupt_packets += 1

    def _dispatch_rx_frame(self, msg_type: str, data: dict):
        now = self.get_clock().now().to_msg()

        # Handle Heartbeat
        if msg_type == 'HEARTBEAT':
            self.heartbeat_mgr.process_incoming_heartbeat(data)
            return

        if msg_type in ('JOB_ASSIGN', 'JOB') and self.role == 'rover':
            cmd_id = str(data.get('id', 'JOB_01'))

            # Idempotency check: prevent duplicate job assignment
            if self.ack_mgr.is_duplicate_incoming(cmd_id):
                self.get_logger().info(f"Duplicate Job {cmd_id} received. Suppressing execution, re-ACKing.")
                self.send_frame('CMD_ACK', TelemetryPriority.SAFETY, {
                    'id': cmd_id,
                    'status': 'ACCEPTED',
                    'error_code': ErrorCodeEnum.IDEMPOTENT_DUPLICATE
                })
                return

            job = JobAssignment()
            job.header.stamp = now
            job.job_id = cmd_id
            job.assigned_job = str(data.get('job', 'NONE')).upper()
            job.status = str(data.get('stat', 'ASSIGNED'))
            job.map_name = str(data.get('map', ''))
            self.pub_job.publish(job)
            self.pub_job_compat.publish(job)

            self.send_frame('CMD_ACK', TelemetryPriority.SAFETY, {
                'id': cmd_id,
                'status': 'RECEIVED',
                'job': job.assigned_job,
                'map': job.map_name,
                'error_code': ErrorCodeEnum.NONE
            })

        elif msg_type in ('JOB_ACK', 'ACK') and self.role == 'station':
            cmd_id = str(data.get('id', ''))
            status = str(data.get('stat', 'JOB_ACCEPTED'))
            ack = JobAssignment()
            ack.header.stamp = now
            ack.job_id = cmd_id
            ack.assigned_job = str(data.get('job', 'NONE')).upper()
            ack.status = status
            ack.map_name = str(data.get('map', ''))
            self.pub_job.publish(ack)
            self.pub_job_ack_compat.publish(ack)

            if cmd_id in self.command_tracker:
                self.command_tracker[cmd_id]['status'] = ack.status
            self.ack_mgr.process_incoming_ack(cmd_id, status)
            self.session_logger.log_command_ack(cmd_id, 'JOB_ASSIGN', status)

        elif msg_type == 'COMMAND' and self.role == 'rover':
            cmd_id = str(data.get('id', 'CMD_00'))
            cmd_type = str(data.get('cmd', ''))

            # Idempotency check: prevent duplicate command execution
            if self.ack_mgr.is_duplicate_incoming(cmd_id):
                self.get_logger().info(f"Duplicate Command {cmd_id} received. Suppressing execution, re-ACKing.")
                self.send_frame('CMD_ACK', TelemetryPriority.SAFETY, {
                    'id': cmd_id,
                    'status': 'ACCEPTED',
                    'error_code': ErrorCodeEnum.IDEMPOTENT_DUPLICATE
                })
                return

            cmd = MissionCommand()
            cmd.header.stamp = now
            cmd.command_id = cmd_id
            cmd.command_type = cmd_type
            cmd.target_x = float(data.get('x', 0.0))
            cmd.target_y = float(data.get('y', 0.0))
            cmd.target_yaw = float(data.get('yaw', 0.0))
            cmd.emergency_stop = bool(data.get('estop', False))
            self.pub_cmd.publish(cmd)
            self.pub_cmd_compat.publish(cmd)

            self.send_frame('CMD_ACK', TelemetryPriority.SAFETY, {
                'id': cmd_id,
                'status': 'RECEIVED',
                'cmd': cmd.command_type,
                'error_code': ErrorCodeEnum.NONE
            })

        elif msg_type == 'CMD_ACK' and self.role == 'station':
            cmd_id = str(data.get('id', ''))
            status = str(data.get('status', 'ACCEPTED'))
            err_code = str(data.get('error_code', ErrorCodeEnum.NONE))
            if cmd_id in self.command_tracker:
                self.command_tracker[cmd_id]['status'] = status
            self.ack_mgr.process_incoming_ack(cmd_id, status, err_code)
            self.session_logger.log_command_ack(cmd_id, 'COMMAND', status, err_code)

        elif msg_type in ('RECOVERY_COMMAND', 'REC') and self.role == 'rover':
            cmd_id = str(data.get('id', 'REC_00'))
            cmd_type = str(data.get('cmd', 'STOP_RECOVERY'))

            if self.ack_mgr.is_duplicate_incoming(cmd_id):
                self.send_frame('CMD_ACK', TelemetryPriority.EMERGENCY, {
                    'id': cmd_id,
                    'status': 'ACCEPTED',
                    'error_code': ErrorCodeEnum.IDEMPOTENT_DUPLICATE
                })
                return

            rec = RecoveryCommand()
            rec.header.stamp = now
            rec.command_type = cmd_type
            rec.actuator_extension = float(data.get('ext', 0.0))
            rec.force_override = bool(data.get('ovr', False))
            self.pub_rec_cmd.publish(rec)

            self.send_frame('CMD_ACK', TelemetryPriority.EMERGENCY, {
                'id': cmd_id,
                'status': 'RECEIVED',
                'cmd': rec.command_type,
                'error_code': ErrorCodeEnum.NONE
            })

        elif msg_type == 'RECOVERY_STATUS' and self.role == 'station':
            st = RecoveryStatus()
            st.header.stamp = now
            st.state = str(data.get('st', 'IDLE'))
            st.active_side = str(data.get('side', 'NONE'))
            st.current_roll = float(data.get('roll', 0.0))
            st.actuator_position = float(data.get('pos', 0.0))
            st.fault = bool(data.get('fault', False))
            st.cooldown_remaining = float(data.get('cool', 0.0))
            self.pub_rec_status.publish(st)

        elif msg_type in ('MISSION_STATE', 'STATE') and self.role == 'station':
            st = MissionState()
            st.header.stamp = now
            st.mission_id = str(data.get('id', 'MISSION_01'))
            st.current_state = str(data.get('st', 'IDLE'))
            st.active_job = str(data.get('job', 'NONE'))
            st.current_phase = str(data.get('phase', 'IDLE'))
            st.progress_percentage = float(data.get('pct', 0.0))
            st.current_x = float(data.get('x', 0.0))
            st.current_y = float(data.get('y', 0.0))
            st.distance_to_target = float(data.get('dist', 0.0))
            st.discovered_frontiers_count = int(data.get('frontiers', 0))
            self.pub_state.publish(st)

        elif msg_type in ('TELEMETRY', 'TELEM') and self.role == 'station':
            telem = RoverTelemetry()
            telem.header.stamp = now
            telem.battery_percentage = float(data.get('bat', 0.0))
            telem.battery_voltage = float(data.get('volt', 0.0))
            telem.current_draw = float(data.get('curr', 0.0))
            telem.linear_velocity = float(data.get('vx', 0.0))
            telem.angular_velocity = float(data.get('wz', 0.0))
            telem.pitch_angle = float(data.get('pitch', 0.0))
            telem.roll_angle = float(data.get('roll', 0.0))
            telem.yaw_angle = float(data.get('yaw', 0.0))
            telem.pose_x = float(data.get('x', 0.0))
            telem.pose_y = float(data.get('y', 0.0))
            telem.temperature = float(data.get('temp', 0.0))
            telem.humidity = float(data.get('hum', 0.0))
            telem.gas_ppm = float(data.get('gas', 0.0))
            telem.methane_ppm = float(data.get('ch4', 0.0))
            telem.carbon_monoxide_ppm = float(data.get('co', 0.0))
            telem.oxygen_percentage = float(data.get('o2', 0.0))
            telem.dust_concentration = float(data.get('dust', 0.0))
            telem.current_state = str(data.get('st', 'IDLE'))
            telem.active_job = str(data.get('job', 'NONE'))
            telem.emergency_stopped = bool(data.get('estop', False))
            telem.lora_rssi = float(self.simulated_rssi)
            self.pub_telemetry.publish(telem)
            self.pub_telemetry_compat.publish(telem)

        elif msg_type == 'SAFETY_EVENT':
            estop_active = bool(data.get('estop', True))
            b = Bool()
            b.data = estop_active
            self.pub_estop.publish(b)

        elif msg_type == 'TELEOP' and self.role == 'rover':
            tw = Twist()
            tw.linear.x = float(data.get('vx', 0.0))
            tw.angular.z = float(data.get('wz', 0.0))
            self.pub_cmd_vel.publish(tw)

        elif msg_type == 'HAZARD_EVENT' and self.role == 'station':
            hz = HazardEvent()
            hz.header.stamp = now
            hz.event_id = str(data.get('id', 'HZ_00'))
            hz.event_type = str(data.get('type', 'HAZARD'))
            hz.severity = str(data.get('sev', 'WARNING'))
            hz.description = str(data.get('desc', ''))
            hz.measured_value = float(data.get('val', 0.0))
            hz.threshold_value = float(data.get('th', 0.0))
            hz.location_x = float(data.get('x', 0.0))
            hz.location_y = float(data.get('y', 0.0))
            self.pub_hazard.publish(hz)

        elif msg_type == 'PROBE':
            self.send_frame('PROBE_ACK', TelemetryPriority.NORMAL, {'echo': True, 'ts': time.time()})

    def _publish_link_status_and_watchdog(self):
        now = time.time()
        link_state = self.evaluate_link_state()
        old_state = self.current_link_state
        self.current_link_state = link_state
        is_connected = (link_state != 'LOST')

        if old_state != link_state:
            self.session_logger.log_connection_change(old_state, link_state, self.simulated_rssi, self.simulated_loss)

        # Transmit compact heartbeat
        hb_payload = self.heartbeat_mgr.create_heartbeat_payload(
            session_id=self.session_id,
            rover_mode=self.last_mode,
            mission_state=self.last_fsm_state,
            battery_pct=self.last_battery_pct,
            runtime_min=self.last_runtime_min,
            comm_state=link_state,
            pose_x=self.last_pose_x,
            pose_y=self.last_pose_y
        )
        self._transmit_wire_frame('HEARTBEAT', TelemetryPriority.SAFETY, hb_payload)

        # Sync buffered items from PriorityQueue if link is active
        if is_connected and self.priority_queue.size() > 0:
            burst = self.priority_queue.drain_batch(max_items=5)
            for item in burst:
                self._transmit_wire_frame(item.msg_type, item.priority, item.payload)

        if not is_connected and (now - self.last_probe_tx_time) >= (1.0 / self.rate_lost_probe):
            self.last_probe_tx_time = now
            self._transmit_wire_frame('PROBE', TelemetryPriority.NORMAL, {'ping': True, 'role': self.role})

        # Retry unacknowledged commands
        to_retry, timed_out = self.ack_mgr.check_retries_and_timeouts()
        for cmd_entry in to_retry:
            self.get_logger().warn(f"Retrying unacknowledged command {cmd_entry.cmd_id} (Attempt {cmd_entry.retry_count}/{cmd_entry.max_retries})")
            pri = TelemetryPriority.EMERGENCY if cmd_entry.cmd_type in ("EMERGENCY_STOP", "RETURN_HOME") else TelemetryPriority.SAFETY
            self._transmit_wire_frame('COMMAND', pri, cmd_entry.payload)

        for cmd_entry in timed_out:
            self.get_logger().error(f"Command {cmd_entry.cmd_id} ({cmd_entry.cmd_type}) TIMED OUT after {cmd_entry.max_retries} retries.")
            if cmd_entry.cmd_id in self.command_tracker:
                self.command_tracker[cmd_entry.cmd_id]['status'] = 'TIMEOUT'

        link = LinkStatus()
        link.header.stamp = self.get_clock().now().to_msg()
        mode_str = 'LORA_SERIAL' if self.hardware_active else 'UDP_SIMULATION'
        link.link_type = f"{mode_str} [{link_state}]"
        link.connected = is_connected

        if link_state == 'GOOD':
            link.signal_strength_rssi = self.simulated_rssi
            link.snr_db = self.simulated_snr
            link.packet_loss_rate = self.simulated_loss
            link.latency_ms = 45.0
        elif link_state == 'DEGRADED':
            link.signal_strength_rssi = -82.0
            link.snr_db = 4.0
            link.packet_loss_rate = 0.22
            link.latency_ms = 120.0
        elif link_state == 'WEAK':
            link.signal_strength_rssi = -98.0
            link.snr_db = -2.0
            link.packet_loss_rate = 0.55
            link.latency_ms = 350.0
        elif link_state == 'CRITICAL':
            link.signal_strength_rssi = -112.0
            link.snr_db = -8.0
            link.packet_loss_rate = 0.80
            link.latency_ms = 850.0
        else:
            link.signal_strength_rssi = -125.0
            link.snr_db = -18.0
            link.packet_loss_rate = 1.0
            link.latency_ms = 9999.0

        link.packets_sent = self.packets_sent
        link.packets_received = self.packets_recv

        self.pub_link.publish(link)
        self.pub_link_compat.publish(link)

    def destroy_node(self):
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        if self.udp_sock:
            self.udp_sock.close()
        self.session_logger.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LoRaTransceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
