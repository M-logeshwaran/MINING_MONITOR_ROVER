#!/usr/bin/env python3
# =========================================================
# FILE: dashboard_node.py
# PURPOSE: Full-Stack Web Dashboard Server & ROS 2 Bridge for
#          DrillPulse Operator Control. Enforces dual-panel
#          architecture: Explored vs Unexplored modes, where the
#          active panel turns GREEN only upon Rover ACK, and the
#          inactive panel is strictly disabled/blanked.
#          Implements zero fake data, 5-state link handling,
#          command lifecycle tracking, interactive click-to-point,
#          manual teleoperation, recovery shortcuts, and decoupled
#          high-bandwidth video streaming.
# =========================================================

import os
import base64
import time
import math
import threading
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import Image, CompressedImage

from drillpulse_msgs.msg import (
    RoverTelemetry,
    OdometryDiagnostics,
    MissionState,
    MissionCommand,
    HazardEvent,
    LinkStatus,
    JobAssignment,
    RecoveryCommand,
    RecoveryStatus,
    SafetyStatus
)

from flask import Flask, jsonify, send_from_directory, request
from flask_socketio import SocketIO
from ament_index_python.packages import get_package_share_directory

WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
try:
    pkg_share = get_package_share_directory('drillpulse_dashboard')
    share_web = os.path.join(pkg_share, 'web')
    if os.path.exists(share_web):
        WEB_DIR = share_web
except Exception:
    pass

DEFAULT_PORT = 8080
MAP_YAML_PATH = "/home/loki/SIH_REPO_FINAL/maps/rover_map.yaml"
REPORT_DIR = "/home/loki/SIH_REPO_FINAL/reports"
STATE_EMIT_RATE_HZ = 10.0


class DrillPulseDashboardNode(Node):
    """ROS 2 Node bridging DrillPulse telemetry and commands to the dual-panel web dashboard."""

    def __init__(self):
        super().__init__('drillpulse_dashboard_node')

        self.declare_parameter('web_port', DEFAULT_PORT)
        self.declare_parameter('map_yaml', MAP_YAML_PATH)
        self.declare_parameter('report_dir', REPORT_DIR)

        self.web_port = int(self.get_parameter('web_port').value)
        self.map_yaml = str(self.get_parameter('map_yaml').value)
        self.report_dir = str(self.get_parameter('report_dir').value)

        self.app = Flask(__name__, static_folder=WEB_DIR, static_url_path="")
        self.app.config["SECRET_KEY"] = "drillpulse-mine-secret"
        self.socketio = SocketIO(self.app, async_mode="threading", cors_allowed_origins="*")

        self._map_cache = None
        self._slam_map_cache = None
        self._lock = threading.Lock()

        self.requested_mode = "NONE"
        self.rover_acknowledged_mode = "NONE"
        self.rover_state = "IDLE"
        self.last_rover_seen = 0.0
        self.last_video_frame_time = 0.0

        self.latest_telemetry = None
        self.telemetry_history = []

        self.latest_diagnostics = {
            "fusion_state": "UNKNOWN",
            "source": "UNKNOWN",
            "vel_err": 0.0,
            "yaw_err": 0.0,
            "enc_valid": False,
            "imu_valid": False,
            "enc_conf": 0.0,
            "imu_conf": 0.0
        }

        self.latest_link = {
            "connected": False,
            "link_state": "LOST",
            "overall_state": "DISCONNECTED",
            "rssi": None,
            "snr": None,
            "packet_loss": None,
            "latency": None,
            "transport": "LORA_MESH",
            "video_online": False,
            "last_packet_age": 999.0
        }

        self.recent_hazards = []
        self.recent_recovery_status = "READY"
        self.latest_plan_points = []
        self.frontiers_count = 0
        self.active_commands = {}
        self.latest_safety = {
            "safety_state": "NORMAL",
            "motion_allowed": True,
            "emergency_stop_active": False,
            "obstacle_distance_m": 99.0
        }

        qos_transient = QoSProfile(depth=1)
        qos_transient.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(RoverTelemetry, '/rover/telemetry', self._on_telemetry, 10)
        self.create_subscription(RoverTelemetry, '/drillpulse/telemetry', self._on_telemetry, 10)
        self.create_subscription(RoverTelemetry, '/drillpulse/rover_telemetry', self._on_telemetry, 10)

        self.create_subscription(SafetyStatus, '/rover/safety_status', self._on_safety_status, 10)
        self.create_subscription(SafetyStatus, '/drillpulse/safety_status', self._on_safety_status, 10)

        self.create_subscription(OdometryDiagnostics, '/odom/diagnostics', self._on_diagnostics, 10)

        self.create_subscription(MissionState, '/rover/mission_state', self._on_mission_state, 10)
        self.create_subscription(MissionState, '/mission/state', self._on_mission_state, 10)

        self.create_subscription(HazardEvent, '/rover/hazard_event', self._on_hazard, 10)
        self.create_subscription(HazardEvent, '/drillpulse/hazard_event', self._on_hazard, 10)

        self.create_subscription(LinkStatus, '/rover/link_status', self._on_link_status, 10)
        self.create_subscription(LinkStatus, '/drillpulse/link_status', self._on_link_status, 10)

        self.create_subscription(JobAssignment, '/rover/job_assignment_ack', self._on_job_ack, 10)
        self.create_subscription(JobAssignment, '/rover/job_ack', self._on_job_ack, 10)

        self.create_subscription(RecoveryStatus, '/rover/recovery_status', self._on_recovery_status_msg, 10)
        self.create_subscription(String, '/recovery/status', self._on_recovery_status_str, 10)

        self.create_subscription(OccupancyGrid, '/map', self._on_slam_map, qos_transient)
        self.create_subscription(Path, '/plan', self._on_plan, 10)

        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self._on_camera_comp, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self._on_camera_raw, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, '/camera/image_compressed', self._on_camera_comp, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, '/camera/thermal/compressed', self._on_thermal_comp, qos_profile_sensor_data)
        self.create_subscription(Image, '/thermal/image_raw', self._on_thermal_raw, qos_profile_sensor_data)

        self.pub_job = self.create_publisher(JobAssignment, '/rover/job_assignment', 10)
        self.pub_station_job = self.create_publisher(JobAssignment, '/station/job_assignment', 10)
        self.pub_mission_cmd = self.create_publisher(MissionCommand, '/rover/mission_command', 10)
        self.pub_rover_cmd_compat = self.create_publisher(MissionCommand, '/rover/mission_cmd', 10)
        self.pub_drillpulse_cmd = self.create_publisher(MissionCommand, '/drillpulse/mission_command', 10)
        self.pub_goal_pose = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub_recovery_cmd = self.create_publisher(RecoveryCommand, '/rover/recovery_command', 10)
        self.pub_rec_cmd_compat = self.create_publisher(RecoveryCommand, '/recovery/command', 10)
        self.pub_gen_report = self.create_publisher(String, '/drillpulse/generate_report', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/rover/cmd_vel', 10)
        self.pub_cmd_vel_compat = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_estop = self.create_publisher(Bool, '/rover/emergency_stop', 10)

        self._setup_http_routes()
        self._setup_socket_handlers()

        self._running = True
        self._emitter_thread = threading.Thread(target=self._state_emit_loop, daemon=True)
        self._emitter_thread.start()

        self.get_logger().info(f"DrillPulse Unified Dashboard Node active on port {self.web_port}")

    def _setup_http_routes(self):
        @self.app.route('/')
        def index():
            return send_from_directory(WEB_DIR, 'index.html')

        @self.app.route('/api/map')
        def get_explored_map():
            payload = self._load_yaml_map()
            if payload is None:
                return jsonify({"error": "Explored map not available"}), 404
            return jsonify(payload)

        @self.app.route('/api/reports')
        def list_reports():
            if not os.path.exists(self.report_dir):
                return jsonify([])
            files = [f for f in os.listdir(self.report_dir) if f.endswith('.html')]
            files.sort(reverse=True)
            return jsonify([{"filename": f, "url": f"/reports/{f}"} for f in files])

        @self.app.route('/reports/<path:filename>')
        def serve_report(filename):
            return send_from_directory(self.report_dir, filename)

    def _setup_socket_handlers(self):
        @self.socketio.on('connect')
        def handle_connect():
            self.get_logger().info("Operator client connected to DrillPulse Dashboard")
            self._emit_full_state()

        @self.socketio.on('request_mode')
        def handle_request_mode(data):
            mode = str(data.get('mode', 'NONE')).upper()
            cmd_id = f"JOB_{int(time.time()*1000)}"
            self.get_logger().info(f"Operator requested Mode switch to: {mode} (id={cmd_id})")
            self.requested_mode = mode

            job_msg = JobAssignment()
            job_msg.header.stamp = self.get_clock().now().to_msg()
            job_msg.job_id = cmd_id
            job_msg.assigned_job = mode
            if mode == 'EXPLORED':
                job_msg.status = "ASSIGNED"
                job_msg.map_name = "rover_map"
            elif mode == 'UNEXPLORED':
                job_msg.status = "ASSIGNED"
                job_msg.map_name = "slam_toolbox"
            else:
                job_msg.status = "IDLE"

            self.active_commands[cmd_id] = {
                'status': 'SENT',
                'timestamp': time.time(),
                'type': f"MODE_{mode}"
            }

            self.pub_job.publish(job_msg)
            self.pub_station_job.publish(job_msg)
            self.socketio.emit('command_update', {'cmd_id': cmd_id, 'status': 'SENT', 'type': f"MODE_{mode}"})

        @self.socketio.on('send_waypoint')
        def handle_send_waypoint(data):
            x = float(data.get('x', 0.0))
            y = float(data.get('y', 0.0))
            yaw = float(data.get('yaw', 0.0))
            cmd_id = f"WP_{int(time.time()*1000)}"

            self.get_logger().info(f"Operator dispatched Waypoint {cmd_id}: ({x:.2f}, {y:.2f}, yaw={yaw:.1f})")

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            yaw_rad = math.radians(yaw)
            pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
            pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
            self.pub_goal_pose.publish(pose)

            mcmd = MissionCommand()
            mcmd.header.stamp = self.get_clock().now().to_msg()
            mcmd.command_type = "GOTO_WAYPOINT"
            mcmd.target_x = x
            mcmd.target_y = y
            mcmd.target_yaw = yaw_rad

            self.active_commands[cmd_id] = {
                'status': 'SENT',
                'timestamp': time.time(),
                'type': 'WAYPOINT',
                'coords': [x, y]
            }

            self.pub_mission_cmd.publish(mcmd)
            self.pub_rover_cmd_compat.publish(mcmd)
            self.pub_drillpulse_cmd.publish(mcmd)

            self.socketio.emit('command_update', {
                'cmd_id': cmd_id,
                'status': 'SENT',
                'type': 'WAYPOINT',
                'coords': [x, y]
            })

        @self.socketio.on('return_home')
        def handle_return_home():
            cmd_id = f"RTH_{int(time.time()*1000)}"
            self.get_logger().info(f"Operator dispatched RETURN TO START (id={cmd_id})")

            mcmd = MissionCommand()
            mcmd.header.stamp = self.get_clock().now().to_msg()
            mcmd.command_type = "RETURN_HOME"
            mcmd.target_x = 0.0
            mcmd.target_y = 0.0

            self.active_commands[cmd_id] = {
                'status': 'SENT',
                'timestamp': time.time(),
                'type': 'RETURN_HOME'
            }

            self.pub_mission_cmd.publish(mcmd)
            self.pub_rover_cmd_compat.publish(mcmd)
            self.pub_drillpulse_cmd.publish(mcmd)

            self.socketio.emit('command_update', {'cmd_id': cmd_id, 'status': 'SENT', 'type': 'RETURN_HOME'})

        @self.socketio.on('teleop_vel')
        def handle_teleop(data):
            vx = float(data.get('vx', 0.0))
            wz = float(data.get('wz', 0.0))
            tw = Twist()
            tw.linear.x = vx
            tw.angular.z = wz
            self.pub_cmd_vel.publish(tw)
            self.pub_cmd_vel_compat.publish(tw)

        @self.socketio.on('emergency_stop')
        def handle_estop():
            self.get_logger().warn("EMERGENCY STOP TRIGGERED BY OPERATOR!")
            b = Bool()
            b.data = True
            self.pub_estop.publish(b)

            mcmd = MissionCommand()
            mcmd.header.stamp = self.get_clock().now().to_msg()
            mcmd.command_type = "EMERGENCY_STOP"
            mcmd.emergency_stop = True
            self.pub_mission_cmd.publish(mcmd)
            self.pub_rover_cmd_compat.publish(mcmd)
            self.pub_drillpulse_cmd.publish(mcmd)

            tw = Twist()
            self.pub_cmd_vel.publish(tw)
            self.pub_cmd_vel_compat.publish(tw)

        @self.socketio.on('trigger_recovery')
        def handle_recovery(data):
            action = str(data.get('action', 'SWIVEL_45')).upper()
            cmd_id = f"REC_{int(time.time()*1000)}"
            self.get_logger().info(f"Operator triggered Recovery: {action} (id={cmd_id})")

            cmd = RecoveryCommand()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.command_type = action
            if action in ('LEFT_RECOVERY', 'EXTEND_LEFT'):
                cmd.actuator_extension = 0.15
            elif action in ('RIGHT_RECOVERY', 'EXTEND_RIGHT'):
                cmd.actuator_extension = 0.15
            elif action == 'SWIVEL_45':
                cmd.actuator_extension = 0.10

            self.active_commands[cmd_id] = {
                'status': 'SENT',
                'timestamp': time.time(),
                'type': f"RECOVERY_{action}"
            }

            self.pub_recovery_cmd.publish(cmd)
            self.pub_rec_cmd_compat.publish(cmd)
            self.socketio.emit('command_update', {'cmd_id': cmd_id, 'status': 'SENT', 'type': f"RECOVERY_{action}"})

        @self.socketio.on('generate_report')
        def handle_generate_report():
            self.get_logger().info("Operator requested manual report generation")
            s = String()
            s.data = "OPERATOR_TRIGGERED"
            self.pub_gen_report.publish(s)

    def _on_telemetry(self, msg: RoverTelemetry):
        now = time.time()
        with self._lock:
            self.last_rover_seen = now
            soc = max(0.0, min(1.0, msg.battery_percentage / 100.0))
            curr = max(0.5, float(msg.current_draw))
            runtime_est = round((soc * 10.0) / curr * 60.0, 1)

            self.latest_telemetry = {
                "battery": round(float(msg.battery_percentage), 1),
                "voltage": round(float(msg.battery_voltage), 1),
                "current": round(float(msg.current_draw), 1),
                "runtime_min": runtime_est,
                "temp": round(float(msg.temperature), 1),
                "humidity": round(float(msg.humidity), 1),
                "gas_ppm": round(float(msg.gas_ppm), 1),
                "ch4": round(float(msg.methane_ppm), 2),
                "co": round(float(msg.carbon_monoxide_ppm), 2),
                "o2": round(float(msg.oxygen_percentage), 1),
                "dust": round(float(msg.dust_concentration), 2),
                "speed": round(float(msg.linear_velocity), 2),
                "pitch": round(float(msg.pitch_angle), 1),
                "roll": round(float(msg.roll_angle), 1),
                "yaw": round(float(msg.yaw_angle), 2),
                "x": round(float(msg.pose_x), 3),
                "y": round(float(msg.pose_y), 3),
                "lora_rssi": round(float(msg.lora_rssi), 1),
                "active_mode": msg.active_job,
                "state": msg.current_state,
                "estop": msg.emergency_stopped,
                "stale": False
            }

    def _on_diagnostics(self, msg: OdometryDiagnostics):
        with self._lock:
            self.latest_diagnostics = {
                "fusion_state": msg.fusion_state,
                "source": msg.selected_source,
                "vel_err": round(float(msg.velocity_error), 3),
                "yaw_err": round(float(msg.yaw_error), 3),
                "enc_valid": bool(msg.encoder_valid),
                "imu_valid": bool(msg.imu_valid),
                "enc_conf": round(float(msg.encoder_confidence), 2),
                "imu_conf": round(float(msg.imu_confidence), 2)
            }

    def _on_mission_state(self, msg: MissionState):
        with self._lock:
            self.rover_state = msg.current_state
            self.rover_acknowledged_mode = msg.active_job
            self.frontiers_count = int(getattr(msg, 'discovered_frontiers_count', 0))
            for c_info in self.active_commands.values():
                st = c_info.get('status')
                if st == 'ACCEPTED' and msg.current_state in (
                    'AUTONOMOUS_NAVIGATION', 'NAVIGATING', 'EXPLORING', 'RETURNING', 'RETURNING_HOME'
                ):
                    c_info['status'] = 'EXECUTING'
                elif st in ('ACCEPTED', 'EXECUTING') and msg.current_state in (
                    'MISSION_COMPLETE', 'MISSION_COMPLETED', 'RETURNED_TO_START'
                ):
                    c_info['status'] = 'COMPLETED'
                elif st in ('ACCEPTED', 'EXECUTING') and msg.current_state in (
                    'MISSION_ABORTED', 'ABORTED', 'EMERGENCY_STOP'
                ):
                    c_info['status'] = 'FAILED'

    def _on_job_ack(self, msg: JobAssignment):
        with self._lock:
            jid = getattr(msg, 'job_id', '')
            status = getattr(msg, 'status', '')
            if jid in self.active_commands:
                if 'ACCEPTED' in status:
                    self.active_commands[jid]['status'] = 'ACCEPTED'
                elif 'REJECTED' in status or 'FAILED' in status:
                    self.active_commands[jid]['status'] = 'FAILED'
                elif 'COMPLETE' in status:
                    self.active_commands[jid]['status'] = 'COMPLETED'
            if getattr(msg, 'assigned_job', ''):
                self.rover_acknowledged_mode = msg.assigned_job

    def _on_hazard(self, msg: HazardEvent):
        h = {
            "time": time.strftime("%H:%M:%S"),
            "id": getattr(msg, 'event_id', 'HZ_00'),
            "type": getattr(msg, 'event_type', getattr(msg, 'hazard_type', 'HAZARD')),
            "severity": getattr(msg, 'severity', 'WARNING'),
            "desc": getattr(msg, 'description', ''),
            "value": round(float(getattr(msg, 'measured_value', 0.0)), 2),
            "threshold": round(float(getattr(msg, 'threshold_value', 0.0)), 2),
            "x": round(float(getattr(msg, 'location_x', 0.0)), 2),
            "y": round(float(getattr(msg, 'location_y', 0.0)), 2)
        }
        with self._lock:
            self.recent_hazards.insert(0, h)
            if len(self.recent_hazards) > 15:
                self.recent_hazards.pop()
        self.socketio.emit('hazard_alert', h)

    def _on_link_status(self, msg: LinkStatus):
        with self._lock:
            link_type_str = str(getattr(msg, 'link_type', 'LORA_MESH'))
            rssi = float(getattr(msg, 'signal_strength_rssi', -65.0))
            loss = float(getattr(msg, 'packet_loss_rate', 0.0))

            if not msg.connected:
                link_st = "LOST"
            elif rssi >= -75.0 and loss < 0.15:
                link_st = "GOOD"
            elif rssi >= -90.0 and loss < 0.40:
                link_st = "DEGRADED"
            elif rssi >= -105.0 and loss < 0.70:
                link_st = "WEAK"
            else:
                link_st = "CRITICAL"

            now = time.time()
            packet_age = round(now - self.last_rover_seen, 1) if self.last_rover_seen > 0 else 999.0
            video_active = (now - self.last_video_frame_time < 2.5)

            is_conn = bool(msg.connected)
            overall_st = getattr(msg, 'overall_state', '')
            if not overall_st:
                overall_st = link_st if is_conn else "DISCONNECTED"

            self.latest_link = {
                "connected": is_conn,
                "link_state": link_st,
                "overall_state": str(overall_st),
                "rssi": round(rssi, 1) if is_conn else None,
                "snr": round(float(getattr(msg, 'snr_db', getattr(msg, 'snr', 9.5))), 1) if is_conn else None,
                "packet_loss": round(loss, 2) if is_conn else None,
                "latency": round(float(getattr(msg, 'latency_ms', 45.0)), 1) if is_conn else None,
                "transport": link_type_str,
                "video_online": video_active,
                "last_packet_age": packet_age
            }

    def _on_safety_status(self, msg: SafetyStatus):
        with self._lock:
            self.latest_safety = {
                "safety_state": str(getattr(msg, 'safety_state', 'NORMAL')),
                "motion_allowed": bool(getattr(msg, 'motion_allowed', True)),
                "emergency_stop_active": bool(getattr(msg, 'emergency_stop_active', False)),
                "obstacle_distance_m": float(getattr(msg, 'obstacle_distance_m', 99.0))
            }
        self.socketio.emit('safety_status', self.latest_safety)

    def _on_recovery_status_msg(self, msg: RecoveryStatus):
        st_text = f"STATE:{msg.state}, SIDE:{msg.active_side}, ROLL:{msg.current_roll:.1f}°"
        with self._lock:
            self.recent_recovery_status = st_text
        self.socketio.emit('recovery_status', {"status": st_text})

    def _on_recovery_status_str(self, msg: String):
        with self._lock:
            self.recent_recovery_status = msg.data
        self.socketio.emit('recovery_status', {"status": msg.data})

    def _on_slam_map(self, msg: OccupancyGrid):
        try:
            w = msg.info.width
            h = msg.info.height
            if w > 0 and h > 0:
                data = np.array(msg.data, dtype=np.int8).reshape((h, w))
                img = np.zeros((h, w, 3), dtype=np.uint8)
                img[data == -1] = [40, 44, 52]
                img[data == 0] = [220, 220, 220]
                img[data > 50] = [20, 20, 20]
                ok, png = cv2.imencode('.png', img)
                if ok:
                    b64 = base64.b64encode(png).decode('ascii')
                    self._slam_map_cache = {
                        "image_data": f"data:image/png;base64,{b64}",
                        "width": w,
                        "height": h,
                        "resolution": msg.info.resolution,
                        "origin_x": msg.info.origin.position.x,
                        "origin_y": msg.info.origin.position.y
                    }
        except Exception:
            pass

    def _on_plan(self, msg: Path):
        pts = [[round(p.pose.position.x, 2), round(p.pose.position.y, 2)] for p in msg.poses[::4]]
        with self._lock:
            self.latest_plan_points = pts

    def _on_camera_comp(self, msg: CompressedImage):
        try:
            self.last_video_frame_time = time.time()
            b64 = base64.b64encode(msg.data).decode('ascii')
            self.socketio.emit('live_frame', {"data": f"data:image/jpeg;base64,{b64}"})
        except Exception:
            pass

    def _on_camera_raw(self, msg: Image):
        try:
            self.last_video_frame_time = time.time()
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
            ok, jpg = cv2.imencode('.jpg', arr, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
            if ok:
                b64 = base64.b64encode(jpg).decode('ascii')
                self.socketio.emit('live_frame', {"data": f"data:image/jpeg;base64,{b64}"})
        except Exception:
            pass

    def _on_thermal_comp(self, msg: CompressedImage):
        try:
            self.last_video_frame_time = time.time()
            b64 = base64.b64encode(msg.data).decode('ascii')
            self.socketio.emit('thermal_frame', {"data": f"data:image/jpeg;base64,{b64}"})
        except Exception:
            pass

    def _on_thermal_raw(self, msg: Image):
        try:
            self.last_video_frame_time = time.time()
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
            ok, jpg = cv2.imencode('.jpg', arr, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
            if ok:
                b64 = base64.b64encode(jpg).decode('ascii')
                self.socketio.emit('thermal_frame', {"data": f"data:image/jpeg;base64,{b64}"})
        except Exception:
            pass

    def _load_yaml_map(self):
        if self._map_cache is not None:
            return self._map_cache
        if not os.path.exists(self.map_yaml):
            return None
        try:
            with open(self.map_yaml, 'r') as f:
                meta = yaml.safe_load(f)
            pgm_rel = meta.get('image', '')
            pgm_path = os.path.join(os.path.dirname(self.map_yaml), pgm_rel)
            if not os.path.exists(pgm_path):
                return None
            raw = cv2.imread(pgm_path, cv2.IMREAD_UNCHANGED)
            if raw is None:
                return None
            h, w = raw.shape
            rgba = cv2.cvtColor(raw, cv2.COLOR_GRAY2RGBA)
            ok, png = cv2.imencode('.png', rgba)
            if not ok:
                return None
            b64 = base64.b64encode(png).decode('ascii')
            self._map_cache = {
                "image_data": f"data:image/png;base64,{b64}",
                "width": int(w),
                "height": int(h),
                "resolution": float(meta.get('resolution', 0.05)),
                "origin": meta.get('origin', [0.0, 0.0, 0.0])
            }
            return self._map_cache
        except Exception as e:
            self.get_logger().error(f"Error loading map: {e}")
            return None

    def _state_emit_loop(self):
        rate = 1.0 / STATE_EMIT_RATE_HZ
        while self._running:
            self._emit_full_state()
            time.sleep(rate)

    def _emit_full_state(self):
        now = time.time()
        with self._lock:
            is_stale = (self.last_rover_seen == 0.0) or ((now - self.last_rover_seen) > 3.5)
            is_connected = not is_stale

            is_active_explored = (self.rover_acknowledged_mode == "EXPLORED")
            is_active_unexplored = (self.rover_acknowledged_mode == "UNEXPLORED")

            video_online = (now - self.last_video_frame_time < 2.5)
            self.latest_link["video_online"] = video_online
            self.latest_link["connected"] = is_connected
            self.latest_link["last_packet_age"] = round(now - self.last_rover_seen, 1) if self.last_rover_seen > 0 else 999.0
            if is_stale:
                self.latest_link["link_state"] = "LOST"
                self.latest_link["overall_state"] = "DISCONNECTED"
                self.latest_link["rssi"] = None
                self.latest_link["packet_loss"] = None
                self.latest_link["latency"] = None

            telem_payload = None
            if not is_stale and self.latest_telemetry is not None:
                telem_payload = dict(self.latest_telemetry)
                telem_payload["stale"] = False
            else:
                telem_payload = {
                    "stale": True,
                    "battery": None,
                    "voltage": None,
                    "current": None,
                    "runtime_min": None,
                    "temp": None,
                    "humidity": None,
                    "gas_ppm": None,
                    "ch4": None,
                    "co": None,
                    "o2": None,
                    "dust": None,
                    "speed": None,
                    "pitch": None,
                    "roll": None,
                    "yaw": None,
                    "x": None,
                    "y": None,
                    "state": "DISCONNECTED" if is_stale else "UNKNOWN",
                    "estop": False
                }

            payload = {
                "active_panel": "EXPLORED" if is_active_explored else ("UNEXPLORED" if is_active_unexplored else "NONE"),
                "rover_ack": (is_active_explored or is_active_unexplored),
                "requested_mode": self.requested_mode,
                "rover_state": "DISCONNECTED" if is_stale else self.rover_state,
                "frontiers_count": self.frontiers_count if not is_stale else 0,
                "telemetry": telem_payload,
                "diagnostics": self.latest_diagnostics if not is_stale else {"fusion_state": "NO DATA"},
                "link": self.latest_link,
                "recovery_status": self.recent_recovery_status,
                "plan": self.latest_plan_points if not is_stale else [],
                "hazards": self.recent_hazards,
                "connected": is_connected,
                "slam_map": self._slam_map_cache
            }

        self.socketio.emit('dashboard_state', payload)

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def run_flask(node):
    node.socketio.run(node.app, host="0.0.0.0", port=node.web_port, use_reloader=False)


def main(args=None):
    rclpy.init(args=args)
    node = DrillPulseDashboardNode()

    server_thread = threading.Thread(target=run_flask, args=(node,), daemon=True)
    server_thread.start()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
