#!/usr/bin/env python3
# ============================================================
# FILE: mission_manager_node.py
# PURPOSE: Canonical Mission Orchestrator and Authoritative State Machine
#          for DrillPulse underground mine safety and rescue rover.
#          Coordinates Explored vs Unexplored modes, job acceptance,
#          hazard-aware exploration, Nav2 return-to-start, recovery
#          interlocks, collision-resistant session IDs, and report triggers.
# ============================================================

import os
import math
import time
from datetime import datetime
from typing import Optional, Dict, Set, List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import String, Bool

from drillpulse_msgs.msg import (
    JobAssignment,
    MissionCommand,
    MissionState,
    LinkStatus,
    SafetyStatus,
    RecoveryStatus,
    HazardEvent,
    OdometryDiagnostics
)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q


def extract_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MissionStateEnum:
    DISCONNECTED = "DISCONNECTED"
    STANDBY = "STANDBY"
    JOB_RECEIVED = "JOB_RECEIVED"
    JOB_ACCEPTED = "JOB_ACCEPTED"
    INITIALIZING = "INITIALIZING"
    READY = "READY"
    MANUAL_CONTROL = "MANUAL_CONTROL"
    AUTONOMOUS_NAVIGATION = "AUTONOMOUS_NAVIGATION"
    NAVIGATING = "NAVIGATING"  # Backward compatibility alias
    EXPLORING = "EXPLORING"
    HAZARD_RESPONSE = "HAZARD_RESPONSE"
    RECOVERY = "RECOVERY"
    RETURNING = "RETURNING"
    RETURNING_HOME = "RETURNING_HOME"  # Backward compatibility alias
    RETURNING_LOW_BATTERY = "RETURNING_LOW_BATTERY"  # Backward compatibility alias
    RETURNING_COMM_LOSS = "RETURNING_COMM_LOSS"  # Backward compatibility alias
    RETURN_PAUSED = "RETURN_PAUSED"
    RETURNED_TO_START = "RETURNED_TO_START"
    RETURNED_TO_BASE = "RETURNED_TO_START"  # Alias
    NAVIGATION_FAILED = "NAVIGATION_FAILED"
    SAFE_HOLD = "SAFE_HOLD"
    HOLD_POSITION = "HOLD_POSITION"  # Backward compatibility alias
    MISSION_COMPLETE = "MISSION_COMPLETE"
    MISSION_COMPLETED = "MISSION_COMPLETED"  # Backward compatibility alias
    MISSION_ABORTED = "MISSION_ABORTED"
    ABORTED = "ABORTED"  # Backward compatibility alias
    EMERGENCY_STOP = "EMERGENCY_STOP"
    IDLE = "IDLE"  # Standby alias


class ReturnReasonEnum:
    BATTERY_LOW = "BATTERY_LOW"
    LOW_BATTERY = "BATTERY_LOW"
    BATTERY_CRITICAL = "CRITICAL_BATTERY"
    CRITICAL_BATTERY = "CRITICAL_BATTERY"
    BATTERY_RETURN = "BATTERY_RETURN"
    CONNECTION_CRITICAL = "CONNECTION_CRITICAL"
    CONNECTION_LOST = "CONNECTION_LOST"
    COMM_LOSS = "COMM_LOSS"
    OPERATOR_REQUEST = "OPERATOR_REQUEST"
    OPERATOR = "OPERATOR"
    HAZARD = "HAZARD"
    NAVIGATION_POLICY = "NAVIGATION_POLICY"
    MISSION_COMPLETE = "MISSION_COMPLETE"
    NONE = "NONE"


class RejectionReasonEnum:
    INVALID_JOB = "INVALID_JOB"
    INVALID_MODE = "INVALID_MODE"
    INVALID_TARGET = "INVALID_TARGET"
    BATTERY_TOO_LOW = "BATTERY_TOO_LOW"
    LOCALIZATION_UNAVAILABLE = "LOCALIZATION_UNAVAILABLE"
    NAVIGATION_UNAVAILABLE = "NAVIGATION_UNAVAILABLE"
    SAFETY_LOCK = "SAFETY_LOCK"
    COMMUNICATION_UNSTABLE = "COMMUNICATION_UNSTABLE"


# State transition rules: Map current state to allowed next states
VALID_TRANSITIONS: Dict[str, Set[str]] = {
    MissionStateEnum.IDLE: {
        MissionStateEnum.STANDBY, MissionStateEnum.JOB_RECEIVED, MissionStateEnum.JOB_ACCEPTED,
        MissionStateEnum.INITIALIZING, MissionStateEnum.READY, MissionStateEnum.NAVIGATING,
        MissionStateEnum.AUTONOMOUS_NAVIGATION, MissionStateEnum.EXPLORING, MissionStateEnum.DISCONNECTED,
        MissionStateEnum.EMERGENCY_STOP, MissionStateEnum.RETURNING, MissionStateEnum.RETURNING_HOME,
        MissionStateEnum.RETURNING_LOW_BATTERY, MissionStateEnum.RETURNING_COMM_LOSS,
        MissionStateEnum.HOLD_POSITION, MissionStateEnum.SAFE_HOLD, MissionStateEnum.RECOVERY
    },
    MissionStateEnum.STANDBY: {
        MissionStateEnum.JOB_RECEIVED, MissionStateEnum.JOB_ACCEPTED, MissionStateEnum.INITIALIZING,
        MissionStateEnum.READY, MissionStateEnum.NAVIGATING, MissionStateEnum.AUTONOMOUS_NAVIGATION,
        MissionStateEnum.EXPLORING, MissionStateEnum.DISCONNECTED, MissionStateEnum.EMERGENCY_STOP,
        MissionStateEnum.HOLD_POSITION, MissionStateEnum.SAFE_HOLD, MissionStateEnum.RECOVERY
    },
    MissionStateEnum.JOB_RECEIVED: {
        MissionStateEnum.JOB_ACCEPTED, MissionStateEnum.STANDBY, MissionStateEnum.SAFE_HOLD,
        MissionStateEnum.HOLD_POSITION, MissionStateEnum.EMERGENCY_STOP, MissionStateEnum.IDLE
    },
    MissionStateEnum.JOB_ACCEPTED: {
        MissionStateEnum.INITIALIZING, MissionStateEnum.READY, MissionStateEnum.AUTONOMOUS_NAVIGATION,
        MissionStateEnum.NAVIGATING, MissionStateEnum.EXPLORING, MissionStateEnum.SAFE_HOLD,
        MissionStateEnum.HOLD_POSITION, MissionStateEnum.EMERGENCY_STOP
    },
    MissionStateEnum.INITIALIZING: {
        MissionStateEnum.READY, MissionStateEnum.AUTONOMOUS_NAVIGATION, MissionStateEnum.NAVIGATING,
        MissionStateEnum.EXPLORING, MissionStateEnum.SAFE_HOLD, MissionStateEnum.HOLD_POSITION,
        MissionStateEnum.EMERGENCY_STOP
    },
    MissionStateEnum.READY: {
        MissionStateEnum.MANUAL_CONTROL, MissionStateEnum.AUTONOMOUS_NAVIGATION, MissionStateEnum.NAVIGATING,
        MissionStateEnum.EXPLORING, MissionStateEnum.RETURNING, MissionStateEnum.RETURNING_HOME,
        MissionStateEnum.RETURNING_LOW_BATTERY, MissionStateEnum.RETURNING_COMM_LOSS,
        MissionStateEnum.HAZARD_RESPONSE, MissionStateEnum.RECOVERY, MissionStateEnum.SAFE_HOLD,
        MissionStateEnum.HOLD_POSITION, MissionStateEnum.STANDBY, MissionStateEnum.MISSION_COMPLETE,
        MissionStateEnum.MISSION_ABORTED, MissionStateEnum.EMERGENCY_STOP, MissionStateEnum.IDLE
    },
    MissionStateEnum.MANUAL_CONTROL: {
        MissionStateEnum.READY, MissionStateEnum.AUTONOMOUS_NAVIGATION, MissionStateEnum.NAVIGATING,
        MissionStateEnum.EXPLORING, MissionStateEnum.SAFE_HOLD, MissionStateEnum.HOLD_POSITION,
        MissionStateEnum.HAZARD_RESPONSE, MissionStateEnum.RECOVERY, MissionStateEnum.EMERGENCY_STOP,
        MissionStateEnum.RETURNING, MissionStateEnum.RETURNING_HOME
    },
    MissionStateEnum.AUTONOMOUS_NAVIGATION: {
        MissionStateEnum.READY, MissionStateEnum.EXPLORING, MissionStateEnum.HAZARD_RESPONSE,
        MissionStateEnum.RECOVERY, MissionStateEnum.RETURNING, MissionStateEnum.RETURNING_HOME,
        MissionStateEnum.RETURNING_LOW_BATTERY, MissionStateEnum.RETURNING_COMM_LOSS,
        MissionStateEnum.NAVIGATION_FAILED, MissionStateEnum.SAFE_HOLD, MissionStateEnum.HOLD_POSITION,
        MissionStateEnum.RETURNED_TO_START, MissionStateEnum.MISSION_COMPLETE,
        MissionStateEnum.MISSION_ABORTED, MissionStateEnum.EMERGENCY_STOP, MissionStateEnum.IDLE
    },
    MissionStateEnum.NAVIGATING: {
        MissionStateEnum.READY, MissionStateEnum.AUTONOMOUS_NAVIGATION, MissionStateEnum.EXPLORING,
        MissionStateEnum.HAZARD_RESPONSE, MissionStateEnum.RECOVERY, MissionStateEnum.RETURNING,
        MissionStateEnum.RETURNING_HOME, MissionStateEnum.RETURNING_LOW_BATTERY,
        MissionStateEnum.RETURNING_COMM_LOSS, MissionStateEnum.NAVIGATION_FAILED,
        MissionStateEnum.SAFE_HOLD, MissionStateEnum.HOLD_POSITION, MissionStateEnum.RETURNED_TO_START,
        MissionStateEnum.MISSION_COMPLETE, MissionStateEnum.MISSION_ABORTED,
        MissionStateEnum.EMERGENCY_STOP, MissionStateEnum.IDLE
    },
    MissionStateEnum.EXPLORING: {
        MissionStateEnum.READY, MissionStateEnum.AUTONOMOUS_NAVIGATION, MissionStateEnum.NAVIGATING,
        MissionStateEnum.HAZARD_RESPONSE, MissionStateEnum.RECOVERY, MissionStateEnum.RETURNING,
        MissionStateEnum.RETURNING_HOME, MissionStateEnum.RETURNING_LOW_BATTERY,
        MissionStateEnum.RETURNING_COMM_LOSS, MissionStateEnum.NAVIGATION_FAILED,
        MissionStateEnum.SAFE_HOLD, MissionStateEnum.HOLD_POSITION, MissionStateEnum.RETURNED_TO_START,
        MissionStateEnum.MISSION_COMPLETE, MissionStateEnum.MISSION_ABORTED,
        MissionStateEnum.EMERGENCY_STOP, MissionStateEnum.IDLE
    },
    MissionStateEnum.HAZARD_RESPONSE: {
        MissionStateEnum.READY, MissionStateEnum.AUTONOMOUS_NAVIGATION, MissionStateEnum.NAVIGATING,
        MissionStateEnum.EXPLORING, MissionStateEnum.RETURNING, MissionStateEnum.RETURNING_HOME,
        MissionStateEnum.RETURNING_LOW_BATTERY, MissionStateEnum.RETURNING_COMM_LOSS,
        MissionStateEnum.RECOVERY, MissionStateEnum.SAFE_HOLD, MissionStateEnum.HOLD_POSITION,
        MissionStateEnum.MISSION_ABORTED, MissionStateEnum.EMERGENCY_STOP
    },
    MissionStateEnum.RECOVERY: {
        MissionStateEnum.READY, MissionStateEnum.SAFE_HOLD, MissionStateEnum.HOLD_POSITION,
        MissionStateEnum.RETURNING, MissionStateEnum.RETURNING_HOME, MissionStateEnum.AUTONOMOUS_NAVIGATION,
        MissionStateEnum.NAVIGATING, MissionStateEnum.EXPLORING, MissionStateEnum.MISSION_ABORTED,
        MissionStateEnum.EMERGENCY_STOP
    },
    MissionStateEnum.RETURNING: {
        MissionStateEnum.RETURNED_TO_START, MissionStateEnum.RETURN_PAUSED, MissionStateEnum.NAVIGATION_FAILED,
        MissionStateEnum.SAFE_HOLD, MissionStateEnum.HOLD_POSITION, MissionStateEnum.MISSION_COMPLETE,
        MissionStateEnum.MISSION_ABORTED, MissionStateEnum.RECOVERY, MissionStateEnum.HAZARD_RESPONSE,
        MissionStateEnum.EMERGENCY_STOP
    },
    MissionStateEnum.RETURNING_HOME: {
        MissionStateEnum.RETURNED_TO_START, MissionStateEnum.RETURN_PAUSED, MissionStateEnum.NAVIGATION_FAILED,
        MissionStateEnum.SAFE_HOLD, MissionStateEnum.HOLD_POSITION, MissionStateEnum.MISSION_COMPLETE,
        MissionStateEnum.MISSION_ABORTED, MissionStateEnum.RECOVERY, MissionStateEnum.HAZARD_RESPONSE,
        MissionStateEnum.EMERGENCY_STOP
    },
    MissionStateEnum.RETURNING_LOW_BATTERY: {
        MissionStateEnum.RETURNED_TO_START, MissionStateEnum.RETURN_PAUSED, MissionStateEnum.NAVIGATION_FAILED,
        MissionStateEnum.SAFE_HOLD, MissionStateEnum.HOLD_POSITION, MissionStateEnum.MISSION_COMPLETE,
        MissionStateEnum.MISSION_ABORTED, MissionStateEnum.RECOVERY, MissionStateEnum.HAZARD_RESPONSE,
        MissionStateEnum.EMERGENCY_STOP
    },
    MissionStateEnum.RETURNING_COMM_LOSS: {
        MissionStateEnum.RETURNED_TO_START, MissionStateEnum.RETURN_PAUSED, MissionStateEnum.NAVIGATION_FAILED,
        MissionStateEnum.SAFE_HOLD, MissionStateEnum.HOLD_POSITION, MissionStateEnum.MISSION_COMPLETE,
        MissionStateEnum.MISSION_ABORTED, MissionStateEnum.RECOVERY, MissionStateEnum.HAZARD_RESPONSE,
        MissionStateEnum.EMERGENCY_STOP
    },
    MissionStateEnum.RETURN_PAUSED: {
        MissionStateEnum.RETURNING, MissionStateEnum.RETURNING_HOME, MissionStateEnum.SAFE_HOLD,
        MissionStateEnum.HOLD_POSITION, MissionStateEnum.MISSION_ABORTED, MissionStateEnum.EMERGENCY_STOP
    },
    MissionStateEnum.RETURNED_TO_START: {
        MissionStateEnum.READY, MissionStateEnum.STANDBY, MissionStateEnum.MISSION_COMPLETE,
        MissionStateEnum.SAFE_HOLD, MissionStateEnum.HOLD_POSITION, MissionStateEnum.IDLE,
        MissionStateEnum.EMERGENCY_STOP
    },
    MissionStateEnum.NAVIGATION_FAILED: {
        MissionStateEnum.RETURNING, MissionStateEnum.RETURNING_HOME, MissionStateEnum.SAFE_HOLD,
        MissionStateEnum.HOLD_POSITION, MissionStateEnum.READY, MissionStateEnum.MISSION_ABORTED,
        MissionStateEnum.EMERGENCY_STOP
    },
    MissionStateEnum.SAFE_HOLD: {
        MissionStateEnum.READY, MissionStateEnum.RETURNING, MissionStateEnum.RETURNING_HOME,
        MissionStateEnum.MANUAL_CONTROL, MissionStateEnum.MISSION_COMPLETE, MissionStateEnum.MISSION_ABORTED,
        MissionStateEnum.STANDBY, MissionStateEnum.IDLE, MissionStateEnum.EMERGENCY_STOP
    },
    MissionStateEnum.HOLD_POSITION: {
        MissionStateEnum.READY, MissionStateEnum.RETURNING, MissionStateEnum.RETURNING_HOME,
        MissionStateEnum.MANUAL_CONTROL, MissionStateEnum.MISSION_COMPLETE, MissionStateEnum.MISSION_ABORTED,
        MissionStateEnum.STANDBY, MissionStateEnum.IDLE, MissionStateEnum.EMERGENCY_STOP
    },
    MissionStateEnum.MISSION_COMPLETE: {
        MissionStateEnum.STANDBY, MissionStateEnum.IDLE, MissionStateEnum.EMERGENCY_STOP
    },
    MissionStateEnum.MISSION_COMPLETED: {
        MissionStateEnum.STANDBY, MissionStateEnum.IDLE, MissionStateEnum.EMERGENCY_STOP
    },
    MissionStateEnum.MISSION_ABORTED: {
        MissionStateEnum.STANDBY, MissionStateEnum.IDLE, MissionStateEnum.EMERGENCY_STOP
    },
    MissionStateEnum.ABORTED: {
        MissionStateEnum.STANDBY, MissionStateEnum.IDLE, MissionStateEnum.EMERGENCY_STOP
    },
    MissionStateEnum.DISCONNECTED: {
        MissionStateEnum.STANDBY, MissionStateEnum.READY, MissionStateEnum.RETURNING,
        MissionStateEnum.RETURNING_COMM_LOSS, MissionStateEnum.SAFE_HOLD, MissionStateEnum.HOLD_POSITION,
        MissionStateEnum.EMERGENCY_STOP, MissionStateEnum.IDLE
    },
    MissionStateEnum.EMERGENCY_STOP: {
        MissionStateEnum.SAFE_HOLD, MissionStateEnum.HOLD_POSITION, MissionStateEnum.STANDBY,
        MissionStateEnum.IDLE
    }
}


class MissionManagerNode(Node):
    """Authoritative mission coordinator managing mode separation, safety returns, and orchestration."""

    def __init__(self):
        super().__init__('mission_manager_node')

        # Parameters
        self.declare_parameter('initial_state', 'IDLE')
        self.declare_parameter('mission_id', 'MISSION_ALPHA_01')
        self.declare_parameter('comm_timeout_sec', 10.0)
        self.declare_parameter('max_return_retries', 3)
        self.declare_parameter('nav_stuck_timeout_sec', 30.0)
        self.declare_parameter('battery_normal_soc', 35.0)
        self.declare_parameter('battery_low_soc', 25.0)
        self.declare_parameter('battery_return_soc', 20.0)
        self.declare_parameter('battery_critical_soc', 15.0)
        self.declare_parameter('battery_normal_v', 11.4)
        self.declare_parameter('battery_low_v', 11.0)
        self.declare_parameter('battery_return_v', 10.8)
        self.declare_parameter('battery_critical_v', 10.5)
        self.declare_parameter('battery_capacity_ah', 10.0)

        self.mission_id = str(self.get_parameter('mission_id').value)
        self.session_id = f"DRILLPULSE-{datetime.now().strftime('%Y%m%d-%H%M%S')}-{os.urandom(2).hex().upper()}"
        self.current_state = str(self.get_parameter('initial_state').value).upper()
        self.active_job = 'NONE'
        self.current_phase = 'IDLE'

        # Thresholds
        self.comm_timeout = float(self.get_parameter('comm_timeout_sec').value)
        self.max_retries = int(self.get_parameter('max_return_retries').value)
        self.nav_stuck_timeout = float(self.get_parameter('nav_stuck_timeout_sec').value)

        self.bat_norm_soc = float(self.get_parameter('battery_normal_soc').value)
        self.bat_low_soc = float(self.get_parameter('battery_low_soc').value)
        self.bat_ret_soc = float(self.get_parameter('battery_return_soc').value)
        self.bat_crit_soc = float(self.get_parameter('battery_critical_soc').value)
        self.bat_norm_v = float(self.get_parameter('battery_normal_v').value)
        self.bat_low_v = float(self.get_parameter('battery_low_v').value)
        self.bat_ret_v = float(self.get_parameter('battery_return_v').value)
        self.bat_crit_v = float(self.get_parameter('battery_critical_v').value)
        self.bat_capacity_ah = float(self.get_parameter('battery_capacity_ah').value)

        # Odometry / Pose Tracking
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.has_received_odom = True
        self.first_odom_received = False
        self.distance_travelled = 0.0
        self.mission_start_time = time.time()

        # Mission Start Pose (Return Target Reference)
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.start_pose_locked = True  # Origin (0,0) locked by default until first odom arrives

        # Navigation State
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.navigation_active = False
        self.last_nav_progress_time = time.time()
        self.last_nav_dist = 999.0
        self.return_retries = 0
        self.return_reason = ReturnReasonEnum.NONE
        self.navigation_failed_latched = False

        # Battery Tracking
        self.battery_voltage: Optional[float] = None
        self.battery_percentage: Optional[float] = None
        self.battery_current: Optional[float] = None
        self.estimated_runtime_min: Optional[float] = None
        self.battery_health = "NORMAL"
        self.battery_low = False

        # Connection Tracking
        self.comm_connected = True
        self.last_comm_time = time.time()
        self.consecutive_comm_drops = 0

        # Safety & Recovery Interlocks
        self.emergency_stop_active = False
        self.motion_allowed = True
        self.in_recovery = False
        self.is_recovery_active = False

        # Transition History & Hazards Journal
        self.transition_history: List[Dict] = []
        self.confirmed_hazards: List[Dict] = []
        self.spatial_hazard_journal = self.confirmed_hazards
        self.last_transition_reason = "INITIAL"
        self.map_name = "rover_map"
        self.discovered_frontiers_count = 0

        # Publishers
        self.state_pub = self.create_publisher(MissionState, '/rover/mission_state', 10)
        self.ack_pub = self.create_publisher(JobAssignment, '/rover/job_assignment_ack', 10)
        self.ack_pub_short = self.create_publisher(JobAssignment, '/rover/job_ack', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.report_trigger_pub = self.create_publisher(String, '/drillpulse/generate_report', 10)

        # Subscriptions
        self.create_subscription(JobAssignment, '/rover/job_assignment', self.job_assignment_callback, 10)
        self.create_subscription(JobAssignment, '/rover/job', self.job_assignment_callback, 10)
        self.create_subscription(MissionCommand, '/rover/mission_command', self.mission_cmd_callback, 10)
        self.create_subscription(MissionCommand, '/rover/mission_cmd', self.mission_cmd_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(BatteryState, '/rover/battery', self.battery_callback, 10)
        self.create_subscription(LinkStatus, '/rover/link_status', self.link_callback, 10)
        self.create_subscription(LinkStatus, '/drillpulse/link_status', self.link_callback, 10)
        self.create_subscription(SafetyStatus, '/rover/safety_status', self.safety_callback, 10)
        self.create_subscription(HazardEvent, '/rover/hazard_event', self.hazard_callback, 10)
        self.create_subscription(HazardEvent, '/drillpulse/hazard_event', self.hazard_callback, 10)
        self.create_subscription(RecoveryStatus, '/rover/recovery_status', self.recovery_callback, 10)
        self.create_subscription(OdometryDiagnostics, '/odom/diagnostics', self.diagnostics_callback, 10)

        self.timer = self.create_timer(0.2, self.control_and_publish_cycle)
        self.get_logger().info(f'Mission Manager active. Session: {self.session_id}, State: {self.current_state}')

    # -------------------------------------------------------------
    # State Machine Transition Engine
    # -------------------------------------------------------------
    def transition_to(self, new_state: str, reason: str = "UNSPECIFIED", source: str = "INTERNAL") -> bool:
        """Executes an authoritative state transition with validation against permitted rules."""
        new_state = new_state.upper()
        current = self.current_state

        # EMERGENCY_STOP is universally permitted
        if new_state == MissionStateEnum.EMERGENCY_STOP:
            allowed = True
        else:
            allowed_next = VALID_TRANSITIONS.get(current, set())
            allowed = (new_state in allowed_next) or (current == new_state)

        if not allowed:
            self.get_logger().warn(
                f'REJECTED INVALID TRANSITION: {current} -> {new_state} (Reason: {reason}, Source: {source})'
            )
            return False

        prev = self.current_state
        self.current_state = new_state
        self.current_phase = new_state
        self.last_transition_reason = reason

        event = {
            "timestamp": time.time(),
            "iso_time": datetime.now().isoformat(),
            "previous_state": prev,
            "new_state": new_state,
            "reason": reason,
            "source": source
        }
        self.transition_history.append(event)
        self.get_logger().info(f'STATE TRANSITION: {prev} -> {new_state} [Reason: {reason}] [Source: {source}]')
        return True

    # -------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = extract_yaw(msg.pose.pose.orientation)

        # Lock first valid odometry packet as true mission start pose
        if not self.first_odom_received:
            self.first_odom_received = True
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_yaw = self.current_yaw
            self.start_pose_locked = True
            self.prev_x = self.current_x
            self.prev_y = self.current_y
            self.get_logger().info(f'Mission Start Pose Locked: ({self.start_x:.2f}, {self.start_y:.2f})')

        # Distance calculation
        step_dist = math.hypot(self.current_x - self.prev_x, self.current_y - self.prev_y)
        if step_dist > 0.002:
            self.distance_travelled += step_dist
            self.prev_x = self.current_x
            self.prev_y = self.current_y

        self.has_received_odom = True

        # Check goal proximity
        if self.navigation_active:
            dist = math.hypot(self.target_x - self.current_x, self.target_y - self.current_y)
            if dist < self.last_nav_dist - 0.05:
                self.last_nav_dist = dist
                self.last_nav_progress_time = time.time()

            # Target reached (within 35cm tolerance)
            if dist <= 0.35:
                self.navigation_active = False
                if self.current_state in (
                    MissionStateEnum.RETURNING, MissionStateEnum.RETURNING_HOME,
                    MissionStateEnum.RETURNING_LOW_BATTERY, MissionStateEnum.RETURNING_COMM_LOSS
                ):
                    self.transition_to(MissionStateEnum.RETURNED_TO_START, reason="ARRIVED_AT_BASE", source="ODOM")
                    self.current_phase = 'AT_START_SAFE'
                    self.get_logger().info('Rover successfully returned to mission start pose.')
                else:
                    self.current_phase = 'GOAL_REACHED'

    def battery_callback(self, msg: BatteryState):
        v = msg.voltage
        soc = msg.percentage
        curr = msg.current

        if math.isnan(v) or math.isnan(soc) or v < 5.0 or v > 20.0 or soc < 0.0 or soc > 100.0:
            self.battery_voltage = None
            self.battery_percentage = None
            self.battery_current = None
            self.estimated_runtime_min = None
            self.battery_health = "UNKNOWN"
            return

        self.battery_voltage = float(v)
        self.battery_percentage = float(soc)
        self.battery_current = float(curr) if not math.isnan(curr) and curr > 0.1 else None

        # Runtime estimation
        if self.battery_current and self.battery_current > 0.2:
            soc_frac = max(0.0, min(1.0, self.battery_percentage / 100.0))
            self.estimated_runtime_min = round((soc_frac * self.bat_capacity_ah) / max(0.5, self.battery_current) * 60.0, 1)

        # Progression: NORMAL -> LOW -> RETURN_REQUIRED -> CRITICAL
        if self.battery_voltage < self.bat_crit_v or self.battery_percentage < self.bat_crit_soc:
            self.battery_health = "CRITICAL"
            self.battery_low = True
            if not self.current_state.startswith('RETURN') and self.current_state != MissionStateEnum.RETURNED_TO_START:
                self.get_logger().warn(
                    f'CRITICAL BATTERY ({self.battery_voltage:.2f}V, {self.battery_percentage:.1f}%). Priority return initiated.'
                )
                self.trigger_return_to_start(MissionStateEnum.RETURNING_LOW_BATTERY, reason=ReturnReasonEnum.CRITICAL_BATTERY)

        elif self.battery_voltage < self.bat_ret_v or self.battery_percentage < self.bat_ret_soc:
            self.battery_health = "RETURN_REQUIRED"
            self.battery_low = True
            if not self.current_state.startswith('RETURN') and self.current_state != MissionStateEnum.RETURNED_TO_START:
                self.get_logger().warn(
                    f'BATTERY RETURN THRESHOLD REACHED ({self.battery_voltage:.2f}V, {self.battery_percentage:.1f}%). Safe return initiated.'
                )
                self.trigger_return_to_start(MissionStateEnum.RETURNING_LOW_BATTERY, reason=ReturnReasonEnum.BATTERY_RETURN)

        elif self.battery_voltage < self.bat_low_v or self.battery_percentage < self.bat_low_soc:
            self.battery_health = "LOW"
            self.battery_low = True
        else:
            self.battery_health = "NORMAL"
            self.battery_low = False

    def link_callback(self, msg: LinkStatus):
        self.last_comm_time = time.time()
        self.consecutive_comm_drops = 0
        self.comm_connected = bool(msg.connected)
        if not msg.connected or getattr(msg, 'overall_state', '') == 'LOST':
            self.comm_connected = False
            if not self.current_state.startswith('RETURN') and self.current_state not in (MissionStateEnum.RETURNED_TO_START, MissionStateEnum.EMERGENCY_STOP):
                self.get_logger().warn('Link lost in link_callback. Triggering comm loss return.')
                self.trigger_return_to_start(MissionStateEnum.RETURNING_COMM_LOSS, reason=ReturnReasonEnum.COMM_LOSS)
        elif msg.packet_loss_rate > 0.70:
            self.get_logger().warn('Link severely degraded.')

    def safety_callback(self, msg: SafetyStatus):
        self.emergency_stop_active = msg.emergency_stop_active
        self.motion_allowed = msg.motion_allowed

        if self.emergency_stop_active:
            if self.current_state != MissionStateEnum.EMERGENCY_STOP:
                self.transition_to(MissionStateEnum.EMERGENCY_STOP, reason="SAFETY_ESTOP_ACTIVE", source="SAFETY_MONITOR")
                self.navigation_active = False

        elif not self.motion_allowed:
            if self.navigation_active:
                self.get_logger().warn(f'Safety lock active ({msg.safety_state}). Halting Nav2 navigation.')
                self.navigation_active = False
                self.current_phase = 'HALTED_SAFETY'

    def hazard_callback(self, msg: HazardEvent):
        h_record = {
            "event_id": getattr(msg, 'event_id', f"HZ_{int(time.time()*1000)}"),
            "event_type": getattr(msg, 'event_type', getattr(msg, 'hazard_type', 'HAZARD')),
            "severity": getattr(msg, 'severity', 'WARNING'),
            "description": getattr(msg, 'description', ''),
            "measured_value": round(float(getattr(msg, 'measured_value', 0.0)), 2),
            "threshold_value": round(float(getattr(msg, 'threshold_value', 0.0)), 2),
            "confidence": round(float(getattr(msg, 'confidence', 1.0)), 2),
            "location_x": round(float(getattr(msg, 'location_x', self.current_x)), 2),
            "location_y": round(float(getattr(msg, 'location_y', self.current_y)), 2),
            "timestamp": time.time()
        }
        self.confirmed_hazards.append(h_record)

        if h_record["severity"] == "CRITICAL":
            self.get_logger().warn(f"CRITICAL HAZARD DETECTED: {h_record['event_type']} ({h_record['description']}).")
            if not self.current_state.startswith('RETURN') and self.current_state != MissionStateEnum.EMERGENCY_STOP:
                self.transition_to(MissionStateEnum.HAZARD_RESPONSE, reason=f"CRITICAL_{h_record['event_type']}", source="HAZARD_DETECTOR")
                # Execute safe return on critical environmental gas/heat hazard
                self.trigger_return_to_start(MissionStateEnum.RETURNING, reason=ReturnReasonEnum.HAZARD)

    def recovery_callback(self, msg: RecoveryStatus):
        # Recovery interlock: When rollover recovery is active, lock navigation
        self.is_recovery_active = (msg.state in ('EXTENDING', 'RETRACTING', 'SETTLE', 'RECOVERING'))
        self.in_recovery = self.is_recovery_active
        if self.is_recovery_active:
            if self.current_state != MissionStateEnum.RECOVERY:
                self.navigation_active = False
                self.transition_to(MissionStateEnum.RECOVERY, reason="ROLLOVER_ACTIVE", source="RECOVERY_CONTROLLER")
        elif msg.state in ('IDLE', 'READY', 'COOLDOWN') and self.in_recovery:
            self.in_recovery = False
            self.is_recovery_active = False
            # Verify orientation return before resuming
            if abs(msg.current_roll) <= 15.0 and abs(msg.current_pitch) <= 15.0:
                self.get_logger().info('Rollover recovery completed successfully. Returning to READY.')
                self.transition_to(MissionStateEnum.READY, reason="RECOVERY_SUCCESS", source="RECOVERY_CONTROLLER")
            else:
                self.get_logger().warn('Recovery finished but rover still tilted. Holding position.')
                self.transition_to(MissionStateEnum.SAFE_HOLD, reason="RECOVERY_TILT_PERSISTS", source="RECOVERY_CONTROLLER")
        elif msg.state in ('READY', 'IDLE'):
            self.is_recovery_active = False
            self.in_recovery = False

    def diagnostics_callback(self, msg: OdometryDiagnostics):
        if msg.fusion_state in ('SENSOR_TIMEOUT', 'SENSOR_DISAGREEMENT'):
            if self.navigation_active and self.current_state in (MissionStateEnum.NAVIGATING, MissionStateEnum.EXPLORING):
                self.get_logger().warn(f'Sensor diagnostics fault: {msg.fusion_state}. Pausing navigation.')
                self.navigation_active = False
                self.transition_to(MissionStateEnum.SAFE_HOLD, reason=f"DIAG_{msg.fusion_state}", source="ODOM_DIAG")

    # -------------------------------------------------------------
    # Job Validation & Acceptance Logic
    # -------------------------------------------------------------
    def validate_job(self, msg: JobAssignment) -> Tuple[bool, str]:
        """Validates incoming mission job assignment against all operating and safety parameters."""
        req_mode = getattr(msg, 'assigned_job', '').upper().strip()
        if not req_mode or req_mode in ('INVALID', 'NONE'):
            return False, RejectionReasonEnum.INVALID_JOB

        if req_mode not in ('EXPLORED', 'UNEXPLORED', 'IDLE', 'STANDBY'):
            return False, RejectionReasonEnum.INVALID_MODE

        if self.emergency_stop_active or not self.motion_allowed:
            return False, RejectionReasonEnum.SAFETY_LOCK

        if self.battery_health in ('RETURN_REQUIRED', 'CRITICAL'):
            return False, RejectionReasonEnum.BATTERY_TOO_LOW
        if self.battery_percentage is not None and self.battery_percentage < 20.0:
            return False, RejectionReasonEnum.BATTERY_TOO_LOW

        if not self.has_received_odom:
            return False, RejectionReasonEnum.LOCALIZATION_UNAVAILABLE

        if self.navigation_failed_latched:
            return False, RejectionReasonEnum.NAVIGATION_UNAVAILABLE

        if not self.comm_connected:
            return False, RejectionReasonEnum.COMMUNICATION_UNSTABLE

        return True, "VALID"

    def job_assignment_callback(self, msg: JobAssignment):
        if not getattr(msg, 'job_id', '').strip():
            msg.job_id = f"JOB_{int(time.time()*1000)}"

        req_job = msg.assigned_job.upper().strip()
        ack = JobAssignment()
        ack.header.stamp = self.get_clock().now().to_msg()
        ack.job_id = msg.job_id
        ack.assigned_job = req_job

        if req_job in ('IDLE', 'STANDBY'):
            self.active_job = 'NONE'
            self.transition_to(MissionStateEnum.IDLE, reason="OPERATOR_IDLE", source="JOB_ASSIGNMENT")
            self.navigation_active = False
            ack.status = 'JOB_ACCEPTED'
            ack.comments = 'Rover transitioned to IDLE/STANDBY'
            self.ack_pub.publish(ack)
            self.ack_pub_short.publish(ack)
            return

        is_valid, reason = self.validate_job(msg)
        if not is_valid:
            self.get_logger().warn(f'Rejecting job assignment {msg.job_id}: {reason}')
            ack.status = 'JOB_REJECTED'
            ack.comments = reason
            self.ack_pub.publish(ack)
            self.ack_pub_short.publish(ack)
            return

        # Job Accepted: Initialize session and start pose
        ack.status = 'JOB_ACCEPTED'
        ack.comments = f'Job {msg.job_id} accepted. Mode: {req_job}'

        # Generate fresh collision-resistant session ID
        self.session_id = f"DRILLPULSE-{datetime.now().strftime('%Y%m%d-%H%M%S')}-{os.urandom(2).hex().upper()}"
        self.mission_start_time = time.time()
        self.distance_travelled = 0.0

        # Lock true start pose
        self.start_x = self.current_x
        self.start_y = self.current_y
        self.start_yaw = self.current_yaw
        self.start_pose_locked = True

        if req_job == 'EXPLORED':
            self.active_job = 'EXPLORED'
            self.map_name = getattr(msg, 'map_name', 'rover_map') or 'rover_map'
            self.transition_to(MissionStateEnum.NAVIGATING, reason="JOB_ACCEPTED_EXPLORED", source="JOB_ASSIGNMENT")
            self.current_phase = 'READY'
        elif req_job == 'UNEXPLORED':
            self.active_job = 'UNEXPLORED'
            self.transition_to(MissionStateEnum.EXPLORING, reason="JOB_ACCEPTED_UNEXPLORED", source="JOB_ASSIGNMENT")
            self.current_phase = 'READY'

        self.ack_pub.publish(ack)
        self.ack_pub_short.publish(ack)
        self.get_logger().info(f'Job {msg.job_id} ACCEPTED. Rover in {self.current_state} (Mode: {self.active_job})')

    def mission_cmd_callback(self, msg: MissionCommand):
        cmd = msg.command_type.upper()

        if msg.emergency_stop or cmd in ('EMERGENCY_STOP', 'ESTOP'):
            self.transition_to(MissionStateEnum.EMERGENCY_STOP, reason="OPERATOR_ESTOP", source="MISSION_COMMAND")
            self.navigation_active = False
            self.current_phase = 'HALTED'
            self.get_logger().warn('Emergency Stop engaged via Mission Command.')
            return

        if cmd in ('RETURN_HOME', 'RETURN_TO_START', 'RETURN'):
            self.get_logger().info('Operator commanded RETURN TO START.')
            self.trigger_return_to_start(MissionStateEnum.RETURNING, reason=ReturnReasonEnum.OPERATOR)
            return

        if cmd in ('STOP', 'STOP_MISSION', 'ABORT_MISSION'):
            self.get_logger().warn('Operator commanded STOP/ABORT.')
            self.navigation_active = False
            self.transition_to(MissionStateEnum.MISSION_ABORTED, reason="OPERATOR_ABORT", source="MISSION_COMMAND")
            self._request_report_generation("ABORTED")
            return

        if cmd in ('COMPLETE_MISSION', 'FINISH'):
            self.get_logger().info('Operator commanded COMPLETE_MISSION.')
            self.navigation_active = False
            self.transition_to(MissionStateEnum.MISSION_COMPLETE, reason="OPERATOR_COMPLETE", source="MISSION_COMMAND")
            self._request_report_generation("COMPLETED")
            return

        if cmd in ('PAUSE', 'PAUSE_MISSION'):
            self.navigation_active = False
            if self.current_state.startswith('RETURN'):
                self.transition_to(MissionStateEnum.RETURN_PAUSED, reason="OPERATOR_PAUSE", source="MISSION_COMMAND")
            else:
                self.transition_to(MissionStateEnum.SAFE_HOLD, reason="OPERATOR_PAUSE", source="MISSION_COMMAND")
            return

        if cmd in ('RESUME', 'RESUME_MISSION'):
            if self.current_state == MissionStateEnum.RETURN_PAUSED:
                self.trigger_return_to_start(MissionStateEnum.RETURNING, reason=self.return_reason)
            elif self.current_state in (MissionStateEnum.SAFE_HOLD, MissionStateEnum.HOLD_POSITION):
                if self.active_job == 'EXPLORED':
                    self.transition_to(MissionStateEnum.NAVIGATING, reason="OPERATOR_RESUME", source="MISSION_COMMAND")
                elif self.active_job == 'UNEXPLORED':
                    self.transition_to(MissionStateEnum.EXPLORING, reason="OPERATOR_RESUME", source="MISSION_COMMAND")
            return

        if cmd == 'MANUAL_CONTROL':
            if self.emergency_stop_active or not self.motion_allowed:
                self.get_logger().warn('Rejecting Manual Control: Safety lock is engaged.')
                return
            self.navigation_active = False
            self.transition_to(MissionStateEnum.MANUAL_CONTROL, reason="OPERATOR_MANUAL", source="MISSION_COMMAND")
            return

        if cmd in ('GOTO_WAYPOINT', 'SEND_TARGET'):
            if self.battery_health in ('RETURN_REQUIRED', 'CRITICAL'):
                self.get_logger().warn('Rejecting waypoint: Low battery safety return in effect.')
                return
            if self.emergency_stop_active or not self.motion_allowed:
                self.get_logger().warn('Rejecting waypoint: Safety interlock engaged.')
                return

            self.target_x = msg.target_x
            self.target_y = msg.target_y
            self.target_yaw = msg.target_yaw
            self.navigation_active = True
            self.current_phase = 'NAVIGATING'
            self.last_nav_progress_time = time.time()
            self.last_nav_dist = math.hypot(self.target_x - self.current_x, self.target_y - self.current_y)

            self._dispatch_nav2_goal(self.target_x, self.target_y, self.target_yaw)

    def trigger_return_to_start(self, return_state=MissionStateEnum.RETURNING, reason=ReturnReasonEnum.OPERATOR):
        """Directs the rover back to the locked mission start pose via Nav2."""
        self.return_reason = reason

        # Missing start pose protection: Do NOT drive blindly to (0,0)
        if not self.start_pose_locked:
            self.get_logger().error('Return triggered but mission start pose is unavailable. Holding position.')
            self.transition_to(MissionStateEnum.HOLD_POSITION, reason="RETURN_TARGET_UNAVAILABLE", source="NAV_VALIDATOR")
            self.current_phase = 'RETURN_TARGET_UNAVAILABLE'
            self.navigation_active = False
            return

        self.transition_to(return_state, reason=str(reason), source="RETURN_SYSTEM")
        self.target_x = self.start_x
        self.target_y = self.start_y
        self.target_yaw = self.start_yaw
        self.navigation_active = True
        self.current_phase = 'RETURNING_TO_START'
        self.return_retries += 1

        self._dispatch_nav2_goal(self.start_x, self.start_y, self.start_yaw)
        self.get_logger().info(
            f'Dispatched Nav2 Return Goal: ({self.start_x:.2f}, {self.start_y:.2f}) [Attempt {self.return_retries}/{self.max_retries}] [Reason: {reason}]'
        )

    def _dispatch_nav2_goal(self, x: float, y: float, yaw: float):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = 0.0
        goal.pose.orientation = yaw_to_quaternion(yaw)
        self.goal_pub.publish(goal)

    def _request_report_generation(self, outcome: str):
        msg = String()
        msg.data = f"TRIGGER_REPORT:{self.session_id}:{outcome}"
        self.report_trigger_pub.publish(msg)
        self.get_logger().info(f'Requested mission report generation for session {self.session_id} ({outcome})')

    def control_and_publish_cycle(self):
        now = time.time()

        # Connection watchdog
        if (now - self.last_comm_time) > self.comm_timeout:
            self.consecutive_comm_drops += 1
            self.comm_connected = False
            if self.active_job in ('EXPLORED', 'UNEXPLORED') and not self.current_state.startswith('RETURN') and self.current_state != MissionStateEnum.RETURNED_TO_START:
                self.get_logger().warn(f'Communication silent for {now - self.last_comm_time:.1f}s. Triggering safe return.')
                self.trigger_return_to_start(MissionStateEnum.RETURNING_COMM_LOSS, reason=ReturnReasonEnum.COMM_LOSS)

        # Nav2 stuck / retry monitoring during return
        if self.navigation_active and self.current_state.startswith('RETURN'):
            if (now - self.last_nav_progress_time) > self.nav_stuck_timeout:
                if self.return_retries < self.max_retries:
                    self.get_logger().warn('Nav2 return progress stalled. Retrying return goal dispatch.')
                    self.trigger_return_to_start(self.current_state, reason=self.return_reason)
                else:
                    self.get_logger().error('Nav2 return exceeded maximum retry count. Entering SAFE_HOLD.')
                    self.navigation_failed_latched = True
                    self.transition_to(MissionStateEnum.HOLD_POSITION, reason="NAV2_STUCK_EXCEEDED_RETRIES", source="NAV_WATCHDOG")
                    self.current_phase = 'NAV_FAILED'
                    self.navigation_active = False

        dist_to_return = math.hypot(self.start_x - self.current_x, self.start_y - self.current_y) if self.start_pose_locked else 0.0

        # Publish MissionState
        m = MissionState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.mission_id = self.session_id
        m.current_state = self.current_state
        m.active_job = self.active_job
        m.current_phase = self.current_phase
        m.current_x = float(self.current_x)
        m.current_y = float(self.current_y)
        m.target_x = float(self.target_x)
        m.target_y = float(self.target_y)
        m.distance_to_target = float(math.hypot(self.target_x - self.current_x, self.target_y - self.current_y))
        m.navigation_active = bool(self.navigation_active)
        m.progress_percentage = min(100.0, max(0.0, (1.0 - (dist_to_return / max(1.0, self.distance_travelled))) * 100.0)) if self.distance_travelled > 0 else 0.0
        m.error_message = f"ReturnReason:{self.return_reason}|Retries:{self.return_retries}|DistRet:{dist_to_return:.1f}m"
        self.state_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
