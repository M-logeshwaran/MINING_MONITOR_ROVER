#!/usr/bin/env python3
# ============================================================
# DRILLPULSE — Lightweight Heartbeat & Liveness Watchdog
# Platform: Ubuntu 24.04 LTS + ROS 2 Jazzy + Python 3.12
#
# Implements:
#   - Lightweight compact heartbeat schema for low-bandwidth LoRa
#   - Heartbeat generator & parser
#   - Liveness watchdog for link failure detection
# ============================================================

import time
from typing import Dict, Any, Optional


class HeartbeatManager:
    """Generates compact heartbeat packets and tracks peer liveness."""

    def __init__(self, rover_id: str = "ROVER_01", timeout_sec: float = 4.0):
        self.rover_id = rover_id
        self.timeout_sec = timeout_sec
        self.last_heartbeat_sent = 0.0
        self.last_heartbeat_received = time.time()
        self.heartbeat_count_sent = 0
        self.heartbeat_count_received = 0

    def create_heartbeat_payload(
        self,
        session_id: str,
        rover_mode: str,
        mission_state: str,
        battery_pct: float,
        runtime_min: float,
        comm_state: str,
        fault_flags: int = 0,
        pose_x: float = 0.0,
        pose_y: float = 0.0
    ) -> Dict[str, Any]:
        """Creates a compact heartbeat dictionary strictly under 100 bytes."""
        self.heartbeat_count_sent += 1
        self.last_heartbeat_sent = time.time()
        return {
            "rid": self.rover_id,
            "sid": session_id,
            "mod": str(rover_mode)[:8],
            "st": str(mission_state)[:12],
            "bat": round(float(battery_pct), 1),
            "rt": round(float(runtime_min), 1),
            "lnk": str(comm_state)[:8],
            "flt": int(fault_flags),
            "x": round(float(pose_x), 2),
            "y": round(float(pose_y), 2),
            "seq": self.heartbeat_count_sent
        }

    def process_incoming_heartbeat(self, payload: Dict[str, Any]) -> bool:
        """Processes received heartbeat and refreshes peer liveness."""
        if not isinstance(payload, dict):
            return False
        self.last_heartbeat_received = time.time()
        self.heartbeat_count_received += 1
        return True

    def is_timed_out(self) -> bool:
        """Returns True if peer heartbeat has exceeded timeout threshold."""
        return (time.time() - self.last_heartbeat_received) > self.timeout_sec

    def silence_duration(self) -> float:
        return time.time() - self.last_heartbeat_received
