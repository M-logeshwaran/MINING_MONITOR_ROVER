#!/usr/bin/env python3
# ============================================================
# DRILLPULSE — Persistent Local Session & Communication Logger
# Platform: Ubuntu 24.04 LTS + ROS 2 Jazzy + Python 3.12
#
# Implements:
#   - Bounded, rotated JSONL session logging on Raspberry Pi filesystem
#   - Records all connection state changes, commands, ACKs, alerts,
#     packet loss, and return decisions
#   - Prevents SD card overflow with size limits and log rotation
# ============================================================

import os
import json
import time
import threading
from typing import Dict, Any, Optional

DEFAULT_LOG_DIR = "/home/loki/SIH_REPO_FINAL/reports"
MAX_LOG_SIZE_BYTES = 5 * 1024 * 1024  # 5 MB
MAX_BACKUP_COUNT = 5


class SessionCommunicationLogger:
    """Thread-safe persistent session and communication event logger."""

    def __init__(self, session_id: str, log_dir: str = DEFAULT_LOG_DIR, max_bytes: int = MAX_LOG_SIZE_BYTES):
        self.session_id = session_id
        self.log_dir = log_dir
        self.max_bytes = max_bytes
        self._lock = threading.Lock()

        os.makedirs(self.log_dir, exist_ok=True)
        self.log_path = os.path.join(self.log_dir, f"session_comm_{self.session_id}.jsonl")
        self.log_event("SESSION_START", {"status": "INITIALIZED", "path": self.log_path})

    def log_event(self, event_type: str, details: Dict[str, Any], severity: str = "INFO"):
        """Appends a structured event record to the session log with rotation check."""
        record = {
            "session_id": self.session_id,
            "timestamp": round(time.time(), 3),
            "iso_time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "event_type": str(event_type),
            "severity": str(severity),
            "details": details
        }
        line = json.dumps(record, separators=(',', ':')) + "\n"

        with self._lock:
            try:
                self._check_rotation()
                with open(self.log_path, "a", encoding="utf-8") as f:
                    f.write(line)
            except Exception:
                pass

    def _check_rotation(self):
        """Rotates log file if it exceeds maximum allowed size."""
        if not os.path.exists(self.log_path):
            return
        try:
            if os.path.getsize(self.log_path) >= self.max_bytes:
                for i in range(MAX_BACKUP_COUNT - 1, 0, -1):
                    sfn = f"{self.log_path}.{i}"
                    dfn = f"{self.log_path}.{i + 1}"
                    if os.path.exists(sfn):
                        os.replace(sfn, dfn)
                os.replace(self.log_path, f"{self.log_path}.1")
        except Exception:
            pass

    def log_connection_change(self, old_state: str, new_state: str, rssi: float, loss: float):
        self.log_event("CONNECTION_CHANGE", {
            "from": old_state,
            "to": new_state,
            "rssi": round(rssi, 1),
            "packet_loss": round(loss, 2)
        }, severity="WARN" if new_state in ("DISCONNECTED", "INTERMITTENT") else "INFO")

    def log_command_ack(self, cmd_id: str, cmd_type: str, status: str, error_code: str = "NONE"):
        self.log_event("COMMAND_ACK", {
            "cmd_id": cmd_id,
            "cmd_type": cmd_type,
            "status": status,
            "error_code": error_code
        })

    def log_critical_alert(self, alert_type: str, message: str, location: Optional[tuple] = None):
        self.log_event("CRITICAL_ALERT", {
            "alert_type": alert_type,
            "message": message,
            "location": location
        }, severity="CRITICAL")

    def close(self):
        self.log_event("SESSION_END", {"status": "CLOSED"})
