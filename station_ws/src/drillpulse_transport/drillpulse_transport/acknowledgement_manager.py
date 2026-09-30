#!/usr/bin/env python3
# ============================================================
# DRILLPULSE — Command Acknowledgement & Idempotency Manager
# Platform: Ubuntu 24.04 LTS + ROS 2 Jazzy + Python 3.12
#
# Implements:
#   - Full command lifecycle tracking:
#     REQUESTED -> RECEIVED -> ACCEPTED -> EXECUTING -> COMPLETED / FAILED
#   - Explicit error codes and ACK statuses
#   - Outgoing command tracking and retry watchdog
#   - Idempotency protection: detects and rejects duplicate command execution
# ============================================================

import time
import threading
from typing import Dict, Any, Optional, Tuple, Set


class AckStatusEnum:
    REQUESTED = "REQUESTED"
    RECEIVED = "RECEIVED"
    ACCEPTED = "ACCEPTED"
    REJECTED = "REJECTED"
    EXECUTING = "EXECUTING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    TIMEOUT = "TIMEOUT"


class ErrorCodeEnum:
    NONE = "NONE"
    IDEMPOTENT_DUPLICATE = "IDEMPOTENT_DUPLICATE"
    SAFETY_INTERLOCK = "SAFETY_INTERLOCK"
    TIMEOUT = "TIMEOUT"
    INVALID_PAYLOAD = "INVALID_PAYLOAD"
    RECOVERY_ACTIVE = "RECOVERY_ACTIVE"
    UNKNOWN_COMMAND = "UNKNOWN_COMMAND"


class OutgoingCommandEntry:
    def __init__(self, cmd_id: str, cmd_type: str, payload: Dict[str, Any], max_retries: int = 3, timeout_sec: float = 2.0):
        self.cmd_id = cmd_id
        self.cmd_type = cmd_type
        self.payload = payload
        self.status = AckStatusEnum.REQUESTED
        self.error_code = ErrorCodeEnum.NONE
        self.created_at = time.time()
        self.last_sent_at = self.created_at
        self.retry_count = 0
        self.max_retries = max_retries
        self.timeout_sec = timeout_sec
        self.completed_at = 0.0


class AcknowledgementManager:
    """Manages command tracking, retries, and incoming duplicate prevention."""

    def __init__(self, default_timeout_sec: float = 2.0, max_retries: int = 3):
        self.default_timeout_sec = default_timeout_sec
        self.max_retries = max_retries
        self._lock = threading.Lock()

        # Outgoing command tracking
        self._outgoing: Dict[str, OutgoingCommandEntry] = {}

        # Idempotency cache: tracks recently executed commands (cmd_id -> execution timestamp)
        self._executed_commands: Dict[str, float] = {}
        self.idempotency_window_sec = 60.0

    def register_outgoing(self, cmd_id: str, cmd_type: str, payload: Dict[str, Any]) -> OutgoingCommandEntry:
        """Registers a new outgoing command awaiting remote ACK."""
        with self._lock:
            entry = OutgoingCommandEntry(cmd_id, cmd_type, payload, self.max_retries, self.default_timeout_sec)
            self._outgoing[cmd_id] = entry
            return entry

    def process_incoming_ack(self, cmd_id: str, status: str, error_code: str = ErrorCodeEnum.NONE) -> Optional[OutgoingCommandEntry]:
        """Updates outgoing command status upon receiving ACK from remote."""
        with self._lock:
            if cmd_id in self._outgoing:
                entry = self._outgoing[cmd_id]
                entry.status = status
                entry.error_code = error_code
                if status in (AckStatusEnum.COMPLETED, AckStatusEnum.FAILED, AckStatusEnum.REJECTED):
                    entry.completed_at = time.time()
                return entry
            return None

    def check_retries_and_timeouts(self) -> Tuple[list, list]:
        """Checks pending commands. Returns (list_of_commands_to_retry, list_of_timed_out_commands)."""
        now = time.time()
        to_retry = []
        timed_out = []

        with self._lock:
            for cmd_id, entry in list(self._outgoing.items()):
                if entry.status in (AckStatusEnum.REQUESTED, AckStatusEnum.RECEIVED, AckStatusEnum.ACCEPTED, AckStatusEnum.EXECUTING):
                    if (now - entry.last_sent_at) > entry.timeout_sec:
                        if entry.retry_count < entry.max_retries:
                            entry.retry_count += 1
                            entry.last_sent_at = now
                            to_retry.append(entry)
                        else:
                            entry.status = AckStatusEnum.TIMEOUT
                            entry.error_code = ErrorCodeEnum.TIMEOUT
                            entry.completed_at = now
                            timed_out.append(entry)

        return to_retry, timed_out

    def is_duplicate_incoming(self, cmd_id: str) -> bool:
        """Checks if an incoming command has already been received/executed (idempotency guard)."""
        now = time.time()
        with self._lock:
            # Clean expired cache entries
            expired = [cid for cid, ts in self._executed_commands.items() if (now - ts) > self.idempotency_window_sec]
            for cid in expired:
                del self._executed_commands[cid]

            if cmd_id in self._executed_commands:
                return True
            self._executed_commands[cmd_id] = now
            return False

    def get_command_status(self, cmd_id: str) -> Optional[str]:
        with self._lock:
            entry = self._outgoing.get(cmd_id)
            return entry.status if entry else None
