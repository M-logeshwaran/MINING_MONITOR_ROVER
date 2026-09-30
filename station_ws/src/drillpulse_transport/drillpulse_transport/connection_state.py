#!/usr/bin/env python3
# ============================================================
# DRILLPULSE — Connection Quality & Link State Decision Engine
# Platform: Ubuntu 24.04 LTS + ROS 2 Jazzy + Python 3.12
#
# Implements:
#   - Explicit states: CONNECTED_GOOD, CONNECTED_DEGRADED, CONNECTED_WEAK,
#     INTERMITTENT, DISCONNECTED, RECOVERING
#   - Multi-metric evaluation (RSSI, packet loss, latency, timeouts, retries,
#     queue backlog, link availability)
#   - Configurable parameter thresholds
#   - Adaptive transmission rate mapping
#   - Wi-Fi HaLow and composite overall link evaluation
# ============================================================

import time
from typing import Dict, Any, Tuple


class LinkStateEnum:
    CONNECTED_GOOD = "CONNECTED_GOOD"
    CONNECTED_DEGRADED = "CONNECTED_DEGRADED"
    CONNECTED_WEAK = "CONNECTED_WEAK"
    INTERMITTENT = "INTERMITTENT"
    DISCONNECTED = "DISCONNECTED"
    RECOVERING = "RECOVERING"


class OverallLinkStateEnum:
    GOOD = "GOOD"
    DEGRADED = "DEGRADED"
    WEAK = "WEAK"
    INTERMITTENT = "INTERMITTENT"
    DISCONNECTED = "DISCONNECTED"


class WifiLinkStateEnum:
    CONNECTED = "CONNECTED"
    WEAK = "WEAK"
    LOST = "LOST"


class ConnectionQualityEvaluator:
    """Evaluates multi-metric link quality and decides canonical communication state."""

    def __init__(
        self,
        rssi_good: float = -75.0,
        rssi_degraded: float = -90.0,
        rssi_weak: float = -105.0,
        loss_degraded: float = 0.15,
        loss_weak: float = 0.40,
        loss_critical: float = 0.70,
        latency_good_ms: float = 120.0,
        latency_degraded_ms: float = 350.0,
        latency_weak_ms: float = 800.0,
        heartbeat_timeout_sec: float = 4.0,
        recovery_window_sec: float = 3.0
    ):
        self.rssi_good = rssi_good
        self.rssi_degraded = rssi_degraded
        self.rssi_weak = rssi_weak
        self.loss_degraded = loss_degraded
        self.loss_weak = loss_weak
        self.loss_critical = loss_critical
        self.latency_good_ms = latency_good_ms
        self.latency_degraded_ms = latency_degraded_ms
        self.latency_weak_ms = latency_weak_ms
        self.heartbeat_timeout_sec = heartbeat_timeout_sec
        self.recovery_window_sec = recovery_window_sec

        self.last_state = LinkStateEnum.CONNECTED_GOOD
        self.disconnect_time = 0.0
        self.recovery_start_time = 0.0
        self.consecutive_timeouts = 0
        self.last_heartbeat_time = time.time()

    def evaluate(
        self,
        rssi: float,
        packet_loss: float,
        latency_ms: float,
        last_recv_time: float,
        retry_count: int = 0,
        queue_backlog: int = 0,
        link_hardware_ok: bool = True
    ) -> str:
        """Determines explicit link state based on combined communication metrics."""
        now = time.time()
        elapsed_silence = now - last_recv_time

        if not link_hardware_ok or elapsed_silence > self.heartbeat_timeout_sec:
            if self.last_state != LinkStateEnum.DISCONNECTED:
                self.disconnect_time = now
                self.consecutive_timeouts += 1
            self.last_state = LinkStateEnum.DISCONNECTED
            return LinkStateEnum.DISCONNECTED

        # If we were disconnected and just resumed communication
        if self.last_state == LinkStateEnum.DISCONNECTED:
            self.recovery_start_time = now
            self.last_state = LinkStateEnum.RECOVERING
            return LinkStateEnum.RECOVERING

        if self.last_state == LinkStateEnum.RECOVERING:
            if (now - self.recovery_start_time) < self.recovery_window_sec:
                return LinkStateEnum.RECOVERING

        # Check for INTERMITTENT behavior (frequent timeouts, high loss, or high retries)
        if packet_loss >= self.loss_critical or retry_count >= 3 or (packet_loss >= 0.45 and self.consecutive_timeouts > 1):
            self.last_state = LinkStateEnum.INTERMITTENT
            return LinkStateEnum.INTERMITTENT

        # Check Signal & Latency boundaries
        if rssi >= self.rssi_good and packet_loss < self.loss_degraded and latency_ms <= self.latency_good_ms:
            self.consecutive_timeouts = 0
            self.last_state = LinkStateEnum.CONNECTED_GOOD
            return LinkStateEnum.CONNECTED_GOOD
        elif rssi >= self.rssi_degraded and packet_loss < self.loss_weak and latency_ms <= self.latency_degraded_ms:
            self.last_state = LinkStateEnum.CONNECTED_DEGRADED
            return LinkStateEnum.CONNECTED_DEGRADED
        elif rssi >= self.rssi_weak and packet_loss < self.loss_critical and latency_ms <= self.latency_weak_ms:
            self.last_state = LinkStateEnum.CONNECTED_WEAK
            return LinkStateEnum.CONNECTED_WEAK
        else:
            self.last_state = LinkStateEnum.INTERMITTENT
            return LinkStateEnum.INTERMITTENT

    def get_transmission_rate(self, state: str) -> float:
        """Maps link state to target transmission frequency (Hz)."""
        rate_table = {
            LinkStateEnum.CONNECTED_GOOD: 10.0,
            LinkStateEnum.CONNECTED_DEGRADED: 3.0,
            LinkStateEnum.CONNECTED_WEAK: 1.0,
            LinkStateEnum.INTERMITTENT: 0.5,
            LinkStateEnum.DISCONNECTED: 0.2,   # Probe ping rate
            LinkStateEnum.RECOVERING: 2.0
        }
        return rate_table.get(state, 1.0)

    @staticmethod
    def get_overall_state(lora_state: str, wifi_connected: bool) -> str:
        """Synthesizes overall dashboard communication health indicator."""
        if lora_state == LinkStateEnum.DISCONNECTED and not wifi_connected:
            return OverallLinkStateEnum.DISCONNECTED
        elif lora_state == LinkStateEnum.INTERMITTENT:
            return OverallLinkStateEnum.INTERMITTENT
        elif lora_state == LinkStateEnum.CONNECTED_WEAK or (lora_state == LinkStateEnum.DISCONNECTED and wifi_connected):
            return OverallLinkStateEnum.WEAK
        elif lora_state == LinkStateEnum.CONNECTED_DEGRADED:
            return OverallLinkStateEnum.DEGRADED
        elif lora_state == LinkStateEnum.CONNECTED_GOOD:
            return OverallLinkStateEnum.GOOD
        elif lora_state == LinkStateEnum.RECOVERING:
            return OverallLinkStateEnum.DEGRADED
        return OverallLinkStateEnum.DISCONNECTED
