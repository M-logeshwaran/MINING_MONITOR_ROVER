#!/usr/bin/env python3
# ============================================================
# DRILLPULSE — Priority-Aware Bounded Telemetry & Event Queue
# Platform: Ubuntu 24.04 LTS + ROS 2 Jazzy + Python 3.12
#
# Implements:
#   - Priority 0: EMERGENCY / CRITICAL (Rollover, E-stop, Fire/Gas, etc.)
#   - Priority 1: SAFETY / MISSION (Battery, State, Pose, Nav, Health)
#   - Priority 2: NORMAL TELEMETRY (Temp, Gas, Speed, IMU, Diagnostics)
#   - Priority 3: BULK / HIGH-BANDWIDTH (Video meta, Maps, Large reports)
#
# Guarantees:
#   - Strictly bounded memory (max_size) to prevent RAM growth.
#   - Priority-aware eviction: oldest low-priority items discarded first.
#   - Priority 0 (Critical) events are strictly preserved and never dropped
#     by lower-priority items.
#   - Thread-safe non-blocking queue operations.
# ============================================================

import time
import threading
from collections import deque
from typing import Dict, Any, List, Optional


class TelemetryPriority:
    EMERGENCY = 0   # Rollover, E-Stop, Gas Alarm, Batt Critical, Comms Lost
    SAFETY = 1      # Battery %, State, Pose, Nav Status, Sensor Health
    NORMAL = 2      # Ambient Sensors, Speed, Wheel Odometry Diagnostics
    BULK = 3        # Large Map Payloads, Reports, Video Sync Metadata


class QueueItem:
    """Encapsulates a prioritized telemetry/event packet."""
    def __init__(self, item_id: str, priority: int, msg_type: str, payload: Dict[str, Any], session_id: str = ""):
        self.item_id = item_id
        self.priority = int(priority)
        self.msg_type = str(msg_type)
        self.payload = dict(payload)
        self.session_id = str(session_id)
        self.timestamp = time.time()
        self.retry_count = 0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.item_id,
            "pri": self.priority,
            "type": self.msg_type,
            "session": self.session_id,
            "ts": self.timestamp,
            "data": self.payload
        }


class PriorityTelemetryQueue:
    """Thread-safe bounded priority queue with graceful degradation eviction."""

    def __init__(self, max_size: int = 500):
        self.max_size = max(1, int(max_size))
        self._lock = threading.Lock()
        # Separate deques for O(1) push and pop per priority level
        self._buckets = {
            TelemetryPriority.EMERGENCY: deque(),
            TelemetryPriority.SAFETY: deque(),
            TelemetryPriority.NORMAL: deque(),
            TelemetryPriority.BULK: deque()
        }
        self.total_dropped_low_pri = 0
        self.total_critical_enqueued = 0

    def push(self, item: QueueItem) -> bool:
        """Pushes an item into the queue. Evicts lowest-priority item if full."""
        with self._lock:
            current_total = sum(len(b) for b in self._buckets.values())

            if current_total >= self.max_size:
                # Need to evict an item to make room.
                # Never evict a higher or equal priority if lower exists.
                # Try evicting from BULK (3) first, then NORMAL (2), then SAFETY (1).
                evicted = False
                for pri_level in (TelemetryPriority.BULK, TelemetryPriority.NORMAL, TelemetryPriority.SAFETY):
                    if pri_level > item.priority and self._buckets[pri_level]:
                        self._buckets[pri_level].popleft()
                        self.total_dropped_low_pri += 1
                        evicted = True
                        break

                if not evicted:
                    # No lower-priority item available.
                    # If this is Priority 0, we can drop from Priority 0 itself if completely full of P0,
                    # but if this is P1, P2, P3 and queue is full of equal/higher, reject new item.
                    if item.priority == TelemetryPriority.EMERGENCY:
                        if self._buckets[TelemetryPriority.EMERGENCY]:
                            self._buckets[TelemetryPriority.EMERGENCY].popleft()
                            evicted = True
                    else:
                        # Cannot evict; discard incoming low-priority item
                        self.total_dropped_low_pri += 1
                        return False

            self._buckets[item.priority].append(item)
            if item.priority == TelemetryPriority.EMERGENCY:
                self.total_critical_enqueued += 1
            return True

    def pop_highest(self) -> Optional[QueueItem]:
        """Pops and returns the highest priority available item."""
        with self._lock:
            for pri in (TelemetryPriority.EMERGENCY, TelemetryPriority.SAFETY, TelemetryPriority.NORMAL, TelemetryPriority.BULK):
                if self._buckets[pri]:
                    return self._buckets[pri].popleft()
            return None

    def drain_batch(self, max_items: int = 10) -> List[QueueItem]:
        """Drains up to max_items in strict priority order for non-blocking burst sync."""
        batch = []
        with self._lock:
            while len(batch) < max_items:
                item = None
                for pri in (TelemetryPriority.EMERGENCY, TelemetryPriority.SAFETY, TelemetryPriority.NORMAL, TelemetryPriority.BULK):
                    if self._buckets[pri]:
                        item = self._buckets[pri].popleft()
                        break
                if item is None:
                    break
                batch.append(item)
        return batch

    def peek_highest(self) -> Optional[QueueItem]:
        with self._lock:
            for pri in (TelemetryPriority.EMERGENCY, TelemetryPriority.SAFETY, TelemetryPriority.NORMAL, TelemetryPriority.BULK):
                if self._buckets[pri]:
                    return self._buckets[pri][0]
            return None

    def size(self) -> int:
        with self._lock:
            return sum(len(b) for b in self._buckets.values())

    def size_by_priority(self) -> Dict[int, int]:
        with self._lock:
            return {pri: len(b) for pri, b in self._buckets.items()}

    def clear(self):
        with self._lock:
            for b in self._buckets.values():
                b.clear()
