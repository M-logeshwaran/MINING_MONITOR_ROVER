# DrillPulse Mission Logging & Post-Mission Reporting

## Overview

In underground mine safety operations, post-mission telemetry audit trails are mandatory for regulatory compliance (DGMS / Mine Safety Regulations) and incident investigation. The DrillPulse logging architecture (`drillpulse_transport/session_logger.py` and `drillpulse_report/report_generator_node.py`) maintains an immutable, power-cut resilient local audit log on the rover onboard storage, syncing asynchronously with the base station when communication links allow.

---

## 1. JSON Lines (.jsonl) Immutable Log Format

Standard databases or monolith JSON files can become corrupted if the rover experiences a sudden power loss or brownout. DrillPulse writes all session data as newline-delimited JSON Lines (`.jsonl`), ensuring every entry is an independent, valid JSON object written via `fsync`.

---

## 2. Storage Protection & File Rotation

- **Chunk Size Limit**: Active session logs rotate automatically upon reaching **5 MB**.
- **Rotation Filename Pattern**: `logs/drillpulse_session_<YYYYMMDD_HHMMSS>_part<N>.jsonl`
- **Disk Space Safeguard**: The session logger monitors available onboard flash memory. If free disk space drops below 500 MB, older non-critical logs are compressed into `.gz` archives, and lowest-priority bulk telemetry is pruned while preserving all safety, incident, and pose records.

---

## 3. Automated Post-Mission Report Generation

At the conclusion of a mission (or upon operator request), `report_generator_node.py` parses the session logs to synthesize a comprehensive engineering and safety report containing:
1. Executive Mission Summary (duration, distance traversed, energy consumed).
2. Hazard & Environmental Exposure Profile (peak CH4, CO, temperature gradient).
3. Safety Events & Anomaly Table (warnings, critical alerts, rollover recoveries).
4. Traversal Trajectory & Cartography overlay.
