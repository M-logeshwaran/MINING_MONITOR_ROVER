# DrillPulse — Canonical Interface Audit & Reconciliation Report

**Document:** `docs/CANONICAL_INTERFACE_AUDIT.md`  
**Status:** Audit & Resolution Record  
**Target:** Ubuntu 24.04 LTS | ROS 2 Jazzy | Python 3.12

---

## 1. Executive Summary

During Step 1 audit of `SIH_FINAL_PROTOTYPE`, significant inconsistencies and discrepancies were discovered across rover and station packages:
1. **Message Definition Inconsistencies:** `drillpulse_msgs` differed in fields and schemas between `rover_ws` and `station_ws`.
2. **Topic Divergence:** Nodes used fragmented topic names (e.g., `/drillpulse/rover_telemetry`, `/drillpulse/telemetry`, `/mission/state`).
3. **Transport Protocol Fragmentation:** Transceiver implementations had differing CRC, packet types, and JSON wire formats.
4. **Duplicate Packages:** Redundant copies of `lora_transceiver_node` and `report_generator_node` were found across workspace trees.
5. **Hardware Bridge Assumptions:** Microcontroller communication protocols were unrecorded, and microcontroller source files were absent.

---

## 2. Reconciled Canonical Contract

### 2.1 Reconciled Messages (`drillpulse_msgs`)
All 12 message specifications are synchronized 1:1 between `rover_ws` and `station_ws`:
- `RoverTelemetry.msg`
- `OdometryDiagnostics.msg`
- `MissionState.msg`
- `MissionCommand.msg`
- `HazardEvent.msg`
- `EnvironmentState.msg`
- `RecoveryCommand.msg`
- `RecoveryStatus.msg`
- `SafetyStatus.msg`
- `NavigationStatus.msg`
- `LinkStatus.msg`
- `JobAssignment.msg`

### 2.2 Reconciled Topics
- Primary telemetry topic: `/rover/telemetry`
- Primary mission state topic: `/rover/mission_state`
- Primary hazard topic: `/rover/hazard_event`
- Primary link status: `/rover/link_status`
- Primary recovery status: `/rover/recovery_status`
- Primary safety status: `/rover/safety_status`
- Primary navigation status: `/rover/navigation_status`
- Primary velocity command: `/rover/cmd_vel`
- Primary emergency stop: `/rover/emergency_stop`

### 2.3 LoRa Transport V2 Protocol
- Wire framing: `DP2:<JSON_PAYLOAD>#<CRC32_HEX>`
- Strict sequence tracking and deduplication
- Bidirectional ACK handshake for mission-critical jobs and commands

### 2.4 Duplicate Code Removal
- Removed duplicate nodes from `drillpulse_mission`
- Removed redundant standalone copies from `drillpulse_station`
- Standardized `drillpulse_transport` and `drillpulse_report` as single canonical packages.
