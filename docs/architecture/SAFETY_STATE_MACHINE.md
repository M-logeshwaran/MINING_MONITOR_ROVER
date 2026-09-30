# DRILLPULSE — ONBOARD SAFETY STATE MACHINE
**Step 5: Deterministic 12-State FSM, Priority Arbitration & Transition Logging**
**Date:** 2026-09-29
**Platform:** Ubuntu 24.04 LTS, ROS 2 Jazzy, Python 3.12

---

## 1. Safety Architecture & Deterministic State Machine Overview

The DrillPulse Rover incorporates a deterministic onboard Safety Monitor (`drillpulse_safety/safety_monitor_node.py`). Safety decisions are computed **onboard the vehicle** and do not depend on the operator workstation or ground station.

```
+-----------------------------------------------------------------------------------+
|                         ONBOARD SAFETY STATE MACHINE                              |
|                                                                                   |
|  [NORMAL] <------------------------------------+                                  |
|     |                                          | Recovery Success                 |
|     +--> [CAUTION] (High Tilt / Minor Warn)    | or Clear                         |
|     |                                          |                                  |
|     +--> [DEGRADED] (Sensor Fault / Mismatch)  |                                  |
|     |                                          |                                  |
|     +--> [LOW_BATTERY] (SoC < 25% or V < 11.2V)|                                  |
|     |                                          |                                  |
|     +--> [CONNECTION_LOST] (Silence > 5.0s)    |                                  |
|     |       \                                  |                                  |
|     |        +---> [RETURNING] (Nav2 Origin)   |                                  |
|     |                 \                        |                                  |
|     |                  +--> [MISSION_COMPLETE] |                                  |
|     |                                          |                                  |
|     +--> [ROLLOVER_DETECTED]                   |                                  |
|             \                                  |                                  |
|              +--> [RECOVERY_READY]             |                                  |
|                      \                         |                                  |
|                       +--> [RECOVERY_ACTIVE] --+                                  |
|                               \                                                   |
|                                +--> [RECOVERY_FAILED]                             |
|                                                                                   |
|  ======================= HIGHEST PRIORITY OVERRIDE =============================  |
|  [EMERGENCY_STOP] (Latched, Immediate Zero Velocity, Hardware Actuator Lock)      |
+-----------------------------------------------------------------------------------+
```

---

## 2. Twelve Formal Safety States

| State | Priority | Description | Motion Permitted? | Recovery Permitted? |
|---|---|---|---|---|
| **`EMERGENCY_STOP`** | 1 (Highest) | Operator manual stop or fatal safety fault; latched until explicit reset | **NO** | **NO** |
| **`ROLLOVER_DETECTED`** | 2 | Sustained roll angle $> 45.0^\circ$ for $> 0.30\text{s}$ confirmation | **NO** | **YES** |
| **`RECOVERY_READY`** | 3 | Actuator pre-checks satisfied; prepared for stroke execution | **NO** | **YES** |
| **`RECOVERY_ACTIVE`** | 4 | Linear actuator actively extending/retracting to self-right vehicle | **NO** | **ACTIVE** |
| **`RECOVERY_FAILED`** | 5 | Self-righting unsuccessful after 3 attempts or stroke timeout | **NO** | Manual Only |
| **`LOW_BATTERY`** | 6 | Battery $< 25\%$ SoC; if $< 15\%$, triggers return to start | **YES** (Throttled) | Conditional |
| **`CONNECTION_LOST`** | 7 | Low-bandwidth link silent $> 5.0\text{s}$; initiates safe return or stop | **YES** (Autonomous) | **NO** |
| **`RETURNING`** | 8 | Autonomous navigation executing path back to locked start pose $(x_0, y_0)$ | **YES** (Autonomous) | **NO** |
| **`DEGRADED`** | 9 | Sensor timeout or cross-sensor disagreement detected | **YES** (Cautious) | **NO** |
| **`CAUTION`** | 10 | Roll/pitch angle $> 30.0^\circ$ but below rollover threshold | **YES** | **NO** |
| **`NORMAL`** | 11 | All sensors valid, orientation nominal, battery $> 25\%$, link active | **YES** | **NO** |
| **`MISSION_COMPLETE`**| 12 | Rover arrived safely at destination / return location | **NO** (Standby) | **NO** |

---

## 3. Transition Event Logging Schema

Every state transition records an audit record emitted to `/rover/safety_events` as JSON:
```json
{
  "previous_state": "NORMAL",
  "new_state": "ROLLOVER_DETECTED",
  "reason": "Rollover confirmed on LEFT side (Roll: -52.4 deg)",
  "timestamp": 1790704260.123,
  "triggering_sensor": "IMU",
  "severity": "CRITICAL"
}
```
Severities conform to: `INFO`, `WARNING`, `CRITICAL`, `FATAL`.
