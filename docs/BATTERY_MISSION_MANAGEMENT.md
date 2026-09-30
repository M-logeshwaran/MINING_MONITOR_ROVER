# DRILLPULSE — BATTERY & MISSION MANAGEMENT ARCHITECTURE
**Step 6 Technical Specification**
**Date:** 2026-09-30
**Platform:** Ubuntu 24.04 LTS, ROS 2 Jazzy, Python 3.12
**Target Nodes:** `MissionManagerNode`, `ReportGeneratorNode`, `UnifiedMobilityControllerNode`

---

## 1. Executive Summary

In subterranean mine operations, battery exhaustion deep within unmapped tunnels results in permanent vehicle loss or hazardous human retrieval missions. The rover must autonomously track power reserves, enforce a strict four-tier battery degradation progression, calculate remaining operating time from real-time current draw, and guarantee that enough energy remains to return to the deployment entry point.

---

## 2. Four-Tier Battery Degradation State Machine

The battery progression is evaluated on every `/rover/battery` (or `/sparkfun/battery`) message:

```
+-----------------------------------------------------------------------------------+
|                            BATTERY STATE PROGRESSION                              |
|                                                                                   |
|      Voltage > 11.4V  &  SoC > 35%                                                |
|   +------------------------------------+                                          |
|   |              NORMAL                |  (Normal exploration and mapping active) |
|   +-----------------+------------------+                                          |
|                     | Voltage < 11.0V or SoC < 25%                                |
|                     v                                                             |
|   +------------------------------------+                                          |
|   |               LOW                  |  (Operator warned; dashboard yellow badge)|
|   +-----------------+------------------+                                          |
|                     | Voltage < 10.8V or SoC < 20%                                |
|                     v                                                             |
|   +------------------------------------+                                          |
|   |          RETURN_REQUIRED           |  (Frontier exploration cancelled;         |
|   +-----------------+------------------+   autonomous return-to-start dispatched) |
|                     | Voltage < 10.5V or SoC < 15%                                |
|                     v                                                             |
|   +------------------------------------+                                          |
|   |              CRITICAL              |  (Emergency priority return; non-essential|
|   +------------------------------------+   sensors throttled; recovery actuators  |
|                                            interlocked against high-power draw)   |
+-----------------------------------------------------------------------------------+
```

### Parameter Reference
| Parameter Name | Default Value | Description |
|---|---|---|
| `battery_normal_soc` | `35.0%` | Nominal operating threshold |
| `battery_low_soc` | `25.0%` | Warning threshold |
| `battery_return_soc` | `20.0%` | Mandatory return-to-base trigger threshold |
| `battery_critical_soc` | `15.0%` | Priority failsafe threshold |
| `battery_normal_v` | `11.4 V` | Nominal 3S pack voltage |
| `battery_low_v` | `11.0 V` | Low voltage alarm |
| `battery_return_v` | `10.8 V` | Return pack voltage trigger |
| `battery_critical_v` | `10.5 V` | Critical voltage cutoff warning |
| `battery_capacity_ah` | `10.0 Ah` | Nominal onboard battery pack capacity |

---

## 3. Physical Runtime Estimation Formula

Rather than presenting arbitrary percentages, DrillPulse computes the physical remaining runtime using:
1. Measured State-of-Charge ($\text{SoC} \in [0.0, 1.0]$)
2. Nominal pack capacity ($C_{\text{battery}} = 10.0\text{ Ah}$)
3. Instantaneous or filtered current draw ($I_{\text{draw}}$ in Amperes)

$$\text{Runtime}_{\text{min}} = \frac{\text{SoC} \cdot C_{\text{battery}}}{\max(0.5, I_{\text{draw}})} \cdot 60\text{ minutes}$$

### Edge Case Handling & Fallbacks
- **Missing Telemetry:** If voltage or SoC contains `NaN`, is outside physical bounds ($<5\text{V}$ or $>20\text{V}$), or no battery message has arrived, runtime is strictly set to `None` (displaying `-- min` on dashboard), preventing false operator confidence.
- **Minimum Current Clamping:** Current draw is clamped to $\ge 0.5\text{ A}$ (rover idle quiescent draw) to prevent division by zero or unrealistic infinite runtimes when motors are stationary.

---

## 4. Nav2 Return-to-Start Integration

### 1. Mission Start Pose Locking
- When the first odometry message is received, `MissionManagerNode` locks the coordinates:
  $$(x_{\text{start}}, y_{\text{start}}, \theta_{\text{start}}) = (x_0, y_0, \theta_0)$$
- This origin point is recorded into memory and logged to the SD-card audit log.

### 2. Return Triggering
Return can be triggered autonomously or by the operator:
- `OPERATOR`: Manual "RETURN TO BASE" command from dashboard UI.
- `BATTERY_RETURN`: Battery SoC drops below $20\%$.
- `CRITICAL_BATTERY`: Battery SoC drops below $15\%$.
- `COMM_LOSS`: Radio silence exceeding $5.0\text{s}$.
- `HAZARD_ABORT`: Lethal gas concentration or rollover limit reached.

### 3. Missing Start Pose Protection
If a return is commanded before odometry has locked the start pose:
- Rover does **NOT** drive blindly to $(0,0)$.
- Rover transitions to state `HOLD_POSITION` with phase `RETURN_TARGET_UNAVAILABLE`.
- Velocity commands are zeroed and an audible/dashboard alarm is generated.

### 4. Goal Dispatch & Progress Tracking
- Dispatches Nav2 goal to $(x_{\text{start}}, y_{\text{start}}, \theta_{\text{start}})$ with frame ID `map`.
- Continuously evaluates distance:
  $$d_{\text{target}} = \sqrt{(x_{\text{start}} - x)^2 + (y_{\text{start}} - y)^2}$$
- **Arrival Condition:** When $d_{\text{target}} \le 0.35\text{ m}$ (tolerance radius), navigation halts, state transitions to `RETURNED_TO_START`, and phase becomes `AT_START_SAFE`.

### 5. Stalled Progress & Retry Escalation
- If distance to goal does not decrease by $\ge 0.05\text{ m}$ over a $30.0\text{s}$ window (`nav_stuck_timeout_sec`), the rover retries goal dispatch.
- Up to 3 retries (`max_return_retries = 3`) are permitted.
- Upon exceeding maximum retries, the rover enters `HOLD_POSITION` with phase `NAV_FAILED`, cuts motor drive, and sounds location beaconing.

---

## 5. Persistent SD-Card Mission Logging

`ReportGeneratorNode` continuously logs records in asynchronous JSON-Lines (`.jsonl`) format at:
`/home/loki/SIH_FINAL_PROTOTYPE/logs/sd_mission_{session_id}.jsonl`

Logged event types include:
- `SESSION_START`: Session ID, initial timestamp, log paths.
- `TELEMETRY`: Battery %, voltage, current, runtime, environmental gases, pose, speed, link quality.
- `HAZARD`: Hazard type, severity, threshold, measured value, $(x, y)$ coordinates.
- `STATE_TRANSITION`: FSM state transitions, active job, frontier counts.
- `RETURN_TRIGGER`: Return reason, origin coordinates, return target coordinates.
- `LINK_TRANSITION`: Link state changes (`GOOD`, `DEGRADED`, `WEAK`, `CRITICAL`, `LOST`), RSSI, packet loss.
- `COMM_LOSS_EVENT`: Timestamped connection loss alerts.
- `SAFETY_ALERT`: E-Stop triggers, rollover risks, sensor fault codes.
- `COMMAND_RECEIVED`: Operator and autonomous commands.

Upon mission completion or operator request, a standalone HTML and JSON summary audit report is generated in `/home/loki/SIH_FINAL_PROTOTYPE/reports/`.
