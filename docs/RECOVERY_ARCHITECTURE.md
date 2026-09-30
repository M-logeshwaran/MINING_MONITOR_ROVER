# DRILLPULSE — ROLLOVER RECOVERY ARCHITECTURE
**Step 5: Directional Actuation, Controlled Recovery FSM & Motor Interlocking**
**Date:** 2026-09-29
**Platform:** Ubuntu 24.04 LTS, ROS 2 Jazzy, Python 3.12

---

## 1. Physical Self-Righting Mechanism Overview

The DrillPulse Rover is equipped with dual high-thrust linear actuators mounted along the lateral flanks:
- **Left Linear Actuator:** Mounted on the LEFT flank, extending outward and downward toward the mine floor to push against the terrain and rotate the chassis clockwise when rolled onto the left side.
- **Right Linear Actuator:** Mounted on the RIGHT flank, extending outward and downward to rotate the chassis counter-clockwise when rolled onto the right side.
- **Physical Safety Invariant:** Dual simultaneous extension is strictly disallowed to prevent mechanical collision, chassis binding, and power bus overload.

---

## 2. IMU Rollover Detection & Debounce Algorithm

1. **Roll Calculation:**
   $$\text{roll} = \text{atan2}(a_y, a_z) \times \frac{180.0}{\pi}$$
2. **Debounce Confirmation:**
   - Candidate Trigger: $|\text{roll}| > 45.0^\circ$.
   - Must persist continuously for $\ge 0.30\text{s}$ (`detection_confirmation_time`). Single noisy samples are filtered out.
3. **Direction Mapping:**
   - $\text{roll} < 0 \rightarrow$ **LEFT ROLLOVER** $\rightarrow$ Activates **LEFT Actuator** (`L_EXT`).
   - $\text{roll} > 0 \rightarrow$ **RIGHT ROLLOVER** $\rightarrow$ Activates **RIGHT Actuator** (`R_EXT`).
4. **Hysteresis Recovery Threshold:**
   - Rollover status only clears once $|\text{roll}| \le 15.0^\circ$ (`roll_stable_threshold_deg`).

---

## 3. Controlled Recovery FSM

```
[IDLE]
  | Rollover Confirmed (|roll| > 45 deg, dt >= 0.3s) or Manual (Key 6/7)
  v
[ACTUATOR_PRECHECK]  (Verify E-Stop, battery > 15%, cooldown >= 2.0s, attempts <= 3)
  | Pre-checks PASS
  v
[EXTENDING]  (Target actuator extends; max duration: 3.5s; mobility locked)
  | |roll| <= 15 deg OR timeout >= 3.5s
  v
[RETRACTING] (Actuator commanded RESET / RETRACT_ALL; duration: 2.0s)
  | Retraction complete
  v
[ORIENTATION_CHECK]
  +---> [RECOVERED] (|roll| <= 15 deg) ---> [IDLE] (Reset attempt counter to 0)
  |
  +---> Attempts < 3 ---> [COOLDOWN] (2.0s pause) ---> Retry
  |
  +---> Attempts >= 3 ---> [FAILED] (Latch fault, report error, require manual reset)
```

---

## 4. Safety Interlocks & Motor Coordination

- **Mobility Priority Ladder:** During any recovery phase (`EXTENDING`, `RETRACTING`, `ACTIVE`), `actuator_bridge` publishes `/rover/actuator_state`. `mobility_controller_node.py` detects active keywords and enforces `ACTUATOR_INTERLOCK` (priority level 2, below only E-Stop), immediately driving wheel velocity targets to zero.
- **Anti-Fighting Protection:** Teleoperation (WASD) and Nav2 autonomous commands are suppressed until recovery is fully completed and actuators are retracted.
- **Attempt Limiting:** Maximum 3 recovery attempts permitted. Prevents battery drain or actuator motor burn out if vehicle is physically wedged in rock strata.
- **Manual Overrides:**
  - **Key 6:** Initiates LEFT recovery stroke.
  - **Key 7:** Initiates RIGHT recovery stroke.
  - **ESC:** Immediate abort; halts actuator (`STOP_ALL`) and enters safe cooldown.
