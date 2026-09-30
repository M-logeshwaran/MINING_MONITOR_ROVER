# DRILLPULSE — MISSION ORCHESTRATION ARCHITECTURE & SPECIFICATION
**Autonomous Exploration, Job Lifecycle, Failsafe Return, and Multi-Modal Coordination**
**Platform:** Ubuntu 24.04 LTS, ROS 2 Jazzy, Python 3.12
**Package:** `drillpulse_mission` & `drillpulse_dashboard`
**Project:** `/home/loki/SIH_FINAL_PROTOTYPE`

---

## 1. Subsystem Architecture Overview

The DrillPulse Mission Orchestration layer coordinates high-level autonomy, exploration, and return maneuvers across all underground mine operations. It integrates:
- **Canonical 18-State Finite State Machine (FSM):** Complete deterministic transitions across mission life stages.
- **Job Validation & Lifecycle Engine:** Machine-readable validation rejecting invalid jobs with strict diagnostic codes (`INVALID_JOB`, `INVALID_MODE`, `BATTERY_TOO_LOW`, `LOCALIZATION_UNAVAILABLE`, `SAFETY_LOCK`, `COMMUNICATION_UNSTABLE`).
- **Exploration vs Explored Operation:**
  - *Explored Mode:* AMCL localization against pre-surveyed static occupancy grids, Nav2 waypoint navigation.
  - *Unexplored Mode:* Online SLAM (`slam_toolbox` / `cartographer`), real-time occupancy grid frontier exploration.
- **Multi-Factor Frontier Scoring & Candidate Evaluation:** Frontier discovery with information gain bonus, distance penalty, obstacle clearance constraints, and commitment hysteresis.
- **Fail-Safe Return-to-Start Architecture:** True physical pose locking, return reason tracking, stalled Nav2 retry escalation, and recovery motion interlocks.
- **Hazard Association & Spatial Logging:** Live environmental and structural hazard registration tied to odometric map coordinates.
- **Interactive Click-to-Point Dashboard Targeting:** Coordinate transformation matrix mapping web canvas clicks $(cx, cy)$ to global navigation frames $(wx, wy)$, full lifecycle command tracking (`REQUESTED` $\to$ `ACCEPTED` $\to$ `EXECUTING` $\to$ `COMPLETED` / `FAILED`), and 5 distinct canvas visual markers.

```
+-----------------------------------------------------------------------------------+
|                            OPERATOR DASHBOARD (Web UI)                            |
|  - Canvas Map (Click-to-Point)       - Return-to-Base Trigger                     |
|  - 5 Marker Layers                   - Command Lifecycle Tracking                 |
+------------------------------------------+----------------------------------------+
                                           | WebSocket / SocketIO
                                           v
+-----------------------------------------------------------------------------------+
|                        STATION BRIDGE (dashboard_node)                            |
|  - /rover/job_assignment             - /mission/command                           |
|  - /rover/job_ack                    - /mission/command_ack                       |
+------------------------------------------+----------------------------------------+
                                           | LoRa / Wi-Fi HaLow ROS 2 Transport
                                           v
+-----------------------------------------------------------------------------------+
|                     MISSION MANAGER NODE (mission_manager_node)                   |
|  - 18-State Mission FSM              - Pose Locking (Physical Odometry)           |
|  - Job Assignment Validation (6 codes)- Return Watchdog (3-Retry Escalation)      |
|  - Safety & Recovery Interlocks      - Report Generation Trigger Dispatch         |
+---------------------+--------------------+----------------------------------------+
                      |                    |
        Explored Mode |                    | Unexplored Mode
                      v                    v
       +-----------------------+   +---------------------------------+
       |         Nav2          |   |     FRONTIER EXPLORATION NODE   |
       |  Waypoint Following   |   |   - Candidate Extraction        |
       |  AMCL Localization    |   |   - Multi-Factor Scoring        |
       +-----------------------+   |   - Hysteresis Commitment       |
                                   +----------------+----------------+
                                                    |
                                                    v
                                   +---------------------------------+
                                   |              Nav2               |
                                   |   SLAM Online Exploration       |
                                   +---------------------------------+
```

---

## 2. Canonical 18-State Mission Finite State Machine

The Mission Manager enforces deterministic transitions via `VALID_TRANSITIONS` preventing illegal state jumps.

### 2.1 State Definitions
| State Name | State Enum | Description |
|---|---|---|
| `IDLE` | `IDLE` | Rover booted, waiting for base station or mission assignment. |
| `STANDBY` | `STANDBY` | Pre-mission posture, communication online, ready for job ingest. |
| `JOB_RECEIVED` | `JOB_RECEIVED` | Incoming job payload under syntactic and structural inspection. |
| `JOB_ACCEPTED` | `JOB_ACCEPTED` | Job parameters validated; mission session ID generated and bound. |
| `INITIALIZING` | `INITIALIZING` | Subsystems booting; verifying sensor fusion and localization. |
| `READY` | `READY` | All pre-flight checks passed; start pose locked in map frame. |
| `NAVIGATING` | `NAVIGATING` | Autonomous waypoint traversal in known/surveyed mine sector (Explored). |
| `EXPLORING` | `EXPLORING` | Frontier exploration and active mapping in unknown sector (Unexplored). |
| `AUTONOMOUS_NAVIGATION` | `AUTONOMOUS_NAVIGATION` | Generic autonomous navigation state (alias/umbrella). |
| `MANUAL_OVERRIDE` | `MANUAL_OVERRIDE` | Operator teleoperation via low-bandwidth or Wi-Fi control channel. |
| `HAZARD_RESPONSE` | `HAZARD_RESPONSE` | Immediate halt or standoff triggered by environmental/structural danger. |
| `RETURNING` | `RETURNING` | Autonomous return-to-base navigation underway. |
| `RETURNING_COMM_LOSS` | `RETURNING_COMM_LOSS` | Return triggered specifically by sustained communication link failure. |
| `RETURNING_LOW_BATTERY`| `RETURNING_LOW_BATTERY`| Return triggered by battery capacity dropping below return threshold. |
| `RETURNED_TO_BASE` | `RETURNED_TO_BASE` | Rover has achieved proximity threshold ($\le 0.35\text{ m}$) to locked base pose. |
| `HOLD_POSITION` | `HOLD_POSITION` | Active position hold and motor freeze (e.g. Nav2 stall or unresolvable hazard). |
| `RECOVERY` | `RECOVERY` | Dual linear actuator self-righting sequence active; motion interlock engaged. |
| `EMERGENCY_STOP` | `EMERGENCY_STOP` | Hardware or software E-Stop active; zero velocity clamped; brakes locked. |
| `MISSION_COMPLETE` | `MISSION_COMPLETE` | Mission successfully closed; persistent SD report generated and archived. |

### 2.2 Canonical State Transition Matrix
```
IDLE                   -> STANDBY, JOB_RECEIVED, JOB_ACCEPTED, NAVIGATING, EXPLORING, RETURNING, RECOVERY, EMERGENCY_STOP
STANDBY                -> JOB_RECEIVED, JOB_ACCEPTED, INITIALIZING, READY, EMERGENCY_STOP
JOB_RECEIVED           -> JOB_ACCEPTED, STANDBY, IDLE
JOB_ACCEPTED           -> INITIALIZING, READY, NAVIGATING, EXPLORING, EMERGENCY_STOP
INITIALIZING           -> READY, STANDBY, HOLD_POSITION, EMERGENCY_STOP
READY                  -> NAVIGATING, EXPLORING, AUTONOMOUS_NAVIGATION, MANUAL_OVERRIDE, RETURNING, EMERGENCY_STOP
NAVIGATING             -> READY, HAZARD_RESPONSE, RETURNING, RETURNING_COMM_LOSS, RETURNING_LOW_BATTERY, HOLD_POSITION, RECOVERY, EMERGENCY_STOP, RETURNED_TO_BASE
EXPLORING              -> READY, HAZARD_RESPONSE, RETURNING, RETURNING_COMM_LOSS, RETURNING_LOW_BATTERY, HOLD_POSITION, RECOVERY, EMERGENCY_STOP, RETURNED_TO_BASE
AUTONOMOUS_NAVIGATION  -> READY, HAZARD_RESPONSE, RETURNING, RETURNING_COMM_LOSS, RETURNING_LOW_BATTERY, HOLD_POSITION, RECOVERY, EMERGENCY_STOP, RETURNED_TO_BASE
MANUAL_OVERRIDE        -> READY, STANDBY, HOLD_POSITION, RECOVERY, EMERGENCY_STOP
HAZARD_RESPONSE        -> RETURNING, HOLD_POSITION, MANUAL_OVERRIDE, EMERGENCY_STOP
RETURNING              -> RETURNED_TO_BASE, HOLD_POSITION, RECOVERY, EMERGENCY_STOP, RETURNING (reason escalation)
RETURNING_COMM_LOSS    -> RETURNED_TO_BASE, HOLD_POSITION, RECOVERY, EMERGENCY_STOP
RETURNING_LOW_BATTERY  -> RETURNED_TO_BASE, HOLD_POSITION, RECOVERY, EMERGENCY_STOP
RETURNED_TO_BASE       -> MISSION_COMPLETE, STANDBY, IDLE
HOLD_POSITION          -> READY, RETURNING, MANUAL_OVERRIDE, RECOVERY, EMERGENCY_STOP
RECOVERY               -> READY, HOLD_POSITION, RETURNING, EMERGENCY_STOP
EMERGENCY_STOP         -> READY, STANDBY, HOLD_POSITION, IDLE
MISSION_COMPLETE       -> IDLE, STANDBY
```

---

## 3. Job Validation & Machine-Readable Rejections

Before executing any job assignment (`drillpulse_msgs/msg/JobAssignment`), the Mission Manager validates system readiness against six canonical error conditions:

1. **`INVALID_JOB`**: Missing, blank, or malformed job assignment payload or missing targets when required.
2. **`INVALID_MODE`**: Job specifies an unrecognized mode outside of `EXPLORED`, `UNEXPLORED`, `INSPECTION`, or `PATROL`.
3. **`BATTERY_TOO_LOW`**: Rover battery State of Charge is below minimum required mission threshold ($< 25\%$).
4. **`LOCALIZATION_UNAVAILABLE`**: Odometry fusion is in `SENSOR_TIMEOUT` or failed state; start pose cannot be anchored.
5. **`SAFETY_LOCK`**: Hardware or software E-stop is actively latched, or rover is actively in `RECOVERY`.
6. **`COMMUNICATION_UNSTABLE`**: Link packet loss $\ge 70\%$ or link disconnected prior to mission dispatch.

When a job is accepted, an acknowledgement (`JobAssignment` with `accepted=True` and assigned `session_id`) is published to `/rover/job_ack`. If rejected, `accepted=False` and the specific machine-readable `rejection_reason` string are dispatched.

---

## 4. Multi-Factor Frontier Exploration & Scoring

In `UNEXPLORED` mode, the `FrontierExplorationNode` evaluates unknown boundaries extracted from the active Occupancy Grid.

### 4.1 Scoring Function
Frontier candidate clusters are scored according to:

$$\text{Score} = w_{\text{gain}} \cdot N_{\text{cells}} - w_{\text{dist}} \cdot D(p_{\text{rover}}, p_{\text{cand}}) - w_{\text{obs}} \cdot \max(0, D_{\text{safe}} - D_{\text{obs}}) + H_{\text{current}}$$

Where:
- $N_{\text{cells}}$: Information gain count (number of unmapped/unknown cells bordering the frontier cluster).
- $D(p_{\text{rover}}, p_{\text{cand}})$: Euclidean distance from current rover pose to frontier candidate centroid.
- $D_{\text{obs}}$: Nearest obstacle distance from LiDAR raycasting. Penalized heavily if $D_{\text{obs}} < D_{\text{safe}}$ ($0.6\text{ m}$).
- $H_{\text{current}}$: Hysteresis bonus ($+15.0\text{ pts}$) awarded to the actively pursued frontier candidate to prevent erratic goal thrashing / oscillating behavior.

---

## 5. Fail-Safe Return-to-Start Architecture

Autonomous return ensures the rover can safely navigate back to the deployment ingress point in underground mines.

### 5.1 True Odometric Pose Locking
- When the mission transitions into `READY` / starts navigation, the rover anchors its current odometric coordinate $(x_{\text{start}}, y_{\text{start}}, \theta_{\text{start}})$ from `/odom`.
- If return is triggered but no valid start pose was locked, the rover **refuses to drive blindly** and transitions to `HOLD_POSITION` with phase `RETURN_TARGET_UNAVAILABLE`.

### 5.2 Return Triggers & Priority Escalation
Return can be triggered by:
- **`OPERATOR`**: Manual recall command from base station.
- **`COMM_LOSS`**: Sustained link failure exceeding timeout.
- **`BATTERY_RETURN`**: Battery SoC dropping below $20\%$.
- **`CRITICAL_BATTERY`**: Battery SoC dropping below $15\%$ (highest priority preemption).
- **`HAZARD`**: Critical gas threshold breach (e.g. Methane $> 1.25\%$, CO $> 50\text{ ppm}$) or structural instability.

### 5.3 Stalled Nav2 Retry Escalation
- The Mission Manager monitors progress toward the return target.
- If Nav2 stalls or reports navigation failure, the manager retries dispatch up to **3 attempts**.
- Upon exceeding 3 failed attempts, the manager escalates to `HOLD_POSITION` with reason `NAV2_STUCK_EXCEEDED_RETRIES`, zeroing drive commands to prevent battery exhaustion or catastrophic wall collisions.

### 5.4 Recovery & Safety Interlocks
- If rollover or tip-over occurs during return, the active linear actuator recovery sequence claims exclusive control. Autonomous drive commands are inhibited until the rover is re-oriented and stabilized.

---

## 6. Dashboard Click-to-Point Integration

The unified web dashboard (`drillpulse_dashboard`) provides real-time situational awareness and operator dispatch capabilities:

### 6.1 Canvas Click-to-Point Coordinate Conversion
Operator clicks on the 2D mine map canvas are translated into global navigation coordinates $(wx, wy)$ via the affine transform:
```javascript
const mapX = (clickX - offsetX) / zoomLevel;
const mapY = (clickY - offsetY) / zoomLevel;
const wx = originX + (mapX * resolution);
const wy = originY + ((canvasHeight - mapY) * resolution);
```
Target coordinates are presented to the operator in a confirmation modal displaying estimated Euclidean distance, battery margin, and mode selection prior to dispatch.

### 6.2 Command Lifecycle Tracking
Commands transition through explicit lifecycle stages visualized via a topbar HUD indicator:
$$\text{REQUESTED} \longrightarrow \text{ACCEPTED} \longrightarrow \text{EXECUTING} \longrightarrow \begin{cases} \text{COMPLETED} \\ \text{FAILED} \end{cases}$$

### 6.3 Layered Canvas Visualization
The canvas renderer displays 5 distinct, high-contrast operational markers:
1. **Current Rover Pose (Cyan `#06b6d4`):** Oriented arrow indicating rover heading and footprint.
2. **Start / Base Marker (Green `#22c55e`):** Ingress base coordinate with return radius indicator.
3. **Selected Target (Red `#ef4444`):** Active Nav2 goal coordinate with dashed path vector.
4. **Hazard Event Markers (Yellow `#eab308`):** Warning triangles positioned at exact spatial detection points with gas/severity tags.
5. **Return Target (Blue `#3b82f6`):** Dedicated return navigation waypoint.
