# DRILLPULSE — ROBUST COMMUNICATION & ADAPTIVE TELEMETRY SPECIFICATION
**Underground Mine Dual-Channel Transport, Priority-Aware Queuing, Idempotent Commands & Dashboard Integration**
**Platform:** Ubuntu 24.04 LTS, ROS 2 Jazzy, Python 3.12
**Packages:** `drillpulse_transport`, `drillpulse_camera`, `drillpulse_dashboard`
**Project:** `/home/loki/SIH_FINAL_PROTOTYPE`

---

## 1. Executive Subsystem Architecture

In underground coal and metal mines, wireless propagation experiences severe RF attenuation, multi-path fading, and non-line-of-sight (NLOS) signal collapse. The DrillPulse communication architecture decouples low-bandwidth high-reliability command/safety traffic from high-bandwidth bulk data streams across two dedicated physical transport layers.

```
+-----------------------------------------------------------------------------------------+
|                                    BASE STATION DASHBOARD                               |
|   - Real-Time Comm Health HUD (LoRa, HaLow, Overall)   - Return to Base & E-Stop        |
|   - Click-to-Point Waypoint Dispatch                   - Structured Alert Journal       |
|   - Zero Fake Data Policy (Explicit None / "--")       - Dual-Panel Mode Separation     |
+--------------------------------------------+--------------------------------------------+
                                             |
                      +----------------------+----------------------+
                      |                                             |
                      v Channel A                                   v Channel B
        +----------------------------+                +----------------------------+
        |   LORA MESH TRANSCEIVER    |                |    WI-FI HALOW MESH        |
        |  (Semtech SX1262 / Serial) |                |   (Sub-1GHz 802.11ah)      |
        | - Range: 2-5 km NLOS       |                | - Range: 200-800 m         |
        | - Bandwidth: 1-15 kbps     |                | - Bandwidth: 1-8 Mbps      |
        | - Priority 0, 1, 2 traffic |                | - Priority 3 bulk & video  |
        +--------------+-------------+                +--------------+-------------+
                       |                                             |
                       +----------------------+----------------------+
                                              |
                                              v
+-----------------------------------------------------------------------------------------+
|                                   ROVER COMPUTER (Raspberry Pi)                         |
|   +---------------------------------------------------------------------------------+   |
|   |                      LORA TRANSCEIVER NODE (drillpulse_transport)               |   |
|   |  - 4-Tier Priority Telemetry Engine (P0: Emerg, P1: Safety, P2: Norm, P3: Bulk) |   |
|   |  - Priority-Aware Bounded Queue (O(1) bucketed deque, max 500 items)           |   |
|   |  - Connection Quality State Machine (GOOD, DEGRADED, WEAK, INTERMITTENT, LOST)  |   |
|   |  - Idempotent Command Tracker (Sliding window cache, duplicate rejection)       |   |
|   |  - Compact Heartbeat Protocol (<100 bytes at 1.0 Hz) & Liveness Watchdog        |   |
|   |  - Local SD-Card Rotated Session Logging (.jsonl)                              |   |
|   +-------------------+-----------------------------+-------------------------------+   |
|                       |                             |                                   |
|                       v                             v                                   |
|        +------------------------------+     +-------------------------------+           |
|        |     MISSION & SAFETY NODES   |     |    ADAPTIVE VIDEO STREAMER    |           |
|        |   - MissionManagerNode       |     |  - High HaLow: 20 FPS / Q75   |           |
|        |   - SafetyManagerNode        |     |  - Weak HaLow:  5 FPS / Q30   |           |
|        |   - Navigation / Nav2        |     |  - Lost HaLow: 0 FPS (Paused) |           |
|        +------------------------------+     +-------------------------------+           |
+-----------------------------------------------------------------------------------------+
```

---

## 2. Channel Separation

### Channel A: Low-Bandwidth / High-Reliability Channel (LoRa Mesh)
Operates over UHF/Sub-1GHz ISM band (868/915 MHz) via `LoRaTransceiverNode`.
Strictly reserved for:
- Rover compact heartbeat (under 100 bytes).
- Canonical mission states and active operating modes.
- Battery percentage and estimated remaining physical runtime.
- Connection quality metrics (RSSI, SNR, packet loss rate).
- Critical sensor hazard triggers (Methane $> 1.25\%$, CO $> 50\text{ ppm}$, Oxygen $< 19.5\%$).
- Rollover / tip-over fall detection flags.
- Dual linear recovery actuator status and manual recovery commands.
- Emergency Stop (E-Stop) commands.
- Return-to-Base / Return-to-Start commands.
- Command acknowledgements (`CMD_ACK`) and job assignment handshakes.
- Compact rover navigation pose $(x, y, \theta)$.

### Channel B: High-Bandwidth Channel (Wi-Fi HaLow Mesh)
Operates over 802.11ah Sub-1GHz Wi-Fi HaLow interface.
Reserved for:
- 640x480 RGB Optical compressed video stream.
- FLIR thermal heat anomaly infrared camera stream.
- Dense OccupancyGrid SLAM map synchronization.
- Detailed LiDAR point clouds (`/scan`).
- Comprehensive mission audit reports (`.html` and `.json`).

> [!IMPORTANT]
> Video streaming over LoRa is strictly prohibited in software. When Wi-Fi HaLow is degraded or lost, the `VideoStreamerNode` automatically throttles or suspends video publishing to eliminate socket contention, ensuring 100% of onboard CPU and low-bandwidth capacity is dedicated to critical safety telemetry.

---

## 3. 4-Tier Adaptive Telemetry Priority System

Telemetry packets are classified into four discrete priority tiers:

| Tier | Priority | Category | Payload Contents | Bandwidth Handling & Eviction Rules |
|---|---|---|---|---|
| **P0** | `0` | **EMERGENCY / CRITICAL** | Rollover detected, E-stop active, actuator fault, critical gas threshold breach, critical battery ($< 15\%$), communication loss state, trapped path, return-to-base request, nav failure, sensor disagreement. | **Highest Priority.** Transmitted immediately. Strictly preserved in local priority queue; **NEVER** dropped or evicted by lower-priority packets. |
| **P1** | `1` | **SAFETY / MISSION** | Battery %, runtime estimation, rover mode, mission state, localization state, current position $(x, y)$, nav status, link quality, sensor health, recovery state. | Transmitted at rate allowed by link state (1.0 to 10.0 Hz). Preserved over normal telemetry. Evicted only if queue saturates with P0/P1. |
| **P2** | `2` | **NORMAL TELEMETRY** | Temperature, humidity, ambient gas ppm, IMU attitude (pitch/roll/yaw), encoder RPM, LiDAR status, linear velocity, odometry diagnostics. | Transmitted only during `GOOD` and `DEGRADED` link states. Throttled or dropped during `WEAK` or `INTERMITTENT` states. |
| **P3** | `3` | **BULK / HIGH BANDWIDTH** | Optical video frames, thermal frames, dense SLAM occupancy maps, historical SD logs, HTML reports. | Routed exclusively through Wi-Fi HaLow. Discarded first upon queue saturation. Suspended during link loss. |

---

## 4. Connection Quality State Machine

The link state is determined by `ConnectionQualityEvaluator` evaluating multiple independent metrics:

```
                      +-------------------+
                      |  CONNECTED_GOOD   | <-------+
                      +---------+---------+         |
                                |                   |
                      +---------v---------+         |
                      | CONNECTED_DEGRADED|         |
                      +---------+---------+         |
                                |                   |
                      +---------v---------+         |
                      |  CONNECTED_WEAK   |         |
                      +---------+---------+         |
                                |                   |
                      +---------v---------+         |
       +------------> |   INTERMITTENT    |         |
       |              +---------+---------+         |
       |                        |                   |
       |              +---------v---------+         |
       |              |   DISCONNECTED    |         |
       |              +---------+---------+         |
       |                        |                   |
       |              +---------v---------+         |
       +------------> |    RECOVERING     +---------+
                      +-------------------+
```

### 4.1 Transition Criteria
- **`CONNECTED_GOOD`**: $\text{RSSI} \ge -75\text{ dBm}$, $\text{Loss} < 15\%$, $\text{Latency} \le 120\text{ ms}$. Target Rate: **$10.0\text{ Hz}$**.
- **`CONNECTED_DEGRADED`**: $\text{RSSI} \ge -90\text{ dBm}$, $\text{Loss} < 40\%$, $\text{Latency} \le 350\text{ ms}$. Target Rate: **$3.0\text{ Hz}$**.
- **`CONNECTED_WEAK`**: $\text{RSSI} \ge -105\text{ dBm}$, $\text{Loss} < 70\%$, $\text{Latency} \le 800\text{ ms}$. Target Rate: **$1.0\text{ Hz}$**.
- **`INTERMITTENT`**: $\text{Loss} \ge 70\%$, rapid timeout oscillations, or command retries $\ge 3$. Target Rate: **$0.5\text{ Hz}$**.
- **`DISCONNECTED`**: Heartbeat silence $> 4.0\text{s}$ or physical link failure. Target Rate: **$0.2\text{ Hz}$** (probe pings only).
- **`RECOVERING`**: Packets resume after disconnection. Held for $2.0\text{s}$ stability window before transitioning to `GOOD`/`DEGRADED`/`WEAK`. Rate: **$2.0\text{ Hz}$**.

---

## 5. Priority-Aware Bounded Queue & Local Buffering

When communication is degraded or lost, the rover buffers critical data locally using `PriorityTelemetryQueue`:
- **Bounded Storage:** Configured capacity (default $500$ items) prevents unbounded RAM growth on the Raspberry Pi.
- **$O(1)$ Operations:** Four internal `deque` buckets map directly to priority levels $0, 1, 2, 3$.
- **Eviction Policy:** When full, the queue evicts oldest Priority 3 (Bulk) items first, then Priority 2 (Normal), then Priority 1 (Safety).
- **Critical Preservation:** Priority 0 items are **strictly preserved** and will never be discarded by buffer overflow while non-critical items exist.
- **Burst Synchronization:** Upon reconnecting, the queue is drained in non-blocking batches (`drain_batch(max_items=5)`) during regular transmission intervals, preventing executor thread starvation.

---

## 6. Command Acknowledgement & Idempotency Engine

All safety-critical operator commands require two-way acknowledgement managed by `AcknowledgementManager`:

### 6.1 Lifecycle States
$$\text{REQUESTED} \longrightarrow \text{RECEIVED} \longrightarrow \text{ACCEPTED} \longrightarrow \text{EXECUTING} \longrightarrow \begin{cases} \text{COMPLETED} \\ \text{FAILED} \end{cases}$$

### 6.2 Idempotency Protection
To guard against wireless retransmissions causing duplicate physical actions (e.g. re-triggering actuator swivel or re-setting navigation waypoints):
- The rover maintains an in-memory sliding cache of executed `command_id` values ($60\text{s}$ retention window).
- If a duplicate `command_id` arrives, the rover suppresses physical re-execution, logs the duplicate, and re-transmits an immediate ACK with:
  ```json
  {"id": "CMD_01", "status": "ACCEPTED", "error_code": "IDEMPOTENT_DUPLICATE"}
  ```

---

## 7. Compact Heartbeat Protocol

Heartbeats communicate liveness over LoRa at $1.0\text{ Hz}$. Payloads are constrained strictly under $100\text{ bytes}$:
```json
{
  "rid": "ROVER_01",
  "sid": "DRILLPULSE-20260930-103000-8A12",
  "mod": "EXPLORED",
  "st": "NAVIGATING",
  "bat": 84.5,
  "rt": 126.8,
  "lnk": "GOOD",
  "flt": 0,
  "x": 8.42,
  "y": -3.15,
  "seq": 142
}
```

---

## 8. Dashboard Zero Fake Data Policy

The operator dashboard (`drillpulse_dashboard`) adheres to a strict zero-fake-data contract:
- If a sensor or link metric is unavailable or the rover is disconnected, the payload field is explicitly set to `None` (`null`).
- The frontend renderer explicitly displays `"--"`, `"NO DATA"`, or `"UNAVAILABLE"`.
- It **never** displays default numbers like `0`, `100%`, or `-65 dBm` when communication is offline.
