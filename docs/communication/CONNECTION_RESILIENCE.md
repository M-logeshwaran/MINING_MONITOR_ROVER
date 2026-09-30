# DRILLPULSE — CONNECTION-AWARE & RESILIENCE ARCHITECTURE
**Step 6 Technical Specification**
**Date:** 2026-09-30
**Platform:** Ubuntu 24.04 LTS, ROS 2 Jazzy, Python 3.12
**Target Systems:** DrillPulse Underground Mine Safety & Rescue Rover (`rover_ws`) & Control Station (`station_ws`)

---

## 1. Executive Summary

Underground mine tunnels present extreme RF attenuation, non-line-of-sight multipath reflections, rock mass shielding, and dust interference. A rover operating in this environment cannot assume constant or reliable high-bandwidth communication.

The DrillPulse Connection-Aware Resilience Layer enables the rover to:
1. Continuously measure link quality on both LoRa Mesh and Wi-Fi / Wi-Fi HaLow transports.
2. Adapt transmission rates dynamically across five discrete operating states to prevent radio buffer bloat and queue collapse.
3. Prioritize safety-critical telemetry, hazard alarms, and emergency commands over non-essential metrics.
4. Execute autonomous return-to-base and safe holding procedures when communication is lost, without operator intervention.
5. Provide the base station and operator dashboard with clear, unvarnished insight into link status, packet loss, RSSI, and transceiver health.

---

## 2. Dual-Transport Network Topology

The rover communication architecture separates low-bandwidth resilient telemetry from high-bandwidth sensor payloads:

```
+-------------------------------------------------------------------------------+
|                             DRILLPULSE ROVER                                  |
|                                                                               |
|   +-----------------------+                    +--------------------------+   |
|   |   Critical Telemetry  |                    |  RGB Optical / FLIR Feed |   |
|   |  & Emergency Commands |                    |   Occupancy Grid Stream  |   |
|   +-----------+-----------+                    +------------+-------------+   |
|               |                                             |                 |
|               v                                             v                 |
|   +-----------------------+                    +--------------------------+   |
|   |   SX1262 LoRa Mesh    |                    |  802.11ah Wi-Fi HaLow    |   |
|   |  (868 / 915 MHz SPI)  |                    |    (Sub-1 GHz OFDM)      |   |
|   +-----------+-----------+                    +------------+-------------+   |
+---------------|---------------------------------------------|-----------------+
                | (Low Bandwidth, High Penetration)           | (Decoupled Video)
                v                                             v
+---------------|---------------------------------------------|-----------------+
|   +-----------+-----------+                    +------------+-------------+   |
|   |  LoRa Base Station    |                    | HaLow Access Point (AP)  |   |
|   +-----------+-----------+                    +------------+-------------+   |
|               |                                             |                 |
|               +----------------------+----------------------+                 |
|                                      v                                        |
|                     DRILLPULSE WORKSTATION / DASHBOARD                        |
+-------------------------------------------------------------------------------+
```

### Transport Isolation & Decoupling
- **LoRa Mesh (SPI / Serial / UDP Sim):** Transports rover state, hazard events, E-Stop, recovery commands, odometry diagnostics, and core environmental gas levels.
- **Wi-Fi HaLow (OFDM Sub-1 GHz):** Transports video frames (optical and thermal compressed streams) and dense 2D occupancy grids.
- **Isolation Guarantee:** If the Wi-Fi HaLow link drops due to distance or heavy rock obstruction, video streams freeze gracefully (`video_online = False`), but core LoRa telemetry and safety interlocks operate without interruption.

---

## 3. Five-State Normalized Link Quality Spectrum

Both `LoRaTransceiverNode` and `DrillPulseDashboardNode` categorize the radio channel into 5 normalized states based on Signal Strength (RSSI) and Packet Loss Rate:

| State | RSSI Range | Packet Loss Rate | Transmission Rate | Telemetry Payload Content |
|---|---|---|---|---|
| **`GOOD`** | $\ge -75\text{ dBm}$ | $< 15\%$ | **$10.0\text{ Hz}$** | Complete: Pose, velocity, all gases (CH4, CO, O2, dust), attitude, temps, full odometry diagnostics. |
| **`DEGRADED`** | $[-90, -75)\text{ dBm}$ | $[15\%, 40\%)$ | **$3.0\text{ Hz}$** | Throttled: Pose, gas concentrations, attitude, battery, link status. |
| **`WEAK`** | $[-105, -90)\text{ dBm}$ | $[40\%, 70\%)$ | **$1.0\text{ Hz}$** | Essential: Critical gas levels (CH4, CO), battery %, E-stop status, rover operating mode. |
| **`CRITICAL`** | $< -105\text{ dBm}$ | $\ge 70\%$ | **$0.5\text{ Hz}$** | Vital: Emergency flags, battery SoC, rollover status. |
| **`LOST`** | Silent $> 5.0\text{s}$ | $100\%$ | **$0.2\text{ Hz}$ (Probe)** | Discovery Probe: Compact ping packets to re-establish connection. |

### Hysteresis & Debouncing
- To prevent state chatter near boundary thresholds (e.g., oscillating between $-89\text{ dBm}$ and $-91\text{ dBm}$), an exponential moving average (EMA) filter with $\alpha = 0.3$ is applied to measured RSSI:
  $$\text{RSSI}_{\text{filtered}}[k] = 0.3 \cdot \text{RSSI}_{\text{raw}}[k] + 0.7 \cdot \text{RSSI}_{\text{filtered}}[k-1]$$
- Upward state transitions require 3 consecutive samples exceeding the higher threshold.

---

## 4. Heartbeat Watchdogs & Comms-Loss Failsafe

### Heartbeat Watchdog Timing
- **Rover Transceiver:** Expects packets or acknowledgments every cycle. If no packet arrives for $t > 5.0\text{s}$ (`link_timeout_lost`), link state transitions to `LOST`.
- **Mission Manager:** Monitors `/rover/link_status` and `/transport/lora/link_status`. If no link update arrives for $t > 5.0\text{s}$ (`comm_timeout_sec`), `consecutive_comm_drops` increments.
- **Station Dashboard:** Marks rover telemetry as stale if packet age exceeds $3.5\text{s}$, replacing live numbers with zero-fake-data indicators (`None` / `--`).

### Safe Return-to-Start on Comms Loss
When `MissionManagerNode` detects communication silence while actively navigating in `EXPLORED` or `UNEXPLORED` mode:
1. Active frontier exploration and manual teleoperation are immediately cancelled.
2. A Nav2 return goal is dispatched to the locked mission start pose $(x_0, y_0, \theta_0)$.
3. Rover transitions to `RETURNING_COMM_LOSS` with phase `RETURNING_TO_START`.
4. If start pose is unavailable, the rover enters `HOLD_POSITION` with phase `RETURN_TARGET_UNAVAILABLE` to prevent blind driving.

---

## 5. Message Contracts

### Canonical `LinkStatus.msg` Definition
```
std_msgs/Header header
bool connected
string link_type
float32 signal_strength_rssi
float32 snr_db
float32 packet_loss_rate
float32 latency_ms
int32 packets_sent
int32 packets_received
int32 retransmissions
string link_quality_state
string lora_state
string wifi_state
string overall_state
```

Both `rover_ws` and `station_ws` share this exact definition, validated via AST and schema synchronization tools.
