# DRILLPULSE — ADAPTIVE TELEMETRY SYSTEM
**Step 4: Dynamic Rate Scaling, Multi-State Link Evaluation & Priority Content Filtering**
**Date:** 2026-09-29
**Platform:** ROS 2 Jazzy, Python 3.12

---

## 1. Adaptive Telemetry System Overview

In underground mine workings, radio frequency propagation degrades rapidly as the rover advances through rock bends, collapsed stopes, or flooded shafts. DrillPulse employs an **Adaptive Telemetry Governor** running within `lora_transceiver_node.py` that continuously monitors link quality and dynamically throttles transmission frequency and message payloads.

---

## 2. Five Connection States & Throttle Matrix

The link quality is assessed continuously using a sliding window of received signal indicators (RSSI, SNR, and packet acknowledgement success rates):

| Link State | RSSI Threshold (dBm) | SNR Threshold (dB) | Packet Loss Rate | Transmit Rate (Hz) | Transmission Content |
|---|---|---|---|---|---|
| **GOOD** | $\ge -70$ | $\ge 6.0$ | $< 5\%$ | **10.0 Hz** | Full telemetry: state, battery, pose, all gases ($\text{CH}_4$, $\text{CO}$, $\text{CO}_2$), dust, humidity, ambient temp, diagnostics |
| **DEGRADED** | $[-85, -70)$ | $[2.0, 6.0)$ | $[5\%, 15\%)$ | **3.0 Hz** | Full telemetry at reduced rate |
| **WEAK** | $[-100, -85)$ | $[-5.0, 2.0)$ | $[15\%, 35\%)$ | **1.0 Hz** | Filtered telemetry: strips non-critical ambient data (dust, humidity, raw gases) |
| **CRITICAL** | $< -100$ | $< -5.0$ | $\ge 35\%$ | **0.5 Hz** | Minimal safety telemetry: mode, battery, E-Stop status, rollover status only |
| **LOST** | Silence $> 5.0\text{s}$ | N/A | $100\%$ | **0.2 Hz (Probe)** | Heartbeat probe beacon only to re-establish handshake |

---

## 3. Priority Content Filtering (Payload Management)

To guarantee that safety-critical packets fit within the 220-byte LoRa MTU and survive low link budgets:

1. **GOOD / DEGRADED Mode Payload:**
   ```json
   {
     "mode": "EXPLORED",
     "mission": "NAVIGATING",
     "bat_v": 12.1,
     "bat_soc": 84,
     "bat_rem_m": 72,
     "link_state": "GOOD",
     "rssi": -65,
     "snr": 8.5,
     "pose": [2.45, 1.12, 0.78],
     "hazard": "NONE",
     "gases": {"co": 4.2, "ch4": 0.05, "co2": 450},
     "env": {"temp": 24.1, "hum": 68.2, "dust": 12.0},
     "rec_status": "IDLE"
   }
   ```

2. **WEAK Mode Payload (Filtered):**
   - Non-vital sensor values (`env.dust`, `env.hum`, raw gas levels) are stripped.
   - Only actionable hazards are retained:
   ```json
   {
     "mode": "EXPLORED",
     "mission": "NAVIGATING",
     "bat_soc": 84,
     "bat_rem_m": 72,
     "link_state": "WEAK",
     "rssi": -92,
     "hazard": "NONE",
     "rec_status": "IDLE"
   }
   ```

3. **CRITICAL Mode Payload (Barebones Survival):**
   - Strips all non-essential fields down to under 60 bytes:
   ```json
   {
     "mode": "EMERGENCY_STOP",
     "bat_soc": 14,
     "bat_rem_m": 8,
     "link_state": "CRITICAL",
     "hazard": "ROLLOVER_WARN"
   }
   ```

---

## 4. Remaining Battery Runtime Estimation

The onboard node continuously estimates remaining operational time based on State of Charge ($\text{SoC}$) and active current draw:

$$\text{Remaining Minutes} = \frac{\text{SoC} \cdot C_{\text{battery}}}{\max(0.5, I_{\text{current}})} \times 60$$

- $C_{\text{battery}}$ is the rated battery pack capacity (e.g. $10.0\text{ Ah}$).
- $I_{\text{current}}$ is the filtered load current in Amperes, clamped to a minimum idle consumption of $0.5\text{ A}$ to prevent division by zero or unrealistic infinite runtimes.
- If current sensor telemetry is unavailable, a baseline model based on active drivetrain actuation mode is used ($1.2\text{ A}$ idle, $3.5\text{ A}$ moving, $6.0\text{ A}$ recovery/heavy torque).

---

## 5. Configurable ROS Parameters

The following ROS parameters are exposed by `rover_lora_transceiver`:

| Parameter | Type | Default | Description |
|---|---|---|---|
| `rssi_good_threshold` | `double` | `-70.0` | RSSI threshold for GOOD state (dBm) |
| `rssi_degraded_threshold` | `double` | `-85.0` | RSSI threshold for DEGRADED state (dBm) |
| `rssi_weak_threshold` | `double` | `-100.0` | RSSI threshold for WEAK state (dBm) |
| `rate_good_hz` | `double` | `10.0` | Transmission rate under GOOD conditions |
| `rate_degraded_hz` | `double` | `3.0` | Transmission rate under DEGRADED conditions |
| `rate_weak_hz` | `double` | `1.0` | Transmission rate under WEAK conditions |
| `rate_critical_hz` | `double` | `0.5` | Transmission rate under CRITICAL conditions |
| `rate_probe_hz` | `double` | `0.2` | Beacon rate when link is LOST |
| `battery_capacity_ah` | `double` | `10.0` | Total nominal battery capacity |
