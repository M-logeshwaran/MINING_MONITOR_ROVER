# DRILLPULSE — SENSOR HEALTH & VALIDATION ARCHITECTURE
**Step 5: Multi-Stream Validation, Sanity Gating & Cross-Consistency**
**Date:** 2026-09-29
**Platform:** Ubuntu 24.04 LTS, ROS 2 Jazzy, Python 3.12

---

## 1. Multi-Stream Sensor Health Registry

The rover does not assume a sensor is healthy simply because its ROS topic exists. Eight critical sensor streams are tracked independently:

| Sensor Stream | Source Topic | Expected Rate | Freshness Timeout | Physical Range / Sanity Envelope |
|---|---|---|---|---|
| **IMU** | `/sparkfun/imu/data`, `/imu/data` | 50.0 Hz | 0.80 s | Accel norm $\in [0.5, 35.0]\text{ m/s}^2$; Angular vel $\in [-15, 15]\text{ rad/s}$ |
| **Encoders** | `/encoder`, `/odom_encoder` | 50.0 Hz | 0.80 s | Wheel RPM $\in [-500, 500]$; Linear vel $\in [-5.0, 5.0]\text{ m/s}$ |
| **LiDAR** | `/scan` | 10.0 Hz | 2.00 s | Valid range count $\ge 5$; Range values $\in [0.05, 25.0]\text{ m}$ |
| **Gas Sensor** | `/drillpulse/gas`, `/environment/state` | 1.0 - 20 Hz | 3.00 s | $\text{CH}_4 \in [0.0, 15000.0]\text{ ppm}$; NaN/Inf rejected |
| **Temperature** | `/drillpulse/temperature` | 1.0 - 20 Hz | 3.00 s | Ambient temp $\in [-30.0, 95.0]^\circ\text{C}$ |
| **Humidity** | `/drillpulse/humidity` | 1.0 - 20 Hz | 3.00 s | Relative humidity $\in [0.0, 100.0]\%$ |
| **Battery** | `/rover/battery` | 1.0 - 10 Hz | 3.00 s | Voltage $\in [5.0, 20.0]\text{V}$; SoC $\in [0.0, 100.0]\%$ |
| **Connection** | `/rover/link_status`, `/drillpulse/link_status` | 0.2 - 10 Hz | 5.00 s | Protocol V2 heartbeat & packet freshness |

---

## 2. Four Operational Health States per Sensor

1. **`VALID`:** Sensor publishing regularly within timeout; all numerical readings finite and within physical bounds; no cross-disagreements.
2. **`DEGRADED`:** Minor anomalies or cross-sensor disagreement (e.g. wheels slipping while IMU indicates stationary posture). Subsystem confidence reduced.
3. **`STALE`:** No message received within the configured `timeout_sec` window.
4. **`FAILED`:** Sensor stream contains NaN, Inf, impossible physical values (e.g. $120^\circ\text{C}$ in underground mine), or catastrophic frame corruption.

---

## 3. Cross-Sensor Disagreement & Fusion Gating

- **Wheel Slip vs Skid:** Fused in `threshold_consistency_node.py`. When velocity difference $|v_{\text{enc}} - v_{\text{imu}}| > 0.35\text{ m/s}$ or yaw rate difference $> 0.30\text{ rad/s}$, the node outputs `SENSOR_DISAGREEMENT`.
- **Gating Action:** Rather than guessing which sensor is broken, both affected sensor streams are downgraded to `DEGRADED`, and the Safety Monitor transitions the vehicle to `DEGRADED` state to prevent reckless trajectory following.
