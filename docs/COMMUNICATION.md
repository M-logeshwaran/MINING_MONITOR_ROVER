# DrillPulse Dual-Channel Communication & Adaptive Telemetry

## Overview

Underground mining environments present severe RF attenuation, multipath reflection, and non-line-of-sight (NLOS) conditions through solid rock and coal pillars. DrillPulse addresses this by deploying an asymmetrical **Dual-Channel Hybrid Communication Architecture**:

1. **Primary Low-Bandwidth / High-Reliability Channel (LoRa Mesh)**: Ultra-narrowband sub-GHz RF (868/915 MHz) ensuring continuous delivery of mission-critical safety telemetry, heartbeat, and basic command frames over several kilometers through tunnels.
2. **Secondary High-Bandwidth Channel (Wi-Fi HaLow 802.11ah / 2.4 GHz Mesh)**: Sub-1 GHz Wi-Fi HaLow providing 150 kbps - 2 Mbps throughput for SLAM occupancy grid streaming, LiDAR point clouds, and compressed video feeds when line-of-sight or repeater nodes permit.

---

## 1. 4-Tier Priority Message Queue

The `adaptive_telemetry_node` maintains separate outbound ring buffers segregated by message priority to guarantee zero-loss transmission of emergency telemetry even when total network bandwidth collapses:

| Tier | Priority Level | Content / Topics | Transmit Channel | Max Latency Budget | Fallback Action Under Degradation |
|:---|:---|:---|:---|:---|:---|
| **Tier 0** | **CRITICAL** | E-Stop, Gas Alarm, Rollover, Actuator Status, Watchdog Heartbeat | LoRa + Wi-Fi (Dual-cast) | < 100 ms | Instant retransmit; preempts all other packets |
| **Tier 1** | **HIGH** | Mission State, Waypoint Ack, Battery State, Mode Changes | LoRa + Wi-Fi | < 500 ms | Throttled from 2 Hz to 1 Hz |
| **Tier 2** | **MEDIUM** | Filtered Odometry, IMU Euler Angles, Temperature/Humidity | LoRa (downsampled) / Wi-Fi | < 1000 ms | Downsampled to 0.2 Hz on LoRa; preserved on Wi-Fi |
| **Tier 3** | **LOW / BULK** | OccupancyGrid map slices, Nav2 Costmaps, Camera JPEG frames | Wi-Fi Only | Best Effort | **Completely suppressed** if Wi-Fi quality < 50% or state drops below `GOOD` |

---

## 2. 6-State Connection Quality State Machine

The connection resilience manager continuously samples packet arrival rate, round-trip time (RTT), RSSI, SNR, and packet loss ratio across both interfaces, transitioning through 6 distinct link states:

- **EXCELLENT**: LoRa RSSI > -75dBm, Wi-Fi RSSI > -65dBm, Loss < 1%
- **GOOD**: LoRa RSSI > -90dBm, Wi-Fi RSSI > -80dBm, Loss < 5%
- **DEGRADED**: Wi-Fi dropping frames (>15% loss) / RSSI < -85dBm. Action: Suppress video feed, compress map updates.
- **LORA_ONLY**: Wi-Fi link completely lost; LoRa intact (<10% loss). Action: Zero video/map streaming; 100B telemetry only.
- **INTERMITTENT**: LoRa packet loss > 30%; missed 2 consecutive beats. Action: Arm onboard return timer (15s deadline).
- **DISCONNECTED**: No packets received on any channel for > 15.0 seconds. Action: Abort mission; trigger Autonomous Return.

---

## 3. Ultra-Compact LoRa Packet Protocol (< 100 Bytes)

LoRa transmission over SX1262 / SX1276 modules at Spreading Factor 9 (SF9) or SF10 is bandwidth-constrained. DrillPulse enforces a strict binary telemetry payload format guaranteeing packet sizes **under 70 bytes**:

```
[0xAA 0x55] [Seq:2] [Timestamp:4] [Mode:1] [MissionState:1] [BattPct:1] 
[Voltage:2] [Current:2] [PoseX:4] [PoseY:4] [Yaw:2] [GasPPM:2] 
[Temp:2] [Roll:2] [Pitch:2] [ActuatorState:1] [CRC16:2] [0x0D 0x0A]
```

Total wire size is **38 bytes**, well within the 250-byte maximum payload of LoRa modulation schemes, providing headroom for forward error correction (FEC 4/5) and transmission times under 95 ms.
