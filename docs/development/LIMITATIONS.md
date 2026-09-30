# DrillPulse Engineering Status & System Limitations

## Overview

In accordance with strict SIH engineering standards, this document provides an honest assessment of the DrillPulse codebase status, delineating what has been **Implemented**, **Software-Tested**, **Simulated**, and what remains **Hardware-Unverified** or **Field-Unverified**.

---

## 1. System Status Taxonomy & Verification Matrix

| Subsystem / Feature | Software Code Status | Verification Level | Operational Status & Notes |
|:---|:---|:---|:---|
| **ROS 2 Jazzy Core Packages** | Complete (`rover_ws`, `station_ws`) | `SOFTWARE-TESTED` | Fully compiled and verified on Ubuntu 24.04 + ROS 2 Jazzy. All 17 automated tests passing. |
| **Arduino Motor Firmware** | Complete (`firmware/`) | `SOFTWARE-TESTED` / `HARDWARE-UNVERIFIED` | 500ms watchdog, XOR checksum, and INT0 E-stop verified in C++ test harness. Requires physical Arduino Mega on rover chassis for motor current verification. |
| **Arduino Actuator Firmware** | Complete (`firmware/`) | `SOFTWARE-TESTED` / `HARDWARE-UNVERIFIED` | 3.5s stroke limit, 2.0s thermal cooldown, and relay mutual exclusion verified in C++ test harness. |
| **STM32 6-Wheel Encoders** | Complete (`firmware/`) | `SOFTWARE-TESTED` / `HARDWARE-UNVERIFIED` | 1040 PPR tick parsing, binary framing, and glitch filtering verified in C11 harness. ST-LINK VCP serial tested in loopback. |
| **Odometry & EKF Fusion** | Complete (`drillpulse_odometry`) | `SOFTWARE-TESTED` | Slip detection and covariance scaling tested in simulation. EKF node verified with `diagnostic_updater_overlay` ABI compatibility. |
| **Nav2 Explored Navigation** | Complete (`drillpulse_navigation`) | `SIMULATED` / `SOFTWARE-TESTED` | AMCL and Navfn/DWB planners verified against synthetic mine costmap (`sample_mine_map.yaml`). |
| **SLAM Frontier Exploration** | Complete (`drillpulse_navigation`) | `SIMULATED` / `SOFTWARE-TESTED` | Asynchronous SLAM Toolbox and BFS frontier clustering tested with simulated LiDAR scans. |
| **Dual-Channel Transport** | Complete (`drillpulse_transport`) | `SOFTWARE-TESTED` | TCP sockets and LoRa UART framing tested over local loopback with artificial packet loss injection. |
| **4-Tier Priority Queuing** | Complete (`drillpulse_transport`) | `SOFTWARE-TESTED` | Priority ring buffers tested under bandwidth-throttled conditions with automatic tier shedding. |
| **Web Dashboard & Canvas** | Complete (`drillpulse_dashboard`) | `SOFTWARE-TESTED` | Flask + Socket.IO dual-panel UI, click-to-point canvas coordinate transforms, and zero-fake-data policy tested. |
| **Mission Logging & Reports** | Complete (`drillpulse_report`) | `SOFTWARE-TESTED` | JSONL rotation and markdown report generation tested with multi-event mission logs. |
| **Autonomous Rollover Recovery** | Complete (`drillpulse_recovery`) | `SIMULATED` / `HARDWARE-UNVERIFIED` | Angle-triggered righting state machine verified. Physical chassis ground reaction forces unverified. |
| **Mine Field Operations** | Architecture Complete | `FIELD-UNVERIFIED` | Deep subterranean deployment in active coal/metal mine galleries with true rock dust, explosive gas mixtures, and severe NLOS RF absorption has not been physically conducted. |

---

## 2. Technical Limitations & Operational Boundaries

1. **Optical Sensing & Darkness**: The rover camera relies on ambient light unless auxiliary LED illumination is powered. In zero-lux mine galleries, visual odometry cannot function without onboard lighting.
2. **Subterranean RF Attenuation**: Sub-GHz LoRa penetrates around tunnel corners via waveguide effect, but range drops past 2-3 unrepeated turns in hard rock mines. High-throughput Wi-Fi HaLow requires tactical drop-off repeater nodes beyond 300m.
3. **Rocker-Bogie Obstacle Clearance**: The physical chassis features a ground clearance of ~120 mm. Rubble piles exceeding this height will high-center the chassis.
4. **Battery Discharge**: Under continuous concurrent drive of 6 motors and dual actuators, current peaks at 18-22A. Battery runtime on a 3S 5000mAh pack is ~35-45 minutes in continuous traverse, up to 90 minutes stationary.
5. **Intrinsic Safety Certification**: The current prototype is an engineering prototype. Zone 0/1 coal mine deployment requires certified explosion-proof (Ex-d) enclosures.
