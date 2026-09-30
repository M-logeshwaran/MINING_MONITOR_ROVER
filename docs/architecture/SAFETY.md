# DrillPulse Safety Architecture & Hazard Management

## Overview

Underground coal and metal mines present hazardous atmospheres consisting of explosive firedamp (CH4), toxic afterdamp (CO), oxygen deficiency, and high risk of roof fall or steep incline roll-overs. DrillPulse employs an autonomous multi-layer safety supervisor (`safety_manager_node.py`) running onboard the Raspberry Pi that operates independently of operator communication.

---

## 1. 4-State Safety State Machine

The safety system evaluates sensor readings at 20 Hz, transitioning deterministically across four hierarchical states:
- **NOMINAL**: All parameters within standard mining safety limits.
- **WARNING**: Speed derated 50%; Audio-visual warning on Dashboard.
- **CRITICAL**: Autonomous Mission Abort; Auto-Return Triggered.
- **EMERGENCY_STOP**: Full motor cut-off; Actuator locks; Emergency Beacon.

---

## 2. Hazard Threshold Specifications

Safety thresholds comply with Directorate General of Mines Safety (DGMS) India and OSHA underground mining standards:

| Parameter | Unit | Nominal Range | Warning Level | Critical Action | Emergency Stop |
|:---|:---|:---|:---|:---|:---|
| **Methane (CH4)** | ppm | 0 - 5,000 (< 0.5%) | 10,000 (1.0%) | 15,000 (1.5%) | >= 20,000 (2.0%) |
| **Carbon Monoxide (CO)** | ppm | 0 - 25 | 50 | 100 | >= 200 |
| **Ambient Temperature** | deg C | 15 - 35 | 45.0 | 55.0 | >= 65.0 |
| **Relative Humidity** | % | 40 - 85 | 90.0 | 95.0 | > 99.0 |
| **Pitch Incline** | deg | -15 to +15 | +-25.0 | +-35.0 | +-45.0 |
| **Roll Incline** | deg | -15 to +15 | +-30.0 | +-40.0 | |roll| >= 45.0 (Rollover) |
| **LiDAR Proximity** | m | > 1.0 | < 0.60 | < 0.35 | < 0.20 |
| **Battery Voltage** | V | 11.8 - 12.6 | 11.1 (20%) | 10.6 (15%) | < 9.9 (LVC) |

---

## 3. Motion Interlocks & Hardware E-Stop Integration

When the safety manager enters `CRITICAL` or `EMERGENCY_STOP`:
1. **Software Velocity Clamp**: The mobility controller immediately zeroes all linear and angular velocity commands dispatched to `/cmd_vel_out`.
2. **Serial Halt Packet**: The serial bridge sends the immediate zero command `M,0,0,0\n` to the motor Arduino.
3. **Hardware Watchdog Deprivation**: High-level heartbeat packets to the Arduino motor controller are withheld, forcing the 500 ms firmware watchdog to de-energize the L298N H-bridges.
4. **Physical E-Stop Line**: Pin D2 on the Arduino triggers external interrupt INT0 if an onboard safety switch or wireless emergency receiver asserts the hardware stop bus.
