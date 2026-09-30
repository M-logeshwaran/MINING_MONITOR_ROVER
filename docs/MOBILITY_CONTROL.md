# DRILLPULSE — ROVER MOBILITY CONTROL ARCHITECTURE

## 1. Overview
The DrillPulse rover employs a 6-wheel rocker-bogie suspension system driven by dual L298N motor drivers. This document defines the single authoritative mobility pipeline, kinematic equations, priority ladder, and safety cutoffs.

---

## 2. Mobility Control Pipeline

```
+-------------------------------------------------------------------------+
|                           COMMAND SOURCES                               |
|   /rover/estop            (std_msgs/Bool)                               |
|   /rover/actuator_state   (std_msgs/String - Interlock)                 |
|   /rover/battery          (drillpulse_msgs/BatteryState - Throttling)   |
|   /cmd_vel_teleop         (geometry_msgs/Twist - Manual Operator)       |
|   /cmd_vel_nav            (geometry_msgs/Twist - Nav2 / Autonomous)     |
|   /cmd_vel                (geometry_msgs/Twist - Generic / Fallback)    |
+-------------------------------------------------------------------------+
                                    |
                                    v
+-------------------------------------------------------------------------+
|                  unified_mobility_controller_node                       |
|   - 8-Level Priority Ladder Resolution                                 |
|   - Slew-rate Acceleration Limiter (max 1.5 m/s2, 3.0 rad/s2)           |
|   - 500ms Command Watchdog                                              |
|   - 6-Wheel Rocker-Bogie Differential Kinematics                       |
|   - Publishes: /rover/motor_cmd_vel, /rover/mobility_state              |
+-------------------------------------------------------------------------+
                                    |
                                    v
+-------------------------------------------------------------------------+
|                       motor_telemetry_bridge                            |
|   - Formats Framed Packets: CMD,X,Y,SPEED,SEQ#CS\n                     |
|   - Serial USB to Arduino Uno @ 115200 baud                             |
|   - Receives Telemetry: S,TEMP,HUM,MQ4,DIST_L,DIST_R,BATT#CS\n          |
+-------------------------------------------------------------------------+
                                    |
                                    v
+-------------------------------------------------------------------------+
|                     Arduino Uno Motor Controller                        |
|   - Dual L298N H-Bridge Drivers                                         |
|   - Hardware E-Stop (INT0)                                              |
|   - 500ms Hardware Watchdog Auto-Cutoff                                 |
|   - Left Side Motors: LF, LM, LB                                        |
|   - Right Side Motors: RF, RM, RB                                       |
+-------------------------------------------------------------------------+
```

---

## 3. Strict 8-Level Priority Ladder

Every motion command passes through the single authoritative arbiter (`mobility_controller_node.py`):

| Priority Level | Trigger Condition | Arbiter Action | Resulting State |
| :---: | :--- | :--- | :--- |
| **1 (Highest)** | Emergency Stop (`/rover/estop = true`) | Immediate zero velocity | `ESTOP_TRIGGERED` |
| **2** | Actuator Interlock (`/rover/actuator_state` extending) | Immediate zero velocity | `ACTUATOR_INTERLOCK` |
| **3** | Hardware Fault (IMU/encoder failure, driver error) | Forced safe stop | `HARDWARE_FAULT` |
| **4** | Low Battery (V < 10.5V, SoC < 15%) | Speed throttled by 50% | `BATTERY_THROTTLED` |
| **5** | Comm Watchdog / Failsafe (no telemetry for > 3.0s) | Return to safe halt | `COMM_FAILSAFE` |
| **6** | Manual Teleoperation (`/cmd_vel_teleop`) | Operator joystick takes over | `MANUAL_TELEOP` |
| **7** | Autonomous Navigation (`/cmd_vel_nav`) | Nav2 path planner executes | `AUTONOMOUS_NAV` |
| **8 (Lowest)** | Standard Compatibility (`/cmd_vel`) | Fallback command | `GENERIC_CMD` |

---

## 4. Rocker-Bogie Kinematics & Serial Packet Mapping

For a 6-wheel rover with track width L = 0.90 m and wheel diameter D = 0.22 m:

### 4.1 Differential Velocities
- v_left = v_x - (omega_z * L / 2)
- v_right = v_x + (omega_z * L / 2)

### 4.2 Microcontroller Command Mapping
The Arduino controller expects normalized coordinate vectors (X, Y) in [-100, 100] and overall speed scalar S in [0, 255]:
- X = clamp(omega_z / omega_max * 100, -100, 100)
- Y = clamp(v_x / v_max * 100, -100, 100)
- S = clamp(max(|v_left|, |v_right|) * 255, 0, 255)

Serial Packet:
```text
CMD,<X>,<Y>,<Speed>,<Sequence_ID>#<Checksum>\n
```
Example:
```text
CMD,0,100,200,42#5A
```

---

## 5. Watchdog & Fail-Safe Timing Guarantees

1. **Pi-side Software Watchdog:** If no command is received on any motion topic within 500 ms, the mobility controller publishes (v_x = 0, omega_z = 0).
2. **Arduino Hardware Watchdog:** If no valid serial byte is received over USB for 500 ms, the firmware drops all PWM outputs to zero (`stop_all_motors()`).
3. **Rollover Interlock:** If roll or pitch angle exceeds 35 deg, the recovery system engages and locks mobility until self-righting is complete.
