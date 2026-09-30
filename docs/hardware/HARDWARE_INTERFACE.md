# DrillPulse — Hardware Interface Specification

**Document:** `docs/HARDWARE_INTERFACE.md`  
**Status:** Authoritative Specification  
**Architecture:** Ubuntu 24.04 LTS | ROS 2 Jazzy

---

## 1. Overview

The DrillPulse rover relies on multiple microcontroller and sensor bridges interfacing over serial UART, USB, and I2C buses.

> [!NOTE]
> Microcontroller firmware source code for Arduino and STM32 is maintained externally. The Python drivers within `drillpulse_hardware` implement the canonical serial wire contracts.

---

## 2. Hardware Subsystems

### 2.1 STM32 Wheel Encoder Subsystem
- **Port:** `/dev/ttyACM0` or `/dev/ttyUSB0` (default: 115200 baud)
- **Protocol:** ASCII comma-separated wheel RPM values: `ENC,w1,w2,w3,w4,w5,w6\n`
- **Driver Node:** `stm32_encoder_bridge` in `drillpulse_hardware`
- **Output Topic:** `/stm32/wheel_rpms` (`std_msgs/Float32MultiArray`)
- **Status:** Driver operational in simulation fallback when physical serial disconnected. `FIRMWARE SOURCE NOT INCLUDED`.

### 2.2 Arduino Motor & Environmental Bridge
- **Port:** `/dev/rfcomm0` or `/dev/ttyUSB1` (default: 9600 baud)
- **Motor Control Protocol:** `CMD,x_vel,y_vel,speed_scale\n`
- **Environmental Telemetry:** `S,temp,humidity,gas_analog,ldist,rdist\n`
- **Driver Node:** `motor_telemetry_bridge` in `drillpulse_hardware`
- **Topics:** `/rover/cmd_vel` (input), `/rover/telemetry`, `/drillpulse/temperature`, `/drillpulse/humidity`, `/drillpulse/gas`
- **Status:** `FIRMWARE SOURCE NOT INCLUDED`.

### 2.3 SparkFun ICM-20948 IMU
- **Port:** `/dev/ttyACM1` or `/dev/ttyUSB2` (default: 115200 baud)
- **Wire Format:** `ax,ay,az,gx,gy,gz\n` (calibrated m/s² and rad/s)
- **Driver Node:** `sparkfun_imu_node` in `drillpulse_imu`
- **Output Topics:** `/imu/data` (canonical), `/sparkfun/imu/data` (compatibility)

### 2.4 RPLIDAR A1/A2
- **Port:** `/dev/ttyUSB0` (115200 baud)
- **Driver:** `rplidar_ros`
- **Output Topic:** `/scan` (`sensor_msgs/LaserScan`, frame: `laser_frame`)

### 2.5 Rollover Recovery Linear Actuators
- **Port:** `/dev/ttyACM2` (115200 baud)
- **Commands:** `L_EXT\n`, `R_EXT\n`, `RETRACT_ALL\n`, `STOP_ALL\n`
- **Driver Node:** `actuator_bridge` in `drillpulse_hardware`
- **Topics:** `/rover/recovery_command` (input), `/rover/actuator_state` (output)
