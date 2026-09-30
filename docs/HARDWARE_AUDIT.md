# DRILLPULSE — HARDWARE & SENSOR SUBSYSTEM AUDIT

## 1. Overview
This document provides a comprehensive line-by-line audit of all microcontroller firmware, serial communication bridges, sensor drivers, and hardware controllers in the DrillPulse rover platform.

---

## 2. Microcontroller Firmware & Pin Allocation

### 2.1 Arduino Uno — Motor & Telemetry Controller
* **File Location:** `firmware/arduino_motor_controller/arduino_motor_controller.ino`
* **Target MCU:** ATmega328P (Arduino Uno) @ 16 MHz
* **Baudrate:** 115200 baud
* **Pin Assignments:**
  | Subsystem | Signal | Pin | Type | Notes |
  | :--- | :--- | :--- | :--- | :--- |
  | **Drivetrain** | Left PWM (ENA) | D5 | PWM (Timer0) | Left-side motor speed |
  | | Left Direction (IN1) | D7 | Digital Out | Direction polarity |
  | | Left Direction (IN2) | D8 | Digital Out | Direction polarity |
  | | Right PWM (ENB) | D6 | PWM (Timer0) | Right-side motor speed |
  | | Right Direction (IN3) | D9 | Digital Out | Direction polarity |
  | | Right Direction (IN4) | D10 | Digital Out | Direction polarity |
  | **Safety** | Emergency Stop | D2 | External INT0 | Active LOW physical bumper / e-stop |
  | **Sensors** | DHT11 Environmental | D4 | OneWire | Temperature & Humidity |
  | | MQ-4 Methane Gas | A0 | Analog In | Concentration voltage (0-5V) |
  | | MQ-4 Gas Alarm | D3 | Digital In | Fast threshold trip |
  | | Left Ultrasonic Trig | D11 | Digital Out | Ranging trigger |
  | | Left Ultrasonic Echo | D12 | Digital In | Echo pulse duration |
  | | Right Ultrasonic Trig | D13 | Digital Out | Ranging trigger |
  | | Right Ultrasonic Echo | A1 | Digital In | Echo pulse duration |
  | | Battery Voltage Monitor | A2 | Analog In | 3S LiPo potential divider (11:1) |

* **Hardware Watchdog:** ATmega Timer / millis check cuts PWM to 0 if no valid serial command is received within 500 ms.
* **Serial Protocol:**
  - Inbound (Pi -> Uno): `CMD,<x>,<y>,<speed>,<seq>#<checksum>\n`
  - Outbound (Uno -> Pi): `S,<temp>,<hum>,<mq4_raw>,<mq4_dig>,<ldist>,<rdist>,<batt_v>,<status>#<checksum>\n`
  - Fallback: Legacy comma-separated format without checksum is parsed if checksum is absent.

---

### 2.2 STM32 — 6-Wheel Quadrature Encoder Tracker
* **File Locations:** `firmware/stm32_encoder_controller/stm32_encoder_controller.h`, `.c`
* **Target MCU:** STM32F401 / STM32F411 BlackPill
* **Baudrate:** 115200 baud
* **Timer Allocations & Pin Assignments:**
  | Wheel Position | Hardware Timer | Pins | Mode |
  | :--- | :--- | :--- | :--- |
  | **Left Front (LF)** | TIM2 | PA0, PA1 | 4x Quadrature Encoder Mode |
  | **Right Front (RF)** | TIM3 | PA6, PA7 | 4x Quadrature Encoder Mode |
  | **Left Middle (LM)** | TIM4 | PB6, PB7 | 4x Quadrature Encoder Mode |
  | **Right Middle (RM)** | TIM1 | PA8, PA9 | 4x Quadrature Encoder Mode |
  | **Left Back (LB)** | TIM5 | PA0, PA1 (or TIM9) | Software interrupt capture |
  | **Right Back (RB)** | TIM10/11 | PB8, PB9 | Software interrupt capture |

* **Gating & Anomaly Filter:** RPM values exceeding 250 RPM are clamped and flagged as `DEGRADED`. Overflow and negative rollover are handled via 32-bit delta tracking.
* **Serial Protocol:**
  - Outbound (STM32 -> Pi): `ENC,<timestamp_ms>,<lf>,<rf>,<lm>,<rm>,<lb>,<rb>,<health>#<checksum>\n`
  - Health Codes: `0` = OK, `1` = DEGRADED, `2` = ERROR.

---

### 2.3 Arduino Uno / Nano — Self-Righting Linear Actuators
* **File Location:** `firmware/arduino_actuator_controller/arduino_actuator_controller.ino`
* **Target MCU:** ATmega328P
* **Baudrate:** 115200 baud
* **Pin Assignments:**
  | Actuator | Function | Pin | Notes |
  | :--- | :--- | :--- | :--- |
  | **Left Actuator** | Extend Relay/PWM | D4 | Drives left piston outward |
  | | Retract Relay/PWM | D5 | Retracts left piston inward |
  | **Right Actuator** | Extend Relay/PWM | D6 | Drives right piston outward |
  | | Retract Relay/PWM | D7 | Retracts right piston inward |
  | **Limit Switches** | Left Retracted (Home) | D8 | Active LOW optical/microswitch |
  | | Left Extended | D9 | Active LOW stroke cutoff |
  | | Right Retracted (Home) | D10 | Active LOW optical/microswitch |
  | | Right Extended | D11 | Active LOW stroke cutoff |

* **Hardware Interlocks:**
  1. **Mutual Exclusion:** Left and Right extend relays are electronically and programmatically prohibited from simultaneous activation.
  2. **3.5s Stroke Timeout:** Firmware automatically cuts motor driver power if stroke exceeds 3.5 seconds.
  3. **2.0s Cooldown:** Mandatory 2.0s resting period enforced between consecutive actuations to prevent thermal overload.

---

## 3. Onboard ROS 2 Hardware Bridges

| Node Name | Package | Executable | Port | Health Topic | Fallback Policy |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `mobility_controller_node` | `drillpulse_hardware` | `mobility_controller_node` | Internal ROS 2 | `/rover/mobility_state` | Holds last stop, arbitration failsafe |
| `motor_telemetry_bridge` | `drillpulse_hardware` | `motor_telemetry_bridge` | `/dev/ttyACM0` | `/rover/battery` (status) | Reports `DISCONNECTED`; no fake motion |
| `stm32_encoder_bridge` | `drillpulse_hardware` | `stm32_encoder_bridge` | `/dev/ttyACM1` | `/rover/encoder_health` | Reports `DISCONNECTED`; no fake ticks |
| `sparkfun_imu_node` | `drillpulse_imu` | `sparkfun_imu_node` | `/dev/ttyACM0` | `/rover/imu_health` | Sets covariance -1.0; no false flip |
| `actuator_bridge` | `drillpulse_hardware` | `actuator_bridge` | `/dev/ttyACM2` | `/rover/actuator_state` | Reports `DISCONNECTED`; locks mobility |

---

## 4. Verification Status
* **Compilation & Unit Testing:** All microcontroller firmware C/C++ test harnesses compiled and passed 100%.
* **ROS 2 Bridges:** All Python bridges checked against AST interface validation and passed.
* **Hardware Bench Requirement:** Complete end-to-end motor spin and piston actuation verified via software integration test; physical verification requires USB device enumeration on physical rover hardware.
