# DrillPulse Firmware Architecture & Hardware Controllers

## Overview

The DrillPulse physical rover utilizes three dedicated embedded microcontrollers running specialized C/C++ firmware to isolate real-time motor actuation, linear safety recovery mechanisms, and high-frequency wheel encoder pulse counting from the high-level ROS 2 Jazzy compute environment on the Raspberry Pi 5.

```
                      +---------------------------------------+
                      |   Raspberry Pi 5 (ROS 2 Jazzy Linux)  |
                      +---------------------------------------+
                           |              |              |
          /dev/ttyACM0 USB |  /dev/ttyACM1|  /dev/ttyUSB0| USB-UART
                           v              v              v
            +-------------------+  +---------------+  +---------------------+
            | Arduino Mega 2560 |  |  Arduino Uno  |  | STM32F401RE Nucleo  |
            | Motor Controller  |  | Actuators     |  | 6-Wheel Encoders    |
            +-------------------+  +---------------+  +---------------------+
                     |                     |                     |
                     v                     v                     v
            Dual L298N Drivers     Dual Linear Jacks     6x 1040 PPR Optical
            6-Wheel Drive Motors   Roll Recovery Arms    Wheel Encoders
```

---

## 1. Arduino Motor Controller (`arduino_motor_controller.ino`)

### Target Hardware & Driver Topology
- **Board**: Arduino Mega 2560 (ATmega2560 @ 16 MHz) / Uno compatible
- **Motor Drivers**: Dual L298N H-Bridge drivers driving 6 high-torque planetary DC gear motors (3 Left in parallel, 3 Right in parallel)
- **Serial Interface**: USB CDC serial at 115200 baud (`/dev/ttyACM0` or `/dev/drillpulse_motors`)

### Pin Assignments
| Pin | Direction | Function | Description |
|:---|:---|:---|:---|
| `D5` | OUTPUT | Left Motor PWM A | Left side forward PWM speed (0-255) |
| `D6` | OUTPUT | Left Motor PWM B | Left side reverse PWM speed (0-255) |
| `D9` | OUTPUT | Right Motor PWM A | Right side forward PWM speed (0-255) |
| `D10` | OUTPUT | Right Motor PWM B | Right side reverse PWM speed (0-255) |
| `D2` | INPUT_PULLUP | Hardware E-Stop | External INT0; triggers instantaneous motor halt on falling edge |
| `D13` | OUTPUT | Status LED | Heartbeat / Watchdog indicator |

### Serial Protocol & Checksum Validation
The motor controller accepts ASCII newline-terminated commands with XOR checksum verification:
- **Format**: `M,<left_pwm>,<right_pwm>,<checksum>\n`
- **Range**: `-255 <= left_pwm, right_pwm <= 255`
- **Checksum Calculation**: Bitwise XOR of all characters between `M,` and the comma preceding the checksum.
- **Example**: `M,150,150,56\n`
- **Response**: `OK\n` or `ERR,<code:TIMEOUT|CHECKSUM|INVALID_PWM>\n`

### Fail-Safe Watchdog & E-Stop
- **Software Watchdog**: Implemented via `millis()` timer checking time elapsed since last valid packet. If elapsed time exceeds **500 ms**, all motor PWM outputs are instantly driven to `0` (motor braking).
- **Hardware E-Stop Interrupt**: Pin `D2` is wired to an active-low physical emergency push-button using internal pullup resistor. When pulled to GND (`FALLING` edge interrupt INT0), the firmware invokes `emergencyStopISR()`, setting a volatile flag that overrides all PWM outputs to zero in `< 5 microseconds`.

---

## 2. Arduino Actuator Controller (`arduino_actuator_controller.ino`)

### Target Hardware & Driver Topology
- **Board**: Arduino Uno / Nano (ATmega328P @ 16 MHz)
- **Actuators**: Dual high-torque 12V DC linear actuators (stroke length 150 mm) configured as stabilizer/righting arms for roll-angle recovery.
- **Drivers**: Relay H-bridge / BTS7960 high-current driver module.
- **Serial Interface**: USB CDC serial at 115200 baud (`/dev/ttyACM1` or `/dev/drillpulse_actuators`).

### Pin Assignments
| Pin | Direction | Function | Description |
|:---|:---|:---|:---|
| `D3` | OUTPUT | Actuator 1 Extend | Relay/FET control to extend Left linear jack |
| `D4` | OUTPUT | Actuator 1 Retract | Relay/FET control to retract Left linear jack |
| `D7` | OUTPUT | Actuator 2 Extend | Relay/FET control to extend Right linear jack |
| `D8` | OUTPUT | Actuator 2 Retract | Relay/FET control to retract Right linear jack |
| `D13` | OUTPUT | Actuator Activity LED | Lit when actuators are actively powered |

### Safety Constraints & Hardware Interlocks
1. **Mutual Exclusion**: Hardware and software prevent simultaneous assertion of Extend and Retract lines for any actuator. Extending pin must be brought LOW before Retracting pin can be activated.
2. **Stroke Time Limit (3.5 Seconds Max)**: Due to physical stroke limits (150 mm @ 42 mm/s), continuous power is capped at **3500 ms**. Once this timer expires, power is automatically cut to prevent motor stall, overheating, or gear stripping.
3. **Thermal Cooldown Delay (2.0 Seconds Min)**: After any stroke motion ceases, an enforced 2000 ms cooldown period blocks reverse actuation to allow motor windings and relays to de-energize and dissipate inductive flyback spikes.
4. **Command Protocol**:
   - `EXTEND,<id:1|2|ALL>`
   - `RETRACT,<id:1|2|ALL>`
   - `STOP`
   - `STATUS` (returns current extension state, elapsed stroke time, and thermal cooldown status).

---

## 3. STM32 Encoder Controller (`stm32_encoder_controller.c` / `.h`)

### Target Hardware & Encoder Topology
- **Board**: STM32F401RE Nucleo-64 (ARM Cortex-M4 @ 84 MHz)
- **Encoders**: 6x optical quadrature encoders mounted directly to the wheel axles:
  - Front-Left (FL), Front-Right (FR)
  - Mid-Left (ML), Mid-Right (MR)
  - Rear-Left (RL), Rear-Right (RR)
- **Resolution**: 1040 Pulses Per Revolution (PPR) per wheel.
- **Serial Interface**: Hardware USART2 via onboard ST-LINK VCP (`/dev/ttyUSB0` or `/dev/drillpulse_encoders`) at 115200 baud.

### Digital Filtering & Glitch Rejection
Underground mine operations produce high vibrational shock and electrical interference from motor brushes:
- **Sample Debouncing**: Encoder channel inputs pass through a 3-sample digital consensus filter running in an 84 MHz SysTick interrupt (timer sampling frequency 100 kHz).
- **Glitch Threshold**: Pulses shorter than 10 microseconds are rejected as brush noise spikes.
- **Direction Decoding**: Quadrature state transitions (Gray code decode) update 32-bit signed pulse counters `ticks[6]` with overflow protection.

### Binary Telemetry Packet Protocol
To minimize serial overhead and guarantee deterministic 50 Hz reporting, encoder data is packed into a compact binary frame:

| Offset | Field | Type | Description |
|:---|:---|:---|:---|
| 0 | Header 1 | `uint8_t` | Magic byte `0xAA` |
| 1 | Header 2 | `uint8_t` | Magic byte `0x55` |
| 2-5 | FL Ticks | `int32_t` (Little-endian) | Front Left wheel cumulative tick count |
| 6-9 | FR Ticks | `int32_t` (Little-endian) | Front Right wheel cumulative tick count |
| 10-13 | ML Ticks | `int32_t` (Little-endian) | Middle Left wheel cumulative tick count |
| 14-17 | MR Ticks | `int32_t` (Little-endian) | Middle Right wheel cumulative tick count |
| 18-21 | RL Ticks | `int32_t` (Little-endian) | Rear Left wheel cumulative tick count |
| 22-25 | RR Ticks | `int32_t` (Little-endian) | Rear Right wheel cumulative tick count |
| 26 | Checksum | `uint8_t` | XOR of bytes 2 through 25 |
| 27 | Delimiter 1 | `uint8_t` | `0x0D` (`\r`) |
| 28 | Delimiter 2 | `uint8_t` | `0x0A` (`\n`) |

Total frame length: **29 bytes**. At 50 Hz, serial bandwidth requirement is only 1450 bytes/second.

---

## 4. Host Unit Testing & Verification

All three firmware codebases feature native host-compilable C/C++ unit test harnesses in `firmware/`:
- `firmware/arduino_motor_controller/test_arduino_motor_controller.cpp`
- `firmware/arduino_actuator_controller/test_actuator_controller.cpp`
- `firmware/stm32_encoder_controller/test_stm32_encoder.c`

All 3 firmware tests are integrated into the master test pipeline `./test_all.sh`.
