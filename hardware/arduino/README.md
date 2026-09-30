# DrillPulse Arduino Embedded Firmware

This directory contains Arduino firmware sketches used across the physical DrillPulse mine rover platform and Version-1 prototype.

## Directory Structure & Modules

### 1. `mobility/` (Arduino Mega 2560 - Current SIH Architecture)
- **File**: `arduino_motor_controller.ino`
- **Unit Test**: `test_arduino_motor_controller.cpp`
- **Target Hardware**: Arduino Mega 2560 (ATmega2560 @ 16 MHz)
- **Role**: Drives 6 high-torque planetary DC gear motors via dual L298N H-Bridge drivers (3 Left motors in parallel, 3 Right motors in parallel).
- **Key Safety Features**:
  - 500ms serial watchdog timer (auto-stops motors if high-level commands stall).
  - Hardware E-Stop line (D2 - INT0 interrupt) cutting PWM outputs in < 5 microseconds.
  - XOR checksum verification on ASCII command frames: `M,<left_pwm>,<right_pwm>#<checksum>\n`.

### 2. `actuators/` (Arduino Uno - Current SIH Architecture)
- **File**: `arduino_actuator_controller.ino`
- **Unit Test**: `test_actuator_controller.cpp`
- **Target Hardware**: Arduino Uno / Nano (ATmega328P @ 16 MHz)
- **Role**: Controls dual 12V high-torque telescoping linear actuator jacks for self-righting after vehicle rollover.
- **Key Safety Features**:
  - Mutual exclusion interlock: left and right actuators are physically and logically blocked from concurrent extension.
  - 3.5s maximum stroke timeout: automatically cuts motor power to prevent mechanical stall and winding burnout.
  - 2.0s mandatory thermal cooldown: blocks directional reversal to allow inductive flyback dissipation.

### 3. `sensors/` (Arduino Uno - Sensor Subsystem)
- **File**: `temp_humidity.ino`
- **Target Hardware**: Arduino Uno / Nano with DHT22 / AM2302 sensor on pin A1.
- **Role**: Reads ambient subterranean temperature and relative humidity, transmitting readings over 9600 baud serial.

### 4. `wifi_bridge/` (Version-1 Prototype ESP32 / Arduino Bridge)
- **Files**: `drillpulse_arduino_wifi_bridge.ino`, `esp32_cam_publish.ino`
- **Role**: Early prototype wireless bridging and ESP32 camera streaming over local Wi-Fi network.

## Build and Testing
Microcontroller sketches can be compiled via Arduino IDE or `arduino-cli`.
Host unit tests for mobility and actuator logic can be compiled directly on Linux using G++:
```bash
g++ -std=c++17 -Wall -Wextra mobility/test_arduino_motor_controller.cpp -o /tmp/test_motor && /tmp/test_motor
g++ -std=c++17 -Wall -Wextra actuators/test_actuator_controller.cpp -o /tmp/test_actuator && /tmp/test_actuator
```
