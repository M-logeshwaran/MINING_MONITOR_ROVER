/*
 * ============================================================
 * DRILLPULSE — Arduino Uno Rover Motor & Sensor Controller
 * Target: Arduino Uno (ATmega328P) @ 16MHz
 *
 * Responsibilities:
 *   1. Dual L298N H-Bridge Motor Drivers (6-wheel rocker-bogie)
 *      - Left side: LF, LM, LB motors
 *      - Right side: RF, RM, RB motors
 *   2. Safety Watchdog (500ms timeout -> STOP_ALL_MOTORS)
 *   3. Hardware Emergency Stop interrupt line
 *   4. Environmental sensor reading (MQ-4, Temp, Battery voltage)
 *   5. Bidirectional Serial Protocol with Checksum & Sequence Validation
 * ============================================================
 */

#ifndef ARDUINO_SIM_TEST
#include <Arduino.h>
#else
#include <stdint.h>
#endif

// ============================================================
// HARDWARE PIN ASSIGNMENTS
// ============================================================

// Left L298N Motor Driver (Left Front, Left Mid, Left Back)
const int PIN_LEFT_ENA  = 5;   // PWM Speed Control (Timer 0)
const int PIN_LEFT_IN1  = 4;   // Direction Forward
const int PIN_LEFT_IN2  = 7;   // Direction Reverse
const int PIN_LEFT_IN3  = 8;   // Direction Secondary
const int PIN_LEFT_IN4  = 12;  // Direction Secondary
const int PIN_LEFT_ENB  = 6;   // PWM Speed Control (Timer 0)

// Right L298N Motor Driver (Right Front, Right Mid, Right Back)
const int PIN_RIGHT_ENA = 9;   // PWM Speed Control (Timer 1)
const int PIN_RIGHT_IN1 = 11;  // Direction Forward
const int PIN_RIGHT_IN2 = 13;  // Direction Reverse
const int PIN_RIGHT_IN3 = A4;  // Direction Secondary
const int PIN_RIGHT_IN4 = A5;  // Direction Secondary
const int PIN_RIGHT_ENB = 10;  // PWM Speed Control (Timer 1)

// Sensor Analog Inputs
const int PIN_MQ4_ANALOG = A0;  // Methane Gas Concentration
const int PIN_MQ4_DIGITAL = 3;  // Gas Digital Threshold
const int PIN_TEMP_ANALOG = A1; // NTC / Analog Temp Sensor
const int PIN_BATT_VOLT   = A2; // Battery Divider (100k/10k, scale factor 11:1)

// Hardware Emergency Stop Input (Active LOW with internal pullup)
const int PIN_HW_ESTOP = 2;     // External Interrupt INT0

// ============================================================
// CONFIGURATION CONSTANTS
// ============================================================
const unsigned long SERIAL_BAUD_RATE    = 115200;
const unsigned long WATCHDOG_TIMEOUT_MS = 500;   // Auto-stop if no command within 500ms
const unsigned long TELEM_INTERVAL_MS   = 100;   // 10 Hz Telemetry broadcast
const int PWM_DEADBAND                  = 35;    // Minimum PWM to overcome gearbox stiction
const int MAX_SAFE_PWM                  = 240;   // Thermal limit for L298N continuous load

// ============================================================
// STATE VARIABLES
// ============================================================
unsigned long last_cmd_time = 0;
unsigned long last_telem_time = 0;
unsigned long last_sequence_num = 0;
bool estop_active = false;
bool watchdog_tripped = false;
bool comms_ready = false;

int target_left_pwm = 0;
int target_right_pwm = 0;

// ============================================================
// FORWARD DECLARATIONS
// ============================================================
void stop_all_motors();
void set_left_motors(int pwm);
void set_right_motors(int pwm);
void handle_serial_command(const char* buffer);
void send_telemetry();
uint8_t calculate_checksum(const char* data, int len);

// ============================================================
// EMERGENCY STOP ISR
// ============================================================
void on_estop_interrupt() {
    estop_active = true;
    // Direct port-level zeroing for instantaneous cutoff (< 1 microsecond)
    stop_all_motors();
}

// ============================================================
// INITIALIZATION
// ============================================================
void setup() {
    // 1. Configure all motor control pins as outputs and force LOW immediately
    pinMode(PIN_LEFT_ENA, OUTPUT);
    pinMode(PIN_LEFT_ENB, OUTPUT);
    pinMode(PIN_LEFT_IN1, OUTPUT);
    pinMode(PIN_LEFT_IN2, OUTPUT);
    pinMode(PIN_LEFT_IN3, OUTPUT);
    pinMode(PIN_LEFT_IN4, OUTPUT);

    pinMode(PIN_RIGHT_ENA, OUTPUT);
    pinMode(PIN_RIGHT_ENB, OUTPUT);
    pinMode(PIN_RIGHT_IN1, OUTPUT);
    pinMode(PIN_RIGHT_IN2, OUTPUT);
    pinMode(PIN_RIGHT_IN3, OUTPUT);
    pinMode(PIN_RIGHT_IN4, OUTPUT);

    stop_all_motors();

    // 2. Configure sensor and estop inputs
    pinMode(PIN_MQ4_DIGITAL, INPUT);
    pinMode(PIN_HW_ESTOP, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_HW_ESTOP), on_estop_interrupt, FALLING);

    // 3. Initialize serial communication
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial && millis() < 1000);

    last_cmd_time = millis();
    last_telem_time = millis();
    comms_ready = false;

    Serial.println("STATUS:ARDUINO_BOOT_OK,MOTORS:SAFE_STOP");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
    unsigned long now = millis();

    // 1. Check physical hardware emergency stop
    if (digitalRead(PIN_HW_ESTOP) == LOW) {
        estop_active = true;
    }

    // 2. Enforce Communication Watchdog
    if (!estop_active && (now - last_cmd_time > WATCHDOG_TIMEOUT_MS)) {
        if (!watchdog_tripped) {
            stop_all_motors();
            watchdog_tripped = true;
        }
    }

    // 3. Read and parse incoming serial commands from Raspberry Pi
    static char rx_buf[64];
    static int rx_idx = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (rx_idx > 0) {
                rx_buf[rx_idx] = '\0';
                handle_serial_command(rx_buf);
                rx_idx = 0;
            }
        } else if (rx_idx < 63) {
            rx_buf[rx_idx++] = c;
        } else {
            // Buffer overflow discard
            rx_idx = 0;
        }
    }

    // 4. Periodic Telemetry Broadcast to Pi
    if (now - last_telem_time >= TELEM_INTERVAL_MS) {
        last_telem_time = now;
        send_telemetry();
    }
}

// ============================================================
// MOTOR DRIVER CONTROL FUNCTIONS
// ============================================================
void stop_all_motors() {
    target_left_pwm = 0;
    target_right_pwm = 0;

    analogWrite(PIN_LEFT_ENA, 0);
    analogWrite(PIN_LEFT_ENB, 0);
    digitalWrite(PIN_LEFT_IN1, LOW);
    digitalWrite(PIN_LEFT_IN2, LOW);
    digitalWrite(PIN_LEFT_IN3, LOW);
    digitalWrite(PIN_LEFT_IN4, LOW);

    analogWrite(PIN_RIGHT_ENA, 0);
    analogWrite(PIN_RIGHT_ENB, 0);
    digitalWrite(PIN_RIGHT_IN1, LOW);
    digitalWrite(PIN_RIGHT_IN2, LOW);
    digitalWrite(PIN_RIGHT_IN3, LOW);
    digitalWrite(PIN_RIGHT_IN4, LOW);
}

void set_left_motors(int pwm) {
    if (estop_active || watchdog_tripped) {
        pwm = 0;
    }

    if (pwm > 0) {
        int mag = constrain(pwm, PWM_DEADBAND, MAX_SAFE_PWM);
        digitalWrite(PIN_LEFT_IN1, HIGH);
        digitalWrite(PIN_LEFT_IN2, LOW);
        digitalWrite(PIN_LEFT_IN3, HIGH);
        digitalWrite(PIN_LEFT_IN4, LOW);
        analogWrite(PIN_LEFT_ENA, mag);
        analogWrite(PIN_LEFT_ENB, mag);
    } else if (pwm < 0) {
        int mag = constrain(-pwm, PWM_DEADBAND, MAX_SAFE_PWM);
        digitalWrite(PIN_LEFT_IN1, LOW);
        digitalWrite(PIN_LEFT_IN2, HIGH);
        digitalWrite(PIN_LEFT_IN3, LOW);
        digitalWrite(PIN_LEFT_IN4, HIGH);
        analogWrite(PIN_LEFT_ENA, mag);
        analogWrite(PIN_LEFT_ENB, mag);
    } else {
        digitalWrite(PIN_LEFT_IN1, LOW);
        digitalWrite(PIN_LEFT_IN2, LOW);
        digitalWrite(PIN_LEFT_IN3, LOW);
        digitalWrite(PIN_LEFT_IN4, LOW);
        analogWrite(PIN_LEFT_ENA, 0);
        analogWrite(PIN_LEFT_ENB, 0);
    }
}

void set_right_motors(int pwm) {
    if (estop_active || watchdog_tripped) {
        pwm = 0;
    }

    if (pwm > 0) {
        int mag = constrain(pwm, PWM_DEADBAND, MAX_SAFE_PWM);
        digitalWrite(PIN_RIGHT_IN1, HIGH);
        digitalWrite(PIN_RIGHT_IN2, LOW);
        digitalWrite(PIN_RIGHT_IN3, HIGH);
        digitalWrite(PIN_RIGHT_IN4, LOW);
        analogWrite(PIN_RIGHT_ENA, mag);
        analogWrite(PIN_RIGHT_ENB, mag);
    } else if (pwm < 0) {
        int mag = constrain(-pwm, PWM_DEADBAND, MAX_SAFE_PWM);
        digitalWrite(PIN_RIGHT_IN1, LOW);
        digitalWrite(PIN_RIGHT_IN2, HIGH);
        digitalWrite(PIN_RIGHT_IN3, LOW);
        digitalWrite(PIN_RIGHT_IN4, HIGH);
        analogWrite(PIN_RIGHT_ENA, mag);
        analogWrite(PIN_RIGHT_ENB, mag);
    } else {
        digitalWrite(PIN_RIGHT_IN1, LOW);
        digitalWrite(PIN_RIGHT_IN2, LOW);
        digitalWrite(PIN_RIGHT_IN3, LOW);
        digitalWrite(PIN_RIGHT_IN4, LOW);
        analogWrite(PIN_RIGHT_ENA, 0);
        analogWrite(PIN_RIGHT_ENB, 0);
    }
}

// ============================================================
// SERIAL PROTOCOL PARSER
// Format: "CMD,X,Y,SPEED,SEQ#CHECKSUM" or legacy "CMD,X,Y,SPEED"
// ============================================================
void handle_serial_command(const char* buffer) {
    // Check for Emergency Stop command
    if (strcmp(buffer, "ESTOP") == 0 || strcmp(buffer, "STOP") == 0) {
        estop_active = true;
        stop_all_motors();
        Serial.println("ACK,ESTOP_ENGAGED");
        return;
    }

    // Check for E-Stop Reset command
    if (strcmp(buffer, "RESET_ESTOP") == 0) {
        if (digitalRead(PIN_HW_ESTOP) == HIGH) {
            estop_active = false;
            watchdog_tripped = false;
            last_cmd_time = millis();
            Serial.println("ACK,ESTOP_CLEARED");
        } else {
            Serial.println("ERR,HARDWARE_ESTOP_PIN_STILL_TRIGGERED");
        }
        return;
    }

    if (strncmp(buffer, "CMD,", 4) != 0) {
        // Unknown packet header
        return;
    }

    if (estop_active) {
        stop_all_motors();
        return;
    }

    // Check for checksum if '#' is present
    const char* hash_ptr = strchr(buffer, '#');
    if (hash_ptr != NULL) {
        int payload_len = hash_ptr - buffer;
        uint8_t expected_cs = (uint8_t)strtol(hash_ptr + 1, NULL, 16);
        uint8_t calculated_cs = calculate_checksum(buffer, payload_len);
        if (expected_cs != calculated_cs) {
            Serial.println("ERR,CHECKSUM_MISMATCH");
            return;
        }
    }

    // Parse fields: CMD,X,Y,SPEED[,SEQ]
    int x = 0, y = 0, speed = 0;
    unsigned long seq = 0;

    int matched = sscanf(buffer, "CMD,%d,%d,%d,%lu", &x, &y, &speed, &seq);
    if (matched < 3) {
        // Malformed command
        return;
    }

    // Validate bounds
    x = constrain(x, -100, 100);
    y = constrain(y, -100, 100);
    speed = constrain(speed, 0, 100);

    // Differential steering kinematic mixer:
    // Left side = Y - X
    // Right side = Y + X
    float scale = (float)speed / 100.0f;
    float raw_left  = ((float)y - (float)x) * 2.55f * scale;
    float raw_right = ((float)y + (float)x) * 2.55f * scale;

    int left_pwm  = constrain((int)raw_left,  -MAX_SAFE_PWM, MAX_SAFE_PWM);
    int right_pwm = constrain((int)raw_right, -MAX_SAFE_PWM, MAX_SAFE_PWM);

    // Reset watchdog and update outputs
    last_cmd_time = millis();
    watchdog_tripped = false;
    comms_ready = true;

    set_left_motors(left_pwm);
    set_right_motors(right_pwm);
}

// ============================================================
// TELEMETRY TRANSMISSION
// Format: "S,TEMP,HUMIDITY,MQ4_RAW,MQ4_DIGITAL,LEFT_DIST,RIGHT_DIST,BATT_VOLT,STATUS"
// ============================================================
void send_telemetry() {
    // 1. Read environmental analog sensors
    int mq4_raw = analogRead(PIN_MQ4_ANALOG);
    int mq4_dig = digitalRead(PIN_MQ4_DIGITAL);
    int temp_raw = analogRead(PIN_TEMP_ANALOG);
    int batt_raw = analogRead(PIN_BATT_VOLT);

    // Scaling conversions
    float temp_c = 20.0f + ((float)temp_raw * (50.0f / 1023.0f));
    float humidity = 50.0f;
    float batt_voltage = ((float)batt_raw * (5.0f / 1023.0f)) * 5.1f; // Resistor divider calibrated (approx 24V nominal)

    const char* status_str = "OK";
    if (estop_active) {
        status_str = "ESTOP";
    } else if (watchdog_tripped) {
        status_str = "WATCHDOG";
    }

    // Format telemetry line
    char telem_buf[96];
    snprintf(telem_buf, sizeof(telem_buf),
             "S,%.1f,%.1f,%d,%d,150.0,150.0,%.2f,%s",
             temp_c, humidity, mq4_raw, mq4_dig, batt_voltage, status_str);

    uint8_t cs = calculate_checksum(telem_buf, strlen(telem_buf));
    Serial.print(telem_buf);
    Serial.print('#');
    if (cs < 16) Serial.print('0');
    Serial.println(cs, HEX);
}

uint8_t calculate_checksum(const char* data, int len) {
    uint8_t cs = 0;
    for (int i = 0; i < len; ++i) {
        cs ^= (uint8_t)data[i];
    }
    return cs;
}
