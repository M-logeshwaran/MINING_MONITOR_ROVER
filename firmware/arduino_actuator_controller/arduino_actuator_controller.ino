/*
 * ============================================================
 * DRILLPULSE — Linear Actuator Rollover Recovery Controller
 * Target: Arduino Uno / Nano / Pro Mini
 *
 * Responsibilities:
 *   1. Left and Right Telescoping Linear Actuator H-Bridge Relays
 *   2. Strict Physical Safety Interlocks:
 *      - Mutual Exclusion: NEVER activate Left and Right simultaneously
 *      - Maximum Extension Stroke Timer (3.5 seconds cutoff)
 *      - Thermal Cooldown Protection (2.0 seconds minimum rest)
 *   3. Hardware Emergency Cutoff
 *   4. Serial Command Interface: L_EXT, R_EXT, RETRACT_ALL, STOP_ALL
 * ============================================================
 */

#ifndef ARDUINO_SIM_TEST
#include <Arduino.h>
#else
#include <stdint.h>
#endif

// ============================================================
// HARDWARE PIN DEFINITIONS
// ============================================================
// Left Actuator Relay / H-Bridge Pins
const int PIN_LEFT_EXT   = 7;   // Left Piston Extend (Active HIGH)
const int PIN_LEFT_RET   = 8;   // Left Piston Retract (Active HIGH)

// Right Actuator Relay / H-Bridge Pins
const int PIN_RIGHT_EXT  = 9;   // Right Piston Extend (Active HIGH)
const int PIN_RIGHT_RET  = 10;  // Right Piston Retract (Active HIGH)

// Emergency Stop Line
const int PIN_ACT_ESTOP  = 2;

// ============================================================
// TIMING CONSTANTS
// ============================================================
const unsigned long MAX_STROKE_TIME_MS = 3500;  // Auto-cutoff after 3.5s extension
const unsigned long COOLDOWN_TIME_MS   = 2000;  // 2.0s mandatory thermal cooldown
const unsigned long SERIAL_BAUD        = 115200;

// ============================================================
// STATE MACHINE
// ============================================================
enum ActuatorState {
    ACT_IDLE,
    ACT_EXTENDING_LEFT,
    ACT_EXTENDING_RIGHT,
    ACT_RETRACTING,
    ACT_COOLDOWN,
    ACT_ESTOP
};

ActuatorState act_state = ACT_IDLE;
unsigned long state_start_time = 0;
unsigned long cooldown_start_time = 0;
bool estop_latched = false;

void handle_command(const char* cmd);
void report_status();
void emergency_stop_isr();

// ============================================================
// CONTROL FUNCTIONS
// ============================================================
void stop_all_actuators() {
    digitalWrite(PIN_LEFT_EXT, LOW);
    digitalWrite(PIN_LEFT_RET, LOW);
    digitalWrite(PIN_RIGHT_EXT, LOW);
    digitalWrite(PIN_RIGHT_RET, LOW);
}

void setup() {
    pinMode(PIN_LEFT_EXT, OUTPUT);
    pinMode(PIN_LEFT_RET, OUTPUT);
    pinMode(PIN_RIGHT_EXT, OUTPUT);
    pinMode(PIN_RIGHT_RET, OUTPUT);
    stop_all_actuators();

    pinMode(PIN_ACT_ESTOP, INPUT_PULLUP);

    Serial.begin(SERIAL_BAUD);
    act_state = ACT_IDLE;
    Serial.println("STATUS:ACTUATORS_INITIALIZED_SAFE_IDLE");
}

void loop() {
    unsigned long now = millis();

    // 1. Check physical hardware emergency stop
    if (digitalRead(PIN_ACT_ESTOP) == LOW) {
        estop_latched = true;
        stop_all_actuators();
        act_state = ACT_ESTOP;
    }

    // 2. Monitor Maximum Stroke Duration
    if (act_state == ACT_EXTENDING_LEFT || act_state == ACT_EXTENDING_RIGHT || act_state == ACT_RETRACTING) {
        if (now - state_start_time >= MAX_STROKE_TIME_MS) {
            stop_all_actuators();
            act_state = ACT_COOLDOWN;
            cooldown_start_time = now;
            Serial.println("WARN:MAX_STROKE_TIMEOUT_ENGAGED_COOLDOWN");
        }
    }

    // 3. Monitor Cooldown State
    if (act_state == ACT_COOLDOWN) {
        if (now - cooldown_start_time >= COOLDOWN_TIME_MS) {
            act_state = ACT_IDLE;
            Serial.println("STATUS:COOLDOWN_COMPLETE_READY");
        }
    }

    // 4. Parse incoming serial commands
    static char buf[32];
    static int idx = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (idx > 0) {
                buf[idx] = '\0';
                handle_command(buf);
                idx = 0;
            }
        } else if (idx < 31) {
            buf[idx++] = c;
        } else {
            idx = 0;
        }
    }
}

void handle_command(const char* cmd) {
    unsigned long now = millis();

    if (strcmp(cmd, "STOP_ALL") == 0 || strcmp(cmd, "ESTOP") == 0) {
        stop_all_actuators();
        act_state = ACT_COOLDOWN;
        cooldown_start_time = now;
        Serial.println("ACK:STOP_ALL");
        return;
    }

    if (estop_latched) {
        Serial.println("ERR:ESTOP_ACTIVE");
        return;
    }

    if (act_state == ACT_COOLDOWN) {
        Serial.println("ERR:IN_COOLDOWN");
        return;
    }

    if (strcmp(cmd, "L_EXT") == 0) {
        // Enforce Mutual Exclusion: Right must be completely OFF
        stop_all_actuators();
        digitalWrite(PIN_LEFT_EXT, HIGH);
        act_state = ACT_EXTENDING_LEFT;
        state_start_time = now;
        Serial.println("ACK:EXTENDING_LEFT");
    } else if (strcmp(cmd, "R_EXT") == 0) {
        // Enforce Mutual Exclusion: Left must be completely OFF
        stop_all_actuators();
        digitalWrite(PIN_RIGHT_EXT, HIGH);
        act_state = ACT_EXTENDING_RIGHT;
        state_start_time = now;
        Serial.println("ACK:EXTENDING_RIGHT");
    } else if (strcmp(cmd, "RETRACT_ALL") == 0) {
        stop_all_actuators();
        digitalWrite(PIN_LEFT_RET, HIGH);
        digitalWrite(PIN_RIGHT_RET, HIGH);
        act_state = ACT_RETRACTING;
        state_start_time = now;
        Serial.println("ACK:RETRACTING_ALL");
    } else {
        Serial.println("ERR:UNKNOWN_COMMAND");
    }
}
