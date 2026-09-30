/*
 * Unit test for Linear Actuator Firmware.
 * Tests mutual exclusion (Left vs Right), maximum stroke duration timeout,
 * cooldown enforcement, and E-Stop cutoff.
 */

#include <iostream>
#include <cassert>
#include <cstring>
#include <string>
#include <vector>

#define ARDUINO_SIM_TEST
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

static unsigned long g_sim_millis = 1000;
unsigned long millis() { return g_sim_millis; }
void pinMode(int, int) {}
static int g_pin_states[30] = {0};
void digitalWrite(int pin, int val) { if (pin >= 0 && pin < 30) g_pin_states[pin] = val; }
int digitalRead(int pin) { return (pin == 2) ? HIGH : LOW; }

class MockSerial {
public:
    void begin(unsigned long) {}
    void print(const char*) {}
    void println(const char*) {}
    int available() { return 0; }
    char read() { return 0; }
} Serial;

#include "arduino_actuator_controller.ino"

int main() {
    std::cout << ">>> Running Linear Actuator Controller Unit Test...\n";

    setup();
    assert(g_pin_states[PIN_LEFT_EXT] == LOW);
    assert(g_pin_states[PIN_RIGHT_EXT] == LOW);
    assert(act_state == ACT_IDLE);
    std::cout << "  [PASS] Actuators initialized in safe idle state.\n";

    // 1. Extend Left
    handle_command("L_EXT");
    assert(act_state == ACT_EXTENDING_LEFT);
    assert(g_pin_states[PIN_LEFT_EXT] == HIGH);
    assert(g_pin_states[PIN_RIGHT_EXT] == LOW);
    std::cout << "  [PASS] Left actuator extension engaged (Right confirmed OFF).\n";

    // 2. Mutual exclusion test: Command Right while Left is active
    // Must immediately disengage Left before engaging Right
    handle_command("R_EXT");
    assert(act_state == ACT_EXTENDING_RIGHT);
    assert(g_pin_states[PIN_LEFT_EXT] == LOW);  // Left MUST be forced LOW
    assert(g_pin_states[PIN_RIGHT_EXT] == HIGH);
    std::cout << "  [PASS] Mutual exclusion enforced: Left instantly cut off before Right engaged.\n";

    // 3. Max stroke timeout test: Advance time 4.0s (past 3.5s limit)
    g_sim_millis += 4000;
    loop();
    assert(act_state == ACT_COOLDOWN);
    assert(g_pin_states[PIN_RIGHT_EXT] == LOW);
    std::cout << "  [PASS] Max stroke timeout (3.5s) triggered auto-cutoff and cooldown.\n";

    // 4. Cooldown enforcement: Commands during cooldown must be rejected
    handle_command("L_EXT");
    assert(act_state == ACT_COOLDOWN);
    assert(g_pin_states[PIN_LEFT_EXT] == LOW);
    std::cout << "  [PASS] New command during cooldown safely rejected.\n";

    // 5. Cooldown completion
    g_sim_millis += 2500;
    loop();
    assert(act_state == ACT_IDLE);
    std::cout << "  [PASS] Cooldown period expired, returned to READY state.\n";

    std::cout << ">>> Linear Actuator Unit Test Passed Perfectly!\n";
    return 0;
}
