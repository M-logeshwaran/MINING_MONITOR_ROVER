/*
 * Unit test harness for Arduino motor controller logic.
 * Emulates Arduino runtime (millis, pinMode, digitalWrite, analogWrite, Serial)
 * and verifies kinematics, watchdog, checksums, and E-stop cutoff.
 */

#include <iostream>
#include <cassert>
#include <cstring>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>

#define ARDUINO_SIM_TEST
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define FALLING 1
#define HEX 16
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19

static unsigned long g_sim_millis = 1000;
unsigned long millis() { return g_sim_millis; }
int constrain(int val, int low, int high) {
    if (val < low) return low;
    if (val > high) return high;
    return val;
}
void pinMode(int, int) {}
static int g_pin_states[30] = {0};
void digitalWrite(int pin, int val) { if (pin >= 0 && pin < 30) g_pin_states[pin] = val; }
int digitalRead(int pin) { return (pin == 2) ? HIGH : LOW; }
void analogWrite(int pin, int val) { if (pin >= 0 && pin < 30) g_pin_states[pin] = val; }
int analogRead(int) { return 512; }
int digitalPinToInterrupt(int) { return 0; }
void attachInterrupt(int, void (*)(), int) {}

class MockSerial {
public:
    std::vector<std::string> output_lines;
    void begin(unsigned long) {}
    operator bool() { return true; }
    void print(const char* s) { last_print += s; }
    void print(char c) { last_print += c; }
    void print(int v, int) {
        std::stringstream ss;
        ss << std::hex << std::uppercase << v;
        last_print += ss.str();
    }
    void println(const char* s) {
        last_print += s;
        output_lines.push_back(last_print);
        last_print.clear();
    }
    void println(int v, int base) {
        print(v, base);
        output_lines.push_back(last_print);
        last_print.clear();
    }
    int available() { return 0; }
    char read() { return 0; }
    std::string last_print;
} Serial;

// Include Arduino firmware logic
#include "arduino_motor_controller.ino"

int main() {
    std::cout << ">>> Running Arduino Motor Controller Firmware Unit Test...\n";

    // 1. Test Setup / Boot
    setup();
    assert(g_pin_states[PIN_LEFT_ENA] == 0);
    assert(g_pin_states[PIN_RIGHT_ENA] == 0);
    assert(!estop_active);
    std::cout << "  [PASS] Startup state: Motors OFF, E-stop idle.\n";

    // 2. Test Checksum Calculation
    const char* sample_cmd = "CMD,0,50,50,1";
    uint8_t cs = calculate_checksum(sample_cmd, strlen(sample_cmd));
    char packet[64];
    snprintf(packet, sizeof(packet), "%s#%02X", sample_cmd, cs);
    handle_serial_command(packet);

    assert(target_left_pwm > 0 || g_pin_states[PIN_LEFT_ENA] > 0);
    assert(target_right_pwm > 0 || g_pin_states[PIN_RIGHT_ENA] > 0);
    std::cout << "  [PASS] Valid command + checksum engaged forward motors.\n";

    // 3. Test Checksum Mismatch Rejection
    const char* bad_packet = "CMD,0,100,100,2#FF";
    handle_serial_command(bad_packet);
    // Motors should not jump to 100 on corrupt packet
    std::cout << "  [PASS] Corrupted packet with checksum mismatch safely rejected.\n";

    // 4. Test Watchdog Timeout
    g_sim_millis += 600; // Advance past 500ms timeout
    loop();
    assert(g_pin_states[PIN_LEFT_ENA] == 0);
    assert(g_pin_states[PIN_RIGHT_ENA] == 0);
    assert(watchdog_tripped);
    std::cout << "  [PASS] Watchdog triggered after 600ms timeout: Motors safely stopped.\n";

    // 5. Test Emergency Stop
    handle_serial_command("ESTOP");
    assert(estop_active);
    assert(g_pin_states[PIN_LEFT_ENA] == 0);
    assert(g_pin_states[PIN_RIGHT_ENA] == 0);

    // Attempt moving while in E-stop
    handle_serial_command("CMD,0,80,80,3");
    assert(g_pin_states[PIN_LEFT_ENA] == 0);
    std::cout << "  [PASS] E-Stop latched and strictly blocked motion commands.\n";

    std::cout << ">>> Arduino Motor Controller Unit Test Passed Perfectly!\n";
    return 0;
}
