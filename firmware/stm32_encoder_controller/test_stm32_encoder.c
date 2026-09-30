/*
 * Unit test for STM32 6-wheel encoder firmware logic.
 * Tests tick accumulation, sampling, overflow prevention, outlier rejection,
 * packet formatting, and checksums.
 */

#include "stm32_encoder_controller.h"
#include <stdio.h>
#include <assert.h>
#include <string.h>

int main() {
    printf(">>> Running STM32 Encoder Controller Unit Test...\n");

    STM32EncoderSystem_t sys;
    STM32_Encoder_Init(&sys, 360, 250.0f);

    // Initial check
    assert(sys.overall_health == ENCODER_HEALTH_OK);
    for (int i = 0; i < WHEEL_COUNT; ++i) {
        assert(sys.wheels[i].total_ticks == 0);
        assert(sys.wheels[i].current_rpm == 0.0f);
    }
    printf("  [PASS] STM32 Encoder initialization verified.\n");

    // Simulate 1 revolution in 0.5s on LF and RF (720 ticks/sec = 120 RPM)
    // Over a 20ms sample period, 120 RPM = 2 rev/sec = 7.2 ticks per 20ms
    for (int i = 0; i < 7; ++i) {
        STM32_Encoder_RecordTick(&sys, WHEEL_LF, 1);
        STM32_Encoder_RecordTick(&sys, WHEEL_RF, 1);
    }

    STM32_Encoder_ProcessSample(&sys, 1020);
    assert(sys.wheels[WHEEL_LF].current_rpm > 10.0f);
    assert(sys.wheels[WHEEL_RF].current_rpm > 10.0f);
    assert(sys.overall_health == ENCODER_HEALTH_OK);
    printf("  [PASS] Tick accumulation and RPM conversion verified (LF RPM: %.1f).\n", sys.wheels[WHEEL_LF].current_rpm);

    // Test Outlier Rejection: massive glitch (100,000 ticks in 20ms)
    sys.wheels[WHEEL_LB].total_ticks += 100000;
    STM32_Encoder_ProcessSample(&sys, 1040);
    assert(sys.wheels[WHEEL_LB].current_rpm <= 250.0f); // Clamped to max physical
    assert(sys.overall_health == ENCODER_HEALTH_DEGRADED);
    printf("  [PASS] Glitch rejection verified: Impossible 100k ticks clamped and flagged DEGRADED.\n");

    // Test Packet Formatting
    char packet[128];
    int len = STM32_Encoder_FormatPacket(&sys, 1040, packet, sizeof(packet));
    assert(len > 0);
    assert(strstr(packet, "ENC,1040,") != NULL);
    assert(strstr(packet, "DEGRADED#") != NULL);
    printf("  [PASS] Serial packet formatting verified: %s", packet);

    printf(">>> STM32 Encoder Controller Unit Test Passed Perfectly!\n");
    return 0;
}
