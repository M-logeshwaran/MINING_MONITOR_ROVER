/*
 * ============================================================
 * DRILLPULSE — STM32 6-Wheel Encoder Controller Implementation
 * ============================================================
 */

#include "stm32_encoder_controller.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

void STM32_Encoder_Init(STM32EncoderSystem_t* sys, uint32_t ticks_per_rev, float max_rpm) {
    if (!sys) return;
    sys->ticks_per_rev = (ticks_per_rev > 0) ? ticks_per_rev : 360;
    sys->max_physical_rpm = (max_rpm > 0.0f) ? max_rpm : 250.0f; // 250 RPM ~ 2.8 m/s on 22cm wheel
    sys->sample_interval_ms = 20; // 50 Hz
    sys->overall_health = ENCODER_HEALTH_OK;

    for (int i = 0; i < WHEEL_COUNT; ++i) {
        sys->wheels[i].total_ticks = 0;
        sys->wheels[i].last_ticks = 0;
        sys->wheels[i].current_rpm = 0.0f;
        sys->wheels[i].last_pulse_time_ms = 0;
        sys->wheels[i].valid = true;
    }
}

void STM32_Encoder_RecordTick(STM32EncoderSystem_t* sys, WheelIndex_t wheel, int8_t direction) {
    if (!sys || wheel >= WHEEL_COUNT) return;
    if (direction >= 0) {
        sys->wheels[wheel].total_ticks++;
    } else {
        sys->wheels[wheel].total_ticks--;
    }
}

void STM32_Encoder_ProcessSample(STM32EncoderSystem_t* sys, uint32_t current_time_ms) {
    if (!sys) return;

    float dt_sec = (float)sys->sample_interval_ms / 1000.0f;
    if (dt_sec <= 0.001f) dt_sec = 0.02f;

    int degraded_count = 0;
    int timeout_count = 0;

    for (int i = 0; i < WHEEL_COUNT; ++i) {
        int32_t current_ticks = sys->wheels[i].total_ticks;
        int32_t delta_ticks = current_ticks - sys->wheels[i].last_ticks;
        sys->wheels[i].last_ticks = current_ticks;

        // RPM = (delta_ticks / PPR) * (60 / dt)
        float revs = (float)delta_ticks / (float)sys->ticks_per_rev;
        float raw_rpm = (revs / dt_sec) * 60.0f;

        // Glitch rejection: impossible physics check
        if (fabsf(raw_rpm) > sys->max_physical_rpm) {
            sys->wheels[i].valid = false;
            degraded_count++;
            // Clamp to max physical
            sys->wheels[i].current_rpm = (raw_rpm > 0.0f) ? sys->max_physical_rpm : -sys->max_physical_rpm;
        } else {
            sys->wheels[i].valid = true;
            // Low-pass exponential smoothing
            sys->wheels[i].current_rpm = (0.7f * sys->wheels[i].current_rpm) + (0.3f * raw_rpm);
        }
    }

    if (degraded_count >= 3) {
        sys->overall_health = ENCODER_HEALTH_INVALID;
    } else if (degraded_count > 0) {
        sys->overall_health = ENCODER_HEALTH_DEGRADED;
    } else {
        sys->overall_health = ENCODER_HEALTH_OK;
    }
}

uint8_t STM32_Encoder_Checksum(const char* data, int len) {
    uint8_t cs = 0;
    for (int i = 0; i < len; ++i) {
        cs ^= (uint8_t)data[i];
    }
    return cs;
}

int STM32_Encoder_FormatPacket(const STM32EncoderSystem_t* sys, uint32_t timestamp_ms, char* out_buf, int max_len) {
    if (!sys || !out_buf || max_len < 64) return -1;

    const char* health_str = "OK";
    if (sys->overall_health == ENCODER_HEALTH_DEGRADED) health_str = "DEGRADED";
    else if (sys->overall_health == ENCODER_HEALTH_TIMEOUT) health_str = "TIMEOUT";
    else if (sys->overall_health == ENCODER_HEALTH_INVALID) health_str = "INVALID";

    char payload[96];
    int len = snprintf(payload, sizeof(payload),
                       "ENC,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%s",
                       timestamp_ms,
                       sys->wheels[WHEEL_LF].current_rpm,
                       sys->wheels[WHEEL_RF].current_rpm,
                       sys->wheels[WHEEL_LM].current_rpm,
                       sys->wheels[WHEEL_RM].current_rpm,
                       sys->wheels[WHEEL_LB].current_rpm,
                       sys->wheels[WHEEL_RB].current_rpm,
                       health_str);

    uint8_t cs = STM32_Encoder_Checksum(payload, len);
    int total = snprintf(out_buf, max_len, "%s#%02X\n", payload, cs);
    return total;
}
