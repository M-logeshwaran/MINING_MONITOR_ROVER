/*
 * ============================================================
 * DRILLPULSE — STM32 6-Wheel Encoder Controller
 * Target: STM32F4 / STM32F1 (ARM Cortex-M)
 *
 * Responsibilities:
 *   1. Hardware Timer & EXTI Interrupt counting for 6 wheels:
 *      - LF (Left Front), LM (Left Mid), LB (Left Back)
 *      - RF (Right Front), RM (Right Mid), RB (Right Back)
 *   2. Overflow-safe 32-bit tick delta calculation
 *   3. Outlier and glitch rejection (Max acceleration / speed gating)
 *   4. Transmission of signed RPM values to Raspberry Pi over UART
 *   5. Health status monitoring: OK, DEGRADED, TIMEOUT, INVALID
 * ============================================================
 */

#ifndef STM32_ENCODER_CONTROLLER_H
#define STM32_ENCODER_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WHEEL_COUNT 6

typedef enum {
    WHEEL_LF = 0,
    WHEEL_RF = 1,
    WHEEL_LM = 2,
    WHEEL_RM = 3,
    WHEEL_LB = 4,
    WHEEL_RB = 5
} WheelIndex_t;

typedef enum {
    ENCODER_HEALTH_OK = 0,
    ENCODER_HEALTH_DEGRADED = 1,
    ENCODER_HEALTH_TIMEOUT = 2,
    ENCODER_HEALTH_INVALID = 3
} EncoderHealth_t;

typedef struct {
    volatile int32_t total_ticks;
    int32_t last_ticks;
    float current_rpm;
    uint32_t last_pulse_time_ms;
    bool valid;
} WheelEncoderState_t;

typedef struct {
    WheelEncoderState_t wheels[WHEEL_COUNT];
    uint32_t sample_interval_ms;
    uint32_t ticks_per_rev;
    float max_physical_rpm;
    EncoderHealth_t overall_health;
} STM32EncoderSystem_t;

void STM32_Encoder_Init(STM32EncoderSystem_t* sys, uint32_t ticks_per_rev, float max_rpm);
void STM32_Encoder_RecordTick(STM32EncoderSystem_t* sys, WheelIndex_t wheel, int8_t direction);
void STM32_Encoder_ProcessSample(STM32EncoderSystem_t* sys, uint32_t current_time_ms);
int  STM32_Encoder_FormatPacket(const STM32EncoderSystem_t* sys, uint32_t timestamp_ms, char* out_buf, int max_len);
uint8_t STM32_Encoder_Checksum(const char* data, int len);

#ifdef __cplusplus
}
#endif

#endif // STM32_ENCODER_CONTROLLER_H
