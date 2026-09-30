# DrillPulse STM32 Embedded Firmware

This directory contains ARM Cortex-M firmware for high-frequency wheel encoder pulse counting and glitch rejection.

## Directory Structure & Modules

### `encoder/` (STM32F401RE Nucleo - Current SIH Architecture)
- **Files**: `stm32_encoder_controller.c`, `stm32_encoder_controller.h`
- **Unit Test**: `test_stm32_encoder.c`
- **Target Hardware**: STM32F401RE Nucleo-64 (ARM Cortex-M4 @ 84 MHz)
- **Role**: Captures quadrature pulse trains from 6x optical encoders mounted to all 6 rover wheels (1040 PPR per wheel).
- **Key Features**:
  - Digital consensus glitch filter rejecting electrical brush noise pulses shorter than 10 microseconds.
  - 50 Hz deterministic binary telemetry packet output over USART2 DMA (115200 baud).
  - Compact 29-byte binary packet with Little-endian int32 tick counts, XOR checksum, and CRLF framing:
    `[0xAA 0x55] [FL:4] [FR:4] [ML:4] [MR:4] [RL:4] [RR:4] [Checksum:1] [\r\n]`

## Build and Testing
Can be built with STM32CubeIDE, GCC ARM Embedded toolchain (`arm-none-eabi-gcc`), or verified using the included host C11 unit test harness:
```bash
gcc -std=c11 -Wall -Wextra encoder/test_stm32_encoder.c encoder/stm32_encoder_controller.c -o /tmp/test_encoder && /tmp/test_encoder
```
