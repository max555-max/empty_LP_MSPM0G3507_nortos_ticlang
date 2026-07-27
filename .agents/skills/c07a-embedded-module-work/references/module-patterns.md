# Module Patterns

## Current Repository Shape

- Public headers live in `Inc/`. Evidence A.
- Implementations live in `Src/`. Evidence A.
- Entry point is `empty.c`. Evidence A.
- Generated configuration is included as `ti_msp_dl_config.h`. Evidence A.
- Current module style uses fixed-width integer types, `bool` for success/failure where useful, and small public APIs in headers. Evidence A.

## Common Patterns

- Initialization functions: `*_init()` usually reset internal state and configure runtime interrupt or GPIO state if needed.
- Foreground processing: `*_process()` or `*_update()` handles parsing, control, display, or sensor reads.
- ISR split: UART ISRs collect bytes into buffers; foreground code parses packets or commands.
- Error handling: drivers often return `bool` for initialization/read success or clear output structs on failure.
- Timing: blocking delays exist in initialization and some drivers; avoid adding them to ISRs or timing-critical loops.
- Generated macros: modules use generated names such as `UART_1_INST`, the legacy-named IMU bus macro `I2C_ICM42688_INST`, `PWM_INST`, and GPIO group macros instead of raw pins where available. The `I2C_ICM42688` instance name may still be used by MPU6050 code until the user explicitly approves a SysConfig rename.

## Current Examples

- `Src/bluetooth.c`: UART1 RX ISR writes a ring buffer; `bluetooth_process()` parses framed packets. Evidence A.
- `Src/uart_cmd.c`: UART0 RX ISR buffers a line; `uart_cmd_process()` parses it. Evidence A.
- `Src/mpu6050.c`: preferred IMU driver for future work; it reuses the existing legacy-named `I2C_ICM42688_INST` bus unless the user explicitly approves renaming SysConfig resources. Evidence A for module pattern when present.
- `Src/icm42688.c`: legacy IMU driver. Do not extend or select it for new IMU work unless the user explicitly requests ICM42688. Evidence A for module presence; Evidence B for legacy-only policy from user direction.
- `Src/oled.c`: GPIO bit-banged display output; foreground use only. Evidence A.
- `Src/gray_serial.c`: bit-banged clock/data read with short delay cycles. Evidence A.

## Rule

Do not invent alternative source directories, RTOS tasks, or board files that do not exist in the current repository.
