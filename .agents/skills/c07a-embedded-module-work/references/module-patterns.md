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
- Generated macros: modules use generated names such as `UART_1_INST`, `I2C_ICM42688_INST`, `PWM_INST`, and GPIO group macros instead of raw pins where available.

## Current Examples

- `Src/bluetooth.c`: UART1 RX ISR writes a ring buffer; `bluetooth_process()` parses framed packets. Evidence A.
- `Src/uart_cmd.c`: UART0 RX ISR buffers a line; `uart_cmd_process()` parses it. Evidence A.
- `Src/icm42688.c`: I2C driver has timeouts, address probing, and raw-data clearing on failure. Evidence A.
- `Src/oled.c`: GPIO bit-banged display output; foreground use only. Evidence A.
- `Src/gray_serial.c`: bit-banged clock/data read with short delay cycles. Evidence A.

## Rule

Do not invent alternative source directories, RTOS tasks, or board files that do not exist in the current repository.
