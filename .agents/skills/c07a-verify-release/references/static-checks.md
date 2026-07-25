# Static Checks

Use these checks before release reports and after source changes when the user permits checks.

## Checks

- Source registration: every current `Src/*.c` should appear in `Debug/Src/subdir_vars.mk`. Evidence A when the script confirms it.
- Interface consistency: public declarations in `Inc/*.h` should match definitions in `Src/*.c`; modified interfaces require searching all call sites.
- ISR blocking: interrupt handlers should not call `delay_ms`, OLED display routines, blocking UART send, I2C polling, VOFA output, or complex parsing.
- Generated macro validity: source files should not reference deleted or renamed SysConfig macros.
- Duplicate handlers: only one definition should exist for each generated interrupt handler name.
- Bounds: packet, UART, OLED, and parser buffers should be checked for index limits and null termination.
- Integer range: PID, encoder conversion, PWM limits, and scaled gains should be checked for overflow or truncation.
- UART sharing: UART0 can be used by VOFA and `uart_cmd`; active use must be checked from current `main()` or scheduler.
- Control writers: scan `motor_set_pwm`, `speed_pid_set_speed`, and `speed_pid_set_target` before changing control ownership.
- Blocking period effects: foreground OLED, VOFA, Bluetooth replies, I2C polling, and `delay_ms` can change loop timing.

## Current High-Value Searches

- `motor_set_pwm`
- `speed_pid_set_speed`
- `speed_pid_set_target`
- `speed_pid_control_update`
- `angle_control_update`
- `line_track_update`
- `IRQHandler`
- `delay_ms`
- `DL_UART_Main_transmitDataBlocking`

## Evidence Labels

Script findings are Evidence A for file contents found by the script. Runtime impact remains Evidence B unless the active call path is confirmed. Hardware behavior remains Evidence C without user-provided hardware evidence.
