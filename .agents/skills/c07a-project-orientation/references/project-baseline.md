# C07A Project Baseline

This baseline records the current code-confirmed project state. Re-read `empty.c` before relying on it because `main()` is the source of truth for current runtime integration.

## Currently Initialized Or Called By `main()`

Evidence A, confirmed from current `empty.c`:

- Generated system configuration: `SYSCFG_DL_init()`.
- Bluetooth module: `bluetooth_init()` and `bluetooth_process()`.
- OLED module: `oled_init()`, line clearing, cursor setting, string printing, hex printing, float printing.
- Attitude module: `attitude_init()`, gyro calibration/update helpers, and yaw/euler accessors when called by `main()`.
- IMU module: re-read `empty.c` each time. User direction as of 2026-07-26 is MPU6050-only for future IMU work. If `empty.c` still calls `icm42688_*`, report those calls as current code facts and a migration mismatch, not as the desired future architecture.
- Delay module: `delay_ms()`, `delay_get_ms()`, and `delay_tick()` through `SysTick_Handler()`.
- VOFA module: `vofa_send_six_float()`.

Evidence A, confirmed from current `empty.c`, main-loop behavior:

- Processes Bluetooth commands every loop iteration.
- Retries the active IMU initialization when IMU status is false.
- Reads active IMU raw data when IMU status is true.
- Checks the active IMU `WHO_AM_I` value according to the driver currently called by `empty.c`.
- Updates attitude using measured `dt`, clamped to a default when outside the allowed range.
- Sends six floats to VOFA for IMU telemetry or failure status.
- Refreshes OLED every 250 ms when OLED initialization succeeded.
- Delays the loop by `IMU_UPDATE_PERIOD_MS`, currently `10U`.

## Present In Repository But Not Called By Current `main()`

Evidence A for repository presence and absence of direct calls from current `main()`:

- `angle_control`: `Src/angle_control.c`, `Inc/angle_control.h`.
- `encoder`: `Src/encoder.c`, `Inc/encoder.h`.
- `gray_serial`: `Src/gray_serial.c`, `Inc/gray_serial.h`.
- `line_track`: `Src/line_track.c`, `Inc/line_track.h`.
- `motor`: `Src/motor.c`, `Inc/motor.h`; `motor.h` is included by `empty.c`, but no motor function is directly called by current `main()`.
- `pid`: `Src/pid.c`, `Inc/pid.h`.
- `uart_cmd`: `Src/uart_cmd.c`, `Inc/uart_cmd.h`.

## Current Project Status

- Code-confirmed state: the current `main()` is an IMU, attitude, OLED, Bluetooth processing, and VOFA telemetry integration loop. Evidence A.
- Code-confirmed state: full motor closed-loop driving, encoder feedback, line tracking, angle control, gray serial reading, and UART command parsing are not directly connected by current `main()`. Evidence A.
- Hardware state: OLED, Bluetooth, MPU6050 physical wiring, motors, encoders, gray sensors, and real-car behavior are not proven by repository files alone. Evidence C unless the user provides fresh hardware confirmation.
- Project direction: future IMU code should use MPU6050 only. Evidence B from user instruction; verify source files before claiming it has been fully migrated.
- Validation state: code inspection and build artifacts can support software conclusions, but they do not prove real-car validation. Evidence C for real-car behavior.

## Baseline Update Rules

- Update this baseline after any change to `empty.c`, scheduler logic, interrupt dispatch, module initialization, or main-loop calls.
- Re-run `rg --files` before updating module presence.
- Re-read `empty.c` before changing the active and inactive module lists.
- Mark each updated statement with A, B, or C.
- Do not describe a module as active only because its files exist, its header is included, or SysConfig contains related pins.
- Do not describe build success as real-car validation.

## Important Distinction

Module exists does not mean module is currently running.

Evidence A can prove that a module file exists and that its functions compile into the project. Evidence A can also prove whether `main()` directly calls it. Actual runtime behavior still depends on initialization order, scheduler calls, interrupts, hardware state, and real-car validation.
