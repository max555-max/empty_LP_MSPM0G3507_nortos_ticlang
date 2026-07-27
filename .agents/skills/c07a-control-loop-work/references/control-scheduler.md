# Control Scheduler

## Current Runtime State

- Re-read `empty.c` before using this section; it changes frequently during tests. User direction as of 2026-07-26: future IMU work should use MPU6050 only. If `main()` still calls `icm42688_*`, report it as current code and a migration mismatch, not as the desired future path. Evidence A for current calls; Evidence B for project direction.
- `SysTick_Handler()` timing duties must be checked from current `empty.c`; it may call only `delay_tick()` or may also call `encoder_tick_1ms()` in control tests. Evidence A after re-reading `empty.c`.
- `encoder_tick_1ms()`, `speed_pid_control_update()`, `angle_control_update()`, and `line_track_update()` may exist without being scheduled. Check current `main()` before describing any loop as active. Evidence A for source calls.
- `delay_ms(10)` does not prove a strict 10 ms control period because I2C reads, UART output, OLED refresh, retries, and blocking calls add time. Evidence B for timing impact; C for measured period.

## Dependency Order For Closed-Loop Driving

When the driving stack is connected, the intended dependency order should be explicit:

1. Encoder GPIO ISR updates counts.
2. `encoder_tick_1ms()` periodically updates speed feedback.
3. Sensor/attitude/gray data updates complete.
4. Angle loop or line loop writes speed targets if enabled.
5. `speed_pid_control_update()` reads feedback and writes final PWM.

The periods do not have to be numerically identical, but data dependencies, update order, and discrete PID parameters must match.

## Scheduling Rules

- A periodic function should have one owner and one time source.
- Do not call the same control update from both `main()` and an interrupt/timer without an explicit design.
- If a control period changes, re-check PID gains, derivative terms, encoder speed period, integral accumulation, and debug-output cost.
- Do not put OLED refresh, VOFA output, Bluetooth parsing, or I2C polling inside a timing-critical ISR.
