# Control Scheduler

## Current Runtime State

- `main()` currently initializes system config, Bluetooth, OLED, attitude, ICM42688, then loops through Bluetooth processing, IMU read/update, VOFA output, OLED refresh, and `delay_ms(10)`. Evidence A: `empty.c`.
- `SysTick_Handler()` calls `delay_tick()` only. Evidence A: `empty.c`.
- `encoder_tick_1ms()`, `speed_pid_control_update()`, `angle_control_update()`, and `line_track_update()` exist but are not directly called by current `main()`. Evidence A: `empty.c`, `Inc/*.h`, `Src/*.c`.
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
