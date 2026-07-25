# C07A Data Flow

Use this reference when tracing current behavior. Re-check source files before final answers.

## Read-Only Trace Method

1. Start at `main()` in `empty.c`.
2. List initialization calls in order.
3. List repeated calls in the main loop in order.
4. For each repeated call, trace its public interface into source files.
5. Identify every writer of motor PWM, speed targets, PID parameters, display buffer, UART output, and shared sensor state.
6. Mark whether the call path is active in the current build. Use evidence A only when code confirms it.

## Sensor Input Paths

- ICM42688 path: I2C read functions feed raw accel/gyro/temp data into attitude estimation and debug/display output. Evidence A when confirmed in `empty.c`, `Src/icm42688.c`, `Inc/icm42688.h`, `Src/attitude.c`, and `Inc/attitude.h`.
- Encoder path: GPIO interrupt counting feeds speed calculation, then speed PID feedback. Evidence A for module behavior when confirmed in `Src/encoder.c`, `Inc/encoder.h`, `Src/pid.c`, and `Inc/pid.h`; active runtime use depends on `main()` and interrupt configuration.
- Gray-sensor path: GPIO or bit-banged serial gray data feeds line-tracking error calculation. Evidence A for module behavior when confirmed in `Src/line_track.c`, `Inc/line_track.h`, `Src/gray_serial.c`, and `Inc/gray_serial.h`; active runtime use depends on calls from `main()` or other scheduler code.
- Bluetooth path: UART1 receive ISR fills a buffer, while foreground processing parses commands and updates runtime parameters. Evidence A when confirmed in `Src/bluetooth.c` and `Inc/bluetooth.h`.
- UART0 debug path: VOFA telemetry and UART command parsing may share UART0 if both are enabled. Evidence A for module presence; active conflict risk depends on current call paths.

## Control And Output Paths

- Attitude estimation updates roll, pitch, yaw, and gyro-derived values. Evidence A when confirmed in `Src/attitude.c` and `Inc/attitude.h`.
- Angle control computes yaw error and writes left/right target speed through `speed_pid_set_speed`. Evidence A for module behavior; active use depends on scheduler calls.
- Line tracking computes gray-sensor error and writes left/right target speed through `speed_pid_set_speed`. Evidence A for module behavior; active use depends on scheduler calls.
- Speed PID reads encoder feedback and writes motor commands through motor output helpers. Evidence A when confirmed in `Src/pid.c`, `Inc/pid.h`, `Src/motor.c`, and `Inc/motor.h`.
- Motor output converts signed speed commands into direction GPIO and PWM duty. Evidence A when confirmed in `Src/motor.c`, `Inc/motor.h`, and generated config.
- OLED output writes to a software display buffer and refreshes the display through GPIO bit-banging. Evidence A when confirmed in `Src/oled.c` and `Inc/oled.h`.

## Timing And Concurrency Checks

- Do not assume a precise control period from `delay_ms(10)` alone. Blocking I2C, OLED refresh, UART sending, or retries can change loop timing. Evidence B unless measured or tightly bounded by code.
- Check whether SysTick or timer interrupts update timing state.
- Check GPIO and UART interrupt handlers for shared variables with foreground code.
- Check whether shared variables are `volatile`, protected, copied atomically, or only read/written in one context.
- Check for duplicate writers to motor output or speed targets before describing a control path as deterministic.

## Risk Patterns To Report

- A module exists but is not called by current `main()`.
- A control loop has parameters scaled by integer factors, such as PID gains stored in thousandths.
- Multiple modules write the same control target without an explicit arbiter.
- Blocking display, telemetry, sensor retry, or delay code sits in a loop expected to be periodic.
- A command parser and telemetry sender share the same UART.
- Array writes, packet parsers, or display functions accept external length or index inputs.
- Motor output sign conventions and IMU orientation cannot be confirmed without real-car validation.
