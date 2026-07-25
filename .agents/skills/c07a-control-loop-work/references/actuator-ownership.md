# Actuator Ownership

## Current Writers

- `motor_set_pwm()` is defined in `Src/motor.c`. Evidence A.
- `motor_set_pwm()` is called by `speed_pid_init()`, `speed_pid_stop()`, and `speed_pid_control_update()` in `Src/pid.c`. Evidence A.
- `speed_pid_set_speed()` is called by angle control and line tracking modules. Evidence A: `Src/angle_control.c`, `Src/line_track.c`.
- `speed_pid_set_target()` is called by `Src/pid.c` internal wrappers and `Src/uart_cmd.c`. Evidence A.
- Current `main()` does not call `motor_set_pwm()`, speed PID update, angle update, line update, or encoder update directly. Evidence A: `empty.c`.

## Ownership Model

- Bottom layer: `motor_set_pwm()` writes direction GPIO and PWM compare values.
- Speed loop: `speed_pid_control_update()` should be the final normal owner of motor PWM during closed-loop driving.
- Upper loops: angle and line modules should produce speed targets or corrections, not direct PWM, unless a new explicit arbitration design is created.
- Bluetooth and UART command modules can change parameters or targets, but active use depends on current initialization and process calls.

## Risks

- Angle loop and line loop both write speed targets through `speed_pid_set_speed()`. If both are scheduled in the same period without arbitration, later calls can overwrite earlier targets. Evidence A for writer locations; B for runtime conflict unless both are scheduled.
- `uart_cmd` and line/angle loops can also write targets if active together. Evidence A for code paths; B for runtime conflict.
- Direct new `motor_set_pwm()` calls bypass speed-loop ownership and require explicit design review.

## Required Check Before Control Changes

Run or reproduce `c07a-verify-release/scripts/scan_control_writers.py` and report all writers before changing actuator logic.
