---
name: c07a-control-loop-work
description: Analyze, tune, or integrate C07A control loops including speed PID, encoder speed estimation, angle/yaw control, line-tracking PD, multi-loop composition, Kp/Ki/Kd changes, control periods, unit and scale checks, sign conventions, output limiting, integral saturation, derivative behavior, motor target ownership, PWM write order, and scheduler hookup. Use when the user asks about PID parameters, angle loop, yaw control, line tracking, encoder feedback, control timing, target speed mixing, actuator ownership, or control-module scheduling. Do not use for ordinary project summaries, pure pin/SysConfig questions, OLED/UART/I2C driver repair, build approval, flashing, or automatic real-car testing.
---

# C07A Control Loop Work

## Core Rules

1. Read and obey repository root `AGENTS.md` first.
2. If current runtime state matters, use or read `c07a-project-orientation`.
3. If PWM, encoder, IMU, gray-sensor, or peripheral configuration matters, use or read `c07a-hardware-syscfg-guard`.
4. After changes, verify through `c07a-verify-release` checks.
5. Compilation does not prove parameters are reasonable.
6. Do not raise target speed or PWM limits without explicit user authorization.
7. Make one patch solve one main control behavior unless the user requests a broader change.
8. Do not change Kp, Ki, Kd, and scheduling structure together unless explicitly requested.
9. For every parameter change, state physical meaning, stored value, scaled value, current period, reason, expected effect, side effects, and validation method.

## Workflow

1. Classify the task as speed loop, angle loop, line loop, encoder feedback, or combined control.
2. Read related `.h/.c` files and all call sites.
3. Confirm inputs, outputs, feedback, units, and scales.
4. Confirm error sign and left/right wheel mixing formula.
5. Confirm integral implementation, integral limit, and reset behavior.
6. Confirm derivative implementation, whether it divides by `dt`, and whether it uses error or feedback.
7. Confirm target period and actual scheduler entry.
8. Confirm encoder speed update occurs before speed PID if the speed loop is active.
9. Search all target-speed and PWM writers.
10. Name the final actuator owner.
11. Check whether angle loop and line loop can overwrite each other.
12. Before editing, give a minimal change plan.
13. After editing, run allowed verification checks or build flow.
14. Output safe real-car test steps, but do not execute them.

## References

- Read `references/control-units.md` for current units, scales, limits, and code-confirmed facts.
- Read `references/control-scheduler.md` when timing, `dt`, or task order matters.
- Read `references/actuator-ownership.md` before changing speed targets or motor output.
- Read `references/pid-discretization.md` before changing PID gains or periods.
- Read `references/tuning-safety.md` before recommending parameter changes or test steps.

## Related Skills

Use `c07a-project-orientation` for baseline runtime integration, `c07a-hardware-syscfg-guard` for hardware-facing configuration, and `c07a-verify-release` for checks after edits.

## Boundaries

Do not flash, start motors, or claim real-car behavior without user evidence. Do not use this Skill for generic driver repair or pure pin planning.
