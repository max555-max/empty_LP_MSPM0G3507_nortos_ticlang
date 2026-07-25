---
name: c07a-embedded-module-work
description: Implement, repair, or integrate a single C07A embedded module, including I2C, UART, GPIO, PWM, OLED, Bluetooth, IMU, gray sensor, VOFA, interrupt receive buffering, packet parsing, timeouts, public interfaces, new .h/.c files, and hooking a module into main() or a scheduler. Use when the user asks to add or fix one driver/module, expose a public API, connect initialization or foreground processing, repair UART buffering or parsing, or add a new source/header module. Do not use for PID tuning, pure pin/SysConfig changes, ordinary project summaries, release approval, broad refactors, flashing, or automatic real-car tests.
---

# C07A Embedded Module Work

## Core Rules

1. Read and obey repository root `AGENTS.md` first.
2. If external resources, pins, macros, or peripheral instances change, use the hardware SysConfig guard flow first.
3. After edits, use the verify-release flow for static checks or user-authorized builds.
4. Prefer the smallest module-scoped change.
5. Before editing, list target files, expected new files, public interfaces, call sites, SysConfig dependencies, interrupt paths, and blocking risks.
6. Do not refactor unrelated modules.
7. Do not add blocking work in ISRs.
8. If adding `.c`, check CCS build registration.
9. Public interface changes require searching all declarations, definitions, and call sites.
10. Do not treat compilation as module hardware validation.
11. Do not flash or test hardware without explicit authorization.

## Workflow

1. Define the target module and feature boundary.
2. Read existing module code and adjacent modules for style.
3. Search all related interfaces and call sites.
4. Confirm SysConfig macros and peripheral resources.
5. Confirm initialization location and foreground call location.
6. Split ISR duties from foreground parsing or slow work.
7. Confirm timeout, error return, retry, and disconnected-device behavior.
8. Check arrays, buffers, and length bounds.
9. Make the minimum edit.
10. Check header, implementation, and callers for consistency.
11. Check new source build registration.
12. Run allowed static checks or build flow.
13. Report edits, checks, unverified hardware facts, and real-car confirmation needs.

## References

- Read `references/module-patterns.md` before matching local source style.
- Read `references/isr-rules.md` before editing UART/GPIO/SysTick interrupt paths.
- Read `references/build-registration.md` when adding or renaming `.c` files.
- Read `references/api-change-checklist.md` before changing headers or public functions.
- Read `references/module-integration-checklist.md` before connecting a module into `main()` or a scheduler.

## Related Skills

Use `c07a-hardware-syscfg-guard` for pins/peripherals, `c07a-control-loop-work` for PID/control behavior, and `c07a-verify-release` for checks after implementation.

## Boundaries

Do not use this Skill for broad architecture summaries, PID tuning, release signoff, flashing, or hardware tests.
