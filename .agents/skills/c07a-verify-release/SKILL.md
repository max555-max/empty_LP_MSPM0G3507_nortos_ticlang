---
name: c07a-verify-release
description: Verify C07A repository readiness through read-only static checks, optional user-authorized Debug builds, source registration checks, ISR blocking scans, pinmap consistency checks, control writer scans, pre-flash review, and competition release review. Use when the user asks to compile, build, run static checks, check whether new sources are registered, inspect ISR blocking risks, compare SysConfig/generated/source pin usage, scan motor or speed-target writers, prepare a burn-before checklist, review a competition version, or clarify the reached validation level. Do not use for feature development, PID tuning, control algorithm changes, pin changes, driver implementation, automatic flashing, automatic motor tests, or real-car pass claims.
---

# C07A Verify Release

## Core Rules

1. Read and obey repository root `AGENTS.md` first.
2. Protect user changes; inspect `git status --short` before reporting or running checks.
3. Default to read-only checks.
4. Explain that a build can update `Debug/` outputs before any build command is run.
5. Do not build unless the user explicitly allows it for the current turn.
6. Do not flash, start motors, run lifted-wheel tests, or run real-car tests.
7. Separate validation levels: file reading, static checks, build passed, hardware communication, lifted-wheel validation, low-speed ground validation, and full competition validation.
8. Never describe build success as real-car success.
9. Scripts report findings only; they do not modify files.
10. Release reports must keep unresolved risks and required human confirmations visible.

## Workflow

1. Check workspace status.
2. Identify current build system, compiler, and Debug build entry from `.project`, `.cproject`, `.ccsproject`, `Debug/makefile`, and generated make metadata.
3. Check whether every current `Src/*.c` file is registered in CCS Debug metadata.
4. Check public headers and implementations for obvious interface drift when relevant.
5. Scan ISR bodies for blocking or high-latency calls.
6. Check `empty.syscfg`, generated `Debug/ti_msp_dl_config.*`, and source macro references.
7. Scan all `motor_set_pwm`, `speed_pid_set_speed`, `speed_pid_set_target`, control-update functions, and interrupt handlers.
8. If the user allowed a build, run the Debug build plan with `scripts/build_debug.ps1 -Run`.
9. Collect warnings, errors, changed build products, and script exit codes.
10. Output validation level, checks actually run, residual risks, and real-car confirmation items.

## References

- Read `references/build-environment.md` before planning or running a build.
- Read `references/static-checks.md` before selecting scripts or writing a static-check report.
- Read `references/release-checklist.md` for pre-flash or competition review.
- Read `references/validation-levels.md` before naming the achieved validation level.

## Scripts

- `scripts/build_debug.ps1`: show or run the Debug build plan; default is plan-only.
- `scripts/check_source_registered.py`: compare `Src/*.c` with CCS Debug source lists.
- `scripts/check_isr_blocking.py`: scan interrupt handlers for blocking or high-latency calls.
- `scripts/check_pinmap_consistency.py`: compare software config, generated macros, source usage, and wiring notes.
- `scripts/scan_control_writers.py`: list motor PWM and speed-target writer locations.
- `scripts/check_skill_integrity.py`: validate repository-level Skill structure.

## Output

Report files read, scripts run, validation level reached, generated artifacts changed or not checked, failures/warnings, residual risks, and whether real-car confirmation is still required.

## Boundaries

Do not modify source, SysConfig, generated files, build files, wiring notes, or existing Skills from this Skill unless a later request explicitly changes the allowed scope. Do not use this Skill as a substitute for control-loop analysis, driver implementation, pin planning, flashing, or hardware testing.
