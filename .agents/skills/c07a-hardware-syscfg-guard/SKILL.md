---
name: c07a-hardware-syscfg-guard
description: Audit and guard C07A hardware-facing software configuration, SysConfig resources, generated pin/peripheral macros, and wiring-document consistency. Use when the user asks to query or review pin assignments; UART, I2C, PWM, GPIO, timer, interrupt, or peripheral instances; check whether empty.syscfg matches generated ti_msp_dl_config macros; check whether driver source uses current SysConfig macros correctly; compare software configuration, wiring notes, and user-described wiring for conflicts; plan pin or peripheral configuration changes; or confirm available peripherals and pins before adding a module. Do not use for PID tuning, control-loop algorithm analysis, ordinary project architecture summaries, module logic changes that do not touch peripheral configuration, build/release approval, flashing, or real-car testing.
---

# C07A Hardware SysConfig Guard

## Core Rules

1. Read and obey the repository root `AGENTS.md` first.
2. Treat `empty.syscfg` and generated `Debug/ti_msp_dl_config.*` as evidence of software configuration, not proof of physical wiring.
3. Do not reject current software configuration only because old wiring notes disagree.
4. Do not claim real-car wiring is correct from software configuration alone.
5. Mark important conclusions with evidence level A, B, or C.
6. Operate read-only by default.
7. If the user explicitly asks for changes, first output a change plan and affected resources; do not guess and edit directly.
8. Change pins, peripheral instances, or generated macros through the SysConfig source path. Do not hand-maintain generated files as the only source of truth.
9. Do not perform flashing, motor tests, or real-car validation from this Skill.
10. Do not describe build success as hardware configuration correctness.

## Workflow

1. Determine whether the user is asking about software configuration, physical wiring, or consistency between them.
2. Read `empty.syscfg`.
3. Read `Debug/ti_msp_dl_config.h` and `Debug/ti_msp_dl_config.c`.
4. Search related generated macros in `Inc/`, `Src/`, and `empty.c`.
5. Check peripheral instance, pin, direction, baud rate, timer channel, GPIO mode, and interrupt configuration consistency.
6. Compare wiring notes with software configuration and report conflicts.
7. Check whether one pin is assigned to multiple software functions.
8. Check whether source still uses deleted or renamed generated macros.
9. Separate software configuration facts, document records, user-confirmed wiring, and items requiring real-car confirmation.
10. Output conflicts, risks, recommendations, and human-confirmation items.

## References

- Read `references/pin-map.md` for the current software configuration pin/peripheral map.
- Read `references/syscfg-rules.md` when reviewing or planning SysConfig, generated macro, or peripheral-resource changes.
- Read `references/hardware-unknowns.md` when the user asks what cannot be proven from the repository.
- Read `references/configuration-conflicts.md` when comparing `empty.syscfg`, generated files, source macro usage, wiring notes, or user descriptions.

## Standard Output

Include the software configuration table, source macro usage, configuration conflicts, physical-wiring unknowns, a minimal change plan when changes are requested, human-confirmation items, and the files actually read.

## Boundaries

Do not create scripts from this Skill.
Do not modify `AGENTS.md`, existing Skills, source files, `empty.syscfg`, generated files, build files, or wiring notes unless a later user request explicitly changes the allowed scope.
Do not infer physical wiring, supply voltage, signal level, motor direction, encoder direction, IMU orientation, or real-car behavior without hardware evidence.
