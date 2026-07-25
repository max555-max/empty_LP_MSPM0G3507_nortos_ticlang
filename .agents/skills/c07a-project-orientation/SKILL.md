---
name: c07a-project-orientation
description: Understand and audit the C07A MSPM0G3507 smart-car repository without modifying files. Use when the user asks to explain the project, identify entry points, map modules, trace data flow, classify facts by evidence level, check what has been integrated in the current main program, or prepare context for later development. Do not use for code changes, pin changes, PID tuning, build execution, flashing, or real-car validation.
---

# C07A Project Orientation

## Core Rule

Operate read-only unless the user explicitly asks for edits in a later turn.

## Evidence Levels

- A: directly confirmed by code, SysConfig, generated config, or build files.
- B: inferred from code patterns and requires human confirmation.
- C: cannot be confirmed from repository; requires hardware, schematic, IDE, or real-car validation.

## Workflow

1. Inspect repository layout with `rg --files`.
2. Identify MCU, SDK, compiler, build system, and entry point.
3. Read `empty.c`, `empty.syscfg`, `Debug/ti_msp_dl_config.h`, public headers, and relevant source files.
4. Build a module map with each module's public interface and ownership.
5. Build a data-flow map from sensors to control logic, actuator output, display, and debug communication.
6. Identify modules that exist but are not called by the current `main()`.
7. Mark every conclusion with evidence level A, B, or C.

## References

- Read `references/evidence-levels.md` before classifying facts or writing audit conclusions.
- Read `references/project-map.md` when module ownership, entry points, build system, or files matter.
- Read `references/data-flow.md` when tracing sensor-control-output paths or main-loop behavior.
- Read `references/project-baseline.md` when the user asks what is currently complete, which modules are connected in `main()`, or which functions are still experimental or code-only.

## Output

Return concise architecture notes, module list, data flow, uncertainty list, and risks.

## Boundaries

Do not edit files.
Do not infer physical wiring from SysConfig alone.
Do not infer hardware wiring from old docs when SysConfig disagrees.
Do not run flashing, motor, or real-car validation commands.
Do not use this Skill for implementation, PID tuning, pin changes, or release approval.
