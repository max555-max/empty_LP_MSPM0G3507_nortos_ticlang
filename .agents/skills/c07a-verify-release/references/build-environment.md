# Build Environment

Re-read project metadata before each verification because versions and generated files can change.

## Current Code-Confirmed Facts

- Project type: Code Composer Studio managed C project with TI CCS nature and CDT managed build nature. Evidence A: `.project`, `.ccsproject`, `.cproject`.
- MCU family/target: MSPM0G3507 / MSPM0G350X configuration. Evidence A: `.cproject`, `empty.syscfg`, `Debug/ti_msp_dl_config.h`.
- Compiler family: TI ARM Clang/TICLANG. Evidence A: `.cproject`, `Debug/Src/subdir_rules.mk`.
- Build configuration: `Debug`, generated GNU make metadata under `Debug/`. Evidence A: `.cproject`, `Debug/makefile`, `Debug/subdir_vars.mk`, `Debug/Src/subdir_vars.mk`.
- SysConfig input: `empty.syscfg`; generated C/H files are `Debug/ti_msp_dl_config.c` and `Debug/ti_msp_dl_config.h`. Evidence A.
- Current Debug source list includes `empty.c`, generated config, startup file, and current `Src/*.c` modules listed in `Debug/Src/subdir_vars.mk`. Evidence A.

## Version Handling

Do not hard-code version facts in long-lived rules. For reports, re-read:

- `.ccsproject` for CCS project metadata.
- `.cproject` for device, products, compiler, include paths, and toolchain options.
- `empty.syscfg` for SysConfig product/tool annotations.
- `Debug/*subdir*.mk` for generated build commands and source registration.

## Build Entry

- The expected Debug build entry is `Debug/makefile`. Evidence A.
- `scripts/build_debug.ps1` defaults to showing the build plan and does not build unless called with `-Run`.
- A build can update `Debug/` outputs including `.o`, `.d`, `.out`, `.map`, link info, generated config, and clangd cache files. Evidence B from current repository layout and generated outputs.

## Reporting Rule

Use "build passed" only for compiler/linker success. Do not call it hardware communication, lifted-wheel validation, ground validation, or competition validation.
