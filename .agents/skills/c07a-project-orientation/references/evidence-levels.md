# Evidence Levels

Use evidence labels on every important conclusion.

## A: Directly Confirmed

Use A only when the claim is directly visible in repository files, including:

- Source files such as `empty.c` and `Src/*.c`.
- Public headers such as `Inc/*.h`.
- SysConfig input such as `empty.syscfg`.
- Generated TI configuration such as `Debug/ti_msp_dl_config.h` and `Debug/ti_msp_dl_config.c`.
- Build metadata such as `.cproject`, `.project`, generated makefiles, or linker command files.

Examples:

- MCU family or part number found in `.cproject` or `empty.syscfg`.
- A function call present in `main()`.
- A software pin assignment present in SysConfig or generated config.
- A public interface declared in a header file.

## B: Inferred From Code

Use B when the claim follows from code structure but still needs human confirmation.

Examples:

- A module appears intended for a hardware feature because of function names and comments.
- A scaling factor appears to represent a physical unit but is not documented at the call site.
- A module appears inactive because it is not called by the current `main()`, while it may be used by another build configuration.

Write B claims as assumptions or likely intent, not as confirmed hardware facts.

## C: Not Confirmable From Repository

Use C when the claim needs real hardware, schematic, sensor datasheet inspection, IDE state, flashing, or real-car testing.

Examples:

- Whether the real board wiring matches SysConfig.
- Whether a motor direction sign matches forward motion.
- Whether an IMU orientation is mechanically correct.
- Whether PID parameters are stable on the car.
- Whether an OLED, Bluetooth module, encoder, or gray sensor works on the physical vehicle.

## Required Wording

- Say "代码可确认" or "A" for direct evidence.
- Say "根据代码推测，需人工确认" or "B" for inferred claims.
- Say "仓库无法确认，需实车/原理图/开发环境验证" or "C" for unknowns.
- Never describe "build passed" as "real-car validated".
- Never convert B or C into definite fact in summaries.
