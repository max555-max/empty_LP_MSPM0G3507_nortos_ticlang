# C07A Project Map

This reference records repository facts useful for read-only orientation. Re-check files before final answers because the project may change.

## Confirmed Project Identity

- MCU: MSPM0G3507, confirmed by `.cproject` and `empty.syscfg`. Evidence A.
- SDK in `.cproject`: `MSPM0-SDK:2.10.0.04`, from the `PRODUCTS` list option. Evidence A.
- SDK in `empty.syscfg`: `mspm0_sdk@2.10.00.04`, from `@cliArgs` and `@v2CliArgs`. Evidence A.
- SysConfig tool in `.cproject`: `sysconfig:1.27.0`, from the `PRODUCTS` list option. Evidence A.
- SysConfig tool in `empty.syscfg`: `1.27.0+4565`, from `@versions`. Evidence A.
- SysConfig input: `empty.syscfg`, with generated output under `Debug/ti_msp_dl_config.*`. Evidence A.
- Compiler: `TICLANG_4.0.4.LTS`, confirmed by `.cproject`. Evidence A.
- Build system: Code Composer Studio / SysConfig generated make project. Evidence A.
- Main entry: `empty.c` and its `main()` function. Evidence A.

## Orientation File Reading Order

1. `empty.c`
2. `empty.syscfg`
3. `Debug/ti_msp_dl_config.h`
4. `Debug/ti_msp_dl_config.c`
5. Public headers in `Inc/*.h`
6. Matching source files in `Src/*.c`
7. Wiring notes only after software configuration and current user or hardware evidence have been checked

Treat old wiring notes as secondary references. They may be outdated and cannot override current schematic, hardware inspection, or real-car testing.

## Module Map

- `delay`: millisecond delay, SysTick tick handling, and elapsed time access. Evidence A.
- `icm42688`: I2C access to ICM42688, WHO_AM_I check, raw accel/gyro/temp reads. Evidence A.
- `attitude`: gyro calibration and attitude estimation from IMU data. Evidence A.
- `angle_control`: yaw-angle control that writes speed targets through the speed PID interface. Evidence A.
- `encoder`: encoder GPIO interrupt counting and wheel speed calculation. Evidence A.
- `pid`: generic PID helpers and speed PID control output to motors. Evidence A.
- `motor`: signed motor direction and PWM output helpers. Evidence A.
- `line_track`: gray-sensor line tracking control that writes speed targets through the speed PID interface. Evidence A.
- `gray_serial`: bit-banged gray-sensor serial reading and debug print helpers. Evidence A.
- `oled`: bit-banged OLED display initialization, text, number, and refresh helpers. Evidence A.
- `bluetooth`: UART1 receive buffer and command processing for runtime parameter adjustment. Evidence A.
- `vofa`: UART0 FireWater float telemetry. Evidence A.
- `uart_cmd`: UART command parser for runtime commands, present in the repository but not necessarily active in current `main()`. Evidence A for presence, B for intended use.

## Current Main Behavior Pattern

The current `main()` should be inspected directly each time. At the time this reference was written, it initializes system config, Bluetooth, OLED, attitude, and ICM42688; calibrates gyro; loops through Bluetooth processing, IMU retry/read, attitude update, VOFA telemetry, OLED refresh, and a delay. Evidence A when re-confirmed in `empty.c`.

Modules may exist without being called by the current `main()`. Do not describe them as active runtime behavior unless call paths confirm that.

## Software Configuration And Physical Wiring

Separate software configuration facts from physical wiring facts.

- SysConfig and generated macros prove the software peripheral and pin configuration used by the project. Evidence A for software configuration.
- SysConfig and generated macros do not, by themselves, prove the board is physically wired that way. Physical wiring remains Evidence C unless confirmed by the user, current schematic, direct hardware inspection, or real-car testing.
- User confirmation, current schematic, hardware inspection, and real-car testing are the evidence sources for physical wiring.
- Old wiring notes may be useful hints, but they may be outdated. Treat them as Evidence B unless confirmed by current hardware evidence.
- When software configuration conflicts with old wiring notes, report the conflict instead of silently choosing one as physical truth.

Known current software mappings must be re-checked before use. Previously observed mappings included I2C0 for ICM42688, UART0 for debug telemetry, UART1 on PB6/PB7 for Bluetooth, PWM and GPIO outputs for motors, GPIO inputs for encoders and gray sensors, and OLED software pins including PA28 and PA31. Treat these as software-configuration facts only after re-checking current SysConfig/generated files.
