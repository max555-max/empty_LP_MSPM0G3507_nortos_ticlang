# Configuration Conflicts

This file records current inconsistencies observed between software configuration, generated files, source macro usage, and wiring notes. Do not automatically modify conflicts.

## Summary

- `empty.syscfg` and `Debug/ti_msp_dl_config.h/.c` are consistent for the checked current software resources. Evidence A.
- Source macro usage was found for current generated macros in the relevant modules. No missing generated macro usage was observed by `rg` search. Evidence A.
- `接线说明.md` contains multiple mappings that conflict with current SysConfig and generated macros. Treat the document as outdated until the user confirms otherwise. Evidence B for document staleness, C for real wiring.

## Conflict Table

| Conflict object | Current software configuration | Document or other source content | Evidence level | Recommended handling | Needs user or real-car confirmation |
|---|---|---|---|---|---|
| Motor PWM pins | PWM uses `TIMG6`: C0 PB2, C1 PB3; direction GPIOs are AIN1 PA14, AIN2 PA13, BIN1 PA16, BIN2 PA17 | `接线说明.md` says left PWM PA26, left direction PA24/PA25, right PWM PA27, right direction PB25/PB24 | A for software config; B for old document claim; C for real wiring | Do not change code automatically. Ask user whether wiring notes should be updated or hardware was rewired. | Yes |
| Encoder pins | E1A PA25, E1B PA26, E2A PB20, E2B PB24, rise/fall interrupts | `接线说明.md` says right PA14/PA15 and left PA16/PA17. `Inc/encoder.h` comment says left E2A/E2B PA16/PA17 and right E1A/E1B PA14/PA15 | A for software config and header comment content; B for comment/document intent; C for physical wheel mapping | Treat generated macros as current software truth. Fix comments/docs only if user allows. Confirm physical left/right before changing direction logic. | Yes |
| Encoder left/right naming | `Src/encoder.c` maps left to generated E1 pins and right to generated E2 pins | Generated names do not encode physical wheel side, while comments/documents disagree on wheel side | A for source mapping; C for physical wheel assignment | Report ambiguity. Do not rename pins or swap logic without hardware confirmation. | Yes |
| Gray sensor pins | DAT PA12, CLK PB16 | `接线说明.md` says DAT PA22, CLK PB20 | A for software config; B for old document claim; C for physical wiring | Use current SysConfig for software analysis. Confirm real wiring before test or doc update. | Yes |
| Gray active level | `Inc/line_track.h` sets `LINE_TRACK_ACTIVE_LEVEL (1U)` | `接线说明.md` says current active level is `0U` | A for current header; B for old document claim; C for actual sensor polarity | Do not change polarity from the document alone. Confirm raw values on white/black surface before changing. | Yes |
| ICM42688 bus | Current software uses hardware I2C0 on PA0 SDA and PA1 SCL, bus speed 400 kHz, auto-tries 0x68/0x69 | `接线说明.md` says ICM42688 uses SPI1 with PB09/PB08/PB07/PB06 and CS | A for software config/source; B for old document claim; C for physical wiring | Treat document section as stale relative to current software. Confirm actual module wiring before hardware test. | Yes |
| OLED interface | Current software uses GPIO bit-bang: RST PB14, DC PB15, SCL PA28, SDA PA31 | `接线说明.md` lists planned I2C OLED PB02/PB03 under modules not enabled | A for current software config; B for old planning note | Do not infer I2C OLED. Current software uses 4-wire GPIO-style control names. | Yes for actual display hardware |
| Bluetooth module | Current software config has UART1 TX PB6, RX PB7, 9600 baud, RX interrupt enabled | `接线说明.md` lists Bluetooth as a planned module with TBD UART | A for software config; B for old planning note; C for physical module | Treat current software as integrated configuration. Confirm actual module model, baud, and wiring. | Yes |
| Buzzer pin | Current software config has buzzer PB17 | `接线说明.md` says buzzer PB05 | A for software config; B for old document claim; C for physical wiring | Do not change pin from document alone. Ask whether hardware was rewired or document is stale. | Yes |
| LED pin | Current software config has LED PB9 | `接线说明.md` says LED PB22 | A for software config; B for old document claim; C for physical wiring | Do not change pin from document alone. Confirm hardware before updating docs or software. | Yes |

## SysConfig And Generated File Consistency

Checked current resources:

- PWM: `empty.syscfg` assigns `TIMG6`, PB2 CCP0, PB3 CCP1; generated macros match. Evidence A.
- I2C: `empty.syscfg` assigns `I2C0`, PA0 SDA, PA1 SCL, Fast mode; generated macros show `I2C_ICM42688_INST I2C0` and 400000 Hz. Evidence A.
- UART0: `empty.syscfg` assigns PA10 TX, PA11 RX, 115200; generated macros match. Evidence A.
- UART_1: `empty.syscfg` assigns UART1 PB6 TX, PB7 RX, 9600, RX interrupt priority 1; generated macros and generated init match. Evidence A.
- GPIO groups AIN, BIN, LED, BUZZER, OLED, ENCODER, and GRAY_SERIAL match generated macros for checked pins. Evidence A.

No `empty.syscfg` versus generated-file conflict was observed in the checked resources.

## Generated Macro And Source Usage

Checked current usage:

- `Src/icm42688.c` uses `I2C_ICM42688_INST`.
- `Src/vofa.c` uses `UART0_INST`.
- `Src/uart_cmd.c` uses `UART0_INST` and `UART0_INST_INT_IRQN`.
- `Src/bluetooth.c` uses `UART_1_INST` and `UART_1_INST_INT_IRQN`.
- `Src/motor.c` uses `AIN_*`, `BIN_*`, `PWM_INST`, `GPIO_PWM_C0_IDX`, and `GPIO_PWM_C1_IDX`.
- `Src/encoder.c` uses generated encoder pin macros, but also uses raw `GPIOA_INT_IRQn` instead of the generated `ENCODER_GPIOA_INT_IRQN` macro. This matches current generated IRQ naming but is less resilient if SysConfig naming changes. Evidence A for current source behavior; B for maintainability risk.
- `Src/gray_serial.c` uses `GRAY_SERIAL_*` macros.
- `Src/oled.c` uses `OLED_*` macros.

No deleted or renamed generated macro usage was observed by the current search.

## Unresolved Configuration Questions

- Which generated encoder group corresponds to the physical left and right wheels. Evidence C.
- Whether the real motor driver wiring follows current software configuration or old wiring notes. Evidence C.
- Whether gray sensor polarity is `1U` active as current code says or `0U` as old wiring notes say. Evidence C.
- Whether physical ICM42688 wiring is now I2C on PA0/PA1 or still follows the old SPI note. Evidence C.
- Whether wiring notes should be updated to match current software configuration. This is a documentation task and should not be done unless explicitly requested.
