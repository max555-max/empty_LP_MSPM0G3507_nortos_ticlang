# C07A Software Configuration Pin Map

This table is a software configuration map, not proof of physical wiring.

Evidence A means the item is confirmed by current `empty.syscfg`, `Debug/ti_msp_dl_config.h`, and/or `Debug/ti_msp_dl_config.c`. Physical wiring remains Evidence C unless confirmed by current schematic, hardware inspection, user confirmation, or real-car testing.

## Core Generated Configuration

| Resource | Software configuration | Generated names or macros | Related code | Evidence |
|---|---|---|---|---|
| Device | MSPM0G3507 target in SysConfig v2 args; generated config also defines `CONFIG_MSPM0G3507` | `CONFIG_MSPM0G350X`, `CONFIG_MSPM0G3507` | `empty.c` includes `ti_msp_dl_config.h` | A: `empty.syscfg`, `Debug/ti_msp_dl_config.h` |
| System init | Generated init calls GPIO, SYSCTL, PWM, I2C, UART0, UART_1, SYSTICK | `SYSCFG_DL_init()` | `empty.c` calls `SYSCFG_DL_init()` | A: `Debug/ti_msp_dl_config.c`, `empty.c` |
| SysTick | 1 ms generated SysTick period | `SYSCFG_DL_SYSTICK_init()` | `SysTick_Handler()` calls `delay_tick()` | A: `empty.syscfg`, `Debug/ti_msp_dl_config.c`, `empty.c` |

## Peripheral And Pin Map

| Function | Software instance | Software pin or channel | Generated names or macros | Related modules | Evidence |
|---|---|---|---|---|---|
| IMU I2C SDA, legacy instance name | `I2C_ICM42688` / `I2C0` | PA0, 400 kHz I2C bus | `I2C_ICM42688_INST`, `GPIO_I2C_ICM42688_SDA_*`, `I2C_ICM42688_BUS_SPEED_HZ` | Preferred: `Src/mpu6050.c`, `Inc/mpu6050.h`; legacy: `Src/icm42688.c`, `Inc/icm42688.h` | A for software config and module references; B for MPU6050-only policy from user instruction |
| IMU I2C SCL, legacy instance name | `I2C_ICM42688` / `I2C0` | PA1, 400 kHz I2C bus | `GPIO_I2C_ICM42688_SCL_*` | Preferred: `Src/mpu6050.c`, `Inc/mpu6050.h`; legacy: `Src/icm42688.c`, `Inc/icm42688.h` | A for software config and module references; B for MPU6050-only policy from user instruction |
| UART0 TX | `UART0` | PA10, 115200 baud | `UART0_INST`, `GPIO_UART0_TX_*`, `UART0_BAUD_RATE` | `Src/vofa.c`, `Src/uart_cmd.c` | A |
| UART0 RX | `UART0` | PA11, 115200 baud | `GPIO_UART0_RX_*`, `UART0_INST_INT_IRQN` | `Src/uart_cmd.c` if enabled by user code | A |
| Bluetooth UART TX | `UART_1` / hardware `UART1` | PB6, 9600 baud | `UART_1_INST`, `GPIO_UART_1_TX_*`, `UART_1_BAUD_RATE` | `Src/bluetooth.c`, `Inc/bluetooth.h` | A |
| Bluetooth UART RX | `UART_1` / hardware `UART1` | PB7, 9600 baud, RX interrupt enabled | `GPIO_UART_1_RX_*`, `UART_1_INST_INT_IRQN` | `Src/bluetooth.c` | A |
| Motor PWM left channel | `PWM` / `TIMG6` | PB2 / CCP0, period 4000, inverted output | `PWM_INST`, `GPIO_PWM_C0_*`, `GPIO_PWM_C0_IDX` | `Src/motor.c`, `Inc/motor.h` | A |
| Motor PWM right channel | `PWM` / `TIMG6` | PB3 / CCP1, period 4000, inverted output | `GPIO_PWM_C1_*`, `GPIO_PWM_C1_IDX` | `Src/motor.c`, `Inc/motor.h` | A |
| Motor A direction 1 | GPIO group `AIN` | PA14 output, initial SET | `AIN_PORT`, `AIN_AIN1_PIN` | `Src/motor.c` | A |
| Motor A direction 2 | GPIO group `AIN` | PA13 output, initial SET | `AIN_AIN2_PIN` | `Src/motor.c` | A |
| Motor B direction 1 | GPIO group `BIN` | PA16 output, initial SET | `BIN_PORT`, `BIN_BIN1_PIN` | `Src/motor.c` | A |
| Motor B direction 2 | GPIO group `BIN` | PA17 output, initial SET | `BIN_BIN2_PIN` | `Src/motor.c` | A |
| Encoder E1A | GPIO group `ENCODER` | PA25 input, rise/fall interrupt | `ENCODER_E1A_PORT`, `ENCODER_E1A_PIN`, `ENCODER_GPIOA_INT_IRQN` | `Src/encoder.c`, `Inc/encoder.h` | A |
| Encoder E1B | GPIO group `ENCODER` | PA26 input, rise/fall interrupt | `ENCODER_E1B_PORT`, `ENCODER_E1B_PIN` | `Src/encoder.c`, `Inc/encoder.h` | A |
| Encoder E2A | GPIO group `ENCODER` | PB20 input, rise/fall interrupt | `ENCODER_E2A_PORT`, `ENCODER_E2A_PIN`, `ENCODER_GPIOB_INT_IRQN` | `Src/encoder.c`, `Inc/encoder.h` | A |
| Encoder E2B | GPIO group `ENCODER` | PB24 input, rise/fall interrupt | `ENCODER_E2B_PORT`, `ENCODER_E2B_PIN` | `Src/encoder.c`, `Inc/encoder.h` | A |
| Gray serial DAT | GPIO group `GRAY_SERIAL` | PA12 input | `GRAY_SERIAL_DAT_PORT`, `GRAY_SERIAL_DAT_PIN` | `Src/gray_serial.c`, `Inc/gray_serial.h`, `Src/line_track.c` | A |
| Gray serial CLK | GPIO group `GRAY_SERIAL` | PB16 output, initial SET | `GRAY_SERIAL_CLK_PORT`, `GRAY_SERIAL_CLK_PIN` | `Src/gray_serial.c`, `Inc/gray_serial.h` | A |
| OLED RST | GPIO group `OLED_RST` | PB14 output, initial SET | `OLED_RST_PORT`, `OLED_RST_PIN_RST_PIN` | `Src/oled.c`, `Inc/oled.h` | A |
| OLED DC | GPIO group `OLED_DC` | PB15 output, initial SET | `OLED_DC_PORT`, `OLED_DC_PIN_DC_PIN` | `Src/oled.c`, `Inc/oled.h` | A |
| OLED SCL | GPIO group `OLED_SCL` | PA28 output, initial SET | `OLED_SCL_PORT`, `OLED_SCL_PIN_SCL_PIN` | `Src/oled.c`, `Inc/oled.h` | A |
| OLED SDA | GPIO group `OLED_SDA` | PA31 output, initial SET | `OLED_SDA_PORT`, `OLED_SDA_PIN_SDA_PIN` | `Src/oled.c`, `Inc/oled.h` | A |
| LED | GPIO group `LED` | PB9 output, initial SET | `LED_PORT`, `LED_LED1_PIN` | No current dedicated source module observed | A |
| Buzzer | GPIO group `BUZZER` | PB17 output, generated code clears it during init | `BUZZER_PORT`, `BUZZER_BZ_PIN` | No current dedicated source module observed | A |

## Source Macro Usage Summary

- `empty.c` calls only `SYSCFG_DL_init()` from generated config. Evidence A.
- `Src/mpu6050.c` requires `I2C_ICM42688_INST` and uses it through `MPU6050_I2C_INST`. Evidence A when present.
- `Src/icm42688.c` also uses `I2C_ICM42688_INST`; treat it as legacy unless the user explicitly asks for ICM42688. Evidence A for source usage; Evidence B for legacy-only policy.
- `Src/vofa.c` uses `UART0_INST` for blocking UART0 transmit. Evidence A.
- `Src/uart_cmd.c` uses `UART0_INST` and `UART0_INST_INT_IRQN` when that command module is initialized. Evidence A.
- `Src/bluetooth.c` uses `UART_1_INST` and `UART_1_INST_INT_IRQN`, and defines `UART_1_INST_IRQHandler()`. Evidence A.
- `Src/motor.c` uses `AIN_*`, `BIN_*`, `PWM_INST`, `GPIO_PWM_C0_IDX`, and `GPIO_PWM_C1_IDX`. Evidence A.
- `Src/encoder.c` uses generated encoder pin macros and raw GPIOA/GPIOB interrupt handling. Evidence A.
- `Src/gray_serial.c` uses `GRAY_SERIAL_DAT_*` and `GRAY_SERIAL_CLK_*`. Evidence A.
- `Src/oled.c` uses `OLED_RST_*`, `OLED_DC_*`, `OLED_SCL_*`, and `OLED_SDA_*`. Evidence A.

## Software Resource Occupancy

No duplicate software pin assignment was observed in current `empty.syscfg` for the listed active resources. Evidence A for current SysConfig content. This does not prove physical wiring correctness.
