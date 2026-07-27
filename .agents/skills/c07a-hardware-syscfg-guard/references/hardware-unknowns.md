# Hardware Unknowns

The repository can confirm software configuration, not the physical state of the car. The following items remain Evidence C unless fresh hardware evidence is provided.

| Unknown hardware fact | Why the repository cannot prove it | Evidence needed to confirm |
|---|---|---|
| Whether real wiring matches current SysConfig pins | `empty.syscfg` defines software pinmux only | Current schematic, direct wiring inspection, continuity check, or user confirmation based on the current car |
| Whether all modules share GND correctly | Software files cannot observe wiring ground reference | Hardware inspection or measured continuity |
| Module supply voltage and logic level compatibility | SysConfig does not encode module power rails or module voltage tolerance | Module datasheets, schematic, and measured voltage |
| Motor driver model and enable/STBY wiring | Current software config has PWM and direction pins, but no proof of driver board wiring or enable state | Driver module model, schematic, wiring photo, or bench test with explicit user approval |
| Motor direction for positive PWM | `motor_set_pwm()` defines software direction pins, but physical motor polarity is unknown | Low-limit lifted-wheel test with user authorization |
| Encoder A/B direction and left/right physical assignment | Code can count configured pins, but cannot prove which wheel they are wired to or sign direction | Manual wheel rotation test, scope/logic analyzer, or user-confirmed wiring |
| MPU6050 physical bus wiring | Current project direction is MPU6050-only, and current software may reuse the legacy-named `I2C_ICM42688` I2C0 PA0/PA1 bus, but the repository cannot prove SDA/SCL, AD0, power, or pull-ups | Wiring inspection, I2C scan/WHO_AM_I test, pull-up verification, module datasheet |
| ICM42688 still connected or still intended | ICM42688 source may remain in the repository, but the user directed future work to MPU6050 only | Explicit user request to use ICM42688, wiring inspection, or removal/migration task |
| IMU mounting orientation | Source can define math and signs, but cannot know board mounting direction | Real-car orientation test or mechanical mounting reference |
| Gray sensor effective level and channel physical order | Code currently sets `LINE_TRACK_ACTIVE_LEVEL` and bit map, but sensor output polarity and orientation are hardware-dependent | Serial raw print over white/black surface and physical channel test |
| OLED controller model and panel orientation | Code sends SSD1306-like commands over GPIO bit-bang, but repository cannot prove the exact display variant | Module model, datasheet, or display test with user confirmation |
| Bluetooth module model and current baud setting | SysConfig sets UART1 to 9600 baud, but cannot prove module firmware baud or wiring | Module model, AT setting, wiring inspection, or communication test |
| UART0 USB-TTL adapter wiring | SysConfig sets UART0 PA10/PA11, but cannot prove adapter TX/RX/GND wiring | Wiring inspection or loopback/terminal test |
| Buzzer active level and module type | Generated code configures PB17 output and clears it at init, but module active level is hardware-specific | Module datasheet or controlled GPIO test |
| LED polarity and board connection | SysConfig sets PB9 output, but hardware LED polarity is not proven | Schematic or controlled GPIO test |

## Confirmation Rules

- Real-car tests, motor tests, flashing, or hardware motion require explicit user authorization.
- Prefer low-risk confirmation order: inspect wiring, check power/GND, static continuity, logic-level checks, then low-limit functional tests.
- Do not convert a software-configured pin map into a physical wiring statement.
