# SysConfig Guard Rules

## Source Of Truth

- Treat `empty.syscfg` as the editable source for generated peripheral and pin configuration.
- Treat `Debug/ti_msp_dl_config.h` and `Debug/ti_msp_dl_config.c` as generated outputs used by C code.
- Do not hand-maintain generated files as the only change when the intended change is a pin, peripheral instance, baud rate, timer channel, interrupt, or GPIO mode change.
- If generated files disagree with `empty.syscfg`, report the mismatch and ask the user whether SysConfig regeneration is expected. Do not auto-repair generated files by hand.

## Required Checks Before Configuration Changes

1. Read `AGENTS.md`.
2. Read `empty.syscfg`.
3. Read `Debug/ti_msp_dl_config.h` and `Debug/ti_msp_dl_config.c`.
4. Search `Inc/`, `Src/`, and `empty.c` for the affected generated macro names and interrupt handlers.
5. Check whether the affected `.c` files are included in the CCS build metadata when adding a new module.
6. Check current wiring notes only as documentation, not physical proof.
7. List software facts, document claims, user-confirmed wiring, and hardware unknowns separately.

## Peripheral Review Checklist

### GPIO

- Check port, pin, IOMUX, direction, initial value, output enable, input resistor, inversion, and interrupt edge.
- Check whether a GPIO is used as a plain pin or peripheral function.
- Check whether source code uses the generated group macros rather than hard-coded port/pin values.
- Check whether comments or wiring notes name a different pin than generated macros.

### UART

- Check hardware instance, generated instance name, TX/RX pins, baud rate, interrupt enable, interrupt priority, and handler name.
- Check whether more than one module wants the same UART RX interrupt.
- Check whether telemetry and command parsing share one UART and whether the main loop enables both intentionally.
- Confirm UART module wiring with user or hardware evidence before calling it a physical fact.

### I2C

- Check hardware instance, generated instance name, SDA/SCL pins, bus speed, controller mode, clock source, and pull-up assumptions.
- Check driver device addresses separately from bus pin configuration.
- Confirm external pull-ups, sensor address strap, power, and GND by hardware evidence; SysConfig cannot prove them.

### PWM And Timers

- Check timer instance, period, start mode, channel index, CCP pin, output inversion, and compare-value limits used by source code.
- Check whether `MOTOR_PWM_MAX` or control output limits exceed the generated PWM period.
- Check whether a timer is reused for another periodic task.

### Interrupts And Handlers

- Check generated `*_IRQHandler` and `*_INT_IRQN` names against source-defined handlers.
- Do not add blocking work to interrupt handlers.
- For shared GPIO interrupt groups, check all pins on that IRQ path.

## New Module Resource Review

- Identify required peripheral type and whether an existing instance is already occupied.
- Check unused pins in SysConfig rather than assuming a pin is available.
- Check conflicts with PWM, UART, I2C, SWD/debug pins, encoder inputs, motor outputs, OLED, gray sensor, LED, and buzzer.
- If a module needs RX interrupts or periodic timing, check existing interrupt handlers and scheduler ownership.
- Before implementation, state whether the work is software configuration only or requires physical wiring confirmation.

## After SysConfig Regeneration

Re-check:

- `Debug/ti_msp_dl_config.h` macro names and instance names.
- `Debug/ti_msp_dl_config.c` initialization order and module init functions.
- All source files using affected macros.
- Interrupt handler names and enabled IRQs.
- Build metadata if new source files were added.
- Wiring notes only if the user explicitly asks to update documentation.

## Reporting Rules

- Report "software configuration" for SysConfig/generated facts.
- Report "document says" for wiring notes.
- Report "user confirmed" only when the user explicitly provides current hardware confirmation.
- Report "requires hardware confirmation" for real wiring, electrical level, power, module variant, motor direction, encoder direction, and real-car behavior.
- Never describe "build passed" as "hardware configuration correct".
