# Module Integration Checklist

Use before connecting a module into `main()` or a scheduler.

## Resource And Initialization

- Confirm SysConfig resources, generated macros, and physical-wiring unknowns.
- Confirm initialization order relative to `SYSCFG_DL_init()`.
- Confirm repeated foreground call location and target period.
- Confirm ISR enable location if the module uses interrupts.
- Confirm error return, timeout, retry, and disconnected-device handling.

## Runtime Interaction

- Check UART, I2C, GPIO, PWM, timer, and interrupt ownership.
- Check whether the module blocks inside the main loop.
- Check whether debug output changes loop timing.
- Check whether the module writes shared state or control targets.
- Check whether multiple modules can own the same actuator or UART.

## Verification

- Check header/source/call-site consistency.
- Check new source build registration.
- Run allowed static checks.
- Build only with user permission.
- Do not claim hardware success without hardware evidence.
