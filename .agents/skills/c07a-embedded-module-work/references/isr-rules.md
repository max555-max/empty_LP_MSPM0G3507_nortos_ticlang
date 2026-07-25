# ISR Rules

## Current Interrupt Paths

- `SysTick_Handler()` calls `delay_tick()`. Evidence A: `empty.c`.
- `UART_1_INST_IRQHandler()` reads UART1 RX bytes into a ring buffer for Bluetooth. Evidence A: `Src/bluetooth.c`.
- `UART0_IRQHandler()` calls `uart_cmd_irq_handler()` for UART0 command buffering. Evidence A: `Src/uart_cmd.c`.
- `GROUP1_IRQHandler()` calls `encoder_gpio_irq_handler()` for GPIO encoder edges. Evidence A: `Src/encoder.c`.

## Allowed ISR Work

- Read pending interrupt state.
- Read or write one byte/sample/count.
- Update volatile indexes or counters.
- Copy to fixed-size buffers with bounds checks.
- Clear interrupt status.
- Set flags for foreground work.

## Prohibited ISR Work

- `delay_ms()`.
- OLED refresh or printing.
- Blocking UART transmit.
- I2C polling/transactions.
- VOFA output.
- Complex string parsing.
- Long loops unrelated to draining a hardware FIFO.

## Shared State

- Variables modified by ISR and foreground should be `volatile` or copied under a short critical section.
- Ring buffers need overflow behavior.
- Foreground parsing should consume a snapshot or bounded buffer, not parse directly while ISR mutates it.
- Handler names must match generated interrupt names or the startup/vector configuration.
