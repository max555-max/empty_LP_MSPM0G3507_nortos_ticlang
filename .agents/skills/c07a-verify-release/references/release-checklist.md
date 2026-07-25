# Release Checklist

This checklist supports review only. It does not authorize flashing or hardware tests.

## File And Configuration State

- Record `git status --short`.
- Confirm requested changes are limited to intended files.
- Confirm no unrelated user changes were overwritten.
- Confirm `empty.syscfg` and generated config status is understood.
- Confirm new `.c` files are registered in Debug CCS metadata.

## Build And Warning State

- State whether build was not run, plan-only, or run with explicit user permission.
- If built, record command, exit code, warnings, errors, and changed artifacts.
- Do not report build success as hardware success.

## Scheduling And Control State

- List current initialization and call locations for changed modules.
- Confirm each periodic task has one owner and one intended time source.
- Confirm encoder speed update, speed PID, angle loop, and line tracking dependency order if control code changed.

## Actuator Safety

- Search all `motor_set_pwm` calls.
- Search all speed target writers.
- Confirm final motor PWM owner is explicit.
- Confirm default PWM and target-speed limits are understood.
- Do not raise PWM limit or target speed without explicit user authorization.

## Debug Output

- Confirm UART/OLED/VOFA/Bluetooth output does not unexpectedly dominate a timing-critical loop.
- Confirm ISR only buffers or clears flags.

## Human Approval Gates

- Ask before flashing.
- Ask before motor startup.
- Ask before lifted-wheel test.
- Ask before ground test.
- Ask before increasing limits or speed.

## Competition Freeze

- Record baseline parameter set.
- Record validation level reached.
- Preserve rollback point.
- List unresolved Evidence B/C items.
