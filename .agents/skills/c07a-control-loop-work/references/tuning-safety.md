# Tuning Safety

## Before Tuning

- Confirm the loop is actually initialized and scheduled. Module presence is not active runtime evidence.
- Confirm feedback direction before increasing gains.
- Confirm target speed and PWM limits are low.
- Save the current parameter baseline.
- Decide which curve or observation will prove improvement.

## Change Discipline

- Change one primary parameter or behavior at a time.
- Do not change Kp, Ki, Kd, limits, and scheduler timing in one unreviewed patch.
- Record stored value, scaled value, unit, reason, expected effect, and rollback value.
- Treat one good run as a data point, not proof of optimal tuning.

## Suggested Order

1. Encoder direction and speed feedback sanity.
2. Motor direction and low-limit PWM response.
3. Speed PID at low target speed.
4. Angle loop with low base speed or lifted wheels as appropriate.
5. Line tracking at low base speed after gray raw values and active level are confirmed.
6. Combined mode only after single loops are predictable.

## Authorization Boundary

Lifted-wheel tests, ground tests, flashing, motor startup, higher PWM limits, and higher target speeds require explicit user authorization.
