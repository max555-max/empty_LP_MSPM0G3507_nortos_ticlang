# PID Discretization

## Current Speed PID

- Error is `target - feedback`. Evidence A: `Src/pid.c`.
- Integral accumulates raw error each control call and is clamped by `integralLimit`. Evidence A.
- Derivative is `error - previousError`. Evidence A.
- Output is `(kp*error + ki*integral + kd*derivative) / 1000`. Evidence A.
- The derivative term does not divide by `dt`; changing call period changes effective D behavior. Evidence A for formula, B for tuning impact.
- Integral accumulation is per call; changing call period changes effective I behavior. Evidence A for formula, B for tuning impact.
- Output is clamped to `MOTOR_PWM_MAX`, currently 3800. Evidence A.
- Nonzero target with small nonzero PWM is compensated to minimum start PWM. Evidence A.
- Stop condition resets PID and writes `motor_set_pwm(0,0)`. Evidence A.

## Current Angle Loop

- Error is wrapped `targetYaw - currentYaw` in degrees. Evidence A.
- D term uses `-attitude_get_gyro_z_dps()` instead of error difference over `dt`. Evidence A.
- Gains are stored scaled by 1000. Evidence A.
- Output correction and final targets are limited. Evidence A.

## Current Line Loop

- Error is a weighted average of active gray channels. Evidence A.
- Derivative is `error - previousError`, clamped to 1600 and not divided by `dt`. Evidence A.
- P and D terms are each divided by 1000 and separately limited before summing. Evidence A.
- Lost-line handling either stops or rotates based on last error depending on selected update function. Evidence A.

## Review Rules

- Period changes require re-checking Ki, Kd, integral limit, derivative clamp, and minimum start PWM behavior.
- Integer multiplications should be reviewed for overflow; speed PID uses `int64_t` for output calculation. Evidence A.
- Do not tune gains until the loop is actually scheduled and feedback direction is confirmed.
