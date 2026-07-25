# Control Units

Re-read current headers and sources before changing values.

## Code-Confirmed Units And Scales

- Speed targets: `speed_pid_set_target()` and `speed_pid_set_speed()` use mm/s. Evidence A: `Inc/pid.h`, `Src/pid.c`.
- Encoder feedback: `encoder_get_left_speed_mm_s()` and `encoder_get_right_speed_mm_s()` return mm/s. Evidence A: `Inc/encoder.h`, `Src/encoder.c`.
- Encoder parameters: 13 motor lines, AB x4, gear ratio `20409/1000`, wheel diameter 48 mm, speed period 10 ms. Evidence A: `Inc/encoder.h`.
- PWM command: `motor_set_pwm()` accepts signed PWM counts; `MOTOR_PWM_MAX` is 3800 and generated PWM period is 4000. Evidence A: `Inc/motor.h`, `empty.syscfg`, `Debug/ti_msp_dl_config.c`.
- Speed PID gains: stored as integers scaled by `PID_GAIN_SCALE` 1000. Evidence A: `Inc/pid.h`, `Src/pid.c`.
- Angle yaw unit: degrees; gyro Z for angle D term is deg/s. Evidence A: `Inc/angle_control.h`, `Src/angle_control.c`, `Inc/attitude.h`.
- Angle-loop correction: mm/s differential correction; default max correction 220 mm/s; max target 900 mm/s. Evidence A: `Inc/angle_control.h`.
- Line-track error: weighted integer using channel weights from left positive to right negative. Evidence A: `Src/line_track.c`.
- Line-track gains: integer scale `/1000`; default Kp 250 and Kd 120. Evidence A: `Inc/line_track.h`, `Src/line_track.c`.

## Hardware-Dependent Facts

- Positive motor PWM physical direction requires lifted-wheel or real-car confirmation. Evidence C.
- Encoder left/right physical assignment and direction signs require hardware confirmation. Evidence C.
- IMU mounting orientation and yaw sign require hardware confirmation. Evidence C.
- Gray-sensor active level and channel order require raw sensor confirmation on the actual car. Evidence C.

## Review Rule

When changing parameters, report both stored integer value and real scaled value. Do not call a parameter stable without real-car evidence.
