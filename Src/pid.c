#include "pid.h"
#include "encoder.h"
#include "motor.h"
#include "vofa.h"

static pid_t g_leftSpeedPid;
static pid_t g_rightSpeedPid;
static int32_t g_leftTargetMmS = SPEED_PID_DEFAULT_LEFT_TARGET_MM_S;
static int32_t g_rightTargetMmS = SPEED_PID_DEFAULT_RIGHT_TARGET_MM_S;
static int32_t g_leftPwm = 0;
static int32_t g_rightPwm = 0;

static int32_t pid_limit(int32_t value, int32_t limit)
{
    if (limit <= 0) {
        return value;
    }

    if (value > limit) {
        return limit;
    }

    if (value < -limit) {
        return -limit;
    }

    return value;
}

static int32_t pid_apply_min_start_pwm(
    int32_t pwm, int32_t target, int32_t minStartPwm)
{
    if ((target == 0) || (pwm == 0) || (minStartPwm <= 0)) {
        return pwm;
    }

    if ((pwm > 0) && (pwm < minStartPwm)) {
        return minStartPwm;
    }

    if ((pwm < 0) && (pwm > -minStartPwm)) {
        return -minStartPwm;
    }

    return pwm;
}

void pid_init(pid_t *pid,
              int32_t kp,
              int32_t ki,
              int32_t kd,
              int32_t integralLimit,
              int32_t outputLimit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0;
    pid->previousError = 0;
    pid->integralLimit = integralLimit;
    pid->outputLimit = outputLimit;
}

void pid_reset(pid_t *pid)
{
    pid->integral = 0;
    pid->previousError = 0;
}

int32_t pid_calculate(pid_t *pid, int32_t target, int32_t feedback)
{
    int32_t error = target - feedback;
    int32_t derivative = error - pid->previousError;
    int64_t output;

    pid->integral += error;
    pid->integral = pid_limit(pid->integral, pid->integralLimit);

    output = (int64_t) pid->kp * error +
             (int64_t) pid->ki * pid->integral +
             (int64_t) pid->kd * derivative;
    output /= PID_GAIN_SCALE;

    pid->previousError = error;

    return pid_limit((int32_t) output, pid->outputLimit);
}

void speed_pid_init(void)
{
    g_leftTargetMmS = SPEED_PID_DEFAULT_LEFT_TARGET_MM_S;
    g_rightTargetMmS = SPEED_PID_DEFAULT_RIGHT_TARGET_MM_S;
    g_leftPwm = 0;
    g_rightPwm = 0;

    pid_init(&g_leftSpeedPid,
             SPEED_PID_LEFT_KP,
             SPEED_PID_LEFT_KI,
             SPEED_PID_LEFT_KD,
             SPEED_PID_LEFT_INTEGRAL_LIMIT,
             MOTOR_PWM_MAX);
    pid_init(&g_rightSpeedPid,
             SPEED_PID_RIGHT_KP,
             SPEED_PID_RIGHT_KI,
             SPEED_PID_RIGHT_KD,
             SPEED_PID_RIGHT_INTEGRAL_LIMIT,
             MOTOR_PWM_MAX);

    motor_set_pwm(0, 0);
}

void speed_pid_set_target(int32_t leftTargetMmS, int32_t rightTargetMmS)
{
    g_leftTargetMmS = leftTargetMmS;
    g_rightTargetMmS = rightTargetMmS;
}

void speed_pid_set_left_gains(int32_t kp, int32_t ki, int32_t kd)
{
    g_leftSpeedPid.kp = kp;
    g_leftSpeedPid.ki = ki;
    g_leftSpeedPid.kd = kd;
    pid_reset(&g_leftSpeedPid);
}

void speed_pid_set_right_gains(int32_t kp, int32_t ki, int32_t kd)
{
    g_rightSpeedPid.kp = kp;
    g_rightSpeedPid.ki = ki;
    g_rightSpeedPid.kd = kd;
    pid_reset(&g_rightSpeedPid);
}

void speed_pid_set_left_integral_limit(int32_t integralLimit)
{
    g_leftSpeedPid.integralLimit = integralLimit;
    g_leftSpeedPid.integral = pid_limit(g_leftSpeedPid.integral, integralLimit);
}

void speed_pid_set_right_integral_limit(int32_t integralLimit)
{
    g_rightSpeedPid.integralLimit = integralLimit;
    g_rightSpeedPid.integral =
        pid_limit(g_rightSpeedPid.integral, integralLimit);
}

void speed_pid_control_update(void)
{
    int32_t leftSpeed = encoder_get_left_speed_mm_s();
    int32_t rightSpeed = encoder_get_right_speed_mm_s();

    g_leftPwm = pid_calculate(&g_leftSpeedPid, g_leftTargetMmS, leftSpeed);
    g_rightPwm = pid_calculate(&g_rightSpeedPid, g_rightTargetMmS, rightSpeed);
    g_leftPwm = pid_apply_min_start_pwm(
        g_leftPwm, g_leftTargetMmS, SPEED_PID_LEFT_MIN_START_PWM);
    g_rightPwm = pid_apply_min_start_pwm(
        g_rightPwm, g_rightTargetMmS, SPEED_PID_RIGHT_MIN_START_PWM);

    motor_set_pwm(g_leftPwm, g_rightPwm);

    /*
     * FireWater:
     * ch0 left target, ch1 left feedback, ch2 left PWM,
     * ch3 right target, ch4 right feedback, ch5 right PWM.
     */
    vofa_send_six_int(g_leftTargetMmS,
                      leftSpeed,
                      g_leftPwm,
                      g_rightTargetMmS,
                      rightSpeed,
                      g_rightPwm);
}

int32_t speed_pid_get_left_target(void)
{
    return g_leftTargetMmS;
}

int32_t speed_pid_get_right_target(void)
{
    return g_rightTargetMmS;
}

int32_t speed_pid_get_left_pwm(void)
{
    return g_leftPwm;
}

int32_t speed_pid_get_right_pwm(void)
{
    return g_rightPwm;
}

int32_t speed_pid_get_left_kp(void)
{
    return g_leftSpeedPid.kp;
}

int32_t speed_pid_get_left_ki(void)
{
    return g_leftSpeedPid.ki;
}

int32_t speed_pid_get_left_kd(void)
{
    return g_leftSpeedPid.kd;
}

int32_t speed_pid_get_right_kp(void)
{
    return g_rightSpeedPid.kp;
}

int32_t speed_pid_get_right_ki(void)
{
    return g_rightSpeedPid.ki;
}

int32_t speed_pid_get_right_kd(void)
{
    return g_rightSpeedPid.kd;
}
