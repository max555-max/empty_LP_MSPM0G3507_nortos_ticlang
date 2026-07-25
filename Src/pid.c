#include "pid.h"
#include "encoder.h"
#include "motor.h"
#include "vofa.h"

#define SPEED_PID_VOFA_PERIOD_MS    (50U)
#define SPEED_PID_VOFA_DIVIDER      ((SPEED_PID_VOFA_PERIOD_MS + \
                                      SPEED_PID_CONTROL_PERIOD_MS - 1U) / \
                                      SPEED_PID_CONTROL_PERIOD_MS)

/*
 * pid.c
 *
 * 包含两层内容：
 *   1. 通用 PID 计算函数 pid_calculate()；
 *   2. 左右轮速度闭环 speed_pid_xxx()。
 *
 * 速度环输入：
 *   目标速度：mm/s；
 *   反馈速度：encoder.c 计算出的 mm/s。
 *
 * 速度环输出：
 *   motor_set_pwm() 使用的 PWM 数值。
 */

/* 左右轮独立 PID，方便分别调参。 */
static pid_t g_leftSpeedPid;
static pid_t g_rightSpeedPid;

/* 左右轮速度目标，单位 mm/s。 */
static int32_t g_leftTargetMmS = SPEED_PID_DEFAULT_LEFT_TARGET_MM_S;
static int32_t g_rightTargetMmS = SPEED_PID_DEFAULT_RIGHT_TARGET_MM_S;

/* 最近一次计算出的 PWM，供 VOFA 和调试接口读取。 */
static int32_t g_leftPwm = 0;
static int32_t g_rightPwm = 0;

static uint8_t g_speedPidVofaTick = 0U;

/* 对 value 做正负限幅。limit<=0 时认为不启用限幅。 */
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

/*
 * 最小启动 PWM 补偿。
 *
 * 原因：
 *   小车电机存在静摩擦，PID 输出很小时可能有 PWM 但电机不转。
 *
 * 逻辑：
 *   目标速度非 0 且 PID 输出非 0 时，如果 PWM 绝对值小于最小启动值，
 *   就把它补到 minStartPwm。
 */
static int32_t pid_apply_min_start_pwm(
    int32_t pwm, int32_t target, int32_t minStartPwm)
{
    /* 目标为 0、PID 输出为 0、或没有设置最小启动 PWM 时，不做补偿。 */
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

static uint8_t speed_pid_should_send_vofa(void)
{
    g_speedPidVofaTick++;

    if (g_speedPidVofaTick >= SPEED_PID_VOFA_DIVIDER) {
        g_speedPidVofaTick = 0U;
        return 1U;
    }

    return 0U;
}

void pid_init(pid_t *pid,
              int32_t kp,
              int32_t ki,
              int32_t kd,
              int32_t integralLimit,
              int32_t outputLimit)
{
    /* 保存 PID 参数并清空历史状态。 */
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
    /* 只清积分和历史误差，不改变 kp/ki/kd。 */
    pid->integral = 0;
    pid->previousError = 0;
}

int32_t pid_calculate(pid_t *pid, int32_t target, int32_t feedback)
{
    /* error > 0 表示反馈速度低于目标速度，需要增大输出。 */
    int32_t error = target - feedback;

    /* D 项使用本次误差与上次误差的差值。 */
    int32_t derivative = error - pid->previousError;
    int64_t output;

    /*
     * 积分项：
     *   持续误差会累计到 integral 中，用来消除稳态误差。
     *   积分立即限幅，避免长时间堵转/按住轮子时积分爆炸。
     */
    pid->integral += error;
    pid->integral = pid_limit(pid->integral, pid->integralLimit);

    /* 定点 PID：所有增益都按 PID_GAIN_SCALE 缩放。 */
    output = (int64_t) pid->kp * error +
             (int64_t) pid->ki * pid->integral +
             (int64_t) pid->kd * derivative;
    output /= PID_GAIN_SCALE;

    /* 保存当前误差，供下次计算 D 项。 */
    pid->previousError = error;

    /* PID 输出最终也要限幅，速度环里一般限制到电机 PWM 最大值。 */
    return pid_limit((int32_t) output, pid->outputLimit);
}

void speed_pid_init(void)
{
    /* 初始化默认目标和输出。上层任务启动后通常会重新设置目标速度。 */
    g_leftTargetMmS = SPEED_PID_DEFAULT_LEFT_TARGET_MM_S;
    g_rightTargetMmS = SPEED_PID_DEFAULT_RIGHT_TARGET_MM_S;
    g_leftPwm = 0;
    g_rightPwm = 0;
    g_speedPidVofaTick = 0U;

    /* 分别初始化左右轮 PID，左右轮参数可以不同。 */
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

    /* 初始化后立即确保电机停止。 */
    motor_set_pwm(0, 0);
}

void speed_pid_set_target(int32_t leftTargetMmS, int32_t rightTargetMmS)
{
    /* 只改目标速度，不清 PID；连续控制时响应更平滑。 */
    g_leftTargetMmS = leftTargetMmS;
    g_rightTargetMmS = rightTargetMmS;
}

void speed_pid_set_speed(int32_t leftSpeedMmS, int32_t rightSpeedMmS)
{
    speed_pid_set_target(leftSpeedMmS, rightSpeedMmS);
}

void speed_pid_set_left_speed(int32_t leftSpeedMmS)
{
    speed_pid_set_target(leftSpeedMmS, g_rightTargetMmS);
}

void speed_pid_set_right_speed(int32_t rightSpeedMmS)
{
    speed_pid_set_target(g_leftTargetMmS, rightSpeedMmS);
}

void speed_pid_stop(void)
{
    /* 停车时目标清零，并清掉积分，避免下次启动带着旧积分冲出去。 */
    speed_pid_set_target(0, 0);
    pid_reset(&g_leftSpeedPid);
    pid_reset(&g_rightSpeedPid);
    g_leftPwm = 0;
    g_rightPwm = 0;
    g_speedPidVofaTick = 0U;
    motor_set_pwm(0, 0);
}

void speed_pid_set_left_gains(int32_t kp, int32_t ki, int32_t kd)
{
    /* 在线改参后复位该侧 PID，避免新参数叠加旧积分导致突变。 */
    g_leftSpeedPid.kp = kp;
    g_leftSpeedPid.ki = ki;
    g_leftSpeedPid.kd = kd;
    pid_reset(&g_leftSpeedPid);
}

void speed_pid_set_right_gains(int32_t kp, int32_t ki, int32_t kd)
{
    /* 在线改参后复位该侧 PID。 */
    g_rightSpeedPid.kp = kp;
    g_rightSpeedPid.ki = ki;
    g_rightSpeedPid.kd = kd;
    pid_reset(&g_rightSpeedPid);
}

void speed_pid_set_left_integral_limit(int32_t integralLimit)
{
    /* 修改积分限幅后，当前积分也要重新夹到新范围内。 */
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
    /* 读取编码器反馈速度，单位 mm/s。 */
    int32_t leftSpeed = encoder_get_left_speed_mm_s();
    int32_t rightSpeed = encoder_get_right_speed_mm_s();

    /*
     * 停车保护：
     *
     * 当上层要求两轮都停时，直接输出 PWM=0。
     * 不再让 PID 根据“目标 0、反馈非 0”计算反向制动 PWM。
     *
     * 原因：
     *   在这台车上，停车点附近如果继续让 PID 反向修正，
     *   可能表现成“到点后还慢慢往前/往后蹭”。
     */
    if ((g_leftTargetMmS == 0) && (g_rightTargetMmS == 0)) {
        pid_reset(&g_leftSpeedPid);
        pid_reset(&g_rightSpeedPid);
        g_leftPwm = 0;
        g_rightPwm = 0;
        motor_set_pwm(0, 0);

        if (speed_pid_should_send_vofa() != 0U) {
            vofa_send_six_int(g_leftTargetMmS,
                              leftSpeed,
                              g_leftPwm,
                              g_rightTargetMmS,
                              rightSpeed,
                              g_rightPwm);
        }
        return;
    }

    /* 正常速度闭环：分别计算左右轮 PWM。 */
    g_leftPwm = pid_calculate(&g_leftSpeedPid, g_leftTargetMmS, leftSpeed);
    g_rightPwm = pid_calculate(&g_rightSpeedPid, g_rightTargetMmS, rightSpeed);
    /* 克服静摩擦：目标非 0 但 PWM 太小时补到最小启动 PWM。 */
    g_leftPwm = pid_apply_min_start_pwm(
        g_leftPwm, g_leftTargetMmS, SPEED_PID_LEFT_MIN_START_PWM);
    g_rightPwm = pid_apply_min_start_pwm(
        g_rightPwm, g_rightTargetMmS, SPEED_PID_RIGHT_MIN_START_PWM);

    /* 下发到底层电机驱动。 */
    motor_set_pwm(g_leftPwm, g_rightPwm);

    /*
     * VOFA FireWater 调试输出：
     *   ch0：左轮目标速度
     *   ch1：左轮反馈速度
     *   ch2：左轮 PWM
     *   ch3：右轮目标速度
     *   ch4：右轮反馈速度
     *   ch5：右轮 PWM
     */
    if (speed_pid_should_send_vofa() != 0U) {
        vofa_send_six_int(g_leftTargetMmS,
                          leftSpeed,
                          g_leftPwm,
                          g_rightTargetMmS,
                          rightSpeed,
                          g_rightPwm);
    }
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
