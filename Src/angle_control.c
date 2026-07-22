#include "angle_control.h"

#include "attitude.h"
#include "pid.h"

/*
 * angle_control.c
 *
 * 角度环实现文件。
 *
 * 角度环不直接输出 PWM，而是把航向角误差转换为左右轮目标速度差，
 * 再交给左右轮速度 PID 控制电机。
 *
 * 控制形式：
 *
 * correction = Kp × yawError - Kd × gyroZ
 *
 * 对于固定目标角度：
 *
 * error = targetYaw - currentYaw
 * d(error)/dt = -yawRate
 */

/* 角度环当前状态，用于控制和调试。 */
static angle_control_status_t g_angleControlStatus;

/*
 * 将角度归一化到 -180°～180°。
 *
 * 例如：
 *   190°  -> -170°
 *  -190°  ->  170°
 */
static float angle_control_wrap_180(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }

    while (angle < -180.0f) {
        angle += 360.0f;
    }

    return angle;
}

/* 将 int32_t 数值限制在 -limit～+limit。 */
static int32_t angle_control_limit_i32(int32_t value, int32_t limit)
{
    if (value > limit) {
        return limit;
    }

    if (value < -limit) {
        return -limit;
    }

    return value;
}

/* float 转换为 int32_t，并进行四舍五入。 */
static int32_t angle_control_float_to_i32(float value)
{
    if (value >= 0.0f) {
        return (int32_t)(value + 0.5f);
    }

    return (int32_t)(value - 0.5f);
}

void angle_control_init(void)
{
    g_angleControlStatus.enabled = false;

    g_angleControlStatus.targetYawDeg = 0.0f;
    g_angleControlStatus.currentYawDeg = 0.0f;
    g_angleControlStatus.errorDeg = 0.0f;
    g_angleControlStatus.errorRateDps = 0.0f;

    g_angleControlStatus.baseSpeedMmS = 0;
    g_angleControlStatus.correctionMmS = 0;
    g_angleControlStatus.leftTargetMmS = 0;
    g_angleControlStatus.rightTargetMmS = 0;

    /*
     * 初始化时将左右轮目标速度清零，
     * 防止控制模块初始化后电机意外运行。
     */
    speed_pid_set_speed(0, 0);
}

void angle_control_enable(bool enable)
{
    g_angleControlStatus.enabled = enable;

    if (!enable) {
        /*
         * 关闭角度环时清零速度目标。
         *
         * 基础速度和目标角度暂时保留，
         * 下次使能时仍然使用原来的设置。
         */
        speed_pid_set_speed(0, 0);

        g_angleControlStatus.correctionMmS = 0;
        g_angleControlStatus.leftTargetMmS = 0;
        g_angleControlStatus.rightTargetMmS = 0;
        g_angleControlStatus.errorRateDps = 0.0f;
    }
}

void angle_control_stop(void)
{
    /*
     * 完全停止角度环，同时清零基础速度和输出。
     */
    g_angleControlStatus.enabled = false;

    g_angleControlStatus.baseSpeedMmS = 0;
    g_angleControlStatus.correctionMmS = 0;
    g_angleControlStatus.leftTargetMmS = 0;
    g_angleControlStatus.rightTargetMmS = 0;
    g_angleControlStatus.errorRateDps = 0.0f;

    speed_pid_set_speed(0, 0);
}

void angle_control_set_base_speed(int32_t baseSpeedMmS)
{
    /*
     * 对基础速度限幅，防止超过速度环和电机的控制范围。
     */
    g_angleControlStatus.baseSpeedMmS =
        angle_control_limit_i32(
            baseSpeedMmS,
            ANGLE_CONTROL_MAX_TARGET_MM_S);
}

void angle_control_lock_current_yaw(void)
{
    attitude_euler_t euler;

    /*
     * 将当前 yaw 设置为新的目标航向。
     */
    attitude_get_euler(&euler);
    angle_control_set_target_yaw(euler.yaw);
}

void angle_control_set_target_yaw(float yawDeg)
{
    /*
     * 目标航向统一归一化到 -180°～180°。
     */
    g_angleControlStatus.targetYawDeg =
        angle_control_wrap_180(yawDeg);
}

void angle_control_update(float dt)
{
    attitude_euler_t euler;

    float errorDeg;
    float gyroZDps;
    float errorRateDps;
    float correctionFloat;

    int32_t correction;
    int32_t leftTarget;
    int32_t rightTarget;

    if (!g_angleControlStatus.enabled) {
        return;
    }

    /*
     * 虽然当前 D 项不再通过 dt 计算，
     * 但保留 dt 合法性检查，避免控制周期异常时继续输出。
     */
    if (dt <= 0.0f) {
        return;
    }

    /*
     * 陀螺仪还没有完成零偏校准时，不执行角度控制。
     */
    if (!attitude_is_gyro_calibrated()) {
        speed_pid_set_speed(0, 0);
        return;
    }

    attitude_get_euler(&euler);

    /*
     * 计算目标航向和当前航向之间的最短角度误差。
     *
     * 例如：
     *   target = -170°
     *   current = 170°
     *
     * 普通相减得到 -340°，
     * 归一化后得到 +20°。
     */
    errorDeg =
        angle_control_wrap_180(
            g_angleControlStatus.targetYawDeg - euler.yaw);

    /*
     * 获取经过以下处理后的 Z 轴角速度：
     *
     *   原始 gyroZ
     *   -> 零偏扣除
     *   -> LSB 转 °/s
     *   -> 死区处理
     *
     * 对于固定目标角度：
     *
     *   error = targetYaw - currentYaw
     *   d(error)/dt = -yawRate
     */
    gyroZDps = attitude_get_gyro_z_dps();
    errorRateDps = -gyroZDps;

    /*
     * 角度 PD 控制。
     *
     * P 项：
     *   根据航向误差产生转向修正。
     *
     * D 项：
     *   根据当前转动角速度提前制动，降低过冲。
     *
     * 最终单位为 mm/s。
     */
    correctionFloat =
        ANGLE_CONTROL_DIRECTION *
        (ANGLE_CONTROL_KP_MM_S_PER_DEG * errorDeg +
         ANGLE_CONTROL_KD_MM_S_PER_DPS * errorRateDps);

    correction =
        angle_control_float_to_i32(correctionFloat);

    /*
     * 对角度环差速修正量限幅。
     */
    correction =
        angle_control_limit_i32(
            correction,
            ANGLE_CONTROL_MAX_CORRECTION_MM_S);

    /*
     * 差速混控：
     *
     *   左轮 = 基础速度 - 修正量
     *   右轮 = 基础速度 + 修正量
     *
     * 原地转向时 baseSpeedMmS = 0：
     *
     *   左轮 = -correction
     *   右轮 = +correction
     */
    leftTarget =
        g_angleControlStatus.baseSpeedMmS - correction;

    rightTarget =
        g_angleControlStatus.baseSpeedMmS + correction;

    /*
     * 对最终左右轮目标速度限幅。
     */
    leftTarget =
        angle_control_limit_i32(
            leftTarget,
            ANGLE_CONTROL_MAX_TARGET_MM_S);

    rightTarget =
        angle_control_limit_i32(
            rightTarget,
            ANGLE_CONTROL_MAX_TARGET_MM_S);

    /*
     * 将角度环输出交给左右轮速度 PID。
     */
    speed_pid_set_speed(leftTarget, rightTarget);

    /*
     * 保存控制状态，供串口或 VOFA 调试。
     */
    g_angleControlStatus.currentYawDeg = euler.yaw;
    g_angleControlStatus.errorDeg = errorDeg;
    g_angleControlStatus.errorRateDps = errorRateDps;
    g_angleControlStatus.correctionMmS = correction;
    g_angleControlStatus.leftTargetMmS = leftTarget;
    g_angleControlStatus.rightTargetMmS = rightTarget;
}

void angle_control_get_status(angle_control_status_t *status)
{
    if (status == 0) {
        return;
    }

    *status = g_angleControlStatus;
}