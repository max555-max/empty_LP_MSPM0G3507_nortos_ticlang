#include "angle_control.h"

#include "attitude.h"
#include "pid.h"

/*
 * angle_control.c
 *
 * 角度环实现文件。
 * 它不直接输出 PWM，而是把“航向误差”转换成左右轮目标速度差，
 * 再交给速度 PID 控制左右电机。
 */

/* 角度环当前状态，既用于控制，也用于调试读取。 */
static angle_control_status_t g_angleControlStatus;

/* 上一次角度误差，用于计算 D 项。 */
static float g_lastErrorDeg = 0.0f;

/* 标记 g_lastErrorDeg 是否有效，避免第一次计算 D 项时出现突变。 */
static bool g_hasLastError = false;

/* 将角度归一化到 -180~180，避免 179° 到 -179° 附近误差跳变。 */
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

/* 对 int32_t 数值做正负限幅。limit=220 表示限制在 -220~220。 */
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

/* float 转 int32_t，带四舍五入，避免直接截断带来小误差。 */
static int32_t angle_control_float_to_i32(float value)
{
    if (value >= 0.0f) {
        return (int32_t) (value + 0.5f);
    }

    return (int32_t) (value - 0.5f);
}

void angle_control_init(void)
{
    /* 清空所有状态，保证上电后角度环不会残留旧目标。 */
    g_angleControlStatus.enabled = false;
    g_angleControlStatus.targetYawDeg = 0.0f;
    g_angleControlStatus.currentYawDeg = 0.0f;
    g_angleControlStatus.errorDeg = 0.0f;
    g_angleControlStatus.errorRateDps = 0.0f;
    g_angleControlStatus.baseSpeedMmS = 0;
    g_angleControlStatus.correctionMmS = 0;
    g_angleControlStatus.leftTargetMmS = 0;
    g_angleControlStatus.rightTargetMmS = 0;

    g_lastErrorDeg = 0.0f;
    g_hasLastError = false;

    /* 初始化时明确让速度环目标为 0，防止电机意外启动。 */
    speed_pid_set_speed(0, 0);
}

void angle_control_enable(bool enable)
{
    /* 每次重新使能都清掉 D 项历史，避免切状态时微分冲击。 */
    g_angleControlStatus.enabled = enable;
    g_hasLastError = false;

    if (!enable) {
        /* 关闭角度环时同步清零速度目标。 */
        speed_pid_set_speed(0, 0);
        g_angleControlStatus.correctionMmS = 0;
        g_angleControlStatus.leftTargetMmS = 0;
        g_angleControlStatus.rightTargetMmS = 0;
    }
}

void angle_control_stop(void)
{
    /* stop 比 enable(false) 更彻底：基础速度和输出状态也全部清零。 */
    g_angleControlStatus.enabled = false;
    g_angleControlStatus.baseSpeedMmS = 0;
    g_angleControlStatus.correctionMmS = 0;
    g_angleControlStatus.leftTargetMmS = 0;
    g_angleControlStatus.rightTargetMmS = 0;
    g_hasLastError = false;

    speed_pid_set_speed(0, 0);
}

void angle_control_set_base_speed(int32_t baseSpeedMmS)
{
    /* 基础速度也要限幅，否则 correction 叠加后容易超过速度环能力。 */
    g_angleControlStatus.baseSpeedMmS =
        angle_control_limit_i32(baseSpeedMmS, ANGLE_CONTROL_MAX_TARGET_MM_S);
}

void angle_control_lock_current_yaw(void)
{
    attitude_euler_t euler;

    /* 读取当前姿态，把当前 yaw 当成接下来要保持的目标方向。 */
    attitude_get_euler(&euler);
    angle_control_set_target_yaw(euler.yaw);
}

void angle_control_set_target_yaw(float yawDeg)
{
    /* 目标角度统一归一化，后面误差计算会更稳定。 */
    g_angleControlStatus.targetYawDeg = angle_control_wrap_180(yawDeg);

    /* 目标改变时清掉 D 项历史，避免目标阶跃导致微分突变。 */
    g_lastErrorDeg = 0.0f;
    g_hasLastError = false;
}

void angle_control_update(float dt)
{
    attitude_euler_t euler;
    float errorDeg;
    float errorRateDps = 0.0f;
    float correctionFloat;
    int32_t correction;
    int32_t leftTarget;
    int32_t rightTarget;

    if (!g_angleControlStatus.enabled) {
        /* 未使能时不改速度目标，由 stop/enable 负责清零。 */
        return;
    }

    if (dt <= 0.0f) {
        /* dt 异常时不计算 D 项，避免除 0。 */
        return;
    }

    attitude_get_euler(&euler);

    /*
     * error > 0 表示“目标角度比当前角度大”。
     * 如果实车修正方向相反，不要改这里的公式，优先改头文件里的
     * ANGLE_CONTROL_DIRECTION。
     */
    errorDeg =
        angle_control_wrap_180(g_angleControlStatus.targetYawDeg - euler.yaw);

    if (g_hasLastError) {
        /* D 项：看误差变化速度，用于抑制左右摆动。 */
        errorRateDps = (errorDeg - g_lastErrorDeg) / dt;
    } else {
        /* 第一次没有历史误差，D 项强制为 0。 */
        errorRateDps = 0.0f;
        g_hasLastError = true;
    }
    g_lastErrorDeg = errorDeg;

    /* 角度 PD 输出，单位是 mm/s 的差速修正量。 */
    correctionFloat =
        ANGLE_CONTROL_DIRECTION *
        (ANGLE_CONTROL_KP_MM_S_PER_DEG * errorDeg +
         ANGLE_CONTROL_KD_MM_S_PER_DPS * errorRateDps);

    correction = angle_control_float_to_i32(correctionFloat);

    /* 差速限幅：防止角度误差大时直接把速度目标打满。 */
    correction =
        angle_control_limit_i32(correction,
                                ANGLE_CONTROL_MAX_CORRECTION_MM_S);

    /*
     * 差速混控：
     *   左轮减 correction，右轮加 correction。
     * 如果实车方向相反，改 ANGLE_CONTROL_DIRECTION。
     */
    leftTarget = g_angleControlStatus.baseSpeedMmS - correction;
    rightTarget = g_angleControlStatus.baseSpeedMmS + correction;

    /* 最终目标速度限幅，保护速度环和电机。 */
    leftTarget =
        angle_control_limit_i32(leftTarget, ANGLE_CONTROL_MAX_TARGET_MM_S);
    rightTarget =
        angle_control_limit_i32(rightTarget, ANGLE_CONTROL_MAX_TARGET_MM_S);

    /* 把角度环输出下发给速度环。 */
    speed_pid_set_speed(leftTarget, rightTarget);

    /* 保存状态，方便串口/VOFA 调试。 */
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
        /* 防止空指针。 */
        return;
    }

    *status = g_angleControlStatus;
}
