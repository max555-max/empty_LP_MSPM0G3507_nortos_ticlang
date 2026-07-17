#include "angle_control.h"

#include "attitude.h"
#include "pid.h"

static angle_control_status_t g_angleControlStatus;
static float g_lastErrorDeg = 0.0f;
static bool g_hasLastError = false;

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

static int32_t angle_control_float_to_i32(float value)
{
    if (value >= 0.0f) {
        return (int32_t) (value + 0.5f);
    }

    return (int32_t) (value - 0.5f);
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

    g_lastErrorDeg = 0.0f;
    g_hasLastError = false;

    speed_pid_set_speed(0, 0);
}

void angle_control_enable(bool enable)
{
    g_angleControlStatus.enabled = enable;
    g_hasLastError = false;

    if (!enable) {
        speed_pid_set_speed(0, 0);
        g_angleControlStatus.correctionMmS = 0;
        g_angleControlStatus.leftTargetMmS = 0;
        g_angleControlStatus.rightTargetMmS = 0;
    }
}

void angle_control_stop(void)
{
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
    g_angleControlStatus.baseSpeedMmS =
        angle_control_limit_i32(baseSpeedMmS, ANGLE_CONTROL_MAX_TARGET_MM_S);
}

void angle_control_lock_current_yaw(void)
{
    attitude_euler_t euler;

    attitude_get_euler(&euler);
    angle_control_set_target_yaw(euler.yaw);
}

void angle_control_set_target_yaw(float yawDeg)
{
    g_angleControlStatus.targetYawDeg = angle_control_wrap_180(yawDeg);
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
        return;
    }

    if (dt <= 0.0f) {
        return;
    }

    attitude_get_euler(&euler);

    /*
     * error > 0 means current yaw is smaller than target yaw.
     * The correction sign can be flipped with ANGLE_CONTROL_DIRECTION if the
     * actual chassis steering direction is opposite.
     */
    errorDeg =
        angle_control_wrap_180(g_angleControlStatus.targetYawDeg - euler.yaw);

    if (g_hasLastError) {
        errorRateDps = (errorDeg - g_lastErrorDeg) / dt;
    } else {
        errorRateDps = 0.0f;
        g_hasLastError = true;
    }
    g_lastErrorDeg = errorDeg;

    correctionFloat =
        ANGLE_CONTROL_DIRECTION *
        (ANGLE_CONTROL_KP_MM_S_PER_DEG * errorDeg +
         ANGLE_CONTROL_KD_MM_S_PER_DPS * errorRateDps);

    correction = angle_control_float_to_i32(correctionFloat);
    correction =
        angle_control_limit_i32(correction,
                                ANGLE_CONTROL_MAX_CORRECTION_MM_S);

    leftTarget = g_angleControlStatus.baseSpeedMmS - correction;
    rightTarget = g_angleControlStatus.baseSpeedMmS + correction;

    leftTarget =
        angle_control_limit_i32(leftTarget, ANGLE_CONTROL_MAX_TARGET_MM_S);
    rightTarget =
        angle_control_limit_i32(rightTarget, ANGLE_CONTROL_MAX_TARGET_MM_S);

    speed_pid_set_speed(leftTarget, rightTarget);

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
