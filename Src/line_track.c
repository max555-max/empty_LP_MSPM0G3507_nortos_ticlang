#include "line_track.h"

#include "gray_serial.h"
#include "pid.h"

/* 保存循迹当前状态，供 OLED 和调试接口读取。 */
static line_track_status_t g_lineTrackStatus;

/* 运行时参数：默认来自 line_track.h，也可以通过蓝牙动态修改。 */
static int32_t g_baseSpeedMmS = LINE_TRACK_BASE_SPEED_MM_S;
static int32_t g_turnKp = LINE_TRACK_TURN_KP;
static int32_t g_turnKd = LINE_TRACK_TURN_KD;
static int32_t g_maxCorrectionMmS = LINE_TRACK_MAX_CORRECTION_MM_S;

/* 历史误差，用于 D 项和丢线找线方向判断。 */
static int32_t g_lastError = 0;
static int32_t WEIFEN = 0;
static bool g_hasPreviousError = false;
static int32_t g_previousError = 0;

static void line_track_update_with_raw_impl(uint8_t raw, bool stopOnLost);

static int32_t line_track_abs(int32_t value)
{
    if (value < 0) {
        return -value;
    }

    return value;
}

/* 对数值做正负限幅。 */
static int32_t line_track_limit(int32_t value, int32_t limit)
{
    if (value > limit) {
        return limit;
    }

    if (value < -limit) {
        return -limit;
    }

    return value;
}

/* 中心死区：小偏差不参与控制，超过死区后只保留超出的部分。 */
static int32_t line_track_apply_deadband(int32_t error)
{
    if (error > LINE_TRACK_ERROR_DEADBAND) {
        return error - LINE_TRACK_ERROR_DEADBAND;
    }

    if (error < -LINE_TRACK_ERROR_DEADBAND) {
        return error + LINE_TRACK_ERROR_DEADBAND;
    }

    return 0;
}

/* 大弯增强：线压到外侧传感器附近时，临时增加差速余量。 */
static int32_t line_track_curve_extra_correction(int32_t error)
{
    if (line_track_abs(error) >= LINE_TRACK_CURVE_ERROR_THRESHOLD) {
        return LINE_TRACK_CURVE_EXTRA_CORRECTION_MM_S;
    }

    return 0;
}

/* 大弯降速：线压到外侧传感器附近时，临时降低基础速度。 */
static int32_t line_track_curve_base_reduce(int32_t error)
{
    if (line_track_abs(error) >= LINE_TRACK_CURVE_ERROR_THRESHOLD) {
        return LINE_TRACK_CURVE_BASE_REDUCE_MM_S;
    }

    return 0;
}

/* 大弯时降低基础速度，避免外侧轮最高速度继续增加。 */
static int32_t line_track_reduce_forward_base(int32_t baseSpeed, int32_t reduce)
{
    if (baseSpeed <= 0) {
        return baseSpeed;
    }

    if (baseSpeed <= reduce) {
        return 0;
    }

    return baseSpeed - reduce;
}

/* 判断某个原始 bit 是否表示检测到黑线。 */
static bool line_track_is_active(uint8_t raw, uint8_t bit)
{
    uint8_t level = (uint8_t)((raw >> bit) & 0x01U);

    return level == LINE_TRACK_ACTIVE_LEVEL;
}

/* 根据 8 路传感器加权平均计算循迹误差。 */
static int32_t line_track_calculate_error(uint8_t raw, bool *lineDetected)
{
    /* 串行读回 bit 顺序为 1,2,3,4,5,6,7,0，对应物理 ch1..ch8。 */
    static const uint8_t channelBitMap[8] = {
        1U, 2U, 3U, 4U, 5U, 6U, 7U, 0U
    };
    static const int16_t channelWeight[8] = {
        4250, 3036, 1821, 607, -607, -1821, -3036, -4250
    };

    int32_t weightedSum = 0;
    int32_t activeCount = 0;

    for (uint8_t i = 0U; i < 8U; i++) {
        if (line_track_is_active(raw, channelBitMap[i])) {
            weightedSum += channelWeight[i];
            activeCount++;
        }
    }

    if (activeCount == 0) {
        *lineDetected = false;
        return g_lastError;
    }

    *lineDetected = true;
    return weightedSum / activeCount;
}

void line_track_init(void)
{
    g_lineTrackStatus.sensorRaw = 0U;
    g_lineTrackStatus.lineDetected = false;
    g_lineTrackStatus.error = 0;
    g_lineTrackStatus.correction = 0;
    g_lineTrackStatus.leftTargetMmS = 0;
    g_lineTrackStatus.rightTargetMmS = 0;

    g_baseSpeedMmS = LINE_TRACK_BASE_SPEED_MM_S;

    g_lastError = 0;
    g_previousError = 0;
    WEIFEN = 0;
    g_hasPreviousError = false;
}

void line_track_set_base_speed(int32_t baseSpeedMmS)
{
    g_baseSpeedMmS = baseSpeedMmS;
}

void line_track_set_turn_gains(int32_t kp, int32_t kd)
{
    g_turnKp = kp;
    g_turnKd = kd;
}

void line_track_set_turn_kp(int32_t kp)
{
    g_turnKp = kp;
}

void line_track_set_turn_kd(int32_t kd)
{
    g_turnKd = kd;
}

void line_track_set_max_correction(int32_t maxCorrectionMmS)
{
    if (maxCorrectionMmS < 0) {
        maxCorrectionMmS = -maxCorrectionMmS;
    }

    g_maxCorrectionMmS = maxCorrectionMmS;
}

int32_t line_track_get_base_speed(void)
{
    return g_baseSpeedMmS;
}

int32_t line_track_get_turn_kp(void)
{
    return g_turnKp;
}

int32_t line_track_get_turn_kd(void)
{
    return g_turnKd;
}

int32_t line_track_get_max_correction(void)
{
    return g_maxCorrectionMmS;
}

void line_track_update(void)
{
    uint8_t raw = gray_serial_read();

    line_track_update_with_raw_search_on_lost(raw);
}

static void line_track_update_with_raw_impl(uint8_t raw, bool stopOnLost)
{
    bool lineDetected;

    int32_t error = line_track_calculate_error(raw, &lineDetected);
    int32_t controlError = line_track_apply_deadband(error);
    int32_t correction;
    int32_t leftTarget;
    int32_t rightTarget;

    if (lineDetected) {
        int32_t pTerm;
        int32_t dTerm;
        int32_t pTermLimit;
        int32_t dTermLimit;
        int32_t curveExtraCorrection =
            line_track_curve_extra_correction(error);
        int32_t curveBaseReduce = line_track_curve_base_reduce(error);
        int32_t correctionLimit =
            g_maxCorrectionMmS + curveExtraCorrection;
        int32_t mixedBaseSpeed =
            line_track_reduce_forward_base(g_baseSpeedMmS, curveBaseReduce);

        if (g_hasPreviousError) {
            WEIFEN = controlError - g_previousError;
            WEIFEN = line_track_limit(WEIFEN, 1600);
        } else {
            WEIFEN = 0;
            g_hasPreviousError = true;
        }

        g_previousError = controlError;
        g_lastError = error;

        pTerm = (int32_t)(((int64_t)controlError * g_turnKp) / 1000);
        dTerm = (int32_t)(((int64_t)WEIFEN * g_turnKd) / 1000);

        if (g_turnKd == 0) {
            pTermLimit = correctionLimit;
            dTermLimit = 0;
        } else {
            pTermLimit = (correctionLimit * 3) / 4;
            dTermLimit = correctionLimit - pTermLimit;
        }

        pTerm = line_track_limit(pTerm, pTermLimit);
        dTerm = line_track_limit(dTerm, dTermLimit);

        correction = pTerm + dTerm;
        correction = line_track_limit(correction, correctionLimit);

        leftTarget = mixedBaseSpeed - correction;
        rightTarget = mixedBaseSpeed + correction;
    } else {
        correction = 0;
        WEIFEN = 0;
        g_hasPreviousError = false;

        if (stopOnLost) {
            leftTarget = 0;
            rightTarget = 0;
        } else if (g_lastError < 0) {
            leftTarget = LINE_TRACK_LOST_SEARCH_SPEED_MM_S;
            rightTarget = -LINE_TRACK_LOST_SEARCH_SPEED_MM_S;
        } else if (g_lastError > 0) {
            leftTarget = -LINE_TRACK_LOST_SEARCH_SPEED_MM_S;
            rightTarget = LINE_TRACK_LOST_SEARCH_SPEED_MM_S;
        } else {
            leftTarget = 0;
            rightTarget = 0;
        }
    }

    speed_pid_set_speed(leftTarget, rightTarget);

    g_lineTrackStatus.sensorRaw = raw;
    g_lineTrackStatus.lineDetected = lineDetected;
    g_lineTrackStatus.error = error;
    g_lineTrackStatus.correction = correction;
    g_lineTrackStatus.leftTargetMmS = leftTarget;
    g_lineTrackStatus.rightTargetMmS = rightTarget;
}

void line_track_update_with_raw(uint8_t raw)
{
    line_track_update_with_raw_impl(raw, true);
}

void line_track_update_with_raw_search_on_lost(uint8_t raw)
{
    line_track_update_with_raw_impl(raw, false);
}

void line_track_get_status(line_track_status_t *status)
{
    if (status == 0) {
        return;
    }

    *status = g_lineTrackStatus;
}