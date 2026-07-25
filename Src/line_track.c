#include "line_track.h"

#include "gray_serial.h"
#include "pid.h"

/*
 * line_track.c
 *
 * 八路灰度循迹控制。
 *
 * 主要流程：
 *   1. 读取 8 路灰度 raw；
 *   2. 按“通道 1~8 从左到右”的映射取出每个通道；
 *   3. 对检测到黑线的通道做加权平均，得到 error；
 *   4. 使用 PD 计算 correction；
 *   5. 把左右轮目标速度交给 speed_pid。
 */

/* 保存循迹当前状态，便于调试读取。 */
static line_track_status_t g_lineTrackStatus;

/* 循迹基础速度，上层任务可动态修改。 */
static int32_t g_baseSpeedMmS = LINE_TRACK_BASE_SPEED_MM_S;
static int32_t g_turnKp = LINE_TRACK_TURN_KP;
static int32_t g_turnKd = LINE_TRACK_TURN_KD;
static int32_t g_maxCorrectionMmS = LINE_TRACK_MAX_CORRECTION_MM_S;

/* 最近一次有效误差。丢线时用于判断往哪边找线。 */
static int32_t g_lastError = 0;

/* 微分项原始差值：本次误差 - 上次误差。 */
static int32_t WEIFEN = 0;

/* 是否已有上一帧误差，避免第一帧微分突变。 */
static bool g_hasPreviousError = false;

/* 上一帧误差。 */
static int32_t g_previousError = 0;

/* 将 P 项和 D 项分开限幅，防止 D 项尖峰把输出打满。 */
/* 内部统一实现，stopOnLost 决定丢线时停车还是旋转找线。 */
static void line_track_update_with_raw_impl(uint8_t raw, bool stopOnLost);

/* 对循迹修正量做正负限幅。 */
static int32_t line_track_limit(int32_t value, int32_t limit)//输出限幅
{
    if (value > limit) {
        return limit;
    }

    if (value < -limit) {
        return -limit;
    }

    return value;
}

/*
 * 判断某个原始 bit 是否为“检测到黑线”。
 *
 * LINE_TRACK_ACTIVE_LEVEL 在 line_track.h 中配置：
 *   1 表示高电平为黑线；
 *   0 表示低电平为黑线。
 */
static bool line_track_is_active(uint8_t raw, uint8_t bit)
{
    uint8_t level = (uint8_t) ((raw >> bit) & 0x01U);

    return level == LINE_TRACK_ACTIVE_LEVEL;
}

/*
 * 根据 8 路灰度计算循迹误差。
 *
 * channelBitMap：
 *   传感器实际从左到右是通道 1~8；
 *   但串行读回的原始 bit 顺序为 1,2,3,4,5,6,7,0。
 *
 * channelWeight：
 *   左侧通道为正，右侧通道为负；
 *   黑线在中心时，正负权重抵消，error 接近 0。
 */
static int32_t line_track_calculate_error(uint8_t raw, bool *lineDetected)
{
    /*
     * gray_serial_print() 中已经确认过传感器从左到右为通道 1~8，
     * 对应原始 bit 顺序为 1,2,3,4,5,6,7,0。
     */
    static const uint8_t channelBitMap[8] = {
        1U, 2U, 3U, 4U, 5U, 6U, 7U, 0U
    };
    static const int16_t channelWeight[8] = {
        4250, 3036, 1821, 607, -607, -1821, -3036, -4250
    };

    int32_t weightedSum = 0;
    int32_t activeCount = 0;

    /* 遍历 8 个实际通道，只统计检测到黑线的通道。 */
    for (uint8_t i = 0U; i < 8U; i++) {
        if (line_track_is_active(raw, channelBitMap[i])) {
            weightedSum += channelWeight[i];
            activeCount++;
        }
    }

    if (activeCount == 0) {
        /* 没有任何通道检测到线，返回上一次误差，便于丢线找线判断方向。 */
        *lineDetected = false;
        return g_lastError;
    }

    /* 多个通道同时压线时，取平均位置作为黑线中心。 */
    *lineDetected = true;
    return weightedSum / activeCount;
}

void line_track_init(void)
{
    /* 清空调试状态。 */
    g_lineTrackStatus.sensorRaw = 0U;
    g_lineTrackStatus.lineDetected = false;
    g_lineTrackStatus.error = 0;
    g_lineTrackStatus.correction = 0;
    g_lineTrackStatus.leftTargetMmS = 0;
    g_lineTrackStatus.rightTargetMmS = 0;

    /* 恢复默认基础速度和历史误差。 */
    g_baseSpeedMmS = LINE_TRACK_BASE_SPEED_MM_S;

    g_lastError = 0;
    g_previousError = 0;
    WEIFEN = 0;
    g_hasPreviousError = false;
}

void line_track_set_base_speed(int32_t baseSpeedMmS)
{
    /* 上层任务可根据路段切换低速/高速。 */
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
    /* 默认接口：自己读取灰度，然后执行“丢线找线”版本。 */
    uint8_t raw = gray_serial_read();

    line_track_update_with_raw_search_on_lost(raw);
}

static void line_track_update_with_raw_impl(uint8_t raw, bool stopOnLost)
{
    bool lineDetected;

    int32_t error = line_track_calculate_error(raw, &lineDetected);
    int32_t correction;
    int32_t leftTarget;
    int32_t rightTarget;

    if (lineDetected) {
        int32_t pTerm;
        int32_t dTerm;
        int32_t pTermLimit;
        int32_t dTermLimit;

        /*
         * 第一次识线或丢线后重新识线时，
         * 不计算微分项，避免微分冲击。
         */
        if (g_hasPreviousError) {
            WEIFEN = error - g_previousError;
            WEIFEN = line_track_limit(WEIFEN, 1600);
        } else {
            WEIFEN = 0;
            g_hasPreviousError = true;
        }

        g_previousError = error;
        g_lastError = error;

        /* P 项：当前位置偏差越大，差速越大。 */
        pTerm = (int32_t) (((int64_t) error * g_turnKp) / 1000);

        /* D 项：偏差变化越快，给一个反向阻尼，抑制摆动。 */
        dTerm = (int32_t) (((int64_t) WEIFEN * g_turnKd) / 1000);

        /* P/D 分别限幅，再合成总修正量。 */
        pTermLimit = (g_maxCorrectionMmS * 3) / 4;
        dTermLimit = g_maxCorrectionMmS - pTermLimit;
        pTerm = line_track_limit(pTerm, pTermLimit);
        dTerm = line_track_limit(dTerm, dTermLimit);

        correction = pTerm + dTerm;
        correction = line_track_limit(correction, g_maxCorrectionMmS);

        /*
         * 差速混控：
         *   error 为正表示线偏左，需要向左修正；
         *   左轮加速、右轮减速。
         */
        leftTarget = g_baseSpeedMmS + correction;
        rightTarget = g_baseSpeedMmS - correction;
    } else {
        correction = 0;

        /*
         * 丢线后清除微分历史。
         * 下一次重新识线时，第一次 D 项为 0。
         */
        WEIFEN = 0;
        g_hasPreviousError = false;

        if (stopOnLost) {
            /* 某些任务希望丢线立即停车，例如最终停车段。 */
            leftTarget = 0;
            rightTarget = 0;
        } else if (g_lastError < 0) {
            /* 上次线在右边：原地向右侧找线。 */
            leftTarget = -LINE_TRACK_LOST_SEARCH_SPEED_MM_S;
            rightTarget = LINE_TRACK_LOST_SEARCH_SPEED_MM_S;
        } else if (g_lastError > 0) {
            /* 上次线在左边：原地向左侧找线。 */
            leftTarget = LINE_TRACK_LOST_SEARCH_SPEED_MM_S;
            rightTarget = -LINE_TRACK_LOST_SEARCH_SPEED_MM_S;
        } else {
            /* 没有历史方向时，不盲目找线。 */
            leftTarget = 0;
            rightTarget = 0;
        }
    }

    /* 把循迹得到的左右轮目标速度交给速度环。 */
    speed_pid_set_speed(leftTarget, rightTarget);

    /* 保存调试状态。 */
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
