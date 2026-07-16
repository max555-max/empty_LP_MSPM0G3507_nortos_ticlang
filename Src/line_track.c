#include "line_track.h"

#include "gray_serial.h"
#include "pid.h"

static line_track_status_t g_lineTrackStatus;
static int32_t g_baseSpeedMmS = LINE_TRACK_BASE_SPEED_MM_S;
static int32_t g_lastError = 0;
static int32_t WEIFEN = 0;
static bool g_hasPreviousError = false;
static int32_t g_previousError = 0;

#define LINE_TRACK_P_TERM_LIMIT_MM_S \
    ((LINE_TRACK_MAX_CORRECTION_MM_S * 3) / 4)
#define LINE_TRACK_D_TERM_LIMIT_MM_S \
    (LINE_TRACK_MAX_CORRECTION_MM_S - LINE_TRACK_P_TERM_LIMIT_MM_S)

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

static bool line_track_is_active(uint8_t raw, uint8_t bit)
{
    uint8_t level = (uint8_t) ((raw >> bit) & 0x01U);

    return level == LINE_TRACK_ACTIVE_LEVEL;
}

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

void line_track_update(void)
{
    uint8_t raw = gray_serial_read();
    bool lineDetected;

    int32_t error = line_track_calculate_error(raw, &lineDetected);
    int32_t correction;
    int32_t leftTarget;
    int32_t rightTarget;

    if (lineDetected) {
        int32_t pTerm;
        int32_t dTerm;

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

        pTerm = (int32_t) (((int64_t) error * LINE_TRACK_TURN_KP) / 1000);
        dTerm = (int32_t) (((int64_t) WEIFEN * LINE_TRACK_TURN_KD) / 1000);

        pTerm = line_track_limit(pTerm, LINE_TRACK_P_TERM_LIMIT_MM_S);
        dTerm = line_track_limit(dTerm, LINE_TRACK_D_TERM_LIMIT_MM_S);

        correction = pTerm + dTerm;
        correction = line_track_limit(
            correction,
            LINE_TRACK_MAX_CORRECTION_MM_S);

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

        if (g_lastError < 0) {
            leftTarget = -LINE_TRACK_LOST_SEARCH_SPEED_MM_S;
            rightTarget = LINE_TRACK_LOST_SEARCH_SPEED_MM_S;
        } else if (g_lastError > 0) {
            leftTarget = LINE_TRACK_LOST_SEARCH_SPEED_MM_S;
            rightTarget = -LINE_TRACK_LOST_SEARCH_SPEED_MM_S;
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

void line_track_get_status(line_track_status_t *status)
{
    if (status == 0) {
        return;
    }

    *status = g_lineTrackStatus;
}
