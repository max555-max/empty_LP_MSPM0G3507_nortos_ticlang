#include "line_track.h"

#include "gray_serial.h"
#include "pid.h"

typedef enum {
    LINE_TRACK_TURN_LOST = 0,
    LINE_TRACK_TURN_STRAIGHT,
    LINE_TRACK_TURN_SMALL_LEFT,
    LINE_TRACK_TURN_SMALL_RIGHT,
    LINE_TRACK_TURN_LARGE_LEFT,
    LINE_TRACK_TURN_LARGE_RIGHT
} line_track_turn_t;

static line_track_status_t g_lineTrackStatus;

static int32_t g_baseSpeedMmS = LINE_TRACK_BASE_SPEED_MM_S;
static int32_t g_smallTurnPercent = LINE_TRACK_SMALL_TURN_INNER_PERCENT;
static int32_t g_largeTurnPercent = LINE_TRACK_LARGE_TURN_INNER_PERCENT;
static int32_t g_leftBaseBiasMmS = LINE_TRACK_LEFT_BASE_BIAS_MM_S;
static int32_t g_rightBaseBiasMmS = LINE_TRACK_RIGHT_BASE_BIAS_MM_S;

static line_track_turn_t g_lastTurn = LINE_TRACK_TURN_STRAIGHT;
static int32_t g_lastError = 0;

static bool line_track_is_active(uint8_t raw, uint8_t bit)
{
    uint8_t level = (uint8_t)((raw >> bit) & 0x01U);

    return level == LINE_TRACK_ACTIVE_LEVEL;
}

static line_track_turn_t line_track_classify(uint8_t raw,
                                              bool *lineDetected,
                                              int32_t *error)
{
    bool o1 = line_track_is_active(raw, 0U);
    bool o2 = line_track_is_active(raw, 1U);
    bool o3 = line_track_is_active(raw, 2U);
    bool o4 = line_track_is_active(raw, 3U);

    if (!o1 && !o2 && !o3 && !o4) {
        *lineDetected = false;
        *error = g_lastError;
        return LINE_TRACK_TURN_LOST;
    }

    *lineDetected = true;

    /* Both center sensors active means the line is centered. */
    if (o2 && o3) {
        *error = 0;
        return LINE_TRACK_TURN_STRAIGHT;
    }

    /* An exclusive outer sensor has priority and requests a large turn. */
    if (o1 && !o4) {
        *error = 2;
        return LINE_TRACK_TURN_LARGE_LEFT;
    }
    if (o4 && !o1) {
        *error = -2;
        return LINE_TRACK_TURN_LARGE_RIGHT;
    }

    /* A single center sensor requests a small turn. */
    if (o2 && !o3) {
        *error = 1;
        return LINE_TRACK_TURN_SMALL_LEFT;
    }
    if (o3 && !o2) {
        *error = -1;
        return LINE_TRACK_TURN_SMALL_RIGHT;
    }

    /* Symmetric or otherwise ambiguous active patterns go straight. */
    *error = 0;
    return LINE_TRACK_TURN_STRAIGHT;
}

static int32_t line_track_scale_percent(int32_t speed, int32_t percent)
{
    return (int32_t)(((int64_t)speed * percent) / 100);
}

static void line_track_apply_turn(line_track_turn_t turn,
                                  int32_t *leftTarget,
                                  int32_t *rightTarget)
{
    int32_t left = g_baseSpeedMmS + g_leftBaseBiasMmS;
    int32_t right = g_baseSpeedMmS + g_rightBaseBiasMmS;

    switch (turn) {
    case LINE_TRACK_TURN_SMALL_LEFT:
        left = line_track_scale_percent(left, g_smallTurnPercent);
        break;

    case LINE_TRACK_TURN_SMALL_RIGHT:
        right = line_track_scale_percent(right, g_smallTurnPercent);
        break;

    case LINE_TRACK_TURN_LARGE_LEFT:
        left = line_track_scale_percent(left, g_largeTurnPercent);
        break;

    case LINE_TRACK_TURN_LARGE_RIGHT:
        right = line_track_scale_percent(right, g_largeTurnPercent);
        break;

    case LINE_TRACK_TURN_STRAIGHT:
    case LINE_TRACK_TURN_LOST:
    default:
        break;
    }

    *leftTarget = left;
    *rightTarget = right;
}

static void line_track_store_and_write(uint8_t raw,
                                       bool lineDetected,
                                       int32_t error,
                                       int32_t leftTarget,
                                       int32_t rightTarget)
{
    speed_pid_set_speed(leftTarget, rightTarget);

    g_lineTrackStatus.sensorRaw = raw;
    g_lineTrackStatus.lineDetected = lineDetected;
    g_lineTrackStatus.error = error;
    g_lineTrackStatus.correction = rightTarget - leftTarget;
    g_lineTrackStatus.leftTargetMmS = leftTarget;
    g_lineTrackStatus.rightTargetMmS = rightTarget;
}

static void line_track_update_with_raw_impl(uint8_t raw, bool stopOnLost)
{
    bool lineDetected;
    int32_t error;
    int32_t leftTarget;
    int32_t rightTarget;
    line_track_turn_t turn = line_track_classify(raw, &lineDetected, &error);

    if (lineDetected) {
        g_lastTurn = turn;
        g_lastError = error;
        line_track_apply_turn(turn, &leftTarget, &rightTarget);
    } else if (stopOnLost) {
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

    line_track_store_and_write(raw, lineDetected, error,
                               leftTarget, rightTarget);
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
    g_smallTurnPercent = LINE_TRACK_SMALL_TURN_INNER_PERCENT;
    g_largeTurnPercent = LINE_TRACK_LARGE_TURN_INNER_PERCENT;
    g_leftBaseBiasMmS = LINE_TRACK_LEFT_BASE_BIAS_MM_S;
    g_rightBaseBiasMmS = LINE_TRACK_RIGHT_BASE_BIAS_MM_S;
    g_lastTurn = LINE_TRACK_TURN_STRAIGHT;
    g_lastError = 0;
}

void line_track_set_base_speed(int32_t baseSpeedMmS)
{
    g_baseSpeedMmS = baseSpeedMmS;
}

bool line_track_set_turn_ratios(int32_t smallTurnPercent,
                                int32_t largeTurnPercent)
{
    if ((smallTurnPercent < 0) || (smallTurnPercent > 100) ||
        (largeTurnPercent < 0) || (largeTurnPercent > 100) ||
        (largeTurnPercent > smallTurnPercent)) {
        return false;
    }

    g_smallTurnPercent = smallTurnPercent;
    g_largeTurnPercent = largeTurnPercent;
    return true;
}

void line_track_set_left_base_bias(int32_t leftBiasMmS)
{
    g_leftBaseBiasMmS = leftBiasMmS;
}

void line_track_set_right_base_bias(int32_t rightBiasMmS)
{
    g_rightBaseBiasMmS = rightBiasMmS;
}

void line_track_set_base_bias(int32_t leftBiasMmS, int32_t rightBiasMmS)
{
    g_leftBaseBiasMmS = leftBiasMmS;
    g_rightBaseBiasMmS = rightBiasMmS;
}

int32_t line_track_get_base_speed(void)
{
    return g_baseSpeedMmS;
}

int32_t line_track_get_small_turn_percent(void)
{
    return g_smallTurnPercent;
}

int32_t line_track_get_large_turn_percent(void)
{
    return g_largeTurnPercent;
}

int32_t line_track_get_left_base_bias(void)
{
    return g_leftBaseBiasMmS;
}

int32_t line_track_get_right_base_bias(void)
{
    return g_rightBaseBiasMmS;
}

void line_track_update(void)
{
    line_track_update_with_raw_search_on_lost(gray_serial_read());
}

void line_track_update_with_raw(uint8_t raw)
{
    line_track_update_with_raw_impl(raw, true);
}

void line_track_update_with_raw_search_on_lost(uint8_t raw)
{
    line_track_update_with_raw_impl(raw, false);
}

void line_track_update_with_raw_hold_on_lost(uint8_t raw)
{
    bool lineDetected;
    int32_t error;
    int32_t leftTarget;
    int32_t rightTarget;
    line_track_turn_t turn = line_track_classify(raw, &lineDetected, &error);

    if (lineDetected) {
        g_lastTurn = turn;
        g_lastError = error;
    } else {
        turn = g_lastTurn;
        error = g_lastError;
    }

    line_track_apply_turn(turn, &leftTarget, &rightTarget);
    line_track_store_and_write(raw, lineDetected, error,
                               leftTarget, rightTarget);
}

void line_track_get_status(line_track_status_t *status)
{
    if (status != 0) {
        *status = g_lineTrackStatus;
    }
}
