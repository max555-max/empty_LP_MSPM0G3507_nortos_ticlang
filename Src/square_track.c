#include "square_track.h"

#include "delay.h"
#include "encoder.h"
#include "gray_serial.h"
#include "pid.h"

#define SQUARE_TRACK_CONTROL_PERIOD_MS          (10U)

/*
 * Set to 1 if the gray sensor outputs high level on black line.
 * Set to 0 if the gray sensor outputs low level on black line.
 */
#define SQUARE_TRACK_ACTIVE_LEVEL               (1U)

/*
 * Normal line tracking parameters.
 *
 * Sensor layout:
 *   ch1 ch2 ch3 ch4 ch5 ch6 ch7 ch8
 *    L                       center                       R
 *
 * Error definition:
 *   center = 0
 *   left   = positive
 *   right  = negative
 *
 * PID output definition:
 *   correction > 0 means turn left:
 *       left wheel target  = base - correction
 *       right wheel target = base + correction
 */
#define SQUARE_TRACK_BASE_SPEED_MM_S            (400)
#define SQUARE_TRACK_LINE_KP                    (100)
#define SQUARE_TRACK_MAX_CORRECTION_MM_S        (350)
#define SQUARE_TRACK_LOST_SEARCH_SPEED_MM_S     (160)

/*
 * Right-angle corner handling parameters.
 */
#define SQUARE_TRACK_FORWARD_DISTANCE_MM        (70)
#define SQUARE_TRACK_TURN_RIGHT_WHEEL_MM_S      (280)
#define SQUARE_TRACK_CORNER_DEBOUNCE_COUNT      (3U)
#define SQUARE_TRACK_CENTER_DEBOUNCE_COUNT      (2U)
#define SQUARE_TRACK_TURN_MIN_TIME_MS           (180U)
#define SQUARE_TRACK_COOLDOWN_DISTANCE_MM       (100)

#define SQUARE_TRACK_PI_X1000000                (3141593LL)
#define SQUARE_TRACK_CPR_X1000                  ((int64_t) \
    ENCODER_LINES_PER_MOTOR_REV * ENCODER_QUADRATURE_MULTIPLIER * \
    ENCODER_GEAR_RATIO_X1000)
#define SQUARE_TRACK_CIRCUM_MM_X1000            (((int64_t) \
    ENCODER_WHEEL_DIAMETER_MM * SQUARE_TRACK_PI_X1000000) / 1000LL)

static square_track_status_t g_squareTrackStatus;

static square_track_state_t g_state = SQUARE_TRACK_STATE_TRACKING;
static int32_t g_lastError = 0;
static int32_t g_forwardStartLeftCount = 0;
static int32_t g_forwardStartRightCount = 0;
static int32_t g_cooldownStartLeftCount = 0;
static int32_t g_cooldownStartRightCount = 0;
static uint32_t g_stateTimeMs = 0;
static uint8_t g_cornerDebounce = 0;
static uint8_t g_centerDebounce = 0;
static bool g_cornerCooldownActive = false;
static uint32_t g_cornerCount = 0;

static int32_t square_track_abs_i32(int32_t value)
{
    return (value >= 0) ? value : -value;
}

static int32_t square_track_limit_i32(int32_t value, int32_t limit)
{
    if (value > limit) {
        return limit;
    }

    if (value < -limit) {
        return -limit;
    }

    return value;
}

static bool square_track_is_active(uint8_t raw, uint8_t bit)
{
    uint8_t level = (uint8_t) ((raw >> bit) & 0x01U);

    return level == SQUARE_TRACK_ACTIVE_LEVEL;
}

static bool square_track_channel_active(uint8_t raw, uint8_t channelIndex)
{
    static const uint8_t channelBitMap[8] = {
        1U, 2U, 3U, 4U, 5U, 6U, 7U, 0U
    };

    return square_track_is_active(raw, channelBitMap[channelIndex]);
}

static bool square_track_left_three_active(uint8_t raw)
{
    return square_track_channel_active(raw, 0U) &&
           square_track_channel_active(raw, 1U) &&
           square_track_channel_active(raw, 2U);
}

static bool square_track_center_two_active(uint8_t raw)
{
    return square_track_channel_active(raw, 3U) &&
           square_track_channel_active(raw, 4U);
}

static int32_t square_track_calculate_error(uint8_t raw, bool *lineDetected)
{
    static const int16_t channelWeight[8] = {
         4250,  3036,  1821,  607,
         -607, -1821, -3036, -4250
    };

    int32_t weightedSum = 0;
    int32_t activeCount = 0;

    for (uint8_t i = 0U; i < 8U; i++) {
        if (square_track_channel_active(raw, i)) {
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

static int32_t square_track_counts_to_mm(int32_t counts)
{
    int64_t distance =
        (int64_t) square_track_abs_i32(counts) * SQUARE_TRACK_CIRCUM_MM_X1000;

    distance += SQUARE_TRACK_CPR_X1000 / 2;
    distance /= SQUARE_TRACK_CPR_X1000;

    return (int32_t) distance;
}

static int32_t square_track_average_distance_from(
    int32_t startLeftCount,
    int32_t startRightCount)
{
    int32_t leftDelta = encoder_get_left_count() - startLeftCount;
    int32_t rightDelta = encoder_get_right_count() - startRightCount;
    int32_t leftDistance = square_track_counts_to_mm(leftDelta);
    int32_t rightDistance = square_track_counts_to_mm(rightDelta);

    return (leftDistance + rightDistance) / 2;
}

static void square_track_set_state(square_track_state_t nextState)
{
    g_state = nextState;
    g_stateTimeMs = 0;
    g_cornerDebounce = 0;
    g_centerDebounce = 0;
}

static bool square_track_cooldown_finished(void)
{
    int32_t distance;

    if (!g_cornerCooldownActive) {
        return true;
    }

    distance = square_track_average_distance_from(
        g_cooldownStartLeftCount, g_cooldownStartRightCount);

    if (distance >= SQUARE_TRACK_COOLDOWN_DISTANCE_MM) {
        g_cornerCooldownActive = false;
        return true;
    }

    return false;
}

static void square_track_apply_normal_tracking(uint8_t raw)
{
    bool lineDetected;
    int32_t error = square_track_calculate_error(raw, &lineDetected);
    int32_t correction;
    int32_t leftTarget;
    int32_t rightTarget;

    if (lineDetected) {
        g_lastError = error;
        correction = (error * SQUARE_TRACK_LINE_KP) / 1000;
        correction =
            square_track_limit_i32(correction,
                                   SQUARE_TRACK_MAX_CORRECTION_MM_S);

        leftTarget = SQUARE_TRACK_BASE_SPEED_MM_S - correction;
        rightTarget = SQUARE_TRACK_BASE_SPEED_MM_S + correction;
    } else {
        correction = 0;

        if (g_lastError > 0) {
            leftTarget = -SQUARE_TRACK_LOST_SEARCH_SPEED_MM_S;
            rightTarget = SQUARE_TRACK_LOST_SEARCH_SPEED_MM_S;
        } else if (g_lastError < 0) {
            leftTarget = SQUARE_TRACK_LOST_SEARCH_SPEED_MM_S;
            rightTarget = -SQUARE_TRACK_LOST_SEARCH_SPEED_MM_S;
        } else {
            leftTarget = 0;
            rightTarget = 0;
        }
    }

    speed_pid_set_speed(leftTarget, rightTarget);

    g_squareTrackStatus.lineDetected = lineDetected;
    g_squareTrackStatus.lineError = error;
    g_squareTrackStatus.correction = correction;
    g_squareTrackStatus.leftTargetMmS = leftTarget;
    g_squareTrackStatus.rightTargetMmS = rightTarget;
}

static void square_track_update(void)
{
    uint8_t raw = gray_serial_read();

    g_stateTimeMs += SQUARE_TRACK_CONTROL_PERIOD_MS;
    g_squareTrackStatus.sensorRaw = raw;

    switch (g_state) {
    case SQUARE_TRACK_STATE_TRACKING:
        square_track_apply_normal_tracking(raw);

        if (square_track_cooldown_finished() &&
            square_track_left_three_active(raw)) {
            if (g_cornerDebounce < SQUARE_TRACK_CORNER_DEBOUNCE_COUNT) {
                g_cornerDebounce++;
            }
        } else {
            g_cornerDebounce = 0;
        }

        if (g_cornerDebounce >= SQUARE_TRACK_CORNER_DEBOUNCE_COUNT) {
            g_forwardStartLeftCount = encoder_get_left_count();
            g_forwardStartRightCount = encoder_get_right_count();
            speed_pid_set_speed(SQUARE_TRACK_BASE_SPEED_MM_S,
                                SQUARE_TRACK_BASE_SPEED_MM_S);
            square_track_set_state(SQUARE_TRACK_STATE_FORWARD_AFTER_CORNER);
        }
        break;

    case SQUARE_TRACK_STATE_FORWARD_AFTER_CORNER:
        speed_pid_set_speed(SQUARE_TRACK_BASE_SPEED_MM_S,
                            SQUARE_TRACK_BASE_SPEED_MM_S);
        g_squareTrackStatus.lineDetected = true;
        g_squareTrackStatus.lineError = 0;
        g_squareTrackStatus.correction = 0;
        g_squareTrackStatus.leftTargetMmS = SQUARE_TRACK_BASE_SPEED_MM_S;
        g_squareTrackStatus.rightTargetMmS = SQUARE_TRACK_BASE_SPEED_MM_S;

        if (square_track_average_distance_from(g_forwardStartLeftCount,
                                               g_forwardStartRightCount) >=
            SQUARE_TRACK_FORWARD_DISTANCE_MM) {
            speed_pid_set_speed(0, SQUARE_TRACK_TURN_RIGHT_WHEEL_MM_S);
            square_track_set_state(SQUARE_TRACK_STATE_TURN_LEFT);
        }
        break;

    case SQUARE_TRACK_STATE_TURN_LEFT:
        speed_pid_set_speed(0, SQUARE_TRACK_TURN_RIGHT_WHEEL_MM_S);
        g_squareTrackStatus.lineDetected = square_track_center_two_active(raw);
        g_squareTrackStatus.lineError = 0;
        g_squareTrackStatus.correction = 0;
        g_squareTrackStatus.leftTargetMmS = 0;
        g_squareTrackStatus.rightTargetMmS =
            SQUARE_TRACK_TURN_RIGHT_WHEEL_MM_S;

        if ((g_stateTimeMs >= SQUARE_TRACK_TURN_MIN_TIME_MS) &&
            square_track_center_two_active(raw)) {
            if (g_centerDebounce < SQUARE_TRACK_CENTER_DEBOUNCE_COUNT) {
                g_centerDebounce++;
            }
        } else {
            g_centerDebounce = 0;
        }

        if (g_centerDebounce >= SQUARE_TRACK_CENTER_DEBOUNCE_COUNT) {
            g_cornerCount++;
            g_cooldownStartLeftCount = encoder_get_left_count();
            g_cooldownStartRightCount = encoder_get_right_count();
            g_cornerCooldownActive = true;
            g_lastError = 0;
            square_track_set_state(SQUARE_TRACK_STATE_TRACKING);
        }
        break;

    default:
        square_track_set_state(SQUARE_TRACK_STATE_TRACKING);
        break;
    }

    g_squareTrackStatus.state = g_state;
    g_squareTrackStatus.cornerCount = g_cornerCount;
}

void square_track_run(void)
{
    encoder_init();
    gray_serial_init();
    speed_pid_init();

    g_state = SQUARE_TRACK_STATE_TRACKING;
    g_lastError = 0;
    g_stateTimeMs = 0;
    g_cornerDebounce = 0;
    g_centerDebounce = 0;
    g_cornerCooldownActive = false;
    g_cornerCount = 0;

    while (1) {
        square_track_update();
        speed_pid_control_update();
        delay_ms(SQUARE_TRACK_CONTROL_PERIOD_MS);
    }
}

void square_track_get_status(square_track_status_t *status)
{
    if (status == 0) {
        return;
    }

    *status = g_squareTrackStatus;
}
