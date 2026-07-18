#include "task3_acbd.h"

#include <stdbool.h>
#include <stdint.h>

#include "angle_control.h"
#include "attitude.h"
#include "board_notify.h"
#include "delay.h"
#include "encoder.h"
#include "gray_serial.h"
#include "icm42688.h"
#include "line_track.h"
#include "pid.h"

#define TASK3_CONTROL_PERIOD_MS             (10U)

/*
 * 第三问路线：
 *   A -> C：从 A 正对 B 出发，向右偏 38 度，角度环直行 1280mm。
 *   C -> B：循迹右半圆，到 B 点丢线后退出。
 *   B -> D：车头对准 D，角度环直行 1280mm。
 *   D -> A：循迹左半圆，到 A 点丢线停车。
 *
 * 如果实测“向右旋转”对应 yaw 负方向，把下面两个角度宏整体改成负号：
 *   AC: -38
 *   BD: -142
 */
#define TASK3_AC_DISTANCE_MM                (1280)
#define TASK3_BD_DISTANCE_MM                (1280)

#define TASK3_AC_HEADING_DEG                (38.0f)
#define TASK3_BD_HEADING_DEG                (142.0f)

#define TASK3_STRAIGHT_LOW_SPEED_MM_S       (280)
#define TASK3_STRAIGHT_HIGH_SPEED_MM_S      (520)
#define TASK3_STRAIGHT_START_LOW_MM         (120)
#define TASK3_STRAIGHT_END_LOW_MM           (220)

#define TASK3_ARC_EXPECT_DISTANCE_MM        (1260)
#define TASK3_ARC_LOW_SPEED_MM_S            (260)
#define TASK3_ARC_HIGH_SPEED_MM_S           (380)
#define TASK3_ARC_START_LOW_MM              (120)
#define TASK3_ARC_END_LOW_MM                (180)
#define TASK3_ARC_SEARCH_FORWARD_MM_S       (220)

#define TASK3_ALIGN_FORWARD_SPEED_MM_S      (220)
#define TASK3_ALIGN_OK_DEG                  (3.0f)
#define TASK3_ALIGN_OK_COUNT                (8U)
#define TASK3_ALIGN_TIMEOUT_MS              (2500U)

#define TASK3_LOST_DEBOUNCE_COUNT           (8U)
#define TASK3_FINAL_LOST_DEBOUNCE_COUNT     (12U)

/*
 * 第三问专用“完全丢线”判断。
 *
 * 当前按实测第二问可停车的设置：无黑线时八路全为 1。
 * 如果串口打印发现无黑线时八路全为 0，只改这里为 0U。
 */
#define TASK3_NO_LINE_LEVEL                 (1U)

#define TASK3_GYRO_CALIB_SAMPLE_COUNT       (1000U)
#define TASK3_IMU_SETTLE_MS                 (200U)
#define TASK3_START_DELAY_MS                (1000U)
#define TASK3_STOP_SETTLE_MS                (300U)

#define TASK3_POINT_BEEP_MS                 (60U)
#define TASK3_POINT_GAP_MS                  (40U)

#define TASK3_PI_X1000000                   (3141593LL)
#define TASK3_CPR_X1000                     ((int64_t) \
    ENCODER_LINES_PER_MOTOR_REV * ENCODER_QUADRATURE_MULTIPLIER * \
    ENCODER_GEAR_RATIO_X1000)
#define TASK3_CIRCUM_MM_X1000               (((int64_t) \
    ENCODER_WHEEL_DIAMETER_MM * TASK3_PI_X1000000) / 1000LL)

static uint32_t g_task3LastAttitudeMs = 0U;
static float g_task3StartYawDeg = 0.0f;

static int32_t task3_abs_i32(int32_t value)
{
    return (value >= 0) ? value : -value;
}

static int32_t task3_counts_to_mm(int32_t counts)
{
    int64_t distance =
        (int64_t) task3_abs_i32(counts) * TASK3_CIRCUM_MM_X1000;

    distance += TASK3_CPR_X1000 / 2;
    distance /= TASK3_CPR_X1000;

    return (int32_t) distance;
}

static int32_t task3_average_distance_from(
    int32_t startLeftCount,
    int32_t startRightCount)
{
    int32_t leftDelta = encoder_get_left_count() - startLeftCount;
    int32_t rightDelta = encoder_get_right_count() - startRightCount;
    int32_t leftDistance = task3_counts_to_mm(leftDelta);
    int32_t rightDistance = task3_counts_to_mm(rightDelta);

    return (leftDistance + rightDistance) / 2;
}

static float task3_wrap_180(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }

    while (angle < -180.0f) {
        angle += 360.0f;
    }

    return angle;
}

static bool task3_all_channels_level(uint8_t raw, uint8_t targetLevel)
{
    static const uint8_t channelBitMap[8] = {
        1U, 2U, 3U, 4U, 5U, 6U, 7U, 0U
    };

    for (uint8_t i = 0U; i < 8U; i++) {
        uint8_t bit = channelBitMap[i];
        uint8_t level = (uint8_t) ((raw >> bit) & 0x01U);

        if (level != targetLevel) {
            return false;
        }
    }

    return true;
}

static bool task3_no_line_detected(uint8_t raw)
{
    return task3_all_channels_level(raw, TASK3_NO_LINE_LEVEL);
}

static bool task3_line_present_for_task(uint8_t raw)
{
    return !task3_no_line_detected(raw);
}

static int32_t task3_straight_speed_profile(
    int32_t distanceMm,
    int32_t targetDistanceMm)
{
    if (distanceMm < TASK3_STRAIGHT_START_LOW_MM) {
        return TASK3_STRAIGHT_LOW_SPEED_MM_S;
    }

    if (distanceMm > (targetDistanceMm - TASK3_STRAIGHT_END_LOW_MM)) {
        return TASK3_STRAIGHT_LOW_SPEED_MM_S;
    }

    return TASK3_STRAIGHT_HIGH_SPEED_MM_S;
}

static int32_t task3_arc_speed_profile(
    int32_t distanceMm,
    bool finalStop)
{
    if (distanceMm < TASK3_ARC_START_LOW_MM) {
        return TASK3_ARC_LOW_SPEED_MM_S;
    }

    /*
     * C->B 需要“进入和驶出低速”；D->A 按你的方案只要求进入低速、
     * 之后高速、最后丢线停车，所以 finalStop 段不主动按距离降速。
     */
    if (!finalStop &&
        (distanceMm > (TASK3_ARC_EXPECT_DISTANCE_MM - TASK3_ARC_END_LOW_MM))) {
        return TASK3_ARC_LOW_SPEED_MM_S;
    }

    return TASK3_ARC_HIGH_SPEED_MM_S;
}

static float task3_update_attitude(void)
{
    icm42688_raw_t raw;
    uint32_t nowMs;
    float dt;

    icm42688_read_raw(&raw);

    nowMs = delay_get_ms();
    dt = (float) (nowMs - g_task3LastAttitudeMs) / 1000.0f;
    g_task3LastAttitudeMs = nowMs;

    attitude_update_from_icm42688(&raw, dt);

    return dt;
}

static void task3_control_delay(void)
{
    board_notify_buzzer_off();
    delay_ms(TASK3_CONTROL_PERIOD_MS);
    board_notify_buzzer_off();
}

static void task3_stop_for_ms(uint32_t holdMs)
{
    uint32_t elapsedMs = 0U;

    angle_control_stop();
    speed_pid_stop();
    board_notify_buzzer_off();

    while (elapsedMs < holdMs) {
        board_notify_buzzer_off();
        task3_update_attitude();
        speed_pid_control_update();
        task3_control_delay();
        elapsedMs += TASK3_CONTROL_PERIOD_MS;
    }

    board_notify_buzzer_off();
}

static void task3_notify_point(bool finalPoint)
{
    board_notify_buzzer_off();
    delay_ms(TASK3_POINT_GAP_MS);

    board_notify_buzzer_on();
    board_notify_led_on();
    delay_ms(TASK3_POINT_BEEP_MS);

    board_notify_buzzer_off();

    if (!finalPoint) {
        board_notify_led_off();
    } else {
        board_notify_led_on();
    }

    board_notify_buzzer_off();
}

static void task3_drive_straight_distance(
    float targetYawDeg,
    int32_t targetDistanceMm)
{
    int32_t startLeftCount;
    int32_t startRightCount;

    task3_update_attitude();
    board_notify_buzzer_off();
    angle_control_set_target_yaw(targetYawDeg);
    angle_control_set_base_speed(TASK3_STRAIGHT_LOW_SPEED_MM_S);
    angle_control_enable(true);

    startLeftCount = encoder_get_left_count();
    startRightCount = encoder_get_right_count();

    while (1) {
        int32_t distanceMm =
            task3_average_distance_from(startLeftCount, startRightCount);
        float dt = task3_update_attitude();

        board_notify_buzzer_off();
        angle_control_set_base_speed(
            task3_straight_speed_profile(distanceMm, targetDistanceMm));
        angle_control_update(dt);
        speed_pid_control_update();

        if (distanceMm >= targetDistanceMm) {
            break;
        }

        task3_control_delay();
    }
}

static void task3_align_heading(float targetYawDeg)
{
    uint32_t elapsedMs = 0U;
    uint8_t okCount = 0U;

    angle_control_set_target_yaw(targetYawDeg);
    angle_control_set_base_speed(TASK3_ALIGN_FORWARD_SPEED_MM_S);
    angle_control_enable(true);

    while (elapsedMs < TASK3_ALIGN_TIMEOUT_MS) {
        attitude_euler_t euler;
        float errorDeg;
        float dt;

        dt = task3_update_attitude();
        attitude_get_euler(&euler);

        errorDeg = task3_wrap_180(targetYawDeg - euler.yaw);

        if ((errorDeg > -TASK3_ALIGN_OK_DEG) &&
            (errorDeg < TASK3_ALIGN_OK_DEG)) {
            if (okCount < TASK3_ALIGN_OK_COUNT) {
                okCount++;
            }
        } else {
            okCount = 0U;
        }

        angle_control_update(dt);
        speed_pid_control_update();

        if (okCount >= TASK3_ALIGN_OK_COUNT) {
            break;
        }

        task3_control_delay();
        elapsedMs += TASK3_CONTROL_PERIOD_MS;
    }
}

static void task3_follow_arc_until_lost(bool finalStop)
{
    int32_t startLeftCount;
    int32_t startRightCount;
    uint8_t lostDebounce = 0U;
    bool hasSeenLine = false;

    angle_control_stop();
    board_notify_buzzer_off();
    line_track_init();
    line_track_set_base_speed(TASK3_ARC_LOW_SPEED_MM_S);

    startLeftCount = encoder_get_left_count();
    startRightCount = encoder_get_right_count();

    while (1) {
        int32_t distanceMm =
            task3_average_distance_from(startLeftCount, startRightCount);
        uint8_t raw = gray_serial_read();
        bool lineDetected = task3_line_present_for_task(raw);

        task3_update_attitude();
        board_notify_buzzer_off();
        line_track_set_base_speed(
            task3_arc_speed_profile(distanceMm, finalStop));

        if (lineDetected) {
            hasSeenLine = true;
            lostDebounce = 0U;
            line_track_update_with_raw(raw);
        } else if (hasSeenLine) {
            uint8_t targetLostCount =
                finalStop ? TASK3_FINAL_LOST_DEBOUNCE_COUNT
                          : TASK3_LOST_DEBOUNCE_COUNT;

            if (lostDebounce < targetLostCount) {
                lostDebounce++;
            }

            if (finalStop) {
                speed_pid_stop();
            } else {
                line_track_update_with_raw_search_on_lost(raw);
            }
        } else {
            lostDebounce = 0U;
            speed_pid_set_speed(TASK3_ARC_SEARCH_FORWARD_MM_S,
                                TASK3_ARC_SEARCH_FORWARD_MM_S);
        }

        speed_pid_control_update();

        if (hasSeenLine) {
            uint8_t targetLostCount =
                finalStop ? TASK3_FINAL_LOST_DEBOUNCE_COUNT
                          : TASK3_LOST_DEBOUNCE_COUNT;

            if (lostDebounce >= targetLostCount) {
                break;
            }
        }

        task3_control_delay();
    }
}

static void task3_hold_stopped_forever(void)
{
    while (1) {
        board_notify_buzzer_off();
        speed_pid_stop();
        speed_pid_control_update();
        delay_ms(50U);
    }
}

void task3_acbd_run(void)
{
    bool imuOk;

    board_notify_init();
    board_notify_buzzer_off();
    encoder_init();
    gray_serial_init();
    speed_pid_init();
    angle_control_init();
    line_track_init();
    attitude_init();

    imuOk = icm42688_init();
    if (!imuOk) {
        task3_stop_for_ms(TASK3_STOP_SETTLE_MS);
        task3_notify_point(true);
        task3_hold_stopped_forever();
    }

    board_notify_buzzer_off();
    delay_ms(TASK3_IMU_SETTLE_MS);
    board_notify_buzzer_off();
    attitude_calibrate_gyro(TASK3_GYRO_CALIB_SAMPLE_COUNT);
    board_notify_buzzer_off();

    delay_ms(TASK3_START_DELAY_MS);
    board_notify_buzzer_off();

    g_task3LastAttitudeMs = delay_get_ms();
    encoder_reset_count();

    {
        attitude_euler_t euler;

        task3_update_attitude();
        attitude_get_euler(&euler);
        g_task3StartYawDeg = euler.yaw;
    }

    /*
     * A -> C：向右偏 38 度，直行 1280mm。
     */
    task3_drive_straight_distance(
        task3_wrap_180(g_task3StartYawDeg + TASK3_AC_HEADING_DEG),
        TASK3_AC_DISTANCE_MM);

    /*
     * C -> B：右半圆循迹，到 B 点丢线后退出。
     */
    task3_follow_arc_until_lost(false);
    task3_notify_point(false);

    /*
     * B 点后：对准 D，再进入 B -> D 直线段。
     */
    task3_align_heading(
        task3_wrap_180(g_task3StartYawDeg + TASK3_BD_HEADING_DEG));

    /*
     * B -> D：对准后高速，快到 D 低速，直行 1280mm。
     */
    task3_drive_straight_distance(
        task3_wrap_180(g_task3StartYawDeg + TASK3_BD_HEADING_DEG),
        TASK3_BD_DISTANCE_MM);

    /*
     * D -> A：左半圆循迹，到 A 点丢线停车。
     */
    task3_follow_arc_until_lost(true);

    angle_control_stop();
    speed_pid_stop();
    speed_pid_control_update();
    task3_stop_for_ms(TASK3_STOP_SETTLE_MS);
    task3_notify_point(true);

    task3_hold_stopped_forever();
}
