#include "task2_abcd.h"

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

#define TASK2_CONTROL_PERIOD_MS             (10U)

/*
 * 第二问路线：
 *   A -> B：角度环直行，编码器约束 1000mm，检测到 B 点黑线后进入循迹。
 *   B -> C：八路灰度循迹右半圆，连续丢线后认为离开半圆。
 *   C -> D：先用陀螺仪把方向拉到与出发方向平行但反向，再直行到 D。
 *   D -> A：八路灰度循迹左半圆，连续丢线后停车。
 */
#define TASK2_STRAIGHT_DISTANCE_MM          (1000)
#define TASK2_STRAIGHT_LINE_ENABLE_MM       \
    ((TASK2_STRAIGHT_DISTANCE_MM * 65) / 100)
#define TASK2_STRAIGHT_MAX_DISTANCE_MM      \
    (TASK2_STRAIGHT_DISTANCE_MM + 150)

#define TASK2_STRAIGHT_LOW_SPEED_MM_S       (350)
#define TASK2_STRAIGHT_HIGH_SPEED_MM_S      (700)
#define TASK2_STRAIGHT_START_LOW_MM         (50)
#define TASK2_STRAIGHT_END_LOW_MM           (120)

#define TASK2_ARC_EXPECT_DISTANCE_MM        (1260)
#define TASK2_ARC_LOW_SPEED_MM_S            (300)
#define TASK2_ARC_HIGH_SPEED_MM_S           (420)
#define TASK2_ARC_START_LOW_MM              (50)
#define TASK2_ARC_END_LOW_MM                (50)
#define TASK2_ARC_SEARCH_FORWARD_MM_S       (220)
#define TASK2_ALIGN_FORWARD_SPEED_MM_S      (260)

/*
 * 角度补偿：
 *   起点方向记为 0 度。
 *   A -> B 理论目标为 startYaw + 0 度。
 *   C -> D 理论目标为 startYaw + 180 度。
 *
 * 如果实际车往外侧偏，就把对应补偿改成相反符号。
 */
#define TASK2_AB_HEADING_COMP_DEG           (0.0f)
#define TASK2_CD_HEADING_COMP_DEG           (0.0f)
#define TASK2_CD_BASE_HEADING_DEG           (180.0f)

#define TASK2_LINE_DEBOUNCE_COUNT           (3U)
#define TASK2_LOST_DEBOUNCE_COUNT           (8U)
#define TASK2_FINAL_LOST_DEBOUNCE_COUNT     (12U)

/*
 * 第二问专用“完全丢线”判断。
 *
 * 注意：这个宏不等同于 LINE_TRACK_ACTIVE_LEVEL。
 * LINE_TRACK_ACTIVE_LEVEL 用于循迹误差计算；
 * TASK2_NO_LINE_LEVEL 用于判断八路传感器是否已经完全离开黑线。
 *
 * 如果串口打印显示“无黑线时八路全为 1”，保持 1U；
 * 如果“无黑线时八路全为 0”，改成 0U。
 */
#define TASK2_NO_LINE_LEVEL                 (1U)

#define TASK2_ALIGN_OK_DEG                  (3.0f)
#define TASK2_ALIGN_OK_COUNT                (8U)
#define TASK2_ALIGN_TIMEOUT_MS              (2500U)

#define TASK2_GYRO_CALIB_SAMPLE_COUNT       (1000U)
#define TASK2_IMU_SETTLE_MS                 (200U)
#define TASK2_START_DELAY_MS                (1000U)
#define TASK2_STOP_SETTLE_MS                (300U)

#define TASK2_POINT_BEEP_MS                 (60U)
#define TASK2_POINT_GAP_MS                  (40U)

#define TASK2_PI_X1000000                   (3141593LL)
#define TASK2_CPR_X1000                     ((int64_t) \
    ENCODER_LINES_PER_MOTOR_REV * ENCODER_QUADRATURE_MULTIPLIER * \
    ENCODER_GEAR_RATIO_X1000)
#define TASK2_CIRCUM_MM_X1000               (((int64_t) \
    ENCODER_WHEEL_DIAMETER_MM * TASK2_PI_X1000000) / 1000LL)

static uint32_t g_task2LastAttitudeMs = 0U;
static float g_task2StartYawDeg = 0.0f;

static int32_t task2_abs_i32(int32_t value)
{
    return (value >= 0) ? value : -value;
}

static int32_t task2_counts_to_mm(int32_t counts)
{
    int64_t distance =
        (int64_t) task2_abs_i32(counts) * TASK2_CIRCUM_MM_X1000;

    distance += TASK2_CPR_X1000 / 2;
    distance /= TASK2_CPR_X1000;

    return (int32_t) distance;
}

static int32_t task2_average_distance_from(
    int32_t startLeftCount,
    int32_t startRightCount)
{
    int32_t leftDelta = encoder_get_left_count() - startLeftCount;
    int32_t rightDelta = encoder_get_right_count() - startRightCount;
    int32_t leftDistance = task2_counts_to_mm(leftDelta);
    int32_t rightDistance = task2_counts_to_mm(rightDelta);

    return (leftDistance + rightDistance) / 2;
}

static float task2_wrap_180(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }

    while (angle < -180.0f) {
        angle += 360.0f;
    }

    return angle;
}

static bool task2_all_channels_level(uint8_t raw, uint8_t targetLevel)
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

static bool task2_no_line_detected(uint8_t raw)
{
    return task2_all_channels_level(raw, TASK2_NO_LINE_LEVEL);
}

static bool task2_line_present_for_task(uint8_t raw)
{
    return !task2_no_line_detected(raw);
}

static int32_t task2_straight_speed_profile(int32_t distanceMm)
{
    if (distanceMm < TASK2_STRAIGHT_START_LOW_MM) {
        return TASK2_STRAIGHT_LOW_SPEED_MM_S;
    }

    if (distanceMm >
        (TASK2_STRAIGHT_DISTANCE_MM - TASK2_STRAIGHT_END_LOW_MM)) {
        return TASK2_STRAIGHT_LOW_SPEED_MM_S;
    }

    return TASK2_STRAIGHT_HIGH_SPEED_MM_S;
}

static int32_t task2_arc_speed_profile(int32_t distanceMm)
{
    if (distanceMm < TASK2_ARC_START_LOW_MM) {
        return TASK2_ARC_LOW_SPEED_MM_S;
    }

    if (distanceMm >
        (TASK2_ARC_EXPECT_DISTANCE_MM - TASK2_ARC_END_LOW_MM)) {
        return TASK2_ARC_LOW_SPEED_MM_S;
    }

    return TASK2_ARC_HIGH_SPEED_MM_S;
}

static float task2_update_attitude(void)
{
    icm42688_raw_t raw;
    uint32_t nowMs;
    float dt;

    icm42688_read_raw(&raw);

    nowMs = delay_get_ms();
    dt = (float) (nowMs - g_task2LastAttitudeMs) / 1000.0f;
    g_task2LastAttitudeMs = nowMs;

    attitude_update_from_icm42688(&raw, dt);

    return dt;
}

static void task2_control_delay(void)
{
    board_notify_buzzer_off();
    delay_ms(TASK2_CONTROL_PERIOD_MS);
    board_notify_buzzer_off();
}

static void task2_stop_for_ms(uint32_t holdMs)
{
    uint32_t elapsedMs = 0U;

    angle_control_stop();
    speed_pid_stop();
    board_notify_buzzer_off();

    while (elapsedMs < holdMs) {
        board_notify_buzzer_off();
        task2_update_attitude();
        speed_pid_control_update();
        task2_control_delay();
        elapsedMs += TASK2_CONTROL_PERIOD_MS;
    }

    board_notify_buzzer_off();
}

static void task2_notify_point(bool finalPoint)
{
    board_notify_buzzer_off();
    delay_ms(TASK2_POINT_GAP_MS);

    board_notify_buzzer_on();
    board_notify_led_on();
    delay_ms(TASK2_POINT_BEEP_MS);

    board_notify_buzzer_off();

    if (!finalPoint) {
        board_notify_led_off();
    } else {
        board_notify_led_on();
    }

    board_notify_buzzer_off();
}

static void task2_drive_straight_until_line(float targetYawDeg)
{
    int32_t startLeftCount;
    int32_t startRightCount;
    uint8_t lineDebounce = 0U;

    task2_update_attitude();
    board_notify_buzzer_off();
    angle_control_set_target_yaw(targetYawDeg);
    angle_control_set_base_speed(TASK2_STRAIGHT_LOW_SPEED_MM_S);
    angle_control_enable(true);

    startLeftCount = encoder_get_left_count();
    startRightCount = encoder_get_right_count();

    while (1) {
        int32_t distanceMm =
            task2_average_distance_from(startLeftCount, startRightCount);
        uint8_t raw = gray_serial_read();
        bool lineDetected = task2_line_present_for_task(raw);
        float dt = task2_update_attitude();

        board_notify_buzzer_off();
        angle_control_set_base_speed(task2_straight_speed_profile(distanceMm));
        angle_control_update(dt);
        speed_pid_control_update();

        /*
         * A 点和 C 点附近可能也有黑线，所以先用编码器走过一段距离，
         * 再允许黑线检测触发切段。
         */
        if ((distanceMm >= TASK2_STRAIGHT_LINE_ENABLE_MM) && lineDetected) {
            if (lineDebounce < TASK2_LINE_DEBOUNCE_COUNT) {
                lineDebounce++;
            }
        } else {
            lineDebounce = 0U;
        }

        if (lineDebounce >= TASK2_LINE_DEBOUNCE_COUNT) {
            break;
        }

        /*
         * 如果黑线没有被可靠识别，用最大距离兜底，避免一直直行。
         * 正常情况下应由黑线触发退出。
         */
        if (distanceMm >= TASK2_STRAIGHT_MAX_DISTANCE_MM) {
            break;
        }

        task2_control_delay();
    }
}

static void task2_align_heading(float targetYawDeg)
{
    uint32_t elapsedMs = 0U;
    uint8_t okCount = 0U;

    angle_control_set_target_yaw(targetYawDeg);
    angle_control_set_base_speed(TASK2_ALIGN_FORWARD_SPEED_MM_S);
    angle_control_enable(true);

    while (elapsedMs < TASK2_ALIGN_TIMEOUT_MS) {
        attitude_euler_t euler;
        float errorDeg;
        float dt;

        dt = task2_update_attitude();
        attitude_get_euler(&euler);

        errorDeg = task2_wrap_180(targetYawDeg - euler.yaw);

        if ((errorDeg > -TASK2_ALIGN_OK_DEG) &&
            (errorDeg < TASK2_ALIGN_OK_DEG)) {
            if (okCount < TASK2_ALIGN_OK_COUNT) {
                okCount++;
            }
        } else {
            okCount = 0U;
        }

        angle_control_update(dt);
        speed_pid_control_update();

        if (okCount >= TASK2_ALIGN_OK_COUNT) {
            break;
        }

        task2_control_delay();
        elapsedMs += TASK2_CONTROL_PERIOD_MS;
    }
}

static void task2_follow_arc_until_lost(bool finalStop)
{
    int32_t startLeftCount;
    int32_t startRightCount;
    uint8_t lostDebounce = 0U;
    bool hasSeenLine = false;

    angle_control_stop();
    board_notify_buzzer_off();
    line_track_init();
    line_track_set_base_speed(TASK2_ARC_LOW_SPEED_MM_S);

    startLeftCount = encoder_get_left_count();
    startRightCount = encoder_get_right_count();

    while (1) {
        int32_t distanceMm =
            task2_average_distance_from(startLeftCount, startRightCount);
        uint8_t raw = gray_serial_read();
        bool lineDetected = task2_line_present_for_task(raw);

        task2_update_attitude();
        board_notify_buzzer_off();
        line_track_set_base_speed(task2_arc_speed_profile(distanceMm));

        if (lineDetected) {
            hasSeenLine = true;
            lostDebounce = 0U;
            line_track_update_with_raw(raw);
        } else if (hasSeenLine) {
            if (lostDebounce < TASK2_FINAL_LOST_DEBOUNCE_COUNT) {
                lostDebounce++;
            }

            if (finalStop) {
                speed_pid_stop();
            } else {
                line_track_update_with_raw_search_on_lost(raw);
            }
        } else {
            /*
             * 刚切入半圆时，如果第一帧还没压上线，不允许直接停车。
             * 先低速向前，让灰度模块重新压到黑线。
             */
            lostDebounce = 0U;
            speed_pid_set_speed(TASK2_ARC_SEARCH_FORWARD_MM_S,
                                TASK2_ARC_SEARCH_FORWARD_MM_S);
        }

        speed_pid_control_update();

        if (hasSeenLine) {
            uint8_t targetLostCount =
                finalStop ? TASK2_FINAL_LOST_DEBOUNCE_COUNT
                          : TASK2_LOST_DEBOUNCE_COUNT;

            if (lostDebounce >= targetLostCount) {
                break;
            }
        }

        task2_control_delay();
    }
}

static void task2_hold_stopped_forever(void)
{
    while (1) {
        board_notify_buzzer_off();
        speed_pid_stop();
        speed_pid_control_update();
        delay_ms(50U);
    }
}

void task2_abcd_run(void)
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
        task2_stop_for_ms(TASK2_STOP_SETTLE_MS);
        task2_notify_point(true);
        task2_hold_stopped_forever();
    }

    /*
     * 陀螺仪校准期间小车必须保持静止。
     * 这里不蜂鸣，避免上电时误认为已经过点。
     */
    board_notify_buzzer_off();
    delay_ms(TASK2_IMU_SETTLE_MS);
    board_notify_buzzer_off();
    attitude_calibrate_gyro(TASK2_GYRO_CALIB_SAMPLE_COUNT);
    board_notify_buzzer_off();

    delay_ms(TASK2_START_DELAY_MS);
    board_notify_buzzer_off();

    g_task2LastAttitudeMs = delay_get_ms();
    encoder_reset_count();

    {
        attitude_euler_t euler;

        task2_update_attitude();
        attitude_get_euler(&euler);
        g_task2StartYawDeg = euler.yaw;
    }

    /*
     * A -> B：
     * 角度环按起点方向直行，叠加 A->B 补偿角。
     * 走过一定距离后检测到黑线即进入 B->C 循迹。
     */
    task2_drive_straight_until_line(
        task2_wrap_180(g_task2StartYawDeg + TASK2_AB_HEADING_COMP_DEG));
    task2_notify_point(false);

    /*
     * B -> C：
     * 八路灰度循迹半圆。连续检测不到黑线后，认为半圆结束。
     */
    task2_follow_arc_until_lost(false);
    task2_notify_point(false);

    /*
     * C 点后：
     * 用陀螺仪把方向拉到与出发方向平行但反向，即 startYaw + 180 度。
     */
    task2_align_heading(
        task2_wrap_180(g_task2StartYawDeg +
                       TASK2_CD_BASE_HEADING_DEG +
                       TASK2_CD_HEADING_COMP_DEG));

    /*
     * C -> D：
     * 继续角度环直行 1000mm 附近，检测到 D 点黑线后进入 D->A 循迹。
     */
    task2_drive_straight_until_line(
        task2_wrap_180(g_task2StartYawDeg +
                       TASK2_CD_BASE_HEADING_DEG +
                       TASK2_CD_HEADING_COMP_DEG));
    task2_notify_point(false);

    /*
     * D -> A：
     * 八路灰度循迹半圆。连续检测不到黑线后最终停车。
     */
    task2_follow_arc_until_lost(true);

    angle_control_stop();
    speed_pid_stop();
    speed_pid_control_update();
    task2_stop_for_ms(TASK2_STOP_SETTLE_MS);
    task2_notify_point(true);

    task2_hold_stopped_forever();
}
