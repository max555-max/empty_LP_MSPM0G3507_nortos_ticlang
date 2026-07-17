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

#define TASK2_CONTROL_PERIOD_MS        (10U)

/*
 * Geometry from the problem statement.
 * Straight sections A-B and C-D are symmetric, both about 100 cm.
 * Semicircle radius is 40 cm, so arc length is pi * R ~= 1256 mm.
 */
#define TASK2_STRAIGHT_DISTANCE_MM     (1000)
#define TASK2_ARC_DISTANCE_MM          (1260)

#define TASK2_STRAIGHT_SPEED_MM_S      (300)
#define TASK2_ARC_SPEED_MM_S           (280)

#define TASK2_GYRO_CALIB_SAMPLE_COUNT  (1000U)
#define TASK2_IMU_SETTLE_MS            (200U)
#define TASK2_START_DELAY_MS           (1000U)
#define TASK2_STOP_SETTLE_MS           (300U)

#define TASK2_POINT_BEEP_MS            (120U)
#define TASK2_POINT_GAP_MS             (80U)

#define TASK2_PI_X1000000              (3141593LL)
#define TASK2_CPR_X1000                ((int64_t) \
    ENCODER_LINES_PER_MOTOR_REV * ENCODER_QUADRATURE_MULTIPLIER * \
    ENCODER_GEAR_RATIO_X1000)
#define TASK2_CIRCUM_MM_X1000          (((int64_t) \
    ENCODER_WHEEL_DIAMETER_MM * TASK2_PI_X1000000) / 1000LL)

static uint32_t g_task2LastAttitudeMs = 0U;

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
        delay_ms(TASK2_POINT_GAP_MS);
    } else {
        board_notify_led_on();
    }

    board_notify_buzzer_off();
}

static void task2_drive_straight_mm(int32_t distanceMm)
{
    int32_t startLeftCount;
    int32_t startRightCount;

    task2_update_attitude();
    board_notify_buzzer_off();
    angle_control_set_base_speed(TASK2_STRAIGHT_SPEED_MM_S);
    angle_control_lock_current_yaw();
    angle_control_enable(true);

    startLeftCount = encoder_get_left_count();
    startRightCount = encoder_get_right_count();

    while (task2_average_distance_from(startLeftCount, startRightCount) <
           distanceMm) {
        float dt = task2_update_attitude();

        board_notify_buzzer_off();
        angle_control_update(dt);
        speed_pid_control_update();
        task2_control_delay();
    }

    task2_stop_for_ms(TASK2_STOP_SETTLE_MS);
}

static void task2_follow_arc_mm(int32_t distanceMm)
{
    int32_t startLeftCount;
    int32_t startRightCount;

    angle_control_stop();
    board_notify_buzzer_off();
    line_track_init();
    line_track_set_base_speed(TASK2_ARC_SPEED_MM_S);

    startLeftCount = encoder_get_left_count();
    startRightCount = encoder_get_right_count();

    while (task2_average_distance_from(startLeftCount, startRightCount) <
           distanceMm) {
        task2_update_attitude();
        board_notify_buzzer_off();
        line_track_update();
        speed_pid_control_update();
        task2_control_delay();
    }

    task2_stop_for_ms(TASK2_STOP_SETTLE_MS);
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
     * Keep the car still during calibration.
     * No buzzer is used here, so the start point will not beep.
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

    task2_drive_straight_mm(TASK2_STRAIGHT_DISTANCE_MM);
    task2_notify_point(false);  /* B */

    task2_follow_arc_mm(TASK2_ARC_DISTANCE_MM);
    task2_notify_point(false);  /* C */

    task2_drive_straight_mm(TASK2_STRAIGHT_DISTANCE_MM);
    task2_notify_point(false);  /* D */

    task2_follow_arc_mm(TASK2_ARC_DISTANCE_MM);
    task2_notify_point(true);   /* A */

    task2_hold_stopped_forever();
}
