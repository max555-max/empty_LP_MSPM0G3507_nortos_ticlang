#include "task1_ab.h"

#include <stdbool.h>
#include <stdint.h>

#include "angle_control.h"
#include "attitude.h"
#include "board_notify.h"
#include "delay.h"
#include "encoder.h"
#include "icm42688.h"
#include "pid.h"

#define TASK1_CONTROL_PERIOD_MS        (10U)
#define TASK1_TARGET_DISTANCE_MM       (1000)
#define TASK1_BASE_SPEED_MM_S          (300)
#define TASK1_GYRO_CALIB_SAMPLE_COUNT  (1000U)
#define TASK1_IMU_SETTLE_MS            (200U)
#define TASK1_START_DELAY_MS           (1000U)
#define TASK1_STOP_SETTLE_MS           (500U)

#define TASK1_PI_X1000000              (3141593LL)
#define TASK1_CPR_X1000                ((int64_t) \
    ENCODER_LINES_PER_MOTOR_REV * ENCODER_QUADRATURE_MULTIPLIER * \
    ENCODER_GEAR_RATIO_X1000)
#define TASK1_CIRCUM_MM_X1000          (((int64_t) \
    ENCODER_WHEEL_DIAMETER_MM * TASK1_PI_X1000000) / 1000LL)

static int32_t task1_abs_i32(int32_t value)
{
    return (value >= 0) ? value : -value;
}

static int32_t task1_counts_to_mm(int32_t counts)
{
    int64_t distance =
        (int64_t) task1_abs_i32(counts) * TASK1_CIRCUM_MM_X1000;

    distance += TASK1_CPR_X1000 / 2;
    distance /= TASK1_CPR_X1000;

    return (int32_t) distance;
}

static int32_t task1_get_average_distance_mm(
    int32_t startLeftCount,
    int32_t startRightCount)
{
    int32_t leftDelta = encoder_get_left_count() - startLeftCount;
    int32_t rightDelta = encoder_get_right_count() - startRightCount;
    int32_t leftDistance = task1_counts_to_mm(leftDelta);
    int32_t rightDistance = task1_counts_to_mm(rightDelta);

    return (leftDistance + rightDistance) / 2;
}

static void task1_stop_and_hold(void)
{
    uint32_t elapsedMs = 0U;

    angle_control_stop();
    speed_pid_stop();

    while (elapsedMs < TASK1_STOP_SETTLE_MS) {
        speed_pid_control_update();
        delay_ms(TASK1_CONTROL_PERIOD_MS);
        elapsedMs += TASK1_CONTROL_PERIOD_MS;
    }
}

void task1_ab_run(void)
{
    icm42688_raw_t raw;
    bool imuOk;
    uint32_t lastUpdateMs;
    uint32_t nowMs;
    float dt;
    int32_t startLeftCount;
    int32_t startRightCount;

    board_notify_init();
    encoder_init();
    speed_pid_init();
    angle_control_init();
    attitude_init();

    imuOk = icm42688_init();
    if (!imuOk) {
        task1_stop_and_hold();
        board_notify_arrived();
        while (1) {
            speed_pid_stop();
            speed_pid_control_update();
            delay_ms(50U);
        }
    }

    /*
     * Keep the car still during this stage.
     */
    delay_ms(TASK1_IMU_SETTLE_MS);
    attitude_calibrate_gyro(TASK1_GYRO_CALIB_SAMPLE_COUNT);

    /*
     * Calibration is done. Wait 1s, then start moving.
     */
    delay_ms(TASK1_START_DELAY_MS);

    encoder_reset_count();
    startLeftCount = encoder_get_left_count();
    startRightCount = encoder_get_right_count();

    lastUpdateMs = delay_get_ms();
    angle_control_set_base_speed(TASK1_BASE_SPEED_MM_S);
    angle_control_lock_current_yaw();
    angle_control_enable(true);

    while (task1_get_average_distance_mm(startLeftCount, startRightCount) <
           TASK1_TARGET_DISTANCE_MM) {
        icm42688_read_raw(&raw);

        nowMs = delay_get_ms();
        dt = (float) (nowMs - lastUpdateMs) / 1000.0f;
        lastUpdateMs = nowMs;

        attitude_update_from_icm42688(&raw, dt);
        angle_control_update(dt);
        speed_pid_control_update();

        delay_ms(TASK1_CONTROL_PERIOD_MS);
    }

    task1_stop_and_hold();
    board_notify_arrived();

    while (1) {
        speed_pid_stop();
        speed_pid_control_update();
        delay_ms(50U);
    }
}
