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

/*
 * task1_ab.c
 *
 * 第一问：A -> B 直行停车。
 *
 * 控制方式：
 *   1. 上电后 IMU 静止校准；
 *   2. 锁定当前 yaw 作为直行方向；
 *   3. 角度环保持直线；
 *   4. 编码器平均距离达到目标后停车并声光提示。
 */

/* 控制周期，和速度 PID/编码器测速周期保持一致。 */
#define TASK1_CONTROL_PERIOD_MS        (10U)

/* A->B 目标距离，单位 mm。 */
#define TASK1_TARGET_DISTANCE_MM       (1000)

/* 第一问直行基础速度，单位 mm/s。 */
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

/* int32 绝对值。 */
static int32_t task1_abs_i32(int32_t value)
{
    return (value >= 0) ? value : -value;
}

/* 编码器计数换算为距离 mm。 */
static int32_t task1_counts_to_mm(int32_t counts)
{
    int64_t distance =
        (int64_t) task1_abs_i32(counts) * TASK1_CIRCUM_MM_X1000;

    distance += TASK1_CPR_X1000 / 2;
    distance /= TASK1_CPR_X1000;

    return (int32_t) distance;
}

/* 计算从起点开始左右轮平均前进距离。 */
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

/* 停车并保持一段时间，确保 PWM 真正清零。 */
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

    /* 初始化第一问需要用到的模块。 */
    board_notify_init();
    encoder_init();
    speed_pid_init();
    angle_control_init();
    attitude_init();

    /* IMU 初始化失败时直接停车提示。 */
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
     * IMU 校准阶段小车必须保持静止。
     */
    delay_ms(TASK1_IMU_SETTLE_MS);
    attitude_calibrate_gyro(TASK1_GYRO_CALIB_SAMPLE_COUNT);

    /*
     * 校准完成后等待 1s，再开始运动。
     */
    delay_ms(TASK1_START_DELAY_MS);

    /* 清零编码器，并记录直线段起点。 */
    encoder_reset_count();
    startLeftCount = encoder_get_left_count();
    startRightCount = encoder_get_right_count();

    lastUpdateMs = delay_get_ms();
    /* 锁定当前 yaw，开始角度环直行。 */
    angle_control_set_base_speed(TASK1_BASE_SPEED_MM_S);
    angle_control_lock_current_yaw();
    angle_control_enable(true);

    /* 距离未达到目标前，周期运行姿态更新、角度环和速度环。 */
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

    /* 到达 B 点后停车并提示。 */
    task1_stop_and_hold();
    board_notify_arrived();

    while (1) {
        speed_pid_stop();
        speed_pid_control_update();
        delay_ms(50U);
    }
}
