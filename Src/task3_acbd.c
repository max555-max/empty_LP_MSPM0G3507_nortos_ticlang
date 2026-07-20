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

#if defined(__GNUC__)
#define TASK3_NOINLINE __attribute__((noinline))
#else
#define TASK3_NOINLINE
#endif

#define TASK3_CONTROL_PERIOD_MS             (10U)

/*
 * 第三问路线：
 *   A -> C：手动让车头从 A 对准 C，上电后锁定当前 yaw，角度环直行 1280mm。
 *   C -> B：循迹右半圆，到 B 点丢线后退出。
 *   B -> D：车头对准 D，角度环直行 1280mm。
 *   D -> A：循迹左半圆，到 A 点丢线停车。
 *
 * 这里的角度都以“上电时手动对准 A->C 的方向”为 0 度。
 * B 点后不再使用固定绝对角度，而是读取当前 yaw 后相对旋转 38 度对准 D。
 */
#define TASK3_AC_DISTANCE_MM                (1500)
#define TASK3_BD_DISTANCE_MM                (1150)
#define TASK3_AC_LINE_ENABLE_MM             (700)
#define TASK3_AC_LINE_DEBOUNCE_COUNT        (3U)
#define TASK3_AC_SEARCH_SPEED_MM_S          (220)

#define TASK3_AC_HEADING_DEG                (0.0f)
#define TASK3_BD_TURN_FROM_B_DEG            (39.0f)

#define TASK3_STRAIGHT_LOW_SPEED_MM_S       (280)
#define TASK3_STRAIGHT_HIGH_SPEED_MM_S      (520)
#define TASK3_STRAIGHT_START_LOW_MM         (120)
#define TASK3_STRAIGHT_END_LOW_MM           (100)

#define TASK3_BD_LOW_SPEED_MM_S             (220)
#define TASK3_BD_HIGH_SPEED_MM_S            (380)
#define TASK3_BD_START_LOW_MM               (160)
#define TASK3_BD_END_LOW_MM                 (300)
#define TASK3_BD_LINE_ENABLE_MM             (650)
#define TASK3_BD_LINE_DEBOUNCE_COUNT        (2U)
#define TASK3_BD_SEARCH_SPEED_MM_S          (200)

#define TASK3_ARC_EXPECT_DISTANCE_MM        (1260)
#define TASK3_ARC_LOW_SPEED_MM_S            (260)
#define TASK3_ARC_HIGH_SPEED_MM_S           (350)
#define TASK3_ARC_START_LOW_MM              (120)
#define TASK3_ARC_END_LOW_MM                (100)
#define TASK3_ARC_EXIT_MIN_DISTANCE_MM      (700)
#define TASK3_ARC_SEARCH_FORWARD_MM_S       (220)

#define TASK3_ALIGN_OK_DEG                  (3.0f)
#define TASK3_ALIGN_OK_COUNT                (8U)
#define TASK3_ALIGN_TIMEOUT_MS              (1200U)
#define TASK3_ALIGN_BASE_SPEED_MM_S         (220)
#define TASK3_ALIGN_RAMP_MS                 (300U)
#define TASK3_TURN_KP_MM_S_PER_DEG          (4.0f)
#define TASK3_TURN_MIN_SPEED_MM_S           (70)
#define TASK3_TURN_MAX_SPEED_MM_S           (140)

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

/* 上一次姿态更新时间，用于计算本次姿态更新 dt。 */
static uint32_t g_task3LastAttitudeMs = 0U;

/* 上电并完成校准后记录的初始 yaw。第三问所有角度都相对它计算。 */
static float g_task3StartYawDeg = 0.0f;

/* int32 绝对值，编码器距离只关心走过的长度，不关心正反方向。 */
static int32_t task3_abs_i32(int32_t value)
{
    return (value >= 0) ? value : -value;
}

/*
 * 编码器计数换算为轮子走过的距离，单位 mm。
 *
 * 计算思路：
 *   count / CPR = 轮子转过的圈数；
 *   圈数 * 轮子周长 = 距离。
 */
static int32_t task3_counts_to_mm(int32_t counts)
{
    int64_t distance =
        (int64_t) task3_abs_i32(counts) * TASK3_CIRCUM_MM_X1000;

    distance += TASK3_CPR_X1000 / 2;
    distance /= TASK3_CPR_X1000;

    return (int32_t) distance;
}

/*
 * 从某个起点计数开始，计算左右轮平均行驶距离。
 * 平均值更适合作为“小车整体前进距离”。
 */
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

/* 将角度归一化到 -180~180，避免跨 ±180° 时误差突变。 */
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

/* float 绝对值。 */
static float task3_abs_f32(float value)
{
    return (value >= 0.0f) ? value : -value;
}

/*
 * 根据角度误差计算“单轮前进转向”的速度。
 * 这里不输出负速度。
 */
static int32_t task3_turn_speed_from_error(float errorDeg)
{
    float absErrorDeg = task3_abs_f32(errorDeg);
    int32_t speedMmS;

    speedMmS = (int32_t)
        (absErrorDeg * TASK3_TURN_KP_MM_S_PER_DEG + 0.5f);

    if (speedMmS < TASK3_TURN_MIN_SPEED_MM_S) {
        speedMmS = TASK3_TURN_MIN_SPEED_MM_S;
    }

    if (speedMmS > TASK3_TURN_MAX_SPEED_MM_S) {
        speedMmS = TASK3_TURN_MAX_SPEED_MM_S;
    }

    return speedMmS;
}

/*
 * 第三问专用循迹更新：不允许轮速目标为负。
 * 避免普通循迹在误差很大时让小车原地旋转。
 */
static void task3_line_track_update_no_reverse(uint8_t raw)
{
    line_track_status_t status;
    int32_t leftTargetMmS;
    int32_t rightTargetMmS;

    line_track_update_with_raw(raw);
    line_track_get_status(&status);

    leftTargetMmS = status.leftTargetMmS;
    rightTargetMmS = status.rightTargetMmS;

    if (leftTargetMmS < 0) {
        leftTargetMmS = 0;
    }

    if (rightTargetMmS < 0) {
        rightTargetMmS = 0;
    }

    speed_pid_set_speed(leftTargetMmS, rightTargetMmS);
}

/*
 * 判断 8 路灰度是否全部等于 targetLevel。
 * channelBitMap 与 gray_serial_print()/line_track.c 保持一致。
 */
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

/* 判断当前是否完全丢线。 */
static bool task3_no_line_detected(uint8_t raw)
{
    return task3_all_channels_level(raw, TASK3_NO_LINE_LEVEL);
}

/* 第三问使用的“是否还有线”判断。 */
static bool task3_line_present_for_task(uint8_t raw)
{
    return !task3_no_line_detected(raw);
}

/*
 * 直线段速度规划。
 * 进入直线时低速，中间高速，快到目标距离时低速。
 */
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

/*
 * B-D 专用直线速度规划。
 *
 * B-D 段目前实测“距离偏长、速度偏快”，所以单独使用更低速度，
 * 并增加末端低速距离，方便进入 D-A 循迹。
 */
static int32_t task3_bd_speed_profile(
    int32_t distanceMm,
    int32_t targetDistanceMm)
{
    if (distanceMm < TASK3_BD_START_LOW_MM) {
        return TASK3_BD_LOW_SPEED_MM_S;
    }

    if (distanceMm > (targetDistanceMm - TASK3_BD_END_LOW_MM)) {
        return TASK3_BD_LOW_SPEED_MM_S;
    }

    return TASK3_BD_HIGH_SPEED_MM_S;
}

/*
 * 半圆弧循迹速度规划。
 * finalStop=false：C->B，中途高速，接近末端低速驶出；
 * finalStop=true ：D->A，进入低速后高速，最后只靠丢线停车。
 */
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

/*
 * 读取 IMU 并更新姿态。
 * 返回本次姿态更新使用的 dt，单位秒。
 */
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

/*
 * 第三问控制周期延时。
 * 反复关闭蜂鸣器，是为了防止低电平触发蜂鸣器误响。
 */
static void task3_control_delay(void)
{
    board_notify_buzzer_off();
    delay_ms(TASK3_CONTROL_PERIOD_MS);
    board_notify_buzzer_off();
}

/* 停车并保持 holdMs 毫秒，同时继续刷新速度环确保 PWM 为 0。 */
static void task3_stop_for_ms(uint32_t holdMs)
{
    uint32_t elapsedMs = 0U;

    /* 进入循迹段前关闭角度环，只让循迹模块控制左右轮。 */
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

/*
 * 到点提示。
 * finalPoint=false：中途点，提示后灯关闭；
 * finalPoint=true ：终点，提示后灯保持亮。
 */
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

/*
 * 角度环定距直行。
 * targetYawDeg：该直线段要保持的航向角；
 * targetDistanceMm：该段目标距离，单位 mm。
 */
static TASK3_NOINLINE void task3_drive_straight_distance(
    float targetYawDeg,
    int32_t targetDistanceMm,
    bool useBdProfile)
{
    int32_t startLeftCount;
    int32_t startRightCount;

    task3_update_attitude();
    board_notify_buzzer_off();

    /* 开启角度环，初始使用低速进入。 */
    angle_control_set_target_yaw(targetYawDeg);
    angle_control_set_base_speed(
        useBdProfile ? TASK3_BD_LOW_SPEED_MM_S
                     : TASK3_STRAIGHT_LOW_SPEED_MM_S);
    angle_control_enable(true);

    /* 记录该直线段起点编码器计数。 */
    startLeftCount = encoder_get_left_count();
    startRightCount = encoder_get_right_count();

    while (1) {
        int32_t distanceMm =
            task3_average_distance_from(startLeftCount, startRightCount);
        float dt = task3_update_attitude();

        board_notify_buzzer_off();
        /* 按当前位置选择低速/高速/末端低速。 */
        angle_control_set_base_speed(
            useBdProfile
                ? task3_bd_speed_profile(distanceMm, targetDistanceMm)
                : task3_straight_speed_profile(distanceMm, targetDistanceMm));
        /* 角度环输出左右轮速度目标，速度环输出 PWM。 */
        angle_control_update(dt);
        speed_pid_control_update();

        /* 到达目标距离后退出直线段。 */
        if (distanceMm >= targetDistanceMm) {
            break;
        }

        task3_control_delay();
    }
}

/*
 * A -> C 专用直行函数：不死跑固定距离。
 *
 * 逻辑：
 *   1. 先按角度环保持方向直行；
 *   2. 距离超过 lineEnableDistanceMm 后，开始看灰度是否检测到黑线；
 *   3. 连续检测到黑线 TASK3_AC_LINE_DEBOUNCE_COUNT 次，就提前退出；
 *   4. 如果一直没有检测到黑线，最多跑到 maxDistanceMm 后也退出，
 *      防止小车一直向前冲出赛道。
 */
static TASK3_NOINLINE void task3_drive_straight_until_line(
    float targetYawDeg,
    int32_t lineEnableDistanceMm,
    int32_t maxDistanceMm)
{
    int32_t startLeftCount;
    int32_t startRightCount;
    uint8_t lineDebounce = 0U;

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

        if (distanceMm >= lineEnableDistanceMm) {
            /*
             * 进入 C 点附近后主动降速找线，避免高速冲过 C-B 弧线入口。
             */
            angle_control_set_base_speed(TASK3_AC_SEARCH_SPEED_MM_S);
        } else {
            angle_control_set_base_speed(
                task3_straight_speed_profile(distanceMm, maxDistanceMm));
        }
        angle_control_update(dt);
        speed_pid_control_update();

        /*
         * 前半段不看灰度，避免刚从 A 点出发时把 A 点附近黑线误认为 C-B。
         */
        if (distanceMm >= lineEnableDistanceMm) {
            uint8_t raw = gray_serial_read();

            if (task3_line_present_for_task(raw)) {
                if (lineDebounce < TASK3_AC_LINE_DEBOUNCE_COUNT) {
                    lineDebounce++;
                }
            } else {
                lineDebounce = 0U;
            }

            if (lineDebounce >= TASK3_AC_LINE_DEBOUNCE_COUNT) {
                break;
            }
        }

        /*
         * 最大距离保护：如果没有检测到线，也不要无限直行。
         * 退出后会进入 C-B 循迹函数，由它低速直行接线。
         */
        if (distanceMm >= maxDistanceMm) {
            break;
        }

        task3_control_delay();
    }
}

/*
 * B -> D 专用直行函数：检测到 D-A 黑线后提前切入循迹。
 *
 * 和 A->C 类似，但使用 B-D 专用速度曲线：
 *   1. 前半段按 B-D 低/高速规划行驶；
 *   2. 超过 TASK3_BD_LINE_ENABLE_MM 后降速并开始检测黑线；
 *   3. 连续检测到黑线 TASK3_BD_LINE_DEBOUNCE_COUNT 次后退出；
 *   4. 如果没检测到线，最多走到 TASK3_BD_DISTANCE_MM 后退出。
 */
static TASK3_NOINLINE void task3_drive_bd_until_line(float targetYawDeg)
{
    int32_t startLeftCount;
    int32_t startRightCount;
    uint8_t lineDebounce = 0U;

    task3_update_attitude();
    board_notify_buzzer_off();

    angle_control_set_target_yaw(targetYawDeg);
    angle_control_set_base_speed(TASK3_BD_LOW_SPEED_MM_S);
    angle_control_enable(true);

    startLeftCount = encoder_get_left_count();
    startRightCount = encoder_get_right_count();

    while (1) {
        int32_t distanceMm =
            task3_average_distance_from(startLeftCount, startRightCount);
        float dt = task3_update_attitude();

        board_notify_buzzer_off();

        if (distanceMm >= TASK3_BD_LINE_ENABLE_MM) {
            /*
             * 接近 D-A 黑线入口后降速找线，避免冲过 D 点。
             */
            angle_control_set_base_speed(TASK3_BD_SEARCH_SPEED_MM_S);
        } else {
            angle_control_set_base_speed(
                task3_bd_speed_profile(distanceMm, TASK3_BD_DISTANCE_MM));
        }

        angle_control_update(dt);
        speed_pid_control_update();

        if (distanceMm >= TASK3_BD_LINE_ENABLE_MM) {
            uint8_t raw = gray_serial_read();

            if (task3_line_present_for_task(raw)) {
                if (lineDebounce < TASK3_BD_LINE_DEBOUNCE_COUNT) {
                    lineDebounce++;
                }
            } else {
                lineDebounce = 0U;
            }

            if (lineDebounce >= TASK3_BD_LINE_DEBOUNCE_COUNT) {
                break;
            }
        }

        if (distanceMm >= TASK3_BD_DISTANCE_MM) {
            break;
        }

        task3_control_delay();
    }
}

/*
 * 使用角度环前进对准目标角度。
 *
 * 不再手写“左快右慢/右快左慢”的分支，避免 yaw 符号和车体方向
 * 判断不一致时越转越偏。
 *
 * 这里让小车保持前进，由 angle_control.c 根据 yaw 误差自动产生差速。
 */
static TASK3_NOINLINE void task3_align_heading(float targetYawDeg)
{
    uint32_t elapsedMs = 0U;
    uint8_t okCount = 0U;

    angle_control_set_target_yaw(targetYawDeg);
    angle_control_set_base_speed(TASK3_ALIGN_BASE_SPEED_MM_S);
    angle_control_enable(true);

    while (elapsedMs < TASK3_ALIGN_TIMEOUT_MS) {
        attitude_euler_t euler;
        float errorDeg;
        float dt;

        dt = task3_update_attitude();
        attitude_get_euler(&euler);

        /* 计算目标 yaw 与当前 yaw 的最短角度误差。 */
        errorDeg = task3_wrap_180(targetYawDeg - euler.yaw);

        if ((errorDeg > -TASK3_ALIGN_OK_DEG) &&
            (errorDeg < TASK3_ALIGN_OK_DEG)) {
            if (okCount < TASK3_ALIGN_OK_COUNT) {
                okCount++;
            }

            /*
             * 误差进入允许范围后不停车，而是两轮同速前进。
             * 连续稳定若干次后认为对准完成，随后直接进入 B-D 直行。
             */
            angle_control_set_base_speed(TASK3_ALIGN_BASE_SPEED_MM_S);
        } else {
            /* 误差超出范围，重新开始稳定计数。 */
            okCount = 0U;
        }

        /*
         * 软启动：
         *   对准刚开始时先低速前进，再逐步恢复到对准基础速度，
         *   减少从循迹切入角度环时的速度突变。
         */
        if (elapsedMs < TASK3_ALIGN_RAMP_MS) {
            int32_t rampSpeed =
                (int32_t) (((int64_t) TASK3_ALIGN_BASE_SPEED_MM_S *
                            elapsedMs) / TASK3_ALIGN_RAMP_MS);

            if (rampSpeed < TASK3_AC_SEARCH_SPEED_MM_S) {
                rampSpeed = TASK3_AC_SEARCH_SPEED_MM_S;
            }

            angle_control_set_base_speed(rampSpeed);
        } else {
            angle_control_set_base_speed(TASK3_ALIGN_BASE_SPEED_MM_S);
        }

        angle_control_update(dt);
        speed_pid_control_update();

        /* 连续多次处于允许误差内，认为对准成功。 */
        if (okCount >= TASK3_ALIGN_OK_COUNT) {
            break;
        }

        task3_control_delay();
        elapsedMs += TASK3_CONTROL_PERIOD_MS;
    }

    /*
     * 对准完成或超时后不强制停车。
     * 后续 B-D 直行函数会重新设置角度环目标并立即接管。
     */
    speed_pid_control_update();
}

/*
 * 半圆弧循迹，直到“已经见过线后又连续丢线”。
 *
 * finalStop=false：
 *   C->B 段，丢线后退出，进入 B->D 转向；
 *   为防止刚接入弧线就误判退出，要求距离超过 TASK3_ARC_EXIT_MIN_DISTANCE_MM。
 *
 * finalStop=true：
 *   D->A 段，丢线后认为到 A，退出后最终停车。
 */
static TASK3_NOINLINE void task3_follow_arc_until_lost(bool finalStop)
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
        /* 读取灰度并判断是否还有线。 */
        uint8_t raw = gray_serial_read();
        bool lineDetected = task3_line_present_for_task(raw);

        task3_update_attitude();
        board_notify_buzzer_off();
        /* 根据弧线段距离和是否终点段设置当前基础速度。 */
        line_track_set_base_speed(
            task3_arc_speed_profile(distanceMm, finalStop));

        if (lineDetected) {
            /* 检测到线：正常循迹，并清除丢线防抖计数。 */
            hasSeenLine = true;
            lostDebounce = 0U;
            task3_line_track_update_no_reverse(raw);
        } else if (hasSeenLine) {
            /* 已经见过线后又丢线：开始防抖计数。 */
            uint8_t targetLostCount =
                finalStop ? TASK3_FINAL_LOST_DEBOUNCE_COUNT
                          : TASK3_LOST_DEBOUNCE_COUNT;

            if (lostDebounce < targetLostCount) {
                lostDebounce++;
            }

            if (finalStop) {
                /* 最终段 D->A：丢线期间先停车，等待防抖确认。 */
                speed_pid_stop();
            } else {
                /* C->B：短暂丢线时不旋转找线，只低速直行等待重新压线。 */
                speed_pid_set_speed(TASK3_ARC_SEARCH_FORWARD_MM_S,
                                    TASK3_ARC_SEARCH_FORWARD_MM_S);
            }
        } else {
            /* 刚进入弧线段还没见到线：低速直行去接线。 */
            lostDebounce = 0U;
            speed_pid_set_speed(TASK3_ARC_SEARCH_FORWARD_MM_S,
                                TASK3_ARC_SEARCH_FORWARD_MM_S);
        }

        speed_pid_control_update();

        if (hasSeenLine) {
            uint8_t targetLostCount =
                finalStop ? TASK3_FINAL_LOST_DEBOUNCE_COUNT
                          : TASK3_LOST_DEBOUNCE_COUNT;

            /*
             * 退出条件：
             *   1. 已经连续丢线达到防抖次数；
             *   2. 若是 C->B，还必须超过最小退出距离；
             *   3. 若是 D->A，丢线防抖满足即可停车。
             */
            if ((lostDebounce >= targetLostCount) &&
                (finalStop ||
                 (distanceMm >= TASK3_ARC_EXIT_MIN_DISTANCE_MM))) {
                break;
            }
        }

        task3_control_delay();
    }
}

/* 终点后永久停车，防止任务函数返回后主循环继续执行其他逻辑。 */
static TASK3_NOINLINE void task3_hold_stopped_forever(void)
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
    float bdTargetYawDeg;

    /* 初始化第三问会用到的全部模块。 */
    board_notify_init();
    board_notify_buzzer_off();
    encoder_init();
    gray_serial_init();
    speed_pid_init();
    angle_control_init();
    line_track_init();
    attitude_init();

    /* IMU 初始化失败时直接声光提示并永久停车。 */
    imuOk = icm42688_init();
    if (!imuOk) {
        task3_stop_for_ms(TASK3_STOP_SETTLE_MS);
        task3_notify_point(true);
        task3_hold_stopped_forever();
    }

    /* 等 IMU 稳定后进行静止陀螺仪校准。校准期间小车必须保持不动。 */
    board_notify_buzzer_off();
    delay_ms(TASK3_IMU_SETTLE_MS);
    board_notify_buzzer_off();
    attitude_calibrate_gyro(TASK3_GYRO_CALIB_SAMPLE_COUNT);
    board_notify_buzzer_off();

    /* 给操作者 1 秒准备时间。 */
    delay_ms(TASK3_START_DELAY_MS);
    board_notify_buzzer_off();

    /* 校准后重置姿态时间戳和编码器，作为任务起点。 */
    g_task3LastAttitudeMs = delay_get_ms();
    encoder_reset_count();

    {
        attitude_euler_t euler;

        /* 记录上电初始 yaw，第三问全部目标角度都相对它计算。 */
        task3_update_attitude();
        attitude_get_euler(&euler);
        g_task3StartYawDeg = euler.yaw;
    }

    /*
     * A -> C：
     *   上电前已手动对准 C；
     *   保持初始方向直行；
     *   超过 TASK3_AC_LINE_ENABLE_MM 后检测到黑线就提前进入 C-B 循迹；
     *   如果没检测到线，最多走到 TASK3_AC_DISTANCE_MM 后进入 C-B 接线。
     */
    task3_drive_straight_until_line(
        task3_wrap_180(g_task3StartYawDeg + TASK3_AC_HEADING_DEG),
        TASK3_AC_LINE_ENABLE_MM,
        TASK3_AC_DISTANCE_MM);

    /*
     * C -> B：
     *   右半圆循迹；
     *   跑过最小退出距离后，连续丢线认为到 B。
     */
    task3_follow_arc_until_lost(false);
    /*
     * C-B 到 B 后不再清零速度。
     * 让 B-D 转向函数直接接管速度目标，避免 0 速度插入造成抽搐。
     */
    /*
     * B 点中途提示先取消。
     * 原因：
     *   蜂鸣提示内部有 delay_ms()，会打断从 C-B 循迹到 B-D 转向的衔接，
     *   容易造成停车/启动瞬间抽搐。
     */
    /* task3_notify_point(false); */

    {
        attitude_euler_t euler;

        /*
         * 到 B 点后读取当前车头方向，再相对旋转 38 度对准 D。
         * 如果实测转向方向相反，把 TASK3_BD_TURN_FROM_B_DEG 改成 -38.0f。
         */
        task3_update_attitude();
        attitude_get_euler(&euler);
        bdTargetYawDeg =
            task3_wrap_180(euler.yaw + TASK3_BD_TURN_FROM_B_DEG);
    }

    /*
     * B 点后：
     *   先按相对角度对准 D；
     *   对准后重新读取当前 yaw，锁定实际车头方向进入 B -> D 直线段。
     *
     * 这样做的原因：
     *   转向阶段结束时，实际 yaw 不一定和 bdTargetYawDeg 完全一致；
     *   如果继续用旧目标角直行，角度环一进入 B-D 就可能大幅修正，
     *   表现为左右轮乱晃。
    */
    task3_align_heading(bdTargetYawDeg);

    {
        attitude_euler_t euler;

        task3_update_attitude();
        attitude_get_euler(&euler);
        bdTargetYawDeg = euler.yaw;
    }

    /*
     * B -> D：
     *   保持转向结束后的实际车头方向直行；
     *   后半段检测到 D-A 黑线后提前进入循迹。
     */
    task3_drive_bd_until_line(bdTargetYawDeg);

    /*
     * D -> A：
     *   左半圆循迹；
     *   到 A 后连续丢线，最终停车。
     */
    task3_follow_arc_until_lost(true);

    angle_control_stop();
    speed_pid_stop();
    speed_pid_control_update();
    task3_stop_for_ms(TASK3_STOP_SETTLE_MS);
    task3_notify_point(true);

    task3_hold_stopped_forever();
}
