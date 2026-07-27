#include "square_track.h"

#include <stdbool.h>
#include <stdint.h>

#include "encoder.h"
#include "gray_serial.h"
#include "line_track.h"
#include "pid.h"

/*
 * 最常调的参数：
 * - BASE_SPEED：循迹和压到横线时的匀速前进速度；
 * - TURN_SPEED：原地右转时左右轮的目标速度；
 * - FORWARD_AFTER_LOST：完全丢线后继续向前走的距离。
 *
 * 这里的 140 mm 由编码器参数换算得到，实际距离还会受轮径、
 * 编码器减速比、地面摩擦和打滑影响，需要实车微调。
 */
#define SQUARE_TRACK_BASE_SPEED_MM_S       (300)
#define SQUARE_TRACK_TURN_SPEED_MM_S       (180)
#define SQUARE_TRACK_FORWARD_AFTER_LOST_MM (140)
/* 3+ black channels: treat as a wide line/cross line and go straight. */
#define SQUARE_TRACK_STRAIGHT_ACTIVE_MIN   (3U)

/* 物理通道下标：按小车从左到右排列，经过 g_squareChannelBitMap 映射后使用。 */
#define SQUARE_TRACK_CENTER_LEFT_INDEX     (3U)
#define SQUARE_TRACK_CENTER_RIGHT_INDEX    (4U)

#define SQUARE_TRACK_PI_X1000000           (3141593LL)

static square_track_status_t g_squareStatus;

static const uint8_t g_squareChannelBitMap[8] = {
    1U, 2U, 3U, 4U, 5U, 6U, 7U, 0U
};

/*
 * 把“从左到右的物理通道下标 0..7”转换成灰度模块返回值里的原始 bit。
 * 除非确认灰度传感器 bit 顺序变了，否则这里要和 gray/line_track 保持一致。
 */
static bool square_track_channel_active(uint8_t raw, uint8_t channelIndex)//通道映射
{
    uint8_t bit = g_squareChannelBitMap[channelIndex];
    uint8_t level = (uint8_t)((raw >> bit) & 0x01U);

    return level == LINE_TRACK_ACTIVE_LEVEL;
}

static uint8_t square_track_count_active(uint8_t raw)//计数
{
    uint8_t count = 0U;

    for (uint8_t i = 0U; i < 8U; i++) {
        if (square_track_channel_active(raw, i)) {
            count++;
        }
    }

    return count;
}

static bool square_track_center_detected(uint8_t raw)
{
    return
        square_track_channel_active(raw, SQUARE_TRACK_CENTER_LEFT_INDEX) &&
        square_track_channel_active(raw, SQUARE_TRACK_CENTER_RIGHT_INDEX);
}

static int32_t square_track_abs_i32(int32_t value)
{
    if (value < 0) {
        return -value;
    }

    return value;
}

static int32_t square_track_counts_to_mm(int32_t count)
{
    /*
     * 距离 = 编码器计数 / 每圈计数 * 轮子周长。
     * 这里用 1000/1000000 的整数放大倍数计算，避免在状态机里使用浮点。
     */
    int64_t cprX1000 =
        (int64_t)ENCODER_LINES_PER_MOTOR_REV *
        ENCODER_QUADRATURE_MULTIPLIER *
        ENCODER_GEAR_RATIO_X1000;
    int64_t circumferenceMmX1000 =
        ((int64_t)ENCODER_WHEEL_DIAMETER_MM *
         SQUARE_TRACK_PI_X1000000) / 1000LL;
    int64_t numerator =
        (int64_t)square_track_abs_i32(count) *
        circumferenceMmX1000;

    return (int32_t)((numerator + (cprX1000 / 2LL)) / cprX1000);
}

static int32_t square_track_forward_distance_mm(void)
{
    int32_t leftMm = square_track_counts_to_mm(encoder_get_left_count());
    int32_t rightMm = square_track_counts_to_mm(encoder_get_right_count());

    return (leftMm + rightMm) / 2;
}

static void square_track_set_targets(int32_t leftMmS, int32_t rightMmS)
{
    g_squareStatus.leftTargetMmS = leftMmS;
    g_squareStatus.rightTargetMmS = rightMmS;
    speed_pid_set_speed(leftMmS, rightMmS);
}

static void square_track_start_advance(void)
{
    /* 从“完全丢线”的这一刻开始清零编码器，测量后续前进距离。 */
    encoder_reset_count();
    g_squareStatus.advanceDistanceMm = 0;
    g_squareStatus.state = SQUARE_TRACK_STATE_ADVANCE_AFTER_LOST;
    square_track_set_targets(
        SQUARE_TRACK_BASE_SPEED_MM_S,
        SQUARE_TRACK_BASE_SPEED_MM_S);
}

static void square_track_start_turn_right(void)
{
    /* 原地右转：左轮后退，右轮前进；如果实车方向反了，优先检查电机方向约定。 */
    g_squareStatus.state = SQUARE_TRACK_STATE_TURN_RIGHT;
    square_track_set_targets(
        -SQUARE_TRACK_TURN_SPEED_MM_S,
        SQUARE_TRACK_TURN_SPEED_MM_S);
}

static void square_track_start_next_segment(uint8_t raw)
{
    line_track_status_t lineStatus;

    /* 一次右转完成后，边编号按 A->B->C->D->A 循环，并恢复循迹状态。 */
    g_squareStatus.segmentIndex =
        (uint8_t)((g_squareStatus.segmentIndex + 1U) & 0x03U);
    g_squareStatus.state = SQUARE_TRACK_STATE_TRACK;
    line_track_update_with_raw(raw);

    line_track_get_status(&lineStatus);
    g_squareStatus.leftTargetMmS = lineStatus.leftTargetMmS;
    g_squareStatus.rightTargetMmS = lineStatus.rightTargetMmS;
}

void square_track_init(void)
{
    g_squareStatus.state = SQUARE_TRACK_STATE_TRACK;
    g_squareStatus.segmentIndex = 0U;
    g_squareStatus.sensorRaw = 0U;
    g_squareStatus.activeCount = 0U;
    g_squareStatus.centerDetected = 0U;
    g_squareStatus.advanceDistanceMm = 0;
    g_squareStatus.leftTargetMmS = 0;
    g_squareStatus.rightTargetMmS = 0;

    line_track_init();
    line_track_set_base_speed(SQUARE_TRACK_BASE_SPEED_MM_S);
    speed_pid_stop();
}

void square_track_update(void)
{
    uint8_t raw = gray_serial_read();       //只能得到传感器数据，读取01数据，用于判断有几个传感器压线
    uint8_t activeCount = square_track_count_active(raw);       //计数
    bool centerDetected = square_track_center_detected(raw);    //
    line_track_status_t lineStatus;

    g_squareStatus.sensorRaw = raw;
    g_squareStatus.activeCount = activeCount;
    g_squareStatus.centerDetected = centerDetected ? 1U : 0U;

    switch (g_squareStatus.state) {
    case SQUARE_TRACK_STATE_TRACK:
        g_squareStatus.advanceDistanceMm = 0;

        if (activeCount == 0U) {
            /* 当前边走到末端：没有任何通道看到黑线，进入丢线后前进阶段。 */
            square_track_start_advance();
        } else if (activeCount >= SQUARE_TRACK_STRAIGHT_ACTIVE_MIN) {
            /* 压到较宽黑线/横线：不做循迹修正，左右轮保持同速直行。 */
            square_track_set_targets(
                SQUARE_TRACK_BASE_SPEED_MM_S,
                SQUARE_TRACK_BASE_SPEED_MM_S);
        } else {
            /* 普通循迹：复用已有 line_track 模块计算左右轮差速。 */
            line_track_update_with_raw(raw);
            line_track_get_status(&lineStatus);
            g_squareStatus.leftTargetMmS = lineStatus.leftTargetMmS;
            g_squareStatus.rightTargetMmS = lineStatus.rightTargetMmS;
        }
        break;

    case SQUARE_TRACK_STATE_ADVANCE_AFTER_LOST:
        g_squareStatus.advanceDistanceMm =
            square_track_forward_distance_mm();

        if (g_squareStatus.advanceDistanceMm >=
            SQUARE_TRACK_FORWARD_AFTER_LOST_MM) {
            square_track_start_turn_right();
        } else {
            /* 还没走完约 14 cm：继续保持左右轮同速向前。 */
            square_track_set_targets(
                SQUARE_TRACK_BASE_SPEED_MM_S,
                SQUARE_TRACK_BASE_SPEED_MM_S);
        }
        break;

    case SQUARE_TRACK_STATE_TURN_RIGHT:
        if (centerDetected) {
            /* 找到下一条边：中间两个通道压到黑线，退出右转并恢复循迹。 */
            square_track_start_next_segment(raw);
        } else {
            /* 还没找到下一条边：继续原地右转搜索黑线。 */
            square_track_set_targets(
                -SQUARE_TRACK_TURN_SPEED_MM_S,
                SQUARE_TRACK_TURN_SPEED_MM_S);
        }
        break;

    default:
        square_track_init();
        break;
    }
}

void square_track_get_status(square_track_status_t *status)
{
    if (status == 0) {
        return;
    }

    *status = g_squareStatus;
}

char square_track_get_segment_start_label(void)
{
    static const char labels[4] = {'A', 'B', 'C', 'D'};

    return labels[g_squareStatus.segmentIndex & 0x03U];
}

char square_track_get_segment_end_label(void)
{
    static const char labels[4] = {'B', 'C', 'D', 'A'};

    return labels[g_squareStatus.segmentIndex & 0x03U];
}
