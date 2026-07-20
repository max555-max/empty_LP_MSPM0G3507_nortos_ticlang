#ifndef __LINE_TRACK_H_
#define __LINE_TRACK_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * 八路灰度循迹模块
 *
 * 传感器方向约定：
 *   从小车左侧到右侧依次为通道 1~8。
 *
 * 偏差方向约定：
 *   黑线偏左时 error 为正；
 *   黑线偏右时 error 为负；
 *   黑线在中间时 error 接近 0。
 *
 * 控制方式：
 *   灰度加权平均得到 error；
 *   使用 PD 计算差速 correction；
 *   左轮目标速度 = baseSpeed + correction；
 *   右轮目标速度 = baseSpeed - correction。
 */
typedef struct {
    uint8_t sensorRaw;       /* 灰度传感器原始 8bit 数据。 */
    bool lineDetected;       /* 当前是否检测到有效黑线。 */
    int32_t error;           /* 加权平均得到的循迹偏差，左正右负。 */
    int32_t correction;      /* PD 算出的左右轮差速修正量。 */
    int32_t leftTargetMmS;   /* 下发给速度环的左轮目标速度。 */
    int32_t rightTargetMmS;  /* 下发给速度环的右轮目标速度。 */
} line_track_status_t;

/*
 * 八路灰度循迹参数
 *
 * LINE_TRACK_ACTIVE_LEVEL：
 *   1：传感器输出 1 表示检测到黑线；
 *   0：传感器输出 0 表示检测到黑线。
 *
 * 如果小车明显把白底当线，优先检查/修改这个宏。
 */
#define LINE_TRACK_ACTIVE_LEVEL           (1U)

/* 默认循迹基础速度，单位 mm/s。上层任务可动态覆盖。 */
#define LINE_TRACK_BASE_SPEED_MM_S        (300)

/* 循迹 PD 参数，内部按 /1000 使用，例如 250 表示 0.250。 */
#define LINE_TRACK_TURN_KP                (250)
#define LINE_TRACK_TURN_KD                (120)

/* 循迹最大差速修正量，防止偏差大时输出过猛。 */
#define LINE_TRACK_MAX_CORRECTION_MM_S    (280)

/* 丢线后旋转找线速度。部分任务会选择不用旋转找线。 */
#define LINE_TRACK_LOST_SEARCH_SPEED_MM_S (180)

/* 初始化循迹状态和历史误差。 */
void line_track_init(void);

/* 设置循迹基础速度，单位 mm/s。 */
void line_track_set_base_speed(int32_t baseSpeedMmS);

/* 读取一次灰度并执行循迹控制。 */
void line_track_update(void);

/* 使用外部传入的 raw 执行循迹；丢线时停车。 */
void line_track_update_with_raw(uint8_t raw);

/* 使用外部传入的 raw 执行循迹；丢线时按最后误差方向旋转找线。 */
void line_track_update_with_raw_search_on_lost(uint8_t raw);

/* 读取循迹状态，方便串口/VOFA 调试。 */
void line_track_get_status(line_track_status_t *status);

#endif
