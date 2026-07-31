#ifndef __LINE_TRACK_H_
#define __LINE_TRACK_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * 八路循迹模块。
 *
 * 传感器顺序：
 *   ch1 到 ch8 从小车左侧到右侧依次排列。
 *
 * 偏差方向：
 *   黑线在左侧  -> error > 0
 *   黑线在右侧  -> error < 0
 *   黑线在中间  -> error 接近 0
 *
 * 控制方式：
 *   传感器加权平均 -> error
 *   PD 计算        -> correction，单位 mm/s
 *   左轮目标速度   = baseSpeed - correction
 *   右轮目标速度   = baseSpeed + correction
 */
typedef struct {
    uint8_t sensorRaw;       /* 原始 8bit 循迹数据。 */
    bool lineDetected;       /* 当前是否检测到黑线。 */
    int32_t error;           /* 加权平均得到的原始偏差，左正右负。 */
    int32_t correction;      /* 循迹环输出的差速修正量，单位 mm/s。 */
    int32_t leftTargetMmS;   /* 左轮目标速度，单位 mm/s。 */
    int32_t rightTargetMmS;  /* 右轮目标速度，单位 mm/s。 */
} line_track_status_t;

/* 黑线有效电平：0 表示低电平为黑线，1 表示高电平为黑线。 */
#define LINE_TRACK_ACTIVE_LEVEL           (0U)

/* 默认稳定循迹基础速度，单位 mm/s。其他任务默认使用这套稳定参数。 */
#define LINE_TRACK_BASE_SPEED_MM_S        (330)

/* 默认稳定循迹 PD 参数，按 /1000 缩放。例如 250 表示 0.250。 */
#define LINE_TRACK_TURN_KP                (50)
#define LINE_TRACK_TURN_KD                (0)

/* 中心小偏差死区，单位与 error 相同，用于减小直线摆头。 */
#define LINE_TRACK_ERROR_DEADBAND          (700)

/* 大弯判定阈值：误差达到该值后启用弯道增强。 */
#define LINE_TRACK_CURVE_ERROR_THRESHOLD   (2500)

/* 大弯额外差速量，只增加 correction 上限。 */
#define LINE_TRACK_CURVE_EXTRA_CORRECTION_MM_S  (0)

/* 大弯基础速度降低量，只降低 baseSpeed。 */
#define LINE_TRACK_CURVE_BASE_REDUCE_MM_S       (0)

/* 默认稳定循迹最大差速修正量，单位 mm/s。 */
#define LINE_TRACK_MAX_CORRECTION_MM_S    (300)

/* 左侧负重补偿：在循迹基础速度上分别给左右轮叠加偏置，单位 mm/s。 */
#define LINE_TRACK_LEFT_BASE_BIAS_MM_S    (100)
#define LINE_TRACK_RIGHT_BASE_BIAS_MM_S   (0)

/* 丢线后旋转找线速度。Task1 使用丢线停车，不使用该找线模式。 */
#define LINE_TRACK_LOST_SEARCH_SPEED_MM_S (180)

/* 初始化循迹状态、默认参数和历史误差。 */
void line_track_init(void);

/* 设置循迹基础速度，单位 mm/s。 */
void line_track_set_base_speed(int32_t baseSpeedMmS);

/* 设置循迹 PD 参数，Kp/Kd 按 /1000 缩放。 */
void line_track_set_turn_gains(int32_t kp, int32_t kd);
void line_track_set_turn_kp(int32_t kp);
void line_track_set_turn_kd(int32_t kd);

/* 设置最大差速修正量，单位 mm/s；蓝牙 LMX 命令会调用这里。 */
void line_track_set_max_correction(int32_t maxCorrectionMmS);
void line_track_set_left_base_bias(int32_t leftBiasMmS);
void line_track_set_right_base_bias(int32_t rightBiasMmS);
void line_track_set_base_bias(int32_t leftBiasMmS, int32_t rightBiasMmS);

int32_t line_track_get_base_speed(void);
int32_t line_track_get_turn_kp(void);
int32_t line_track_get_turn_kd(void);
int32_t line_track_get_max_correction(void);
int32_t line_track_get_left_base_bias(void);
int32_t line_track_get_right_base_bias(void);

/* 读取一次灰度并循迹；默认丢线后按方向找线。 */
void line_track_update(void);

/* 使用外部传入的 raw 循迹；丢线时停车。 */
void line_track_update_with_raw(uint8_t raw);

/* 使用外部传入的 raw 循迹；丢线时按最后误差方向旋转找线。 */
void line_track_update_with_raw_search_on_lost(uint8_t raw);

/* 使用外部传入的 raw 循迹；丢线时按上一帧有效误差继续行驶。 */
void line_track_update_with_raw_hold_on_lost(uint8_t raw);

/* 读取循迹状态，供 OLED/串口/蓝牙调试显示。 */
void line_track_get_status(line_track_status_t *status);

#endif