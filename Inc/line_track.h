#ifndef __LINE_TRACK_H_
#define __LINE_TRACK_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t sensorRaw;
    bool lineDetected;
    int32_t error;
    int32_t correction;
    int32_t leftTargetMmS;
    int32_t rightTargetMmS;
} line_track_status_t;

/*
 * 八路灰度循迹参数
 *
 * LINE_TRACK_ACTIVE_LEVEL:
 *   1: 传感器输出 1 表示检测到黑线/有效线
 *   0: 传感器输出 0 表示检测到黑线/有效线
 *
 * 如果实测小车反着找线，优先改这个宏。
 */
#define LINE_TRACK_ACTIVE_LEVEL          (1U)
#define LINE_TRACK_BASE_SPEED_MM_S       (300)
#define LINE_TRACK_TURN_KP               (250)
#define LINE_TRACK_TURN_KD               (80)
#define LINE_TRACK_MAX_CORRECTION_MM_S   (280)
#define LINE_TRACK_LOST_SEARCH_SPEED_MM_S (180)

void line_track_init(void);
void line_track_set_base_speed(int32_t baseSpeedMmS);
void line_track_update(void);
void line_track_get_status(line_track_status_t *status);

#endif