#ifndef __SQUARE_TRACK_H_
#define __SQUARE_TRACK_H_

#include <stdint.h>

typedef enum {
    /* 当前边正常循迹。 */
    SQUARE_TRACK_STATE_TRACK = 0,
    /* 所有灰度通道都没检测到黑线：继续向前走一小段距离。 */
    SQUARE_TRACK_STATE_ADVANCE_AFTER_LOST = 1,
    /* 前进距离到达后：原地右转，直到中间两个通道重新检测到黑线。 */
    SQUARE_TRACK_STATE_TURN_RIGHT = 2
} square_track_state_t;

typedef struct {
    square_track_state_t state;
    /* 当前边编号：0=A->B，1=B->C，2=C->D，3=D->A。 */
    uint8_t segmentIndex;
    /* 灰度传感器原始 8 位数据，尚未按实际从左到右通道重新映射。 */
    uint8_t sensorRaw;
    /* 当前检测到黑线的物理通道数量。 */
    uint8_t activeCount;
    /* 中间两个物理通道都检测到黑线时为 1。 */
    uint8_t centerDetected;
    /* 丢线后继续前进阶段的编码器估算距离，单位 mm。 */
    int32_t advanceDistanceMm;
    /* 最近一次写给速度环的左右轮目标速度，单位 mm/s。 */
    int32_t leftTargetMmS;
    int32_t rightTargetMmS;
} square_track_status_t;

void square_track_init(void);
void square_track_update(void);
void square_track_get_status(square_track_status_t *status);
char square_track_get_segment_start_label(void);
char square_track_get_segment_end_label(void);

#endif
