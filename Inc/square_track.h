#ifndef __SQUARE_TRACK_H_
#define __SQUARE_TRACK_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    /* 正常 PD 循迹状态。 */
    SQUARE_TRACK_STATE_TRACKING = 0,

    /* 检测到直角弯后，继续直行一小段距离。 */
    SQUARE_TRACK_STATE_FORWARD_AFTER_CORNER,

    /* 左转状态，直到中间传感器重新压线。 */
    SQUARE_TRACK_STATE_TURN_LEFT,
} square_track_state_t;

typedef struct {
    /* 当前循迹状态机状态。 */
    square_track_state_t state;

    /* 灰度原始值。 */
    uint8_t sensorRaw;

    /* 是否检测到线。 */
    bool lineDetected;

    /* 当前循迹偏差。 */
    int32_t lineError;

    /* 差速修正量。 */
    int32_t correction;

    /* 左右轮目标速度。 */
    int32_t leftTargetMmS;
    int32_t rightTargetMmS;

    /* 已识别的直角弯数量。 */
    uint32_t cornerCount;
} square_track_status_t;

/* 运行正方形循迹任务，主函数只需调用该函数。 */
void square_track_run(void);

/* 获取正方形循迹状态，方便串口调试。 */
void square_track_get_status(square_track_status_t *status);

#endif
