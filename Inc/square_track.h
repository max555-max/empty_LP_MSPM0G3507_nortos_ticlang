#ifndef __SQUARE_TRACK_H_
#define __SQUARE_TRACK_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SQUARE_TRACK_STATE_TRACKING = 0,
    SQUARE_TRACK_STATE_FORWARD_AFTER_CORNER,
    SQUARE_TRACK_STATE_TURN_LEFT,
} square_track_state_t;

typedef struct {
    square_track_state_t state;
    uint8_t sensorRaw;
    bool lineDetected;
    int32_t lineError;
    int32_t correction;
    int32_t leftTargetMmS;
    int32_t rightTargetMmS;
    uint32_t cornerCount;
} square_track_status_t;

void square_track_run(void);
void square_track_get_status(square_track_status_t *status);

#endif
