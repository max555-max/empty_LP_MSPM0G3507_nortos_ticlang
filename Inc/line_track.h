#ifndef __LINE_TRACK_H_
#define __LINE_TRACK_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Four-channel digital line tracking.
 *
 * Sensor order from the left side of the car to the right:
 *   O1, O2, O3, O4
 *
 * Active-low fixed-ratio steering:
 *   O2 + O3 active: both wheels run at base speed.
 *   O2 only:        left wheel is the inner wheel, small-turn ratio.
 *   O3 only:        right wheel is the inner wheel, small-turn ratio.
 *   O1 active:      left wheel is the inner wheel, large-turn ratio.
 *   O4 active:      right wheel is the inner wheel, large-turn ratio.
 *
 * No line-tracking P or D term is used. The lower wheel-speed PID remains
 * responsible for following the left and right speed targets.
 */
typedef struct {
    uint8_t sensorRaw;
    bool lineDetected;
    int32_t error;           /* Discrete direction indication: left +, right -. */
    int32_t correction;      /* rightTargetMmS - leftTargetMmS. */
    int32_t leftTargetMmS;
    int32_t rightTargetMmS;
} line_track_status_t;

#define LINE_TRACK_ACTIVE_LEVEL                    (0U)
#define LINE_TRACK_BASE_SPEED_MM_S                 (500)
#define LINE_TRACK_SMALL_TURN_INNER_PERCENT        (99)
#define LINE_TRACK_LARGE_TURN_INNER_PERCENT        (45)
#define LINE_TRACK_LEFT_BASE_BIAS_MM_S             (0)
#define LINE_TRACK_RIGHT_BASE_BIAS_MM_S            (0)
#define LINE_TRACK_LOST_SEARCH_SPEED_MM_S          (180)

void line_track_init(void);

void line_track_set_base_speed(int32_t baseSpeedMmS);

/* Valid range: 0..100; largeTurnPercent must not exceed smallTurnPercent. */
bool line_track_set_turn_ratios(int32_t smallTurnPercent,
                                int32_t largeTurnPercent);

void line_track_set_left_base_bias(int32_t leftBiasMmS);
void line_track_set_right_base_bias(int32_t rightBiasMmS);
void line_track_set_base_bias(int32_t leftBiasMmS, int32_t rightBiasMmS);

int32_t line_track_get_base_speed(void);
int32_t line_track_get_small_turn_percent(void);
int32_t line_track_get_large_turn_percent(void);
int32_t line_track_get_left_base_bias(void);
int32_t line_track_get_right_base_bias(void);

void line_track_update(void);
void line_track_update_with_raw(uint8_t raw);
void line_track_update_with_raw_search_on_lost(uint8_t raw);
void line_track_update_with_raw_hold_on_lost(uint8_t raw);
void line_track_get_status(line_track_status_t *status);

#endif
