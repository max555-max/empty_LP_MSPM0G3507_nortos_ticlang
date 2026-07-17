#ifndef __ANGLE_CONTROL_H_
#define __ANGLE_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Yaw angle outer-loop for straight-line driving.
 *
 * Control structure:
 *   yaw target/current -> angle PD -> wheel speed correction
 *   wheel targets      -> existing left/right speed PID -> PWM
 *
 * Unit:
 *   yaw angle      : degree
 *   wheel speed    : mm/s
 *   correction     : mm/s
 */

/*
 * ANGLE_CONTROL_DIRECTION:
 *   1.0f  : default correction direction.
 *  -1.0f  : flip if the car corrects farther away from the target heading.
 */
#define ANGLE_CONTROL_DIRECTION              (1.0f)

/*
 * Start with P only, then add D if the car oscillates.
 *
 * Kp unit: mm/s per degree.
 * Kd unit: mm/s per (degree/s).
 */
#define ANGLE_CONTROL_KP_MM_S_PER_DEG        (10.0f)
#define ANGLE_CONTROL_KD_MM_S_PER_DPS        (0.0f)

#define ANGLE_CONTROL_MAX_CORRECTION_MM_S    (220)
#define ANGLE_CONTROL_MAX_TARGET_MM_S        (900)

typedef struct {
    bool enabled;
    float targetYawDeg;
    float currentYawDeg;
    float errorDeg;
    float errorRateDps;
    int32_t baseSpeedMmS;
    int32_t correctionMmS;
    int32_t leftTargetMmS;
    int32_t rightTargetMmS;
} angle_control_status_t;

void angle_control_init(void);
void angle_control_enable(bool enable);
void angle_control_stop(void);

/*
 * Set straight-line base speed. Positive values move forward, negative values
 * move backward. The angle loop only adds differential correction.
 */
void angle_control_set_base_speed(int32_t baseSpeedMmS);

/*
 * Use the current yaw as heading target. Call this when entering a straight
 * segment.
 */
void angle_control_lock_current_yaw(void);

/*
 * Manually set heading target if a higher-level planner already has one.
 */
void angle_control_set_target_yaw(float yawDeg);

/*
 * Run one angle-loop update.
 * dt unit: second. Call after attitude_update_from_icm42688().
 */
void angle_control_update(float dt);

void angle_control_get_status(angle_control_status_t *status);

#endif
