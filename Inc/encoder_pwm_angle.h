#ifndef ENCODER_PWM_ANGLE_H
#define ENCODER_PWM_ANGLE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * PA8 / TIMA0_CCP0 captures the absolute-angle PWM from the stepper encoder.
 *
 * Current integration:
 * - read PWM duty and convert it to raw 0.1 degree units;
 * - convert raw absolute angle to beam-relative angle using the hand-set zero;
 * - expose both raw and relative values to VOFA/debug code;
 * - provide a conservative software safety-limit flag for the ball PID task.
 */
#define ENCODER_PWM_ANGLE_VALID_TIMEOUT_MS     (100U)
#define ENCODER_PWM_ANGLE_FULL_SCALE_X10       (3600U)

/*
 * Beam absolute-angle calibration from the user's real mechanism test.
 *
 * Units:
 * - *_DEG_X10 is 0.1 degree.
 * - RAW values are the direct PWM absolute angle reported by this module.
 * - REL values are beam angles relative to the hand-set mechanical zero.
 *
 * Measured samples:
 * - mechanical zero: raw 237.6 deg;
 * - user-confirmed negative limit: raw 217.8 deg;
 * - user-confirmed positive limit: raw 272.6 deg.
 *
 * The encoder raw angle increases toward the user's positive beam direction,
 * so relative beam angle should be calculated as:
 *   relativeDegX10 = rawDegX10 - BEAM_ENCODER_ZERO_RAW_DEG_X10
 */
#define BEAM_ENCODER_ZERO_RAW_DEG_X10          (2376)
#define BEAM_ENCODER_NEG_LIMIT_RAW_DEG_X10     (2178)
#define BEAM_ENCODER_POS_LIMIT_RAW_DEG_X10     (2726)

#define BEAM_ENCODER_NEG_LIMIT_REL_DEG_X10     (-198)
#define BEAM_ENCODER_POS_LIMIT_REL_DEG_X10     (350)

/*
 * Conservative software safety limit for the next integration step.
 * This is intentionally inside the measured mechanical limits.
 */
#define BEAM_ENCODER_MAX_SAFE_REL_DEG_X10       (150)

typedef struct {
    int32_t angleDegX10;
    int32_t relativeAngleDegX10;
    uint32_t periodTicks;
    uint32_t highTicks;
    uint32_t captureCount;
    uint32_t timeoutCount;
    uint32_t timestampMs;
    bool valid;
    bool fresh;
    bool safeLimitActive;
} encoder_pwm_angle_sample_t;

void encoder_pwm_angle_init(void);
bool encoder_pwm_angle_get_sample(encoder_pwm_angle_sample_t *sample);
int32_t encoder_pwm_angle_raw_to_relative_deg_x10(int32_t rawDegX10);
bool encoder_pwm_angle_relative_is_safe(int32_t relativeDegX10);
void encoder_pwm_angle_irq_handler(void);

#endif /* ENCODER_PWM_ANGLE_H */
