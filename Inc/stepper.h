#ifndef STEPPER_H
#define STEPPER_H

#include <stdbool.h>
#include <stdint.h>

#define STEPPER_MIN_FREQ_HZ                 (120U)
#define STEPPER_MAX_FREQ_HZ                 (3000U)
#define STEPPER_ACCELERATION_HZ_PER_S       (20000.0f)
#define STEPPER_ENABLE_ACTIVE_LEVEL         (1U)
/* Set to 1 when logical positive beam angle needs the opposite DIR level. */
#define STEPPER_DIRECTION_INVERT            (1U)
#define STEPPER_STEPS_PER_BEAM_DEGREE       (8.8889f)
/* The logical position assigned at power-on; this does not perform homing. */
#define STEPPER_INITIAL_POSITION_STEPS      (0)
/* Adjust only after confirming the mechanical safe travel. */
#define STEPPER_MAX_BEAM_ANGLE_DEG          (21.0f)
#define STEPPER_MAX_POSITION_STEPS          \
    ((int32_t)(STEPPER_MAX_BEAM_ANGLE_DEG * STEPPER_STEPS_PER_BEAM_DEGREE + 0.5f))
#define STEPPER_POSITION_TOLERANCE_STEPS    (1)
#define STEPPER_ANGLE_LOOP_KP_HZ_PER_STEP   (8.0f)

/* Vision-ball PID: input is mm and mm/s, output is signed STEP frequency.
 * Kp: Hz/mm, Ki: Hz/(mm*s), Kd: Hz*s/mm.
 */
#define STEPPER_BEAM_PID_POSITION_TARGET_MM     (0.0f)
#define STEPPER_BEAM_PID_KP_HZ_PER_MM             (12.0f)
#define STEPPER_BEAM_PID_KI_HZ_PER_MM_S           (0.0f)
#define STEPPER_BEAM_PID_KD_HZ_S_PER_MM           (0.60f)
#define STEPPER_BEAM_PID_INTEGRAL_LIMIT_MM_S     (100.0f)
#define STEPPER_BEAM_PID_OUTPUT_LIMIT_HZ          (600.0f)
/* Set to 1 if the confirmed ball-to-beam correction direction is opposite. */
#define STEPPER_BEAM_PID_OUTPUT_INVERT           (0U)
/* PID task starts with the driver enabled; no STEP pulses are requested until
 * a fresh, valid vision sample is received. */
#define STEPPER_BEAM_PID_ENABLE_ON_START          (1U)

void stepper_init(void);

void stepper_enable(bool enable);
bool stepper_is_enabled(void);

void stepper_set_speed_target(float speedHz);
void stepper_set_speed_limit_hz(float limitHz);
float stepper_get_speed_limit_hz(void);
float stepper_get_speed_target_hz(void);
void stepper_service(float dtSeconds);
void stepper_stop_smooth(void);
void stepper_emergency_stop(void);
bool stepper_clear_emergency_inhibit(void);
bool stepper_is_emergency_inhibited(void);

int32_t stepper_get_position_steps(void);
bool stepper_set_zero_position(void);
bool stepper_is_moving(void);
bool stepper_is_limit_active(void);
void stepper_clear_limit_flag(void);
float stepper_get_current_speed_hz(void);
float stepper_get_beam_angle_deg(void);
void stepper_set_beam_target_deg(float beamAngleDeg);
float stepper_get_beam_target_deg(void);
void stepper_update_beam_position_loop(void);
void stepper_update_beam_pid(float ballPositionMm, float ballVelocityMmS,
                             bool visionValid, float dtSeconds);
void stepper_reset_beam_pid(void);

bool stepper_start(int32_t steps, uint32_t frequencyHz);
bool stepper_start_dir(uint32_t steps, uint32_t frequencyHz, bool dirHigh);
bool stepper_start_continuous(uint32_t frequencyHz, bool dirHigh);
void stepper_stop(void);

bool stepper_is_busy(void);
bool stepper_get_direction(void);
uint32_t stepper_get_frequency_hz(void);
uint32_t stepper_get_remaining_steps(void);

void stepper_timer_irq_handler(void);

#endif
