#ifndef __TASK4_H_
#define __TASK4_H_

/* Task4: hold the ball at 0 mm while driving straight at low speed. */
#define TASK4_BALL_TARGET_MM                       (0.0f)
#define TASK4_STRAIGHT_SPEED_MM_S                  (200)
#define TASK4_SPEED_RAMP_MM_S2                     (200)
/* Encoder-distance straightness correction. Positive error means the left
 * wheel has travelled farther, so left target is reduced and right increased. */
#define TASK4_STRAIGHT_CORRECTION_KP_X1000         (20)
#define TASK4_STRAIGHT_CORRECTION_MAX_MM_S         (60)
/* Startup beam feed-forward for the vehicle acceleration transient. Flip the
 * sign after a low-speed check if the ball initially moves the wrong way. */
#define TASK4_STARTUP_BEAM_ANGLE_DEG               (0.5f)
#define TASK4_STARTUP_BEAM_HOLD_MS                 (500U)

#define TASK4_BALL_CONTROL_PERIOD_MS               (10U)
#define TASK4_CAR_CONTROL_PERIOD_MS                (10U)
#define TASK4_STEPPER_SERVICE_PERIOD_MS             (5U)    

#define TASK4_READY_POSITION_TOLERANCE_MM           (5.0f)
#define TASK4_READY_VELOCITY_TOLERANCE_MM_S        (50.0f)
#define TASK4_READY_CONFIRM_MS                     (300U)
#define TASK4_CONTROL_INPUT_LOST_ABORT_MS          (150U)

void task4_run(void);

#endif
