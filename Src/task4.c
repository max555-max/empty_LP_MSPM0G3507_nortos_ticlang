#include "task4.h"

#include <stdbool.h>
#include <stdint.h>

#include "delay.h"
#include "encoder.h"
#include "encoder_pwm_angle.h"
#include "pid.h"
#include "stepper.h"
#include "uart_cmd.h"

typedef enum {
    TASK4_STATE_WAIT_BALL = 0,
    TASK4_STATE_RUNNING,
    TASK4_STATE_SENSOR_FAULT
} task4_state_t;

static float task4_abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int32_t task4_ramp_speed(int32_t currentSpeed,
                                int32_t targetSpeed,
                                uint32_t dtMs)
{
    int32_t maximumStep;

    if (dtMs > 100U) {
        dtMs = 100U;
    }
    maximumStep = (int32_t)(((int64_t)TASK4_SPEED_RAMP_MM_S2 * dtMs +
                             999LL) / 1000LL);
    if (maximumStep < 1) {
        maximumStep = 1;
    }

    if (currentSpeed < targetSpeed) {
        currentSpeed += maximumStep;
        if (currentSpeed > targetSpeed) {
            currentSpeed = targetSpeed;
        }
    } else if (currentSpeed > targetSpeed) {
        currentSpeed -= maximumStep;
        if (currentSpeed < targetSpeed) {
            currentSpeed = targetSpeed;
        }
    }

    return currentSpeed;
}

static int32_t task4_straight_correction(int32_t travelErrorCounts)
{
    int64_t correction;

    if (travelErrorCounts >
        (TASK4_STRAIGHT_CORRECTION_MAX_MM_S * 1000LL) /
            TASK4_STRAIGHT_CORRECTION_KP_X1000) {
        return TASK4_STRAIGHT_CORRECTION_MAX_MM_S;
    }

    if (travelErrorCounts <
        (-TASK4_STRAIGHT_CORRECTION_MAX_MM_S * 1000LL) /
            TASK4_STRAIGHT_CORRECTION_KP_X1000) {
        return -TASK4_STRAIGHT_CORRECTION_MAX_MM_S;
    }

    correction = ((int64_t)travelErrorCounts *
                  TASK4_STRAIGHT_CORRECTION_KP_X1000) / 1000LL;

    if (correction > TASK4_STRAIGHT_CORRECTION_MAX_MM_S) {
        correction = TASK4_STRAIGHT_CORRECTION_MAX_MM_S;
    } else if (correction < -TASK4_STRAIGHT_CORRECTION_MAX_MM_S) {
        correction = -TASK4_STRAIGHT_CORRECTION_MAX_MM_S;
    }

    return (int32_t)correction;
}

static void task4_apply_startup_beam_bias(bool *biasActive,
                                          uint32_t startMs,
                                          uint32_t nowMs)
{
    if (!*biasActive) {
        return;
    }

    if ((uint32_t)(nowMs - startMs) >= TASK4_STARTUP_BEAM_HOLD_MS) {
        *biasActive = false;
        return;
    }

    stepper_set_beam_target_deg(stepper_get_beam_target_deg() +
                                TASK4_STARTUP_BEAM_ANGLE_DEG);
}

void task4_run(void)
{
    task4_state_t state = TASK4_STATE_WAIT_BALL;
    uart_cmd_vision_sample_t visionSample = {0};
    encoder_pwm_angle_sample_t angleSample = {0};
    uint32_t nowMs;
    uint32_t ballLastUpdateMs;
    uint32_t carLastUpdateMs;
    uint32_t serviceLastUpdateMs;
    uint32_t readyStartMs = 0U;
    uint32_t inputLostStartMs = 0U;
    uint32_t startupBiasStartMs = 0U;
    int32_t speedCommandMmS = 0;
    int32_t straightStartLeftCount = 0;
    int32_t straightStartRightCount = 0;
    bool ballControlReady = false;
    bool startupBiasActive = false;

    encoder_init();
    speed_pid_init();
    stepper_init();
    encoder_pwm_angle_init();
    uart_cmd_init();

    (void)stepper_set_beam_pid_target_mm(TASK4_BALL_TARGET_MM);
    stepper_enable(true);
    speed_pid_stop();

    nowMs = delay_get_ms();
    ballLastUpdateMs = nowMs;
    carLastUpdateMs = nowMs;
    serviceLastUpdateMs = nowMs;

    while (1) {
        uart_cmd_process();
        nowMs = delay_get_ms();
        (void)uart_cmd_get_vision_sample(&visionSample);
        (void)encoder_pwm_angle_get_sample(&angleSample);

        ballControlReady = visionSample.valid && angleSample.fresh &&
                           !angleSample.safeLimitActive;

        if ((uint32_t)(nowMs - ballLastUpdateMs) >=
            TASK4_BALL_CONTROL_PERIOD_MS) {
            uint32_t ballDtMs = nowMs - ballLastUpdateMs;
            float ballPositionMm = (float)visionSample.positionX10 * 0.1f;
            float ballVelocityMmS = (float)visionSample.velocityX10 * 0.1f;

            stepper_update_beam_pid(ballPositionMm, ballVelocityMmS,
                                    ballControlReady,
                                    (float)ballDtMs / 1000.0f);
            stepper_update_beam_encoder_position_loop(
                angleSample.relativeAngleDegX10, ballControlReady);

            if ((state == TASK4_STATE_RUNNING) && ballControlReady) {
                task4_apply_startup_beam_bias(&startupBiasActive,
                                              startupBiasStartMs, nowMs);
            }

            if (state == TASK4_STATE_WAIT_BALL) {
                bool ballStable = ballControlReady &&
                    (task4_abs_float(ballPositionMm -
                                     TASK4_BALL_TARGET_MM) <=
                     TASK4_READY_POSITION_TOLERANCE_MM) &&
                    (task4_abs_float(ballVelocityMmS) <=
                     TASK4_READY_VELOCITY_TOLERANCE_MM_S);

                if (ballStable) {
                    if (readyStartMs == 0U) {
                        readyStartMs = nowMs;
                    } else if ((uint32_t)(nowMs - readyStartMs) >=
                               TASK4_READY_CONFIRM_MS) {
                        speedCommandMmS = 0;
                        straightStartLeftCount = encoder_get_left_count();
                        straightStartRightCount = encoder_get_right_count();
                        startupBiasStartMs = nowMs;
                        startupBiasActive = true;
                        state = TASK4_STATE_RUNNING;
                    }
                } else {
                    readyStartMs = 0U;
                }
            } else if (state == TASK4_STATE_RUNNING) {
                if (ballControlReady) {
                    inputLostStartMs = 0U;
                } else {
                    speedCommandMmS = 0;
                    if (inputLostStartMs == 0U) {
                        inputLostStartMs = nowMs;
                    } else if ((uint32_t)(nowMs - inputLostStartMs) >=
                               TASK4_CONTROL_INPUT_LOST_ABORT_MS) {
                        state = TASK4_STATE_SENSOR_FAULT;
                    }
                }
            }

            ballLastUpdateMs = nowMs;
        }

        if ((uint32_t)(nowMs - carLastUpdateMs) >=
            TASK4_CAR_CONTROL_PERIOD_MS) {
            uint32_t carDtMs = nowMs - carLastUpdateMs;

            if ((state == TASK4_STATE_RUNNING) && ballControlReady) {
                int32_t leftTravelCounts = encoder_get_left_count() -
                                           straightStartLeftCount;
                int32_t rightTravelCounts = encoder_get_right_count() -
                                            straightStartRightCount;
                int32_t travelErrorCounts = leftTravelCounts -
                                             rightTravelCounts;
                int32_t straightCorrection =
                    task4_straight_correction(travelErrorCounts);
                int32_t leftTargetMmS;
                int32_t rightTargetMmS;

                speedCommandMmS = task4_ramp_speed(
                    speedCommandMmS, TASK4_STRAIGHT_SPEED_MM_S, carDtMs);
                leftTargetMmS = speedCommandMmS - straightCorrection;
                rightTargetMmS = speedCommandMmS + straightCorrection;
                speed_pid_set_speed(leftTargetMmS, rightTargetMmS);
            } else {
                speedCommandMmS = 0;
                speed_pid_stop();
            }
            speed_pid_control_update();
            carLastUpdateMs = nowMs;
        }

        if ((uint32_t)(nowMs - serviceLastUpdateMs) >=
            TASK4_STEPPER_SERVICE_PERIOD_MS) {
            stepper_service((float)(nowMs - serviceLastUpdateMs) / 1000.0f);
            serviceLastUpdateMs = nowMs;
        }

        delay_ms(1U);
    }
}
