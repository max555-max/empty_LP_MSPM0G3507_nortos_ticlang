#include "task3.h"

#include <stdbool.h>
#include <stdint.h>

#include "delay.h"
#include "encoder_pwm_angle.h"
#include "stepper.h"
#include "ti_msp_dl_config.h"
#include "uart_cmd.h"

#define TASK3_BALL_PID_PERIOD_MS          (10U)
#define TASK3_BALL_SERVICE_PERIOD_MS      (5U)
#define TASK3_UART1_MONITOR_PERIOD_MS     (100U)
#define TASK3_UART1_MONITOR_BUFFER_SIZE   (220U)

typedef enum {
    TASK3_PHASE_STARTUP_WAIT = 0,
    TASK3_PHASE_GO_TO_NEG80 = 1,
    TASK3_PHASE_GO_TO_POS80 = 2
} task3_phase_t;

static bool task3_append_char(char *buffer, uint8_t *length, char value)
{
    if (*length >= (TASK3_UART1_MONITOR_BUFFER_SIZE - 1U)) {
        return false;
    }

    buffer[*length] = value;
    (*length)++;
    return true;
}

static bool task3_append_string(char *buffer, uint8_t *length,
                                const char *text)
{
    while (*text != '\0') {
        if (!task3_append_char(buffer, length, *text++)) {
            return false;
        }
    }

    return true;
}

static bool task3_append_int32(char *buffer, uint8_t *length, int32_t value)
{
    char digits[11];
    uint8_t index = 0U;
    uint32_t magnitude;

    if (value < 0) {
        if (!task3_append_char(buffer, length, '-')) {
            return false;
        }
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }

    do {
        digits[index++] = (char)('0' + (magnitude % 10U));
        magnitude /= 10U;
    } while ((magnitude != 0U) && (index < sizeof(digits)));

    while (index > 0U) {
        if (!task3_append_char(buffer, length, digits[--index])) {
            return false;
        }
    }

    return true;
}

static int32_t task3_float_to_x10(float value)
{
    return (value >= 0.0f) ? (int32_t)(value * 10.0f + 0.5f) :
                             (int32_t)(value * 10.0f - 0.5f);
}

static void task3_uart1_monitor_init(void)
{
    DL_UART_Main_disableInterrupt(UART_1_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_DisableIRQ(UART_1_INST_INT_IRQN);
}

static void task3_uart1_monitor_send_pending(const char *buffer,
                                             uint8_t length,
                                             uint8_t *sent)
{
    while ((*sent < length) && !DL_UART_Main_isTXFIFOFull(UART_1_INST)) {
        DL_UART_Main_transmitData(UART_1_INST, (uint8_t)buffer[*sent]);
        (*sent)++;
    }
}

static bool task3_build_line(char *buffer, uint8_t *length,
                             task3_phase_t phase,
                             const uart_cmd_vision_sample_t *visionSample,
                             const encoder_pwm_angle_sample_t *angleSample)
{
    uint32_t flags = 0U;

    if (visionSample->valid) {
        flags |= 1U << 0;
    }
    if (visionSample->fresh) {
        flags |= 1U << 1;
    }
    if (visionSample->linkOnline) {
        flags |= 1U << 2;
    }
    if (stepper_is_enabled()) {
        flags |= 1U << 3;
    }
    if (angleSample->fresh) {
        flags |= 1U << 4;
    }
    if (angleSample->safeLimitActive) {
        flags |= 1U << 5;
    }
    if (stepper_is_emergency_inhibited()) {
        flags |= 1U << 6;
    }
    if (stepper_is_moving()) {
        flags |= 1U << 7;
    }

    *length = 0U;
    if (!task3_append_string(buffer, length, "samples:") ||
        !task3_append_int32(buffer, length, (int32_t)phase) ||
        !task3_append_char(buffer, length, ',') ||
        !task3_append_int32(buffer, length, visionSample->positionX10) ||
        !task3_append_char(buffer, length, ',') ||
        !task3_append_int32(buffer, length, visionSample->velocityX10) ||
        !task3_append_char(buffer, length, ',') ||
        !task3_append_int32(
            buffer, length, task3_float_to_x10(stepper_get_beam_pid_target_mm())) ||
        !task3_append_char(buffer, length, ',') ||
        !task3_append_int32(
            buffer, length, task3_float_to_x10(stepper_get_beam_target_deg())) ||
        !task3_append_char(buffer, length, ',') ||
        !task3_append_int32(
            buffer, length, (int32_t)stepper_get_speed_target_hz()) ||
        !task3_append_char(buffer, length, ',') ||
        !task3_append_int32(
            buffer, length, (int32_t)stepper_get_current_speed_hz()) ||
        !task3_append_char(buffer, length, ',') ||
        !task3_append_int32(
            buffer, length, angleSample->relativeAngleDegX10) ||
        !task3_append_char(buffer, length, ',') ||
        !task3_append_int32(buffer, length, (int32_t)flags) ||
        !task3_append_char(buffer, length, '\n')) {
        *length = 0U;
        return false;
    }

    return true;
}

void task3_run(void)
{
    uint32_t nowMs;
    uint32_t startupMs;
    uint32_t targetStartMs = 0U;
    uint32_t pidLastUpdateMs;
    uint32_t serviceLastUpdateMs;
    uint32_t monitorLastUpdateMs;
    uart_cmd_vision_sample_t visionSample = {0};
    encoder_pwm_angle_sample_t angleSample = {0};
    task3_phase_t phase = TASK3_PHASE_STARTUP_WAIT;
    bool controlActive = false;
    char monitorBuffer[TASK3_UART1_MONITOR_BUFFER_SIZE];
    uint8_t monitorLength = 0U;
    uint8_t monitorSent = 0U;

    stepper_init();
    task3_uart1_monitor_init();
    encoder_pwm_angle_init();
    uart_cmd_init();

    stepper_set_beam_pid_target_mm(TASK3_FIRST_TARGET_MM);
    stepper_enable(true);

    nowMs = delay_get_ms();
    startupMs = nowMs;
    pidLastUpdateMs = nowMs;
    serviceLastUpdateMs = nowMs;
    monitorLastUpdateMs = nowMs;

    while (1) {
        uart_cmd_process();
        task3_uart1_monitor_send_pending(monitorBuffer, monitorLength,
                                         &monitorSent);

        nowMs = delay_get_ms();
        (void)encoder_pwm_angle_get_sample(&angleSample);

        if (!controlActive &&
            ((uint32_t)(nowMs - startupMs) >= TASK3_STARTUP_WAIT_MS)) {
            stepper_set_beam_pid_target_mm(TASK3_FIRST_TARGET_MM);
            stepper_enable(true);
            stepper_reset_beam_pid();
            phase = TASK3_PHASE_GO_TO_NEG80;
            targetStartMs = nowMs;
            controlActive = true;
        }

        if ((uint32_t)(nowMs - pidLastUpdateMs) >= TASK3_BALL_PID_PERIOD_MS) {
            bool visionValid = uart_cmd_get_vision_sample(&visionSample);
            bool beamControlReady = angleSample.fresh;
            float pidDtSeconds =
                (float)(nowMs - pidLastUpdateMs) / 1000.0f;

            if (!controlActive) {
                stepper_reset_beam_pid();
                stepper_stop();
            } else if (!beamControlReady) {
                stepper_reset_beam_pid();
                stepper_stop();
            }

            if ((phase == TASK3_PHASE_GO_TO_NEG80) &&
                ((uint32_t)(nowMs - targetStartMs) >=
                 TASK3_SWITCH_TARGET_DELAY_MS)) {
                stepper_set_beam_pid_target_mm(TASK3_SECOND_TARGET_MM);
                phase = TASK3_PHASE_GO_TO_POS80;
                targetStartMs = nowMs;
            }

            if (controlActive) {
                stepper_update_beam_pid(
                    (float)visionSample.positionX10 * 0.1f,
                    (float)visionSample.velocityX10 * 0.1f,
                    visionValid && beamControlReady,
                    pidDtSeconds);
                stepper_update_beam_encoder_position_loop(
                    angleSample.relativeAngleDegX10,
                    visionValid && beamControlReady);
            }
            pidLastUpdateMs = nowMs;
        }

        if ((uint32_t)(nowMs - serviceLastUpdateMs) >=
            TASK3_BALL_SERVICE_PERIOD_MS) {
            float serviceDtSeconds =
                (float)(nowMs - serviceLastUpdateMs) / 1000.0f;

            stepper_service(serviceDtSeconds);
            serviceLastUpdateMs = nowMs;
        }

        if ((monitorSent >= monitorLength) &&
            ((uint32_t)(nowMs - monitorLastUpdateMs) >=
             TASK3_UART1_MONITOR_PERIOD_MS)) {
            if (task3_build_line(monitorBuffer, &monitorLength, phase,
                                 &visionSample, &angleSample)) {
                monitorSent = 0U;
            }
            monitorLastUpdateMs = nowMs;
        }
    }
}

void task3_motor_zero_run(void)
{
    uint32_t nowMs;
    uint32_t zeroStartMs;
    uint32_t positionLastUpdateMs;
    uint32_t serviceLastUpdateMs;
    encoder_pwm_angle_sample_t angleSample = {0};

    stepper_init();
    encoder_pwm_angle_init();
    stepper_enable(true);
    stepper_set_beam_target_deg(0.0f);

    nowMs = delay_get_ms();
    zeroStartMs = nowMs;
    positionLastUpdateMs = nowMs;
    serviceLastUpdateMs = nowMs;

    while (1) {
        nowMs = delay_get_ms();
        (void)encoder_pwm_angle_get_sample(&angleSample);

        if ((uint32_t)(nowMs - zeroStartMs) >= TASK3_STARTUP_WAIT_MS) {
            stepper_stop();
            task3_run();
            return;
        }

        if ((uint32_t)(nowMs - positionLastUpdateMs) >=
            TASK3_BALL_PID_PERIOD_MS) {
            if (angleSample.fresh) {
                stepper_update_beam_encoder_position_loop(
                    angleSample.relativeAngleDegX10, true);
            } else {
                stepper_set_speed_target(0.0f);
            }
            positionLastUpdateMs = nowMs;
        }

        if ((uint32_t)(nowMs - serviceLastUpdateMs) >=
            TASK3_BALL_SERVICE_PERIOD_MS) {
            float serviceDtSeconds =
                (float)(nowMs - serviceLastUpdateMs) / 1000.0f;

            stepper_service(serviceDtSeconds);
            serviceLastUpdateMs = nowMs;
        }
    }
}
