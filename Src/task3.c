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
#define TASK3_UART1_RX_BUFFER_SIZE        (80U)
#define TASK3_PID_GAIN_REPORT_SCALE        (1000.0f)

typedef enum {
    TASK3_PHASE_ZEROING = 0,
    TASK3_PHASE_WAIT_BALL_ZERO = 1,
    TASK3_PHASE_GO_TO_NEG80 = 2,
    TASK3_PHASE_GO_TO_POS80 = 3,
    TASK3_PHASE_UART_TUNE = 4
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

static int32_t task3_float_to_int32(float value)
{
    return (value >= 0.0f) ? (int32_t)(value + 0.5f) :
                             (int32_t)(value - 0.5f);
}

static float task3_abs_float(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static bool task3_task2_button_pressed(void)
{
    return (DL_GPIO_readPins(TASK_BUTTON_START_PORT,
                             TASK_BUTTON_START_PIN) == 0U);
}

static void task3_skip_separators(const char **text)
{
    while ((**text == ' ') || (**text == '\t') || (**text == ',')) {
        (*text)++;
    }
}

static bool task3_parse_float(const char **text, float *value)
{
    const char *cursor = *text;
    bool negative = false;
    bool hasDigit = false;
    float parsed = 0.0f;
    float decimalScale = 0.1f;

    task3_skip_separators(&cursor);
    if ((*cursor == '+') || (*cursor == '-')) {
        negative = (*cursor == '-');
        cursor++;
    }
    while ((*cursor >= '0') && (*cursor <= '9')) {
        parsed = parsed * 10.0f + (float)(*cursor - '0');
        hasDigit = true;
        cursor++;
    }
    if (*cursor == '.') {
        cursor++;
        while ((*cursor >= '0') && (*cursor <= '9')) {
            parsed += (float)(*cursor - '0') * decimalScale;
            decimalScale *= 0.1f;
            hasDigit = true;
            cursor++;
        }
    }
    if (!hasDigit) {
        return false;
    }

    *value = negative ? -parsed : parsed;
    *text = cursor;
    return true;
}

static bool task3_apply_pid_command(const char *text)
{
    const char *cursor = text;
    float kpDegPerMm;
    float kiDegPerMmS;
    float kdDegSPerMm;
    float targetMm;

    if (!(((cursor[0] == 'P') || (cursor[0] == 'p')) &&
          ((cursor[1] == 'I') || (cursor[1] == 'i')) &&
          ((cursor[2] == 'D') || (cursor[2] == 'd')))) {
        return false;
    }
    cursor += 3;

    if (!task3_parse_float(&cursor, &kpDegPerMm) ||
        !task3_parse_float(&cursor, &kiDegPerMmS) ||
        !task3_parse_float(&cursor, &kdDegSPerMm) ||
        !task3_parse_float(&cursor, &targetMm)) {
        return false;
    }
    task3_skip_separators(&cursor);
    if ((*cursor != '\0') || (kpDegPerMm < 0.0f) ||
        (kiDegPerMmS < 0.0f) || (kdDegSPerMm < 0.0f) ||
        (kpDegPerMm > STEPPER_BEAM_PID_GAIN_LIMIT) ||
        (kiDegPerMmS > STEPPER_BEAM_PID_GAIN_LIMIT) ||
        (kdDegSPerMm > STEPPER_BEAM_PID_GAIN_LIMIT) ||
        (targetMm < -STEPPER_BEAM_PID_TARGET_LIMIT_MM) ||
        (targetMm > STEPPER_BEAM_PID_TARGET_LIMIT_MM)) {
        return false;
    }

    return stepper_set_beam_pid_gains(kpDegPerMm, kiDegPerMmS,
                                      kdDegSPerMm) &&
           stepper_set_beam_pid_target_mm(targetMm);
}

static void task3_uart1_monitor_init(void)
{
    /* Task3 owns UART1: foreground code polls RX, so the Bluetooth ISR
     * cannot consume tuning bytes from the same peripheral. */
    DL_UART_Main_disableInterrupt(UART_1_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_DisableIRQ(UART_1_INST_INT_IRQN);
}

static void task3_uart1_send_entry_marker(void)
{
    static const char text[] = "TASK3 ENTER\r\n";
    uint8_t index = 0U;

    while ((index < (sizeof(text) - 1U)) &&
           !DL_UART_Main_isTXFIFOFull(UART_1_INST)) {
        DL_UART_Main_transmitData(UART_1_INST, (uint8_t)text[index]);
        index++;
    }
}

static void task3_uart1_process_rx(char *buffer, uint8_t *length,
                                   bool *overflow, int32_t *commandStatus,
                                   bool *commandAccepted)
{
    while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)) {
        char value = (char)DL_UART_Main_receiveData(UART_1_INST);

        if (value == '\r') {
            continue;
        }
        if (value == '\n') {
            buffer[*length] = '\0';
            if (!*overflow && (*length > 0U) &&
                task3_apply_pid_command(buffer)) {
                *commandStatus = 1;
                *commandAccepted = true;
            } else if (*overflow || (*length > 0U)) {
                *commandStatus = -1;
            }
            *length = 0U;
            *overflow = false;
        } else if ((uint8_t)value >= 0x20U) {
            if (*length < (TASK3_UART1_RX_BUFFER_SIZE - 1U)) {
                buffer[*length] = value;
                (*length)++;
            } else {
                *overflow = true;
            }
        }
    }
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
                             const encoder_pwm_angle_sample_t *angleSample,
                             int32_t commandStatus)
{
    uint32_t flags = 0U;
    float kpDegPerMm;
    float kiDegPerMmS;
    float kdDegSPerMm;
    uart_cmd_vision_link_status_t visionLinkStatus;

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
    stepper_get_beam_pid_gains(&kpDegPerMm, &kiDegPerMmS, &kdDegSPerMm);
    uart_cmd_get_vision_link_status(&visionLinkStatus);

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
        !task3_append_char(buffer, length, ',') ||
        !task3_append_int32(
            buffer, length,
            task3_float_to_int32(kpDegPerMm * TASK3_PID_GAIN_REPORT_SCALE)) ||
        !task3_append_char(buffer, length, ',') ||
        !task3_append_int32(
            buffer, length,
            task3_float_to_int32(kiDegPerMmS * TASK3_PID_GAIN_REPORT_SCALE)) ||
        !task3_append_char(buffer, length, ',') ||
        !task3_append_int32(
            buffer, length,
            task3_float_to_int32(kdDegSPerMm * TASK3_PID_GAIN_REPORT_SCALE)) ||
        !task3_append_char(buffer, length, ',') ||
        !task3_append_int32(buffer, length, commandStatus) ||
        !task3_append_char(buffer, length, ',') ||
        !task3_append_int32(buffer, length,
                            (int32_t)visionLinkStatus.rxByteCount) ||
        !task3_append_char(buffer, length, ',') ||
        !task3_append_int32(buffer, length,
                            (int32_t)visionLinkStatus.acceptedFrameCount) ||
        !task3_append_char(buffer, length, ',') ||
        !task3_append_int32(buffer, length,
                            (int32_t)visionLinkStatus.badFrameCount) ||
        !task3_append_char(buffer, length, ',') ||
        !task3_append_int32(buffer, length,
                            (int32_t)visionLinkStatus.rxOverflowCount) ||
        !task3_append_char(buffer, length, '\n')) {
        *length = 0U;
        return false;
    }

    return true;
}

void task3_run(void)
{
    uint32_t nowMs;
    uint32_t pidLastUpdateMs;
    uint32_t serviceLastUpdateMs;
    uint32_t monitorLastUpdateMs;
    uart_cmd_vision_sample_t visionSample = {0};
    encoder_pwm_angle_sample_t angleSample = {0};
    task3_phase_t phase = TASK3_PHASE_ZEROING;
    bool controlActive = false;
    bool uartTuneEnabled = false;
    char monitorBuffer[TASK3_UART1_MONITOR_BUFFER_SIZE];
    uint8_t monitorLength = 0U;
    uint8_t monitorSent = 0U;
    char uart1RxBuffer[TASK3_UART1_RX_BUFFER_SIZE];
    uint8_t uart1RxLength = 0U;
    bool uart1RxOverflow = false;
    int32_t uart1CommandStatus = 0;
    bool pidCommandAccepted;
    bool task2ButtonWasPressed;

    stepper_init();
    /* Task3 may be entered directly from main(), bypassing the menu's
     * UART0 setup.  Initialize the vision receiver before processing it. */
    uart_cmd_init();
    task3_uart1_monitor_init();
    task3_uart1_send_entry_marker();
    encoder_pwm_angle_init();

    stepper_set_beam_pid_target_mm(TASK3_FIRST_TARGET_MM);
    stepper_enable(true);
    stepper_set_beam_target_deg(TASK3_ZERO_TARGET_DEG);

    nowMs = delay_get_ms();
    pidLastUpdateMs = nowMs;
    serviceLastUpdateMs = nowMs;
    monitorLastUpdateMs = nowMs;
    task2ButtonWasPressed = task3_task2_button_pressed();

    while (1) {
        bool task2ButtonPressed = task3_task2_button_pressed();

        if (task2ButtonPressed && !task2ButtonWasPressed) {
            stepper_stop();
            uart_cmd_deinit();
            return;
        }
        task2ButtonWasPressed = task2ButtonPressed;

        pidCommandAccepted = false;
        uart_cmd_process();
        task3_uart1_process_rx(uart1RxBuffer, &uart1RxLength,
                               &uart1RxOverflow, &uart1CommandStatus,
                               &pidCommandAccepted);
        if (pidCommandAccepted) {
            uartTuneEnabled = true;
            if (controlActive) {
                phase = TASK3_PHASE_UART_TUNE;
            }
        }
        if (UART_CMD_UART0_RX_MIRROR_TO_UART1_ENABLE == 0U) {
            task3_uart1_monitor_send_pending(monitorBuffer, monitorLength,
                                             &monitorSent);
        }

        nowMs = delay_get_ms();
        (void)encoder_pwm_angle_get_sample(&angleSample);

        if ((uint32_t)(nowMs - pidLastUpdateMs) >= TASK3_BALL_PID_PERIOD_MS) {
            bool visionValid = uart_cmd_get_vision_sample(&visionSample);
            bool beamControlReady = angleSample.fresh;
            float pidDtSeconds =
                (float)(nowMs - pidLastUpdateMs) / 1000.0f;

            if (!controlActive) {
                /* Keep the beam at encoder zero until the ball is also seen
                 * near its visual zero.  No ball-position PID is active yet. */
                stepper_reset_beam_pid();
                stepper_set_beam_target_deg(TASK3_ZERO_TARGET_DEG);
                if (beamControlReady) {
                    stepper_update_beam_encoder_position_loop(
                        angleSample.relativeAngleDegX10, true);

                    if ((phase == TASK3_PHASE_ZEROING) &&
                        (task3_abs_float(
                             (float)angleSample.relativeAngleDegX10) <=
                         (float)TASK3_ZERO_ANGLE_TOLERANCE_DEG_X10)) {
                        phase = TASK3_PHASE_WAIT_BALL_ZERO;
                    }

                    if ((phase == TASK3_PHASE_WAIT_BALL_ZERO) &&
                        (uartTuneEnabled ||
                         (visionValid &&
                          (task3_abs_float(
                               (float)visionSample.positionX10 * 0.1f -
                               TASK3_BALL_ZERO_POSITION_MM) <=
                           TASK3_BALL_ZERO_TOLERANCE_MM)))) {
                        stepper_enable(true);
                        stepper_reset_beam_pid();
                        if (uartTuneEnabled) {
                            phase = TASK3_PHASE_UART_TUNE;
                        } else {
                            stepper_set_beam_pid_target_mm(
                                TASK3_FIRST_TARGET_MM);
                            phase = TASK3_PHASE_GO_TO_NEG80;
                        }
                        controlActive = true;
                    }
                } else {
                    stepper_set_speed_target(0.0f);
                }
            } else if (!beamControlReady) {
                stepper_reset_beam_pid();
                stepper_stop();
            }

            if (!uartTuneEnabled && (phase == TASK3_PHASE_GO_TO_NEG80) &&
                visionValid &&
                (task3_abs_float((float)visionSample.positionX10 * 0.1f -
                                 TASK3_SWITCH_TO_SECOND_POSITION_MM) <=
                 TASK3_SWITCH_TO_SECOND_TOLERANCE_MM)) {
                stepper_set_beam_pid_target_mm(TASK3_SECOND_TARGET_MM);
                phase = TASK3_PHASE_GO_TO_POS80;
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

        if ((UART_CMD_UART0_RX_MIRROR_TO_UART1_ENABLE == 0U) &&
            (monitorSent >= monitorLength) &&
            ((uint32_t)(nowMs - monitorLastUpdateMs) >=
             TASK3_UART1_MONITOR_PERIOD_MS)) {
            if (task3_build_line(monitorBuffer, &monitorLength, phase,
                                 &visionSample, &angleSample,
                                 uart1CommandStatus)) {
                monitorSent = 0U;
            }
            monitorLastUpdateMs = nowMs;
        }
    }
}
