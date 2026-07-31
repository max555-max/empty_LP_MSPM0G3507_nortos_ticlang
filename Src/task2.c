#include <stdbool.h>
#include <stdint.h>
#include <limits.h>

#include "bluetooth.h"
#include "delay.h"
#include "encoder.h"
#include "gray_serial.h"
#include "line_track.h"
#include "oled.h"
#include "pid.h"
#include "stepper.h"
#include "task2.h"
#include "ti_msp_dl_config.h"
#include "uart_cmd.h"

#define TASK2_OLED_REFRESH_PERIOD_MS      (200U)
#define TASK2_STOP_ACTIVE_SENSOR_COUNT    (3U)
#define TASK2_STOP_ENABLE_DELAY_MS        (2000U)
#define TASK2_STOP_APPROACH_SPEED_MM_S    (300)
#define TASK2_STOP_APPROACH_DISTANCE_MM   (150)
#define TASK2_STOP_APPROACH_SLOW_DELAY_MS (0U)
#define TASK2_GRAY_STARTUP_DISCARD_COUNT  (20U)
#define TASK2_GRAY_STARTUP_DISCARD_MS     (2U)
#define TASK2_BALL_PID_PERIOD_MS           (10U)
#define TASK2_BALL_SERVICE_PERIOD_MS       (5U)
#define TASK2_UART1_MONITOR_ENABLE          (1U)
#define TASK2_UART1_MONITOR_PERIOD_MS       (100U)
#define TASK2_UART1_MONITOR_BUFFER_SIZE     (96U)
#define TASK2_STEPPER_TEST_LINE_SIZE         (64U)
#define TASK2_STEPPER_TEST_POSITION_PERIOD_MS (10U)
#define TASK2_STEPPER_TEST_SERVICE_PERIOD_MS  (5U)

typedef enum {
    TASK2_LINE_STATE_RUNNING = 0,
    TASK2_LINE_STATE_APPROACHING_STOP,
    TASK2_LINE_STATE_STOPPED
} task2_line_state_t;

/*
 * Task2 使用竞速参数。
 * line_track.h 中的默认宏作为稳定参数保留给其他任务使用。
 * 这里先让竞速参数与当前稳定参数一致，后续可单独调高。
 */
#define TASK2_RACE_LINE_BASE_SPEED_MM_S       (400)
#define TASK2_RACE_LINE_TURN_KP               (100)
#define TASK2_RACE_LINE_TURN_KD               (20)
#define TASK2_RACE_LINE_MAX_CORRECTION_MM_S   (350)
 
static void task2_apply_race_line_params(void)
{
    line_track_set_base_speed(TASK2_RACE_LINE_BASE_SPEED_MM_S);
    line_track_set_turn_gains(
        TASK2_RACE_LINE_TURN_KP,
        TASK2_RACE_LINE_TURN_KD);
    line_track_set_max_correction(
        TASK2_RACE_LINE_MAX_CORRECTION_MM_S);
}

static void task2_discard_startup_gray_samples(void)
{
    for (uint8_t i = 0U; i < TASK2_GRAY_STARTUP_DISCARD_COUNT; i++) {
        (void)gray_serial_read();
        delay_ms(TASK2_GRAY_STARTUP_DISCARD_MS);
    }
}

static uint8_t task2_count_active_gray_sensors(uint8_t raw)
{
    static const uint8_t channelBitMap[8] = {
        1U, 2U, 3U, 4U, 5U, 6U, 7U, 0U
    };

    uint8_t activeCount = 0U;

    for (uint8_t i = 0U; i < 8U; i++) {
        uint8_t level =
            (uint8_t)((raw >> channelBitMap[i]) & 0x01U);

        if (level == LINE_TRACK_ACTIVE_LEVEL) {
            activeCount++;
        }
    }

    return activeCount;
}


static bool task2_should_exit_line_track(uint8_t activeCount)
{
    return activeCount >= TASK2_STOP_ACTIVE_SENSOR_COUNT;
}

static int32_t task2_abs_i32(int32_t value)
{
    if (value < 0) {
        return -value;
    }

    return value;
}

static int32_t task2_encoder_counts_to_mm(int32_t counts)
{
    int64_t distanceMmX1000;
    int64_t countsPerWheelRevX1000 =
        (int64_t)ENCODER_LINES_PER_MOTOR_REV *
        ENCODER_QUADRATURE_MULTIPLIER *
        ENCODER_GEAR_RATIO_X1000;
    int64_t wheelCircumferenceMmX1000 =
        ((int64_t)ENCODER_WHEEL_DIAMETER_MM * 3141593LL) / 1000LL;

    distanceMmX1000 =
        (int64_t)task2_abs_i32(counts) * wheelCircumferenceMmX1000;

    return (int32_t)((distanceMmX1000 +
                      (countsPerWheelRevX1000 / 2LL)) /
                     countsPerWheelRevX1000);
}

static int32_t task2_get_traveled_distance_mm(int32_t startLeftCount,
                                              int32_t startRightCount)
{
    int32_t leftDistanceMm = task2_encoder_counts_to_mm(
        encoder_get_left_count() - startLeftCount);
    int32_t rightDistanceMm = task2_encoder_counts_to_mm(
        encoder_get_right_count() - startRightCount);

    return (leftDistanceMm + rightDistanceMm) / 2;
}

static bool task2_car_speed_is_zero(void)
{
    return (encoder_get_left_speed_mm_s() == 0) &&
           (encoder_get_right_speed_mm_s() == 0);
}

static void task2_oled_print_line_header(uint8_t page, const char *text)
{
    oled_clear_line(page);
    oled_print_string(text);
}

static void task2_oled_update(bool oledOk,
                              uint8_t grayRaw,
                              uint8_t activeCount,
                              bool stopArmed,
                              task2_line_state_t lineState,
                              uint32_t elapsedMs)
{
    static uint32_t lastRefreshMs = 0U;
    uint32_t nowMs;
    line_track_status_t status;

    if (!oledOk) {
        return;
    }

    nowMs = delay_get_ms();
    if ((uint32_t)(nowMs - lastRefreshMs) < TASK2_OLED_REFRESH_PERIOD_MS) {
        return;
    }
    lastRefreshMs = nowMs;

    line_track_get_status(&status);

    task2_oled_print_line_header(0U, "Task2 Line");

    task2_oled_print_line_header(1U, "RAW:");
    oled_print_hex_u8(grayRaw);
    oled_print_string(" A:");
    oled_print_int(activeCount);
    if (lineState == TASK2_LINE_STATE_STOPPED) {
        status.correction = 0;
        status.leftTargetMmS = 0;
        status.rightTargetMmS = 0;
        oled_print_string(" STOP");
    } else if (lineState == TASK2_LINE_STATE_APPROACHING_STOP) {
        oled_print_string(" SLOW");
    } else if (!stopArmed) {
        oled_print_string(" WAIT");
    } else if (!status.lineDetected) {
        oled_print_string(" LOST");
    } else {
        oled_print_string(" RUN");
    }

    task2_oled_print_line_header(2U, "ERR:");
    oled_print_int(status.error);
    oled_print_string(" C:");
    oled_print_int(status.correction);

    task2_oled_print_line_header(3U, "L:");
    oled_print_int(status.leftTargetMmS);
    oled_print_string(" R:");
    oled_print_int(status.rightTargetMmS);

    task2_oled_print_line_header(4U, "Kp:");
    oled_print_int(line_track_get_turn_kp());
    oled_print_string(" Kd:");
    oled_print_int(line_track_get_turn_kd());

    task2_oled_print_line_header(5U, "Base:");
    oled_print_int(line_track_get_base_speed());
    oled_print_string(" mm/s");

    oled_print_time_large(elapsedMs);
}

static bool task2_uart1_monitor_append_char(char *buffer, uint8_t *length,
                                             char value)
{
    if (*length >= (TASK2_UART1_MONITOR_BUFFER_SIZE - 1U)) {
        return false;
    }

    buffer[*length] = value;
    (*length)++;
    return true;
}

static bool task2_uart1_monitor_append_int32(char *buffer, uint8_t *length,
                                              int32_t value)
{
    char digits[11];
    uint8_t index = 0U;
    uint32_t magnitude;

    if (value < 0) {
        if (!task2_uart1_monitor_append_char(buffer, length, '-')) {
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
        if (!task2_uart1_monitor_append_char(buffer, length, digits[--index])) {
            return false;
        }
    }

    return true;
}

static int32_t task2_float_to_x100(float value)
{
    return (value >= 0.0f) ? (int32_t)(value * 100.0f + 0.5f) :
                             (int32_t)(value * 100.0f - 0.5f);
}

static void task2_uart1_monitor_init(void)
{
#if (TASK2_UART1_MONITOR_ENABLE != 0U)
    /* UART1 RX and its Bluetooth handler stay inactive in monitor-only mode. */
    DL_UART_Main_disableInterrupt(UART_1_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_DisableIRQ(UART_1_INST_INT_IRQN);
#endif
}

static void task2_uart1_monitor_send_pending(const char *buffer,
                                             uint8_t length,
                                             uint8_t *sent)
{
#if (TASK2_UART1_MONITOR_ENABLE != 0U)
    while ((*sent < length) &&
           !DL_UART_Main_isTXFIFOFull(UART_1_INST)) {
        DL_UART_Main_transmitData(UART_1_INST, (uint8_t)buffer[*sent]);
        (*sent)++;
    }
#else
    (void)buffer;
    (void)length;
    (void)sent;
#endif
}

static bool task2_uart1_monitor_build_line(char *buffer, uint8_t *length,
                                           const uart_cmd_vision_sample_t *sample)
{
    uart_cmd_vision_link_status_t linkStatus;
    uint32_t flags = 0U;
    /* VOFA FireWater: samples:ch0,ch1,...\n */
    static const char prefix[] = "samples:";

    if (sample->valid) {
        flags |= 1U << 0;
    }
    if (sample->fresh) {
        flags |= 1U << 1;
    }
    if (sample->linkOnline) {
        flags |= 1U << 2;
    }
    if (stepper_is_enabled()) {
        flags |= 1U << 3;
    }
    if (stepper_is_emergency_inhibited()) {
        flags |= 1U << 4;
    }
    if (stepper_is_limit_active()) {
        flags |= 1U << 5;
    }
    if (stepper_is_moving()) {
        flags |= 1U << 6;
    }

    uart_cmd_get_vision_link_status(&linkStatus);
    *length = 0U;
    for (uint8_t i = 0U; i < (sizeof(prefix) - 1U); i++) {
        if (!task2_uart1_monitor_append_char(buffer, length, prefix[i])) {
            return false;
        }
    }

    if (!task2_uart1_monitor_append_int32(buffer, length, sample->positionX10) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(buffer, length, sample->velocityX10) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(
            buffer, length, (int32_t)stepper_get_speed_target_hz()) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(
            buffer, length, (int32_t)stepper_get_current_speed_hz()) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(
            buffer, length, stepper_get_position_steps()) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(buffer, length, (int32_t)flags) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(
            buffer, length, (int32_t)linkStatus.ballStateFrameRateHz) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(
            buffer, length, (int32_t)linkStatus.badFrameRatioPermille) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(
            buffer, length, (int32_t)linkStatus.rxOverflowCount) ||
        !task2_uart1_monitor_append_char(buffer, length, '\n')) {
        *length = 0U;
        return false;
    }

    return true;
}

static char task2_stepper_test_to_upper(char value)
{
    if ((value >= 'a') && (value <= 'z')) {
        return (char)(value - ('a' - 'A'));
    }

    return value;
}

static const char *task2_stepper_test_skip_separators(const char *text)
{
    while ((*text == ' ') || (*text == '\t') || (*text == ',') ||
           (*text == ':') || (*text == '=')) {
        text++;
    }

    return text;
}

static bool task2_stepper_test_read_word(const char **text, char *word,
                                         uint8_t wordSize)
{
    const char *p = task2_stepper_test_skip_separators(*text);
    uint8_t index = 0U;

    while ((*p != '\0') && (*p != ' ') && (*p != '\t') && (*p != ',') &&
           (*p != ':') && (*p != '=')) {
        if (index >= (wordSize - 1U)) {
            return false;
        }
        word[index++] = task2_stepper_test_to_upper(*p);
        p++;
    }

    if (index == 0U) {
        return false;
    }
    word[index] = '\0';
    *text = p;
    return true;
}

static bool task2_stepper_test_word_equals(const char *word, const char *value)
{
    while ((*word != '\0') && (*value != '\0')) {
        if (*word != *value) {
            return false;
        }
        word++;
        value++;
    }

    return (*word == '\0') && (*value == '\0');
}

static bool task2_stepper_test_read_int32(const char **text, int32_t *value)
{
    const char *p = task2_stepper_test_skip_separators(*text);
    int64_t magnitude = 0;
    int32_t sign = 1;
    bool hasDigit = false;

    if (*p == '-') {
        sign = -1;
        p++;
    } else if (*p == '+') {
        p++;
    }

    while ((*p >= '0') && (*p <= '9')) {
        hasDigit = true;
        magnitude = magnitude * 10 + (int64_t)(*p - '0');
        if (magnitude > ((sign < 0) ? 2147483648LL : INT32_MAX)) {
            return false;
        }
        p++;
    }

    if (!hasDigit) {
        return false;
    }

    if (sign < 0) {
        *value = (magnitude == 2147483648LL) ? INT32_MIN :
                 -(int32_t)magnitude;
    } else {
        *value = (int32_t)magnitude;
    }
    *text = p;
    return true;
}

static bool task2_stepper_test_read_bool(const char **text, bool *value)
{
    int32_t parsed;

    if (!task2_stepper_test_read_int32(text, &parsed) ||
        ((parsed != 0) && (parsed != 1))) {
        return false;
    }

    *value = (parsed != 0);
    return true;
}

static bool task2_stepper_test_has_no_more(const char *text)
{
    return *task2_stepper_test_skip_separators(text) == '\0';
}

static bool task2_uart1_append_string(char *buffer, uint8_t *length,
                                       const char *text)
{
    while (*text != '\0') {
        if (!task2_uart1_monitor_append_char(buffer, length, *text++)) {
            return false;
        }
    }

    return true;
}

static void task2_stepper_test_queue_text(char *buffer, uint8_t *length,
                                          uint8_t *sent, const char *text)
{
    if (*sent < *length) {
        return;
    }

    *length = 0U;
    if (!task2_uart1_append_string(buffer, length, text)) {
        *length = 0U;
    }
    *sent = 0U;
}

static void task2_stepper_test_queue_status(char *buffer, uint8_t *length,
                                            uint8_t *sent)
{
    if (*sent < *length) {
        return;
    }

    *length = 0U;
    if (!task2_uart1_append_string(buffer, length, "STAT EN:") ||
        !task2_uart1_monitor_append_int32(buffer, length,
                                          stepper_is_enabled() ? 1 : 0) ||
        !task2_uart1_append_string(buffer, length, " BUSY:") ||
        !task2_uart1_monitor_append_int32(buffer, length,
                                          stepper_is_busy() ? 1 : 0) ||
        !task2_uart1_append_string(buffer, length, " POS:") ||
        !task2_uart1_monitor_append_int32(buffer, length,
                                          stepper_get_position_steps()) ||
        !task2_uart1_append_string(buffer, length, " ANGX100:") ||
        !task2_uart1_monitor_append_int32(
            buffer, length, task2_float_to_x100(stepper_get_beam_angle_deg())) ||
        !task2_uart1_append_string(buffer, length, " TGT:") ||
        !task2_uart1_monitor_append_int32(
            buffer, length, task2_float_to_x100(stepper_get_beam_target_deg())) ||
        !task2_uart1_append_string(buffer, length, " F:") ||
        !task2_uart1_monitor_append_int32(
            buffer, length, (int32_t)stepper_get_current_speed_hz()) ||
        !task2_uart1_append_string(buffer, length, " LIM:") ||
        !task2_uart1_monitor_append_int32(buffer, length,
                                          stepper_is_limit_active() ? 1 : 0) ||
        !task2_uart1_append_string(buffer, length, " ESTOP:") ||
        !task2_uart1_monitor_append_int32(
            buffer, length, stepper_is_emergency_inhibited() ? 1 : 0) ||
        !task2_uart1_append_string(buffer, length, "\r\n")) {
        *length = 0U;
    }
    *sent = 0U;
}

static void task2_stepper_test_execute(char *line, bool *angleLoopActive,
                                       char *txBuffer, uint8_t *txLength,
                                       uint8_t *txSent)
{
    const char *p = line;
    char word[8];
    int32_t value0;
    int32_t value1;
    bool boolValue;

    if (!task2_stepper_test_read_word(&p, word, sizeof(word))) {
        task2_stepper_test_queue_text(txBuffer, txLength, txSent, "ERR\r\n");
        return;
    }

    if (task2_stepper_test_word_equals(word, "HELP")) {
        task2_stepper_test_queue_text(
            txBuffer, txLength, txSent,
            "CMD: STATUS EN 0/1 MOVE steps hz RUN hz dir STOP ESTOP RESET ZERO ANGLE deg CENTER\r\n");
    } else if (task2_stepper_test_word_equals(word, "STATUS") ||
               task2_stepper_test_word_equals(word, "GET")) {
        task2_stepper_test_queue_status(txBuffer, txLength, txSent);
    } else if (task2_stepper_test_word_equals(word, "EN") &&
               task2_stepper_test_read_bool(&p, &boolValue) &&
               task2_stepper_test_has_no_more(p)) {
        stepper_enable(boolValue);
        if (!boolValue) {
            *angleLoopActive = false;
        }
        task2_stepper_test_queue_status(txBuffer, txLength, txSent);
    } else if (task2_stepper_test_word_equals(word, "MOVE") &&
               task2_stepper_test_read_int32(&p, &value0) &&
               task2_stepper_test_read_int32(&p, &value1) &&
               task2_stepper_test_has_no_more(p) && (value1 > 0) &&
               stepper_start(value0, (uint32_t)value1)) {
        *angleLoopActive = false;
        task2_stepper_test_queue_text(txBuffer, txLength, txSent, "OK\r\n");
    } else if (task2_stepper_test_word_equals(word, "RUN") &&
               task2_stepper_test_read_int32(&p, &value0) &&
               task2_stepper_test_read_bool(&p, &boolValue) &&
               task2_stepper_test_has_no_more(p) && (value0 > 0) &&
               stepper_start_continuous((uint32_t)value0, boolValue)) {
        *angleLoopActive = false;
        task2_stepper_test_queue_text(txBuffer, txLength, txSent, "OK\r\n");
    } else if (task2_stepper_test_word_equals(word, "STOP") &&
               task2_stepper_test_has_no_more(p)) {
        *angleLoopActive = false;
        stepper_stop();
        task2_stepper_test_queue_text(txBuffer, txLength, txSent, "OK\r\n");
    } else if (task2_stepper_test_word_equals(word, "ESTOP") &&
               task2_stepper_test_has_no_more(p)) {
        *angleLoopActive = false;
        stepper_emergency_stop();
        task2_stepper_test_queue_text(txBuffer, txLength, txSent, "OK\r\n");
    } else if (task2_stepper_test_word_equals(word, "RESET") &&
               task2_stepper_test_has_no_more(p)) {
        *angleLoopActive = false;
        (void)stepper_clear_emergency_inhibit();
        task2_stepper_test_queue_text(txBuffer, txLength, txSent, "OK\r\n");
    } else if (task2_stepper_test_word_equals(word, "ZERO") &&
               task2_stepper_test_has_no_more(p) &&
               stepper_set_zero_position()) {
        *angleLoopActive = false;
        task2_stepper_test_queue_text(txBuffer, txLength, txSent, "OK\r\n");
    } else if ((task2_stepper_test_word_equals(word, "ANGLE") ||
                task2_stepper_test_word_equals(word, "CENTER")) &&
               ((task2_stepper_test_word_equals(word, "CENTER") &&
                 task2_stepper_test_has_no_more(p)) ||
                (task2_stepper_test_word_equals(word, "ANGLE") &&
                 task2_stepper_test_read_int32(&p, &value0) &&
                 task2_stepper_test_has_no_more(p))) &&
               stepper_is_enabled() && !stepper_is_moving() &&
               !stepper_is_emergency_inhibited()) {
        stepper_set_beam_target_deg(
            task2_stepper_test_word_equals(word, "CENTER") ? 0.0f :
                                                               (float)value0);
        *angleLoopActive = true;
        task2_stepper_test_queue_status(txBuffer, txLength, txSent);
    } else {
        task2_stepper_test_queue_text(txBuffer, txLength, txSent, "ERR\r\n");
    }
}

static void task2_stepper_test_process_rx(char *line, uint8_t *lineLength,
                                          bool *angleLoopActive,
                                          char *txBuffer, uint8_t *txLength,
                                          uint8_t *txSent)
{
    while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)) {
        uint8_t ch = DL_UART_Main_receiveData(UART_1_INST);

        if ((ch == (uint8_t)'\r') || (ch == (uint8_t)'\n')) {
            if (*lineLength > 0U) {
                line[*lineLength] = '\0';
                task2_stepper_test_execute(line, angleLoopActive, txBuffer,
                                           txLength, txSent);
            }
            *lineLength = 0U;
        } else if ((ch >= (uint8_t)' ') &&
                   (*lineLength < (TASK2_STEPPER_TEST_LINE_SIZE - 1U))) {
            line[*lineLength] = (char)ch;
            (*lineLength)++;
        } else {
            *lineLength = 0U;
            task2_stepper_test_queue_text(txBuffer, txLength, txSent,
                                          "ERR\r\n");
        }
    }
}

void task2_stepper_test_run(void)
{
    uint32_t nowMs;
    uint32_t positionLastUpdateMs;
    uint32_t serviceLastUpdateMs;
    bool angleLoopActive = false;
    char line[TASK2_STEPPER_TEST_LINE_SIZE];
    uint8_t lineLength = 0U;
    char txBuffer[TASK2_UART1_MONITOR_BUFFER_SIZE];
    uint8_t txLength = 0U;
    uint8_t txSent = 0U;

    stepper_init();
    task2_uart1_monitor_init();
    DL_UART_Main_clearInterruptStatus(UART_1_INST, DL_UART_MAIN_INTERRUPT_RX);

    nowMs = delay_get_ms();
    positionLastUpdateMs = nowMs;
    serviceLastUpdateMs = nowMs;
    task2_stepper_test_queue_text(txBuffer, &txLength, &txSent,
                                  "STEPPER TEST READY\r\n");

    while (1) {
        task2_stepper_test_process_rx(line, &lineLength, &angleLoopActive,
                                      txBuffer, &txLength, &txSent);
        nowMs = delay_get_ms();

        if (angleLoopActive &&
            ((uint32_t)(nowMs - positionLastUpdateMs) >=
             TASK2_STEPPER_TEST_POSITION_PERIOD_MS)) {
            stepper_update_beam_position_loop();
            positionLastUpdateMs = nowMs;
        }
        if (angleLoopActive &&
            ((uint32_t)(nowMs - serviceLastUpdateMs) >=
             TASK2_STEPPER_TEST_SERVICE_PERIOD_MS)) {
            stepper_service((float)(nowMs - serviceLastUpdateMs) / 1000.0f);
            serviceLastUpdateMs = nowMs;
        }

        task2_uart1_monitor_send_pending(txBuffer, txLength, &txSent);
    }
}

void task2_ball_balance_run(void)
{
    uint32_t nowMs;
    uint32_t pidLastUpdateMs;
    uint32_t serviceLastUpdateMs;
    uint32_t monitorLastUpdateMs;
    uart_cmd_vision_sample_t visionSample = {0};
    char monitorBuffer[TASK2_UART1_MONITOR_BUFFER_SIZE];
    uint8_t monitorLength = 0U;
    uint8_t monitorSent = 0U;

    stepper_init();
    stepper_enable(STEPPER_BEAM_PID_ENABLE_ON_START != 0U);
    uart_cmd_init();
    task2_uart1_monitor_init();

    nowMs = delay_get_ms();
    pidLastUpdateMs = nowMs;
    serviceLastUpdateMs = nowMs;
    monitorLastUpdateMs = nowMs;

    while (1) {
        uart_cmd_process();
        nowMs = delay_get_ms();

        if ((uint32_t)(nowMs - pidLastUpdateMs) >=
            TASK2_BALL_PID_PERIOD_MS) {
            bool visionValid = uart_cmd_get_vision_sample(&visionSample);
            float pidDtSeconds =
                (float)(nowMs - pidLastUpdateMs) / 1000.0f;

            stepper_update_beam_pid((float)visionSample.positionX10 * 0.1f,
                                    (float)visionSample.velocityX10 * 0.1f,
                                    visionValid, pidDtSeconds);
            pidLastUpdateMs = nowMs;
        }

        if ((uint32_t)(nowMs - serviceLastUpdateMs) >=
            TASK2_BALL_SERVICE_PERIOD_MS) {
            float serviceDtSeconds =
                (float)(nowMs - serviceLastUpdateMs) / 1000.0f;

            stepper_service(serviceDtSeconds);
            serviceLastUpdateMs = nowMs;
        }

        task2_uart1_monitor_send_pending(monitorBuffer, monitorLength,
                                         &monitorSent);
        if ((monitorSent >= monitorLength) &&
            ((uint32_t)(nowMs - monitorLastUpdateMs) >=
             TASK2_UART1_MONITOR_PERIOD_MS)) {
            if (task2_uart1_monitor_build_line(monitorBuffer, &monitorLength,
                                               &visionSample)) {
                monitorSent = 0U;
            }
            monitorLastUpdateMs = nowMs;
        }
    }
}

void task2_run(void)
{
    uint8_t grayRaw;
    uint8_t activeCount;
    uint32_t taskStartMs;
    uint32_t nowMs;
    uint32_t elapsedMs = 0U;
    bool timerStopped = false;
    uint32_t stopDetectMs = 0U;
    int32_t stopStartLeftCount = 0;
    int32_t stopStartRightCount = 0;
    bool stopArmed;
    task2_line_state_t lineState = TASK2_LINE_STATE_RUNNING;
    bool oledOk;

    gray_serial_init();
    task2_discard_startup_gray_samples();
    encoder_init();
    speed_pid_init();
    line_track_init();
    task2_apply_race_line_params();
    bluetooth_init();
    uart_cmd_init();
    oledOk = oled_init();
    taskStartMs = delay_get_ms();

    while (1) {
        bluetooth_process();
        uart_cmd_process();

        nowMs = delay_get_ms();
        grayRaw = gray_serial_read();
        activeCount = task2_count_active_gray_sensors(grayRaw);
        stopArmed = ((uint32_t)(nowMs - taskStartMs) >=
                     TASK2_STOP_ENABLE_DELAY_MS);

        if ((lineState == TASK2_LINE_STATE_RUNNING) && stopArmed &&
            task2_should_exit_line_track(activeCount)) {
            lineState = TASK2_LINE_STATE_APPROACHING_STOP;
            stopDetectMs = nowMs;
            stopStartLeftCount = encoder_get_left_count();
            stopStartRightCount = encoder_get_right_count();
        }

        if (lineState == TASK2_LINE_STATE_APPROACHING_STOP) {
            if ((uint32_t)(nowMs - stopDetectMs) >=
                TASK2_STOP_APPROACH_SLOW_DELAY_MS) {
                line_track_set_base_speed(TASK2_STOP_APPROACH_SPEED_MM_S);
            }
            line_track_update_with_raw_hold_on_lost(grayRaw);

            if (task2_get_traveled_distance_mm(stopStartLeftCount,
                                               stopStartRightCount) >=
                TASK2_STOP_APPROACH_DISTANCE_MM) {
                lineState = TASK2_LINE_STATE_STOPPED;
                speed_pid_stop();
            }
        } else if (lineState == TASK2_LINE_STATE_STOPPED) {
            speed_pid_stop();
        } else {
            line_track_update_with_raw_hold_on_lost(grayRaw);
        }

        speed_pid_control_update();
        if (!timerStopped) {
            elapsedMs = nowMs;
            if ((lineState == TASK2_LINE_STATE_STOPPED) && task2_car_speed_is_zero()) {
                timerStopped = true;
            }
        }
        task2_oled_update(oledOk, grayRaw, activeCount,
                          stopArmed, lineState, elapsedMs);
        delay_ms(SPEED_PID_CONTROL_PERIOD_MS);
    }
}
