#include <stdbool.h>
#include <stdint.h>
#include <limits.h>

#include "delay.h"
#include "bluetooth.h"
#include "encoder.h"
#include "encoder_pwm_angle.h"
#include "gray_serial.h"
#include "line_track.h"
#include "oled.h"
#include "pid.h"
#include "stepper.h"
#include "task2.h"
#include "ti_msp_dl_config.h"
#include "uart_cmd.h"

#define TASK2_OLED_REFRESH_PERIOD_MS      (200U)
#define TASK2_STOP_DETECT_ENABLE_DISTANCE_MM (6000)
#define TASK2_GRAY_STARTUP_DISCARD_COUNT  (20U)
#define TASK2_GRAY_STARTUP_DISCARD_MS     (2U)
#define TASK2_BALL_PID_PERIOD_MS           (10U)
#define TASK2_BALL_SERVICE_PERIOD_MS       (5U)
#define TASK2_UART1_MONITOR_ENABLE          (1U)
#define TASK2_UART1_MONITOR_PERIOD_MS       (100U)
#define TASK2_UART1_MONITOR_BUFFER_SIZE     (220U)
#define TASK2_UART1_COMMAND_LINE_SIZE       (64U)
/*
 * Debug mode for checking the vision UART link:
 * UART0 RX bytes are mirrored by uart_cmd.c to UART1 TX.  Keep task2's own
 * UART1 BOOT / VOFA / PID-tuning text silent so the PC receives only the raw
 * data sent by the vision module.
 */
#define TASK2_UART1_VISION_RAW_ECHO_ONLY    (0U)
#define TASK2_STEPPER_TEST_LINE_SIZE         (64U)
#define TASK2_STEPPER_TEST_POSITION_PERIOD_MS (10U)
#define TASK2_STEPPER_TEST_SERVICE_PERIOD_MS  (5U)
#define TASK2_DIR_TEST_MAX_STEPS              (50)
#define TASK2_DIR_TEST_MIN_FREQ_HZ            (120)
#define TASK2_DIR_TEST_MAX_FREQ_HZ            (300)
#define TASK2_DIR_TEST_SETTLE_MS              (120U)
#define TASK2_BALL_HOME_TOLERANCE_DEG_X10     (10)
#define TASK2_BALL_HOME_CONFIRM_MS            (200U)
#define TASK2_BALL_HOME_MIN_FREQ_HZ           (120.0f)
#define TASK2_BALL_HOME_MAX_FREQ_HZ           (200.0f)
#define TASK2_BALL_HOME_KP_HZ_PER_DEG_X10     (1.0f)
/*
 * Default: positive relative angle needs negative STEP frequency to return to
 * zero. If the real mechanism moves away from zero during startup homing, set
 * this macro to 1.
 */
#define TASK2_BALL_HOME_OUTPUT_INVERT         (0U)

typedef enum {
    TASK2_LINE_STATE_RUNNING = 0,
    TASK2_LINE_STATE_STOPPED
} task2_line_state_t;

typedef enum {
    TASK2_BALL_HOME_STATE_HOMING = 0,
    TASK2_BALL_HOME_STATE_READY = 1,
    TASK2_BALL_HOME_STATE_BLOCKED = 2,
    TASK2_BALL_HOME_STATE_NO_ENCODER = 3,
    TASK2_BALL_HOME_STATE_DISABLED = 4
} task2_ball_home_state_t;

typedef enum {
    TASK2_DIR_TEST_IDLE = 0,
    TASK2_DIR_TEST_WAIT_MOTION_DONE,
    TASK2_DIR_TEST_WAIT_SETTLE
} task2_direction_test_state_t;

/*
 * Task2 使用竞速参数。
 * line_track.h 中的默认宏作为稳定参数保留给其他任务使用。
 * 这里先让竞速参数与当前稳定参数一致，后续可单独调高。
 */
#define TASK2_RACE_LINE_BASE_SPEED_MM_S       (350)
#define TASK2_RACE_SMALL_TURN_PERCENT          (90)
#define TASK2_RACE_LARGE_TURN_PERCENT          (60)
 
static void task2_apply_race_line_params(void)
{
    line_track_set_base_speed(TASK2_RACE_LINE_BASE_SPEED_MM_S);
    (void)line_track_set_turn_ratios(
        TASK2_RACE_SMALL_TURN_PERCENT,
        TASK2_RACE_LARGE_TURN_PERCENT);
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
    uint8_t activeCount = 0U;

    for (uint8_t i = 0U; i < 4U; i++) {
        uint8_t level = (uint8_t)((raw >> i) & 0x01U);

        if (level == LINE_TRACK_ACTIVE_LEVEL) {
            activeCount++;
        }
    }

    return activeCount;
}

static bool task2_stop_marker_detected(uint8_t raw)
{
    uint8_t o2Level = (uint8_t)((raw >> 1U) & 0x01U);
    uint8_t o3Level = (uint8_t)((raw >> 2U) & 0x01U);

    return (o2Level == LINE_TRACK_ACTIVE_LEVEL) &&
           (o3Level == LINE_TRACK_ACTIVE_LEVEL);
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

static void task2_oled_print_line_header(uint8_t page, const char *text)
{
    oled_clear_line(page);
    oled_print_string(text);
}

static void task2_oled_update(bool oledOk,
                              uint8_t grayRaw,
                              uint8_t activeCount,
                              task2_line_state_t lineState,
                              bool stopDetectEnabled,
                              int32_t traveledDistanceMm,
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
    } else if (!status.lineDetected) {
        oled_print_string(" LOST");
    } else if (stopDetectEnabled) {
        oled_print_string(" ARM");
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

    task2_oled_print_line_header(4U, "Small:");
    oled_print_int(line_track_get_small_turn_percent());
    oled_print_string("% Big:");
    oled_print_int(line_track_get_large_turn_percent());
    oled_print_string("%");

    task2_oled_print_line_header(5U, "Dist:");
    oled_print_int(traveledDistanceMm);
    oled_print_string("/");
    oled_print_int(TASK2_STOP_DETECT_ENABLE_DISTANCE_MM);

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

static int32_t task2_float_to_x1000(float value)
{
    return (value >= 0.0f) ? (int32_t)(value * 1000.0f + 0.5f) :
                             (int32_t)(value * 1000.0f - 0.5f);
}

static int32_t task2_float_to_x10(float value)
{
    return (value >= 0.0f) ? (int32_t)(value * 10.0f + 0.5f) :
                             (int32_t)(value * 10.0f - 0.5f);
}

static void task2_uart1_monitor_init(void)
{
#if (TASK2_UART1_MONITOR_ENABLE != 0U)
    /* UART1 RX interrupt stays inactive; foreground code may poll RX FIFO. */
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

static void task2_stepper_test_queue_text(char *buffer, uint8_t *length,
                                          uint8_t *sent, const char *text);

static void task2_uart1_monitor_flush_until_sent(const char *buffer,
                                                 uint8_t length,
                                                 uint8_t *sent,
                                                 uint32_t timeoutMs)
{
    uint32_t startMs = delay_get_ms();

    while ((*sent < length) &&
           ((uint32_t)(delay_get_ms() - startMs) < timeoutMs)) {
        task2_uart1_monitor_send_pending(buffer, length, sent);
    }
}

static void task2_uart1_monitor_queue_text_and_flush(
    char *buffer,
    uint8_t *length,
    uint8_t *sent,
    const char *text,
    uint32_t timeoutMs)
{
    task2_stepper_test_queue_text(buffer, length, sent, text);
    task2_uart1_monitor_flush_until_sent(buffer, *length, sent, timeoutMs);
}

static bool task2_uart1_monitor_build_line(char *buffer, uint8_t *length,
                                           const uart_cmd_vision_sample_t *sample,
                                           const encoder_pwm_angle_sample_t *angleSample,
                                           task2_ball_home_state_t homeState)
{
    uint32_t flags = 0U;
    float kp;
    float ki;
    float kd;
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
    if (angleSample->safeLimitActive) {
        flags |= 1U << 4;
    }
    if (angleSample->fresh) {
        flags |= 1U << 5;
    }
    if (homeState == TASK2_BALL_HOME_STATE_READY) {
        flags |= 1U << 6;
    }
    if (stepper_is_moving()) {
        flags |= 1U << 7;
    }
    if (stepper_is_emergency_inhibited()) {
        flags |= 1U << 8;
    }

    stepper_get_beam_pid_gains(&kp, &ki, &kd);
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
            buffer, length, task2_float_to_x10(stepper_get_beam_target_deg())) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(
            buffer, length, (int32_t)stepper_get_speed_target_hz()) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(
            buffer, length, (int32_t)stepper_get_current_speed_hz()) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(
            buffer, length, angleSample->relativeAngleDegX10) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(
            buffer, length, (int32_t)flags) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(
            buffer, length, task2_float_to_x1000(kp)) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(
            buffer, length, task2_float_to_x1000(ki)) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(
            buffer, length, task2_float_to_x1000(kd)) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(
            buffer, length, (int32_t)homeState) ||
        !task2_uart1_monitor_append_char(buffer, length, '\n')) {
        *length = 0U;
        return false;
    }

    return true;
}

static bool task2_encoder_calibration_build_line(
    char *buffer,
    uint8_t *length,
    const encoder_pwm_angle_sample_t *angleSample)
{
    uint32_t flags = 0U;
    static const char prefix[] = "samples:";

    if (angleSample->valid) {
        flags |= 1U << 0;
    }
    if (angleSample->fresh) {
        flags |= 1U << 1;
    }
    if (angleSample->safeLimitActive) {
        flags |= 1U << 2;
    }

    *length = 0U;
    for (uint8_t i = 0U; i < (sizeof(prefix) - 1U); i++) {
        if (!task2_uart1_monitor_append_char(buffer, length, prefix[i])) {
            return false;
        }
    }

    if (!task2_uart1_monitor_append_int32(
            buffer, length, angleSample->angleDegX10) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(
            buffer, length, angleSample->relativeAngleDegX10) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(buffer, length, (int32_t)flags) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(
            buffer, length, (int32_t)angleSample->periodTicks) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(
            buffer, length, (int32_t)angleSample->highTicks) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(
            buffer, length, (int32_t)angleSample->captureCount) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(
            buffer, length, (int32_t)angleSample->timeoutCount) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(
            buffer, length, BEAM_ENCODER_ZERO_RAW_DEG_X10) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(
            buffer, length, BEAM_ENCODER_MAX_SAFE_REL_DEG_X10) ||
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

static bool task2_stepper_test_read_float(const char **text, float *value)
{
    const char *p = task2_stepper_test_skip_separators(*text);
    float result = 0.0f;
    float scale = 0.1f;
    bool negative = false;
    bool hasDigit = false;

    if (*p == '-') {
        negative = true;
        p++;
    } else if (*p == '+') {
        p++;
    }

    while ((*p >= '0') && (*p <= '9')) {
        hasDigit = true;
        result = result * 10.0f + (float)(*p - '0');
        if (result > 1000000.0f) {
            return false;
        }
        p++;
    }

    if (*p == '.') {
        p++;
        while ((*p >= '0') && (*p <= '9')) {
            hasDigit = true;
            result += (float)(*p - '0') * scale;
            scale *= 0.1f;
            p++;
        }
    }

    if (!hasDigit) {
        return false;
    }

    *value = negative ? -result : result;
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

static bool task2_uart1_monitor_append_fixed(char *buffer, uint8_t *length,
                                             int32_t scaled, uint16_t scale)
{
    uint32_t magnitude;
    uint32_t fractional;
    uint16_t divisor = scale / 10U;

    if (scale == 0U) {
        return false;
    }

    if (scaled < 0) {
        if (!task2_uart1_monitor_append_char(buffer, length, '-')) {
            return false;
        }
        magnitude = (uint32_t)(-(scaled + 1)) + 1U;
    } else {
        magnitude = (uint32_t)scaled;
    }

    if (!task2_uart1_monitor_append_int32(
            buffer, length, (int32_t)(magnitude / scale)) ||
        !task2_uart1_monitor_append_char(buffer, length, '.')) {
        return false;
    }

    fractional = magnitude % scale;
    while (divisor > 0U) {
        if (!task2_uart1_monitor_append_char(
                buffer, length, (char)('0' + (fractional / divisor)))) {
            return false;
        }
        fractional %= divisor;
        divisor /= 10U;
    }

    return true;
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

static void task2_uart1_force_queue_text(char *buffer, uint8_t *length,
                                         uint8_t *sent, const char *text)
{
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

static void task2_ball_pid_queue_status(char *buffer, uint8_t *length,
                                        uint8_t *sent)
{
    float kp;
    float ki;
    float kd;

    stepper_get_beam_pid_gains(&kp, &ki, &kd);
    *length = 0U;
    if (!task2_uart1_append_string(buffer, length, "PID,TGT_MM:") ||
        !task2_uart1_monitor_append_fixed(
            buffer, length,
            task2_float_to_x10(stepper_get_beam_pid_target_mm()), 10U) ||
        !task2_uart1_append_string(buffer, length, ",KP_DEG_PER_MM:") ||
        !task2_uart1_monitor_append_fixed(
            buffer, length, task2_float_to_x1000(kp), 1000U) ||
        !task2_uart1_append_string(buffer, length, ",KI_DEG_PER_MM_S:") ||
        !task2_uart1_monitor_append_fixed(
            buffer, length, task2_float_to_x1000(ki), 1000U) ||
        !task2_uart1_append_string(buffer, length, ",KD_DEG_S_PER_MM:") ||
        !task2_uart1_monitor_append_fixed(
            buffer, length, task2_float_to_x1000(kd), 1000U) ||
        !task2_uart1_append_string(buffer, length, "\r\n")) {
        *length = 0U;
    }
    *sent = 0U;
}

static void task2_ball_pid_execute_command(char *line,
                                           char *txBuffer,
                                           uint8_t *txLength,
                                           uint8_t *txSent)
{
    const char *p = line;
    const char *afterCommand;
    char word[8];
    char subword[8];
    float value0;
    float value1;
    float value2;
    float value3;
    float kp;
    float ki;
    float kd;

    if (!task2_stepper_test_read_word(&p, word, sizeof(word))) {
        return;
    }
    afterCommand = p;

    if (task2_stepper_test_word_equals(word, "HELP")) {
        task2_uart1_force_queue_text(
            txBuffer, txLength, txSent,
            "CMD: GETPID | SET target_mm kp ki kd | ALL target_mm kp ki kd | PID kp ki kd | PID KP/KI/KD value | TARGET mm\r\n");
        return;
    }

    if (task2_stepper_test_word_equals(word, "GETPID") ||
        task2_stepper_test_word_equals(word, "PID?")) {
        task2_ball_pid_queue_status(txBuffer, txLength, txSent);
        return;
    }

    if (task2_stepper_test_word_equals(word, "TARGET") ||
        task2_stepper_test_word_equals(word, "TGT")) {
        if (task2_stepper_test_read_float(&p, &value0) &&
            task2_stepper_test_has_no_more(p) &&
            stepper_set_beam_pid_target_mm(value0)) {
            task2_ball_pid_queue_status(txBuffer, txLength, txSent);
        } else {
            task2_uart1_force_queue_text(txBuffer, txLength, txSent,
                                         "ERR TARGET\r\n");
        }
        return;
    }

    if (task2_stepper_test_word_equals(word, "SET") ||
        task2_stepper_test_word_equals(word, "ALL") ||
        task2_stepper_test_word_equals(word, "PIDALL") ||
        task2_stepper_test_word_equals(word, "PARAM")) {
        if (task2_stepper_test_read_float(&p, &value0) &&
            task2_stepper_test_read_float(&p, &value1) &&
            task2_stepper_test_read_float(&p, &value2) &&
            task2_stepper_test_read_float(&p, &value3) &&
            task2_stepper_test_has_no_more(p) &&
            stepper_set_beam_pid_gains(value1, value2, value3) &&
            stepper_set_beam_pid_target_mm(value0)) {
            task2_ball_pid_queue_status(txBuffer, txLength, txSent);
        } else {
            task2_uart1_force_queue_text(txBuffer, txLength, txSent,
                                         "ERR SET\r\n");
        }
        return;
    }

    if (task2_stepper_test_word_equals(word, "PID")) {
        stepper_get_beam_pid_gains(&kp, &ki, &kd);
        if (task2_stepper_test_read_word(&p, subword, sizeof(subword))) {
            if (task2_stepper_test_word_equals(subword, "KP") ||
                task2_stepper_test_word_equals(subword, "KI") ||
                task2_stepper_test_word_equals(subword, "KD")) {
                if (!task2_stepper_test_read_float(&p, &value0) ||
                    !task2_stepper_test_has_no_more(p)) {
                    task2_uart1_force_queue_text(txBuffer, txLength, txSent,
                                                 "ERR PID\r\n");
                    return;
                }
                if (task2_stepper_test_word_equals(subword, "KP")) {
                    kp = value0;
                } else if (task2_stepper_test_word_equals(subword, "KI")) {
                    ki = value0;
                } else {
                    kd = value0;
                }
                if (stepper_set_beam_pid_gains(kp, ki, kd)) {
                    task2_ball_pid_queue_status(txBuffer, txLength, txSent);
                } else {
                    task2_uart1_force_queue_text(txBuffer, txLength, txSent,
                                                 "ERR PID\r\n");
                }
                return;
            }
        }

        p = afterCommand;
        if (task2_stepper_test_read_float(&p, &value0) &&
            task2_stepper_test_read_float(&p, &value1) &&
            task2_stepper_test_read_float(&p, &value2) &&
            task2_stepper_test_has_no_more(p) &&
            stepper_set_beam_pid_gains(value0, value1, value2)) {
            task2_ball_pid_queue_status(txBuffer, txLength, txSent);
        } else {
            task2_uart1_force_queue_text(txBuffer, txLength, txSent,
                                         "ERR PID\r\n");
        }
        return;
    }

    task2_uart1_force_queue_text(txBuffer, txLength, txSent, "ERR CMD\r\n");
}

static void task2_ball_pid_process_uart1_rx(char *line, uint8_t *lineLength,
                                            char *txBuffer, uint8_t *txLength,
                                            uint8_t *txSent)
{
    while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)) {
        uint8_t ch = DL_UART_Main_receiveData(UART_1_INST);

        if ((ch == (uint8_t)'\r') || (ch == (uint8_t)'\n')) {
            if (*lineLength > 0U) {
                line[*lineLength] = '\0';
                task2_ball_pid_execute_command(line, txBuffer, txLength,
                                               txSent);
            }
            *lineLength = 0U;
        } else if ((ch >= (uint8_t)' ') &&
                   (*lineLength < (TASK2_UART1_COMMAND_LINE_SIZE - 1U))) {
            line[*lineLength] = (char)ch;
            (*lineLength)++;
        } else {
            *lineLength = 0U;
            task2_uart1_force_queue_text(txBuffer, txLength, txSent,
                                         "ERR LINE\r\n");
        }
    }
}

static bool task2_ball_home_angle_in_start_range(
    const encoder_pwm_angle_sample_t *angleSample)
{
    return (angleSample->relativeAngleDegX10 <=
            BEAM_ENCODER_POS_LIMIT_REL_DEG_X10) &&
           (angleSample->relativeAngleDegX10 >=
            BEAM_ENCODER_NEG_LIMIT_REL_DEG_X10);
}

static float task2_ball_home_speed_from_angle(int32_t relativeAngleDegX10)
{
    float speedHz;
    float magnitudeHz;

    if (task2_abs_i32(relativeAngleDegX10) <=
        TASK2_BALL_HOME_TOLERANCE_DEG_X10) {
        return 0.0f;
    }

    magnitudeHz = (float)task2_abs_i32(relativeAngleDegX10) *
                  TASK2_BALL_HOME_KP_HZ_PER_DEG_X10;
    if (magnitudeHz < TASK2_BALL_HOME_MIN_FREQ_HZ) {
        magnitudeHz = TASK2_BALL_HOME_MIN_FREQ_HZ;
    } else if (magnitudeHz > TASK2_BALL_HOME_MAX_FREQ_HZ) {
        magnitudeHz = TASK2_BALL_HOME_MAX_FREQ_HZ;
    }

    speedHz = (relativeAngleDegX10 > 0) ? -magnitudeHz : magnitudeHz;
#if (TASK2_BALL_HOME_OUTPUT_INVERT != 0U)
    speedHz = -speedHz;
#endif
    return speedHz;
}

static task2_ball_home_state_t task2_ball_home_update(
    const encoder_pwm_angle_sample_t *angleSample,
    uint32_t nowMs,
    bool *homeComplete,
    uint32_t *homeStableStartMs)
{
    if (*homeComplete) {
        return TASK2_BALL_HOME_STATE_READY;
    }

    if (!angleSample->fresh) {
        *homeStableStartMs = 0U;
        stepper_reset_beam_pid();
        stepper_enable(false);
        return TASK2_BALL_HOME_STATE_NO_ENCODER;
    }

    if (!task2_ball_home_angle_in_start_range(angleSample)) {
        *homeStableStartMs = 0U;
        stepper_reset_beam_pid();
        stepper_enable(false);
        return TASK2_BALL_HOME_STATE_BLOCKED;
    }

#if (STEPPER_BEAM_PID_ENABLE_ON_START != 0U)
    if (!stepper_is_enabled()) {
        stepper_enable(true);
    }
#else
    *homeStableStartMs = 0U;
    stepper_reset_beam_pid();
    stepper_enable(false);
    return TASK2_BALL_HOME_STATE_DISABLED;
#endif

    stepper_sync_position_from_beam_angle_x10(
        angleSample->relativeAngleDegX10);

    if (task2_abs_i32(angleSample->relativeAngleDegX10) <=
        TASK2_BALL_HOME_TOLERANCE_DEG_X10) {
        stepper_reset_beam_pid();
        stepper_stop_smooth();
        if (*homeStableStartMs == 0U) {
            *homeStableStartMs = nowMs;
        } else if ((uint32_t)(nowMs - *homeStableStartMs) >=
                   TASK2_BALL_HOME_CONFIRM_MS) {
            stepper_stop();
            stepper_sync_position_from_beam_angle_x10(0);
            *homeComplete = true;
            return TASK2_BALL_HOME_STATE_READY;
        }
        return TASK2_BALL_HOME_STATE_HOMING;
    }

    *homeStableStartMs = 0U;
    stepper_reset_beam_pid();
    stepper_set_speed_target(
        task2_ball_home_speed_from_angle(angleSample->relativeAngleDegX10));
    return TASK2_BALL_HOME_STATE_HOMING;
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

static void task2_direction_test_queue_sample(
    char *buffer,
    uint8_t *length,
    uint8_t *sent,
    const encoder_pwm_angle_sample_t *sample)
{
    if (*sent < *length) {
        return;
    }

    *length = 0U;
    if (!task2_uart1_append_string(buffer, length, "ENC RAW:") ||
        !task2_uart1_monitor_append_int32(buffer, length,
                                          sample->angleDegX10) ||
        !task2_uart1_append_string(buffer, length, " REL:") ||
        !task2_uart1_monitor_append_int32(buffer, length,
                                          sample->relativeAngleDegX10) ||
        !task2_uart1_append_string(buffer, length, " FRESH:") ||
        !task2_uart1_monitor_append_int32(buffer, length,
                                          sample->fresh ? 1 : 0) ||
        !task2_uart1_append_string(buffer, length, " VALID:") ||
        !task2_uart1_monitor_append_int32(buffer, length,
                                          sample->valid ? 1 : 0) ||
        !task2_uart1_append_string(buffer, length, " SAFE:") ||
        !task2_uart1_monitor_append_int32(buffer, length,
                                          sample->safeLimitActive ? 0 : 1) ||
        !task2_uart1_append_string(buffer, length, "\r\n")) {
        *length = 0U;
    }
    *sent = 0U;
}

static void task2_direction_test_queue_result(
    char *buffer,
    uint8_t *length,
    uint8_t *sent,
    const encoder_pwm_angle_sample_t *before,
    const encoder_pwm_angle_sample_t *after,
    int32_t commandSteps)
{
    int32_t deltaRel = after->relativeAngleDegX10 -
                       before->relativeAngleDegX10;
    const char *resultText = (deltaRel > 0) ? "REL_POSITIVE" :
                             ((deltaRel < 0) ? "REL_NEGATIVE" : "NO_CHANGE");

    if (*sent < *length) {
        return;
    }

    *length = 0U;
    if (!task2_uart1_append_string(buffer, length, "DIRTEST STEP:") ||
        !task2_uart1_monitor_append_int32(buffer, length, commandSteps) ||
        !task2_uart1_append_string(buffer, length, " BEFORE_RAW:") ||
        !task2_uart1_monitor_append_int32(buffer, length,
                                          before->angleDegX10) ||
        !task2_uart1_append_string(buffer, length, " BEFORE_REL:") ||
        !task2_uart1_monitor_append_int32(buffer, length,
                                          before->relativeAngleDegX10) ||
        !task2_uart1_append_string(buffer, length, " AFTER_RAW:") ||
        !task2_uart1_monitor_append_int32(buffer, length,
                                          after->angleDegX10) ||
        !task2_uart1_append_string(buffer, length, " AFTER_REL:") ||
        !task2_uart1_monitor_append_int32(buffer, length,
                                          after->relativeAngleDegX10) ||
        !task2_uart1_append_string(buffer, length, " DELTA_REL:") ||
        !task2_uart1_monitor_append_int32(buffer, length, deltaRel) ||
        !task2_uart1_append_string(buffer, length, " RESULT:") ||
        !task2_uart1_append_string(buffer, length, resultText) ||
        !task2_uart1_append_string(buffer, length, "\r\n")) {
        *length = 0U;
    }
    *sent = 0U;
}

static bool task2_direction_test_start(
    int32_t commandSteps,
    int32_t frequencyHz,
    task2_direction_test_state_t *state,
    encoder_pwm_angle_sample_t *before,
    char *txBuffer,
    uint8_t *txLength,
    uint8_t *txSent)
{
    int32_t magnitude;

    if ((*state != TASK2_DIR_TEST_IDLE) || stepper_is_busy()) {
        task2_stepper_test_queue_text(txBuffer, txLength, txSent,
                                      "ERR BUSY\r\n");
        return false;
    }
    if (commandSteps == INT32_MIN) {
        task2_stepper_test_queue_text(txBuffer, txLength, txSent,
                                      "ERR RANGE\r\n");
        return false;
    }
    magnitude = (commandSteps >= 0) ? commandSteps : -commandSteps;
    if ((magnitude <= 0) || (magnitude > TASK2_DIR_TEST_MAX_STEPS) ||
        (frequencyHz < TASK2_DIR_TEST_MIN_FREQ_HZ) ||
        (frequencyHz > TASK2_DIR_TEST_MAX_FREQ_HZ)) {
        task2_stepper_test_queue_text(txBuffer, txLength, txSent,
                                      "ERR RANGE\r\n");
        return false;
    }
    if (!encoder_pwm_angle_get_sample(before) || !before->fresh) {
        task2_stepper_test_queue_text(txBuffer, txLength, txSent,
                                      "ERR ENCODER\r\n");
        return false;
    }

    stepper_enable(true);
    if (!stepper_start(commandSteps, (uint32_t)frequencyHz)) {
        stepper_enable(false);
        task2_stepper_test_queue_text(txBuffer, txLength, txSent,
                                      "ERR START\r\n");
        return false;
    }

    *state = TASK2_DIR_TEST_WAIT_MOTION_DONE;
    task2_stepper_test_queue_text(txBuffer, txLength, txSent, "START\r\n");
    return true;
}

static void task2_direction_test_execute_command(
    char *line,
    task2_direction_test_state_t *state,
    encoder_pwm_angle_sample_t *before,
    int32_t *commandSteps,
    char *txBuffer,
    uint8_t *txLength,
    uint8_t *txSent)
{
    const char *p = line;
    char word[8];
    int32_t value0;
    int32_t value1;
    bool boolValue;
    encoder_pwm_angle_sample_t sample;

    if (!task2_stepper_test_read_word(&p, word, sizeof(word))) {
        return;
    }

    if (task2_stepper_test_word_equals(word, "HELP")) {
        task2_stepper_test_queue_text(
            txBuffer, txLength, txSent,
            "CMD: SAMPLE STATUS EN 0/1 TESTP steps hz TESTN steps hz STOP\r\n");
    } else if (task2_stepper_test_word_equals(word, "SAMPLE")) {
        (void)encoder_pwm_angle_get_sample(&sample);
        task2_direction_test_queue_sample(txBuffer, txLength, txSent, &sample);
    } else if (task2_stepper_test_word_equals(word, "STATUS") ||
               task2_stepper_test_word_equals(word, "GET")) {
        (void)encoder_pwm_angle_get_sample(&sample);
        task2_stepper_test_queue_status(txBuffer, txLength, txSent);
        if (*txLength == 0U) {
            task2_direction_test_queue_sample(txBuffer, txLength, txSent,
                                              &sample);
        }
    } else if (task2_stepper_test_word_equals(word, "EN") &&
               task2_stepper_test_read_bool(&p, &boolValue) &&
               task2_stepper_test_has_no_more(p)) {
        if (*state == TASK2_DIR_TEST_IDLE) {
            stepper_enable(boolValue);
            task2_stepper_test_queue_status(txBuffer, txLength, txSent);
        } else {
            task2_stepper_test_queue_text(txBuffer, txLength, txSent,
                                          "ERR BUSY\r\n");
        }
    } else if ((task2_stepper_test_word_equals(word, "TESTP") ||
                task2_stepper_test_word_equals(word, "TESTN")) &&
               task2_stepper_test_read_int32(&p, &value0) &&
               task2_stepper_test_read_int32(&p, &value1) &&
               task2_stepper_test_has_no_more(p)) {
        if (value0 == INT32_MIN) {
            task2_stepper_test_queue_text(txBuffer, txLength, txSent,
                                          "ERR RANGE\r\n");
            return;
        }
        if (value0 < 0) {
            value0 = -value0;
        }
        if (task2_stepper_test_word_equals(word, "TESTN")) {
            value0 = -value0;
        }
        if (task2_direction_test_start(value0, value1, state, before,
                                       txBuffer, txLength, txSent)) {
            *commandSteps = value0;
        }
    } else if (task2_stepper_test_word_equals(word, "STOP") &&
               task2_stepper_test_has_no_more(p)) {
        stepper_stop();
        stepper_enable(false);
        *state = TASK2_DIR_TEST_IDLE;
        task2_stepper_test_queue_text(txBuffer, txLength, txSent, "OK\r\n");
    } else {
        task2_stepper_test_queue_text(txBuffer, txLength, txSent,
                                      "ERR CMD\r\n");
    }
}

static void task2_direction_test_process_rx(
    char *line,
    uint8_t *lineLength,
    task2_direction_test_state_t *state,
    encoder_pwm_angle_sample_t *before,
    int32_t *commandSteps,
    char *txBuffer,
    uint8_t *txLength,
    uint8_t *txSent)
{
    while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)) {
        uint8_t ch = DL_UART_Main_receiveData(UART_1_INST);

        if ((ch == (uint8_t)'\r') || (ch == (uint8_t)'\n')) {
            if (*lineLength > 0U) {
                line[*lineLength] = '\0';
                task2_direction_test_execute_command(
                    line, state, before, commandSteps, txBuffer, txLength,
                    txSent);
            }
            *lineLength = 0U;
        } else if ((ch >= (uint8_t)' ') &&
                   (*lineLength < (TASK2_STEPPER_TEST_LINE_SIZE - 1U))) {
            line[*lineLength] = (char)ch;
            (*lineLength)++;
        } else {
            *lineLength = 0U;
            task2_stepper_test_queue_text(txBuffer, txLength, txSent,
                                          "ERR LINE\r\n");
        }
    }
}

void task2_stepper_direction_test_run(void)
{
    uint32_t nowMs;
    uint32_t settleStartMs = 0U;
    task2_direction_test_state_t state = TASK2_DIR_TEST_IDLE;
    encoder_pwm_angle_sample_t before = {0};
    encoder_pwm_angle_sample_t after = {0};
    int32_t commandSteps = 0;
    char line[TASK2_STEPPER_TEST_LINE_SIZE];
    uint8_t lineLength = 0U;
    char txBuffer[TASK2_UART1_MONITOR_BUFFER_SIZE];
    uint8_t txLength = 0U;
    uint8_t txSent = 0U;

    stepper_init();
    stepper_enable(false);
    encoder_pwm_angle_init();
    task2_uart1_monitor_init();
    DL_UART_Main_clearInterruptStatus(UART_1_INST, DL_UART_MAIN_INTERRUPT_RX);

    task2_stepper_test_queue_text(txBuffer, &txLength, &txSent,
                                  "DIR TEST READY\r\n");

    while (1) {
        task2_direction_test_process_rx(line, &lineLength, &state, &before,
                                        &commandSteps, txBuffer, &txLength,
                                        &txSent);
        nowMs = delay_get_ms();

        if ((state == TASK2_DIR_TEST_WAIT_MOTION_DONE) &&
            !stepper_is_busy()) {
            stepper_stop();
            stepper_enable(false);
            settleStartMs = nowMs;
            state = TASK2_DIR_TEST_WAIT_SETTLE;
        }

        if ((state == TASK2_DIR_TEST_WAIT_SETTLE) &&
            (txSent >= txLength) &&
            ((uint32_t)(nowMs - settleStartMs) >=
             TASK2_DIR_TEST_SETTLE_MS)) {
            (void)encoder_pwm_angle_get_sample(&after);
            task2_direction_test_queue_result(txBuffer, &txLength, &txSent,
                                              &before, &after, commandSteps);
            state = TASK2_DIR_TEST_IDLE;
        }

        task2_uart1_monitor_send_pending(txBuffer, txLength, &txSent);
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

static bool task2_position_loop_build_line(
    char *buffer,
    uint8_t *length,
    bool loopActive,
    const encoder_pwm_angle_sample_t *angleSample)
{
    uint32_t flags = 0U;
    int32_t targetAngleX10 = task2_float_to_x10(stepper_get_beam_target_deg());
    int32_t actualAngleX10 = angleSample->relativeAngleDegX10;
    static const char prefix[] = "samples:";

    if (stepper_is_enabled()) {
        flags |= 1U << 0;
    }
    if (loopActive) {
        flags |= 1U << 1;
    }
    if (stepper_is_moving()) {
        flags |= 1U << 2;
    }
    if (stepper_is_limit_active()) {
        flags |= 1U << 3;
    }
    if (stepper_is_emergency_inhibited()) {
        flags |= 1U << 4;
    }
    if (angleSample->fresh) {
        flags |= 1U << 5;
    }
    if (angleSample->valid) {
        flags |= 1U << 6;
    }
    if (angleSample->safeLimitActive) {
        flags |= 1U << 7;
    }

    *length = 0U;
    for (uint8_t i = 0U; i < (sizeof(prefix) - 1U); i++) {
        if (!task2_uart1_monitor_append_char(buffer, length, prefix[i])) {
            return false;
        }
    }

    if (!task2_uart1_monitor_append_int32(buffer, length, targetAngleX10) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(buffer, length, actualAngleX10) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(buffer, length,
                                          targetAngleX10 - actualAngleX10) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(
            buffer, length, angleSample->angleDegX10) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(
            buffer, length, stepper_get_position_steps()) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(
            buffer, length, (int32_t)stepper_get_speed_target_hz()) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(
            buffer, length, (int32_t)stepper_get_current_speed_hz()) ||
        !task2_uart1_monitor_append_char(buffer, length, ',') ||
        !task2_uart1_monitor_append_int32(buffer, length, (int32_t)flags) ||
        !task2_uart1_monitor_append_char(buffer, length, '\n')) {
        *length = 0U;
        return false;
    }

    return true;
}

static void task2_position_loop_queue_status(
    char *buffer,
    uint8_t *length,
    uint8_t *sent,
    bool loopActive,
    const encoder_pwm_angle_sample_t *angleSample)
{
    if (*sent < *length) {
        return;
    }

    *length = 0U;
    if (!task2_uart1_append_string(buffer, length, "PLOOP EN:") ||
        !task2_uart1_monitor_append_int32(buffer, length,
                                          stepper_is_enabled() ? 1 : 0) ||
        !task2_uart1_append_string(buffer, length, " ACT:") ||
        !task2_uart1_monitor_append_int32(buffer, length,
                                          loopActive ? 1 : 0) ||
        !task2_uart1_append_string(buffer, length, " TGTX10:") ||
        !task2_uart1_monitor_append_int32(
            buffer, length, task2_float_to_x10(stepper_get_beam_target_deg())) ||
        !task2_uart1_append_string(buffer, length, " RAWX10:") ||
        !task2_uart1_monitor_append_int32(buffer, length,
                                          angleSample->angleDegX10) ||
        !task2_uart1_append_string(buffer, length, " RELX10:") ||
        !task2_uart1_monitor_append_int32(buffer, length,
                                          angleSample->relativeAngleDegX10) ||
        !task2_uart1_append_string(buffer, length, " POS:") ||
        !task2_uart1_monitor_append_int32(buffer, length,
                                          stepper_get_position_steps()) ||
        !task2_uart1_append_string(buffer, length, " FT:") ||
        !task2_uart1_monitor_append_int32(
            buffer, length, (int32_t)stepper_get_speed_target_hz()) ||
        !task2_uart1_append_string(buffer, length, " FC:") ||
        !task2_uart1_monitor_append_int32(
            buffer, length, (int32_t)stepper_get_current_speed_hz()) ||
        !task2_uart1_append_string(buffer, length, " FRESH:") ||
        !task2_uart1_monitor_append_int32(buffer, length,
                                          angleSample->fresh ? 1 : 0) ||
        !task2_uart1_append_string(buffer, length, " VALID:") ||
        !task2_uart1_monitor_append_int32(buffer, length,
                                          angleSample->valid ? 1 : 0) ||
        !task2_uart1_append_string(buffer, length, " SAFE:") ||
        !task2_uart1_monitor_append_int32(
            buffer, length, angleSample->safeLimitActive ? 0 : 1) ||
        !task2_uart1_append_string(buffer, length, " LIM:") ||
        !task2_uart1_monitor_append_int32(buffer, length,
                                          stepper_is_limit_active() ? 1 : 0) ||
        !task2_uart1_append_string(buffer, length, "\r\n")) {
        *length = 0U;
    }
    *sent = 0U;
}

static bool task2_position_target_is_valid(float targetDeg)
{
    if (targetDeg != targetDeg) {
        return false;
    }

    return (targetDeg <= STEPPER_MAX_BEAM_ANGLE_DEG) &&
           (targetDeg >= -STEPPER_MAX_BEAM_ANGLE_DEG);
}

static void task2_position_loop_execute_command(
    char *line,
    bool *loopActive,
    const encoder_pwm_angle_sample_t *angleSample,
    char *txBuffer,
    uint8_t *txLength,
    uint8_t *txSent)
{
    const char *p = line;
    char word[8];
    float angleDeg;
    float targetDeg;
    bool boolValue;

    if (!task2_stepper_test_read_word(&p, word, sizeof(word))) {
        task2_stepper_test_queue_text(txBuffer, txLength, txSent, "ERR\r\n");
        return;
    }

    if (task2_stepper_test_word_equals(word, "HELP")) {
        task2_stepper_test_queue_text(
            txBuffer, txLength, txSent,
            "CMD: STATUS | EN 0/1 | ANGLE rel_deg | TARGET abs_deg | CENTER | ZERO | STOP | ESTOP | RESET\r\n");
    } else if (task2_stepper_test_word_equals(word, "STATUS") ||
               task2_stepper_test_word_equals(word, "GET")) {
        task2_position_loop_queue_status(txBuffer, txLength, txSent,
                                         *loopActive, angleSample);
    } else if (task2_stepper_test_word_equals(word, "EN") &&
               task2_stepper_test_read_bool(&p, &boolValue) &&
               task2_stepper_test_has_no_more(p)) {
        stepper_enable(boolValue);
        if (!boolValue) {
            *loopActive = false;
            stepper_stop();
        }
        task2_position_loop_queue_status(txBuffer, txLength, txSent,
                                         *loopActive, angleSample);
    } else if (task2_stepper_test_word_equals(word, "ANGLE") &&
               task2_stepper_test_read_float(&p, &angleDeg) &&
               task2_stepper_test_has_no_more(p)) {
        if (!stepper_is_enabled()) {
            task2_stepper_test_queue_text(txBuffer, txLength, txSent,
                                          "ERR EN\r\n");
        } else if (stepper_is_emergency_inhibited()) {
            task2_stepper_test_queue_text(txBuffer, txLength, txSent,
                                          "ERR ESTOP\r\n");
        } else if (!angleSample->fresh) {
            task2_stepper_test_queue_text(txBuffer, txLength, txSent,
                                          "ERR ENCODER\r\n");
        } else {
            targetDeg = ((float)angleSample->relativeAngleDegX10 * 0.1f) +
                        angleDeg;
            if (!task2_position_target_is_valid(targetDeg)) {
                task2_stepper_test_queue_text(txBuffer, txLength, txSent,
                                              "ERR RANGE\r\n");
            } else {
                stepper_set_beam_target_deg(targetDeg);
                *loopActive = true;
                task2_position_loop_queue_status(txBuffer, txLength, txSent,
                                                 *loopActive, angleSample);
            }
        }
    } else if ((task2_stepper_test_word_equals(word, "TARGET") ||
                task2_stepper_test_word_equals(word, "GOTO")) &&
               task2_stepper_test_read_float(&p, &angleDeg) &&
               task2_stepper_test_has_no_more(p)) {
        if (!stepper_is_enabled()) {
            task2_stepper_test_queue_text(txBuffer, txLength, txSent,
                                          "ERR EN\r\n");
        } else if (stepper_is_emergency_inhibited()) {
            task2_stepper_test_queue_text(txBuffer, txLength, txSent,
                                          "ERR ESTOP\r\n");
        } else if (!angleSample->fresh) {
            task2_stepper_test_queue_text(txBuffer, txLength, txSent,
                                          "ERR ENCODER\r\n");
        } else if (!task2_position_target_is_valid(angleDeg)) {
            task2_stepper_test_queue_text(txBuffer, txLength, txSent,
                                          "ERR RANGE\r\n");
        } else {
            stepper_set_beam_target_deg(angleDeg);
            *loopActive = true;
            task2_position_loop_queue_status(txBuffer, txLength, txSent,
                                             *loopActive, angleSample);
        }
    } else if (task2_stepper_test_word_equals(word, "CENTER") &&
               task2_stepper_test_has_no_more(p)) {
        if (!stepper_is_enabled()) {
            task2_stepper_test_queue_text(txBuffer, txLength, txSent,
                                          "ERR EN\r\n");
        } else if (stepper_is_emergency_inhibited()) {
            task2_stepper_test_queue_text(txBuffer, txLength, txSent,
                                          "ERR ESTOP\r\n");
        } else if (!angleSample->fresh) {
            task2_stepper_test_queue_text(txBuffer, txLength, txSent,
                                          "ERR ENCODER\r\n");
        } else {
            stepper_set_beam_target_deg(0.0f);
            *loopActive = true;
            task2_position_loop_queue_status(txBuffer, txLength, txSent,
                                             *loopActive, angleSample);
        }
    } else if (task2_stepper_test_word_equals(word, "ZERO") &&
               task2_stepper_test_has_no_more(p) &&
               !stepper_is_busy()) {
        stepper_enable(false);
        (void)stepper_set_zero_position();
        stepper_set_beam_target_deg(0.0f);
        *loopActive = false;
        task2_position_loop_queue_status(txBuffer, txLength, txSent,
                                         *loopActive, angleSample);
    } else if (task2_stepper_test_word_equals(word, "STOP") &&
               task2_stepper_test_has_no_more(p)) {
        *loopActive = false;
        stepper_stop();
        task2_stepper_test_queue_text(txBuffer, txLength, txSent, "OK\r\n");
    } else if (task2_stepper_test_word_equals(word, "ESTOP") &&
               task2_stepper_test_has_no_more(p)) {
        *loopActive = false;
        stepper_emergency_stop();
        task2_stepper_test_queue_text(txBuffer, txLength, txSent, "OK\r\n");
    } else if (task2_stepper_test_word_equals(word, "RESET") &&
               task2_stepper_test_has_no_more(p)) {
        *loopActive = false;
        (void)stepper_clear_emergency_inhibit();
        stepper_stop();
        task2_stepper_test_queue_text(txBuffer, txLength, txSent, "OK\r\n");
    } else {
        task2_stepper_test_queue_text(txBuffer, txLength, txSent, "ERR\r\n");
    }
}

static void task2_position_loop_process_rx(
    char *line,
    uint8_t *lineLength,
    bool *loopActive,
    const encoder_pwm_angle_sample_t *angleSample,
    char *txBuffer,
    uint8_t *txLength,
    uint8_t *txSent)
{
    while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)) {
        uint8_t ch = DL_UART_Main_receiveData(UART_1_INST);

        if ((ch == (uint8_t)'\r') || (ch == (uint8_t)'\n')) {
            if (*lineLength > 0U) {
                line[*lineLength] = '\0';
                task2_position_loop_execute_command(
                    line, loopActive, angleSample, txBuffer, txLength, txSent);
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

void task2_stepper_position_loop_test_run(void)
{
    uint32_t nowMs;
    uint32_t positionLastUpdateMs;
    uint32_t serviceLastUpdateMs;
    uint32_t monitorLastUpdateMs;
    bool loopActive = false;
    encoder_pwm_angle_sample_t angleSample = {0};
    char line[TASK2_STEPPER_TEST_LINE_SIZE];
    uint8_t lineLength = 0U;
    char txBuffer[TASK2_UART1_MONITOR_BUFFER_SIZE];
    uint8_t txLength = 0U;
    uint8_t txSent = 0U;

    stepper_init();
    stepper_enable(false);
    encoder_pwm_angle_init();
    task2_uart1_monitor_init();
    DL_UART_Main_clearInterruptStatus(UART_1_INST, DL_UART_MAIN_INTERRUPT_RX);

    nowMs = delay_get_ms();
    positionLastUpdateMs = nowMs;
    serviceLastUpdateMs = nowMs;
    monitorLastUpdateMs = nowMs;
    task2_stepper_test_queue_text(txBuffer, &txLength, &txSent,
                                  "ENCODER POSITION LOOP READY\r\n");

    while (1) {
        (void)encoder_pwm_angle_get_sample(&angleSample);
        task2_position_loop_process_rx(line, &lineLength, &loopActive,
                                       &angleSample, txBuffer, &txLength,
                                       &txSent);
        nowMs = delay_get_ms();

        if (loopActive &&
            ((uint32_t)(nowMs - positionLastUpdateMs) >=
             TASK2_STEPPER_TEST_POSITION_PERIOD_MS)) {
            stepper_update_beam_encoder_position_loop(
                angleSample.relativeAngleDegX10, angleSample.fresh);
            positionLastUpdateMs = nowMs;
        }

        if ((uint32_t)(nowMs - serviceLastUpdateMs) >=
            TASK2_STEPPER_TEST_SERVICE_PERIOD_MS) {
            stepper_service((float)(nowMs - serviceLastUpdateMs) / 1000.0f);
            serviceLastUpdateMs = nowMs;
        }

        task2_uart1_monitor_send_pending(txBuffer, txLength, &txSent);
        if ((txSent >= txLength) &&
            ((uint32_t)(nowMs - monitorLastUpdateMs) >=
             TASK2_UART1_MONITOR_PERIOD_MS)) {
            if (task2_position_loop_build_line(txBuffer, &txLength,
                                               loopActive, &angleSample)) {
                txSent = 0U;
            }
            monitorLastUpdateMs = nowMs;
        }
    }
}

void task2_ball_balance_run(void)
{
    uint32_t nowMs;
    uint32_t pidLastUpdateMs;
    uint32_t serviceLastUpdateMs;
    uint32_t monitorLastUpdateMs;
    uart_cmd_vision_sample_t visionSample = {0};
    encoder_pwm_angle_sample_t angleSample = {0};
    task2_ball_home_state_t homeState = TASK2_BALL_HOME_STATE_READY;
    char monitorBuffer[TASK2_UART1_MONITOR_BUFFER_SIZE];
    uint8_t monitorLength = 0U;
    uint8_t monitorSent = 0U;
    char commandLine[TASK2_UART1_COMMAND_LINE_SIZE];
    uint8_t commandLineLength = 0U;

    stepper_init();
    task2_uart1_monitor_init();
    if (TASK2_UART1_VISION_RAW_ECHO_ONLY == 0U) {
        task2_uart1_monitor_queue_text_and_flush(
            monitorBuffer, &monitorLength, &monitorSent, "BOOT:UART1\r\n",
            50U);
    }
    stepper_enable(false);
    if (TASK2_UART1_VISION_RAW_ECHO_ONLY == 0U) {
        task2_uart1_monitor_queue_text_and_flush(
            monitorBuffer, &monitorLength, &monitorSent, "BOOT:STEPPER\r\n",
            50U);
    }
    encoder_pwm_angle_init();
    if (TASK2_UART1_VISION_RAW_ECHO_ONLY == 0U) {
        task2_uart1_monitor_queue_text_and_flush(
            monitorBuffer, &monitorLength, &monitorSent, "BOOT:ENCODER\r\n",
            50U);
    }
    uart_cmd_init();
    if (TASK2_UART1_VISION_RAW_ECHO_ONLY == 0U) {
        task2_uart1_monitor_queue_text_and_flush(
            monitorBuffer, &monitorLength, &monitorSent, "BOOT:UART0\r\n",
            50U);
    }
    stepper_enable(true);

    nowMs = delay_get_ms();
    pidLastUpdateMs = nowMs;
    serviceLastUpdateMs = nowMs;
    monitorLastUpdateMs = nowMs;

    while (1) {
        uart_cmd_process();
        if (TASK2_UART1_VISION_RAW_ECHO_ONLY == 0U) {
            task2_uart1_monitor_send_pending(monitorBuffer, monitorLength,
                                             &monitorSent);
            task2_ball_pid_process_uart1_rx(commandLine, &commandLineLength,
                                            monitorBuffer, &monitorLength,
                                            &monitorSent);
        }
        nowMs = delay_get_ms();
        (void)encoder_pwm_angle_get_sample(&angleSample);

        if ((uint32_t)(nowMs - pidLastUpdateMs) >=
            TASK2_BALL_PID_PERIOD_MS) {
            bool visionValid = uart_cmd_get_vision_sample(&visionSample);
            bool beamControlReady;
            float pidDtSeconds =
                (float)(nowMs - pidLastUpdateMs) / 1000.0f;

            beamControlReady = angleSample.fresh;
            if (!beamControlReady) {
                stepper_reset_beam_pid();
                stepper_stop();
            }
            stepper_update_beam_pid(
                (float)visionSample.positionX10 * 0.1f,
                (float)visionSample.velocityX10 * 0.1f,
                visionValid && beamControlReady,
                pidDtSeconds);
            stepper_update_beam_encoder_position_loop(
                angleSample.relativeAngleDegX10,
                visionValid && beamControlReady);
            pidLastUpdateMs = nowMs;
        }

        if ((uint32_t)(nowMs - serviceLastUpdateMs) >=
            TASK2_BALL_SERVICE_PERIOD_MS) {
            float serviceDtSeconds =
                (float)(nowMs - serviceLastUpdateMs) / 1000.0f;

            stepper_service(serviceDtSeconds);
            serviceLastUpdateMs = nowMs;
        }

        if ((TASK2_UART1_VISION_RAW_ECHO_ONLY == 0U) &&
            (monitorSent >= monitorLength) &&
            ((uint32_t)(nowMs - monitorLastUpdateMs) >=
             TASK2_UART1_MONITOR_PERIOD_MS)) {
            if (task2_uart1_monitor_build_line(monitorBuffer, &monitorLength,
                                               &visionSample, &angleSample,
                                               homeState)) {
                monitorSent = 0U;
            }
            monitorLastUpdateMs = nowMs;
        }
    }
}

void task2_encoder_calibration_run(void)
{
    uint32_t nowMs;
    uint32_t monitorLastUpdateMs;
    encoder_pwm_angle_sample_t angleSample = {0};
    char monitorBuffer[TASK2_UART1_MONITOR_BUFFER_SIZE];
    uint8_t monitorLength = 0U;
    uint8_t monitorSent = 0U;

    encoder_pwm_angle_init();
    task2_uart1_monitor_init();

    nowMs = delay_get_ms();
    monitorLastUpdateMs = nowMs;

    while (1) {
        nowMs = delay_get_ms();
        (void)encoder_pwm_angle_get_sample(&angleSample);

        task2_uart1_monitor_send_pending(monitorBuffer, monitorLength,
                                         &monitorSent);

        if ((monitorSent >= monitorLength) &&
            ((uint32_t)(nowMs - monitorLastUpdateMs) >=
             TASK2_UART1_MONITOR_PERIOD_MS)) {
            if (task2_encoder_calibration_build_line(
                    monitorBuffer, &monitorLength, &angleSample)) {
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
    bool stopDetectEnabled = false;
    int32_t traveledDistanceMm = 0;
    int32_t runStartLeftCount;
    int32_t runStartRightCount;
    task2_line_state_t lineState = TASK2_LINE_STATE_RUNNING;
    bool oledOk;

    gray_serial_init();
    task2_discard_startup_gray_samples();
    encoder_init();
    speed_pid_init();
    line_track_init();
    task2_apply_race_line_params();
    bluetooth_init();
    oledOk = oled_init();
    taskStartMs = delay_get_ms();
    runStartLeftCount = encoder_get_left_count();
    runStartRightCount = encoder_get_right_count();

    while (1) {
        bluetooth_process();

        nowMs = delay_get_ms();
        grayRaw = gray_serial_read();
        activeCount = task2_count_active_gray_sensors(grayRaw);

        if (lineState == TASK2_LINE_STATE_RUNNING) {
            line_track_update_with_raw_hold_on_lost(grayRaw);
            traveledDistanceMm = task2_get_traveled_distance_mm(
                runStartLeftCount, runStartRightCount);
            stopDetectEnabled =
                (traveledDistanceMm >= TASK2_STOP_DETECT_ENABLE_DISTANCE_MM);

            if (stopDetectEnabled && task2_stop_marker_detected(grayRaw)) {
                lineState = TASK2_LINE_STATE_STOPPED;
                speed_pid_stop();
                elapsedMs = nowMs - taskStartMs;
                timerStopped = true;
            }
        } else {
            speed_pid_stop();
        }

        speed_pid_control_update();
        if (!timerStopped) {
            elapsedMs = nowMs - taskStartMs;
        }
        task2_oled_update(oledOk, grayRaw, activeCount,
                          lineState, stopDetectEnabled,
                          traveledDistanceMm, elapsedMs);
        delay_ms(SPEED_PID_CONTROL_PERIOD_MS);
    }
}
