#include "uart_cmd.h"

#include <stdbool.h>
#include <stdint.h>
#include <limits.h>

#include "delay.h"
#include "stepper.h"
#include "ti_msp_dl_config.h"
#include "vofa.h"

#define UART_CMD_RX_BUFFER_SIZE      (128U)
#define UART_CMD_LINE_SIZE           (80U)
#define UART_CMD_WORD_SIZE           (8U)
#define UART_CMD_VISION_MAX_DATA      (4U)
#define UART_CMD_UART1_MIRROR_BUFFER_SIZE (256U)

typedef enum {
    UART_CMD_VISION_WAIT_HEAD = 0,
    UART_CMD_VISION_WAIT_COMMAND,
    UART_CMD_VISION_WAIT_LENGTH,
    UART_CMD_VISION_WAIT_DATA,
    UART_CMD_VISION_WAIT_CHECKSUM,
    UART_CMD_VISION_WAIT_TAIL
} uart_cmd_vision_rx_state_t;

static volatile uint8_t g_rxBuffer[UART_CMD_RX_BUFFER_SIZE];
static volatile uint8_t g_rxHead = 0U;
static volatile uint8_t g_rxTail = 0U;
static volatile uint16_t g_rxOverflowCount;
static volatile uint32_t g_rxByteCount;
static volatile uint8_t g_uart1MirrorBuffer[UART_CMD_UART1_MIRROR_BUFFER_SIZE];
static volatile uint16_t g_uart1MirrorHead;
static volatile uint16_t g_uart1MirrorTail;
static volatile uint16_t g_uart1MirrorDropCount;

static char g_line[UART_CMD_LINE_SIZE];
static uint8_t g_lineIndex = 0U;
static bool g_packetActive = false;

static uart_cmd_vision_rx_state_t g_visionRxState;
static uint8_t g_visionCommand;
static uint8_t g_visionLength;
static uint8_t g_visionIndex;
static uint8_t g_visionChecksum;
static uint8_t g_visionData[UART_CMD_VISION_MAX_DATA];
static uint32_t g_visionFrameStartMs;
static int16_t g_visionPositionX10;
static int16_t g_visionVelocityX10;
static uint32_t g_visionDataTimestampMs;
static uint32_t g_visionLinkTimestampMs;
static bool g_visionDetectionValid;
static bool g_visionHasPosition;
static bool g_visionHasLink;
static uint16_t g_visionGoodFrames;
static uint16_t g_visionBadFrames;
static uint32_t g_visionStatsWindowStartMs;
static uint16_t g_visionAcceptedFramesInWindow;
static uint16_t g_visionBallFramesInWindow;
static uint16_t g_visionBadFramesInWindow;
static uint16_t g_visionAcceptedFrameRateHz;
static uint16_t g_visionBallFrameRateHz;
static uint16_t g_visionBadFrameRatioPermille;

static int16_t uart_cmd_read_i16_be(const uint8_t *data)
{
    uint16_t value = ((uint16_t)data[0] << 8) | (uint16_t)data[1];

    return (int16_t)value;
}

static void uart_cmd_mirror_uart1_flush_unlocked(void)
{
#if (UART_CMD_UART0_RX_MIRROR_TO_UART1_ENABLE != 0U)
    while ((g_uart1MirrorTail != g_uart1MirrorHead) &&
           (DL_UART_Main_isTXFIFOFull(UART_1_INST) == false)) {
        uint8_t byte = g_uart1MirrorBuffer[g_uart1MirrorTail];

        g_uart1MirrorTail =
            (uint16_t)((g_uart1MirrorTail + 1U) %
                       UART_CMD_UART1_MIRROR_BUFFER_SIZE);
        DL_UART_Main_transmitData(UART_1_INST, byte);
    }
#endif
}

static void uart_cmd_mirror_uart1_flush(void)
{
#if (UART_CMD_UART0_RX_MIRROR_TO_UART1_ENABLE != 0U)
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    uart_cmd_mirror_uart1_flush_unlocked();
    __set_PRIMASK(primask);
#endif
}

static void uart_cmd_mirror_uart0_rx_byte(uint8_t byte)
{
#if (UART_CMD_UART0_RX_MIRROR_TO_UART1_ENABLE != 0U)
    uint16_t nextHead =
        (uint16_t)((g_uart1MirrorHead + 1U) %
                   UART_CMD_UART1_MIRROR_BUFFER_SIZE);

    if (nextHead != g_uart1MirrorTail) {
        g_uart1MirrorBuffer[g_uart1MirrorHead] = byte;
        g_uart1MirrorHead = nextHead;
        uart_cmd_mirror_uart1_flush_unlocked();
        return;
    }

    if (g_uart1MirrorDropCount != UINT16_MAX) {
        g_uart1MirrorDropCount++;
    }
#else
    (void)byte;
#endif
}

static int32_t uart_cmd_abs_i16(int16_t value)
{
    return (value < 0) ? -(int32_t)value : (int32_t)value;
}

static int16_t uart_cmd_vision_apply_direction(int16_t value)
{
    int32_t calibrated = value;

#if (UART_CMD_VISION_POSITION_INVERT != 0U)
    calibrated = -calibrated;
#endif
    if (calibrated > INT16_MAX) {
        return INT16_MAX;
    }
    if (calibrated < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)calibrated;
}

static int16_t uart_cmd_vision_apply_position_calibration(int16_t value)
{
    int32_t calibrated = uart_cmd_vision_apply_direction(value);

    calibrated -= UART_CMD_VISION_POSITION_ZERO_X10;
    if (calibrated > INT16_MAX) {
        return INT16_MAX;
    }
    if (calibrated < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)calibrated;
}

static void uart_cmd_vision_reset_receiver(void)
{
    g_visionRxState = UART_CMD_VISION_WAIT_HEAD;
    g_visionCommand = 0U;
    g_visionLength = 0U;
    g_visionIndex = 0U;
    g_visionChecksum = 0U;
}

static void uart_cmd_vision_increment(uint16_t *counter)
{
    if (*counter != UINT16_MAX) {
        (*counter)++;
    }
}

static uint16_t uart_cmd_vision_rate_from_count(uint16_t count,
                                                  uint32_t elapsedMs)
{
    uint32_t rate;

    if (elapsedMs == 0U) {
        return 0U;
    }

    rate = ((uint32_t)count * 1000U) / elapsedMs;
    return (rate > UINT16_MAX) ? UINT16_MAX : (uint16_t)rate;
}

static void uart_cmd_vision_update_statistics(uint32_t nowMs)
{
    uint32_t elapsedMs;
    uint32_t totalFrames;
    uint32_t ratioPermille;

    elapsedMs = nowMs - g_visionStatsWindowStartMs;
    if (elapsedMs < UART_CMD_VISION_STATS_WINDOW_MS) {
        return;
    }

    g_visionAcceptedFrameRateHz = uart_cmd_vision_rate_from_count(
        g_visionAcceptedFramesInWindow, elapsedMs);
    g_visionBallFrameRateHz = uart_cmd_vision_rate_from_count(
        g_visionBallFramesInWindow, elapsedMs);
    totalFrames = (uint32_t)g_visionAcceptedFramesInWindow +
                  (uint32_t)g_visionBadFramesInWindow;
    ratioPermille = (totalFrames == 0U) ? 0U :
                    ((uint32_t)g_visionBadFramesInWindow * 1000U) /
                    totalFrames;
    g_visionBadFrameRatioPermille = (uint16_t)ratioPermille;
    g_visionAcceptedFramesInWindow = 0U;
    g_visionBallFramesInWindow = 0U;
    g_visionBadFramesInWindow = 0U;
    g_visionStatsWindowStartMs = nowMs;
}

static void uart_cmd_vision_record_good(uint8_t command, uint32_t nowMs)
{
    uart_cmd_vision_update_statistics(nowMs);
    uart_cmd_vision_increment(&g_visionGoodFrames);
    uart_cmd_vision_increment(&g_visionAcceptedFramesInWindow);
    if (command == UART_CMD_VISION_CMD_BALL_STATE) {
        uart_cmd_vision_increment(&g_visionBallFramesInWindow);
    }
}

static void uart_cmd_vision_record_bad(uint32_t nowMs)
{
    uart_cmd_vision_update_statistics(nowMs);
    uart_cmd_vision_increment(&g_visionBadFrames);
    uart_cmd_vision_increment(&g_visionBadFramesInWindow);
}

static bool uart_cmd_vision_has_expected_length(uint8_t command, uint8_t length)
{
    if (command == UART_CMD_VISION_CMD_BALL_STATE) {
        return length == 4U;
    }
    if (command == UART_CMD_VISION_CMD_DETECTION_STATE) {
        return length == 1U;
    }
    if (command == UART_CMD_VISION_CMD_HEARTBEAT) {
        return length == 0U;
    }

    return false;
}

static bool uart_cmd_vision_publish_values(int16_t positionX10,
                                           int16_t velocityX10,
                                           uint32_t nowMs)
{
    int16_t calibratedPositionX10;

    if ((uart_cmd_abs_i16(positionX10) > UART_CMD_VISION_POSITION_LIMIT_X10) ||
        (uart_cmd_abs_i16(velocityX10) > UART_CMD_VISION_VELOCITY_LIMIT_X10)) {
        return false;
    }

    calibratedPositionX10 = uart_cmd_vision_apply_position_calibration(
        positionX10);
    if (g_visionHasPosition &&
        ((uint32_t)(nowMs - g_visionDataTimestampMs) <=
         UART_CMD_VISION_JUMP_WINDOW_MS) &&
        (uart_cmd_abs_i16((int16_t)(calibratedPositionX10 -
                                    g_visionPositionX10)) >
         UART_CMD_VISION_MAX_JUMP_X10)) {
        return false;
    }

    g_visionPositionX10 = calibratedPositionX10;
    g_visionVelocityX10 = uart_cmd_vision_apply_direction(velocityX10);
    g_visionDataTimestampMs = nowMs;
    g_visionHasPosition = true;
    g_visionDetectionValid = true;
    return true;
}

static bool uart_cmd_vision_publish_ball_state(uint32_t nowMs)
{
    int16_t positionX10 = uart_cmd_read_i16_be(&g_visionData[0]);
    int16_t velocityX10 = uart_cmd_read_i16_be(&g_visionData[2]);

    return uart_cmd_vision_publish_values(positionX10, velocityX10, nowMs);
}

static void uart_cmd_vision_execute_frame(uint32_t nowMs)
{
    g_visionHasLink = true;
    g_visionLinkTimestampMs = nowMs;

    if (g_visionCommand == UART_CMD_VISION_CMD_BALL_STATE) {
        if (!uart_cmd_vision_publish_ball_state(nowMs)) {
            uart_cmd_vision_record_bad(nowMs);
            return;
        }
    } else if (g_visionCommand == UART_CMD_VISION_CMD_DETECTION_STATE) {
        if (g_visionData[0] == 0U) {
            g_visionDetectionValid = false;
        } else if ((g_visionData[0] == 1U) && g_visionHasPosition) {
            g_visionDetectionValid = true;
        } else {
            uart_cmd_vision_record_bad(nowMs);
            return;
        }
    }

    uart_cmd_vision_record_good(g_visionCommand, nowMs);
}

static void uart_cmd_vision_check_timeout(uint32_t nowMs)
{
    if ((g_visionRxState != UART_CMD_VISION_WAIT_HEAD) &&
        ((uint32_t)(nowMs - g_visionFrameStartMs) >
         UART_CMD_VISION_FRAME_TIMEOUT_MS)) {
        uart_cmd_vision_record_bad(nowMs);
        uart_cmd_vision_reset_receiver();
    }
}

static bool uart_cmd_vision_process_byte(uint8_t byte, uint32_t nowMs)
{
    bool consumed;

    uart_cmd_vision_check_timeout(nowMs);
    consumed = (g_visionRxState != UART_CMD_VISION_WAIT_HEAD);

    switch (g_visionRxState) {
    case UART_CMD_VISION_WAIT_HEAD:
        if (byte == UART_CMD_VISION_FRAME_HEAD) {
            g_visionFrameStartMs = nowMs;
            g_visionRxState = UART_CMD_VISION_WAIT_COMMAND;
            consumed = true;
        }
        break;

    case UART_CMD_VISION_WAIT_COMMAND:
        g_visionCommand = byte;
        g_visionChecksum = byte;
        g_visionRxState = UART_CMD_VISION_WAIT_LENGTH;
        break;

    case UART_CMD_VISION_WAIT_LENGTH:
        g_visionLength = byte;
        g_visionChecksum ^= byte;
        g_visionIndex = 0U;
        if ((g_visionLength > UART_CMD_VISION_MAX_DATA) ||
            !uart_cmd_vision_has_expected_length(g_visionCommand,
                                                  g_visionLength)) {
            uart_cmd_vision_record_bad(nowMs);
            uart_cmd_vision_reset_receiver();
        } else if (g_visionLength == 0U) {
            g_visionRxState = UART_CMD_VISION_WAIT_CHECKSUM;
        } else {
            g_visionRxState = UART_CMD_VISION_WAIT_DATA;
        }
        break;

    case UART_CMD_VISION_WAIT_DATA:
        g_visionData[g_visionIndex++] = byte;
        g_visionChecksum ^= byte;
        if (g_visionIndex >= g_visionLength) {
            g_visionRxState = UART_CMD_VISION_WAIT_CHECKSUM;
        }
        break;

    case UART_CMD_VISION_WAIT_CHECKSUM:
        if (byte == g_visionChecksum) {
            g_visionRxState = UART_CMD_VISION_WAIT_TAIL;
        } else {
            uart_cmd_vision_record_bad(nowMs);
            uart_cmd_vision_reset_receiver();
        }
        break;

    case UART_CMD_VISION_WAIT_TAIL:
        if (byte == UART_CMD_VISION_FRAME_TAIL) {
            uart_cmd_vision_execute_frame(nowMs);
        } else {
            uart_cmd_vision_record_bad(nowMs);
        }
        uart_cmd_vision_reset_receiver();
        break;

    default:
        uart_cmd_vision_reset_receiver();
        break;
    }

    return consumed;
}

static char uart_cmd_to_upper(char ch)
{
    if ((ch >= 'a') && (ch <= 'z')) {
        return (char)(ch - ('a' - 'A'));
    }

    return ch;
}

static bool uart_cmd_is_digit(char ch)
{
    return (ch >= '0') && (ch <= '9');
}

static bool uart_cmd_is_separator(char ch)
{
    return (ch == ' ') || (ch == '\t') || (ch == ',') ||
           (ch == ':') || (ch == '=');
}

static const char *uart_cmd_skip_separators(const char *text)
{
    while (uart_cmd_is_separator(*text)) {
        text++;
    }

    return text;
}

static bool uart_cmd_read_word(const char **text, char *word, uint8_t wordSize)
{
    uint8_t index = 0U;
    const char *p = uart_cmd_skip_separators(*text);

    if (*p == '\0') {
        return false;
    }

    while ((*p != '\0') && !uart_cmd_is_separator(*p)) {
        if (index < (uint8_t)(wordSize - 1U)) {
            word[index++] = uart_cmd_to_upper(*p);
        }
        p++;
    }

    word[index] = '\0';
    *text = p;
    return index > 0U;
}

static bool uart_cmd_word_equals(const char *word, const char *target)
{
    while ((*word != '\0') && (*target != '\0')) {
        if (*word != *target) {
            return false;
        }
        word++;
        target++;
    }

    return (*word == '\0') && (*target == '\0');
}

static bool uart_cmd_parse_int32(const char **text, int32_t *value)
{
    const char *p = uart_cmd_skip_separators(*text);
    bool negative = false;
    uint32_t result = 0U;
    uint32_t maximumMagnitude = (uint32_t)INT32_MAX;
    bool hasDigit = false;

    if (*p == '-') {
        negative = true;
        maximumMagnitude++;
        p++;
    } else if (*p == '+') {
        p++;
    }

    while (uart_cmd_is_digit(*p)) {
        uint32_t digit = (uint32_t)(*p - '0');

        if (result > ((maximumMagnitude - digit) / 10U)) {
            return false;
        }
        hasDigit = true;
        result = (result * 10U) + digit;
        p++;
    }

    if (!hasDigit) {
        return false;
    }

    if (negative) {
        *value = (result == maximumMagnitude) ? INT32_MIN : -(int32_t)result;
    } else {
        *value = (int32_t)result;
    }
    *text = p;
    return true;
}

static bool uart_cmd_parse_bool(const char **text, bool *value)
{
    int32_t parsed;

    if (!uart_cmd_parse_int32(text, &parsed) ||
        ((parsed != 0) && (parsed != 1))) {
        return false;
    }

    *value = (parsed == 1);
    return true;
}

static bool uart_cmd_parse_scaled_x10(const char **text, int16_t *value)
{
    const char *p = uart_cmd_skip_separators(*text);
    bool negative = false;
    bool hasDigit = false;
    int32_t whole = 0;
    int32_t scaled;
    int32_t firstDecimal = 0;
    int32_t secondDecimal = 0;

    if (*p == '-') {
        negative = true;
        p++;
    } else if (*p == '+') {
        p++;
    }

    while (uart_cmd_is_digit(*p)) {
        int32_t digit = (int32_t)(*p - '0');

        if (whole > ((INT16_MAX / 10) - digit) / 10) {
            return false;
        }
        hasDigit = true;
        whole = (whole * 10) + digit;
        p++;
    }

    if (*p == '.') {
        p++;
        if (uart_cmd_is_digit(*p)) {
            firstDecimal = (int32_t)(*p - '0');
            hasDigit = true;
            p++;
        }
        if (uart_cmd_is_digit(*p)) {
            secondDecimal = (int32_t)(*p - '0');
            p++;
        }
        while (uart_cmd_is_digit(*p)) {
            p++;
        }
    }

    if (!hasDigit) {
        return false;
    }

    scaled = (whole * 10) + firstDecimal;
    if (secondDecimal >= 5) {
        scaled++;
    }
    if (negative) {
        scaled = -scaled;
    }
    if ((scaled > INT16_MAX) || (scaled < INT16_MIN)) {
        return false;
    }

    *value = (int16_t)scaled;
    *text = p;
    return true;
}

static bool uart_cmd_has_no_more(const char *text)
{
    return *uart_cmd_skip_separators(text) == '\0';
}

static bool uart_cmd_parse_vision_text_frame(const char *text)
{
    int16_t positionX10;
    int16_t velocityX10;
    int16_t unusedX10;
    bool detected;
    uint32_t nowMs;

    if (!uart_cmd_parse_scaled_x10(&text, &positionX10) ||
        !uart_cmd_parse_scaled_x10(&text, &velocityX10) ||
        !uart_cmd_parse_scaled_x10(&text, &unusedX10) ||
        !uart_cmd_parse_bool(&text, &detected) ||
        !uart_cmd_has_no_more(text)) {
        uart_cmd_vision_record_bad(delay_get_ms());
        return false;
    }

    (void)unusedX10;
    nowMs = delay_get_ms();
    g_visionHasLink = true;
    g_visionLinkTimestampMs = nowMs;
    if (!detected) {
        g_visionDetectionValid = false;
        uart_cmd_vision_record_good(UART_CMD_VISION_CMD_DETECTION_STATE,
                                    nowMs);
        return true;
    }

    if (!uart_cmd_vision_publish_values(positionX10, velocityX10, nowMs)) {
        uart_cmd_vision_record_bad(nowMs);
        return false;
    }
    uart_cmd_vision_record_good(UART_CMD_VISION_CMD_BALL_STATE, nowMs);
    return true;
}

static uint32_t uart_cmd_abs_to_u32(int32_t value)
{
    if (value >= 0) {
        return (uint32_t)value;
    }

    return (uint32_t)(-(value + 1)) + 1U;
}

static void uart_cmd_send_string(const char *text)
{
#if (UART_CMD_UART0_TEXT_REPLY_ENABLE != 0U)
    uart0_send_string(text);
#else
    (void)text;
#endif
}

static void uart_cmd_send_uint32(uint32_t value)
{
    char digits[10];
    uint8_t index = 0U;

    do {
        digits[index++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (index < sizeof(digits)));

    while (index > 0U) {
        char out[2];
        out[0] = digits[--index];
        out[1] = '\0';
        uart_cmd_send_string(out);
    }
}

static void uart_cmd_send_int32(int32_t value)
{
    if (value < 0) {
        uart_cmd_send_string("-");
        uart_cmd_send_uint32((uint32_t)(-(value + 1)) + 1U);
    } else {
        uart_cmd_send_uint32((uint32_t)value);
    }
}

static void uart_cmd_send_ok(void)
{
    uart_cmd_send_string("OK\r\n");
}

static void uart_cmd_send_error(void)
{
    uart_cmd_send_string("ERR\r\n");
}

static void uart_cmd_send_status(void)
{
    uart_cmd_send_string("STEP EN:");
    uart_cmd_send_int32(stepper_is_enabled() ? 1 : 0);
    uart_cmd_send_string(" BUSY:");
    uart_cmd_send_int32(stepper_is_busy() ? 1 : 0);
    uart_cmd_send_string(" DIR:");
    uart_cmd_send_int32(stepper_get_direction() ? 1 : 0);
    uart_cmd_send_string(" F:");
    uart_cmd_send_uint32(stepper_get_frequency_hz());
    uart_cmd_send_string(" REM:");
    uart_cmd_send_uint32(stepper_get_remaining_steps());
    uart_cmd_send_string(" LOCK:");
    uart_cmd_send_int32(stepper_is_emergency_inhibited() ? 1 : 0);
    uart_cmd_send_string(" LIM:");
    uart_cmd_send_int32(stepper_is_limit_active() ? 1 : 0);
    uart_cmd_send_string(" POS:");
    uart_cmd_send_int32(stepper_get_position_steps());
    uart_cmd_send_string("\r\n");
}

static void uart_cmd_send_help(void)
{
    uart_cmd_send_string("CMD: GET | S steps freq | SD steps freq dir | RUN freq dir | STOP | EN 0/1 | ESTOP | RESET\r\n");
}

static void uart_cmd_parse_line(const char *line)
{
    char word[UART_CMD_WORD_SIZE];
    int32_t value0;
    int32_t value1;
    const char *p = line;

    if (!uart_cmd_read_word(&p, word, sizeof(word))) {
        return;
    }

    if (uart_cmd_word_equals(word, "V") ||
        uart_cmd_word_equals(word, "VISION")) {
        (void)uart_cmd_parse_vision_text_frame(p);
        return;
    }

    if (uart_cmd_word_equals(word, "GET") ||
        uart_cmd_word_equals(word, "?") ||
        uart_cmd_word_equals(word, "STATUS")) {
        uart_cmd_send_status();
        return;
    }

    if (uart_cmd_word_equals(word, "HELP")) {
        uart_cmd_send_help();
        return;
    }

    if (uart_cmd_word_equals(word, "STOP") ||
        uart_cmd_word_equals(word, "ST")) {
        stepper_stop();
        uart_cmd_send_ok();
        return;
    }

    if (uart_cmd_word_equals(word, "ESTOP")) {
        stepper_emergency_stop();
        uart_cmd_send_ok();
        return;
    }

    if (uart_cmd_word_equals(word, "RESET")) {
        if (stepper_clear_emergency_inhibit()) {
            uart_cmd_send_ok();
        } else {
            uart_cmd_send_error();
        }
        return;
    }

    if (uart_cmd_word_equals(word, "EN")) {
        bool enable;

        if (!uart_cmd_parse_bool(&p, &enable)) {
            uart_cmd_send_error();
            return;
        }

        stepper_enable(enable);
        uart_cmd_send_ok();
        return;
    }

    if (uart_cmd_word_equals(word, "S") ||
        uart_cmd_word_equals(word, "MOVE") ||
        uart_cmd_word_equals(word, "M")) {
        if (!uart_cmd_parse_int32(&p, &value0) ||
            !uart_cmd_parse_int32(&p, &value1)) {
            uart_cmd_send_error();
            return;
        }

        if (stepper_start(value0, (uint32_t)uart_cmd_abs_to_u32(value1))) {
            uart_cmd_send_ok();
        } else {
            uart_cmd_send_error();
        }
        return;
    }

    if (uart_cmd_word_equals(word, "SD")) {
        bool direction;

        if (!uart_cmd_parse_int32(&p, &value0) ||
            !uart_cmd_parse_int32(&p, &value1) ||
            !uart_cmd_parse_bool(&p, &direction)) {
            uart_cmd_send_error();
            return;
        }

        if (stepper_start_dir(uart_cmd_abs_to_u32(value0),
                               uart_cmd_abs_to_u32(value1),
                               direction)) {
            uart_cmd_send_ok();
        } else {
            uart_cmd_send_error();
        }
        return;
    }

    if (uart_cmd_word_equals(word, "RUN")) {
        bool direction;

        if (!uart_cmd_parse_int32(&p, &value0) ||
            !uart_cmd_parse_bool(&p, &direction)) {
            uart_cmd_send_error();
            return;
        }

        if (stepper_start_continuous(uart_cmd_abs_to_u32(value0), direction)) {
            uart_cmd_send_ok();
        } else {
            uart_cmd_send_error();
        }
        return;
    }

    uart_cmd_send_error();
}

static void uart_cmd_push_line_char(uint8_t ch)
{
    if (ch == (uint8_t)'{') {
        g_packetActive = true;
        g_lineIndex = 0U;
        return;
    }

    if ((ch == (uint8_t)'}') && g_packetActive) {
        g_line[g_lineIndex] = '\0';
        uart_cmd_parse_line(g_line);
        g_packetActive = false;
        g_lineIndex = 0U;
        return;
    }

    if ((ch == (uint8_t)'\r') || (ch == (uint8_t)'\n')) {
        if ((g_lineIndex > 0U) && !g_packetActive) {
            g_line[g_lineIndex] = '\0';
            uart_cmd_parse_line(g_line);
        }
        g_lineIndex = 0U;
        return;
    }

    if (((ch >= (uint8_t)' ') || (ch == (uint8_t)'\t')) &&
        (g_lineIndex < (UART_CMD_LINE_SIZE - 1U))) {
        g_line[g_lineIndex++] = (char)ch;
    } else if (g_lineIndex >= (UART_CMD_LINE_SIZE - 1U)) {
        g_packetActive = false;
        g_lineIndex = 0U;
        uart_cmd_send_error();
    }
}

void uart_cmd_init(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_rxHead = 0U;
    g_rxTail = 0U;
    g_rxOverflowCount = 0U;
    g_rxByteCount = 0U;
    g_uart1MirrorHead = 0U;
    g_uart1MirrorTail = 0U;
    g_uart1MirrorDropCount = 0U;
    g_lineIndex = 0U;
    g_packetActive = false;
    uart_cmd_vision_reset_receiver();
    g_visionFrameStartMs = 0U;
    g_visionPositionX10 = 0;
    g_visionVelocityX10 = 0;
    g_visionDataTimestampMs = 0U;
    g_visionLinkTimestampMs = 0U;
    g_visionDetectionValid = false;
    g_visionHasPosition = false;
    g_visionHasLink = false;
    g_visionGoodFrames = 0U;
    g_visionBadFrames = 0U;
    g_visionStatsWindowStartMs = delay_get_ms();
    g_visionAcceptedFramesInWindow = 0U;
    g_visionBallFramesInWindow = 0U;
    g_visionBadFramesInWindow = 0U;
    g_visionAcceptedFrameRateHz = 0U;
    g_visionBallFrameRateHz = 0U;
    g_visionBadFrameRatioPermille = 0U;
    __set_PRIMASK(primask);

    DL_UART_Main_clearInterruptStatus(UART0_INST, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enableInterrupt(UART0_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART0_INST_INT_IRQN);

    uart_cmd_send_string("UART READY\r\n");
}

void uart_cmd_deinit(void)
{
    DL_UART_Main_disableInterrupt(UART0_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART0_INST_INT_IRQN);
    NVIC_DisableIRQ(UART0_INST_INT_IRQN);
}

void uart_cmd_process(void)
{
    uint8_t ch;
    uint32_t nowMs;

    uart_cmd_mirror_uart1_flush();

    while (g_rxTail != g_rxHead) {
        uint32_t primask = __get_PRIMASK();

        __disable_irq();
        if (g_rxTail == g_rxHead) {
            __set_PRIMASK(primask);
            break;
        }
        ch = g_rxBuffer[g_rxTail];
        g_rxTail = (uint8_t)((g_rxTail + 1U) % UART_CMD_RX_BUFFER_SIZE);
        __set_PRIMASK(primask);

        nowMs = delay_get_ms();
        if (!uart_cmd_vision_process_byte(ch, nowMs)) {
            uart_cmd_push_line_char(ch);
        }
    }

    nowMs = delay_get_ms();
    uart_cmd_vision_check_timeout(nowMs);
    uart_cmd_vision_update_statistics(nowMs);
    uart_cmd_mirror_uart1_flush();
}

bool uart_cmd_get_vision_sample(uart_cmd_vision_sample_t *sample)
{
    uint32_t nowMs;

    if (sample == 0) {
        return false;
    }

    nowMs = delay_get_ms();
    sample->positionX10 = g_visionPositionX10;
    sample->velocityX10 = g_visionVelocityX10;
    sample->dataTimestampMs = g_visionDataTimestampMs;
    sample->linkTimestampMs = g_visionLinkTimestampMs;
    sample->fresh = g_visionHasPosition &&
                    ((uint32_t)(nowMs - g_visionDataTimestampMs) <=
                     UART_CMD_VISION_DATA_TIMEOUT_MS);
    sample->linkOnline = g_visionHasLink &&
                         ((uint32_t)(nowMs - g_visionLinkTimestampMs) <=
                          UART_CMD_VISION_LINK_TIMEOUT_MS);
    sample->valid = g_visionDetectionValid && sample->fresh &&
                    sample->linkOnline;
    return sample->valid;
}

bool uart_cmd_is_vision_online(void)
{
    return g_visionHasLink &&
           ((uint32_t)(delay_get_ms() - g_visionLinkTimestampMs) <=
            UART_CMD_VISION_LINK_TIMEOUT_MS);
}

void uart_cmd_get_vision_link_status(uart_cmd_vision_link_status_t *status)
{
    uint32_t nowMs;

    if (status == 0) {
        return;
    }

    nowMs = delay_get_ms();
    uart_cmd_vision_update_statistics(nowMs);
    status->rxByteCount = g_rxByteCount;
    status->acceptedFrameRateHz = g_visionAcceptedFrameRateHz;
    status->ballStateFrameRateHz = g_visionBallFrameRateHz;
    status->badFrameRatioPermille = g_visionBadFrameRatioPermille;
    status->acceptedFrameCount = g_visionGoodFrames;
    status->badFrameCount = g_visionBadFrames;
    status->rxOverflowCount = g_rxOverflowCount;
    status->dataFresh = g_visionHasPosition &&
                        ((uint32_t)(nowMs - g_visionDataTimestampMs) <=
                         UART_CMD_VISION_DATA_TIMEOUT_MS);
    status->linkOnline = g_visionHasLink &&
                         ((uint32_t)(nowMs - g_visionLinkTimestampMs) <=
                          UART_CMD_VISION_LINK_TIMEOUT_MS);
}

uint16_t uart_cmd_get_vision_good_frame_count(void)
{
    return g_visionGoodFrames;
}

uint16_t uart_cmd_get_vision_bad_frame_count(void)
{
    return g_visionBadFrames;
}

void uart_cmd_irq_handler(void)
{
    while (DL_UART_Main_isRXFIFOEmpty(UART0_INST) == false) {
        uint8_t nextHead;
        uint8_t ch = DL_UART_Main_receiveData(UART0_INST);

        g_rxByteCount++;
        uart_cmd_mirror_uart0_rx_byte(ch);

        nextHead = (uint8_t)((g_rxHead + 1U) % UART_CMD_RX_BUFFER_SIZE);
        if (nextHead != g_rxTail) {
            g_rxBuffer[g_rxHead] = ch;
            g_rxHead = nextHead;
        } else if (g_rxOverflowCount != UINT16_MAX) {
            g_rxOverflowCount++;
        }
    }
}

void UART0_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART0_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        uart_cmd_irq_handler();
        break;

    default:
        break;
    }
}
