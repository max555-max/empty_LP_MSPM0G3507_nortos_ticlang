#include "bluetooth.h"

#include <stdbool.h>
#include <stdint.h>
#include <limits.h>

#include "line_track.h"
#include "ti_msp_dl_config.h"

#define BLUETOOTH_RX_BUFFER_SIZE    (128U)
#define BLUETOOTH_PACKET_SIZE       (64U)

static volatile uint8_t g_rxBuffer[BLUETOOTH_RX_BUFFER_SIZE];
static volatile uint8_t g_rxHead = 0U;
static volatile uint8_t g_rxTail = 0U;

static char g_packet[BLUETOOTH_PACKET_SIZE];
static uint8_t g_packetIndex = 0U;
static bool g_packetActive = false;

static char bluetooth_to_upper(char ch)
{
    if ((ch >= 'a') && (ch <= 'z')) {
        return (char)(ch - ('a' - 'A'));
    }

    return ch;
}

static bool bluetooth_is_digit(char ch)
{
    return (ch >= '0') && (ch <= '9');
}

static bool bluetooth_is_separator(char ch)
{
    return (ch == '\0') || (ch == ' ') || (ch == '\t') ||
           (ch == '=') || (ch == ':') || (ch == ',');
}

static const char *bluetooth_skip_separators(const char *text)
{
    while (bluetooth_is_separator(*text) && (*text != '\0')) {
        text++;
    }

    return text;
}

static bool bluetooth_read_word(const char **text, char *word, uint8_t size)
{
    const char *p = bluetooth_skip_separators(*text);
    uint8_t index = 0U;

    while ((*p != '\0') && !bluetooth_is_separator(*p)) {
        if ((index + 1U) >= size) {
            return false;
        }
        word[index++] = *p++;
    }

    if (index == 0U) {
        return false;
    }

    word[index] = '\0';
    *text = p;
    return true;
}

static bool bluetooth_parse_int32(const char **text, int32_t *value)
{
    const char *p = bluetooth_skip_separators(*text);
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

    while (bluetooth_is_digit(*p)) {
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

static bool bluetooth_parse_ratio_percent(const char **text,
                                           int32_t *percent)
{
    const char *p = bluetooth_skip_separators(*text);
    int32_t whole;

    if (!bluetooth_parse_int32(&p, &whole) || (whole < 0)) {
        return false;
    }

    if (*p == '.') {
        int32_t fraction = 0;
        uint8_t digits = 0U;

        p++;
        while (bluetooth_is_digit(*p) && (digits < 2U)) {
            fraction = (fraction * 10) + (*p - '0');
            p++;
            digits++;
        }

        if ((digits == 0U) || bluetooth_is_digit(*p) || (whole > 1)) {
            return false;
        }

        if (digits == 1U) {
            fraction *= 10;
        }

        if ((whole == 1) && (fraction != 0)) {
            return false;
        }

        *percent = (whole * 100) + fraction;
    } else {
        *percent = whole;
    }

    if (*percent > 100) {
        return false;
    }

    *text = p;
    return true;
}

static bool bluetooth_is_packet_end(const char *text)
{
    return *bluetooth_skip_separators(text) == '\0';
}

static bool bluetooth_packet_equals(const char *packet, const char *word)
{
    while ((*packet != '\0') && (*word != '\0')) {
        if (bluetooth_to_upper(*packet) != *word) {
            return false;
        }
        packet++;
        word++;
    }

    return (*packet == '\0') && (*word == '\0');
}

static void bluetooth_send_char(char ch)
{
    DL_UART_Main_transmitDataBlocking(UART_1_INST, (uint8_t)ch);
}

static void bluetooth_send_string(const char *text)
{
    while (*text != '\0') {
        bluetooth_send_char(*text++);
    }
}

static void bluetooth_send_ratio(int32_t percent)
{
    bluetooth_send_char((percent == 100) ? '1' : '0');
    bluetooth_send_char('.');
    bluetooth_send_char((char)('0' + ((percent % 100) / 10)));
    bluetooth_send_char((char)('0' + (percent % 10)));
}

static void bluetooth_send_error(void)
{
    bluetooth_send_string("{ERR}\r\n");
}

void bluetooth_send_params(void)
{
    bluetooth_send_string("{DIFF=");
    bluetooth_send_ratio(line_track_get_small_turn_percent());
    bluetooth_send_char(',');
    bluetooth_send_ratio(line_track_get_large_turn_percent());
    bluetooth_send_string("}\r\n");
}

static void bluetooth_parse_packet(const char *packet)
{
    const char *p = packet;
    char word[8];
    int32_t smallTurnPercent;
    int32_t largeTurnPercent;

    if (!bluetooth_read_word(&p, word, sizeof(word))) {
        bluetooth_send_error();
        return;
    }

    if (bluetooth_packet_equals(word, "GET") ||
        bluetooth_packet_equals(word, "?")) {
        bluetooth_send_params();
        return;
    }

    if (bluetooth_packet_equals(word, "HELP")) {
        bluetooth_send_string(
            "{HELP GET | DIFF=0.90,0.60}\r\n");
        return;
    }

    if (bluetooth_packet_equals(word, "SET") ||
        bluetooth_packet_equals(word, "PARAM") ||
        bluetooth_packet_equals(word, "DIFF")) {
        if (bluetooth_parse_ratio_percent(&p, &smallTurnPercent) &&
            bluetooth_parse_ratio_percent(&p, &largeTurnPercent) &&
            bluetooth_is_packet_end(p) &&
            line_track_set_turn_ratios(smallTurnPercent,
                                       largeTurnPercent)) {
            bluetooth_send_params();
        } else {
            bluetooth_send_error();
        }
        return;
    }

    bluetooth_send_error();
}

void bluetooth_init(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_rxHead = 0U;
    g_rxTail = 0U;
    g_packetIndex = 0U;
    g_packetActive = false;
    __set_PRIMASK(primask);

    DL_UART_Main_clearInterruptStatus(UART_1_INST, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enableInterrupt(UART_1_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}

void bluetooth_process(void)
{
    uint8_t ch;

    while (g_rxTail != g_rxHead) {
        uint32_t primask = __get_PRIMASK();

        __disable_irq();
        if (g_rxTail == g_rxHead) {
            __set_PRIMASK(primask);
            break;
        }
        ch = g_rxBuffer[g_rxTail];
        g_rxTail = (uint8_t)((g_rxTail + 1U) % BLUETOOTH_RX_BUFFER_SIZE);
        __set_PRIMASK(primask);

        if (ch == (uint8_t)'{') {
            g_packetActive = true;
            g_packetIndex = 0U;
        } else if ((ch == (uint8_t)'}') && g_packetActive) {
            g_packet[g_packetIndex] = '\0';
            bluetooth_parse_packet(g_packet);
            g_packetActive = false;
            g_packetIndex = 0U;
        } else if (g_packetActive) {
            if (g_packetIndex < (BLUETOOTH_PACKET_SIZE - 1U)) {
                g_packet[g_packetIndex++] = (char)ch;
            } else {
                g_packetActive = false;
                g_packetIndex = 0U;
            }
        }
    }
}

void UART_1_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_1_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        while (DL_UART_Main_isRXFIFOEmpty(UART_1_INST) == false) {
            uint8_t nextHead;
            uint8_t ch = DL_UART_Main_receiveData(UART_1_INST);

            nextHead = (uint8_t)((g_rxHead + 1U) % BLUETOOTH_RX_BUFFER_SIZE);
            if (nextHead != g_rxTail) {
                g_rxBuffer[g_rxHead] = ch;
                g_rxHead = nextHead;
            }
        }
        break;

    default:
        break;
    }
}
