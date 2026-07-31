#include "bluetooth.h"

#include <stdbool.h>
#include <stdint.h>

#include "angle_control.h"
#include "attitude.h"
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

static bool bluetooth_parse_int32(const char *text, int32_t *value)
{
    const char *p = text;
    int32_t sign = 1;
    int32_t result = 0;
    bool hasDigit = false;

    while ((*p != '\0') && !bluetooth_is_digit(*p) &&
           (*p != '-') && (*p != '+')) {
        p++;
    }

    if (*p == '-') {
        sign = -1;
        p++;
    } else if (*p == '+') {
        p++;
    }

    while (bluetooth_is_digit(*p)) {
        hasDigit = true;
        result = result * 10 + (int32_t)(*p - '0');
        p++;
    }

    if (!hasDigit) {
        return false;
    }

    *value = result * sign;
    return true;
}

static const char *bluetooth_find_value(const char *packet)
{
    const char *p = packet;

    while ((*p != '\0') && (*p != '=') && (*p != ':')) {
        p++;
    }

    if (*p == '\0') {
        return 0;
    }

    return p + 1;
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
        bluetooth_send_char(*text);
        text++;
    }
}

static void bluetooth_send_int32(int32_t value)
{
    char digits[12];
    uint8_t index = 0U;
    uint32_t magnitude;

    if (value < 0) {
        bluetooth_send_char('-');
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }

    do {
        digits[index++] = (char)('0' + (magnitude % 10U));
        magnitude /= 10U;
    } while ((magnitude != 0U) && (index < sizeof(digits)));

    while (index > 0U) {
        bluetooth_send_char(digits[--index]);
    }
}

static void bluetooth_send_ok(void)
{
    bluetooth_send_string("{OK}$\r\n");
}

static void bluetooth_send_error(void)
{
    bluetooth_send_string("{ERR}$\r\n");
}

void bluetooth_send_params(void)
{
    bluetooth_send_string("{C:");
    bluetooth_send_int32(angle_control_get_kp_scaled());
    bluetooth_send_char(':');
    bluetooth_send_int32(angle_control_get_kd_scaled());
    bluetooth_send_char(':');
    bluetooth_send_int32(angle_control_get_base_speed());
    bluetooth_send_char(':');
    bluetooth_send_int32(angle_control_get_max_correction());
    bluetooth_send_char(':');
    bluetooth_send_int32(line_track_get_turn_kp());
    bluetooth_send_char(':');
    bluetooth_send_int32(line_track_get_turn_kd());
    bluetooth_send_char(':');
    bluetooth_send_int32(line_track_get_base_speed());
    bluetooth_send_char(':');
    bluetooth_send_int32(line_track_get_max_correction());
    bluetooth_send_char(':');
    bluetooth_send_int32(line_track_get_left_base_bias());
    bluetooth_send_char(':');
    bluetooth_send_int32(line_track_get_right_base_bias());
    bluetooth_send_string("}$\r\n");
}

static bool bluetooth_apply_indexed_command(uint8_t index, int32_t value)
{
    switch (index) {
    case 0U:
        angle_control_set_kp_scaled(value);
        return true;
    case 1U:
        angle_control_set_kd_scaled(value);
        return true;
    case 2U:
        angle_control_set_base_speed(value);
        return true;
    case 3U:
        angle_control_set_max_correction(value);
        return true;
    case 4U:
        line_track_set_turn_kp(value);
        return true;
    case 5U:
        line_track_set_turn_kd(value);
        return true;
    case 6U:
        line_track_set_base_speed(value);
        return true;
    case 7U:
        line_track_set_max_correction(value);
        return true;
    case 8U:
        line_track_set_left_base_bias(value);
        return true;
    case 9U:
        line_track_set_right_base_bias(value);
        return true;
    default:
        return false;
    }
}

static bool bluetooth_apply_named_command(const char *packet, int32_t value)
{
    attitude_euler_t euler;
    char c0 = bluetooth_to_upper(packet[0]);
    char c1 = bluetooth_to_upper(packet[1]);
    char c2 = bluetooth_to_upper(packet[2]);

    if ((c0 == 'A') && (c1 == 'K') && (c2 == 'P')) {
        angle_control_set_kp_scaled(value);
        return true;
    }
    if ((c0 == 'A') && (c1 == 'K') && (c2 == 'D')) {
        angle_control_set_kd_scaled(value);
        return true;
    }
    if ((c0 == 'A') && (c1 == 'B') && (c2 == 'S')) {
        angle_control_set_base_speed(value);
        return true;
    }
    if ((c0 == 'A') && (c1 == 'M') && (c2 == 'X')) {
        angle_control_set_max_correction(value);
        return true;
    }
    if ((c0 == 'A') && (c1 == 'N') && (c2 == 'G')) {
        attitude_get_euler(&euler);
        angle_control_set_target_yaw(euler.yaw + (float)value);
        angle_control_enable(true);
        return true;
    }
    if ((c0 == 'L') && (c1 == 'K') && (c2 == 'P')) {
        line_track_set_turn_kp(value);
        return true;
    }
    if ((c0 == 'L') && (c1 == 'K') && (c2 == 'D')) {
        line_track_set_turn_kd(value);
        return true;
    }
    if ((c0 == 'L') && (c1 == 'B') && (c2 == 'S')) {
        line_track_set_base_speed(value);
        return true;
    }
    if ((c0 == 'L') && (c1 == 'M') && (c2 == 'X')) {
        line_track_set_max_correction(value);
        return true;
    }
    if ((c0 == 'L') && (c1 == 'L') && (c2 == 'B')) {
        line_track_set_left_base_bias(value);
        return true;
    }
    if ((c0 == 'L') && (c1 == 'R') && (c2 == 'B')) {
        line_track_set_right_base_bias(value);
        return true;
    }

    return false;
}

static void bluetooth_parse_packet(const char *packet)
{
    const char *valueText;
    int32_t value;
    bool applied = false;

    if (bluetooth_packet_equals(packet, "GET") ||
        bluetooth_packet_equals(packet, "PID") ||
        bluetooth_packet_equals(packet, "P") ||
        bluetooth_packet_equals(packet, "?")) {
        bluetooth_send_params();
        return;
    }

    valueText = bluetooth_find_value(packet);
    if ((valueText == 0) || !bluetooth_parse_int32(valueText, &value)) {
        bluetooth_send_error();
        return;
    }

    if (bluetooth_is_digit(packet[0])) {
        applied = bluetooth_apply_indexed_command(
            (uint8_t)(packet[0] - '0'),
            value);
    } else {
        applied = bluetooth_apply_named_command(packet, value);
    }

    if (applied) {
        bluetooth_send_ok();
    } else {
        bluetooth_send_error();
    }
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
