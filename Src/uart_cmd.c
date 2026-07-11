#include "uart_cmd.h"

#include <stdint.h>

#include "pid.h"
#include "ti_msp_dl_config.h"
#include "vofa.h"

#define UART_CMD_BUFFER_SIZE    (80U)
#define UART_CMD_VALUE_COUNT    (8U)

static volatile char g_uartCmdBuffer[UART_CMD_BUFFER_SIZE];
static volatile uint8_t g_uartCmdIndex = 0;
static volatile uint8_t g_uartCmdReady = 0;

static int uart_cmd_is_digit(char ch)
{
    return (ch >= '0') && (ch <= '9');
}

static int uart_cmd_parse_int32(const char **cursor, int32_t *value)
{
    const char *p = *cursor;
    int32_t sign = 1;
    int32_t result = 0;
    uint8_t hasDigit = 0;

    while ((*p != '\0') && !uart_cmd_is_digit(*p) &&
           (*p != '-') && (*p != '+')) {
        p++;
    }

    if (*p == '-') {
        sign = -1;
        p++;
    } else if (*p == '+') {
        p++;
    }

    while (uart_cmd_is_digit(*p)) {
        hasDigit = 1;
        result = result * 10 + (int32_t)(*p - '0');
        p++;
    }

    if (hasDigit == 0U) {
        return 0;
    }

    *value = result * sign;
    *cursor = p;

    return 1;
}

static int uart_cmd_parse_values(const char *line, int32_t values[])
{
    uint8_t count = 0;
    const char *cursor = line;

    while (count < UART_CMD_VALUE_COUNT) {
        if (uart_cmd_parse_int32(&cursor, &values[count]) == 0) {
            break;
        }

        count++;
    }

    return (count == UART_CMD_VALUE_COUNT);
}

static void uart_cmd_apply_values(const int32_t values[])
{
    speed_pid_set_target(values[0], values[4]);
    speed_pid_set_left_gains(values[1], values[2], values[3]);
    speed_pid_set_right_gains(values[5], values[6], values[7]);
}

void uart_cmd_init(void)
{
    g_uartCmdIndex = 0;
    g_uartCmdReady = 0;

    DL_UART_Main_enableInterrupt(UART0_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART0_INST_INT_IRQN);
}

void uart_cmd_process(void)
{
    char line[UART_CMD_BUFFER_SIZE];
    int32_t values[UART_CMD_VALUE_COUNT];
    uint8_t i;
    uint32_t primask;

    if (g_uartCmdReady == 0U) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    for (i = 0; i < UART_CMD_BUFFER_SIZE; i++) {
        line[i] = g_uartCmdBuffer[i];
        if (line[i] == '\0') {
            break;
        }
    }
    line[UART_CMD_BUFFER_SIZE - 1U] = '\0';

    g_uartCmdIndex = 0;
    g_uartCmdReady = 0;

    __set_PRIMASK(primask);

    if (uart_cmd_parse_values(line, values) != 0) {
        uart_cmd_apply_values(values);
    }
}

void uart_cmd_irq_handler(void)
{
    uint8_t ch;

    switch (DL_UART_Main_getPendingInterrupt(UART0_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        while (DL_UART_Main_isRXFIFOEmpty(UART0_INST) == false) {
            ch = DL_UART_Main_receiveData(UART0_INST);

            if ((ch == '\n') || (ch == '\r')) {
                if ((g_uartCmdReady == 0U) && (g_uartCmdIndex > 0U)) {
                    g_uartCmdBuffer[g_uartCmdIndex] = '\0';
                    g_uartCmdReady = 1U;
                }
                g_uartCmdIndex = 0;
            } else if (g_uartCmdReady == 0U) {
                if (g_uartCmdIndex < (UART_CMD_BUFFER_SIZE - 1U)) {
                    g_uartCmdBuffer[g_uartCmdIndex++] = (char)ch;
                } else {
                    g_uartCmdIndex = 0;
                }
            }
        }
        break;

    default:
        break;
    }
}

void UART0_IRQHandler(void)
{
    uart_cmd_irq_handler();
}
