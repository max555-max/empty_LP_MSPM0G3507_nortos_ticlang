#include "uart_cmd.h"

#include <stdint.h>

#include "pid.h"
#include "ti_msp_dl_config.h"
#include "vofa.h"

#define UART_CMD_BUFFER_SIZE    (80U)
#define UART_CMD_VALUE_COUNT    (8U)

/*
 * uart_cmd.c
 *
 * 串口调参命令解析。
 *
 * 当前命令格式：
 *   一行中解析 8 个整数：
 *   values[0]：左轮目标速度
 *   values[1]：左轮 kp
 *   values[2]：左轮 ki
 *   values[3]：左轮 kd
 *   values[4]：右轮目标速度
 *   values[5]：右轮 kp
 *   values[6]：右轮 ki
 *   values[7]：右轮 kd
 *
 * 解析特点：
 *   不强制逗号格式，只要一行里能依次解析出 8 个整数即可。
 */

/* 接收缓冲区：中断写入，主循环读取。 */
static volatile char g_uartCmdBuffer[UART_CMD_BUFFER_SIZE];

/* 当前已经接收的字符数量。 */
static volatile uint8_t g_uartCmdIndex = 0;

/* 一行命令是否已经接收完成。 */
static volatile uint8_t g_uartCmdReady = 0;

/* 判断字符是否是数字。 */
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

    /* 跳过非数字、非正负号字符，例如逗号、空格、冒号。 */
    while ((*p != '\0') && !uart_cmd_is_digit(*p) &&
           (*p != '-') && (*p != '+')) {
        p++;
    }

    /* 处理可选正负号。 */
    if (*p == '-') {
        sign = -1;
        p++;
    } else if (*p == '+') {
        p++;
    }

    /* 连续读取数字字符。 */
    while (uart_cmd_is_digit(*p)) {
        hasDigit = 1;
        result = result * 10 + (int32_t)(*p - '0');
        p++;
    }

    if (hasDigit == 0U) {
        /* 没有解析到数字。 */
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

    /* 依次从一行字符串中解析 8 个 int32。 */
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
    /* 应用目标速度和左右轮 PID 参数。 */
    speed_pid_set_target(values[0], values[4]);
    speed_pid_set_left_gains(values[1], values[2], values[3]);
    speed_pid_set_right_gains(values[5], values[6], values[7]);
}

void uart_cmd_init(void)
{
    /* 清空接收状态。 */
    g_uartCmdIndex = 0;
    g_uartCmdReady = 0;

    /* 打开 UART0 RX 中断。 */
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
        /* 没有完整命令行时直接返回。 */
        return;
    }

    /*
     * 中断会写 g_uartCmdBuffer，所以复制命令行时短暂关中断，
     * 防止主循环读取过程中缓冲区被修改。
     */
    primask = __get_PRIMASK();
    __disable_irq();

    for (i = 0; i < UART_CMD_BUFFER_SIZE; i++) {
        line[i] = g_uartCmdBuffer[i];
        if (line[i] == '\0') {
            break;
        }
    }
    line[UART_CMD_BUFFER_SIZE - 1U] = '\0';

    /* 复制完成后清接收状态，允许接收下一行。 */
    g_uartCmdIndex = 0;
    g_uartCmdReady = 0;

    __set_PRIMASK(primask);

    /* 解析成功才应用参数；解析失败则忽略该行。 */
    if (uart_cmd_parse_values(line, values) != 0) {
        uart_cmd_apply_values(values);
    }
}

void uart_cmd_irq_handler(void)
{
    uint8_t ch;

    switch (DL_UART_Main_getPendingInterrupt(UART0_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        /* RX FIFO 里可能有多个字节，要一次读完。 */
        while (DL_UART_Main_isRXFIFOEmpty(UART0_INST) == false) {
            ch = DL_UART_Main_receiveData(UART0_INST);

            if ((ch == '\n') || (ch == '\r')) {
                /* 换行表示一条命令结束。 */
                if ((g_uartCmdReady == 0U) && (g_uartCmdIndex > 0U)) {
                    g_uartCmdBuffer[g_uartCmdIndex] = '\0';
                    g_uartCmdReady = 1U;
                }
                g_uartCmdIndex = 0;
            } else if (g_uartCmdReady == 0U) {
                /* 上一行还没处理时，不覆盖缓冲区。 */
                if (g_uartCmdIndex < (UART_CMD_BUFFER_SIZE - 1U)) {
                    g_uartCmdBuffer[g_uartCmdIndex++] = (char)ch;
                } else {
                    /* 缓冲区溢出，丢弃当前行。 */
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
