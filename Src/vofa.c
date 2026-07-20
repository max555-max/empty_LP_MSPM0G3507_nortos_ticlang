#include "vofa.h"
#include "ti_msp_dl_config.h"

/*
 * vofa.c
 *
 * UART0 基础发送 + VOFA FireWater 数据输出。
 *
 * FireWater 常用格式：
 *   samples:ch0,ch1,ch2,...\n
 *
 * VOFA 遇到换行符才会刷新一帧数据，所以每次发送完整通道后必须
 * 以 '\n' 结尾。
 */

/*
 * UART0 发送一个字节。
 *
 * 注意：
 *   这里是阻塞发送，会等待 UART 空闲。
 */
void uart0_send_byte(uint8_t data)
{
    /* 等待发送硬件空闲。 */
    while (DL_UART_Main_isBusy(UART0_INST))
    {

    }

    /* 发送 1 字节数据。 */
    DL_UART_Main_transmitData(UART0_INST,data);
}

/*
 * UART0 发送 C 字符串。
 *
 * 字符串必须以 '\0' 结尾。
 */
void uart0_send_string(const char *str)
{
    while (*str)
    {
        uart0_send_byte(*str);

        str++;
    }
}

/*
 * UART0 发送有符号整数。
 *
 * 例：
 *   uart0_send_int(-123);
 * 输出：
 *   -123
 */
void uart0_send_int(int32_t num)
{
    char buf[12];
    uint32_t value;

    int i = 0;

    if(num == 0)
    {
        uart0_send_byte('0');
        return;
    }

    if(num < 0)
    {
        uart0_send_byte('-');

        /*
         * 避免直接 -INT32_MIN 溢出。
         * 写成 -(num + 1) + 1 可以安全处理最小负数。
         */
        value = (uint32_t)(-(num + 1)) + 1U;
    }
    else
    {
        value = (uint32_t)num;
    }

    /* 反向存储每一位数字。 */
    while(value > 0U)
    {
        buf[i++] = (char)(value % 10U + '0');

        value /= 10U;
    }

    /* 再倒序发送，得到正常数字顺序。 */
    while(i > 0)
    {
        uart0_send_byte(buf[--i]);
    }
}

/*
 * UART0 发送浮点数。
 *
 * decimals：
 *   保留的小数位数。
 *
 * 注意：
 *   这是轻量级打印函数，不使用 printf，减少嵌入式代码体积。
 */
void uart0_send_float(float num, uint8_t decimals)
{
    int32_t integerPart;
    float fractionPart;

    if (num < 0.0f) {
        uart0_send_byte('-');
        num = -num;
    }

    integerPart = (int32_t) num;
    fractionPart = num - (float) integerPart;

    uart0_send_int(integerPart);

    if (decimals == 0U) {
        return;
    }

    uart0_send_byte('.');
    for (uint8_t i = 0U; i < decimals; i++) {
        uint8_t digit;

        fractionPart *= 10.0f;
        digit = (uint8_t) fractionPart;
        uart0_send_byte((uint8_t) ('0' + digit));
        fractionPart -= (float) digit;
    }
}

/* 按 FireWater 格式发送 2 个整数通道。 */
void vofa_send_two_int(int32_t ch0, int32_t ch1)
{
    uart0_send_string("samples:");
    uart0_send_int(ch0);
    uart0_send_byte(',');
    uart0_send_int(ch1);
    uart0_send_byte('\n');
}

/* 按 FireWater 格式发送 6 个整数通道。 */
void vofa_send_six_int(int32_t ch0,
                       int32_t ch1,
                       int32_t ch2,
                       int32_t ch3,
                       int32_t ch4,
                       int32_t ch5)
{
    uart0_send_string("samples:");
    uart0_send_int(ch0);
    uart0_send_byte(',');
    uart0_send_int(ch1);
    uart0_send_byte(',');
    uart0_send_int(ch2);
    uart0_send_byte(',');
    uart0_send_int(ch3);
    uart0_send_byte(',');
    uart0_send_int(ch4);
    uart0_send_byte(',');
    uart0_send_int(ch5);
    uart0_send_byte('\n');
}

/* 按 FireWater 格式发送 6 个浮点通道。 */
void vofa_send_six_float(float ch0,
                         float ch1,
                         float ch2,
                         float ch3,
                         float ch4,
                         float ch5,
                         uint8_t decimals)
{
    uart0_send_string("samples:");
    uart0_send_float(ch0, decimals);
    uart0_send_byte(',');
    uart0_send_float(ch1, decimals);
    uart0_send_byte(',');
    uart0_send_float(ch2, decimals);
    uart0_send_byte(',');
    uart0_send_float(ch3, decimals);
    uart0_send_byte(',');
    uart0_send_float(ch4, decimals);
    uart0_send_byte(',');
    uart0_send_float(ch5, decimals);
    uart0_send_byte('\n');
}
