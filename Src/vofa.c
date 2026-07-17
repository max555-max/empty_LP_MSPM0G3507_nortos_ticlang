#include "vofa.h"
#include "ti_msp_dl_config.h"



/**
 * @brief UART0发送一个字节
 */
void uart0_send_byte(uint8_t data)
{
    /*
     * 等待发送缓冲区空
     */
    while (DL_UART_Main_isBusy(UART0_INST))
    {

    }


    /*
     * 发送数据
     */
    DL_UART_Main_transmitData(UART0_INST,data);
}



/**
 * @brief UART0发送字符串
 */
void uart0_send_string(const char *str)
{
    while (*str)
    {
        uart0_send_byte(*str);

        str++;
    }
}



/**
 * @brief UART0发送整数
 *
 * 例如：
 * uart0_send_int(123);
 *
 * 输出：
 * 123
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
        value = (uint32_t)(-(num + 1)) + 1U;
    }
    else
    {
        value = (uint32_t)num;
    }


    while(value > 0U)
    {
        buf[i++] = (char)(value % 10U + '0');

        value /= 10U;
    }


    while(i > 0)
    {
        uart0_send_byte(buf[--i]);
    }
}

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

void vofa_send_two_int(int32_t ch0, int32_t ch1)
{
    uart0_send_string("samples:");
    uart0_send_int(ch0);
    uart0_send_byte(',');
    uart0_send_int(ch1);
    uart0_send_byte('\n');
}

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
