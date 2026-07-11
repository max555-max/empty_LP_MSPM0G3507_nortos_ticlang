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
