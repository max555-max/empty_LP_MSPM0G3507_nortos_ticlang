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
void uart0_send_string(char *str)
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

    int i = 0;


    if(num == 0)
    {
        uart0_send_byte('0');
        return;
    }


    if(num < 0)
    {
        uart0_send_byte('-');
        num = -num;
    }


    while(num > 0)
    {
        buf[i++] = num % 10 + '0';

        num /= 10;
    }


    while(i > 0)
    {
        uart0_send_byte(buf[--i]);
    }
}