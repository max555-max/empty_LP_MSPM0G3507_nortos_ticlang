#ifndef __VOFA_H_
#define __VOFA_H_

#include <stdint.h>

/*
 * 串口/VOFA 输出模块
 *
 * VOFA 使用 FireWater 协议时，可以直接接收：
 *   samples:ch0,ch1,ch2,...\n
 * 或：
 *   ch0,ch1,ch2,...\n
 *
 * 本项目统一使用 samples: 前缀，方便 VOFA 自动识别多通道曲线。
 */

/* UART0 发送 1 字节。 */
void uart0_send_byte(uint8_t data);

/* UART0 发送字符串。 */
void uart0_send_string(const char *str);

/* UART0 发送有符号整数。 */
void uart0_send_int(int32_t num);

/* UART0 发送浮点数，decimals 为小数位数。 */
void uart0_send_float(float num, uint8_t decimals);

/* 发送两个整数通道。 */
void vofa_send_two_int(int32_t ch0, int32_t ch1);

/* 发送六个整数通道。 */
void vofa_send_six_int(int32_t ch0,
                       int32_t ch1,
                       int32_t ch2,
                       int32_t ch3,
                       int32_t ch4,
                       int32_t ch5);

/* 发送六个浮点通道。 */
void vofa_send_six_float(float ch0,
                         float ch1,
                         float ch2,
                         float ch3,
                         float ch4,
                         float ch5,
                         uint8_t decimals);

#endif
