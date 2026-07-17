#ifndef __VOFA_H_
#define __VOFA_H_

#include <stdint.h>

void uart0_send_byte(uint8_t data);
void uart0_send_string(const char *str);
void uart0_send_int(int32_t num);
void uart0_send_float(float num, uint8_t decimals);
void vofa_send_two_int(int32_t ch0, int32_t ch1);
void vofa_send_six_int(int32_t ch0,
                       int32_t ch1,
                       int32_t ch2,
                       int32_t ch3,
                       int32_t ch4,
                       int32_t ch5);
void vofa_send_six_float(float ch0,
                         float ch1,
                         float ch2,
                         float ch3,
                         float ch4,
                         float ch5,
                         uint8_t decimals);

#endif
