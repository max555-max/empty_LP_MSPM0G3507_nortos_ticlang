#ifndef __VOFA_H_
#define __VOFA_H_

#include <stdint.h>

void uart0_send_byte(uint8_t data);
void uart0_send_string(const char *str);
void uart0_send_int(int32_t num);
void vofa_send_two_int(int32_t ch0, int32_t ch1);
void vofa_send_six_int(int32_t ch0,
                       int32_t ch1,
                       int32_t ch2,
                       int32_t ch3,
                       int32_t ch4,
                       int32_t ch5);

#endif
