#ifndef __VOFA_H_
#define __VOFA_H_

#include <stdint.h>

void uart0_send_byte(uint8_t data);
void uart0_send_string(char *str);

#endif