#ifndef __GRAY_SERIAL_H_
#define __GRAY_SERIAL_H_

#include <stdint.h>

void gray_serial_init(void);
uint8_t gray_serial_read(void);
void gray_serial_print(uint8_t value);

#endif
