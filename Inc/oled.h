#ifndef __OLED_H_
#define __OLED_H_

#include <stdbool.h>
#include <stdint.h>

bool oled_init(void);
void oled_clear(void);
void oled_clear_line(uint8_t page);
void oled_set_cursor(uint8_t page, uint8_t column);
void oled_print_char(char ch);
void oled_print_string(const char *text);
void oled_print_int(int32_t value);
void oled_print_float(float value, uint8_t decimals);
void oled_print_hex_u8(uint8_t value);

#endif
