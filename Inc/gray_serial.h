#ifndef __GRAY_SERIAL_H_
#define __GRAY_SERIAL_H_

#include <stdint.h>

/*
 * 四路循迹数字输入模块。
 * 物理顺序：O1(最左) -> O4(最右)
 * 有效电平：低电平表示检测到黑线。
 */

void gray_serial_init(void);
uint8_t gray_serial_read(void);
void gray_serial_print(uint8_t value);

#endif
