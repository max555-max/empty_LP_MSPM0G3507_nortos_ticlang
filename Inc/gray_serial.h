#ifndef __GRAY_SERIAL_H_
#define __GRAY_SERIAL_H_

#include <stdint.h>

/*
 * 八路灰度传感器串行读取模块
 *
 * 传感器使用一个时钟线和一个数据线输出 8bit 数字量。
 * gray_serial_read() 返回原始 bit 顺序；
 * gray_serial_print() 会按实际从左到右通道 1~8 的顺序打印。
 */

/* 初始化串行灰度读取引脚，默认把 CLK 拉高。 */
void gray_serial_init(void);

/* 读取一次 8 路灰度数字量，返回原始 8bit 数据。 */
uint8_t gray_serial_read(void);

/* 将一次读取结果按 Digital:ch1,...,ch8 格式打印到串口。 */
void gray_serial_print(uint8_t value);

#endif
