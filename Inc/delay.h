#ifndef __DELAY_H_
#define __DELAY_H_

#include "ti_msp_dl_config.h"
#include <stdint.h>

/*
 * 简单毫秒延时/系统时间模块
 *
 * delay_tick() 一般放在 1ms 定时中断里调用；
 * delay_get_ms() 返回从上电开始累计的毫秒数；
 * delay_ms() 是阻塞延时，适合初始化或状态机短暂停顿。
 */

/* 阻塞延时 ms 毫秒。 */
void delay_ms(uint32_t ms);

/* 1ms 节拍函数：由定时器中断调用，累加系统毫秒计数。 */
void delay_tick(void);

/* 获取当前系统毫秒计数。 */
uint32_t delay_get_ms(void);

#endif
