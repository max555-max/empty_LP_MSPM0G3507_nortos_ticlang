#include "delay.h"

volatile uint32_t delay_times = 0;

/*
 * ms延时
 *
 * 注意：
 * 依赖SysTick 1ms中断
 */
void delay_ms(uint32_t ms)
{
    delay_times = ms;

    while(delay_times != 0)
    {

    }
}

/*
 * SysTick中断调用
 */
void delay_tick(void)
{
    if(delay_times > 0)
    {
        delay_times--;
    }
}