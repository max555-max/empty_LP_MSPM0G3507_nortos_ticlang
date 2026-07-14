#include "delay.h"

volatile uint32_t delay_times = 0;
static volatile uint32_t delay_ms_ticks = 0;

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
    delay_ms_ticks++;

    if(delay_times > 0)
    {
        delay_times--;
    }
}

uint32_t delay_get_ms(void)
{
    return delay_ms_ticks;
}
