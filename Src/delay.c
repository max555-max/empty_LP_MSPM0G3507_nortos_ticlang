#include "delay.h"

/*
 * delay.c
 *
 * 毫秒延时与系统时间模块。
 * 依赖 SysTick 或 1ms 定时中断周期调用 delay_tick()。
 */

/* delay_ms() 使用的倒计时变量，在中断中递减，所以必须 volatile。 */
volatile uint32_t delay_times = 0;

/* 系统累计毫秒计数，用于计算 dt、超时和状态机时间。 */
static volatile uint32_t delay_ms_ticks = 0;

/*
 * 阻塞式 ms 延时。
 *
 * 注意：
 *   这个函数会一直等待 delay_tick() 把 delay_times 递减到 0；
 *   如果 1ms 中断没有正常运行，这里会卡住。
 */
void delay_ms(uint32_t ms)
{
    delay_times = ms;

    while(delay_times != 0)
    {

    }
}

/*
 * 1ms 节拍函数。
 *
 * 调用位置：
 *   SysTick_Handler 或其他 1ms 定时器中断。
 */
void delay_tick(void)
{
    delay_ms_ticks++;

    if(delay_times > 0)
    {
        delay_times--;
    }
}

/*
 * 获取系统运行时间，单位 ms。
 *
 * 主要用途：
 *   姿态解算 dt；
 *   任务超时；
 *   路段状态机计时。
 */
uint32_t delay_get_ms(void)
{
    return delay_ms_ticks;
}
