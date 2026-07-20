/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "encoder.h"
#include "task2_abcd.h"

/*
 * main()
 *
 * 当前工程主入口。
 * 现在运行的是第三问 task3_acbd_run()。
 *
 * 如果后面要切换任务，只需要：
 *   1. 修改 include 的任务头文件；
 *   2. 修改 main() 中调用的任务函数。
 */
int main(void)
{
    /* 初始化 SysConfig 生成的时钟、GPIO、UART、SPI、PWM 等外设。 */
    SYSCFG_DL_init();

    /* 运行第三问完整流程。该函数内部会一直运行，正常不会返回。 */
    task2_abcd_run();

    while (1) {
    }
}

/*
 * SysTick 1ms 中断。
 *
 * delay_tick()：
 *   提供 delay_ms() 和 delay_get_ms() 的时间基准；
 *
 * encoder_tick_1ms()：
 *   每 1ms 调用一次，内部累计到 10ms 后更新编码器速度。
 */
void SysTick_Handler(void)
{
    delay_tick();
    encoder_tick_1ms();
}
