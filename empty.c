/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */

#include "ti_msp_dl_config.h"

#include "delay.h"
#include "encoder.h"
#include "task2.h"

int main(void)
{
    SYSCFG_DL_init();

    task2_run();
}

void SysTick_Handler(void)
{
    delay_tick();
    encoder_tick_1ms();
}