/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */

#include "ti_msp_dl_config.h"

#include "delay.h"
#include "task3.h"

int main(void)
{
    SYSCFG_DL_init();
    task3_run();
}

void SysTick_Handler(void)
{
    delay_tick();
}
