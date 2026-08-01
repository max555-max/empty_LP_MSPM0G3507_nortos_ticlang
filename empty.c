/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */

#include "ti_msp_dl_config.h"

#include <stdint.h>

#include "delay.h"
#include "encoder.h"
#include "task3.h"

int main(void)
{
    SYSCFG_DL_init();

    task3_run();

    while (1) {
    }
}

void SysTick_Handler(void)
{
    delay_tick();
    encoder_tick_1ms();
}
