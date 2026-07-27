/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */

#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

#include "delay.h"
#include "gray_serial.h"
#include "motor.h"
#include "oled.h"

#define GRAY_OLED_TEST_PERIOD_MS    (100U)//刷新时间

static uint8_t gray_channel_left_to_right(uint8_t raw, uint8_t channel)
{
    static const uint8_t channelBitMap[8] = {
        1U, 2U, 3U, 4U, 5U, 6U, 7U, 0U
    };

    if (channel >= 8U) {
        return 0U;
    }

    return (uint8_t)((raw >> channelBitMap[channel]) & 0x01U);
}

static void oled_print_gray_four(uint8_t raw, uint8_t firstChannel)
{
    for (uint8_t i = 0U; i < 4U; i++) {
        uint8_t channel = (uint8_t)(firstChannel + i);

        oled_print_int((int32_t)(channel + 1U));
        oled_print_char(':');
        oled_print_int((int32_t)gray_channel_left_to_right(raw, channel));

        if (i != 3U) {
            oled_print_char(' ');
        }
    }
}

static void gray_oled_display(uint8_t raw)
{
    oled_clear_line(0U);
    oled_print_string("GRAY L->R TEST");

    oled_clear_line(1U);
    oled_print_string("RAW:");
    oled_print_hex_u8(raw);

    oled_clear_line(3U);
    oled_print_gray_four(raw, 0U);

    oled_clear_line(4U);
    oled_print_gray_four(raw, 4U);
}

int main(void)
{
    uint32_t lastUpdateMs;
    uint32_t nowMs;
    uint8_t raw = 0U;
    bool oledOk;

    SYSCFG_DL_init();

    gray_serial_init();
    motor_set_pwm(0, 0);
    oledOk = oled_init();

    lastUpdateMs = delay_get_ms();

    if (oledOk) {
        oled_clear();
        gray_oled_display(raw);
    }

    while (1) {
        nowMs = delay_get_ms();

        if ((uint32_t)(nowMs - lastUpdateMs) >=
            GRAY_OLED_TEST_PERIOD_MS) {
            lastUpdateMs = nowMs;
            raw = gray_serial_read();

            if (oledOk) {
                gray_oled_display(raw);
            }
        }
    }
}

void SysTick_Handler(void)
{
    delay_tick();
}
