/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */

#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

#include "bluetooth.h"
#include "delay.h"
#include "encoder.h"
#include "gray_serial.h"
#include "line_track.h"
#include "oled.h"
#include "pid.h"

#define LINE_TUNE_RUN_ENABLE          (1U)
#define LINE_TUNE_CONTROL_PERIOD_MS   (10U)
#define LINE_TUNE_OLED_PERIOD_MS      (200U)

static void line_tune_show_oled(uint8_t raw, bool oledOk)
{
    line_track_status_t status;

    if (!oledOk) {
        return;
    }

    line_track_get_status(&status);

    oled_clear_line(0U);
    oled_print_string("LINE PID BT RUN");

    oled_clear_line(1U);
    oled_print_string("KP:");
    oled_print_int(line_track_get_turn_kp());
    oled_print_string(" KD:");
    oled_print_int(line_track_get_turn_kd());

    oled_clear_line(2U);
    oled_print_string("BS:");
    oled_print_int(line_track_get_base_speed());
    oled_print_string(" MX:");
    oled_print_int(line_track_get_max_correction());

    oled_clear_line(3U);
    oled_print_string("ERR:");
    oled_print_int(status.error);
    oled_print_string(" C:");
    oled_print_int(status.correction);

    oled_clear_line(4U);
    oled_print_string("RAW:");
    oled_print_hex_u8(raw);
    oled_print_string(" D:");
    oled_print_int(status.lineDetected ? 1 : 0);

    oled_clear_line(5U);
    oled_print_string("L:");
    oled_print_int(status.leftTargetMmS);
    oled_print_string(" R:");
    oled_print_int(status.rightTargetMmS);
}

int main(void)
{
    uint32_t lastControlMs;
    uint32_t lastOledMs;
    uint8_t raw = 0U;
    bool oledOk;

    SYSCFG_DL_init();

    gray_serial_init();
    encoder_init();
    speed_pid_init();
    line_track_init();
    bluetooth_init();
    oledOk = oled_init();

    bluetooth_send_params();

    lastControlMs = delay_get_ms();
    lastOledMs = lastControlMs;

    if (oledOk) {
        oled_clear();
        line_tune_show_oled(raw, oledOk);
    }

    while (1) {
        uint32_t nowMs = delay_get_ms();

        bluetooth_process();

        if ((uint32_t)(nowMs - lastControlMs) >=
            LINE_TUNE_CONTROL_PERIOD_MS) {
            lastControlMs = nowMs;
            raw = gray_serial_read();

#if LINE_TUNE_RUN_ENABLE
            line_track_update_with_raw(raw);
#else
            speed_pid_stop();
#endif
            speed_pid_control_update();
        }

        if ((uint32_t)(nowMs - lastOledMs) >= LINE_TUNE_OLED_PERIOD_MS) {
            lastOledMs = nowMs;
            line_tune_show_oled(raw, oledOk);
        }
    }
}

void SysTick_Handler(void)
{
    delay_tick();
    encoder_tick_1ms();
}
