/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */

#include "ti_msp_dl_config.h"

#include "delay.h"
#include "encoder.h"
#include "gray_serial.h"
#include "oled.h"
#include "pid.h"
#include "square_track.h"

#include <stdint.h>

#define SQUARE_MAIN_PERIOD_MS       (SPEED_PID_CONTROL_PERIOD_MS)
#define SQUARE_MAIN_OLED_PERIOD_MS  (100U)

static void square_main_update_oled(void);

int main(void)
{
    uint32_t lastOledMs;
    uint32_t nowMs;

    SYSCFG_DL_init();

    /*
     * 正方形循迹依赖顺序：
     * 灰度传感器 -> 正方形状态机 -> 速度环 -> 电机 PWM。
     */
    gray_serial_init();
    encoder_init();
    speed_pid_init();
    square_track_init();
    (void)oled_init();

    lastOledMs = delay_get_ms();

    while (1) {
        /*
         * 主循环调度顺序不要随便调换：
         * 1. square_track_update() 先判断状态并写入左右轮目标速度；
         * 2. speed_pid_control_update() 再根据目标速度输出最终 PWM。
         */
        square_track_update();
        speed_pid_control_update();

        nowMs = delay_get_ms();
        if ((uint32_t)(nowMs - lastOledMs) >=
            SQUARE_MAIN_OLED_PERIOD_MS) {

            lastOledMs = nowMs;
            square_main_update_oled();
        }

        delay_ms(SQUARE_MAIN_PERIOD_MS);
    }
}

void SysTick_Handler(void)
{
    /* 1 ms 时间基准：给 delay_get_ms() 和编码器测速使用。 */
    delay_tick();
    encoder_tick_1ms();
}

static void square_main_update_oled(void)
{
    square_track_status_t status;

    square_track_get_status(&status);

    oled_clear_line(0U);
    oled_print_string("SQUARE TRACE");

    oled_clear_line(1U);
    oled_print_string("SEG:");
    oled_print_char(square_track_get_segment_start_label());
    oled_print_string("->");
    oled_print_char(square_track_get_segment_end_label());

    oled_clear_line(2U);
    oled_print_string("ST:");
    oled_print_int((int32_t)status.state);
    oled_print_string(" ACT:");
    oled_print_int(status.activeCount);

    oled_clear_line(3U);
    oled_print_string("RAW:");
    oled_print_hex_u8(status.sensorRaw);
    oled_print_string(" MID:");
    oled_print_int(status.centerDetected);

    oled_clear_line(4U);
    oled_print_string("D:");
    oled_print_int(status.advanceDistanceMm);
    oled_print_string("mm");

    oled_clear_line(5U);
    oled_print_string("L:");
    oled_print_int(status.leftTargetMmS);

    oled_clear_line(6U);
    oled_print_string("R:");
    oled_print_int(status.rightTargetMmS);

    oled_clear_line(7U);
    /* 状态说明：0=循迹，1=丢线后前进，2=原地右转。 */
    oled_print_string("0TR 1FWD 2RIGHT");
}
