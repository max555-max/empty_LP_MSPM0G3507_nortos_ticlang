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
#include "angle_control.h"
#include "attitude.h"
#include "encoder.h"
#include "icm42688.h"
#include "pid.h"

#define ANGLE_CONTROL_PERIOD_MS       (10U)
#define STRAIGHT_BASE_SPEED_MM_S      (300)
#define IMU_GYRO_CALIB_SAMPLE_COUNT   (1000U)

int main(void)
{
    icm42688_raw_t raw;
    bool imuOk;
    uint32_t lastUpdateMs;
    uint32_t nowMs;
    float dt;

    SYSCFG_DL_init();
    encoder_init();
    speed_pid_init();
    angle_control_init();
    attitude_init();
    imuOk = icm42688_init();

    if (imuOk) {
        /*
         * Keep the board still during calibration.
         * Wait for the sensor output to settle first.
         * 1000 samples * 2ms = about 2s.
         */
        delay_ms(200U);
        attitude_calibrate_gyro(IMU_GYRO_CALIB_SAMPLE_COUNT);

        /*
         * ICM42688 and attitude zero point are ready now.
         * Wait 1s before giving the car speed, then lock current yaw as the
         * straight-line heading target.
         */
        delay_ms(1000U);
        angle_control_set_base_speed(STRAIGHT_BASE_SPEED_MM_S);
        angle_control_lock_current_yaw();
        angle_control_enable(true);
    } else {
        speed_pid_stop();
    }

    lastUpdateMs = delay_get_ms();

    while (1) {
        icm42688_read_raw(&raw);
        nowMs = delay_get_ms();
        dt = (float) (nowMs - lastUpdateMs) / 1000.0f;
        lastUpdateMs = nowMs;

        if (imuOk) {
            attitude_update_from_icm42688(&raw, dt);
            angle_control_update(dt);
        } else {
            speed_pid_stop();
        }

        speed_pid_control_update();
        delay_ms(ANGLE_CONTROL_PERIOD_MS);
    }
}

/*
 * SysTick 1ms中断
 */
void SysTick_Handler(void)
{
    delay_tick();
    encoder_tick_1ms();
}
