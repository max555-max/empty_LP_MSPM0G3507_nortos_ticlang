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
#include "attitude.h"
#include "icm42688.h"
#include "vofa.h"

#define IMU_DEBUG_PERIOD_MS           (10U)
#define IMU_GYRO_CALIB_SAMPLE_COUNT   (1000U)
#define IMU_GYRO_DPS_PER_LSB          (2000.0f / 32768.0f)

int main(void)
{
    icm42688_raw_t raw;
    attitude_euler_t euler;
    bool imuOk;
    uint32_t lastUpdateMs;
    uint32_t nowMs;
    float dt;

    SYSCFG_DL_init();
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
    }

    lastUpdateMs = delay_get_ms();

    while (1) {
        icm42688_read_raw(&raw);
        nowMs = delay_get_ms();
        dt = (float) (nowMs - lastUpdateMs) / 1000.0f;
        lastUpdateMs = nowMs;

        if (imuOk) {
            attitude_update_from_icm42688(&raw, dt);
            attitude_get_euler(&euler);

            /*
             * FireWater:
             * ch0 roll, ch1 pitch, ch2 yaw,
             * ch3 gyroX dps, ch4 gyroY dps, ch5 gyroZ dps.
             */
            vofa_send_six_float(euler.roll,
                                euler.pitch,
                                euler.yaw,
                                (float) raw.gyroX * IMU_GYRO_DPS_PER_LSB,
                                (float) raw.gyroY * IMU_GYRO_DPS_PER_LSB,
                                (float) raw.gyroZ * IMU_GYRO_DPS_PER_LSB,
                                2U);
        } else {
            /*
             * If communication fails, ch5 shows WHO_AM_I for diagnosis.
             */
            vofa_send_six_float(0.0f,
                                0.0f,
                                0.0f,
                                0.0f,
                                0.0f,
                                (float) raw.whoAmI,
                                2U);
        }

        delay_ms(IMU_DEBUG_PERIOD_MS);
    }
}

/*
 * SysTick 1ms中断
 */
void SysTick_Handler(void)
{
    delay_tick();
}
