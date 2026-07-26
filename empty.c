/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */

#include "ti_msp_dl_config.h"

#include "attitude.h"
#include "delay.h"
#include "icm42688.h"
#include "mpu6050.h"
#include "vofa.h"

#include <stdbool.h>
#include <stdint.h>

#define MPU6050_UPDATE_PERIOD_MS         (10U)
#define MPU6050_RETRY_PERIOD_MS          (1000U)
#define MPU6050_GYRO_CALIB_SAMPLE_COUNT  (200U)

/*
 * Existing attitude.c expects ICM42688 raw accel at +/-16 g:
 *   2048 LSB/g.
 *
 * This MPU6050 test config uses +/-2 g:
 *   16384 LSB/g.
 *
 * Divide MPU6050 accel raw values by 8 before feeding the ICM attitude
 * algorithm. Gyro is configured as +/-2000 dps on both paths and can be passed
 * through with the existing scale.
 */
#define MPU6050_TO_ICM_ACCEL_DIVIDER      (8)

static void mpu6050_to_attitude_raw(
    const mpu6050_raw_t *mpu,
    icm42688_raw_t *attRaw);

int main(void)
{
    mpu6050_raw_t mpuRaw = {0};
    mpu6050_diag_t diag = {0};
    icm42688_raw_t attitudeRaw = {0};
    attitude_euler_t euler = {0};

    bool mpuOk;
    bool attitudeUpdated = false;
    uint32_t lastRetryMs;
    uint32_t lastUpdateMs;
    uint32_t nowMs;
    uint16_t gyroCalibProgress = 0U;
    float dt;

    SYSCFG_DL_init();

    attitude_init();
    mpuOk = mpu6050_init();
    lastRetryMs = delay_get_ms();
    lastUpdateMs = lastRetryMs;

    while (1) {
        nowMs = delay_get_ms();

        if (mpuOk == false) {
            if ((uint32_t)(nowMs - lastRetryMs) >=
                MPU6050_RETRY_PERIOD_MS) {

                lastRetryMs = nowMs;
                attitude_init();
                gyroCalibProgress = 0U;
                mpuOk = mpu6050_init();
                lastUpdateMs = delay_get_ms();
            }
        }

        if (mpuOk) {
            mpuOk = mpu6050_read_raw(&mpuRaw);
        }

        if (mpuOk) {
            mpu6050_to_attitude_raw(&mpuRaw, &attitudeRaw);

            if (!attitude_is_gyro_calibrated()) {
                attitudeUpdated =
                    attitude_calibrate_gyro_step(
                        &attitudeRaw,
                        MPU6050_GYRO_CALIB_SAMPLE_COUNT);

                if (gyroCalibProgress <
                    MPU6050_GYRO_CALIB_SAMPLE_COUNT) {
                    gyroCalibProgress++;
                }

                lastUpdateMs = delay_get_ms();
            } else {
                nowMs = delay_get_ms();
                dt = (float)(nowMs - lastUpdateMs) / 1000.0f;
                lastUpdateMs = nowMs;

                attitudeUpdated =
                    attitude_update_from_icm42688(
                        &attitudeRaw,
                        dt);
            }
        } else {
            attitudeUpdated = false;
        }

        if (mpuOk && attitudeUpdated && attitude_is_gyro_calibrated()) {
            attitude_get_euler(&euler);

            /*
             * VOFA channels:
             *   ch0 roll, ch1 pitch, ch2 yaw,
             *   ch3 gyroZ dps, ch4 accelZ g, ch5 WHO_AM_I.
             */
            vofa_send_six_float(
                euler.roll,
                euler.pitch,
                euler.yaw,
                attitude_get_gyro_z_dps(),
                (float)mpuRaw.accelZ * MPU6050_ACCEL_G_PER_LSB,
                (float)mpuRaw.whoAmI,
                2U);
        } else {
            mpu6050_get_diag(&diag);

            /*
             * Failure/calibration diagnostics:
             *   ch0 current accepted address, 255 means not detected yet
             *   ch1 MPU initialized flag
             *   ch2 gyro calibration sample progress
             *   ch3 last attempted address, 104 means 0x68, 105 means 0x69
             *   ch4 error stage
             *   ch5 WHO_AM_I when read succeeds, otherwise I2C status low byte
             */
            vofa_send_six_float(
                (float)diag.currentAddress,
                diag.initialized ? 1.0f : 0.0f,
                attitude_is_gyro_calibrated() ?
                    (float)MPU6050_GYRO_CALIB_SAMPLE_COUNT :
                    (float)gyroCalibProgress,
                (float)diag.lastAddress,
                (float)diag.lastError,
                diag.lastWhoReadOk ?
                    (float)diag.lastWhoAmI :
                    (float)(diag.lastStatus & 0xFFU),
                0U);
        }

        delay_ms(MPU6050_UPDATE_PERIOD_MS);
    }
}

void SysTick_Handler(void)
{
    delay_tick();
}

static void mpu6050_to_attitude_raw(
    const mpu6050_raw_t *mpu,
    icm42688_raw_t *attRaw)
{
    if ((mpu == 0) ||
        (attRaw == 0)) {
        return;
    }

    attRaw->accelX =
        (int16_t)(mpu->accelX / MPU6050_TO_ICM_ACCEL_DIVIDER);
    attRaw->accelY =
        (int16_t)(mpu->accelY / MPU6050_TO_ICM_ACCEL_DIVIDER);
    attRaw->accelZ =
        (int16_t)(mpu->accelZ / MPU6050_TO_ICM_ACCEL_DIVIDER);

    attRaw->gyroX = mpu->gyroX;
    attRaw->gyroY = mpu->gyroY;
    attRaw->gyroZ = mpu->gyroZ;
    attRaw->temp = mpu->temp;

    attRaw->whoAmI =
        (mpu->whoAmI == MPU6050_WHO_AM_I_VALUE) ? 0x47U : 0xFFU;
}
