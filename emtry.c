/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */

#include "ti_msp_dl_config.h"

#include "delay.h"
#include "attitude.h"
#include "icm42688.h"
#include "vofa.h"

/* 主循环执行周期。 */
#define IMU_UPDATE_PERIOD_MS             (10U)

/*
 * 陀螺仪零偏校准采样次数。
 *
 * 调试阶段先使用 300 次，明显缩短复位后的等待时间。
 * 后续需要更稳定的零偏时，可以改成 500 或 1000。
 */
#define IMU_GYRO_CALIB_SAMPLE_COUNT      (300U)

/*
 * ICM42688 初始化完成后，额外等待传感器输出稳定。
 *
 * 新版 icm42688_init() 内部已经包含启动等待，
 * 因此这里保留 50 ms 即可。
 */
#define IMU_SENSOR_STABLE_DELAY_MS       (50U)

/*
 * 当前 ICM42688 驱动配置为 ±2000 dps。
 *
 * 原始值转换为 deg/s：
 *
 * gyroDps = raw * 2000 / 32768
 */
#define IMU_GYRO_DPS_PER_LSB             (2000.0f / 32768.0f)

/* 初始化失败或运行中掉线后的重新尝试周期。 */
#define IMU_RETRY_PERIOD_MS              (1000U)

/* ICM42688 正确的 WHO_AM_I。 */
#define ICM42688_EXPECTED_WHO_AM_I       (0x47U)

/* 姿态解算允许的 dt 范围。 */
#define IMU_DT_DEFAULT_S                 (0.010f)
#define IMU_DT_MIN_S                     (0.001f)
#define IMU_DT_MAX_S                     (0.050f)

/*
 * 以下变量可以在 CCS 的 Expressions 窗口中查看。
 *
 * g_imuInitTimeMs：
 *     icm42688_init() 实际耗时。
 *
 * g_gyroCalibTimeMs：
 *     attitude_calibrate_gyro() 实际耗时。
 *
 * g_imuRetryCount：
 *     运行中重新初始化次数。
 */
volatile uint32_t g_imuInitTimeMs = 0U;
volatile uint32_t g_gyroCalibTimeMs = 0U;
volatile uint32_t g_imuRetryCount = 0U;

/*
 * 初始化并校准 ICM42688。
 *
 * 返回：
 *   true：初始化和陀螺仪校准均成功；
 *   false：初始化失败或校准未完成。
 */
static bool imu_initialize_and_calibrate(void)
{
    uint32_t startMs;
    bool result;

    /*
     * 每次重新初始化传感器前，都清空姿态解算器状态。
     */
    attitude_init();

    /*
     * 测量 ICM42688 初始化耗时。
     */
    startMs = delay_get_ms();

    result = icm42688_init();

    g_imuInitTimeMs =
        (uint32_t)(delay_get_ms() - startMs);

    if (result == false) {
        return false;
    }

    /*
     * 等待加速度计和陀螺仪输出进一步稳定。
     */
    delay_ms(IMU_SENSOR_STABLE_DELAY_MS);

    /*
     * 测量陀螺仪零偏校准耗时。
     *
     * 校准期间小车和 ICM42688 必须保持静止。
     */
    startMs = delay_get_ms();

    attitude_calibrate_gyro(
        IMU_GYRO_CALIB_SAMPLE_COUNT);

    g_gyroCalibTimeMs =
        (uint32_t)(delay_get_ms() - startMs);

    /*
     * 校准函数可能因为 I2C 数据无效而未完成，
     * 因此必须检查校准状态。
     */
    if (attitude_is_gyro_calibrated() == false) {
        return false;
    }

    return true;
}

int main(void)
{
    icm42688_raw_t raw = {0};
    attitude_euler_t euler = {0};

    bool imuOk;

    uint32_t lastUpdateMs;
    uint32_t lastRetryMs;
    uint32_t nowMs;

    float dt;

    /*
     * 初始化失败时，保证 WHO_AM_I 显示为 0xFF，
     * 而不是结构体默认的 0。
     */
    raw.whoAmI = 0xFFU;

    /*
     * 初始化 MCU 外设。
     *
     * 硬件 I2C0、PA0、PA1、UART 和 SysTick
     * 都由 SysConfig 初始化。
     */
    SYSCFG_DL_init();

    /*
     * 初始化并校准 ICM42688。
     */
    imuOk = imu_initialize_and_calibrate();

    /*
     * 初始化失败时保持明确的错误标志。
     */
    if (imuOk == false) {
        raw.whoAmI = 0xFFU;
    }

    lastUpdateMs = delay_get_ms();
    lastRetryMs = lastUpdateMs;

    while (1) {
        nowMs = delay_get_ms();

        /*
         * 初始化失败或运行中掉线后，
         * 每隔 1 秒重新初始化一次。
         */
        if (imuOk == false) {
            if ((uint32_t)(nowMs - lastRetryMs) >=
                IMU_RETRY_PERIOD_MS) {

                lastRetryMs = nowMs;
                g_imuRetryCount++;

                imuOk =
                    imu_initialize_and_calibrate();

                if (imuOk) {
                    /*
                     * 校准过程耗时较长。
                     * 重新设置姿态积分的起始时间，
                     * 避免产生异常大的 dt。
                     */
                    lastUpdateMs = delay_get_ms();
                } else {
                    raw.whoAmI = 0xFFU;
                }
            }
        }

        if (imuOk) {
            /*
             * 读取温度、加速度计和陀螺仪原始数据。
             */
            icm42688_read_raw(&raw);

            /*
             * 新版驱动：
             *
             * 读取成功：
             *     raw.whoAmI = 0x47
             *
             * 读取失败：
             *     raw.whoAmI = 0xFF
             *     六轴数据清零
             */
            if (raw.whoAmI !=
                ICM42688_EXPECTED_WHO_AM_I) {

                imuOk = false;

                raw.accelX = 0;
                raw.accelY = 0;
                raw.accelZ = 0;

                raw.gyroX = 0;
                raw.gyroY = 0;
                raw.gyroZ = 0;

                raw.temp = 0;
                raw.whoAmI = 0xFFU;
            }
        }

        nowMs = delay_get_ms();

        /*
         * 根据实际执行间隔计算姿态更新时间。
         */
        dt =
            (float)(nowMs - lastUpdateMs) /
            1000.0f;

        lastUpdateMs = nowMs;

        /*
         * 防止首次运行、重新校准或程序阻塞导致
         * dt 超出合理范围。
         */
        if ((dt < IMU_DT_MIN_S) ||
            (dt > IMU_DT_MAX_S)) {

            dt = IMU_DT_DEFAULT_S;
        }

        if (imuOk) {
            /*
             * 使用有效数据更新姿态。
             */
            if (attitude_update_from_icm42688(
                    &raw,
                    dt)) {

                attitude_get_euler(&euler);

                /*
                 * VOFA+ FireWater：
                 *
                 * ch0：roll，单位 deg
                 * ch1：pitch，单位 deg
                 * ch2：yaw，单位 deg
                 * ch3：原始 gyroX 换算值，单位 deg/s
                 * ch4：原始 gyroY 换算值，单位 deg/s
                 * ch5：姿态模块处理后的 gyroZ，单位 deg/s
                 *
                 * ch5 使用 attitude_get_gyro_z_dps()，
                 * 它已经完成零偏扣除、比例换算和死区处理。
                 */
                vofa_send_six_float(
                    euler.roll,
                    euler.pitch,
                    euler.yaw,
                    (float)raw.gyroX *
                        IMU_GYRO_DPS_PER_LSB,
                    (float)raw.gyroY *
                        IMU_GYRO_DPS_PER_LSB,
                    attitude_get_gyro_z_dps(),
                    2U);
            }
        } else {
            /*
             * 通信失败时：
             *
             * ch0：0
             * ch1：0
             * ch2：0
             * ch3：ICM42688初始化耗时，单位 ms
             * ch4：陀螺仪校准耗时，单位 ms
             * ch5：WHO_AM_I，失败通常为 255
             */
            vofa_send_six_float(
                0.0f,
                0.0f,
                0.0f,
                (float)g_imuInitTimeMs,
                (float)g_gyroCalibTimeMs,
                (float)raw.whoAmI,
                2U);
        }

        delay_ms(IMU_UPDATE_PERIOD_MS);
    }
}

/*
 * SysTick 1 ms 中断。
 */
void SysTick_Handler(void)
{
    delay_tick();
}