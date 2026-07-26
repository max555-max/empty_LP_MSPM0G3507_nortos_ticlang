// #ifndef __ICM42688_H_
// #define __ICM42688_H_

// #include <stdbool.h>
// #include <stdint.h>

// /*
//  * ICM42688 六轴 IMU 硬件 I2C 驱动
//  *
//  * MSPM0G3507 引脚：
//  *   PA0 -> I2C0_SDA
//  *   PA1 -> I2C0_SCL
//  *
//  * I2C 外设和引脚由 SysConfig 配置。
//  *
//  * 本驱动只负责：
//  *   1. 自动识别 I2C 地址 0x68 / 0x69；
//  *   2. 初始化 ICM42688 寄存器；
//  *   3. 读取 WHO_AM_I；
//  *   4. 读取原始加速度、陀螺仪和温度数据。
//  *
//  * 姿态解算仍放在 attitude.c，不在这里完成。
//  */
// typedef struct {
//     /* 加速度计原始值，单位 LSB。 */
//     int16_t accelX;
//     int16_t accelY;
//     int16_t accelZ;

//     /* 陀螺仪原始值，单位 LSB。 */
//     int16_t gyroX;
//     int16_t gyroY;
//     int16_t gyroZ;

//     /* 温度原始值。 */
//     int16_t temp;

//     /* WHO_AM_I 读回值，ICM42688 正常应为 0x47。 */
//     uint8_t whoAmI;
// } icm42688_raw_t;

// /* 初始化 ICM42688，成功返回 true，失败返回 false。 */
// bool icm42688_init(void);

// /* 单独读取 WHO_AM_I，便于排查 I2C、地址和接线问题。 */
// uint8_t icm42688_get_who_am_i(void);

// /* 连续读取一帧原始 IMU 数据。 */
// void icm42688_read_raw(icm42688_raw_t *raw);

// /* 按串口格式打印原始数据。 */
// void icm42688_print_raw(const icm42688_raw_t *raw);

// #endif

#ifndef __ICM42688_H_
#define __ICM42688_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * ICM42688 六轴 IMU 硬件 I2C 驱动
 *
 * MSPM0G3507：
 *   PA0 -> I2C0_SDA
 *   PA1 -> I2C0_SCL
 *
 * SysConfig 中 I2C Controller 实例名称必须为：
 *
 *   I2C_ICM42688
 *
 * 初始化时自动尝试 ICM42688 七位地址 0x68 和 0x69。
 *
 *   WHO_AM_I          = 0x47
 *
 * 当前传感器配置：
 *
 *   accel：±16 g，1 kHz；
 *   gyro ：±2000 dps，1 kHz；
 *   accel/gyro：低噪声模式。
 */
typedef struct {
    /* 加速度计原始值，单位 LSB。 */
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;

    /* 陀螺仪原始值，单位 LSB。 */
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;

    /* 温度原始值。 */
    int16_t temp;

    /*
     * 本帧通信成功时为 0x47；
     * 通信失败时为 0xFF。
     */
    uint8_t whoAmI;
} icm42688_raw_t;

/*
 * 初始化 ICM42688。
 *
 * 成功返回 true，失败返回 false。
 */
bool icm42688_init(void);

/*
 * 单独读取 WHO_AM_I。
 *
 * 正常返回 0x47，通信失败返回 0xFF。
 */
uint8_t icm42688_get_who_am_i(void);

/*
 * 连续读取温度、加速度计和陀螺仪原始值。
 *
 * 成功：
 *   raw->whoAmI = 0x47
 *
 * 失败：
 *   六轴和温度清零；
 *   raw->whoAmI = 0xFF
 */
void icm42688_read_raw(icm42688_raw_t *raw);

/* 按串口文本格式打印原始数据。 */
void icm42688_print_raw(const icm42688_raw_t *raw);

#endif
