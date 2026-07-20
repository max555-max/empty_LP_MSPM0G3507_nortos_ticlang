#ifndef __ICM42688_H_
#define __ICM42688_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * ICM42688 六轴 IMU 硬件 SPI 驱动
 *
 * SPI 引脚由 SysConfig 配置；
 * CS 使用普通 GPIO 手动拉低/拉高。
 *
 * 本驱动只负责：
 *   1. 初始化寄存器；
 *   2. 读取 WHO_AM_I；
 *   3. 读取原始加速度、陀螺仪、温度数据。
 *
 * 姿态解算放在 attitude.c，不在这里做。
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

    /* WHO_AM_I 读回值，ICM42688 正常应为 0x47。 */
    uint8_t whoAmI;
} icm42688_raw_t;

/* 初始化 ICM42688，成功返回 true，失败返回 false。 */
bool icm42688_init(void);

/* 单独读取 WHO_AM_I，便于排查 SPI/接线问题。 */
uint8_t icm42688_get_who_am_i(void);

/* 连续读取一帧原始 IMU 数据。 */
void icm42688_read_raw(icm42688_raw_t *raw);

/* 按串口格式打印原始数据，主要用于接线和寄存器配置调试。 */
void icm42688_print_raw(const icm42688_raw_t *raw);

#endif
