#ifndef __ATTITUDE_H_
#define __ATTITUDE_H_

#include <stdbool.h>
#include <stdint.h>

#include "icm42688.h"

/* 欧拉角，单位均为度。 */
typedef struct {
    float roll;
    float pitch;
    float yaw;
} attitude_euler_t;

/* 清空姿态、滤波器和校准状态。 */
void attitude_init(void);

/*
 * 阻塞式陀螺仪零偏校准。
 * 校准期间传感器必须保持静止。
 */
void attitude_calibrate_gyro(uint16_t sampleCount);

/*
 * 非阻塞式单步校准。
 * 每传入一帧有效数据累计一次，完成后返回 true。
 */
bool attitude_calibrate_gyro_step(
    const icm42688_raw_t *raw,
    uint16_t sampleCount);

/* 查询陀螺仪零偏校准是否完成。 */
bool attitude_is_gyro_calibrated(void);

/*
 * 使用一帧 ICM42688 原始数据更新姿态。
 *
 * dt：本次与上次更新之间的时间，单位秒。
 * 返回 true 表示本帧有效并完成更新。
 */
bool attitude_update_from_icm42688(
    const icm42688_raw_t *raw,
    float dt);

/* 读取当前 roll、pitch、yaw。 */
void attitude_get_euler(attitude_euler_t *euler);

/*
 * 获取当前经过零偏补偿、量程换算、Z轴比例修正和死区处理后的
 * Z轴角速度，单位 deg/s。
 *
 * 角度环的 D 项直接使用该函数。
 */
float attitude_get_gyro_z_dps(void);

/* 串口打印欧拉角，主要用于调试。 */
void attitude_print_euler(const attitude_euler_t *euler);

#endif