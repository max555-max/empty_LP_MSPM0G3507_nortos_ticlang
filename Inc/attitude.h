#ifndef __ATTITUDE_H_
#define __ATTITUDE_H_

#include <stdbool.h>
#include <stdint.h>

#include "icm42688.h"

/*
 * 姿态解算模块
 *
 * 输入：
 *   ICM42688 的原始加速度计/陀螺仪数据。
 *
 * 输出：
 *   roll  ：横滚角，单位 deg。
 *   pitch ：俯仰角，单位 deg。
 *   yaw   ：航向角，单位 deg。当前 yaw 主要由 Z 轴陀螺积分得到，
 *           上电校准完成后从 0 度开始累计。
 *
 * 使用顺序：
 *   attitude_init();
 *   attitude_calibrate_gyro(...); 或 attitude_calibrate_gyro_step(...);
 *   周期调用 attitude_update_from_icm42688(...);
 *   需要查看角度时调用 attitude_get_euler(...);
 */
typedef struct {
    /* 横滚角：车体左右倾斜角。 */
    float roll;

    /* 俯仰角：车体前后抬头/低头角。 */
    float pitch;

    /* 航向角：小车在水平面内旋转的角度。 */
    float yaw;
} attitude_euler_t;

/* 清空姿态状态、滤波状态和陀螺仪零偏状态。 */
void attitude_init(void);

/* 阻塞式静止校准：采集 sampleCount 个样本计算陀螺零偏。 */
void attitude_calibrate_gyro(uint16_t sampleCount);

/* 非阻塞式校准：主循环中反复调用，返回 true 表示校准完成。 */
bool attitude_calibrate_gyro_step(const icm42688_raw_t *raw, uint16_t sampleCount);

/* 查询陀螺仪零偏是否已经校准完成。 */
bool attitude_is_gyro_calibrated(void);

/* 用一帧 ICM42688 原始数据更新姿态，dt 单位为秒。 */
bool attitude_update_from_icm42688(const icm42688_raw_t *raw, float dt);

/* 读取当前欧拉角。 */
void attitude_get_euler(attitude_euler_t *euler);

float attitude_get_gyro_z_dps(void);

/* 按 VOFA FireWater 格式打印 roll/pitch/yaw 等调试数据。 */
void attitude_print_euler(const attitude_euler_t *euler);

#endif
