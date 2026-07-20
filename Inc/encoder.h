#ifndef __ENCODER_H_
#define __ENCODER_H_

#include <stdint.h>
#include "ti_msp_dl_config.h"

/*
 * AB 相编码器计数/测速模块
 *
 * 计数方式：
 *   GPIO 中断读取 A/B 两相；
 *   使用 AB 相四倍频，所以 A/B 任意边沿变化都会参与计数。
 *
 * 当前引脚映射：
 *   左轮：E2A/E2B = PA16/PA17
 *   右轮：E1A/E1B = PA14/PA15
 *
 * 注意：
 *   编码器模块自带上拉电阻，SysConfig 中 GPIO 不要再开内部上下拉。
 */

/* 左右编码器方向修正。某侧前进计数为负/正不符合预期时，只改这里。 */
#define ENCODER_LEFT_DIR                  (-1)
#define ENCODER_RIGHT_DIR                 (1)

/*
 * 编码器/轮子物理参数：
 *   电机编码器线数：13；
 *   AB 四倍频：x4；
 *   减速比：1:20.409；
 *   轮径：48mm；
 *   测速周期：10ms。
 */
#define ENCODER_LINES_PER_MOTOR_REV       (13)
#define ENCODER_QUADRATURE_MULTIPLIER     (4)
#define ENCODER_GEAR_RATIO_X1000          (20409)
#define ENCODER_WHEEL_DIAMETER_MM         (48)
#define ENCODER_SPEED_PERIOD_MS           (10)

/* 初始化编码器 GPIO 状态和内部计数变量。 */
void encoder_init(void);

/* 1ms 周期函数，内部累计到 10ms 后更新一次速度。 */
void encoder_tick_1ms(void);

/* 获取左右轮累计编码器计数。 */
int32_t encoder_get_left_count(void);
int32_t encoder_get_right_count(void);

/* 获取左右轮速度，单位 mm/s。 */
int32_t encoder_get_left_speed_mm_s(void);
int32_t encoder_get_right_speed_mm_s(void);

/* 清零左轮/右轮/两轮累计计数。 */
void encoder_reset_left_count(void);
void encoder_reset_right_count(void);
void encoder_reset_count(void);

/*
 * 兼容旧的单编码器测试代码。
 * 当前返回左轮计数。
 */
int32_t encoder_get_count(void);

/* GPIO 中断处理函数：在 GROUP1_IRQHandler 中调用。 */
void encoder_gpio_irq_handler(void);

#endif
