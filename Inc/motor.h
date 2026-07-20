#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "ti_msp_dl_config.h"

/*
 * 电机 PWM 底层输出模块
 *
 * 上层速度环/循迹/角度环只需要调用 motor_set_pwm()，
 * 不直接关心具体 PWM 通道和方向 GPIO。
 *
 * pwmL/pwmR：
 *   正数：一个方向转动；
 *   负数：反方向转动；
 *   0   ：停止输出。
 *
 * 绝对值会被限制到 MOTOR_PWM_MAX。
 */
#define MOTOR_PWM_MAX       (4000)
#define MOTOR_PWM_MIN       (-4000)

/* 设置左右电机 PWM。参数单位是 PWM 计数值，不是速度。 */
void motor_set_pwm(int pwmL,int pwmR);
#endif
