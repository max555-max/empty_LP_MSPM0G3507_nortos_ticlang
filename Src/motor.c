#include "motor.h"

/*
 * motor.c
 *
 * 双电机底层 PWM 输出。
 *
 * 上层传入 signed PWM：
 *   pwm > 0：前进方向；
 *   pwm < 0：后退方向；
 *   pwm = 0：停止。
 *
 * 注意：
 *   本文件只处理 PWM 和方向 GPIO，不做速度闭环。
 *   速度闭环在 pid.c 中完成。
 */

/* 将 PWM 限制到电机允许范围，保护定时器和驱动。 */
static int motor_limit_pwm(int pwm)
{
    if (pwm > MOTOR_PWM_MAX) {
        return MOTOR_PWM_MAX;
    }

    if (pwm < MOTOR_PWM_MIN) {
        return MOTOR_PWM_MIN;
    }

    return pwm;
}

/*
 * 设置左右电机 PWM。
 *
 * 参数：
 *   pwmL：左电机 signed PWM；
 *   pwmR：右电机 signed PWM。
 */
void motor_set_pwm(int pwmL,int pwmR)
{
    /* 先限幅，防止 PID 或上层控制输出超过 4000。 */
    pwmL = motor_limit_pwm(pwmL);
    pwmR = motor_limit_pwm(pwmR);

    if(pwmL>0)//front
    {
        /* 左电机前进方向。 */
        DL_GPIO_setPins(AIN_PORT, AIN_AIN1_PIN);
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_INST,pwmL,GPIO_PWM_C0_IDX);
    }
    else if(pwmL<0)//back
    {
        /* 左电机后退方向，PWM 取绝对值。 */
        DL_GPIO_setPins(AIN_PORT,AIN_AIN2_PIN);
        DL_GPIO_clearPins(AIN_PORT,AIN_AIN1_PIN);
        DL_Timer_setCaptureCompareValue(PWM_INST,-pwmL,GPIO_PWM_C0_IDX);
    }
    else
    {
        /* 左电机停止。 */
        DL_Timer_setCaptureCompareValue(PWM_INST,0,GPIO_PWM_C0_IDX);
    }

    if(pwmR>0)
    {
        /* 右电机前进方向。 */
        DL_GPIO_setPins(BIN_PORT,BIN_BIN1_PIN);
        DL_GPIO_clearPins(BIN_PORT,BIN_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_INST,pwmR,GPIO_PWM_C1_IDX);
    }
    else if(pwmR<0)
    {
        /* 右电机后退方向，PWM 取绝对值。 */
        DL_GPIO_setPins(BIN_PORT,BIN_BIN2_PIN);
        DL_GPIO_clearPins(BIN_PORT,BIN_BIN1_PIN);
        DL_Timer_setCaptureCompareValue(PWM_INST,-pwmR,GPIO_PWM_C1_IDX);
    }
    else
    {
        /* 右电机停止。 */
        DL_Timer_setCaptureCompareValue(PWM_INST,0,GPIO_PWM_C1_IDX);
    }
}
