#include "motor.h"

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

void motor_set_pwm(int pwmL,int pwmR)
{
    pwmL = motor_limit_pwm(pwmL);
    pwmR = motor_limit_pwm(pwmR);

	 if(pwmL>0)//front
    {
        DL_GPIO_setPins(AIN_PORT, AIN_AIN1_PIN);
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN2_PIN);
		DL_Timer_setCaptureCompareValue(PWM_INST,pwmL,GPIO_PWM_C0_IDX);
    }
    else if(pwmL<0)//back
    {
        DL_GPIO_setPins(AIN_PORT,AIN_AIN2_PIN);
        DL_GPIO_clearPins(AIN_PORT,AIN_AIN1_PIN);
		DL_Timer_setCaptureCompareValue(PWM_INST,-pwmL,GPIO_PWM_C0_IDX);
    }
    else 
    {
        DL_Timer_setCaptureCompareValue(PWM_INST,0,GPIO_PWM_C0_IDX);
    }

    if(pwmR>0)
    {
		    DL_GPIO_setPins(BIN_PORT,BIN_BIN1_PIN);
        DL_GPIO_clearPins(BIN_PORT,BIN_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_INST,pwmR,GPIO_PWM_C1_IDX);
    }
    else if(pwmR<0)
    {
		    DL_GPIO_setPins(BIN_PORT,BIN_BIN2_PIN);
        DL_GPIO_clearPins(BIN_PORT,BIN_BIN1_PIN);
		    DL_Timer_setCaptureCompareValue(PWM_INST,-pwmR,GPIO_PWM_C1_IDX);
    }
    else 
    {
        DL_Timer_setCaptureCompareValue(PWM_INST,0,GPIO_PWM_C1_IDX);
    }
}
