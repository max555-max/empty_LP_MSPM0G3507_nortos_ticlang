#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "ti_msp_dl_config.h"

#define MOTOR_PWM_MAX       (4000)
#define MOTOR_PWM_MIN       (-4000)

void motor_set_pwm(int pwmL,int pwmR);
#endif
