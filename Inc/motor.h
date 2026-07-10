#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "ti_msp_dl_config.h"
#include <stdlib.h>

#define MOTOR_PWM_MAX       (1000)
#define MOTOR_PWM_MIN       (-1000)

void Set_PWM(int pwmL,int pwmR);
#endif
