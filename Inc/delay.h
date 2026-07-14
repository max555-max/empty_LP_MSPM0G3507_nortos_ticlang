#ifndef __DELAY_H_
#define __DELAY_H_

#include "ti_msp_dl_config.h"
#include <stdint.h>

void delay_ms(uint32_t ms);
void delay_tick(void);
uint32_t delay_get_ms(void);

#endif
