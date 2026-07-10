#ifndef __ENCODER_H_
#define __ENCODER_H_

#include <stdint.h>
#include "ti_msp_dl_config.h"

/*
 * Quadrature encoder configuration
 *
 * Existing pin mapping:
 *   Left wheel : E2A/E2B = PA16/PA17
 *   Right wheel: E1A/E1B = PA14/PA15
 *
 * The encoders already have pull-up resistors, so SysConfig should keep the
 * GPIO input resistor setting as DL_GPIO_RESISTOR_NONE.
 *
 * If the count direction is opposite to the wheel's forward direction, change
 * the corresponding direction macro from 1 to -1.
 */
#define ENCODER_LEFT_DIR      (-1)
#define ENCODER_RIGHT_DIR     (1)

void encoder_init(void);

int32_t encoder_get_left_count(void);
int32_t encoder_get_right_count(void);

void encoder_reset_left_count(void);
void encoder_reset_right_count(void);
void encoder_reset_count(void);

/*
 * Compatibility helper for older single-encoder test code.
 * It returns the left wheel encoder count.
 */
int32_t encoder_get_count(void);

void encoder_gpio_irq_handler(void);

#endif
