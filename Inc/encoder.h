#ifndef __ENCODER_H_
#define __ENCODER_H_

#include <stdint.h>
#include "ti_msp_dl_config.h"

/*
 * Quadrature encoder configuration
 *
 * Pin mapping:
 *   Left wheel : E2A/E2B = PA16/PA17
 *   Right wheel: E1A/E1B = PA14/PA15
 *
 * The encoders already have pull-up resistors, so SysConfig should keep the
 * GPIO input resistor setting as DL_GPIO_RESISTOR_NONE.
 */
#define ENCODER_LEFT_DIR                  (-1)
#define ENCODER_RIGHT_DIR                 (1)

/*
 * 13-line motor encoder, AB quadrature x4, gearbox ratio 1:20.409,
 * 48mm wheel diameter, speed updated every 10ms.
 */
#define ENCODER_LINES_PER_MOTOR_REV       (13)
#define ENCODER_QUADRATURE_MULTIPLIER     (4)
#define ENCODER_GEAR_RATIO_X1000          (20409)
#define ENCODER_WHEEL_DIAMETER_MM         (48)
#define ENCODER_SPEED_PERIOD_MS           (10)

void encoder_init(void);
void encoder_tick_1ms(void);

int32_t encoder_get_left_count(void);
int32_t encoder_get_right_count(void);

int32_t encoder_get_left_speed_mm_s(void);
int32_t encoder_get_right_speed_mm_s(void);

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
