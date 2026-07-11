#ifndef __ICM42688_H_
#define __ICM42688_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
    int16_t temp;
    uint8_t whoAmI;
} icm42688_raw_t;

bool icm42688_init(void);
uint8_t icm42688_get_who_am_i(void);
void icm42688_read_raw(icm42688_raw_t *raw);
void icm42688_print_raw(const icm42688_raw_t *raw);

#endif
