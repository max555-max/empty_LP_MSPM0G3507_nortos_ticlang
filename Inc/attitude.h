#ifndef __ATTITUDE_H_
#define __ATTITUDE_H_

#include <stdbool.h>
#include <stdint.h>

#include "icm42688.h"

typedef struct {
    float roll;
    float pitch;
    float yaw;
} attitude_euler_t;

void attitude_init(void);
bool attitude_update_from_icm42688(const icm42688_raw_t *raw, float dt);
void attitude_get_euler(attitude_euler_t *euler);
void attitude_print_euler(const attitude_euler_t *euler);

#endif
