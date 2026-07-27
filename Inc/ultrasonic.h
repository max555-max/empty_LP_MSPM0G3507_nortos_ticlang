#ifndef __ULTRASONIC_H_
#define __ULTRASONIC_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Ultrasonic distance module.
 *
 * Default software pin use:
 *   PA24 -> TRIG output
 *   PA9  -> ECHO input
 *
 * These pins are initialized locally by ultrasonic_init(); they are not
 * generated SysConfig macros in the current project. Physical wiring still
 * needs hardware confirmation.
 */

#define ULTRASONIC_MAX_DISTANCE_MM       (4000U)
#define ULTRASONIC_NO_OBJECT_MM          (0xFFFFU)

typedef enum {
    ULTRASONIC_STATUS_OK = 0,
    ULTRASONIC_STATUS_NULL_POINTER,
    ULTRASONIC_STATUS_ECHO_START_TIMEOUT,
    ULTRASONIC_STATUS_ECHO_END_TIMEOUT
} ultrasonic_status_t;

typedef struct {
    ultrasonic_status_t status;
    uint16_t distanceMm;
    uint32_t echoTimeUs;
} ultrasonic_measurement_t;

void ultrasonic_init(void);
bool ultrasonic_measure(ultrasonic_measurement_t *measurement);
bool ultrasonic_read_mm(uint16_t *distanceMm);
ultrasonic_measurement_t ultrasonic_get_last_measurement(void);

#endif
