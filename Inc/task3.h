#ifndef __TASK3_H_
#define __TASK3_H_

/*
 * Task3:
 * - zero the beam motor after power-on;
 * - then enter the existing ball-control flow.
 */
#define TASK3_STARTUP_WAIT_MS              (1000U)
#define TASK3_FIRST_TARGET_MM              (-80.0f)
#define TASK3_SECOND_TARGET_MM             (80.0f)
#define TASK3_SWITCH_TARGET_DELAY_MS       (2000U)

void task3_run(void);
void task3_motor_zero_run(void);

#endif
