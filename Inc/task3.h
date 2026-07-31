#ifndef __TASK3_H_
#define __TASK3_H_

/*
 * Task3 ball-target step test:
 * - wait 1 s after power-on;
 * - then start with the ball position target at -80 mm;
 * - after 2 s from the first target, immediately switch the target to +80 mm.
 */
#define TASK3_STARTUP_WAIT_MS              (3000U)
#define TASK3_FIRST_TARGET_MM              (-70.0f)
#define TASK3_SECOND_TARGET_MM             (70.0f)
#define TASK3_SWITCH_TARGET_DELAY_MS       (1600U)

void task3_run(void);

#endif
