#ifndef __TASK3_H_
#define __TASK3_H_

/*
 * Task3:
 * - move the beam to its logical encoder zero after task start;
 * - wait until a valid vision sample reports the ball near its zero;
 * - drive the ball toward -80 mm;
 * - switch to +80 mm when vision reports the ball near -50 mm;
 * - return when PA7 is pressed so main() can start Task2;
 * - return when PB8 is pressed so main() can start Task4.
 */
#define TASK3_ZERO_TARGET_DEG                       (0.0f)
#define TASK3_ZERO_ANGLE_TOLERANCE_DEG_X10          (5)
#define TASK3_BALL_ZERO_POSITION_MM                 (0.0f)
#define TASK3_BALL_ZERO_TOLERANCE_MM                (5.0f)
#define TASK3_FIRST_TARGET_MM                       (-70.0f)
#define TASK3_SWITCH_TO_SECOND_POSITION_MM          (-50.0f)
#define TASK3_SWITCH_TO_SECOND_TOLERANCE_MM         (5.0f)
#define TASK3_SECOND_TARGET_MM                      (70.0f)

typedef enum {
    TASK3_EXIT_TO_TASK2 = 2,
    TASK3_EXIT_TO_TASK4 = 4
} task3_exit_t;

task3_exit_t task3_run(void);

#endif
