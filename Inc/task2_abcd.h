#ifndef __TASK2_ABCD_H_
#define __TASK2_ABCD_H_

/*
 * Task 2:
 *   A -> B: drive straight by encoder distance + gyro heading hold
 *   B -> C: follow the right semicircle black line
 *   C -> D: drive straight by encoder distance + gyro heading hold
 *   D -> A: follow the left semicircle black line
 *
 * main() only needs to call task2_abcd_run().
 */
void task2_abcd_run(void);

#endif
