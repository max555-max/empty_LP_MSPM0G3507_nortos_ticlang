#ifndef __TASK3_ACBD_H_
#define __TASK3_ACBD_H_

/*
 * Task 3:
 *   A -> C: gyro heading loop + encoder distance, diagonal straight segment.
 *   C -> B: gray sensor line tracking on the right semicircle.
 *   B -> D: gyro heading loop + encoder distance, diagonal straight segment.
 *   D -> A: gray sensor line tracking on the left semicircle, stop after line lost.
 *
 * main() only needs to call task3_acbd_run().
 */
void task3_acbd_run(void);

#endif
