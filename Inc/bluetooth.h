#ifndef __BLUETOOTH_H_
#define __BLUETOOTH_H_

#include <stdint.h>

/*
 * UART1 Bluetooth tuning module.
 * Current version tunes only the two fixed inner-wheel ratios:
 *   small turn, large turn
 *
 * Frames:
 *   {GET}
 *   {HELP}
 *   {DIFF=0.90,0.60}
 *   {SET 90,60}
 *
 * Note:
 *   UART1 must not be used by another RX listener at the same time.
 */
void bluetooth_init(void);
void bluetooth_process(void);
void bluetooth_send_params(void);

#endif
