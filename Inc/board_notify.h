#ifndef __BOARD_NOTIFY_H_
#define __BOARD_NOTIFY_H_

#include <stdint.h>

void board_notify_init(void);
void board_notify_buzzer_on(void);
void board_notify_buzzer_off(void);
void board_notify_led_on(void);
void board_notify_led_off(void);
void board_notify_arrived(void);

#endif
