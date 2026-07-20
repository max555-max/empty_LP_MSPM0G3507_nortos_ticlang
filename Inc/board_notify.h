#ifndef __BOARD_NOTIFY_H_
#define __BOARD_NOTIFY_H_

#include <stdint.h>

/*
 * 板载/外接提示模块
 *
 * 当前约定：
 *   蜂鸣器：低电平触发，所以“关闭”时要输出高电平。
 *   指示灯：用于到点/完成提示。
 *
 * 这个模块把蜂鸣器和灯封装起来，任务代码里不要直接操作 GPIO，
 * 这样后面换引脚或换触发电平时只改这里。
 */

/* 初始化提示 GPIO，并确保蜂鸣器默认关闭。 */
void board_notify_init(void);

/* 打开蜂鸣器。低电平触发蜂鸣器会在这里输出低电平。 */
void board_notify_buzzer_on(void);

/* 关闭蜂鸣器。低电平触发蜂鸣器会在这里输出高电平。 */
void board_notify_buzzer_off(void);

/* 打开指示灯。 */
void board_notify_led_on(void);

/* 关闭指示灯。 */
void board_notify_led_off(void);

/* 到点提示：蜂鸣器短响、灯闪烁/点亮，具体实现见 board_notify.c。 */
void board_notify_arrived(void);

#endif
