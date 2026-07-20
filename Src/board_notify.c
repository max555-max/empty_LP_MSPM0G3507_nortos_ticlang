#include "board_notify.h"

#include "delay.h"
#include "ti_msp_dl_config.h"

/*
 * board_notify.c
 *
 * 蜂鸣器和 LED 提示封装。
 * 目前蜂鸣器为低电平触发：GPIO 拉低会响，拉高关闭。
 */

/* 单次蜂鸣持续时间。 */
#define BOARD_NOTIFY_BEEP_MS       (180U)

/* 多次提示之间的间隔。 */
#define BOARD_NOTIFY_GAP_MS        (120U)

/* 到点提示重复次数。 */
#define BOARD_NOTIFY_REPEAT_COUNT  (1U)

/*
 * 初始化提示模块。
 *
 * 上电后先关蜂鸣器和 LED，避免低电平触发蜂鸣器误响。
 */
void board_notify_init(void)
{
    board_notify_buzzer_off();
    board_notify_led_off();
}

/* 打开蜂鸣器：低电平触发。 */
void board_notify_buzzer_on(void)
{
    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_BZ_PIN);
}

/* 关闭蜂鸣器：低电平触发，所以拉高关闭。 */
void board_notify_buzzer_off(void)
{
    DL_GPIO_setPins(BUZZER_PORT, BUZZER_BZ_PIN);
}

/* 打开 LED。 */
void board_notify_led_on(void)
{
    DL_GPIO_setPins(LED_PORT, LED_LED1_PIN);
}

/* 关闭 LED。 */
void board_notify_led_off(void)
{
    DL_GPIO_clearPins(LED_PORT, LED_LED1_PIN);
}

/*
 * 到点/完成提示。
 *
 * 逻辑：
 *   蜂鸣器短响；
 *   LED 同步亮灭；
 *   最后 LED 保持亮，表示任务已到点或结束。
 */
void board_notify_arrived(void)
{
    for (uint8_t i = 0U; i < BOARD_NOTIFY_REPEAT_COUNT; i++) {
        board_notify_buzzer_on();
        board_notify_led_on();
        delay_ms(BOARD_NOTIFY_BEEP_MS);

        board_notify_buzzer_off();
        board_notify_led_off();
        delay_ms(BOARD_NOTIFY_GAP_MS);
    }

    board_notify_led_on();
}
