#include "board_notify.h"

#include "delay.h"
#include "ti_msp_dl_config.h"

#define BOARD_NOTIFY_BEEP_MS       (180U)
#define BOARD_NOTIFY_GAP_MS        (120U)
#define BOARD_NOTIFY_REPEAT_COUNT  (1U)

void board_notify_init(void)
{
    board_notify_buzzer_off();
    board_notify_led_off();
}

void board_notify_buzzer_on(void)
{
    /*
     * The buzzer module is low-level triggered.
     */
    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_BZ_PIN);
}

void board_notify_buzzer_off(void)
{
    DL_GPIO_setPins(BUZZER_PORT, BUZZER_BZ_PIN);
}

void board_notify_led_on(void)
{
    DL_GPIO_setPins(LED_PORT, LED_LED1_PIN);
}

void board_notify_led_off(void)
{
    DL_GPIO_clearPins(LED_PORT, LED_LED1_PIN);
}

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
