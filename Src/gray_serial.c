#include "gray_serial.h"

#include "ti_msp_dl_config.h"
#include "vofa.h"

static void gray_serial_delay_us(uint32_t us)
{
    delay_cycles((CPUCLK_FREQ / 1000000U) * us);
}

void gray_serial_init(void)
{
    DL_GPIO_setPins(GRAY_SERIAL_CLK_PORT, GRAY_SERIAL_CLK_PIN);
}

uint8_t gray_serial_read(void)
{
    uint8_t value = 0;
    uint8_t i;

    for (i = 0; i < 8U; i++) {
        DL_GPIO_clearPins(GRAY_SERIAL_CLK_PORT, GRAY_SERIAL_CLK_PIN);
        gray_serial_delay_us(2U);

        if ((DL_GPIO_readPins(GRAY_SERIAL_DAT_PORT, GRAY_SERIAL_DAT_PIN) &
             GRAY_SERIAL_DAT_PIN) != 0U) {
            value |= (uint8_t) (1U << i);
        }

        DL_GPIO_setPins(GRAY_SERIAL_CLK_PORT, GRAY_SERIAL_CLK_PIN);
        gray_serial_delay_us(5U);
    }

    return value;
}

void gray_serial_print(uint8_t value)
{
    uint8_t i;
    static const uint8_t channelBitMap[8] = {
        1U, 2U, 3U, 4U, 5U, 6U, 7U, 0U
    };

    uart0_send_string("Digital:");

    for (i = 0; i < 8U; i++) {
        if (i != 0U) {
            uart0_send_byte(',');
        }

        uart0_send_int((value >> channelBitMap[i]) & 0x01U);
    }

    uart0_send_string("\r\n");
}
