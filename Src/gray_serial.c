#include <stdint.h>

#include "gray_serial.h"

#include "ti_msp_dl_config.h"
#include "vofa.h"

static uint8_t gray_serial_read_pin(uint32_t pinMask)
{
    return (DL_GPIO_readPins(GRAY_SERIAL_PORT, pinMask) & pinMask) != 0U ? 1U : 0U;
}

void gray_serial_init(void)
{
    DL_GPIO_initDigitalInputFeatures(GRAY_SERIAL_O1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GRAY_SERIAL_O2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GRAY_SERIAL_O3_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GRAY_SERIAL_O4_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

uint8_t gray_serial_read(void)
{
    uint8_t raw = 0U;

    raw |= (gray_serial_read_pin(GRAY_SERIAL_O1_PIN) << 0U);
    raw |= (gray_serial_read_pin(GRAY_SERIAL_O2_PIN) << 1U);
    raw |= (gray_serial_read_pin(GRAY_SERIAL_O3_PIN) << 2U);
    raw |= (gray_serial_read_pin(GRAY_SERIAL_O4_PIN) << 3U);

    return raw;
}

void gray_serial_print(uint8_t value)
{
    uart0_send_string("Digital:");
    uart0_send_int((value >> 0U) & 0x01U);
    uart0_send_byte(',');
    uart0_send_int((value >> 1U) & 0x01U);
    uart0_send_byte(',');
    uart0_send_int((value >> 2U) & 0x01U);
    uart0_send_byte(',');
    uart0_send_int((value >> 3U) & 0x01U);
    uart0_send_string("\r\n");
}
