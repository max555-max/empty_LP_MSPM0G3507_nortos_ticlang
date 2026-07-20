#include "gray_serial.h"

#include "ti_msp_dl_config.h"
#include "vofa.h"

/*
 * gray_serial.c
 *
 * 八路灰度模块的串行读取驱动。
 * 本文件只负责把 8bit 数字量读出来；
 * 通道加权、循迹 PD 控制在 line_track.c 中完成。
 */

/* 软件微秒延时，用于满足灰度模块 CLK/DAT 时序。 */
static void gray_serial_delay_us(uint32_t us)
{
    delay_cycles((CPUCLK_FREQ / 1000000U) * us);
}

/*
 * 初始化灰度串行接口。
 *
 * 当前只需要把 CLK 拉高，因为 DAT 是输入。
 */
void gray_serial_init(void)
{
    DL_GPIO_setPins(GRAY_SERIAL_CLK_PORT, GRAY_SERIAL_CLK_PIN);
}

/*
 * 读取 8 路灰度数字量。
 *
 * 返回值：
 *   raw 的 bit0~bit7 为模块串行输出的原始顺序。
 *   注意：这个原始顺序不是“传感器从左到右”的物理顺序。
 */
uint8_t gray_serial_read(void)
{
    uint8_t value = 0;
    uint8_t i;

    /* 连续读取 8bit，每个时钟周期读取一位 DAT。 */
    for (i = 0U; i < 8U; i++) {
        /* CLK 拉低后等待数据稳定。 */
        DL_GPIO_clearPins(GRAY_SERIAL_CLK_PORT, GRAY_SERIAL_CLK_PIN);
        gray_serial_delay_us(2U);

        /* DAT 为高电平则置位当前 bit。 */
        if ((DL_GPIO_readPins(GRAY_SERIAL_DAT_PORT, GRAY_SERIAL_DAT_PIN) &
             GRAY_SERIAL_DAT_PIN) != 0U) {
            value |= (uint8_t) (1U << i);
        }

        /* CLK 拉高，准备下一位。 */
        DL_GPIO_setPins(GRAY_SERIAL_CLK_PORT, GRAY_SERIAL_CLK_PIN);
        gray_serial_delay_us(5U);
    }

    return value;
}

/*
 * 按物理通道顺序打印灰度数据。
 *
 * 输出格式：
 *   Digital:ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8
 */
void gray_serial_print(uint8_t value)
{
    uint8_t i;

    /*
     * 原始 bit 顺序不是物理从左到右顺序。
     * 这里按传感器从左到右通道 1~8 打印，方便人工核对映射。
     */
    static const uint8_t channelBitMap[8] = {
        1U, 2U, 3U, 4U, 5U, 6U, 7U, 0U
    };

    uart0_send_string("Digital:");

    for (i = 0U; i < 8U; i++) {
        if (i != 0U) {
            uart0_send_byte(',');
        }

        uart0_send_int((value >> channelBitMap[i]) & 0x01U);
    }

    uart0_send_string("\r\n");
}
