#include "icm42688.h"

#include "ti_msp_dl_config.h"
#include "vofa.h"

#define ICM42688_DEVICE_CONFIG   (0x11U)
#define ICM42688_TEMP_DATA1      (0x1DU)
#define ICM42688_PWR_MGMT0       (0x4EU)
#define ICM42688_GYRO_CONFIG0    (0x4FU)
#define ICM42688_ACCEL_CONFIG0   (0x50U)
#define ICM42688_WHO_AM_I        (0x75U)
#define ICM42688_WHO_AM_I_VALUE  (0x47U)
#define ICM42688_SPI_READ_BIT    (0x80U)

#define ICM42688_ACCEL_16G_1KHZ  (0x06U)
#define ICM42688_GYRO_2000_1KHZ  (0x06U)
#define ICM42688_ACCEL_GYRO_LN   (0x0FU)

static void icm42688_delay_us(uint32_t us);
static void icm42688_cs_low(void);
static void icm42688_cs_high(void);
static void icm42688_drain_rx_fifo(void);
static uint8_t icm42688_spi_transfer(uint8_t tx);
static void icm42688_write_reg(uint8_t reg, uint8_t data);
static uint8_t icm42688_read_reg(uint8_t reg);
static void icm42688_read_regs(uint8_t reg, uint8_t *data, uint8_t len);
static int16_t icm42688_to_i16(uint8_t high, uint8_t low);

bool icm42688_init(void)
{
    uint8_t who = 0U;
    bool detected = false;

    /*
     * Debug-friendly SPI speed:
     * 80MHz / ((199 + 1) * 2) = 200kHz.
     * Slow down first to make wiring and signal-integrity issues easier to
     * diagnose. It can be raised again after WHO_AM_I is stable.
     */
    DL_SPI_disable(SPI_ICM42688_INST);
    DL_SPI_setFrameFormat(SPI_ICM42688_INST, DL_SPI_FRAME_FORMAT_MOTO3_POL0_PHA1);
    DL_SPI_setBitRateSerialClockDivider(SPI_ICM42688_INST, 199U);
    DL_SPI_enable(SPI_ICM42688_INST);

    icm42688_cs_high();
    delay_cycles(CPUCLK_FREQ / 50U);

    for (uint8_t i = 0U; i < 50U; i++) {
        who = icm42688_get_who_am_i();
        if (who == ICM42688_WHO_AM_I_VALUE) {
            detected = true;
            break;
        }
        delay_cycles(CPUCLK_FREQ / 100U);
    }

    /*
     * 先按商家例程的最小初始化方式配置：关闭传感器、设置量程/ODR、
     * 再打开加速度计和陀螺仪低噪声模式。这里只读取原始值，不做物理量换算。
     */
    icm42688_write_reg(ICM42688_PWR_MGMT0, 0x00U);
    delay_cycles(CPUCLK_FREQ / 100U);
    icm42688_write_reg(ICM42688_ACCEL_CONFIG0, ICM42688_ACCEL_16G_1KHZ);
    icm42688_write_reg(ICM42688_GYRO_CONFIG0, ICM42688_GYRO_2000_1KHZ);
    icm42688_write_reg(ICM42688_PWR_MGMT0, ICM42688_ACCEL_GYRO_LN);
    delay_cycles(CPUCLK_FREQ / 10U);

    return detected;
}

uint8_t icm42688_get_who_am_i(void)
{
    return icm42688_read_reg(ICM42688_WHO_AM_I);
}

void icm42688_read_raw(icm42688_raw_t *raw)
{
    uint8_t data[14];

    if (raw == 0) {
        return;
    }

    icm42688_read_regs(ICM42688_TEMP_DATA1, data, sizeof(data));

    raw->temp = icm42688_to_i16(data[0], data[1]);
    raw->accelX = icm42688_to_i16(data[2], data[3]);
    raw->accelY = icm42688_to_i16(data[4], data[5]);
    raw->accelZ = icm42688_to_i16(data[6], data[7]);
    raw->gyroX = icm42688_to_i16(data[8], data[9]);
    raw->gyroY = icm42688_to_i16(data[10], data[11]);
    raw->gyroZ = icm42688_to_i16(data[12], data[13]);
    raw->whoAmI = icm42688_get_who_am_i();
}

void icm42688_print_raw(const icm42688_raw_t *raw)
{
    if (raw == 0) {
        return;
    }

    uart0_send_string("ICM:");
    uart0_send_int(raw->accelX);
    uart0_send_byte(',');
    uart0_send_int(raw->accelY);
    uart0_send_byte(',');
    uart0_send_int(raw->accelZ);
    uart0_send_byte(',');
    uart0_send_int(raw->gyroX);
    uart0_send_byte(',');
    uart0_send_int(raw->gyroY);
    uart0_send_byte(',');
    uart0_send_int(raw->gyroZ);
    uart0_send_byte(',');
    uart0_send_int(raw->temp);
    uart0_send_byte(',');
    uart0_send_int(raw->whoAmI);
    uart0_send_string("\r\n");
}

static void icm42688_delay_us(uint32_t us)
{
    while (us > 0U) {
        delay_cycles(CPUCLK_FREQ / 1000000U);
        us--;
    }
}

static void icm42688_cs_low(void)
{
    DL_GPIO_clearPins(ICM42688_CS_PORT, ICM42688_CS_CS_PIN);
    icm42688_delay_us(1U);
}

static void icm42688_cs_high(void)
{
    while (DL_SPI_isBusy(SPI_ICM42688_INST)) {
    }
    DL_GPIO_setPins(ICM42688_CS_PORT, ICM42688_CS_CS_PIN);
    icm42688_delay_us(1U);
}

static void icm42688_drain_rx_fifo(void)
{
    while (DL_SPI_isRXFIFOEmpty(SPI_ICM42688_INST) == false) {
        (void) DL_SPI_receiveDataBlocking8(SPI_ICM42688_INST);
    }
}

static uint8_t icm42688_spi_transfer(uint8_t tx)
{
    DL_SPI_transmitDataBlocking8(SPI_ICM42688_INST, tx);
    return DL_SPI_receiveDataBlocking8(SPI_ICM42688_INST);
}

static void icm42688_write_reg(uint8_t reg, uint8_t data)
{
    icm42688_drain_rx_fifo();
    icm42688_cs_low();
    (void) icm42688_spi_transfer(reg & (uint8_t) ~ICM42688_SPI_READ_BIT);
    (void) icm42688_spi_transfer(data);
    icm42688_cs_high();
}

static uint8_t icm42688_read_reg(uint8_t reg)
{
    uint8_t data;

    icm42688_drain_rx_fifo();
    icm42688_cs_low();
    (void) icm42688_spi_transfer(reg | ICM42688_SPI_READ_BIT);
    data = icm42688_spi_transfer(0x00U);
    icm42688_cs_high();

    return data;
}

static void icm42688_read_regs(uint8_t reg, uint8_t *data, uint8_t len)
{
    if ((data == 0) || (len == 0U)) {
        return;
    }

    icm42688_drain_rx_fifo();
    icm42688_cs_low();
    (void) icm42688_spi_transfer(reg | ICM42688_SPI_READ_BIT);
    for (uint8_t i = 0U; i < len; i++) {
        data[i] = icm42688_spi_transfer(0x00U);
    }
    icm42688_cs_high();
}

static int16_t icm42688_to_i16(uint8_t high, uint8_t low)
{
    return (int16_t) (((uint16_t) high << 8) | (uint16_t) low);
}
