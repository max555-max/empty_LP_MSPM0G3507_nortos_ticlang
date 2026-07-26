#include "mpu6050.h"

#include "delay.h"
#include "ti_msp_dl_config.h"

#ifndef I2C_ICM42688_INST
#error "MPU6050 driver reuses the existing I2C_ICM42688 SysConfig instance."
#endif

#define MPU6050_I2C_INST                 (I2C_ICM42688_INST)

#define MPU6050_ADDRESS_AD0_LOW          (0x68U)
#define MPU6050_ADDRESS_AD0_HIGH         (0x69U)

#define MPU6050_RA_SMPLRT_DIV            (0x19U)
#define MPU6050_RA_CONFIG                (0x1AU)
#define MPU6050_RA_GYRO_CONFIG           (0x1BU)
#define MPU6050_RA_ACCEL_CONFIG          (0x1CU)
#define MPU6050_RA_FIFO_EN               (0x23U)
#define MPU6050_RA_INT_PIN_CFG           (0x37U)
#define MPU6050_RA_INT_ENABLE            (0x38U)
#define MPU6050_RA_ACCEL_XOUT_H          (0x3BU)
#define MPU6050_RA_USER_CTRL             (0x6AU)
#define MPU6050_RA_PWR_MGMT_1            (0x6BU)
#define MPU6050_RA_WHO_AM_I              (0x75U)

#define MPU6050_CLOCK_PLL_YGYRO          (0x02U)
#define MPU6050_GYRO_FS_2000_REG         (0x18U)
#define MPU6050_ACCEL_FS_2G_REG          (0x00U)

#define MPU6050_I2C_TIMEOUT_COUNT        (200000U)
#define MPU6050_START_POLL_DELAY_CYCLES  (128U)
#define MPU6050_INIT_RETRY_COUNT         (5U)

typedef enum {
    MPU6050_ERR_NONE = 0U,
    MPU6050_ERR_ARG = 1U,
    MPU6050_ERR_BUS_BUSY_BEFORE = 2U,
    MPU6050_ERR_TX_FIFO = 3U,
    MPU6050_ERR_TX_DONE = 4U,
    MPU6050_ERR_BUS_BUSY_AFTER_TX = 5U,
    MPU6050_ERR_RX_ERROR = 6U,
    MPU6050_ERR_RX_TIMEOUT = 7U,
    MPU6050_ERR_RX_DONE = 8U,
    MPU6050_ERR_BUS_BUSY_AFTER_RX = 9U
} mpu6050_error_t;

static uint8_t g_mpu6050Address = MPU6050_ADDRESS_INVALID;
static uint8_t g_mpu6050LastAddress = MPU6050_ADDRESS_INVALID;
static uint8_t g_mpu6050LastWhoAmI = 0xFFU;
static uint8_t g_mpu6050LastError = MPU6050_ERR_NONE;
static uint32_t g_mpu6050LastStatus = 0U;
static bool g_mpu6050LastWhoReadOk = false;
static bool g_mpu6050Initialized = false;

static void mpu6050_set_error(mpu6050_error_t error);
static bool mpu6050_wait_bus_released(uint32_t timeout);
static bool mpu6050_wait_controller_done(uint32_t timeout);
static bool mpu6050_prepare_transfer(void);
static void mpu6050_abort_transfer(void);

static bool mpu6050_i2c_write(
    uint8_t address,
    const uint8_t *data,
    uint16_t length);

static bool mpu6050_i2c_read_regs(
    uint8_t address,
    uint8_t reg,
    uint8_t *data,
    uint16_t length);

static bool mpu6050_write_reg(uint8_t reg, uint8_t data);
static bool mpu6050_read_reg(uint8_t reg, uint8_t *data);
static bool mpu6050_try_read_who(uint8_t *who);
static int16_t mpu6050_to_i16(uint8_t high, uint8_t low);
static void mpu6050_clear_raw(mpu6050_raw_t *raw);

bool mpu6050_init(void)
{
    static const uint8_t addressList[2] = {
        MPU6050_ADDRESS_AD0_LOW,
        MPU6050_ADDRESS_AD0_HIGH
    };

    uint8_t who = 0xFFU;
    bool detected = false;
    uint8_t retry;
    uint8_t addressIndex;

    g_mpu6050Address = MPU6050_ADDRESS_INVALID;
    g_mpu6050LastAddress = MPU6050_ADDRESS_INVALID;
    g_mpu6050LastWhoAmI = 0xFFU;
    g_mpu6050LastError = MPU6050_ERR_NONE;
    g_mpu6050LastStatus = 0U;
    g_mpu6050LastWhoReadOk = false;
    g_mpu6050Initialized = false;

    mpu6050_recover_i2c_bus();
    delay_ms(100U);

    for (retry = 0U;
         (retry < MPU6050_INIT_RETRY_COUNT) &&
         (detected == false);
         retry++) {

        for (addressIndex = 0U;
             addressIndex < 2U;
             addressIndex++) {

            g_mpu6050Address = addressList[addressIndex];

            if ((mpu6050_try_read_who(&who) == true) &&
                (who == MPU6050_WHO_AM_I_VALUE)) {

                detected = true;
                break;
            }

            g_mpu6050Address = MPU6050_ADDRESS_INVALID;
            mpu6050_abort_transfer();
        }

        if (detected == false) {
            delay_ms(10U);
        }
    }

    if (detected == false) {
        g_mpu6050Address = MPU6050_ADDRESS_INVALID;
        return false;
    }

    /*
     * Register choices are taken from the vendor MPU6050_initialize()
     * sequence, adapted to direct full-register writes.
     */
    if (mpu6050_write_reg(
            MPU6050_RA_PWR_MGMT_1,
            MPU6050_CLOCK_PLL_YGYRO) == false) {
        goto init_failed;
    }

    if (mpu6050_write_reg(
            MPU6050_RA_GYRO_CONFIG,
            MPU6050_GYRO_FS_2000_REG) == false) {
        goto init_failed;
    }

    if (mpu6050_write_reg(
            MPU6050_RA_ACCEL_CONFIG,
            MPU6050_ACCEL_FS_2G_REG) == false) {
        goto init_failed;
    }

    if (mpu6050_write_reg(MPU6050_RA_USER_CTRL, 0x00U) == false) {
        goto init_failed;
    }

    if (mpu6050_write_reg(MPU6050_RA_INT_PIN_CFG, 0x00U) == false) {
        goto init_failed;
    }

    if (mpu6050_write_reg(MPU6050_RA_INT_ENABLE, 0x00U) == false) {
        goto init_failed;
    }

    if (mpu6050_write_reg(MPU6050_RA_FIFO_EN, 0x00U) == false) {
        goto init_failed;
    }

    /*
     * These two registers are not set by vendor MPU6050_initialize(), but are
     * present in the vendor raw-read demo comments. They only set sample divider
     * and DLPF, not pin or bus resources.
     */
    if (mpu6050_write_reg(MPU6050_RA_SMPLRT_DIV, 0x07U) == false) {
        goto init_failed;
    }

    if (mpu6050_write_reg(MPU6050_RA_CONFIG, 0x03U) == false) {
        goto init_failed;
    }

    delay_ms(10U);

    if (mpu6050_get_who_am_i() != MPU6050_WHO_AM_I_VALUE) {
        goto init_failed;
    }

    g_mpu6050Initialized = true;
    return true;

init_failed:
    g_mpu6050Address = MPU6050_ADDRESS_INVALID;
    g_mpu6050Initialized = false;
    mpu6050_abort_transfer();
    return false;
}

uint8_t mpu6050_get_who_am_i(void)
{
    uint8_t who = 0xFFU;

    if (g_mpu6050Address == MPU6050_ADDRESS_INVALID) {
        return 0xFFU;
    }

    if (mpu6050_try_read_who(&who) == false) {
        return 0xFFU;
    }

    return who;
}

bool mpu6050_read_raw(mpu6050_raw_t *raw)
{
    uint8_t data[14];

    if (raw == 0) {
        return false;
    }

    if ((g_mpu6050Address == MPU6050_ADDRESS_INVALID) ||
        (mpu6050_i2c_read_regs(
             g_mpu6050Address,
             MPU6050_RA_ACCEL_XOUT_H,
             data,
             (uint16_t)sizeof(data)) == false)) {

        mpu6050_clear_raw(raw);
        g_mpu6050Initialized = false;
        return false;
    }

    raw->accelX = mpu6050_to_i16(data[0], data[1]);
    raw->accelY = mpu6050_to_i16(data[2], data[3]);
    raw->accelZ = mpu6050_to_i16(data[4], data[5]);
    raw->temp = mpu6050_to_i16(data[6], data[7]);
    raw->gyroX = mpu6050_to_i16(data[8], data[9]);
    raw->gyroY = mpu6050_to_i16(data[10], data[11]);
    raw->gyroZ = mpu6050_to_i16(data[12], data[13]);
    raw->whoAmI = MPU6050_WHO_AM_I_VALUE;
    return true;
}

void mpu6050_get_diag(mpu6050_diag_t *diag)
{
    if (diag == 0) {
        return;
    }

    diag->currentAddress = g_mpu6050Address;
    diag->lastAddress = g_mpu6050LastAddress;
    diag->lastWhoAmI = g_mpu6050LastWhoAmI;
    diag->lastError = g_mpu6050LastError;
    diag->lastStatus = g_mpu6050LastStatus;
    diag->lastWhoReadOk = g_mpu6050LastWhoReadOk;
    diag->initialized = g_mpu6050Initialized;
}

void mpu6050_recover_i2c_bus(void)
{
    uint8_t cycleCount = 0U;

    if (DL_I2C_getSDAStatus(MPU6050_I2C_INST) !=
        DL_I2C_CONTROLLER_SDA_LOW) {
        return;
    }

    DL_I2C_reset(MPU6050_I2C_INST);

    DL_GPIO_initDigitalOutput(GPIO_I2C_ICM42688_IOMUX_SCL);
    DL_GPIO_initDigitalInputFeatures(
        GPIO_I2C_ICM42688_IOMUX_SDA,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_enableOutput(
        GPIO_I2C_ICM42688_SCL_PORT,
        GPIO_I2C_ICM42688_SCL_PIN);

    do {
        DL_GPIO_clearPins(
            GPIO_I2C_ICM42688_SCL_PORT,
            GPIO_I2C_ICM42688_SCL_PIN);
        delay_ms(1U);

        DL_GPIO_setPins(
            GPIO_I2C_ICM42688_SCL_PORT,
            GPIO_I2C_ICM42688_SCL_PIN);
        delay_ms(1U);

        if (DL_GPIO_readPins(
                GPIO_I2C_ICM42688_SDA_PORT,
                GPIO_I2C_ICM42688_SDA_PIN) != 0U) {
            break;
        }

        cycleCount++;
    } while (cycleCount < 100U);

    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_I2C_ICM42688_IOMUX_SDA,
        GPIO_I2C_ICM42688_IOMUX_SDA_FUNC,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_I2C_ICM42688_IOMUX_SCL,
        GPIO_I2C_ICM42688_IOMUX_SCL_FUNC,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_enableHiZ(GPIO_I2C_ICM42688_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_ICM42688_IOMUX_SCL);

    DL_I2C_enablePower(MPU6050_I2C_INST);
    SYSCFG_DL_I2C_ICM42688_init();
}

static bool mpu6050_try_read_who(uint8_t *who)
{
    uint8_t value = 0xFFU;
    bool ok;

    if (who == 0) {
        return false;
    }

    g_mpu6050LastAddress = g_mpu6050Address;
    ok = mpu6050_read_reg(MPU6050_RA_WHO_AM_I, &value);

    g_mpu6050LastWhoReadOk = ok;
    g_mpu6050LastWhoAmI = ok ? value : 0xFFU;
    *who = g_mpu6050LastWhoAmI;

    return ok;
}

static void mpu6050_set_error(mpu6050_error_t error)
{
    g_mpu6050LastError = (uint8_t)error;
    g_mpu6050LastStatus = DL_I2C_getControllerStatus(MPU6050_I2C_INST);
}

static bool mpu6050_wait_bus_released(uint32_t timeout)
{
    uint32_t status;

    while (1) {
        status = DL_I2C_getControllerStatus(MPU6050_I2C_INST);

        if (((status & DL_I2C_CONTROLLER_STATUS_BUSY) == 0U) &&
            ((status & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) == 0U)) {

            g_mpu6050LastStatus = status;
            return
                (status & DL_I2C_CONTROLLER_STATUS_ERROR) == 0U;
        }

        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            g_mpu6050LastStatus = status;
            return false;
        }

        if (timeout == 0U) {
            g_mpu6050LastStatus = status;
            return false;
        }

        timeout--;
    }
}

static bool mpu6050_wait_controller_done(uint32_t timeout)
{
    uint32_t status;

    while (1) {
        status = DL_I2C_getControllerStatus(MPU6050_I2C_INST);

        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            g_mpu6050LastStatus = status;
            return false;
        }

        if ((status & DL_I2C_CONTROLLER_STATUS_BUSY) == 0U) {
            g_mpu6050LastStatus = status;
            return true;
        }

        if (timeout == 0U) {
            g_mpu6050LastStatus = status;
            return false;
        }

        timeout--;
    }
}

static bool mpu6050_prepare_transfer(void)
{
    DL_I2C_resetControllerTransfer(MPU6050_I2C_INST);
    DL_I2C_flushControllerTXFIFO(MPU6050_I2C_INST);
    DL_I2C_flushControllerRXFIFO(MPU6050_I2C_INST);

    if (mpu6050_wait_bus_released(
            MPU6050_I2C_TIMEOUT_COUNT) == false) {

        mpu6050_set_error(MPU6050_ERR_BUS_BUSY_BEFORE);
        mpu6050_abort_transfer();
        return false;
    }

    return true;
}

static void mpu6050_abort_transfer(void)
{
    DL_I2C_resetControllerTransfer(MPU6050_I2C_INST);
    DL_I2C_flushControllerTXFIFO(MPU6050_I2C_INST);
    DL_I2C_flushControllerRXFIFO(MPU6050_I2C_INST);
}

static bool mpu6050_i2c_write(
    uint8_t address,
    const uint8_t *data,
    uint16_t length)
{
    uint16_t written;

    if ((data == 0) ||
        (length == 0U) ||
        (length > 8U)) {
        mpu6050_set_error(MPU6050_ERR_ARG);
        return false;
    }

    if (mpu6050_prepare_transfer() == false) {
        return false;
    }

    written = DL_I2C_fillControllerTXFIFO(
        MPU6050_I2C_INST,
        (uint8_t *)data,
        length);

    if (written != length) {
        mpu6050_set_error(MPU6050_ERR_TX_FIFO);
        mpu6050_abort_transfer();
        return false;
    }

    DL_I2C_startControllerTransfer(
        MPU6050_I2C_INST,
        address,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        length);

    delay_cycles(MPU6050_START_POLL_DELAY_CYCLES);

    if (mpu6050_wait_controller_done(
            MPU6050_I2C_TIMEOUT_COUNT) == false) {

        mpu6050_set_error(MPU6050_ERR_TX_DONE);
        mpu6050_abort_transfer();
        return false;
    }

    if (mpu6050_wait_bus_released(
            MPU6050_I2C_TIMEOUT_COUNT) == false) {

        mpu6050_set_error(MPU6050_ERR_BUS_BUSY_AFTER_TX);
        mpu6050_abort_transfer();
        return false;
    }

    return true;
}

static bool mpu6050_i2c_read_regs(
    uint8_t address,
    uint8_t reg,
    uint8_t *data,
    uint16_t length)
{
    uint16_t received = 0U;
    uint32_t timeout;
    uint32_t status;

    if ((data == 0) ||
        (length == 0U)) {
        mpu6050_set_error(MPU6050_ERR_ARG);
        return false;
    }

    if (mpu6050_prepare_transfer() == false) {
        return false;
    }

    /*
     * Follow the vendor hardware-I2C read flow:
     *   1. Put the register address into TX.
     *   2. Enable RD_ON_TXEMPTY so the controller changes to read after the
     *      register byte is transmitted.
     *   3. Start one RX transfer.
     *
     * This is different from the previous STOP-separated register write/read
     * sequence and matches the vendor bsp_iic.c method more closely.
     */
    DL_I2C_transmitControllerData(MPU6050_I2C_INST, reg);
    MPU6050_I2C_INST->MASTER.MCTR = I2C_MCTR_RD_ON_TXEMPTY_ENABLE;

    DL_I2C_clearInterruptStatus(
        MPU6050_I2C_INST,
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
            DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);

    if (mpu6050_wait_controller_done(
            MPU6050_I2C_TIMEOUT_COUNT) == false) {

        MPU6050_I2C_INST->MASTER.MCTR = 0U;
        mpu6050_set_error(MPU6050_ERR_TX_DONE);
        mpu6050_abort_transfer();
        return false;
    }

    DL_I2C_startControllerTransfer(
        MPU6050_I2C_INST,
        address,
        DL_I2C_CONTROLLER_DIRECTION_RX,
        length);

    delay_cycles(MPU6050_START_POLL_DELAY_CYCLES);

    timeout = MPU6050_I2C_TIMEOUT_COUNT;

    while (received < length) {
        while ((DL_I2C_isControllerRXFIFOEmpty(
                    MPU6050_I2C_INST) == false) &&
               (received < length)) {

            data[received] =
                DL_I2C_receiveControllerData(MPU6050_I2C_INST);
            received++;
        }

        status = DL_I2C_getControllerStatus(MPU6050_I2C_INST);
        g_mpu6050LastStatus = status;

        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            MPU6050_I2C_INST->MASTER.MCTR = 0U;
            mpu6050_set_error(MPU6050_ERR_RX_ERROR);
            mpu6050_abort_transfer();
            return false;
        }

        if (timeout == 0U) {
            MPU6050_I2C_INST->MASTER.MCTR = 0U;
            mpu6050_set_error(MPU6050_ERR_RX_TIMEOUT);
            mpu6050_abort_transfer();
            return false;
        }

        timeout--;
    }

    if (mpu6050_wait_controller_done(
            MPU6050_I2C_TIMEOUT_COUNT) == false) {

        MPU6050_I2C_INST->MASTER.MCTR = 0U;
        mpu6050_set_error(MPU6050_ERR_RX_DONE);
        mpu6050_abort_transfer();
        return false;
    }

    while ((DL_I2C_isControllerRXFIFOEmpty(MPU6050_I2C_INST) == false) &&
           (received < length)) {

        data[received] =
            DL_I2C_receiveControllerData(MPU6050_I2C_INST);
        received++;
    }

    MPU6050_I2C_INST->MASTER.MCTR = 0U;
    DL_I2C_flushControllerTXFIFO(MPU6050_I2C_INST);

    if (received != length) {
        mpu6050_set_error(MPU6050_ERR_RX_DONE);
        mpu6050_abort_transfer();
        return false;
    }

    if (mpu6050_wait_bus_released(
            MPU6050_I2C_TIMEOUT_COUNT) == false) {

        mpu6050_set_error(MPU6050_ERR_BUS_BUSY_AFTER_RX);
        mpu6050_abort_transfer();
        return false;
    }

    mpu6050_set_error(MPU6050_ERR_NONE);
    return true;
}

static bool mpu6050_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t tx[2];

    if (g_mpu6050Address == MPU6050_ADDRESS_INVALID) {
        return false;
    }

    tx[0] = reg;
    tx[1] = data;

    return mpu6050_i2c_write(
        g_mpu6050Address,
        tx,
        (uint16_t)sizeof(tx));
}

static bool mpu6050_read_reg(uint8_t reg, uint8_t *data)
{
    if ((g_mpu6050Address == MPU6050_ADDRESS_INVALID) ||
        (data == 0)) {
        return false;
    }

    return mpu6050_i2c_read_regs(
        g_mpu6050Address,
        reg,
        data,
        1U);
}

static int16_t mpu6050_to_i16(uint8_t high, uint8_t low)
{
    return
        (int16_t)(
            ((uint16_t)high << 8) |
            (uint16_t)low);
}

static void mpu6050_clear_raw(mpu6050_raw_t *raw)
{
    raw->accelX = 0;
    raw->accelY = 0;
    raw->accelZ = 0;
    raw->temp = 0;
    raw->gyroX = 0;
    raw->gyroY = 0;
    raw->gyroZ = 0;
    raw->whoAmI = 0xFFU;
}
