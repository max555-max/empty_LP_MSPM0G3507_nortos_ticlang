#include "icm42688.h"

#include <stdbool.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "vofa.h"

/*
 * SysConfig 中必须把硬件 I2C Controller 实例命名为：
 *
 *     I2C_ICM42688
 *
 * 这样 SysConfig 才会生成 I2C_ICM42688_INST。
 */
#ifndef I2C_ICM42688_INST
#error "请在 SysConfig 中添加 I2C Controller，并把实例名称设置为 I2C_ICM42688"
#endif

#define ICM42688_I2C_INST                  (I2C_ICM42688_INST)

/* ICM42688 Bank 0 常用寄存器。 */
#define ICM42688_TEMP_DATA1                (0x1DU)
#define ICM42688_PWR_MGMT0                 (0x4EU)
#define ICM42688_GYRO_CONFIG0              (0x4FU)
#define ICM42688_ACCEL_CONFIG0             (0x50U)
#define ICM42688_WHO_AM_I                  (0x75U)
#define ICM42688_WHO_AM_I_VALUE            (0x47U)

/*
 * AD0/SDO 固定接高电平时，ICM42688 的 7 位 I2C 地址固定为 0x69。
 *
 * DriverLib 接收的是 7 位地址：
 *   正确：0x69
 *   错误：0xD2 / 0xD3
 */
#define ICM42688_I2C_ADDRESS               (0x69U)
#define ICM42688_I2C_ADDRESS_INVALID       (0xFFU)

/*
 * 当前量程和采样率：
 *
 *   accel：±16 g，1 kHz；
 *   gyro ：±2000 dps，1 kHz；
 *   accel/gyro：低噪声模式。
 */
#define ICM42688_ACCEL_16G_1KHZ            (0x06U)
#define ICM42688_GYRO_2000_1KHZ            (0x06U)
#define ICM42688_ACCEL_GYRO_LN             (0x0FU)

/* 初始化时在固定地址 0x69 上的最大探测次数。 */
#define ICM42688_INIT_RETRY_COUNT           (5U)

/* 正常读写在固定地址 0x69 上的重试次数。 */
#define ICM42688_TRANSFER_RETRY_COUNT       (2U)

/* 错误事务恢复后，再发起下一笔事务前的等待时间。 */
#define ICM42688_RECOVERY_DELAY_CYCLES      (CPUCLK_FREQ / 1000U)
#define ICM42688_ABORT_SETTLE_CYCLES        (CPUCLK_FREQ / 10000U)

/*
 * 软件轮询超时。
 *
 * 该值是循环次数，不是毫秒。
 */
#define ICM42688_I2C_TIMEOUT_COUNT          (200000U)

/*
 * 启动控制器传输后，先等待一小段时间再轮询 BUSY，
 * 避免启动后过早读取状态。
 */
#define ICM42688_START_POLL_DELAY_CYCLES    (128U)

/*
 * 该变量同时表示：
 *   1. 当前使用的 7 位 I2C 地址；
 *   2. ICM42688 是否已经成功完成初始化。
 */
static uint8_t g_icm42688Address =
    ICM42688_I2C_ADDRESS_INVALID;

static bool icm42688_wait_bus_released(uint32_t timeout);
static bool icm42688_wait_controller_done(uint32_t timeout);
static bool icm42688_prepare_transfer(void);
static void icm42688_abort_transfer(void);

static bool icm42688_i2c_write(
    uint8_t address,
    const uint8_t *data,
    uint16_t length);

static bool icm42688_i2c_read_regs(
    uint8_t address,
    uint8_t reg,
    uint8_t *data,
    uint16_t length);

static bool icm42688_read_regs_safe(
    uint8_t reg,
    uint8_t *data,
    uint16_t length);

static bool icm42688_write_reg(uint8_t reg, uint8_t data);
static bool icm42688_read_reg(uint8_t reg, uint8_t *data);

static int16_t icm42688_to_i16(uint8_t high, uint8_t low);
static void icm42688_clear_raw(icm42688_raw_t *raw);

/*
 * 初始化 ICM42688。
 *
 * 硬件前提：
 *   AD0/SDO 接高电平，地址固定为 0x69；
 *   CS 接 VDDIO，使器件工作在 I2C 模式；
 *   SDA、SCL 有合适的上拉电阻。
 *
 * 返回：
 *   true ：0x69 地址读取 WHO_AM_I=0x47，并完成寄存器配置；
 *   false：I2C 通信或传感器初始化失败。
 */
bool icm42688_init(void)
{
    uint8_t who = 0xFFU;
    uint8_t retry;
    bool detected = false;

    g_icm42688Address =
        ICM42688_I2C_ADDRESS_INVALID;

    /* 等待传感器上电稳定约 100 ms。 */
    delay_cycles(CPUCLK_FREQ / 10U);

    /*
     * AD0/SDO 已固定接高，因此只探测固定地址 0x69。
     * 探测失败时只恢复 I2C Controller，不切换到 0x68。
     */
    for (retry = 0U;
         retry < ICM42688_INIT_RETRY_COUNT;
         retry++) {

        who = 0xFFU;

        if (icm42688_i2c_read_regs(
                ICM42688_I2C_ADDRESS,
                ICM42688_WHO_AM_I,
                &who,
                1U) &&
            (who == ICM42688_WHO_AM_I_VALUE)) {

            detected = true;
            break;
        }

        icm42688_abort_transfer();

        /* 失败后等待约 10 ms，再重新探测 0x69。 */
        delay_cycles(CPUCLK_FREQ / 100U);
    }

    if (!detected) {
        g_icm42688Address =
            ICM42688_I2C_ADDRESS_INVALID;

        icm42688_abort_transfer();
        return false;
    }

    /* 探测成功后才正式锁定固定地址。 */
    g_icm42688Address =
        ICM42688_I2C_ADDRESS;

    /*
     * 初始化顺序：
     *
     *   1. 关闭 accel/gyro；
     *   2. 配置加速度计量程和 ODR；
     *   3. 配置陀螺仪量程和 ODR；
     *   4. 开启 accel/gyro 低噪声模式。
     */
    if (!icm42688_write_reg(
            ICM42688_PWR_MGMT0,
            0x00U)) {

        goto init_failed;
    }

    /* 等待模式关闭完成约 10 ms。 */
    delay_cycles(CPUCLK_FREQ / 100U);

    if (!icm42688_write_reg(
            ICM42688_ACCEL_CONFIG0,
            ICM42688_ACCEL_16G_1KHZ)) {

        goto init_failed;
    }

    if (!icm42688_write_reg(
            ICM42688_GYRO_CONFIG0,
            ICM42688_GYRO_2000_1KHZ)) {

        goto init_failed;
    }

    if (!icm42688_write_reg(
            ICM42688_PWR_MGMT0,
            ICM42688_ACCEL_GYRO_LN)) {

        goto init_failed;
    }

    /* 等待 accel/gyro 进入低噪声模式并输出稳定数据。 */
    delay_cycles(CPUCLK_FREQ / 10U);

    /* 配置完成后再次确认固定地址仍能正常通信。 */
    who = 0xFFU;

    if ((!icm42688_read_reg(
             ICM42688_WHO_AM_I,
             &who)) ||
        (who != ICM42688_WHO_AM_I_VALUE)) {

        goto init_failed;
    }

    return true;

init_failed:
    g_icm42688Address =
        ICM42688_I2C_ADDRESS_INVALID;

    icm42688_abort_transfer();
    return false;
}

/* 读取 WHO_AM_I；通信失败返回 0xFF。 */
uint8_t icm42688_get_who_am_i(void)
{
    uint8_t who = 0xFFU;

    if (!icm42688_read_reg(
            ICM42688_WHO_AM_I,
            &who)) {

        return 0xFFU;
    }

    return who;
}

/*
 * 从 TEMP_DATA1 开始连续读取 14 字节：
 *
 *   tempH,tempL,
 *   accelXH,accelXL,
 *   accelYH,accelYL,
 *   accelZH,accelZL,
 *   gyroXH,gyroXL,
 *   gyroYH,gyroYL,
 *   gyroZH,gyroZL。
 */
void icm42688_read_raw(icm42688_raw_t *raw)
{
    uint8_t data[14];

    if (raw == 0) {
        return;
    }

    if (!icm42688_read_regs_safe(
            ICM42688_TEMP_DATA1,
            data,
            (uint16_t)sizeof(data))) {

        icm42688_clear_raw(raw);
        return;
    }

    raw->temp =
        icm42688_to_i16(data[0], data[1]);

    raw->accelX =
        icm42688_to_i16(data[2], data[3]);

    raw->accelY =
        icm42688_to_i16(data[4], data[5]);

    raw->accelZ =
        icm42688_to_i16(data[6], data[7]);

    raw->gyroX =
        icm42688_to_i16(data[8], data[9]);

    raw->gyroY =
        icm42688_to_i16(data[10], data[11]);

    raw->gyroZ =
        icm42688_to_i16(data[12], data[13]);

    /*
     * 14 字节连续读取成功，表示本帧通信正常。
     * 不再额外读取一次 WHO_AM_I，以减少总线事务。
     */
    raw->whoAmI = ICM42688_WHO_AM_I_VALUE;
}

/* 串口打印原始值。 */
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

/*
 * 等待控制器事务结束，并等待物理总线释放。
 *
 * BUSY=0：控制器当前事务已结束；
 * BUSY_BUS=0：START 到 STOP 的物理总线事务已结束。
 */
static bool icm42688_wait_bus_released(uint32_t timeout)
{
    uint32_t status;

    while (1) {
        status =
            DL_I2C_getControllerStatus(
                ICM42688_I2C_INST);

        if ((status &
             DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {

            return false;
        }

        if (((status &
              DL_I2C_CONTROLLER_STATUS_BUSY) == 0U) &&
            ((status &
              DL_I2C_CONTROLLER_STATUS_BUSY_BUS) == 0U)) {

            return true;
        }

        if (timeout == 0U) {
            return false;
        }

        timeout--;
    }
}

/*
 * 只等待控制器当前阶段完成。
 *
 * 发送寄存器地址的第一阶段故意不产生 STOP，
 * 因此这里不能等待 BUSY_BUS 清零。
 */
static bool icm42688_wait_controller_done(uint32_t timeout)
{
    uint32_t status;

    while (1) {
        status =
            DL_I2C_getControllerStatus(
                ICM42688_I2C_INST);

        if ((status &
             DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {

            return false;
        }

        if ((status &
             DL_I2C_CONTROLLER_STATUS_BUSY) == 0U) {

            return true;
        }

        if (timeout == 0U) {
            return false;
        }

        timeout--;
    }
}

/*
 * 开始一个全新的 I2C 事务前：
 *
 *   1. 复位控制器传输寄存器；
 *   2. 清空 TX/RX FIFO；
 *   3. 确认物理总线已经释放。
 */
static bool icm42688_prepare_transfer(void)
{
    DL_I2C_resetControllerTransfer(
        ICM42688_I2C_INST);

    DL_I2C_flushControllerTXFIFO(
        ICM42688_I2C_INST);

    DL_I2C_flushControllerRXFIFO(
        ICM42688_I2C_INST);

    if (!icm42688_wait_bus_released(
            ICM42688_I2C_TIMEOUT_COUNT)) {

        icm42688_abort_transfer();
        return false;
    }

    return true;
}

/*
 * 传输失败时恢复 I2C Controller。
 *
 * 固定地址版本只恢复控制器和 FIFO，绝不切换设备地址。
 */
static void icm42688_abort_transfer(void)
{
    uint32_t timeout;
    uint32_t status;

    DL_I2C_resetControllerTransfer(
        ICM42688_I2C_INST);

    DL_I2C_flushControllerTXFIFO(
        ICM42688_I2C_INST);

    DL_I2C_flushControllerRXFIFO(
        ICM42688_I2C_INST);

    delay_cycles(
        ICM42688_START_POLL_DELAY_CYCLES);

    timeout = ICM42688_I2C_TIMEOUT_COUNT;

    while (timeout > 0U) {
        status =
            DL_I2C_getControllerStatus(
                ICM42688_I2C_INST);

        if (((status &
              DL_I2C_CONTROLLER_STATUS_BUSY) == 0U) &&
            ((status &
              DL_I2C_CONTROLLER_STATUS_BUSY_BUS) == 0U)) {

            break;
        }

        timeout--;
    }

    /* 再复位一次，清除可能残留的传输和 FIFO 状态。 */
    DL_I2C_resetControllerTransfer(
        ICM42688_I2C_INST);

    DL_I2C_flushControllerTXFIFO(
        ICM42688_I2C_INST);

    DL_I2C_flushControllerRXFIFO(
        ICM42688_I2C_INST);

    delay_cycles(
        ICM42688_ABORT_SETTLE_CYCLES);
}

/*
 * 发送一组数据：
 *
 *   START + 地址(W) + 数据 + STOP
 */
static bool icm42688_i2c_write(
    uint8_t address,
    const uint8_t *data,
    uint16_t length)
{
    uint16_t written;

    if ((address != ICM42688_I2C_ADDRESS) ||
        (data == 0) ||
        (length == 0U) ||
        (length > 8U)) {

        return false;
    }

    if (!icm42688_prepare_transfer()) {
        return false;
    }

    written =
        DL_I2C_fillControllerTXFIFO(
            ICM42688_I2C_INST,
            (uint8_t *)data,
            length);

    if (written != length) {
        icm42688_abort_transfer();
        return false;
    }

    DL_I2C_startControllerTransfer(
        ICM42688_I2C_INST,
        address,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        length);

    delay_cycles(
        ICM42688_START_POLL_DELAY_CYCLES);

    if (!icm42688_wait_controller_done(
            ICM42688_I2C_TIMEOUT_COUNT)) {

        icm42688_abort_transfer();
        return false;
    }

    /* startControllerTransfer() 自动产生 STOP。 */
    if (!icm42688_wait_bus_released(
            ICM42688_I2C_TIMEOUT_COUNT)) {

        icm42688_abort_transfer();
        return false;
    }

    return true;
}

/*
 * 从指定寄存器开始连续读取：
 *
 *   START
 *   地址(W)
 *   寄存器地址
 *   REPEATED START
 *   地址(R)
 *   N 字节
 *   最后一个字节 NACK
 *   STOP
 */
static bool icm42688_i2c_read_regs(
    uint8_t address,
    uint8_t reg,
    uint8_t *data,
    uint16_t length)
{
    uint8_t registerAddress = reg;
    uint16_t received = 0U;
    uint16_t written;
    uint32_t timeout;
    uint32_t status;

    if ((address != ICM42688_I2C_ADDRESS) ||
        (data == 0) ||
        (length == 0U)) {

        return false;
    }

    if (!icm42688_prepare_transfer()) {
        return false;
    }

    written =
        DL_I2C_fillControllerTXFIFO(
            ICM42688_I2C_INST,
            &registerAddress,
            1U);

    if (written != 1U) {
        icm42688_abort_transfer();
        return false;
    }

    /*
     * 第一阶段：发送寄存器地址，但不发送 STOP，
     * 为第二阶段的 Repeated START 保留总线。
     */
    DL_I2C_startControllerTransferAdvanced(
        ICM42688_I2C_INST,
        address,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        1U,
        DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_DISABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);

    delay_cycles(
        ICM42688_START_POLL_DELAY_CYCLES);

    if (!icm42688_wait_controller_done(
            ICM42688_I2C_TIMEOUT_COUNT)) {

        icm42688_abort_transfer();
        return false;
    }

    /*
     * 第一阶段结束时 BUSY_BUS=1 是正常现象。
     * TX 和 RX 两阶段之间不能复位控制器、不能等待总线释放、
     * 也不能产生 STOP。
     */
    DL_I2C_flushControllerRXFIFO(
        ICM42688_I2C_INST);

    /*
     * 第二阶段：Repeated START + 地址(R) + N字节 + NACK + STOP。
     *
     * DriverLib 中 ACK_ENABLE 的含义是：
     * 最后一个接收字节不被自动 ACK，即以 NACK 结束读取。
     */
    DL_I2C_startControllerTransferAdvanced(
        ICM42688_I2C_INST,
        address,
        DL_I2C_CONTROLLER_DIRECTION_RX,
        length,
        DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_ENABLE,
        DL_I2C_CONTROLLER_ACK_ENABLE);

    delay_cycles(
        ICM42688_START_POLL_DELAY_CYCLES);

    timeout = ICM42688_I2C_TIMEOUT_COUNT;

    while (received < length) {
        while ((!DL_I2C_isControllerRXFIFOEmpty(
                    ICM42688_I2C_INST)) &&
               (received < length)) {

            data[received] =
                DL_I2C_receiveControllerData(
                    ICM42688_I2C_INST);

            received++;
        }

        status =
            DL_I2C_getControllerStatus(
                ICM42688_I2C_INST);

        if ((status &
             DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {

            icm42688_abort_transfer();
            return false;
        }

        if (timeout == 0U) {
            icm42688_abort_transfer();
            return false;
        }

        timeout--;
    }

    if (!icm42688_wait_controller_done(
            ICM42688_I2C_TIMEOUT_COUNT)) {

        icm42688_abort_transfer();
        return false;
    }

    /* 防止最后时刻仍有数据留在 RX FIFO。 */
    while ((!DL_I2C_isControllerRXFIFOEmpty(
                ICM42688_I2C_INST)) &&
           (received < length)) {

        data[received] =
            DL_I2C_receiveControllerData(
                ICM42688_I2C_INST);

        received++;
    }

    if (received != length) {
        icm42688_abort_transfer();
        return false;
    }

    /* 第二阶段启用了 STOP，最终等待物理总线释放。 */
    if (!icm42688_wait_bus_released(
            ICM42688_I2C_TIMEOUT_COUNT)) {

        icm42688_abort_transfer();
        return false;
    }

    return true;
}

/*
 * 使用固定地址 0x69 连续读取寄存器。
 *
 * 通信失败后只恢复控制器并继续重试 0x69，绝不切换到 0x68。
 */
static bool icm42688_read_regs_safe(
    uint8_t reg,
    uint8_t *data,
    uint16_t length)
{
    uint8_t retry;

    if ((data == 0) ||
        (length == 0U)) {

        return false;
    }

    /* 未完成初始化时不允许进行正常数据读取。 */
    if (g_icm42688Address !=
        ICM42688_I2C_ADDRESS) {

        return false;
    }

    for (retry = 0U;
         retry < ICM42688_TRANSFER_RETRY_COUNT;
         retry++) {

        if (icm42688_i2c_read_regs(
                ICM42688_I2C_ADDRESS,
                reg,
                data,
                length)) {

            return true;
        }

        icm42688_abort_transfer();

        delay_cycles(
            ICM42688_RECOVERY_DELAY_CYCLES);
    }

    return false;
}

/*
 * 使用固定地址 0x69 写一个寄存器。
 *
 * 通信失败后只恢复控制器并继续重试 0x69，绝不切换到 0x68。
 */
static bool icm42688_write_reg(
    uint8_t reg,
    uint8_t data)
{
    uint8_t tx[2];
    uint8_t retry;

    if (g_icm42688Address !=
        ICM42688_I2C_ADDRESS) {

        return false;
    }

    tx[0] = reg;
    tx[1] = data;

    for (retry = 0U;
         retry < ICM42688_TRANSFER_RETRY_COUNT;
         retry++) {

        if (icm42688_i2c_write(
                ICM42688_I2C_ADDRESS,
                tx,
                (uint16_t)sizeof(tx))) {

            return true;
        }

        icm42688_abort_transfer();

        delay_cycles(
            ICM42688_RECOVERY_DELAY_CYCLES);
    }

    return false;
}

/* 读一个寄存器，复用固定地址连续读取函数。 */
static bool icm42688_read_reg(
    uint8_t reg,
    uint8_t *data)
{
    return icm42688_read_regs_safe(
        reg,
        data,
        1U);
}

/* 把高、低字节拼接为 int16_t。 */
static int16_t icm42688_to_i16(
    uint8_t high,
    uint8_t low)
{
    return
        (int16_t)(
            ((uint16_t)high << 8) |
            (uint16_t)low);
}

/*
 * 读取失败时清零原始数据，
 * 并把 WHO_AM_I 字段设置为 0xFF 表示本帧无效。
 */
static void icm42688_clear_raw(
    icm42688_raw_t *raw)
{
    if (raw == 0) {
        return;
    }

    raw->accelX = 0;
    raw->accelY = 0;
    raw->accelZ = 0;

    raw->gyroX = 0;
    raw->gyroY = 0;
    raw->gyroZ = 0;

    raw->temp = 0;
    raw->whoAmI = 0xFFU;
}