#include "icm42688.h"

#include "ti_msp_dl_config.h"
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
 * DriverLib 接收的是 7 位地址，不要左移，不要写成 0xD0/0xD1。
 *
 * 因硬件地址脚状态不稳定，本驱动支持在 0x68 和 0x69 之间
 * 自动探测并锁定当前可用地址。
 */
#define ICM42688_I2C_ADDRESS_AD0_LOW       (0x68U)
#define ICM42688_I2C_ADDRESS_AD0_HIGH      (0x69U)
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

/* 初始化时扫描两个地址的最大尝试次数。 */
#define ICM42688_INIT_RETRY_COUNT          (5U)

/* 每轮地址选择时，0x68/0x69 各尝试的轮数。 */
#define ICM42688_ADDRESS_PROBE_ROUNDS      (2U)

/* 正常读写先在当前已确认地址上重试的次数。 */
#define ICM42688_CURRENT_ADDRESS_RETRY     (2U)

/* 错误事务恢复后，再发起下一笔事务前的等待时间。 */
#define ICM42688_RECOVERY_DELAY_CYCLES     (CPUCLK_FREQ / 1000U)
#define ICM42688_ABORT_SETTLE_CYCLES       (CPUCLK_FREQ / 10000U)

/*
 * 软件轮询超时。
 *
 * 该值是循环次数，不是毫秒。
 */
#define ICM42688_I2C_TIMEOUT_COUNT         (200000U)

/*
 * 启动控制器传输后，至少等待 3 个 I2C 功能时钟，
 * 再读取 BUSY 状态，用于规避 MSPM0 I2C_ERR_13。
 *
 * 当前已经使用 128 个 CPU 周期完成实测验证。
 */
#define ICM42688_START_POLL_DELAY_CYCLES   (128U)

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

static bool icm42688_is_valid_address(uint8_t address);
static uint8_t icm42688_get_other_address(uint8_t address);
static bool icm42688_probe_address(uint8_t address);
static bool icm42688_select_address(void);

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
 * 返回：
 *   true ：0x68 或 0x69 地址读取 WHO_AM_I=0x47，并完成寄存器配置；
 *   false：I2C 通信或传感器初始化失败。
 */
bool icm42688_init(void)
{
    uint8_t who = 0xFFU;
    uint8_t retry;
    bool detected = false;

    g_icm42688Address =
        ICM42688_I2C_ADDRESS_INVALID;

    /*
     * 等待传感器上电稳定约 100 ms。
     */
    delay_cycles(CPUCLK_FREQ / 10U);

    /*
     * 每轮都会依次探测 0x68 和 0x69。
     *
     * 错误地址产生 NACK 后，底层会复位传输状态、清空 FIFO，
     * 等待一小段时间后才探测另一个地址，避免错误状态污染
     * 下一笔事务。
     */
    for (retry = 0U;
         retry < ICM42688_INIT_RETRY_COUNT;
         retry++) {

        if (icm42688_select_address()) {
            detected = true;
            break;
        }

        icm42688_abort_transfer();
        delay_cycles(CPUCLK_FREQ / 100U);
    }

    if (!detected) {
        g_icm42688Address =
            ICM42688_I2C_ADDRESS_INVALID;
        return false;
    }

    /*
     * 再读取一次 WHO_AM_I，确认当前锁定地址仍然有效。
     */
    if ((!icm42688_read_reg(
             ICM42688_WHO_AM_I,
             &who)) ||
        (who != ICM42688_WHO_AM_I_VALUE)) {

        goto init_failed;
    }

    /*
     * 初始化顺序：
     *
     *   1. 关闭 accel/gyro；
     *   2. 配置加速度计量程和 ODR；
     *   3. 配置陀螺仪量程和 ODR；
     *   4. 开启 accel/gyro 低噪声模式。
     *
     * 每个写操作都带当前地址重试和双地址重新探测。
     */
    if (!icm42688_write_reg(
            ICM42688_PWR_MGMT0,
            0x00U)) {

        goto init_failed;
    }

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

    /*
     * 等待 accel/gyro 进入低噪声模式并输出稳定数据。
     */
    delay_cycles(CPUCLK_FREQ / 10U);

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
void icm42688_read_raw(icm42688_raw_t *raw)     //读取原始数据
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
     * 14 字节连续读取已经成功，说明本帧通信正常。
     *
     * 不再额外发起一次 WHO_AM_I 读取，避免每帧产生第二个
     * I2C 事务并降低总线稳定性。
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
 * 不能只检查 IDLE 位。
 * 实测中成功事务结束后状态可能为 0，而不一定保持 IDLE=1。
 */
static bool icm42688_wait_bus_released(uint32_t timeout)
{
    uint32_t status;

    while (1) {
        status =
            DL_I2C_getControllerStatus(
                ICM42688_I2C_INST);

        /*
         * BUSY=0：控制器当前事务已结束；
         * BUSY_BUS=0：START 到 STOP 的物理总线事务已结束。
         */
        if (((status &
              DL_I2C_CONTROLLER_STATUS_BUSY) == 0U) &&
            ((status &
              DL_I2C_CONTROLLER_STATUS_BUSY_BUS) == 0U)) {

            return
                (status &
                 DL_I2C_CONTROLLER_STATUS_ERROR) == 0U;
        }

        if ((status &
             DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            return false;
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
 * 第一阶段发送寄存器地址时故意不产生 STOP，
 * 所以此处不能等待 BUSY_BUS 清零。
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
 *   1. 复位控制器传输状态；
 *   2. 清空 TX/RX FIFO；
 *   3. 确认物理总线已释放。
 */
static bool icm42688_prepare_transfer(void)
{
    DL_I2C_resetControllerTransfer(
        ICM42688_I2C_INST);

    DL_I2C_flushControllerTXFIFO(
        ICM42688_I2C_INST);

    DL_I2C_flushControllerRXFIFO(
        ICM42688_I2C_INST);

    if (icm42688_wait_bus_released(
            ICM42688_I2C_TIMEOUT_COUNT) == false) {

        icm42688_abort_transfer();
        return false;
    }

    return true;
}

/*
 * 传输失败时恢复 I2C Controller。
 *
 * 错误地址通常会产生 NACK。这里不仅复位事务和清空 FIFO，
 * 还等待 BUSY/BUSY_BUS 尽量释放，并在返回前留出恢复时间。
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

    /*
     * 再复位一次，清除恢复过程中可能残留的错误和 FIFO 状态。
     */
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

    if ((data == 0) ||
        (length == 0U) ||
        (length > 8U)) {
        return false;
    }

    if (icm42688_prepare_transfer() == false) {
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

    if (icm42688_wait_controller_done(
            ICM42688_I2C_TIMEOUT_COUNT) == false) {

        icm42688_abort_transfer();
        return false;
    }

    /*
     * startControllerTransfer() 自动产生 STOP，
     * 所以还要等待 BUSY_BUS 清零。
     */
    if (icm42688_wait_bus_released(
            ICM42688_I2C_TIMEOUT_COUNT) == false) {

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

    if ((data == 0) ||
        (length == 0U)) {
        return false;
    }

    if (icm42688_prepare_transfer() == false) {
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

    /* Write register address, then keep the bus active for repeated START. */
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

    if (icm42688_wait_controller_done(
            ICM42688_I2C_TIMEOUT_COUNT) == false) {

        icm42688_abort_transfer();
        return false;
    }

    DL_I2C_flushControllerRXFIFO(
        ICM42688_I2C_INST);

    /*
     * 第一阶段结束时 BUSY_BUS=1 是正常现象。
     *
     * TX 与 RX 之间严禁：
     *
     *   1. resetControllerTransfer；
     *   2. 等待 BUSY_BUS 清零；
     *   3. 产生 STOP。
     */

    /*
     * Read phase:
     *
     * START + address(R) + N bytes + final NACK + STOP
     */
    DL_I2C_startControllerTransferAdvanced(
        ICM42688_I2C_INST,
        address,
        DL_I2C_CONTROLLER_DIRECTION_RX,
        length,
        DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_ENABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);

    delay_cycles(
        ICM42688_START_POLL_DELAY_CYCLES);

    timeout = ICM42688_I2C_TIMEOUT_COUNT;

    while (received < length) {
        while ((DL_I2C_isControllerRXFIFOEmpty(
                    ICM42688_I2C_INST) == false) &&
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

    if (icm42688_wait_controller_done(
            ICM42688_I2C_TIMEOUT_COUNT) == false) {

        icm42688_abort_transfer();
        return false;
    }

    while ((DL_I2C_isControllerRXFIFOEmpty(
                ICM42688_I2C_INST) == false) &&
           (received < length)) {

        data[received] =
            DL_I2C_receiveControllerData(
                ICM42688_I2C_INST);

        received++;
    }

    DL_I2C_flushControllerTXFIFO(
        ICM42688_I2C_INST);

    if (received != length) {
        icm42688_abort_transfer();
        return false;
    }

    /*
     * 第二阶段启用了 STOP，所以最终等待物理总线释放。
     */
    if (icm42688_wait_bus_released(
            ICM42688_I2C_TIMEOUT_COUNT) == false) {

        icm42688_abort_transfer();
        return false;
    }

    return true;
}

/*
 * 判断地址是否是 ICM42688 的两个合法 7 位地址之一。
 */
static bool icm42688_is_valid_address(
    uint8_t address)
{
    return
        (address == ICM42688_I2C_ADDRESS_AD0_LOW) ||
        (address == ICM42688_I2C_ADDRESS_AD0_HIGH);
}

/*
 * 返回另一个候选地址。
 */
static uint8_t icm42688_get_other_address(
    uint8_t address)
{
    if (address == ICM42688_I2C_ADDRESS_AD0_LOW) {
        return ICM42688_I2C_ADDRESS_AD0_HIGH;
    }

    return ICM42688_I2C_ADDRESS_AD0_LOW;
}

/*
 * 只探测指定地址：
 *
 *   读取 WHO_AM_I；
 *   只有通信成功且返回 0x47，才认为该地址有效。
 */
static bool icm42688_probe_address(
    uint8_t address)
{
    uint8_t who = 0xFFU;

    if (!icm42688_is_valid_address(address)) {
        return false;
    }

    if (!icm42688_i2c_read_regs(
            address,
            ICM42688_WHO_AM_I,
            &who,
            1U)) {

        icm42688_abort_transfer();
        return false;
    }

    return who == ICM42688_WHO_AM_I_VALUE;
}

/*
 * 自动选择当前可用地址。
 *
 * 如果已经保存了一个地址，优先探测该地址；
 * 否则先探测 0x68，再探测 0x69。
 *
 * 第一个地址失败后，会完整恢复 I2C Controller，
 * 再探测第二个地址。
 */
static bool icm42688_select_address(void)
{
    uint8_t firstAddress;
    uint8_t secondAddress;
    uint8_t round;

    if (icm42688_is_valid_address(
            g_icm42688Address)) {

        firstAddress = g_icm42688Address;
    } else {
        firstAddress =
            ICM42688_I2C_ADDRESS_AD0_LOW;
    }

    secondAddress =
        icm42688_get_other_address(
            firstAddress);

    for (round = 0U;
         round < ICM42688_ADDRESS_PROBE_ROUNDS;
         round++) {

        if (icm42688_probe_address(
                firstAddress)) {

            g_icm42688Address =
                firstAddress;
            return true;
        }

        icm42688_abort_transfer();
        delay_cycles(
            ICM42688_RECOVERY_DELAY_CYCLES);

        if (icm42688_probe_address(
                secondAddress)) {

            g_icm42688Address =
                secondAddress;
            return true;
        }

        icm42688_abort_transfer();
        delay_cycles(CPUCLK_FREQ / 200U);
    }

    g_icm42688Address =
        ICM42688_I2C_ADDRESS_INVALID;

    return false;
}

/*
 * 带地址容错的连续读取。
 *
 * 正常情况下只使用当前已确认地址。
 * 当前地址连续失败后，才重新探测 0x68/0x69。
 */
static bool icm42688_read_regs_safe(
    uint8_t reg,
    uint8_t *data,
    uint16_t length)
{
    uint8_t retry;
    uint8_t failedAddress;

    if ((data == 0) ||
        (length == 0U)) {

        return false;
    }

    if (!icm42688_is_valid_address(
            g_icm42688Address)) {

        if (!icm42688_select_address()) {
            return false;
        }
    }

    /*
     * 优先使用当前地址重试。
     */
    for (retry = 0U;
         retry < ICM42688_CURRENT_ADDRESS_RETRY;
         retry++) {

        if (icm42688_i2c_read_regs(
                g_icm42688Address,
                reg,
                data,
                length)) {

            return true;
        }

        icm42688_abort_transfer();
        delay_cycles(
            ICM42688_RECOVERY_DELAY_CYCLES);
    }

    /*
     * 当前地址连续失败，优先从另一个地址开始重新探测。
     */
    failedAddress = g_icm42688Address;

    g_icm42688Address =
        icm42688_get_other_address(
            failedAddress);

    if (!icm42688_select_address()) {
        return false;
    }

    /*
     * 地址重新确认后，再执行原来的连续读取。
     */
    for (retry = 0U;
         retry < ICM42688_CURRENT_ADDRESS_RETRY;
         retry++) {

        if (icm42688_i2c_read_regs(
                g_icm42688Address,
                reg,
                data,
                length)) {

            return true;
        }

        icm42688_abort_transfer();
        delay_cycles(
            ICM42688_RECOVERY_DELAY_CYCLES);
    }

    g_icm42688Address =
        ICM42688_I2C_ADDRESS_INVALID;

    return false;
}

/*
 * 写一个寄存器，带当前地址重试和双地址重新探测。
 */
static bool icm42688_write_reg(
    uint8_t reg,
    uint8_t data)
{
    uint8_t tx[2];
    uint8_t retry;
    uint8_t failedAddress;

    tx[0] = reg;
    tx[1] = data;

    if (!icm42688_is_valid_address(
            g_icm42688Address)) {

        if (!icm42688_select_address()) {
            return false;
        }
    }

    /*
     * 优先使用当前已确认地址。
     */
    for (retry = 0U;
         retry < ICM42688_CURRENT_ADDRESS_RETRY;
         retry++) {

        if (icm42688_i2c_write(
                g_icm42688Address,
                tx,
                (uint16_t)sizeof(tx))) {

            return true;
        }

        icm42688_abort_transfer();
        delay_cycles(
            ICM42688_RECOVERY_DELAY_CYCLES);
    }

    /*
     * 当前地址连续写失败，优先探测另一个地址。
     */
    failedAddress = g_icm42688Address;

    g_icm42688Address =
        icm42688_get_other_address(
            failedAddress);

    if (!icm42688_select_address()) {
        return false;
    }

    /*
     * 地址重新确认后，再重试寄存器写入。
     */
    for (retry = 0U;
         retry < ICM42688_CURRENT_ADDRESS_RETRY;
         retry++) {

        if (icm42688_i2c_write(
                g_icm42688Address,
                tx,
                (uint16_t)sizeof(tx))) {

            return true;
        }

        icm42688_abort_transfer();
        delay_cycles(
            ICM42688_RECOVERY_DELAY_CYCLES);
    }

    g_icm42688Address =
        ICM42688_I2C_ADDRESS_INVALID;

    return false;
}

/* 读一个寄存器，复用带地址容错的连续读取。 */
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
 * 读取失败时清零原始数据并把 WHO_AM_I 设置为 0xFF。
 */
static void icm42688_clear_raw(
    icm42688_raw_t *raw)
{
    raw->accelX = 0;
    raw->accelY = 0;
    raw->accelZ = 0;

    raw->gyroX = 0;
    raw->gyroY = 0;
    raw->gyroZ = 0;

    raw->temp = 0;
    raw->whoAmI = 0xFFU;
}