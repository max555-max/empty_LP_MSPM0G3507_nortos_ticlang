// #include "icm42688.h"

// #include "ti_msp_dl_config.h"
// #include "vofa.h"

// /*
//  * SysConfig 中请把硬件 I2C 实例命名为：
//  *
//  *     I2C_ICM42688
//  *
//  * 这样 SysConfig 会生成 I2C_ICM42688_INST。
//  */
// #ifndef I2C_ICM42688_INST
// #error "请在 SysConfig 中添加 I2C Controller,并把实例名称设置为 I2C_ICM42688"
// #endif

// #define ICM42688_I2C_INST              (I2C_ICM42688_INST)

// /* ICM42688 Bank 0 常用寄存器。 */
// #define ICM42688_TEMP_DATA1            (0x1DU)
// #define ICM42688_PWR_MGMT0             (0x4EU)
// #define ICM42688_GYRO_CONFIG0          (0x4FU)
// #define ICM42688_ACCEL_CONFIG0         (0x50U)
// #define ICM42688_WHO_AM_I              (0x75U)
// #define ICM42688_WHO_AM_I_VALUE        (0x47U)

// /*
//  * ICM42688 7 位 I2C 地址：
//  *   AD0 = 0 -> 0x68
//  *   AD0 = 1 -> 0x69
//  *
//  * 初始化时会自动尝试两个地址。
//  */
// #define ICM42688_I2C_ADDRESS_AD0_LOW   (0x68U)
// #define ICM42688_I2C_ADDRESS_AD0_HIGH  (0x69U)
// #define ICM42688_I2C_ADDRESS_INVALID   (0xFFU)

// /*
//  * 当前传感器配置与原 SPI 版本保持一致：
//  *   accel：±16 g，1 kHz；
//  *   gyro ：±2000 dps，1 kHz；
//  *   accel/gyro：低噪声模式。
//  */
// #define ICM42688_ACCEL_16G_1KHZ        (0x06U)
// #define ICM42688_GYRO_2000_1KHZ        (0x06U)
// #define ICM42688_ACCEL_GYRO_LN         (0x0FU)

// /* 初始化探测次数和每次探测间隔。 */
// #define ICM42688_DETECT_RETRY_COUNT    (50U)

// /*
//  * 轮询超时计数。
//  * 该值不是毫秒，而是 while 循环最大次数，用于避免总线异常时死循环。
//  */
// #define ICM42688_I2C_TIMEOUT_COUNT     (200000U)

// /* MFCLK 在 MSPM0G3507 上为 4 MHz。 */
// #define ICM42688_MFCLK_FREQ_HZ         (4000000U)

// static uint8_t g_icm42688Address = ICM42688_I2C_ADDRESS_INVALID;

// /*
//  * TI I2C_ERR_13 要求：启动控制器传输后，至少等待 3 个 I2C 功能时钟，
//  * 再轮询 BUSY 状态。该变量在初始化时依据 I2C 时钟源和分频计算。
//  */
// static uint32_t g_i2cStartPollDelayCycles = 32U;

// static void icm42688_calculate_start_poll_delay(void);
// static bool icm42688_wait_bus_idle(uint32_t timeout);
// static bool icm42688_wait_controller_done(uint32_t timeout);
// static void icm42688_abort_transfer(void);

// static bool icm42688_i2c_write(
//     uint8_t address,
//     const uint8_t *data,
//     uint16_t length);

// static bool icm42688_i2c_read_regs(
//     uint8_t address,
//     uint8_t reg,
//     uint8_t *data,
//     uint16_t length);

// static bool icm42688_probe_address(uint8_t address);
// static bool icm42688_write_reg(uint8_t reg, uint8_t data);
// static bool icm42688_read_reg(uint8_t reg, uint8_t *data);
// static int16_t icm42688_to_i16(uint8_t high, uint8_t low);
// static void icm42688_clear_raw(icm42688_raw_t *raw);

// /*
//  * 初始化 ICM42688。
//  *
//  * 返回：
//  *   true ：在 0x68 或 0x69 地址读到 WHO_AM_I = 0x47，
//  *          并且寄存器配置写入成功；
//  *   false：I2C 地址、接线、上拉、CS/AD0 或器件初始化存在问题。
//  */
// bool icm42688_init(void)
// {
//     static const uint8_t addressList[2] = {
//         ICM42688_I2C_ADDRESS_AD0_LOW,
//         ICM42688_I2C_ADDRESS_AD0_HIGH
//     };

//     uint8_t who = 0xFFU;
//     bool detected = false;

//     g_icm42688Address = ICM42688_I2C_ADDRESS_INVALID;
//     icm42688_calculate_start_poll_delay();

//     /* 等待传感器上电稳定约 20 ms。 */
//     delay_cycles(CPUCLK_FREQ / 50U);

//     /*
//      * 依次尝试 0x68 和 0x69。
//      * DriverLib 接收的是 7 位地址，不要再左移一位。
//      */
//     for (uint8_t retry = 0U;
//          (retry < ICM42688_DETECT_RETRY_COUNT) && (detected == false);
//          retry++) {

//         for (uint8_t i = 0U; i < 2U; i++) {
//             if (icm42688_probe_address(addressList[i]) == false) {
//                 continue;
//             }

//             g_icm42688Address = addressList[i];

//             if ((icm42688_read_reg(ICM42688_WHO_AM_I, &who) == true) &&
//                 (who == ICM42688_WHO_AM_I_VALUE)) {
//                 detected = true;
//                 break;
//             }

//             g_icm42688Address = ICM42688_I2C_ADDRESS_INVALID;
//         }

//         if (detected == false) {
//             delay_cycles(CPUCLK_FREQ / 100U);
//         }
//     }

//     if (detected == false) {
//         return false;
//     }

//     /*
//      * 初始化顺序与原 SPI 版本保持一致：
//      *   1. 关闭 accel/gyro；
//      *   2. 配置量程和 ODR；
//      *   3. 打开 accel/gyro 低噪声模式。
//      */
//     if (icm42688_write_reg(ICM42688_PWR_MGMT0, 0x00U) == false) {
//         return false;
//     }

//     delay_cycles(CPUCLK_FREQ / 100U);

//     if (icm42688_write_reg(
//             ICM42688_ACCEL_CONFIG0,
//             ICM42688_ACCEL_16G_1KHZ) == false) {
//         return false;
//     }

//     if (icm42688_write_reg(
//             ICM42688_GYRO_CONFIG0,
//             ICM42688_GYRO_2000_1KHZ) == false) {
//         return false;
//     }

//     if (icm42688_write_reg(
//             ICM42688_PWR_MGMT0,
//             ICM42688_ACCEL_GYRO_LN) == false) {
//         return false;
//     }

//     /* 等待 accel/gyro 进入低噪声模式。 */
//     delay_cycles(CPUCLK_FREQ / 10U);

//     return icm42688_get_who_am_i() == ICM42688_WHO_AM_I_VALUE;
// }

// /* 读取 WHO_AM_I；通信失败返回 0xFF。 */
// uint8_t icm42688_get_who_am_i(void)
// {
//     uint8_t who = 0xFFU;

//     if (g_icm42688Address == ICM42688_I2C_ADDRESS_INVALID) {
//         return 0xFFU;
//     }

//     if (icm42688_read_reg(ICM42688_WHO_AM_I, &who) == false) {
//         return 0xFFU;
//     }

//     return who;
// }

// /*
//  * 从 TEMP_DATA1 开始连续读取 14 字节：
//  *   tempH,tempL,
//  *   accelXH,accelXL, accelYH,accelYL, accelZH,accelZL,
//  *   gyroXH,gyroXL, gyroYH,gyroYL, gyroZH,gyroZL。
//  */
// void icm42688_read_raw(icm42688_raw_t *raw)
// {
//     uint8_t data[14];

//     if (raw == 0) {
//         return;
//     }

//     if ((g_icm42688Address == ICM42688_I2C_ADDRESS_INVALID) ||
//         (icm42688_i2c_read_regs(
//              g_icm42688Address,
//              ICM42688_TEMP_DATA1,
//              data,
//              (uint16_t) sizeof(data)) == false)) {
//         icm42688_clear_raw(raw);
//         raw->whoAmI = 0xFFU;
//         return;
//     }

//     raw->temp   = icm42688_to_i16(data[0], data[1]);
//     raw->accelX = icm42688_to_i16(data[2], data[3]);
//     raw->accelY = icm42688_to_i16(data[4], data[5]);
//     raw->accelZ = icm42688_to_i16(data[6], data[7]);
//     raw->gyroX  = icm42688_to_i16(data[8], data[9]);
//     raw->gyroY  = icm42688_to_i16(data[10], data[11]);
//     raw->gyroZ  = icm42688_to_i16(data[12], data[13]);
//     raw->whoAmI = icm42688_get_who_am_i();
// }

// /* 串口打印格式与原 SPI 版本保持一致。 */
// void icm42688_print_raw(const icm42688_raw_t *raw)
// {
//     if (raw == 0) {
//         return;
//     }

//     uart0_send_string("ICM:");
//     uart0_send_int(raw->accelX);
//     uart0_send_byte(',');
//     uart0_send_int(raw->accelY);
//     uart0_send_byte(',');
//     uart0_send_int(raw->accelZ);
//     uart0_send_byte(',');
//     uart0_send_int(raw->gyroX);
//     uart0_send_byte(',');
//     uart0_send_int(raw->gyroY);
//     uart0_send_byte(',');
//     uart0_send_int(raw->gyroZ);
//     uart0_send_byte(',');
//     uart0_send_int(raw->temp);
//     uart0_send_byte(',');
//     uart0_send_int(raw->whoAmI);
//     uart0_send_string("\r\n");
// }

// /*
//  * 计算启动传输后等待 3 个 I2C 功能时钟所需的 CPU 周期数，
//  * 用于规避 MSPM0 I2C_ERR_13。
//  */
// static void icm42688_calculate_start_poll_delay(void)
// {
//     DL_I2C_ClockConfig clockConfig;
//     uint32_t i2cClockHz;
//     uint32_t cpuCyclesPerI2cClock;
//     uint32_t divider;

//     DL_I2C_getClockConfig(ICM42688_I2C_INST, &clockConfig);

//     if (clockConfig.clockSel == DL_I2C_CLOCK_MFCLK) {
//         i2cClockHz = ICM42688_MFCLK_FREQ_HZ;
//     } else {
//         /*
//          * SysConfig 通常使用 BUSCLK。
//          * 本工程 BUSCLK 与 CPUCLK 同源时可按 CPUCLK_FREQ 计算。
//          */
//         i2cClockHz = CPUCLK_FREQ;
//     }

//     cpuCyclesPerI2cClock = CPUCLK_FREQ / i2cClockHz;
//     if (cpuCyclesPerI2cClock == 0U) {
//         cpuCyclesPerI2cClock = 1U;
//     }

//     /*
//      * divideRatio 的寄存器编码为 0~7，实际分频为编码值 + 1。
//      */
//     divider = (uint32_t) clockConfig.divideRatio + 1U;

//     g_i2cStartPollDelayCycles =
//         3U * divider * cpuCyclesPerI2cClock;

//     /* 留少量裕量，且确保不会为 0。 */
//     g_i2cStartPollDelayCycles += 8U;
// }

// /* 等待总线进入 IDLE 状态。 */
// static bool icm42688_wait_bus_idle(uint32_t timeout)
// {
//     while ((DL_I2C_getControllerStatus(ICM42688_I2C_INST) &
//             DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {

//         if ((DL_I2C_getControllerStatus(ICM42688_I2C_INST) &
//              DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
//             return false;
//         }

//         if (timeout == 0U) {
//             return false;
//         }
//         timeout--;
//     }

//     return true;
// }

// /* 等待本次控制器传输完成。 */
// static bool icm42688_wait_controller_done(uint32_t timeout)
// {
//     while ((DL_I2C_getControllerStatus(ICM42688_I2C_INST) &
//             DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) {

//         if ((DL_I2C_getControllerStatus(ICM42688_I2C_INST) &
//              DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
//             return false;
//         }

//         if (timeout == 0U) {
//             return false;
//         }
//         timeout--;
//     }

//     return
//         (DL_I2C_getControllerStatus(ICM42688_I2C_INST) &
//          DL_I2C_CONTROLLER_STATUS_ERROR) == 0U;
// }

// /* 传输失败时复位控制器传输状态并清空 FIFO。 */
// static void icm42688_abort_transfer(void)
// {
//     DL_I2C_resetControllerTransfer(ICM42688_I2C_INST);
//     DL_I2C_flushControllerTXFIFO(ICM42688_I2C_INST);
//     DL_I2C_flushControllerRXFIFO(ICM42688_I2C_INST);
// }

// /* 发送一组数据，自动产生 START 和 STOP。 */
// static bool icm42688_i2c_write(
//     uint8_t address,
//     const uint8_t *data,
//     uint16_t length)
// {
//     if ((data == 0) || (length == 0U) || (length > 8U)) {
//         return false;
//     }

//     DL_I2C_resetControllerTransfer(ICM42688_I2C_INST);
//     DL_I2C_flushControllerTXFIFO(ICM42688_I2C_INST);
//     DL_I2C_flushControllerRXFIFO(ICM42688_I2C_INST);

//     if (icm42688_wait_bus_idle(ICM42688_I2C_TIMEOUT_COUNT) == false) {
//         icm42688_abort_transfer();
//         return false;
//     }

//     (void) DL_I2C_fillControllerTXFIFO(
//         ICM42688_I2C_INST,
//         (uint8_t *) data,
//         length);

//     DL_I2C_startControllerTransfer(
//         ICM42688_I2C_INST,
//         address,
//         DL_I2C_CONTROLLER_DIRECTION_TX,
//         length);

//     delay_cycles(g_i2cStartPollDelayCycles);

//     if (icm42688_wait_controller_done(ICM42688_I2C_TIMEOUT_COUNT) == false) {
//         icm42688_abort_transfer();
//         return false;
//     }

//     if (icm42688_wait_bus_idle(ICM42688_I2C_TIMEOUT_COUNT) == false) {
//         icm42688_abort_transfer();
//         return false;
//     }

//     return true;
// }

// /*
//  * 从寄存器开始连续读取：
//  *
//  *   START + 地址(W) + 寄存器
//  *   REPEATED START + 地址(R) + N 字节 + NACK + STOP
//  */
// static bool icm42688_i2c_read_regs(
//     uint8_t address,
//     uint8_t reg,
//     uint8_t *data,
//     uint16_t length)
// {
//     uint16_t received = 0U;
//     uint32_t timeout = ICM42688_I2C_TIMEOUT_COUNT;
//     uint8_t registerAddress = reg;

//     if ((data == 0) || (length == 0U)) {
//         return false;
//     }

//     DL_I2C_resetControllerTransfer(ICM42688_I2C_INST);
//     DL_I2C_flushControllerTXFIFO(ICM42688_I2C_INST);
//     DL_I2C_flushControllerRXFIFO(ICM42688_I2C_INST);

//     if (icm42688_wait_bus_idle(ICM42688_I2C_TIMEOUT_COUNT) == false) {
//         icm42688_abort_transfer();
//         return false;
//     }

//     /*
//      * 第一阶段：发送寄存器地址，但不产生 STOP。
//      */
//     (void) DL_I2C_fillControllerTXFIFO(
//         ICM42688_I2C_INST,
//         &registerAddress,
//         1U);

//     DL_I2C_startControllerTransferAdvanced(
//         ICM42688_I2C_INST,
//         address,
//         DL_I2C_CONTROLLER_DIRECTION_TX,
//         1U,
//         DL_I2C_CONTROLLER_START_ENABLE,
//         DL_I2C_CONTROLLER_STOP_DISABLE,
//         DL_I2C_CONTROLLER_ACK_ENABLE);

//     delay_cycles(g_i2cStartPollDelayCycles);

//     if (icm42688_wait_controller_done(ICM42688_I2C_TIMEOUT_COUNT) == false) {
//         icm42688_abort_transfer();
//         return false;
//     }

//     /*
//      * 第二阶段：产生重复起始信号并读取数据，最后自动 NACK + STOP。
//      */
//     DL_I2C_flushControllerRXFIFO(ICM42688_I2C_INST);

//     DL_I2C_startControllerTransferAdvanced(
//         ICM42688_I2C_INST,
//         address,
//         DL_I2C_CONTROLLER_DIRECTION_RX,
//         length,
//         DL_I2C_CONTROLLER_START_ENABLE,
//         DL_I2C_CONTROLLER_STOP_ENABLE,
//         DL_I2C_CONTROLLER_ACK_DISABLE);

//     delay_cycles(g_i2cStartPollDelayCycles);

//     while (received < length) {
//         while ((DL_I2C_isControllerRXFIFOEmpty(ICM42688_I2C_INST) == false) &&
//                (received < length)) {
//             data[received] =
//                 DL_I2C_receiveControllerData(ICM42688_I2C_INST);
//             received++;
//         }

//         if ((DL_I2C_getControllerStatus(ICM42688_I2C_INST) &
//              DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
//             icm42688_abort_transfer();
//             return false;
//         }

//         if (timeout == 0U) {
//             icm42688_abort_transfer();
//             return false;
//         }
//         timeout--;
//     }

//     if (icm42688_wait_controller_done(ICM42688_I2C_TIMEOUT_COUNT) == false) {
//         icm42688_abort_transfer();
//         return false;
//     }

//     if (icm42688_wait_bus_idle(ICM42688_I2C_TIMEOUT_COUNT) == false) {
//         icm42688_abort_transfer();
//         return false;
//     }

//     return true;
// }

// /*
//  * 地址探测只发送 WHO_AM_I 寄存器地址并产生 STOP。
//  * 这样即使地址不存在，也不会把总线停留在“无 STOP”的中间状态。
//  */
// static bool icm42688_probe_address(uint8_t address)
// {
//     uint8_t reg = ICM42688_WHO_AM_I;

//     return icm42688_i2c_write(address, &reg, 1U);
// }

// /* 写单个寄存器。 */
// static bool icm42688_write_reg(uint8_t reg, uint8_t data)
// {
//     uint8_t tx[2];

//     if (g_icm42688Address == ICM42688_I2C_ADDRESS_INVALID) {
//         return false;
//     }

//     tx[0] = reg;
//     tx[1] = data;

//     return icm42688_i2c_write(
//         g_icm42688Address,
//         tx,
//         (uint16_t) sizeof(tx));
// }

// /* 读单个寄存器。 */
// static bool icm42688_read_reg(uint8_t reg, uint8_t *data)
// {
//     if ((g_icm42688Address == ICM42688_I2C_ADDRESS_INVALID) ||
//         (data == 0)) {
//         return false;
//     }

//     return icm42688_i2c_read_regs(
//         g_icm42688Address,
//         reg,
//         data,
//         1U);
// }

// /* 把高低字节拼成 int16_t。 */
// static int16_t icm42688_to_i16(uint8_t high, uint8_t low)
// {
//     return (int16_t) (((uint16_t) high << 8) | (uint16_t) low);
// }

// /* 读取失败时清零原始数据，whoAmI 由调用处设置为 0xFF。 */
// static void icm42688_clear_raw(icm42688_raw_t *raw)
// {
//     raw->accelX = 0;
//     raw->accelY = 0;
//     raw->accelZ = 0;
//     raw->gyroX = 0;
//     raw->gyroY = 0;
//     raw->gyroZ = 0;
//     raw->temp = 0;
//     raw->whoAmI = 0xFFU;
// }

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
 * 当前硬件已经实测确认：
 *
 *     7 位 I2C 地址 = 0x68
 *
 * DriverLib 接收的是 7 位地址，不要左移，不要写成 0xD0。
 */
#define ICM42688_I2C_ADDRESS               (0x68U)
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

/* 初始化时读取 WHO_AM_I 的最大尝试次数。 */
#define ICM42688_INIT_RETRY_COUNT          (20U)

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

static bool icm42688_write_reg(uint8_t reg, uint8_t data);
static bool icm42688_read_reg(uint8_t reg, uint8_t *data);

static int16_t icm42688_to_i16(uint8_t high, uint8_t low);
static void icm42688_clear_raw(icm42688_raw_t *raw);

/*
 * 初始化 ICM42688。
 *
 * 返回：
 *   true ：0x68 地址读取 WHO_AM_I=0x47，并完成寄存器配置；
 *   false：I2C 通信或传感器初始化失败。
 */
bool icm42688_init(void)
{
    uint8_t who = 0xFFU;
    bool detected = false;
    uint8_t retry;

    g_icm42688Address = ICM42688_I2C_ADDRESS;

    /*
     * 等待传感器上电稳定约 100 ms。
     */
    delay_cycles(CPUCLK_FREQ / 10U);

    /*
     * 固定测试已经确认本模块地址为 0x68。
     * 不再先访问错误地址 0x69，避免地址 NACK 干扰后续事务。
     */
    for (retry = 0U;
         retry < ICM42688_INIT_RETRY_COUNT;
         retry++) {

        if ((icm42688_read_reg(
                 ICM42688_WHO_AM_I,
                 &who) == true) &&
            (who == ICM42688_WHO_AM_I_VALUE)) {

            detected = true;
            break;
        }

        /*
         * 每次失败后留出约 10 ms，再重新发起完整事务。
         */
        delay_cycles(CPUCLK_FREQ / 100U);
    }

    if (detected == false) {
        g_icm42688Address =
            ICM42688_I2C_ADDRESS_INVALID;
        return false;
    }

    /*
     * 初始化顺序：
     *
     *   1. 关闭 accel/gyro；
     *   2. 配置加速度计量程和 ODR；
     *   3. 配置陀螺仪量程和 ODR；
     *   4. 开启 accel/gyro 低噪声模式。
     */
    if (icm42688_write_reg(
            ICM42688_PWR_MGMT0,
            0x00U) == false) {
        goto init_failed;
    }

    delay_cycles(CPUCLK_FREQ / 100U);

    if (icm42688_write_reg(
            ICM42688_ACCEL_CONFIG0,
            ICM42688_ACCEL_16G_1KHZ) == false) {
        goto init_failed;
    }

    if (icm42688_write_reg(
            ICM42688_GYRO_CONFIG0,
            ICM42688_GYRO_2000_1KHZ) == false) {
        goto init_failed;
    }

    if (icm42688_write_reg(
            ICM42688_PWR_MGMT0,
            ICM42688_ACCEL_GYRO_LN) == false) {
        goto init_failed;
    }

    /*
     * 等待 accel/gyro 进入低噪声模式并输出稳定数据。
     */
    delay_cycles(CPUCLK_FREQ / 10U);

    if (icm42688_get_who_am_i() !=
        ICM42688_WHO_AM_I_VALUE) {
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

    if (g_icm42688Address ==
        ICM42688_I2C_ADDRESS_INVALID) {
        return 0xFFU;
    }

    if (icm42688_read_reg(
            ICM42688_WHO_AM_I,
            &who) == false) {
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

    if ((g_icm42688Address ==
         ICM42688_I2C_ADDRESS_INVALID) ||
        (icm42688_i2c_read_regs(
             g_icm42688Address,
             ICM42688_TEMP_DATA1,
             data,
             (uint16_t)sizeof(data)) == false)) {

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

/* 传输失败时复位事务状态并清空 FIFO。 */
static void icm42688_abort_transfer(void)
{
    DL_I2C_resetControllerTransfer(
        ICM42688_I2C_INST);

    DL_I2C_flushControllerTXFIFO(
        ICM42688_I2C_INST);

    DL_I2C_flushControllerRXFIFO(
        ICM42688_I2C_INST);
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
    uint16_t written;
    uint16_t received = 0U;
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

    /*
     * 第一阶段：
     *
     * START + 地址(W) + 寄存器地址
     *
     * 不产生 STOP，保留总线以便下一阶段产生 Repeated START。
     *
     * 该 ACK 参数组合已经通过 WHO_AM_I=0x47 实测验证。
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

    if (icm42688_wait_controller_done(
            ICM42688_I2C_TIMEOUT_COUNT) == false) {

        icm42688_abort_transfer();
        return false;
    }

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
     * 第二阶段：
     *
     * REPEATED START + 地址(R) + N 字节 + NACK + STOP
     *
     * ACK_ENABLE 是当前工程中已经实测成功读取 0x47 的组合。
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

/* 写一个寄存器。 */
static bool icm42688_write_reg(
    uint8_t reg,
    uint8_t data)
{
    uint8_t tx[2];

    if (g_icm42688Address ==
        ICM42688_I2C_ADDRESS_INVALID) {
        return false;
    }

    tx[0] = reg;
    tx[1] = data;

    return icm42688_i2c_write(
        g_icm42688Address,
        tx,
        (uint16_t)sizeof(tx));
}

/* 读一个寄存器。 */
static bool icm42688_read_reg(
    uint8_t reg,
    uint8_t *data)
{
    if ((g_icm42688Address ==
         ICM42688_I2C_ADDRESS_INVALID) ||
        (data == 0)) {
        return false;
    }

    return icm42688_i2c_read_regs(
        g_icm42688Address,
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