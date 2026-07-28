/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */

#include "ti_msp_dl_config.h"

#include "delay.h"
#include "attitude.h"
#include "icm42688.h"
#include "oled.h"
#include "vofa.h"

/* 姿态更新和调试输出周期。 */
#define IMU_UPDATE_PERIOD_MS             (10U)

/* 陀螺仪零偏校准采样次数。 */
#define IMU_GYRO_CALIB_SAMPLE_COUNT      (1000U)

/*
 * 当前 ICM42688 驱动配置为 ±2000 dps。
 * 原始值转换为 °/s：
 *     gyroDps = raw × 2000 / 32768
 */
#define IMU_GYRO_DPS_PER_LSB             (2000.0f / 32768.0f)

/* 初始化失败或掉线后的重新尝试周期。 */
#define IMU_RETRY_PERIOD_MS              (1000U)

/* ICM42688 正常 WHO_AM_I。 */
#define ICM42688_EXPECTED_WHO_AM_I       (0x47U)

/* 姿态解算允许的 dt 范围。 */
#define IMU_DT_DEFAULT_S                 (0.010f)
#define IMU_DT_MIN_S                     (0.001f)
#define IMU_DT_MAX_S                     (0.050f)

#define MENU_ITEM_COUNT                  (5U)
#define MENU_KEY_DEBOUNCE_MS             (30U)

#define MENU_KEY_PORT                    (GPIOA)
#define MENU_KEY_UP_PIN                  (DL_GPIO_PIN_9)
#define MENU_KEY_UP_IOMUX                (IOMUX_PINCM20)
#define MENU_KEY_DOWN_PIN                (DL_GPIO_PIN_27)
#define MENU_KEY_DOWN_IOMUX              (IOMUX_PINCM60)
#define MENU_KEY_OK_PIN                  (DL_GPIO_PIN_24)
#define MENU_KEY_OK_IOMUX                (IOMUX_PINCM54)

typedef enum {
    MENU_KEY_NONE = 0,
    MENU_KEY_UP,
    MENU_KEY_DOWN,
    MENU_KEY_OK
} menu_key_event_t;

typedef struct {
    uint32_t pin;
    bool stablePressed;
    bool lastSamplePressed;
    uint32_t lastChangeMs;
} menu_key_state_t;

typedef struct {
    uint8_t selected;
    uint8_t confirmed;
    bool hasConfirmed;
    bool needsRedraw;
} menu_state_t;

static const char *const g_menuItems[MENU_ITEM_COUNT] = {
    "Option 1",
    "Option 2",
    "Option 3",
    "Option 4",
    "Option 5"
};

static menu_key_state_t g_menuKeys[3];
static menu_state_t g_menu;

static bool menu_key_is_pressed(uint32_t pin)
{
    return (DL_GPIO_readPins(MENU_KEY_PORT, pin) == 0U);
}

static void menu_key_load_state(menu_key_state_t *key, uint32_t pin)
{
    key->pin = pin;
    key->stablePressed = menu_key_is_pressed(pin);
    key->lastSamplePressed = key->stablePressed;
    key->lastChangeMs = delay_get_ms();
}

static void menu_keys_init(void)
{
    DL_GPIO_initDigitalInputFeatures(MENU_KEY_UP_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(MENU_KEY_DOWN_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(MENU_KEY_OK_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    menu_key_load_state(&g_menuKeys[0], MENU_KEY_UP_PIN);
    menu_key_load_state(&g_menuKeys[1], MENU_KEY_DOWN_PIN);
    menu_key_load_state(&g_menuKeys[2], MENU_KEY_OK_PIN);
}

static menu_key_event_t menu_keys_update(void)
{
    const menu_key_event_t events[3] = {
        MENU_KEY_UP,
        MENU_KEY_DOWN,
        MENU_KEY_OK
    };
    uint32_t nowMs = delay_get_ms();

    for (uint8_t i = 0U; i < 3U; i++) {
        bool pressed = menu_key_is_pressed(g_menuKeys[i].pin);

        if (pressed != g_menuKeys[i].lastSamplePressed) {
            g_menuKeys[i].lastSamplePressed = pressed;
            g_menuKeys[i].lastChangeMs = nowMs;
        }

        if ((pressed != g_menuKeys[i].stablePressed) &&
            ((uint32_t)(nowMs - g_menuKeys[i].lastChangeMs) >=
                MENU_KEY_DEBOUNCE_MS)) {
            g_menuKeys[i].stablePressed = pressed;

            if (pressed) {
                return events[i];
            }
        }
    }

    return MENU_KEY_NONE;
}

static void menu_init(void)
{
    g_menu.selected = 0U;
    g_menu.confirmed = 0U;
    g_menu.hasConfirmed = false;
    g_menu.needsRedraw = true;
}

static void menu_handle_key(menu_key_event_t event)
{
    if (event == MENU_KEY_UP) {
        if (g_menu.selected == 0U) {
            g_menu.selected = MENU_ITEM_COUNT - 1U;
        } else {
            g_menu.selected--;
        }
        g_menu.needsRedraw = true;
    } else if (event == MENU_KEY_DOWN) {
        g_menu.selected++;
        if (g_menu.selected >= MENU_ITEM_COUNT) {
            g_menu.selected = 0U;
        }
        g_menu.needsRedraw = true;
    } else if (event == MENU_KEY_OK) {
        g_menu.confirmed = g_menu.selected;
        g_menu.hasConfirmed = true;
        g_menu.needsRedraw = true;
    }
}

static void menu_render(void)
{
    oled_clear();

    oled_set_cursor(0U, 0U);
    oled_print_string("Menu");

    for (uint8_t i = 0U; i < MENU_ITEM_COUNT; i++) {
        oled_set_cursor((uint8_t)(i + 1U), 0U);
        oled_print_char((i == g_menu.selected) ? '>' : ' ');
        oled_print_char(' ');
        oled_print_string(g_menuItems[i]);
    }

    oled_set_cursor(7U, 0U);
    oled_print_string("OK: ");
    if (g_menu.hasConfirmed) {
        oled_print_string(g_menuItems[g_menu.confirmed]);
    } else {
        oled_print_string("--");
    }

    g_menu.needsRedraw = false;
}

int main(void)
{
    icm42688_raw_t raw = {0};
    attitude_euler_t euler = {0};

    bool imuOk;
    bool oledOk;

    uint32_t lastUpdateMs;
    uint32_t lastRetryMs;
    uint32_t nowMs;

    float dt;

    /*
     * 初始化 MCU 外设。
     *
     * 必须先执行 SYSCFG_DL_init()，
     * 因为硬件 I2C0、PA0 和 PA1 都由 SysConfig 初始化。
     */
    SYSCFG_DL_init();

    menu_keys_init();
    menu_init();
    oledOk = oled_init();
    if (oledOk) {
        menu_render();
    }
   
    /* 初始化姿态解算器内部状态。 */
    attitude_init();
    delay_ms(2000);
    /* 初始化 ICM42688 硬件 I2C 通信。 */
    imuOk = icm42688_init();

    if (imuOk) {
        /*
         * 传感器刚启动后先等待输出稳定。
         */
        delay_ms(200U);

        /*
         * 校准期间必须保持小车和传感器静止。
         *
         * 如果 attitude_calibrate_gyro() 内部每次采样延时 2 ms，
         * 1000 次采样大约需要 2 秒。
         */
        attitude_calibrate_gyro(IMU_GYRO_CALIB_SAMPLE_COUNT);
    }

    lastUpdateMs = delay_get_ms();
    lastRetryMs = lastUpdateMs;

    while (1) {
        nowMs = delay_get_ms();

        menu_handle_key(menu_keys_update());
        if (oledOk && g_menu.needsRedraw) {
            menu_render();
        }

        /*
         * 初始化失败或运行中掉线后，每隔 1 秒重新初始化一次。
         *
         * 避免传感器启动失败后，程序永久停留在错误状态。
         */
        if (imuOk == false) {
            if ((uint32_t)(nowMs - lastRetryMs) >= IMU_RETRY_PERIOD_MS) {
                lastRetryMs = nowMs;

                imuOk = icm42688_init();

                if (imuOk) {
                    delay_ms(200U);

                    /*
                     * 重新连接后重新校准零偏。
                     * 校准期间必须保持传感器静止。
                     */
                    attitude_init();
                    attitude_calibrate_gyro(
                        IMU_GYRO_CALIB_SAMPLE_COUNT);

                    lastUpdateMs = delay_get_ms();
                }
            }
        }

        if (imuOk) {
            /*
             * 读取温度、加速度计和陀螺仪原始值。
             */
            icm42688_read_raw(&raw);

            /*
             * 新的 I2C 驱动读取失败时会把 whoAmI 设置为 0xFF。
             * 因此可以通过 WHO_AM_I 判断运行中是否掉线。
             */
            if (raw.whoAmI != ICM42688_EXPECTED_WHO_AM_I) {
                imuOk = false;

                /*
                 * 通信中断后不再使用当前错误数据进行姿态解算。
                 */
                raw.accelX = 0;
                raw.accelY = 0;
                raw.accelZ = 0;
                raw.gyroX = 0;
                raw.gyroY = 0;
                raw.gyroZ = 0;
            }
        }

        nowMs = delay_get_ms();

        /*
         * 根据实际执行间隔计算姿态更新时间。
         *
         * 使用实际 dt 比固定写死 0.01 秒更准确，因为 I2C 读取、
         * 串口发送和程序执行本身也需要时间。
         */
        dt = (float)(nowMs - lastUpdateMs) / 1000.0f;
        lastUpdateMs = nowMs;

        /*
         * 防止首次运行、系统阻塞或计时异常产生不合理 dt。
         */
        if ((dt < IMU_DT_MIN_S) || (dt > IMU_DT_MAX_S)) {
            dt = IMU_DT_DEFAULT_S;
        }

        if (imuOk) {
            /*
             * 使用有效的 ICM42688 数据更新姿态。
             */
            attitude_update_from_icm42688(&raw, dt);
            attitude_get_euler(&euler);

            /*
             * VOFA+ FireWater：
             *
             * ch0：roll
             * ch1：pitch
             * ch2：yaw
             * ch3：gyroX，单位 °/s
             * ch4：gyroY，单位 °/s
             * ch5：gyroZ，单位 °/s
             */
            vofa_send_six_float(
                euler.roll,
                euler.pitch,
                euler.yaw,
                (float)raw.gyroX * IMU_GYRO_DPS_PER_LSB,
                (float)raw.gyroY * IMU_GYRO_DPS_PER_LSB,
                (float)raw.gyroZ * IMU_GYRO_DPS_PER_LSB,
                2U);
        } else {
            /*
             * 通信失败时：
             * ch0~ch4 输出 0；
             * ch5 输出 WHO_AM_I，正常应为 71，即 0x47。
             *
             * 读取失败通常为 255，即 0xFF。
             */
            vofa_send_six_float(
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                (float)raw.whoAmI,
                2U);
        }

        delay_ms(IMU_UPDATE_PERIOD_MS);
    }
}

/*
 * SysTick 1 ms 中断。
 */
void SysTick_Handler(void)
{
    delay_tick();
}

// #include "ti_msp_dl_config.h"
// #include "delay.h"

// int main()
// {
//     SYSCFG_DL_init();
//     while(1)
//     {
//         DL_GPIO_clearPins(BUZZER_PORT, BUZZER_BZ_PIN);
//         delay_ms(2000);
//         DL_GPIO_setPins(BUZZER_PORT, BUZZER_BZ_PIN);
//         delay_ms(2000);
//     }
// }

// void SysTick_Handler(void)
// {
//     delay_tick();
// }