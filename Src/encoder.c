#include "encoder.h"

/*
 * encoder.c
 *
 * 左右轮 AB 相编码器计数与测速。
 *
 * 特点：
 *   1. GPIO 中断触发；
 *   2. AB 相四倍频计数；
 *   3. 每 10ms 根据计数差计算一次轮速，单位 mm/s；
 *   4. 计数变量在中断中修改，所以读取时需要短暂关中断保护。
 */

/* 左右轮 A/B 相引脚别名，方便后面统一处理。 */
#define ENCODER_LEFT_A_PIN        (ENCODER_E1A_PIN)
#define ENCODER_LEFT_B_PIN        (ENCODER_E1B_PIN)
#define ENCODER_RIGHT_A_PIN       (ENCODER_E2A_PIN)
#define ENCODER_RIGHT_B_PIN       (ENCODER_E2B_PIN)

#define ENCODER_LEFT_PINS         (ENCODER_LEFT_A_PIN | ENCODER_LEFT_B_PIN)
#define ENCODER_RIGHT_PINS        (ENCODER_RIGHT_A_PIN | ENCODER_RIGHT_B_PIN)
// #define ENCODER_ALL_PINS          (ENCODER_LEFT_PINS | ENCODER_RIGHT_PINS)

/* π 放大 1000000 倍，避免浮点参与编码器距离计算。 */
#define ENCODER_PI_X1000000       (3141593LL)

/* 轮子转一圈对应的编码器计数，放大 1000 倍保存减速比小数。 */
#define ENCODER_CPR_X1000         ((int64_t) ENCODER_LINES_PER_MOTOR_REV * \
                                   ENCODER_QUADRATURE_MULTIPLIER * \
                                   ENCODER_GEAR_RATIO_X1000)
/* 轮子周长，单位 mm，同样放大 1000 倍。 */
#define ENCODER_CIRCUM_MM_X1000   (((int64_t) ENCODER_WHEEL_DIAMETER_MM * \
                                   ENCODER_PI_X1000000) / 1000LL)

/* 左右轮累计计数：在 GPIO 中断中更新。 */
static volatile int32_t g_encoderLeftCount = 0;
static volatile int32_t g_encoderRightCount = 0;

/* 左右轮速度：在 encoder_tick_1ms() 中每 10ms 更新一次。 */
static volatile int32_t g_encoderLeftSpeedMmS = 0;
static volatile int32_t g_encoderRightSpeedMmS = 0;

/* 上一次 AB 状态，用于四倍频状态机判断方向。 */
static uint8_t g_encoderLeftLastState = 0;
static uint8_t g_encoderRightLastState = 0;

/* 上一次测速时的计数，用于计算 10ms 内的增量。 */
static int32_t g_encoderLeftLastSpeedCount = 0;
static int32_t g_encoderRightLastSpeedCount = 0;

/* 1ms 计数器，累计到 ENCODER_SPEED_PERIOD_MS 后更新速度。 */
static uint8_t g_encoderSpeedTickMs = 0;

/* 读取某一组 A/B 引脚当前状态：bit1=A，bit0=B。 */
static uint8_t encoder_read_A_state(uint32_t aPin, uint32_t bPin)
{
    uint32_t pins = DL_GPIO_readPins(GPIOA, aPin | bPin);
    uint8_t state = 0;

    if ((pins & aPin) != 0U) {
        state |= 0x02U;
    }

    if ((pins & bPin) != 0U) {
        state |= 0x01U;
    }

    return state;
}

static uint8_t encoder_read_B_state(uint32_t aPin, uint32_t bPin)
{
    uint32_t pins = DL_GPIO_readPins(GPIOB, aPin | bPin);
    uint8_t state = 0;

    if ((pins & aPin) != 0U) {
        state |= 0x02U;
    }

    if ((pins & bPin) != 0U) {
        state |= 0x01U;
    }

    return state;
}

static int8_t encoder_get_delta(uint8_t lastState, uint8_t currentState)
{
    /*
     * 四倍频状态表。
     *
     * 索引：
     *   高两位：上一次 AB 状态；
     *   低两位：当前 AB 状态。
     *
     * 合法的一步跳变记为 +1 或 -1；
     * 不动或非法两位同时跳变记为 0，用来抑制简单毛刺。
     */
    static const int8_t quadratureTable[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0,
    };

    return quadratureTable[((lastState & 0x03U) << 2) |
                           (currentState & 0x03U)];
}

static void encoder_update_left(void)
{
    /* 左轮 A/B 状态变化后，根据状态表得到本次增量。 */
    uint8_t currentState =
        encoder_read_A_state(ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN);
    int8_t delta = encoder_get_delta(g_encoderLeftLastState, currentState);

    g_encoderLeftCount += (int32_t) delta * ENCODER_LEFT_DIR;
    g_encoderLeftLastState = currentState;
}

static void encoder_update_right(void)
{
    /* 右轮 A/B 状态变化后，根据状态表得到本次增量。 */
    uint8_t currentState =
        encoder_read_B_state(ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);
    int8_t delta = encoder_get_delta(g_encoderRightLastState, currentState);

    g_encoderRightCount += (int32_t) delta * ENCODER_RIGHT_DIR;
    g_encoderRightLastState = currentState;
}

static int32_t encoder_delta_to_speed_mm_s(int32_t deltaCount)
{
    /*
     * 速度换算：
     *   deltaCount / CPR = 这段时间轮子转了多少圈；
     *   圈数 * 周长 = 距离 mm；
     *   距离 / 周期 = mm/s。
     */
    int64_t numerator = (int64_t) deltaCount * ENCODER_CIRCUM_MM_X1000 *
                        1000LL;
    int64_t denominator = ENCODER_CPR_X1000 * ENCODER_SPEED_PERIOD_MS;

    if (numerator >= 0) {
        /* 正数四舍五入。 */
        numerator += denominator / 2;
    } else {
        /* 负数四舍五入。 */
        numerator -= denominator / 2;
    }

    return (int32_t) (numerator / denominator);
}

static void encoder_update_speed(void)
{
    int32_t leftCount;
    int32_t rightCount;
    int32_t leftDelta;
    int32_t rightDelta;
    uint32_t primask = __get_PRIMASK();

    /* 计数在中断里更新，读取一组左右计数时短暂关中断保证一致性。 */
    __disable_irq();
    leftCount = g_encoderLeftCount;
    rightCount = g_encoderRightCount;
    __set_PRIMASK(primask);

    /* 计算本测速周期内的计数增量。 */
    leftDelta = leftCount - g_encoderLeftLastSpeedCount;
    rightDelta = rightCount - g_encoderRightLastSpeedCount;

    /* 保存本次计数，供下个周期计算 delta。 */
    g_encoderLeftLastSpeedCount = leftCount;
    g_encoderRightLastSpeedCount = rightCount;
    g_encoderLeftSpeedMmS = encoder_delta_to_speed_mm_s(leftDelta);
    g_encoderRightSpeedMmS = encoder_delta_to_speed_mm_s(rightDelta);
}

void encoder_init(void)
{
    uint32_t primask = __get_PRIMASK();

    /* 初始化期间临时禁止全局中断。 */
    __disable_irq();

    g_encoderLeftCount = 0;
    g_encoderRightCount = 0;

    g_encoderLeftSpeedMmS = 0;
    g_encoderRightSpeedMmS = 0;

    g_encoderLeftLastSpeedCount = 0;
    g_encoderRightLastSpeedCount = 0;

    g_encoderSpeedTickMs = 0;

    /* 记录当前AB初始状态。 */
    g_encoderLeftLastState =
        encoder_read_A_state(
            ENCODER_LEFT_A_PIN,
            ENCODER_LEFT_B_PIN);

    g_encoderRightLastState =
        encoder_read_B_state(
            ENCODER_RIGHT_A_PIN,
            ENCODER_RIGHT_B_PIN);

    /*
     * PA25、PA26属于GPIOA上半区16～31。
     */
    DL_GPIO_setUpperPinsPolarity(
        GPIOA,
        DL_GPIO_PIN_25_EDGE_RISE_FALL |
        DL_GPIO_PIN_26_EDGE_RISE_FALL);

    /*
     * PB20、PB24属于GPIOB上半区16～31。
     */
    DL_GPIO_setUpperPinsPolarity(
        GPIOB,
        DL_GPIO_PIN_20_EDGE_RISE_FALL |
        DL_GPIO_PIN_24_EDGE_RISE_FALL);

    /* 清除可能残留的GPIO中断标志。 */
    DL_GPIO_clearInterruptStatus(
        GPIOA,
        ENCODER_LEFT_PINS);

    DL_GPIO_clearInterruptStatus(
        GPIOB,
        ENCODER_RIGHT_PINS);

    /* 使能左右编码器四个引脚的中断。 */
    DL_GPIO_enableInterrupt(
        GPIOA,
        ENCODER_LEFT_PINS);

    DL_GPIO_enableInterrupt(
        GPIOB,
        ENCODER_RIGHT_PINS);

    /*
     * GPIOA和GPIOB在MSPM0G3507中使用同一个IRQ 1，
     * 清除一次共享NVIC挂起标志即可。
     */
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);

    __set_PRIMASK(primask);

    /* GPIOA和GPIOB共用IRQ 1，只需要使能一次。 */
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
}

void encoder_tick_1ms(void)
{
    /* 每 1ms 调用一次，累计到测速周期后更新速度。 */
    g_encoderSpeedTickMs++;

    if (g_encoderSpeedTickMs >= ENCODER_SPEED_PERIOD_MS) {
        g_encoderSpeedTickMs = 0;
        encoder_update_speed();
    }
}

int32_t encoder_get_left_count(void)
{
    uint32_t primask = __get_PRIMASK();
    int32_t count;

    /* 计数变量在中断中修改，读取时需要临界区保护。 */
    __disable_irq();
    count = g_encoderLeftCount;
    __set_PRIMASK(primask);

    return count;
}

int32_t encoder_get_right_count(void)
{
    uint32_t primask = __get_PRIMASK();
    int32_t count;

    /* 计数变量在中断中修改，读取时需要临界区保护。 */
    __disable_irq();
    count = g_encoderRightCount;
    __set_PRIMASK(primask);

    return count;
}

int32_t encoder_get_left_speed_mm_s(void)
{
    uint32_t primask = __get_PRIMASK();
    int32_t speed;

    /* 速度变量在周期函数中更新，读取时也做临界区保护。 */
    __disable_irq();
    speed = g_encoderLeftSpeedMmS;
    __set_PRIMASK(primask);

    return speed;
}

int32_t encoder_get_right_speed_mm_s(void)
{
    uint32_t primask = __get_PRIMASK();
    int32_t speed;

    /* 速度变量在周期函数中更新，读取时也做临界区保护。 */
    __disable_irq();
    speed = g_encoderRightSpeedMmS;
    __set_PRIMASK(primask);

    return speed;
}

void encoder_reset_left_count(void)
{
    uint32_t primask = __get_PRIMASK();

    /* 清零左轮计数，同时重新记录当前 AB 状态，避免清零后第一跳误计。 */
    __disable_irq();
    g_encoderLeftCount = 0;
    g_encoderLeftSpeedMmS = 0;
    g_encoderLeftLastSpeedCount = 0;
    g_encoderLeftLastState =
        encoder_read_A_state(ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN);
    __set_PRIMASK(primask);
}

void encoder_reset_right_count(void)
{
    uint32_t primask = __get_PRIMASK();

    /* 清零右轮计数，同时重新记录当前 AB 状态。 */
    __disable_irq();
    g_encoderRightCount = 0;
    g_encoderRightSpeedMmS = 0;
    g_encoderRightLastSpeedCount = 0;
    g_encoderRightLastState =
        encoder_read_B_state(ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);
    __set_PRIMASK(primask);
}

void encoder_reset_count(void)
{
    uint32_t primask = __get_PRIMASK();

    /* 同时清零两侧计数和测速状态。 */
    __disable_irq();
    g_encoderLeftCount = 0;
    g_encoderRightCount = 0;
    g_encoderLeftSpeedMmS = 0;
    g_encoderRightSpeedMmS = 0;
    g_encoderLeftLastSpeedCount = 0;
    g_encoderRightLastSpeedCount = 0;
    g_encoderSpeedTickMs = 0;
    g_encoderLeftLastState =
        encoder_read_A_state(ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN);
    g_encoderRightLastState =
        encoder_read_B_state(ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);
    __set_PRIMASK(primask);
}

int32_t encoder_get_count(void)
{
    return encoder_get_left_count();
}

void encoder_gpio_irq_handler(void)
{
    uint32_t pendingA;
    uint32_t pendingB;

    /* 分别读取GPIOA和GPIOB的中断状态。 */
    pendingA = DL_GPIO_getEnabledInterruptStatus(
        GPIOA,
        ENCODER_LEFT_PINS);

    pendingB = DL_GPIO_getEnabledInterruptStatus(
        GPIOB,
        ENCODER_RIGHT_PINS);

    /*
     * 先清除本次中断标志，再读取AB状态。
     * 避免在读取AB状态后出现的新边沿被后续清除操作误删。
     */
    if (pendingA != 0U) {
        DL_GPIO_clearInterruptStatus(GPIOA, pendingA);
        encoder_update_left();
    }

    if (pendingB != 0U) {
        DL_GPIO_clearInterruptStatus(GPIOB, pendingB);
        encoder_update_right();
    }
}

void GROUP1_IRQHandler(void)
{
    encoder_gpio_irq_handler();
}
