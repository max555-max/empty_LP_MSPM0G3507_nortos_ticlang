#include "encoder.h"

#define ENCODER_LEFT_A_PIN        (ENCODER_E2A_PIN)
#define ENCODER_LEFT_B_PIN        (ENCODER_E2B_PIN)
#define ENCODER_RIGHT_A_PIN       (ENCODER_E1A_PIN)
#define ENCODER_RIGHT_B_PIN       (ENCODER_E1B_PIN)

#define ENCODER_LEFT_PINS         (ENCODER_LEFT_A_PIN | ENCODER_LEFT_B_PIN)
#define ENCODER_RIGHT_PINS        (ENCODER_RIGHT_A_PIN | ENCODER_RIGHT_B_PIN)
#define ENCODER_ALL_PINS          (ENCODER_LEFT_PINS | ENCODER_RIGHT_PINS)

#define ENCODER_PI_X1000000       (3141593LL)
#define ENCODER_CPR_X1000         ((int64_t) ENCODER_LINES_PER_MOTOR_REV * \
                                   ENCODER_QUADRATURE_MULTIPLIER * \
                                   ENCODER_GEAR_RATIO_X1000)
#define ENCODER_CIRCUM_MM_X1000   (((int64_t) ENCODER_WHEEL_DIAMETER_MM * \
                                   ENCODER_PI_X1000000) / 1000LL)

static volatile int32_t g_encoderLeftCount = 0;
static volatile int32_t g_encoderRightCount = 0;
static volatile int32_t g_encoderLeftSpeedMmS = 0;
static volatile int32_t g_encoderRightSpeedMmS = 0;

static uint8_t g_encoderLeftLastState = 0;
static uint8_t g_encoderRightLastState = 0;
static int32_t g_encoderLeftLastSpeedCount = 0;
static int32_t g_encoderRightLastSpeedCount = 0;
static uint8_t g_encoderSpeedTickMs = 0;

static uint8_t encoder_read_state(uint32_t aPin, uint32_t bPin)
{
    uint32_t pins = DL_GPIO_readPins(ENCODER_PORT, aPin | bPin);
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
     * Index: previous AB state in high two bits, current AB state in low two
     * bits. Valid one-step quadrature transitions are +/-1. No movement and
     * illegal two-bit jumps are treated as 0 to reject simple glitches.
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
    uint8_t currentState =
        encoder_read_state(ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN);
    int8_t delta = encoder_get_delta(g_encoderLeftLastState, currentState);

    g_encoderLeftCount += (int32_t) delta * ENCODER_LEFT_DIR;
    g_encoderLeftLastState = currentState;
}

static void encoder_update_right(void)
{
    uint8_t currentState =
        encoder_read_state(ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);
    int8_t delta = encoder_get_delta(g_encoderRightLastState, currentState);

    g_encoderRightCount += (int32_t) delta * ENCODER_RIGHT_DIR;
    g_encoderRightLastState = currentState;
}

static int32_t encoder_delta_to_speed_mm_s(int32_t deltaCount)
{
    int64_t numerator = (int64_t) deltaCount * ENCODER_CIRCUM_MM_X1000 *
                        1000LL;
    int64_t denominator = ENCODER_CPR_X1000 * ENCODER_SPEED_PERIOD_MS;

    if (numerator >= 0) {
        numerator += denominator / 2;
    } else {
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

    __disable_irq();
    leftCount = g_encoderLeftCount;
    rightCount = g_encoderRightCount;
    __set_PRIMASK(primask);

    leftDelta = leftCount - g_encoderLeftLastSpeedCount;
    rightDelta = rightCount - g_encoderRightLastSpeedCount;

    g_encoderLeftLastSpeedCount = leftCount;
    g_encoderRightLastSpeedCount = rightCount;
    g_encoderLeftSpeedMmS = encoder_delta_to_speed_mm_s(leftDelta);
    g_encoderRightSpeedMmS = encoder_delta_to_speed_mm_s(rightDelta);
}

void encoder_init(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    g_encoderLeftCount = 0;
    g_encoderRightCount = 0;
    g_encoderLeftSpeedMmS = 0;
    g_encoderRightSpeedMmS = 0;
    g_encoderLeftLastState =
        encoder_read_state(ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN);
    g_encoderRightLastState =
        encoder_read_state(ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);
    g_encoderLeftLastSpeedCount = 0;
    g_encoderRightLastSpeedCount = 0;
    g_encoderSpeedTickMs = 0;

    /*
     * Keep GPIO input resistor disabled. The encoder has external pull-ups.
     * These polarity calls are redundant when SysConfig generated the same
     * setting, but they make this module robust against partial regeneration.
     */
    DL_GPIO_setLowerPinsPolarity(ENCODER_PORT,
        DL_GPIO_PIN_14_EDGE_RISE_FALL | DL_GPIO_PIN_15_EDGE_RISE_FALL);
    DL_GPIO_setUpperPinsPolarity(ENCODER_PORT,
        DL_GPIO_PIN_16_EDGE_RISE_FALL | DL_GPIO_PIN_17_EDGE_RISE_FALL);

    DL_GPIO_clearInterruptStatus(ENCODER_PORT, ENCODER_ALL_PINS);
    DL_GPIO_enableInterrupt(ENCODER_PORT, ENCODER_ALL_PINS);

    __set_PRIMASK(primask);

    NVIC_EnableIRQ(ENCODER_INT_IRQN);
}

void encoder_tick_1ms(void)
{
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

    __disable_irq();
    count = g_encoderLeftCount;
    __set_PRIMASK(primask);

    return count;
}

int32_t encoder_get_right_count(void)
{
    uint32_t primask = __get_PRIMASK();
    int32_t count;

    __disable_irq();
    count = g_encoderRightCount;
    __set_PRIMASK(primask);

    return count;
}

int32_t encoder_get_left_speed_mm_s(void)
{
    uint32_t primask = __get_PRIMASK();
    int32_t speed;

    __disable_irq();
    speed = g_encoderLeftSpeedMmS;
    __set_PRIMASK(primask);

    return speed;
}

int32_t encoder_get_right_speed_mm_s(void)
{
    uint32_t primask = __get_PRIMASK();
    int32_t speed;

    __disable_irq();
    speed = g_encoderRightSpeedMmS;
    __set_PRIMASK(primask);

    return speed;
}

void encoder_reset_left_count(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_encoderLeftCount = 0;
    g_encoderLeftSpeedMmS = 0;
    g_encoderLeftLastSpeedCount = 0;
    g_encoderLeftLastState =
        encoder_read_state(ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN);
    __set_PRIMASK(primask);
}

void encoder_reset_right_count(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_encoderRightCount = 0;
    g_encoderRightSpeedMmS = 0;
    g_encoderRightLastSpeedCount = 0;
    g_encoderRightLastState =
        encoder_read_state(ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);
    __set_PRIMASK(primask);
}

void encoder_reset_count(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_encoderLeftCount = 0;
    g_encoderRightCount = 0;
    g_encoderLeftSpeedMmS = 0;
    g_encoderRightSpeedMmS = 0;
    g_encoderLeftLastSpeedCount = 0;
    g_encoderRightLastSpeedCount = 0;
    g_encoderSpeedTickMs = 0;
    g_encoderLeftLastState =
        encoder_read_state(ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN);
    g_encoderRightLastState =
        encoder_read_state(ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);
    __set_PRIMASK(primask);
}

int32_t encoder_get_count(void)
{
    return encoder_get_left_count();
}

void encoder_gpio_irq_handler(void)
{
    uint32_t pending =
        DL_GPIO_getEnabledInterruptStatus(ENCODER_PORT, ENCODER_ALL_PINS);

    if ((pending & ENCODER_LEFT_PINS) != 0U) {
        encoder_update_left();
    }

    if ((pending & ENCODER_RIGHT_PINS) != 0U) {
        encoder_update_right();
    }

    if (pending != 0U) {
        DL_GPIO_clearInterruptStatus(ENCODER_PORT, pending & ENCODER_ALL_PINS);
    }
}

void GROUP1_IRQHandler(void)
{
    encoder_gpio_irq_handler();
}
