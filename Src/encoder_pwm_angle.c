#include "encoder_pwm_angle.h"

#include "delay.h"
#include "ti_msp_dl_config.h"

static volatile bool g_synced;
static volatile bool g_valid;
static volatile uint32_t g_loadValue;
static volatile uint32_t g_periodTicks;
static volatile uint32_t g_highTicks;
static volatile int32_t g_angleDegX10;
static volatile uint32_t g_captureCount;
static volatile uint32_t g_timeoutCount;
static volatile uint32_t g_timestampMs;

static uint32_t encoder_pwm_angle_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void encoder_pwm_angle_exit_critical(uint32_t primask)
{
    if ((primask & 1U) == 0U) {
        __enable_irq();
    }
}

static int32_t encoder_pwm_angle_duty_to_deg_x10(uint32_t highTicks,
                                                  uint32_t periodTicks)
{
    uint32_t angle;

    if (periodTicks == 0U) {
        return 0;
    }

    angle = (uint32_t)(((uint64_t)highTicks *
                        ENCODER_PWM_ANGLE_FULL_SCALE_X10 +
                        (periodTicks / 2U)) /
                       periodTicks);
    if (angle > ENCODER_PWM_ANGLE_FULL_SCALE_X10) {
        angle = ENCODER_PWM_ANGLE_FULL_SCALE_X10;
    }

    return (int32_t)angle;
}

static int32_t encoder_pwm_angle_normalize_relative_deg_x10(int32_t angleDegX10)
{
    while (angleDegX10 > 1800) {
        angleDegX10 -= (int32_t)ENCODER_PWM_ANGLE_FULL_SCALE_X10;
    }
    while (angleDegX10 < -1800) {
        angleDegX10 += (int32_t)ENCODER_PWM_ANGLE_FULL_SCALE_X10;
    }
    return angleDegX10;
}

int32_t encoder_pwm_angle_raw_to_relative_deg_x10(int32_t rawDegX10)
{
    return encoder_pwm_angle_normalize_relative_deg_x10(
        rawDegX10 - (int32_t)BEAM_ENCODER_ZERO_RAW_DEG_X10);
}

bool encoder_pwm_angle_relative_is_safe(int32_t relativeDegX10)
{
    return (relativeDegX10 <= BEAM_ENCODER_MAX_SAFE_REL_DEG_X10) &&
           (relativeDegX10 >= -BEAM_ENCODER_MAX_SAFE_REL_DEG_X10);
}

void encoder_pwm_angle_init(void)
{
    uint32_t primask = encoder_pwm_angle_enter_critical();

    g_synced = false;
    g_valid = false;
    g_loadValue = DL_TimerA_getLoadValue(BEAM_ENCODER_PWM_INST);
    if (g_loadValue == 0U) {
        g_loadValue = BEAM_ENCODER_PWM_INST_LOAD_VALUE;
    }
    g_periodTicks = 0U;
    g_highTicks = 0U;
    g_angleDegX10 = 0;
    g_captureCount = 0U;
    g_timeoutCount = 0U;
    g_timestampMs = 0U;

    DL_TimerA_stopCounter(BEAM_ENCODER_PWM_INST);
    DL_TimerA_setCoreHaltBehavior(BEAM_ENCODER_PWM_INST,
                                  DL_TIMER_CORE_HALT_IMMEDIATE);
    DL_TimerA_setTimerCount(BEAM_ENCODER_PWM_INST, g_loadValue);
    DL_TimerA_clearInterruptStatus(BEAM_ENCODER_PWM_INST,
                                   DL_TIMERA_INTERRUPT_CC1_DN_EVENT |
                                   DL_TIMERA_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(BEAM_ENCODER_PWM_INST_INT_IRQN);
    NVIC_EnableIRQ(BEAM_ENCODER_PWM_INST_INT_IRQN);
    DL_TimerA_startCounter(BEAM_ENCODER_PWM_INST);

    encoder_pwm_angle_exit_critical(primask);
}

bool encoder_pwm_angle_get_sample(encoder_pwm_angle_sample_t *sample)
{
    uint32_t primask;
    uint32_t nowMs;

    if (sample == 0) {
        return false;
    }

    primask = encoder_pwm_angle_enter_critical();
    sample->angleDegX10 = g_angleDegX10;
    sample->periodTicks = g_periodTicks;
    sample->highTicks = g_highTicks;
    sample->captureCount = g_captureCount;
    sample->timeoutCount = g_timeoutCount;
    sample->timestampMs = g_timestampMs;
    sample->valid = g_valid;
    encoder_pwm_angle_exit_critical(primask);

    sample->relativeAngleDegX10 =
        encoder_pwm_angle_raw_to_relative_deg_x10(sample->angleDegX10);
    nowMs = delay_get_ms();
    sample->fresh = sample->valid &&
                    ((uint32_t)(nowMs - sample->timestampMs) <=
                     ENCODER_PWM_ANGLE_VALID_TIMEOUT_MS);
    sample->safeLimitActive =
        sample->fresh &&
        !encoder_pwm_angle_relative_is_safe(sample->relativeAngleDegX10);
    return sample->fresh;
}

void encoder_pwm_angle_irq_handler(void)
{
    switch (DL_TimerA_getPendingInterrupt(BEAM_ENCODER_PWM_INST)) {
    case DL_TIMERA_IIDX_CC1_DN:
        if (g_synced) {
            uint32_t periodCapture = DL_TimerA_getCaptureCompareValue(
                BEAM_ENCODER_PWM_INST, DL_TIMER_CC_1_INDEX);
            uint32_t highCapture = DL_TimerA_getCaptureCompareValue(
                BEAM_ENCODER_PWM_INST, DL_TIMER_CC_0_INDEX);

            if ((periodCapture <= g_loadValue) &&
                (highCapture <= g_loadValue)) {
                uint32_t periodTicks = g_loadValue - periodCapture;
                uint32_t highTicks = g_loadValue - highCapture;

                if ((periodTicks != 0U) && (highTicks <= periodTicks)) {
                    g_periodTicks = periodTicks;
                    g_highTicks = highTicks;
                    g_angleDegX10 = encoder_pwm_angle_duty_to_deg_x10(
                        highTicks, periodTicks);
                    g_timestampMs = delay_get_ms();
                    g_captureCount++;
                    g_valid = true;
                } else {
                    g_valid = false;
                }
            } else {
                g_valid = false;
            }
        } else {
            g_synced = true;
        }

        /*
         * Workaround for the capture limitation shown in the TI duty/period
         * example: manually reload after the period edge is captured.
         */
        DL_TimerA_setTimerCount(BEAM_ENCODER_PWM_INST, g_loadValue);
        break;

    case DL_TIMERA_IIDX_ZERO:
        g_synced = false;
        g_valid = false;
        g_timeoutCount++;
        break;

    default:
        break;
    }
}

void TIMA0_IRQHandler(void)
{
    encoder_pwm_angle_irq_handler();
}
