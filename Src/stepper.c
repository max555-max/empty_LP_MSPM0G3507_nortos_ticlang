#include "stepper.h"

#include "ti_msp_dl_config.h"

static volatile bool g_initialized;
static volatile bool g_enabled;
static volatile bool g_running;
static volatile bool g_continuous;
static volatile bool g_directionHigh;
static volatile bool g_limitActive;
static volatile bool g_emergencyInhibit;
static volatile bool g_stopPending;
static volatile uint32_t g_remainingSteps;
static volatile uint32_t g_frequencyHz;
static volatile int32_t g_positionSteps;
static volatile float g_speedTargetHz;
static volatile float g_speedCurrentHz;
static float g_speedLimitHz;
static float g_beamTargetDeg;
static float g_beamPidIntegralMmS;
static float g_beamPidTargetMm;
static float g_beamPidKpDegPerMm;
static float g_beamPidKiDegPerMmS;
static float g_beamPidKdDegSPerMm;
static float g_beamPidOutputDeg;

static uint32_t stepper_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void stepper_exit_critical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static float stepper_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float stepper_clamp(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static bool stepper_is_finite_float(float value)
{
    return (value == value) && (value <= 3.4e38f) && (value >= -3.4e38f);
}

static uint32_t stepper_limit_frequency(uint32_t frequencyHz)
{
    if (frequencyHz < STEPPER_MIN_FREQ_HZ) {
        return STEPPER_MIN_FREQ_HZ;
    }
    if (frequencyHz > STEPPER_MAX_FREQ_HZ) {
        return STEPPER_MAX_FREQ_HZ;
    }
    return frequencyHz;
}

static void stepper_write_enable(bool enable)
{
#if (STEPPER_ENABLE_ACTIVE_LEVEL != 0U)
    if (enable) {
        DL_GPIO_setPins(STEPPER_EN_PORT, STEPPER_EN_PIN_EN_PIN);
    } else {
        DL_GPIO_clearPins(STEPPER_EN_PORT, STEPPER_EN_PIN_EN_PIN);
    }
#else
    if (enable) {
        DL_GPIO_clearPins(STEPPER_EN_PORT, STEPPER_EN_PIN_EN_PIN);
    } else {
        DL_GPIO_setPins(STEPPER_EN_PORT, STEPPER_EN_PIN_EN_PIN);
    }
#endif
    g_enabled = enable;
}

static void stepper_write_direction(bool dirHigh)
{
    bool outputHigh = dirHigh;

#if (STEPPER_DIRECTION_INVERT != 0U)
    outputHigh = !outputHigh;
#endif
    if (outputHigh) {
        DL_GPIO_setPins(STEPPER_DIR_PORT, STEPPER_DIR_PIN_DIR_PIN);
    } else {
        DL_GPIO_clearPins(STEPPER_DIR_PORT, STEPPER_DIR_PIN_DIR_PIN);
    }
    g_directionHigh = dirHigh;
}

static void stepper_force_step_low(void)
{
    DL_GPIO_initDigitalOutput(GPIO_STEPPER_PWM_C0_IOMUX);
    DL_GPIO_clearPins(GPIO_STEPPER_PWM_C0_PORT, GPIO_STEPPER_PWM_C0_PIN);
    DL_GPIO_enableOutput(GPIO_STEPPER_PWM_C0_PORT, GPIO_STEPPER_PWM_C0_PIN);
}

static void stepper_connect_step_to_timer(void)
{
    DL_GPIO_initPeripheralOutputFunction(GPIO_STEPPER_PWM_C0_IOMUX,
                                         GPIO_STEPPER_PWM_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_STEPPER_PWM_C0_PORT, GPIO_STEPPER_PWM_C0_PIN);
}

static void stepper_stop_timer_locked(void)
{
    DL_TimerA_stopCounter(STEPPER_PWM_INST);
    DL_TimerA_disableInterrupt(STEPPER_PWM_INST,
                               DL_TIMER_INTERRUPT_CC0_DN_EVENT);
    DL_TimerA_clearInterruptStatus(STEPPER_PWM_INST,
                                   DL_TIMER_INTERRUPT_CC0_DN_EVENT);
    DL_TimerA_setTimerCount(STEPPER_PWM_INST, 0U);
    DL_TimerA_setCaptureCompareValue(STEPPER_PWM_INST, 0U,
                                     GPIO_STEPPER_PWM_C0_IDX);
    stepper_force_step_low();

    g_running = false;
    g_stopPending = false;
    g_frequencyHz = 0U;
}

static bool stepper_step_is_high(void)
{
    return (DL_GPIO_readPins(GPIO_STEPPER_PWM_C0_PORT,
                             GPIO_STEPPER_PWM_C0_PIN) != 0U);
}

static void stepper_request_stop_locked(void)
{
    if (!g_running) {
        return;
    }

    if (stepper_step_is_high()) {
        g_stopPending = true;
    } else {
        stepper_stop_timer_locked();
    }
}

static void stepper_count_accepted_pulse(void)
{
    if (g_directionHigh) {
        g_positionSteps++;
        if (g_positionSteps >= STEPPER_MAX_POSITION_STEPS) {
            g_positionSteps = STEPPER_MAX_POSITION_STEPS;
            g_limitActive = true;
            g_speedTargetHz = 0.0f;
            g_speedCurrentHz = 0.0f;
            g_stopPending = true;
        }
    } else {
        g_positionSteps--;
        if (g_positionSteps <= -STEPPER_MAX_POSITION_STEPS) {
            g_positionSteps = -STEPPER_MAX_POSITION_STEPS;
            g_limitActive = true;
            g_speedTargetHz = 0.0f;
            g_speedCurrentHz = 0.0f;
            g_stopPending = true;
        }
    }
}

static bool stepper_can_start_locked(bool dirHigh)
{
    if (!g_initialized || !g_enabled || g_emergencyInhibit || g_running) {
        return false;
    }

    if (((g_positionSteps >= STEPPER_MAX_POSITION_STEPS) && dirHigh) ||
        ((g_positionSteps <= -STEPPER_MAX_POSITION_STEPS) && !dirHigh)) {
        g_limitActive = true;
        return false;
    }

    return true;
}

static void stepper_start_timer_locked(uint32_t frequencyHz, bool dirHigh)
{
    uint32_t periodTicks;
    uint32_t compareTicks;

    frequencyHz = stepper_limit_frequency(frequencyHz);
    periodTicks = 1000000U / frequencyHz;
    if (periodTicks < 2U) {
        periodTicks = 2U;
    }
    compareTicks = periodTicks / 2U;
    if (compareTicks == 0U) {
        compareTicks = 1U;
    }

    DL_TimerA_stopCounter(STEPPER_PWM_INST);
    DL_TimerA_disableInterrupt(STEPPER_PWM_INST,
                               DL_TIMER_INTERRUPT_CC0_DN_EVENT);
    DL_TimerA_clearInterruptStatus(STEPPER_PWM_INST,
                                   DL_TIMER_INTERRUPT_CC0_DN_EVENT);

    stepper_force_step_low();
    stepper_write_direction(dirHigh);
    stepper_connect_step_to_timer();

    DL_TimerA_setLoadValue(STEPPER_PWM_INST, periodTicks - 1U);
    DL_TimerA_setCaptureCompareValue(STEPPER_PWM_INST, compareTicks,
                                     GPIO_STEPPER_PWM_C0_IDX);
    DL_TimerA_setTimerCount(STEPPER_PWM_INST, 0U);
    DL_TimerA_clearInterruptStatus(STEPPER_PWM_INST,
                                   DL_TIMER_INTERRUPT_CC0_DN_EVENT);
    DL_TimerA_enableInterrupt(STEPPER_PWM_INST,
                               DL_TIMER_INTERRUPT_CC0_DN_EVENT);

    g_frequencyHz = frequencyHz;
    g_stopPending = false;
    g_running = true;
    DL_TimerA_startCounter(STEPPER_PWM_INST);
}

static void stepper_apply_speed(float speedHz)
{
    bool dirHigh;
    uint32_t frequencyHz;

    if (!g_enabled || g_emergencyInhibit || (stepper_abs(speedHz) < 0.5f)) {
        stepper_request_stop_locked();
        return;
    }

    dirHigh = (speedHz > 0.0f);
    if (((g_positionSteps >= STEPPER_MAX_POSITION_STEPS) && dirHigh) ||
        ((g_positionSteps <= -STEPPER_MAX_POSITION_STEPS) && !dirHigh)) {
        g_limitActive = true;
        g_speedTargetHz = 0.0f;
        g_speedCurrentHz = 0.0f;
        stepper_request_stop_locked();
        return;
    }

    frequencyHz = stepper_limit_frequency((uint32_t)stepper_abs(speedHz));
    if (g_running && (g_directionHigh != dirHigh)) {
        stepper_request_stop_locked();
        return;
    }

    if (!g_running) {
        stepper_start_timer_locked(frequencyHz, dirHigh);
    } else if (frequencyHz != g_frequencyHz) {
        stepper_start_timer_locked(frequencyHz, dirHigh);
    }
}

void stepper_init(void)
{
    uint32_t primask = stepper_enter_critical();

    stepper_force_step_low();
    stepper_write_direction(true);
    stepper_write_enable(false);
    stepper_stop_timer_locked();

    g_initialized = true;
    g_continuous = false;
    g_directionHigh = true;
    g_limitActive = false;
    g_emergencyInhibit = false;
    g_remainingSteps = 0U;
    g_positionSteps = STEPPER_INITIAL_POSITION_STEPS;
    if (g_positionSteps > STEPPER_MAX_POSITION_STEPS) {
        g_positionSteps = STEPPER_MAX_POSITION_STEPS;
    } else if (g_positionSteps < -STEPPER_MAX_POSITION_STEPS) {
        g_positionSteps = -STEPPER_MAX_POSITION_STEPS;
    }
    g_speedTargetHz = 0.0f;
    g_speedCurrentHz = 0.0f;
    g_speedLimitHz = (float)STEPPER_MAX_FREQ_HZ;
    g_beamTargetDeg = 0.0f;
    g_beamPidIntegralMmS = 0.0f;
    g_beamPidTargetMm = STEPPER_BEAM_PID_POSITION_TARGET_MM;
    g_beamPidKpDegPerMm = STEPPER_BEAM_PID_KP_DEG_PER_MM;
    g_beamPidKiDegPerMmS = STEPPER_BEAM_PID_KI_DEG_PER_MM_S;
    g_beamPidKdDegSPerMm = STEPPER_BEAM_PID_KD_DEG_S_PER_MM;
    g_beamPidOutputDeg = 0.0f;

    NVIC_ClearPendingIRQ(STEPPER_PWM_INST_INT_IRQN);
    NVIC_EnableIRQ(STEPPER_PWM_INST_INT_IRQN);
    stepper_exit_critical(primask);
}

void stepper_enable(bool enable)
{
    uint32_t primask = stepper_enter_critical();

    if (!enable) {
        g_continuous = false;
        g_remainingSteps = 0U;
        g_speedTargetHz = 0.0f;
        g_speedCurrentHz = 0.0f;
        stepper_stop_timer_locked();
        stepper_write_enable(false);
    } else if (!g_emergencyInhibit) {
        stepper_write_enable(true);
    }
    stepper_exit_critical(primask);
}

bool stepper_is_enabled(void)
{
    return g_enabled;
}

void stepper_set_speed_target(float speedHz)
{
    if (speedHz > g_speedLimitHz) {
        speedHz = g_speedLimitHz;
    } else if (speedHz < -g_speedLimitHz) {
        speedHz = -g_speedLimitHz;
    }

    if (((g_positionSteps >= STEPPER_MAX_POSITION_STEPS) && (speedHz > 0.0f)) ||
        ((g_positionSteps <= -STEPPER_MAX_POSITION_STEPS) && (speedHz < 0.0f))) {
        speedHz = 0.0f;
        g_limitActive = true;
    }
    g_speedTargetHz = speedHz;
}

void stepper_set_speed_limit_hz(float limitHz)
{
    if (limitHz <= 0.0f) {
        g_speedLimitHz = 0.0f;
        g_speedTargetHz = 0.0f;
        return;
    }
    g_speedLimitHz = stepper_clamp(limitHz, (float)STEPPER_MIN_FREQ_HZ,
                                   (float)STEPPER_MAX_FREQ_HZ);
    stepper_set_speed_target(g_speedTargetHz);
}

float stepper_get_speed_limit_hz(void)
{
    return g_speedLimitHz;
}

float stepper_get_speed_target_hz(void)
{
    return g_speedTargetHz;
}

void stepper_service(float dtSeconds)
{
    float effectiveTarget = g_speedTargetHz;
    float maxChange;
    float delta;
    uint32_t primask;

    if ((dtSeconds <= 0.0f) || (dtSeconds > 0.1f)) {
        dtSeconds = 0.008f;
    }
    if (((g_speedCurrentHz > 0.0f) && (effectiveTarget < 0.0f)) ||
        ((g_speedCurrentHz < 0.0f) && (effectiveTarget > 0.0f))) {
        effectiveTarget = 0.0f;
    }

    maxChange = STEPPER_ACCELERATION_HZ_PER_S * dtSeconds;
    delta = effectiveTarget - g_speedCurrentHz;
    delta = stepper_clamp(delta, -maxChange, maxChange);
    g_speedCurrentHz += delta;
    if (stepper_abs(g_speedCurrentHz) < 0.5f) {
        g_speedCurrentHz = 0.0f;
    }

    primask = stepper_enter_critical();
    if (g_running && !g_continuous) {
        stepper_exit_critical(primask);
        return;
    }
    g_continuous = true;
    stepper_apply_speed(g_speedCurrentHz);
    stepper_exit_critical(primask);
}

void stepper_stop_smooth(void)
{
    g_speedTargetHz = 0.0f;
}

void stepper_emergency_stop(void)
{
    uint32_t primask = stepper_enter_critical();

    g_emergencyInhibit = true;
    g_continuous = false;
    g_remainingSteps = 0U;
    g_speedTargetHz = 0.0f;
    g_speedCurrentHz = 0.0f;
    stepper_stop_timer_locked();
    stepper_write_enable(false);
    stepper_exit_critical(primask);
}

bool stepper_clear_emergency_inhibit(void)
{
    uint32_t primask = stepper_enter_critical();

    g_emergencyInhibit = false;
    stepper_exit_critical(primask);
    return true;
}

bool stepper_is_emergency_inhibited(void)
{
    return g_emergencyInhibit;
}

int32_t stepper_get_position_steps(void)
{
    int32_t value;
    uint32_t primask = stepper_enter_critical();

    value = g_positionSteps;
    stepper_exit_critical(primask);
    return value;
}

void stepper_sync_position_from_beam_angle_x10(int32_t beamAngleDegX10)
{
    float beamAngleDeg = (float)beamAngleDegX10 * 0.1f;
    float positionSteps = beamAngleDeg * STEPPER_STEPS_PER_BEAM_DEGREE;
    int32_t syncedPosition =
        (positionSteps >= 0.0f) ? (int32_t)(positionSteps + 0.5f) :
                                  (int32_t)(positionSteps - 0.5f);
    uint32_t primask = stepper_enter_critical();

    g_positionSteps = syncedPosition;
    if ((g_positionSteps < STEPPER_MAX_POSITION_STEPS) &&
        (g_positionSteps > -STEPPER_MAX_POSITION_STEPS)) {
        g_limitActive = false;
    }

    stepper_exit_critical(primask);
}

bool stepper_set_zero_position(void)
{
    uint32_t primask = stepper_enter_critical();

    if (!g_initialized || g_enabled || g_running) {
        stepper_exit_critical(primask);
        return false;
    }

    g_positionSteps = 0;
    g_limitActive = false;
    g_continuous = false;
    g_remainingSteps = 0U;
    g_speedTargetHz = 0.0f;
    g_speedCurrentHz = 0.0f;
    g_beamTargetDeg = 0.0f;
    g_beamPidIntegralMmS = 0.0f;
    g_beamPidOutputDeg = 0.0f;
    stepper_exit_critical(primask);
    return true;
}

bool stepper_is_moving(void)
{
    return g_running;
}

bool stepper_is_limit_active(void)
{
    return g_limitActive;
}

void stepper_clear_limit_flag(void)
{
    if ((g_positionSteps > -STEPPER_MAX_POSITION_STEPS) &&
        (g_positionSteps < STEPPER_MAX_POSITION_STEPS)) {
        g_limitActive = false;
    }
}

float stepper_get_current_speed_hz(void)
{
    return g_speedCurrentHz;
}

float stepper_get_beam_angle_deg(void)
{
    return (float)stepper_get_position_steps() / STEPPER_STEPS_PER_BEAM_DEGREE;
}

void stepper_set_beam_target_deg(float beamAngleDeg)
{
    g_beamTargetDeg = stepper_clamp(beamAngleDeg, -STEPPER_MAX_BEAM_ANGLE_DEG,
                                    STEPPER_MAX_BEAM_ANGLE_DEG);
}

float stepper_get_beam_target_deg(void)
{
    return g_beamTargetDeg;
}

void stepper_update_beam_position_loop(void)
{
    float targetSteps = g_beamTargetDeg * STEPPER_STEPS_PER_BEAM_DEGREE;
    int32_t target = (targetSteps >= 0.0f) ? (int32_t)(targetSteps + 0.5f) :
                                              (int32_t)(targetSteps - 0.5f);
    int32_t error = target - stepper_get_position_steps();

    if ((error <= STEPPER_POSITION_TOLERANCE_STEPS) &&
        (error >= -STEPPER_POSITION_TOLERANCE_STEPS)) {
        stepper_set_speed_target(0.0f);
    } else {
        stepper_set_speed_target((float)error *
                                 STEPPER_ANGLE_LOOP_KP_HZ_PER_STEP);
    }
}

void stepper_reset_beam_pid(void)
{
    g_beamPidIntegralMmS = 0.0f;
    g_beamPidOutputDeg = 0.0f;
}

bool stepper_set_beam_pid_target_mm(float targetMm)
{
    if (!stepper_is_finite_float(targetMm) ||
        (targetMm > STEPPER_BEAM_PID_TARGET_LIMIT_MM) ||
        (targetMm < -STEPPER_BEAM_PID_TARGET_LIMIT_MM)) {
        return false;
    }

    g_beamPidTargetMm = targetMm;
    stepper_reset_beam_pid();
    return true;
}

float stepper_get_beam_pid_target_mm(void)
{
    return g_beamPidTargetMm;
}

bool stepper_set_beam_pid_gains(float kpDegPerMm,
                                float kiDegPerMmS,
                                float kdDegSPerMm)
{
    if (!stepper_is_finite_float(kpDegPerMm) ||
        !stepper_is_finite_float(kiDegPerMmS) ||
        !stepper_is_finite_float(kdDegSPerMm) ||
        (kpDegPerMm < 0.0f) ||
        (kiDegPerMmS < 0.0f) ||
        (kdDegSPerMm < 0.0f) ||
        (kpDegPerMm > STEPPER_BEAM_PID_GAIN_LIMIT) ||
        (kiDegPerMmS > STEPPER_BEAM_PID_GAIN_LIMIT) ||
        (kdDegSPerMm > STEPPER_BEAM_PID_GAIN_LIMIT)) {
        return false;
    }

    g_beamPidKpDegPerMm = kpDegPerMm;
    g_beamPidKiDegPerMmS = kiDegPerMmS;
    g_beamPidKdDegSPerMm = kdDegSPerMm;
    stepper_reset_beam_pid();
    return true;
}

void stepper_get_beam_pid_gains(float *kpDegPerMm,
                                float *kiDegPerMmS,
                                float *kdDegSPerMm)
{
    if (kpDegPerMm != 0) {
        *kpDegPerMm = g_beamPidKpDegPerMm;
    }
    if (kiDegPerMmS != 0) {
        *kiDegPerMmS = g_beamPidKiDegPerMmS;
    }
    if (kdDegSPerMm != 0) {
        *kdDegSPerMm = g_beamPidKdDegSPerMm;
    }
}

void stepper_update_beam_pid(float ballPositionMm, float ballVelocityMmS,
                             bool visionValid, float dtSeconds)
{
    float errorMm;
    float integralMmS;
    float targetBeamDeg;
    float outputGain;
    float outputLimitDeg;
    float maxOutputStepDeg;
    float outputDeltaDeg;

    if (!visionValid || !g_enabled || g_emergencyInhibit) {
        stepper_reset_beam_pid();
        stepper_set_beam_target_deg(0.0f);
        stepper_set_speed_target(0.0f);
        return;
    }

    if ((dtSeconds <= 0.0f) || (dtSeconds > 0.1f)) {
        dtSeconds = 0.01f;
    }

    errorMm = g_beamPidTargetMm - ballPositionMm;
    integralMmS = g_beamPidIntegralMmS + errorMm * dtSeconds;
    integralMmS = stepper_clamp(integralMmS,
                                 -STEPPER_BEAM_PID_INTEGRAL_LIMIT_MM_S,
                                 STEPPER_BEAM_PID_INTEGRAL_LIMIT_MM_S);

    /* With a constant position target, error derivative is -ball velocity. */
    targetBeamDeg = g_beamPidKpDegPerMm * errorMm +
                    g_beamPidKiDegPerMmS * integralMmS -
                    g_beamPidKdDegSPerMm * ballVelocityMmS;
#if (STEPPER_BEAM_PID_OUTPUT_INVERT != 0U)
    targetBeamDeg = -targetBeamDeg;
#endif
    if (targetBeamDeg > 0.0f) {
        outputGain = STEPPER_BEAM_PID_POS_OUTPUT_GAIN;
    } else if (targetBeamDeg < 0.0f) {
        outputGain = STEPPER_BEAM_PID_NEG_OUTPUT_GAIN;
    } else {
        outputGain = 1.0f;
    }
    if (outputGain < 0.0f) {
        outputGain = 0.0f;
    }
    targetBeamDeg *= outputGain;

    outputLimitDeg = STEPPER_BEAM_PID_OUTPUT_LIMIT_DEG;
    targetBeamDeg = stepper_clamp(targetBeamDeg, -outputLimitDeg,
                                  outputLimitDeg);
    maxOutputStepDeg = STEPPER_BEAM_PID_OUTPUT_RATE_LIMIT_DEG_S * dtSeconds;
    if (maxOutputStepDeg < 0.0f) {
        maxOutputStepDeg = 0.0f;
    }
    outputDeltaDeg = targetBeamDeg - g_beamPidOutputDeg;
    outputDeltaDeg = stepper_clamp(outputDeltaDeg, -maxOutputStepDeg,
                                   maxOutputStepDeg);
    g_beamPidOutputDeg += outputDeltaDeg;

    g_beamPidIntegralMmS = integralMmS;
    stepper_set_beam_target_deg(g_beamPidOutputDeg);
}

void stepper_update_beam_encoder_position_loop(int32_t beamAngleDegX10,
                                               bool angleValid)
{
    float actualBeamDeg;
    float errorDeg;
    float outputHz;
    float minOutputHz;

    if (!angleValid || !g_enabled || g_emergencyInhibit) {
        stepper_set_speed_target(0.0f);
        return;
    }

    stepper_sync_position_from_beam_angle_x10(beamAngleDegX10);
    actualBeamDeg = (float)beamAngleDegX10 * 0.1f;
    errorDeg = g_beamTargetDeg - actualBeamDeg;
    if (stepper_abs(errorDeg) <= STEPPER_ENCODER_POSITION_DEADBAND_DEG) {
        stepper_set_speed_target(0.0f);
        return;
    }

    outputHz = errorDeg * STEPPER_ENCODER_POSITION_KP_HZ_PER_DEG;
#if (STEPPER_ENCODER_POSITION_OUTPUT_INVERT != 0U)
    outputHz = -outputHz;
#endif
    minOutputHz = STEPPER_ENCODER_POSITION_MIN_OUTPUT_HZ;
    if (minOutputHz > STEPPER_ENCODER_POSITION_OUTPUT_LIMIT_HZ) {
        minOutputHz = STEPPER_ENCODER_POSITION_OUTPUT_LIMIT_HZ;
    }
    if (stepper_abs(outputHz) < minOutputHz) {
        outputHz = (outputHz >= 0.0f) ? minOutputHz : -minOutputHz;
    }
    stepper_set_speed_target(
        stepper_clamp(outputHz, -STEPPER_ENCODER_POSITION_OUTPUT_LIMIT_HZ,
                      STEPPER_ENCODER_POSITION_OUTPUT_LIMIT_HZ));
}

bool stepper_start(int32_t steps, uint32_t frequencyHz)
{
    uint32_t magnitude;

    if (steps == 0) {
        return false;
    }
    if (steps > 0) {
        return stepper_start_dir((uint32_t)steps, frequencyHz, true);
    }
    magnitude = (uint32_t)(-(steps + 1));
    return stepper_start_dir(magnitude + 1U, frequencyHz, false);
}

bool stepper_start_dir(uint32_t steps, uint32_t frequencyHz, bool dirHigh)
{
    uint32_t primask;

    if ((steps == 0U) || (frequencyHz == 0U)) {
        return false;
    }

    primask = stepper_enter_critical();
    if (!stepper_can_start_locked(dirHigh)) {
        stepper_exit_critical(primask);
        return false;
    }
    g_continuous = false;
    g_remainingSteps = steps;
    g_speedTargetHz = dirHigh ? (float)frequencyHz : -(float)frequencyHz;
    g_speedCurrentHz = g_speedTargetHz;
    stepper_start_timer_locked(frequencyHz, dirHigh);
    stepper_exit_critical(primask);
    return true;
}

bool stepper_start_continuous(uint32_t frequencyHz, bool dirHigh)
{
    uint32_t primask;

    if (frequencyHz == 0U) {
        return false;
    }

    primask = stepper_enter_critical();
    if (!stepper_can_start_locked(dirHigh)) {
        stepper_exit_critical(primask);
        return false;
    }
    g_continuous = true;
    g_remainingSteps = 0U;
    g_speedTargetHz = dirHigh ? (float)frequencyHz : -(float)frequencyHz;
    g_speedCurrentHz = g_speedTargetHz;
    stepper_start_timer_locked(frequencyHz, dirHigh);
    stepper_exit_critical(primask);
    return true;
}

void stepper_stop(void)
{
    uint32_t primask = stepper_enter_critical();

    g_continuous = false;
    g_remainingSteps = 0U;
    g_speedTargetHz = 0.0f;
    g_speedCurrentHz = 0.0f;
    stepper_request_stop_locked();
    stepper_exit_critical(primask);
}

bool stepper_is_busy(void)
{
    return g_running;
}

bool stepper_get_direction(void)
{
    return g_directionHigh;
}

uint32_t stepper_get_frequency_hz(void)
{
    return g_frequencyHz;
}

uint32_t stepper_get_remaining_steps(void)
{
    return g_remainingSteps;
}

void stepper_timer_irq_handler(void)
{
    switch (DL_TimerA_getPendingInterrupt(STEPPER_PWM_INST)) {
    case DL_TIMER_IIDX_CC0_DN:
        if (g_running) {
            stepper_count_accepted_pulse();
            if (!g_continuous && (g_remainingSteps > 0U)) {
                g_remainingSteps--;
            }
            if (g_stopPending || (!g_continuous && (g_remainingSteps == 0U))) {
                g_speedTargetHz = 0.0f;
                g_speedCurrentHz = 0.0f;
                stepper_stop_timer_locked();
            }
        }
        break;

    default:
        break;
    }
}

void TIMA1_IRQHandler(void)
{
    stepper_timer_irq_handler();
}
