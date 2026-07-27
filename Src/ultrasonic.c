#include "ultrasonic.h"

#include "ti_msp_dl_config.h"

/*
 * HC-SR04 style ultrasonic ranging.
 *
 * PA24 outputs a 10 us trigger pulse. PA9 receives the echo pulse.
 * The measurement function is blocking, so call it from the foreground loop
 * at a low rate. Do not call it from an ISR or a tight motor-control path.
 */

#define ULTRASONIC_TRIG_PORT             (GPIOA)
#define ULTRASONIC_TRIG_PIN              (DL_GPIO_PIN_24)
#define ULTRASONIC_TRIG_IOMUX            (IOMUX_PINCM54)

#define ULTRASONIC_ECHO_PORT             (GPIOA)
#define ULTRASONIC_ECHO_PIN              (DL_GPIO_PIN_9)
#define ULTRASONIC_ECHO_IOMUX            (IOMUX_PINCM20)

#define ULTRASONIC_TRIGGER_LOW_US        (2U)
#define ULTRASONIC_TRIGGER_HIGH_US       (10U)
#define ULTRASONIC_ECHO_START_TIMEOUT_US (3000U)
#define ULTRASONIC_ECHO_MAX_US           (24000U)

static ultrasonic_measurement_t g_lastMeasurement = {
    ULTRASONIC_STATUS_ECHO_START_TIMEOUT,
    ULTRASONIC_NO_OBJECT_MM,
    0U
};

static void ultrasonic_delay_us(uint32_t us)
{
    uint32_t cyclesPerUs = CPUCLK_FREQ / 1000000U;

    while (us > 0U) {
        delay_cycles(cyclesPerUs);
        us--;
    }
}

static bool ultrasonic_echo_is_high(void)
{
    return (DL_GPIO_readPins(ULTRASONIC_ECHO_PORT,
                             ULTRASONIC_ECHO_PIN) != 0U);
}

static uint16_t ultrasonic_us_to_mm(uint32_t echoTimeUs)
{
    uint32_t distanceMm;

    /*
     * Distance = echo time * speed of sound / 2.
     * 343 m/s is 0.343 mm/us, so distance mm = us * 343 / 2000.
     */
    distanceMm = ((echoTimeUs * 343U) + 1000U) / 1000U;

    if (distanceMm > ULTRASONIC_MAX_DISTANCE_MM) {
        return ULTRASONIC_NO_OBJECT_MM;
    }

    return (uint16_t) distanceMm;
}

void ultrasonic_init(void)
{
    DL_GPIO_initDigitalOutput(ULTRASONIC_TRIG_IOMUX);
    DL_GPIO_initDigitalInputFeatures(ULTRASONIC_ECHO_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_clearPins(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN);
    DL_GPIO_enableOutput(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN);

    g_lastMeasurement.status = ULTRASONIC_STATUS_ECHO_START_TIMEOUT;
    g_lastMeasurement.distanceMm = ULTRASONIC_NO_OBJECT_MM;
    g_lastMeasurement.echoTimeUs = 0U;
}

bool ultrasonic_measure(ultrasonic_measurement_t *measurement)
{
    uint32_t timeoutUs;
    uint32_t echoTimeUs;

    if (measurement == 0) {
        g_lastMeasurement.status = ULTRASONIC_STATUS_NULL_POINTER;
        g_lastMeasurement.distanceMm = ULTRASONIC_NO_OBJECT_MM;
        g_lastMeasurement.echoTimeUs = 0U;
        return false;
    }

    DL_GPIO_clearPins(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN);
    ultrasonic_delay_us(ULTRASONIC_TRIGGER_LOW_US);
    DL_GPIO_setPins(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN);
    ultrasonic_delay_us(ULTRASONIC_TRIGGER_HIGH_US);
    DL_GPIO_clearPins(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN);

    timeoutUs = 0U;
    while (!ultrasonic_echo_is_high()) {
        if (timeoutUs >= ULTRASONIC_ECHO_START_TIMEOUT_US) {
            g_lastMeasurement.status =
                ULTRASONIC_STATUS_ECHO_START_TIMEOUT;
            g_lastMeasurement.distanceMm = ULTRASONIC_NO_OBJECT_MM;
            g_lastMeasurement.echoTimeUs = 0U;
            *measurement = g_lastMeasurement;
            return false;
        }

        ultrasonic_delay_us(1U);
        timeoutUs++;
    }

    echoTimeUs = 0U;
    while (ultrasonic_echo_is_high()) {
        if (echoTimeUs >= ULTRASONIC_ECHO_MAX_US) {
            g_lastMeasurement.status = ULTRASONIC_STATUS_ECHO_END_TIMEOUT;
            g_lastMeasurement.distanceMm = ULTRASONIC_NO_OBJECT_MM;
            g_lastMeasurement.echoTimeUs = echoTimeUs;
            *measurement = g_lastMeasurement;
            return false;
        }

        ultrasonic_delay_us(1U);
        echoTimeUs++;
    }

    g_lastMeasurement.status = ULTRASONIC_STATUS_OK;
    g_lastMeasurement.distanceMm = ultrasonic_us_to_mm(echoTimeUs);
    g_lastMeasurement.echoTimeUs = echoTimeUs;
    *measurement = g_lastMeasurement;

    return g_lastMeasurement.distanceMm != ULTRASONIC_NO_OBJECT_MM;
}

bool ultrasonic_read_mm(uint16_t *distanceMm)
{
    ultrasonic_measurement_t measurement;

    if (distanceMm == 0) {
        g_lastMeasurement.status = ULTRASONIC_STATUS_NULL_POINTER;
        g_lastMeasurement.distanceMm = ULTRASONIC_NO_OBJECT_MM;
        g_lastMeasurement.echoTimeUs = 0U;
        return false;
    }

    if (!ultrasonic_measure(&measurement)) {
        *distanceMm = ULTRASONIC_NO_OBJECT_MM;
        return false;
    }

    *distanceMm = measurement.distanceMm;
    return true;
}

ultrasonic_measurement_t ultrasonic_get_last_measurement(void)
{
    return g_lastMeasurement;
}
