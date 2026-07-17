#include "attitude.h"

#include <math.h>

#include "delay.h"
#include "vofa.h"

/*
 * ICM42688 is configured in icm42688.c as:
 *   accel: +/-16g  -> 2048 LSB/g
 *   gyro : +/-2000 dps -> 16.384 LSB/(deg/s)
 */
#define ATTITUDE_ACCEL_1G_LSB             (2048.0f)
#define ATTITUDE_GYRO_DPS_PER_LSB         (2000.0f / 32768.0f)
#define ATTITUDE_RAD_TO_DEG               (57.29577951308232f)
#define ATTITUDE_DEG_TO_RAD               (0.017453292519943295f)

/*
 * Complementary filter:
 *   0.98: trust gyro more, smoother but drifts slightly more.
 *   0.95: trust accel more, less drift but noisier.
 */
#define ATTITUDE_COMPLEMENTARY_ALPHA       (0.98f)
#define ATTITUDE_ACCEL_LPF_ALPHA           (0.20f)
#define ATTITUDE_ACCEL_MIN_G               (0.70f)
#define ATTITUDE_ACCEL_MAX_G               (1.30f)
#define ATTITUDE_DT_MIN_S                  (0.001f)
#define ATTITUDE_DT_MAX_S                  (0.050f)
#define ATTITUDE_GYRO_DEADBAND_DPS         (0.15f)
#define ATTITUDE_YAW_DEADBAND_DPS          (0.30f)
#define ATTITUDE_STATIONARY_GYRO_DPS       (0.80f)
#define ATTITUDE_GYRO_BIAS_LEARN_ALPHA     (0.002f)
#define ATTITUDE_PITCH_LIMIT_DEG           (89.0f)

static float gyroBiasX = 0.0f;
static float gyroBiasY = 0.0f;
static float gyroBiasZ = 0.0f;

static float accFiltX = 0.0f;
static float accFiltY = 0.0f;
static float accFiltZ = ATTITUDE_ACCEL_1G_LSB;
static bool accFilterReady = false;

static bool gyroCalibrated = false;
static uint16_t gyroCalCount = 0U;
static int32_t gyroCalSumX = 0;
static int32_t gyroCalSumY = 0;
static int32_t gyroCalSumZ = 0;
static int32_t accCalSumX = 0;
static int32_t accCalSumY = 0;
static int32_t accCalSumZ = 0;

static float rollDeg = 0.0f;
static float pitchDeg = 0.0f;
static float yawDeg = 0.0f;
static float rollZeroDeg = 0.0f;
static float pitchZeroDeg = 0.0f;

static float attitude_clamp(float value, float minValue, float maxValue);
static float attitude_apply_deadband(float value, float deadband);
static float attitude_wrap_180(float angle);
static void attitude_accel_to_euler(
    float ax, float ay, float az, float *roll, float *pitch);
static void attitude_set_zero_from_accel(float ax, float ay, float az);

void attitude_init(void)
{
    gyroBiasX = 0.0f;
    gyroBiasY = 0.0f;
    gyroBiasZ = 0.0f;

    accFiltX = 0.0f;
    accFiltY = 0.0f;
    accFiltZ = ATTITUDE_ACCEL_1G_LSB;
    accFilterReady = false;

    gyroCalibrated = false;
    gyroCalCount = 0U;
    gyroCalSumX = 0;
    gyroCalSumY = 0;
    gyroCalSumZ = 0;
    accCalSumX = 0;
    accCalSumY = 0;
    accCalSumZ = 0;

    rollDeg = 0.0f;
    pitchDeg = 0.0f;
    yawDeg = 0.0f;
    rollZeroDeg = 0.0f;
    pitchZeroDeg = 0.0f;
}

void attitude_calibrate_gyro(uint16_t sampleCount)
{
    icm42688_raw_t raw;
    int32_t sumX = 0;
    int32_t sumY = 0;
    int32_t sumZ = 0;
    int32_t sumAx = 0;
    int32_t sumAy = 0;
    int32_t sumAz = 0;

    if (sampleCount == 0U) {
        return;
    }

    for (uint16_t i = 0U; i < sampleCount; i++) {
        icm42688_read_raw(&raw);
        sumX += raw.gyroX;
        sumY += raw.gyroY;
        sumZ += raw.gyroZ;
        sumAx += raw.accelX;
        sumAy += raw.accelY;
        sumAz += raw.accelZ;
        delay_ms(2U);
    }

    gyroBiasX = (float) sumX / (float) sampleCount;
    gyroBiasY = (float) sumY / (float) sampleCount;
    gyroBiasZ = (float) sumZ / (float) sampleCount;

    accFiltX = (float) sumAx / (float) sampleCount;
    accFiltY = (float) sumAy / (float) sampleCount;
    accFiltZ = (float) sumAz / (float) sampleCount;
    accFilterReady = true;

    attitude_set_zero_from_accel(accFiltX, accFiltY, accFiltZ);
    gyroCalibrated = true;
}

bool attitude_calibrate_gyro_step(
    const icm42688_raw_t *raw, uint16_t sampleCount)
{
    if (gyroCalibrated) {
        return true;
    }

    if ((raw == 0) || (sampleCount == 0U)) {
        return false;
    }

    gyroCalSumX += raw->gyroX;
    gyroCalSumY += raw->gyroY;
    gyroCalSumZ += raw->gyroZ;
    accCalSumX += raw->accelX;
    accCalSumY += raw->accelY;
    accCalSumZ += raw->accelZ;
    gyroCalCount++;

    if (gyroCalCount < sampleCount) {
        return false;
    }

    gyroBiasX = (float) gyroCalSumX / (float) gyroCalCount;
    gyroBiasY = (float) gyroCalSumY / (float) gyroCalCount;
    gyroBiasZ = (float) gyroCalSumZ / (float) gyroCalCount;

    accFiltX = (float) accCalSumX / (float) gyroCalCount;
    accFiltY = (float) accCalSumY / (float) gyroCalCount;
    accFiltZ = (float) accCalSumZ / (float) gyroCalCount;
    accFilterReady = true;

    attitude_set_zero_from_accel(accFiltX, accFiltY, accFiltZ);
    gyroCalibrated = true;

    return true;
}

bool attitude_is_gyro_calibrated(void)
{
    return gyroCalibrated;
}

bool attitude_update_from_icm42688(const icm42688_raw_t *raw, float dt)
{
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
    float accMagG;
    float rollRad;
    float pitchRad;
    float rollRateDps;
    float pitchRateDps;
    float rollAccDeg;
    float pitchAccDeg;
    float rollAccRelDeg;
    float pitchAccRelDeg;

    if ((raw == 0) || (dt <= 0.0f)) {
        return false;
    }

    dt = attitude_clamp(dt, ATTITUDE_DT_MIN_S, ATTITUDE_DT_MAX_S);

    gx = ((float) raw->gyroX - gyroBiasX) * ATTITUDE_GYRO_DPS_PER_LSB;
    gy = ((float) raw->gyroY - gyroBiasY) * ATTITUDE_GYRO_DPS_PER_LSB;
    gz = ((float) raw->gyroZ - gyroBiasZ) * ATTITUDE_GYRO_DPS_PER_LSB;

    /*
     * If all gyro axes are very close to zero, treat the board as stationary
     * and slowly refine the zero bias. This mainly suppresses yaw drift.
     */
    if ((fabsf(gx) < ATTITUDE_STATIONARY_GYRO_DPS) &&
        (fabsf(gy) < ATTITUDE_STATIONARY_GYRO_DPS) &&
        (fabsf(gz) < ATTITUDE_STATIONARY_GYRO_DPS)) {
        gyroBiasX += ATTITUDE_GYRO_BIAS_LEARN_ALPHA *
                     ((float) raw->gyroX - gyroBiasX);
        gyroBiasY += ATTITUDE_GYRO_BIAS_LEARN_ALPHA *
                     ((float) raw->gyroY - gyroBiasY);
        gyroBiasZ += ATTITUDE_GYRO_BIAS_LEARN_ALPHA *
                     ((float) raw->gyroZ - gyroBiasZ);

        gx = ((float) raw->gyroX - gyroBiasX) * ATTITUDE_GYRO_DPS_PER_LSB;
        gy = ((float) raw->gyroY - gyroBiasY) * ATTITUDE_GYRO_DPS_PER_LSB;
        gz = ((float) raw->gyroZ - gyroBiasZ) * ATTITUDE_GYRO_DPS_PER_LSB;
    }

    gx = attitude_apply_deadband(gx, ATTITUDE_GYRO_DEADBAND_DPS);
    gy = attitude_apply_deadband(gy, ATTITUDE_GYRO_DEADBAND_DPS);
    gz = attitude_apply_deadband(gz, ATTITUDE_YAW_DEADBAND_DPS);

    /*
     * Gyro integration. This Euler-rate form works much better than simply
     * roll += gx, pitch += gy when the board is already tilted.
     */
    pitchDeg = attitude_clamp(
        pitchDeg, -ATTITUDE_PITCH_LIMIT_DEG, ATTITUDE_PITCH_LIMIT_DEG);
    rollRad = rollDeg * ATTITUDE_DEG_TO_RAD;
    pitchRad = pitchDeg * ATTITUDE_DEG_TO_RAD;

    rollRateDps = gx +
                  sinf(rollRad) * tanf(pitchRad) * gy +
                  cosf(rollRad) * tanf(pitchRad) * gz;
    pitchRateDps = cosf(rollRad) * gy - sinf(rollRad) * gz;

    rollDeg += rollRateDps * dt;
    pitchDeg += pitchRateDps * dt;
    yawDeg += gz * dt;

    /*
     * Accelerometer low-pass and roll/pitch correction.
     * Accel is ignored during strong acceleration/vibration.
     */
    if (!accFilterReady) {
        accFiltX = (float) raw->accelX;
        accFiltY = (float) raw->accelY;
        accFiltZ = (float) raw->accelZ;
        accFilterReady = true;
    } else {
        accFiltX += ATTITUDE_ACCEL_LPF_ALPHA * ((float) raw->accelX - accFiltX);
        accFiltY += ATTITUDE_ACCEL_LPF_ALPHA * ((float) raw->accelY - accFiltY);
        accFiltZ += ATTITUDE_ACCEL_LPF_ALPHA * ((float) raw->accelZ - accFiltZ);
    }

    ax = accFiltX;
    ay = accFiltY;
    az = accFiltZ;

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        accMagG = sqrtf(ax * ax + ay * ay + az * az) / ATTITUDE_ACCEL_1G_LSB;

        if ((accMagG >= ATTITUDE_ACCEL_MIN_G) &&
            (accMagG <= ATTITUDE_ACCEL_MAX_G)) {
            attitude_accel_to_euler(ax, ay, az, &rollAccDeg, &pitchAccDeg);
            rollAccRelDeg = attitude_wrap_180(rollAccDeg - rollZeroDeg);
            pitchAccRelDeg = attitude_wrap_180(pitchAccDeg - pitchZeroDeg);

            rollDeg = ATTITUDE_COMPLEMENTARY_ALPHA * rollDeg +
                      (1.0f - ATTITUDE_COMPLEMENTARY_ALPHA) * rollAccRelDeg;
            pitchDeg = ATTITUDE_COMPLEMENTARY_ALPHA * pitchDeg +
                       (1.0f - ATTITUDE_COMPLEMENTARY_ALPHA) * pitchAccRelDeg;
        }
    }

    rollDeg = attitude_wrap_180(rollDeg);
    pitchDeg = attitude_clamp(
        pitchDeg, -ATTITUDE_PITCH_LIMIT_DEG, ATTITUDE_PITCH_LIMIT_DEG);
    yawDeg = attitude_wrap_180(yawDeg);

    return true;
}

void attitude_get_euler(attitude_euler_t *euler)
{
    if (euler == 0) {
        return;
    }

    euler->roll = rollDeg;
    euler->pitch = pitchDeg;
    euler->yaw = yawDeg;
}

void attitude_print_euler(const attitude_euler_t *euler)
{
    if (euler == 0) {
        return;
    }

    uart0_send_string("samples:");
    uart0_send_float(euler->roll, 2U);
    uart0_send_byte(',');
    uart0_send_float(euler->pitch, 2U);
    uart0_send_byte(',');
    uart0_send_float(euler->yaw, 2U);
    uart0_send_byte('\n');
}

static float attitude_clamp(float value, float minValue, float maxValue)
{
    if (value < minValue) {
        return minValue;
    }

    if (value > maxValue) {
        return maxValue;
    }

    return value;
}

static float attitude_apply_deadband(float value, float deadband)
{
    if ((value > -deadband) && (value < deadband)) {
        return 0.0f;
    }

    return value;
}

static float attitude_wrap_180(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }

    while (angle < -180.0f) {
        angle += 360.0f;
    }

    return angle;
}

static void attitude_accel_to_euler(
    float ax, float ay, float az, float *roll, float *pitch)
{
    if (roll != 0) {
        *roll = atan2f(ay, az) * ATTITUDE_RAD_TO_DEG;
    }

    if (pitch != 0) {
        *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * ATTITUDE_RAD_TO_DEG;
    }
}

static void attitude_set_zero_from_accel(float ax, float ay, float az)
{
    attitude_accel_to_euler(ax, ay, az, &rollZeroDeg, &pitchZeroDeg);

    rollDeg = 0.0f;
    pitchDeg = 0.0f;
    yawDeg = 0.0f;
}
