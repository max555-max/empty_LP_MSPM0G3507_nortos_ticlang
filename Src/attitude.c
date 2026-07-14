#include "attitude.h"

#include <math.h>

#include "delay.h"
#include "vofa.h"

#define ATTITUDE_GYRO_DPS_PER_LSB      (2000.0f / 32768.0f)
#define ATTITUDE_DEG_TO_RAD            (0.017453292519943295f)
#define ATTITUDE_RAD_TO_DEG            (57.29577951308232f)
#define ATTITUDE_TWO_KP                (0.7f)
#define ATTITUDE_TWO_KI                (0.0f)
#define ATTITUDE_ACCEL_LPF_ALPHA       (0.15f)
#define ATTITUDE_ACCEL_1G_LSB          (2048.0f)
#define ATTITUDE_ACCEL_MIN_G           (0.70f)
#define ATTITUDE_ACCEL_MAX_G           (1.30f)
#define ATTITUDE_DT_MAX_S              (0.05f)
#define ATTITUDE_GYRO_DEADBAND_DPS     (2.00f)
#define ATTITUDE_YAW_DEADBAND_DPS      (10.00f)
#define ATTITUDE_YAW_LPF_ALPHA         (0.20f)
#define ATTITUDE_GYRO_BIAS_LEARN_ALPHA (0.01f)
#define ATTITUDE_GYRO_BIAS_LEARN_DPS   (12.00f)
#define ATTITUDE_COMPLEMENTARY_ALPHA   (0.98f)

static float q0 = 1.0f;
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;
static float integralFbX = 0.0f;
static float integralFbY = 0.0f;
static float integralFbZ = 0.0f;
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
static float yawRateFiltDps = 0.0f;

static float attitude_inv_sqrt(float x);
static float attitude_clamp(float x, float minValue, float maxValue);
static float attitude_apply_deadband(float value, float deadband);
static float attitude_wrap_180(float angle);
static void attitude_init_from_accel(float ax, float ay, float az);
static void attitude_accel_to_euler(float ax, float ay, float az, float *roll, float *pitch);

void attitude_init(void)
{
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    integralFbX = 0.0f;
    integralFbY = 0.0f;
    integralFbZ = 0.0f;
    accFiltX = 0.0f;
    accFiltY = 0.0f;
    accFiltZ = ATTITUDE_ACCEL_1G_LSB;
    accFilterReady = false;
    gyroBiasX = 0.0f;
    gyroBiasY = 0.0f;
    gyroBiasZ = 0.0f;
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
    yawRateFiltDps = 0.0f;
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
    attitude_init_from_accel(accFiltX, accFiltY, accFiltZ);
    gyroCalibrated = true;
}

bool attitude_calibrate_gyro_step(const icm42688_raw_t *raw, uint16_t sampleCount)
{
    if (gyroCalibrated == true) {
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
    attitude_init_from_accel(accFiltX, accFiltY, accFiltZ);
    gyroCalibrated = true;

    return true;
}

bool attitude_is_gyro_calibrated(void)
{
    return gyroCalibrated;
}

bool attitude_update_from_icm42688(const icm42688_raw_t *raw, float dt)
{
    float ax;
    float ay;
    float az;
    float accMag;
    float rollAcc;
    float pitchAcc;
    float rollAccRel;
    float pitchAccRel;
    float yawRateDps;

    if ((raw == 0) || (dt <= 0.0f)) {
        return false;
    }

    dt = attitude_clamp(dt, 0.001f, ATTITUDE_DT_MAX_S);

    if (accFilterReady == false) {
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
        accMag = sqrtf(ax * ax + ay * ay + az * az) / ATTITUDE_ACCEL_1G_LSB;
        if ((accMag >= ATTITUDE_ACCEL_MIN_G) && (accMag <= ATTITUDE_ACCEL_MAX_G)) {
            attitude_accel_to_euler(ax, ay, az, &rollAcc, &pitchAcc);
            rollAccRel = attitude_wrap_180(rollAcc - rollZeroDeg);
            pitchAccRel = attitude_wrap_180(pitchAcc - pitchZeroDeg);

            /*
             * 调试稳定版：roll/pitch 只看加速度相对上电零点。
             * 先排除陀螺积分和坐标融合导致的静止漂移。
             */
            rollDeg = rollAccRel;
            pitchDeg = pitchAccRel;
        }
    }

    /*
     * yaw 没有加速度/磁力计绝对参考，只能用 gyroZ 积分。
     * 这里加低通和较大死区，优先保证静止时不乱飞。
     */
    yawRateDps = ((float) raw->gyroZ - gyroBiasZ) * ATTITUDE_GYRO_DPS_PER_LSB;

    if (fabsf(yawRateDps) < ATTITUDE_GYRO_BIAS_LEARN_DPS) {
        /*
         * 静止或近似静止时，不积分 yaw，而是继续微调 Z 轴零偏。
         * 这样能压住 gyroZ 残余零偏导致的 yaw 自漂。
         */
        gyroBiasZ += ATTITUDE_GYRO_BIAS_LEARN_ALPHA * ((float) raw->gyroZ - gyroBiasZ);
        yawRateFiltDps = 0.0f;
    } else {
        yawRateFiltDps += ATTITUDE_YAW_LPF_ALPHA * (yawRateDps - yawRateFiltDps);
        yawRateDps = attitude_apply_deadband(yawRateFiltDps, ATTITUDE_YAW_DEADBAND_DPS);
        yawDeg += yawRateDps * dt;
    }

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

static float attitude_inv_sqrt(float x)
{
    return 1.0f / sqrtf(x);
}

static float attitude_clamp(float x, float minValue, float maxValue)
{
    if (x < minValue) {
        return minValue;
    }
    if (x > maxValue) {
        return maxValue;
    }
    return x;
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

static void attitude_init_from_accel(float ax, float ay, float az)
{
    float roll;
    float pitch;

    if ((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)) {
        return;
    }

    attitude_accel_to_euler(ax, ay, az, &roll, &pitch);
    rollZeroDeg = roll;
    pitchZeroDeg = pitch;
    rollDeg = 0.0f;
    pitchDeg = 0.0f;
    yawDeg = 0.0f;
}

static void attitude_accel_to_euler(float ax, float ay, float az, float *roll, float *pitch)
{
    if (roll != 0) {
        *roll = atan2f(ay, az) * ATTITUDE_RAD_TO_DEG;
    }

    if (pitch != 0) {
        *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * ATTITUDE_RAD_TO_DEG;
    }
}
