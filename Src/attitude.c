#include "attitude.h"

#include <math.h>

#include "delay.h"
#include "vofa.h"

/*
 * attitude.c
 *
 * 姿态解算模块。
 *
 * 当前算法：
 *   roll/pitch：陀螺仪积分 + 加速度计互补滤波修正；
 *   yaw       ：Z 轴陀螺仪积分。
 *
 * 注意：
 *   没有磁力计时，yaw 没有绝对参考，只能依靠陀螺仪积分。
 *   因此 yaw 会有少量漂移，静止零偏学习只能减小漂移，不能完全消除。
 */

/*
 * ICM42688 当前在 icm42688.c 中配置为：
 *   accel：±16g     -> 2048 LSB/g
 *   gyro ：±2000dps -> 16.384 LSB/(deg/s)
 */
#define ATTITUDE_ACCEL_1G_LSB             (2048.0f)
#define ATTITUDE_GYRO_DPS_PER_LSB         (2000.0f / 32768.0f)
#define ATTITUDE_RAD_TO_DEG               (57.29577951308232f)
#define ATTITUDE_DEG_TO_RAD               (0.017453292519943295f)

/*
 * 互补滤波系数：
 *   0.98：更相信陀螺仪，曲线更平滑，但漂移略大；
 *   0.95：更相信加速度计，漂移更小，但更容易受震动影响。
 */
#define ATTITUDE_COMPLEMENTARY_ALPHA       (0.98f)
#define ATTITUDE_ACCEL_LPF_ALPHA           (0.20f)
#define ATTITUDE_ACCEL_MIN_G               (0.70f)
#define ATTITUDE_ACCEL_MAX_G               (1.30f)
#define ATTITUDE_DT_MIN_S                  (0.001f)
#define ATTITUDE_DT_MAX_S                  (0.050f)
#define ATTITUDE_GYRO_DEADBAND_DPS         (0.15f)
#define ATTITUDE_YAW_DEADBAND_DPS          (0.20f)
#define ATTITUDE_STATIONARY_GYRO_DPS       (0.80f)
#define ATTITUDE_GYRO_BIAS_LEARN_ALPHA     (0.002f)
#define ATTITUDE_PITCH_LIMIT_DEG           (89.0f)

/* 陀螺仪零偏，上电静止校准得到。 */
static float gyroBiasX = 0.0f;
static float gyroBiasY = 0.0f;
static float gyroBiasZ = 0.0f;

/* 加速度计低通滤波值，用于 roll/pitch 修正。 */
static float accFiltX = 0.0f;
static float accFiltY = 0.0f;
static float accFiltZ = ATTITUDE_ACCEL_1G_LSB;
static bool accFilterReady = false;

/* 非阻塞校准状态。 */
static bool gyroCalibrated = false;
static uint16_t gyroCalCount = 0U;
static int32_t gyroCalSumX = 0;
static int32_t gyroCalSumY = 0;
static int32_t gyroCalSumZ = 0;
static int32_t accCalSumX = 0;
static int32_t accCalSumY = 0;
static int32_t accCalSumZ = 0;

/* 当前欧拉角。 */
static float rollDeg = 0.0f;
static float pitchDeg = 0.0f;
static float yawDeg = 0.0f;
/* 上电静止时的 roll/pitch 零点，用来抵消安装倾角。 */
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
    /* 清空零偏、滤波、校准和姿态状态。 */
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

    /*
     * 阻塞式校准：
     *   小车必须静止；
     *   采集 sampleCount 次陀螺仪原始值求平均，作为零偏。
     */
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

    /* 陀螺仪零偏平均值。 */
    gyroBiasX = (float) sumX / (float) sampleCount;
    gyroBiasY = (float) sumY / (float) sampleCount;
    gyroBiasZ = (float) sumZ / (float) sampleCount;

    /* 加速度计平均值用于建立 roll/pitch 零点。 */
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
        /* 已校准完成时，后续调用直接返回 true。 */
        return true;
    }

    if ((raw == 0) || (sampleCount == 0U)) {
        return false;
    }

    /* 非阻塞校准：每调用一次累加一帧原始数据。 */
    gyroCalSumX += raw->gyroX;
    gyroCalSumY += raw->gyroY;
    gyroCalSumZ += raw->gyroZ;
    accCalSumX += raw->accelX;
    accCalSumY += raw->accelY;
    accCalSumZ += raw->accelZ;
    gyroCalCount++;

    if (gyroCalCount < sampleCount) {
        /* 样本数还没攒够，校准未完成。 */
        return false;
    }

    /* 样本数足够后计算平均零偏。 */
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
    float yawRateDps;
    float rollAccDeg;
    float pitchAccDeg;
    float rollAccRelDeg;
    float pitchAccRelDeg;

    if ((raw == 0) || (dt <= 0.0f)) {
        return false;
    }

    /* 限制 dt，避免任务卡顿或异常 dt 导致积分突变。 */
    dt = attitude_clamp(dt, ATTITUDE_DT_MIN_S, ATTITUDE_DT_MAX_S);

    /* 原始陀螺仪值减去零偏，再换算为 deg/s。 */
    gx = ((float) raw->gyroX - gyroBiasX) * ATTITUDE_GYRO_DPS_PER_LSB;
    gy = ((float) raw->gyroY - gyroBiasY) * ATTITUDE_GYRO_DPS_PER_LSB;
    gz = ((float) raw->gyroZ - gyroBiasZ) * ATTITUDE_GYRO_DPS_PER_LSB;

    /*
     * 静止零偏慢学习：
     *   如果三轴角速度都很小，认为板子静止；
     *   此时缓慢修正 gyroBias，主要用于减小 yaw 漂移。
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

    /* 小角速度死区，过滤静止时的微小抖动。 */
    gx = attitude_apply_deadband(gx, ATTITUDE_GYRO_DEADBAND_DPS);
    gy = attitude_apply_deadband(gy, ATTITUDE_GYRO_DEADBAND_DPS);
    gz = attitude_apply_deadband(gz, ATTITUDE_YAW_DEADBAND_DPS);

    /*
     * 陀螺仪积分。
     *
     * 这里没有简单使用 roll += gx、pitch += gy；
     * 而是按欧拉角角速度关系计算 roll/pitch 变化，
     * 这样在板子已经倾斜时会更稳定。
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

    /* yaw 没有加速度计修正，只由 Z 轴陀螺仪积分得到。 */
    yawRateDps =
    sinf(rollRad) / cosf(pitchRad) * gy +
    cosf(rollRad) / cosf(pitchRad) * gz;

    yawDeg += yawRateDps * dt;

    /*
     * 加速度计低通滤波，并用于修正 roll/pitch。
     *
     * 如果加速度模长明显不是 1g，说明车体正在加速/震动，
     * 此时不使用加速度计修正，避免把运动加速度误当成重力方向。
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
            /* 加速度计得到的是绝对倾角，需要减去上电零点。 */
            attitude_accel_to_euler(ax, ay, az, &rollAccDeg, &pitchAccDeg);
            rollAccRelDeg = attitude_wrap_180(rollAccDeg - rollZeroDeg);
            pitchAccRelDeg = attitude_wrap_180(pitchAccDeg - pitchZeroDeg);

            /* 互补滤波：陀螺积分为主，加速度计慢慢拉回。 */
            rollDeg = ATTITUDE_COMPLEMENTARY_ALPHA * rollDeg +
                      (1.0f - ATTITUDE_COMPLEMENTARY_ALPHA) * rollAccRelDeg;
            pitchDeg = ATTITUDE_COMPLEMENTARY_ALPHA * pitchDeg +
                       (1.0f - ATTITUDE_COMPLEMENTARY_ALPHA) * pitchAccRelDeg;
        }
    }

    /* 输出角度范围限制，避免跨越 ±180° 时数值无限增长。 */
    rollDeg = attitude_wrap_180(rollDeg);
    pitchDeg = attitude_clamp(
        pitchDeg, -ATTITUDE_PITCH_LIMIT_DEG, ATTITUDE_PITCH_LIMIT_DEG);
    yawDeg = attitude_wrap_180(yawDeg);

    return true;
}

void attitude_get_euler(attitude_euler_t *euler)
{
    if (euler == 0) {
        /* 防止空指针。 */
        return;
    }

    /* 输出当前姿态角。 */
    euler->roll = rollDeg;
    euler->pitch = pitchDeg;
    euler->yaw = yawDeg;
}

void attitude_print_euler(const attitude_euler_t *euler)
{
    if (euler == 0) {
        return;
    }

    /* VOFA FireWater 格式：samples:roll,pitch,yaw */
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
    /* 通用限幅函数。 */
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
    /* 小于死区的值直接置 0，减小静止抖动。 */
    if ((value > -deadband) && (value < deadband)) {
        return 0.0f;
    }

    return value;
}

static float attitude_wrap_180(float angle)
{
    /* 将角度归一化到 -180~180。 */
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
    /* 根据重力方向计算 roll/pitch。 */
    if (roll != 0) {
        *roll = atan2f(ay, az) * ATTITUDE_RAD_TO_DEG;
    }

    if (pitch != 0) {
        *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * ATTITUDE_RAD_TO_DEG;
    }
}

static void attitude_set_zero_from_accel(float ax, float ay, float az)
{
    /* 用上电静止时的加速度方向建立 roll/pitch 零点。 */
    attitude_accel_to_euler(ax, ay, az, &rollZeroDeg, &pitchZeroDeg);

    /* 校准完成后姿态从 0 开始。 */
    rollDeg = 0.0f;
    pitchDeg = 0.0f;
    yawDeg = 0.0f;
}
