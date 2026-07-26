#include "attitude.h"

#include <math.h>

#include "delay.h"
#include "vofa.h"

/*
 * 本文件与 ICM42688 驱动中的量程配置必须保持一致：
 *
 *   加速度计：±16 g     -> 2048 LSB/g
 *   陀螺仪：  ±2000 dps -> 16.384 LSB/(deg/s)
 */
#define ATTITUDE_EXPECTED_WHO_AM_I          (0x47U)
#define ATTITUDE_ACCEL_1G_LSB               (2048.0f)
#define ATTITUDE_GYRO_DPS_PER_LSB           (2000.0f / 32768.0f)

#define ATTITUDE_RAD_TO_DEG                 (57.29577951308232f)
#define ATTITUDE_DEG_TO_RAD                 (0.017453292519943295f)

/*
 * Z轴角速度比例标定系数。
 *
 * 初始保持 1.0。
 * 如果实际旋转 360°，程序累计角度为 measuredDeg：
 *
 *   新系数 = 360.0f / measuredDeg
 *
 * 推荐正反方向分别转多圈后取平均值再填写。
 */
/*
 * Calibration formula:
 *   ATTITUDE_YAW_SCALE = actual_turn_deg / reported_yaw_deg
 *
 * Example: actual 90 deg, reported 80 deg -> 90 / 80 = 1.125f.
 */
#define ATTITUDE_YAW_SCALE                  (0.991000f)

/*
 * roll、pitch 互补滤波。
 * 0.98 表示短期主要相信陀螺仪，长期由加速度计纠偏。
 */
#define ATTITUDE_COMPLEMENTARY_ALPHA        (0.98f)

/* 加速度计一阶低通滤波系数。 */
#define ATTITUDE_ACCEL_LPF_ALPHA            (0.20f)

/*
 * 只有加速度模长接近重力时，才允许使用加速度计纠正 roll、pitch。
 */
#define ATTITUDE_ACCEL_MIN_G                (0.70f)
#define ATTITUDE_ACCEL_MAX_G                (1.30f)

/* 姿态更新时间范围。 */
#define ATTITUDE_DT_MIN_S                   (0.001f)
#define ATTITUDE_DT_MAX_S                   (0.050f)

/*
 * 陀螺仪死区。
 *
 * yaw 死区过大时会漏掉低速转动，导致固定角度旋转偏小。
 * 0.05 dps 作为较保守的初始值。
 */
#define ATTITUDE_GYRO_DEADBAND_DPS          (0.10f)
#define ATTITUDE_YAW_DEADBAND_DPS           (0.05f)

/*
 * 静止在线零偏学习。
 *
 * 阈值设置得较小，避免把真实的慢速旋转学习成零偏。
 * 不需要在线学习时可把 ALPHA 改成 0.0f。
 */
#define ATTITUDE_STATIONARY_GYRO_DPS        (0.20f)
#define ATTITUDE_STATIONARY_ACCEL_MIN_G     (0.95f)
#define ATTITUDE_STATIONARY_ACCEL_MAX_G     (1.05f)
#define ATTITUDE_GYRO_BIAS_LEARN_ALPHA      (0.0002f)

/* 避免欧拉角公式在 pitch 接近 ±90° 时出现奇异。 */
#define ATTITUDE_PITCH_LIMIT_DEG            (89.0f)

/* 阻塞式校准最多允许的尝试倍数。 */
#define ATTITUDE_CALIB_MAX_ATTEMPT_FACTOR   (3U)

/* 陀螺仪零偏，单位为原始 LSB。 */
static float g_gyroBiasX = 0.0f;
static float g_gyroBiasY = 0.0f;
static float g_gyroBiasZ = 0.0f;

/* 加速度计低通滤波值，单位为原始 LSB。 */
static float g_accFiltX = 0.0f;
static float g_accFiltY = 0.0f;
static float g_accFiltZ = ATTITUDE_ACCEL_1G_LSB;
static bool g_accFilterReady = false;

/* 非阻塞校准累计状态。 */
static bool g_gyroCalibrated = false;
static uint32_t g_gyroCalCount = 0U;
static int64_t g_gyroCalSumX = 0;
static int64_t g_gyroCalSumY = 0;
static int64_t g_gyroCalSumZ = 0;
static int64_t g_accCalSumX = 0;
static int64_t g_accCalSumY = 0;
static int64_t g_accCalSumZ = 0;

/* 当前欧拉角，单位 deg。 */
static float g_rollDeg = 0.0f;
static float g_pitchDeg = 0.0f;
static float g_yawDeg = 0.0f;

/*
 * 安装零点：
 * 校准时把传感器当前静止姿态定义为 roll=0、pitch=0。
 */
static float g_rollZeroDeg = 0.0f;
static float g_pitchZeroDeg = 0.0f;

/*
 * 当前最终有效的 Z轴角速度，单位 deg/s。
 * angle_control.c 通过 getter 读取。
 */
static float g_gyroZDps = 0.0f;

static float attitude_clamp(
    float value,
    float minValue,
    float maxValue);

static float attitude_apply_deadband(
    float value,
    float deadband);

static float attitude_wrap_180(float angle);

static bool attitude_raw_is_valid(
    const icm42688_raw_t *raw);

static void attitude_accel_to_euler(
    float ax,
    float ay,
    float az,
    float *roll,
    float *pitch);

static void attitude_set_zero_from_accel(
    float ax,
    float ay,
    float az);

static void attitude_reset_calibration_accumulator(void);

void attitude_init(void)
{
    g_gyroBiasX = 0.0f;
    g_gyroBiasY = 0.0f;
    g_gyroBiasZ = 0.0f;

    g_accFiltX = 0.0f;
    g_accFiltY = 0.0f;
    g_accFiltZ = ATTITUDE_ACCEL_1G_LSB;
    g_accFilterReady = false;

    g_gyroCalibrated = false;
    attitude_reset_calibration_accumulator();

    g_rollDeg = 0.0f;
    g_pitchDeg = 0.0f;
    g_yawDeg = 0.0f;

    g_rollZeroDeg = 0.0f;
    g_pitchZeroDeg = 0.0f;

    g_gyroZDps = 0.0f;
}

void attitude_calibrate_gyro(uint16_t sampleCount)
{
    icm42688_raw_t raw = {0};

    int64_t sumGx = 0;
    int64_t sumGy = 0;
    int64_t sumGz = 0;

    int64_t sumAx = 0;
    int64_t sumAy = 0;
    int64_t sumAz = 0;

    uint32_t validCount = 0U;
    uint32_t attemptCount = 0U;
    uint32_t maxAttempts;

    g_gyroCalibrated = false;
    g_gyroZDps = 0.0f;

    if (sampleCount == 0U) {
        return;
    }

    maxAttempts =
        (uint32_t)sampleCount *
        ATTITUDE_CALIB_MAX_ATTEMPT_FACTOR;

    /*
     * 只累计 WHO_AM_I 正确的有效帧。
     * I2C 偶发失败的数据不会污染零偏。
     */
    while ((validCount < (uint32_t)sampleCount) &&
           (attemptCount < maxAttempts)) {

        attemptCount++;
        icm42688_read_raw(&raw);

        if (!attitude_raw_is_valid(&raw)) {
            delay_ms(2U);
            continue;
        }

        sumGx += raw.gyroX;
        sumGy += raw.gyroY;
        sumGz += raw.gyroZ;

        sumAx += raw.accelX;
        sumAy += raw.accelY;
        sumAz += raw.accelZ;

        validCount++;
        delay_ms(2U);
    }

    /*
     * 未获得足够有效样本时保持“未校准”状态。
     */
    if (validCount < (uint32_t)sampleCount) {
        return;
    }

    g_gyroBiasX = (float)sumGx / (float)validCount;
    g_gyroBiasY = (float)sumGy / (float)validCount;
    g_gyroBiasZ = (float)sumGz / (float)validCount;

    g_accFiltX = (float)sumAx / (float)validCount;
    g_accFiltY = (float)sumAy / (float)validCount;
    g_accFiltZ = (float)sumAz / (float)validCount;
    g_accFilterReady = true;

    attitude_set_zero_from_accel(
        g_accFiltX,
        g_accFiltY,
        g_accFiltZ);

    g_gyroCalibrated = true;
}

bool attitude_calibrate_gyro_step(
    const icm42688_raw_t *raw,
    uint16_t sampleCount)
{
    if (g_gyroCalibrated) {
        return true;
    }

    if ((sampleCount == 0U) ||
        (!attitude_raw_is_valid(raw))) {
        return false;
    }

    g_gyroCalSumX += raw->gyroX;
    g_gyroCalSumY += raw->gyroY;
    g_gyroCalSumZ += raw->gyroZ;

    g_accCalSumX += raw->accelX;
    g_accCalSumY += raw->accelY;
    g_accCalSumZ += raw->accelZ;

    g_gyroCalCount++;

    if (g_gyroCalCount < (uint32_t)sampleCount) {
        return false;
    }

    g_gyroBiasX =
        (float)g_gyroCalSumX / (float)g_gyroCalCount;
    g_gyroBiasY =
        (float)g_gyroCalSumY / (float)g_gyroCalCount;
    g_gyroBiasZ =
        (float)g_gyroCalSumZ / (float)g_gyroCalCount;

    g_accFiltX =
        (float)g_accCalSumX / (float)g_gyroCalCount;
    g_accFiltY =
        (float)g_accCalSumY / (float)g_gyroCalCount;
    g_accFiltZ =
        (float)g_accCalSumZ / (float)g_gyroCalCount;
    g_accFilterReady = true;

    attitude_set_zero_from_accel(
        g_accFiltX,
        g_accFiltY,
        g_accFiltZ);

    g_gyroZDps = 0.0f;
    g_gyroCalibrated = true;

    return true;
}

bool attitude_is_gyro_calibrated(void)
{
    return g_gyroCalibrated;
}

bool attitude_update_from_icm42688(
    const icm42688_raw_t *raw,
    float dt)
{
    float gx;
    float gy;
    float gz;

    float ax;
    float ay;
    float az;
    float rawAccMagG;
    float accMagG;

    float rollRad;
    float pitchRad;
    float cosPitch;

    float rollRateDps;
    float pitchRateDps;
    float yawRateDps;

    float rollAccDeg;
    float pitchAccDeg;
    float rollAccRelDeg;
    float pitchAccRelDeg;

    if ((!attitude_raw_is_valid(raw)) ||
        (!g_gyroCalibrated) ||
        (dt <= 0.0f)) {
        g_gyroZDps = 0.0f;
        return false;
    }

    dt = attitude_clamp(
        dt,
        ATTITUDE_DT_MIN_S,
        ATTITUDE_DT_MAX_S);

    /*
     * 零偏扣除并换算为 deg/s。
     */
    gx =
        ((float)raw->gyroX - g_gyroBiasX) *
        ATTITUDE_GYRO_DPS_PER_LSB;

    gy =
        ((float)raw->gyroY - g_gyroBiasY) *
        ATTITUDE_GYRO_DPS_PER_LSB;

    gz =
        ((float)raw->gyroZ - g_gyroBiasZ) *
        ATTITUDE_GYRO_DPS_PER_LSB *
        ATTITUDE_YAW_SCALE;

    /*
     * 在线零偏学习必须同时满足：
     *   1. 三轴角速度都非常小；
     *   2. 加速度模长非常接近 1g。
     *
     * 这样比只判断陀螺仪更不容易把真实运动学成零偏。
     */
    rawAccMagG =
        sqrtf(
            (float)raw->accelX * (float)raw->accelX +
            (float)raw->accelY * (float)raw->accelY +
            (float)raw->accelZ * (float)raw->accelZ) /
        ATTITUDE_ACCEL_1G_LSB;

    if ((ATTITUDE_GYRO_BIAS_LEARN_ALPHA > 0.0f) &&
        (fabsf(gx) < ATTITUDE_STATIONARY_GYRO_DPS) &&
        (fabsf(gy) < ATTITUDE_STATIONARY_GYRO_DPS) &&
        (fabsf(gz) < ATTITUDE_STATIONARY_GYRO_DPS) &&
        (rawAccMagG >= ATTITUDE_STATIONARY_ACCEL_MIN_G) &&
        (rawAccMagG <= ATTITUDE_STATIONARY_ACCEL_MAX_G)) {

        g_gyroBiasX +=
            ATTITUDE_GYRO_BIAS_LEARN_ALPHA *
            ((float)raw->gyroX - g_gyroBiasX);

        g_gyroBiasY +=
            ATTITUDE_GYRO_BIAS_LEARN_ALPHA *
            ((float)raw->gyroY - g_gyroBiasY);

        g_gyroBiasZ +=
            ATTITUDE_GYRO_BIAS_LEARN_ALPHA *
            ((float)raw->gyroZ - g_gyroBiasZ);

        gx =
            ((float)raw->gyroX - g_gyroBiasX) *
            ATTITUDE_GYRO_DPS_PER_LSB;

        gy =
            ((float)raw->gyroY - g_gyroBiasY) *
            ATTITUDE_GYRO_DPS_PER_LSB;

        gz =
            ((float)raw->gyroZ - g_gyroBiasZ) *
            ATTITUDE_GYRO_DPS_PER_LSB *
            ATTITUDE_YAW_SCALE;
    }

    gx = attitude_apply_deadband(
        gx,
        ATTITUDE_GYRO_DEADBAND_DPS);

    gy = attitude_apply_deadband(
        gy,
        ATTITUDE_GYRO_DEADBAND_DPS);

    gz = attitude_apply_deadband(
        gz,
        ATTITUDE_YAW_DEADBAND_DPS);

    /*
     * 保存最终有效 gyroZ。
     * 角度环的 D 项读取的就是这个值。
     */
    g_gyroZDps = gz;

    /*
     * 机体系角速度转换为欧拉角速度。
     */
    g_pitchDeg = attitude_clamp(
        g_pitchDeg,
        -ATTITUDE_PITCH_LIMIT_DEG,
        ATTITUDE_PITCH_LIMIT_DEG);

    rollRad = g_rollDeg * ATTITUDE_DEG_TO_RAD;
    pitchRad = g_pitchDeg * ATTITUDE_DEG_TO_RAD;
    cosPitch = cosf(pitchRad);

    /*
     * pitch 已限制到 ±89°，这里仍保留最小值保护。
     */
    if (fabsf(cosPitch) < 0.01f) {
        cosPitch = (cosPitch >= 0.0f) ? 0.01f : -0.01f;
    }

    rollRateDps =
        gx +
        sinf(rollRad) * tanf(pitchRad) * gy +
        cosf(rollRad) * tanf(pitchRad) * gz;

    pitchRateDps =
        cosf(rollRad) * gy -
        sinf(rollRad) * gz;

    yawRateDps =
        sinf(rollRad) / cosPitch * gy +
        cosf(rollRad) / cosPitch * gz;

    g_rollDeg += rollRateDps * dt;
    g_pitchDeg += pitchRateDps * dt;
    g_yawDeg += yawRateDps * dt;

    /*
     * 加速度计低通滤波。
     */
    if (!g_accFilterReady) {
        g_accFiltX = (float)raw->accelX;
        g_accFiltY = (float)raw->accelY;
        g_accFiltZ = (float)raw->accelZ;
        g_accFilterReady = true;
    } else {
        g_accFiltX +=
            ATTITUDE_ACCEL_LPF_ALPHA *
            ((float)raw->accelX - g_accFiltX);

        g_accFiltY +=
            ATTITUDE_ACCEL_LPF_ALPHA *
            ((float)raw->accelY - g_accFiltY);

        g_accFiltZ +=
            ATTITUDE_ACCEL_LPF_ALPHA *
            ((float)raw->accelZ - g_accFiltZ);
    }

    ax = g_accFiltX;
    ay = g_accFiltY;
    az = g_accFiltZ;

    if (!((ax == 0.0f) &&
          (ay == 0.0f) &&
          (az == 0.0f))) {

        accMagG =
            sqrtf(ax * ax + ay * ay + az * az) /
            ATTITUDE_ACCEL_1G_LSB;

        /*
         * 强加速、碰撞或振动时不使用加速度计纠正姿态。
         */
        if ((accMagG >= ATTITUDE_ACCEL_MIN_G) &&
            (accMagG <= ATTITUDE_ACCEL_MAX_G)) {

            attitude_accel_to_euler(
                ax,
                ay,
                az,
                &rollAccDeg,
                &pitchAccDeg);

            rollAccRelDeg =
                attitude_wrap_180(
                    rollAccDeg - g_rollZeroDeg);

            pitchAccRelDeg =
                attitude_wrap_180(
                    pitchAccDeg - g_pitchZeroDeg);

            /*
             * 使用“角度差”进行互补修正，
             * 避免 roll 接近 ±180° 时直接加权造成跳变。
             */
            g_rollDeg +=
                (1.0f - ATTITUDE_COMPLEMENTARY_ALPHA) *
                attitude_wrap_180(
                    rollAccRelDeg - g_rollDeg);

            g_pitchDeg +=
                (1.0f - ATTITUDE_COMPLEMENTARY_ALPHA) *
                (pitchAccRelDeg - g_pitchDeg);
        }
    }

    g_rollDeg = attitude_wrap_180(g_rollDeg);

    g_pitchDeg = attitude_clamp(
        g_pitchDeg,
        -ATTITUDE_PITCH_LIMIT_DEG,
        ATTITUDE_PITCH_LIMIT_DEG);

    g_yawDeg = attitude_wrap_180(g_yawDeg);

    return true;
}

void attitude_get_euler(attitude_euler_t *euler)
{
    if (euler == 0) {
        return;
    }

    euler->roll = g_rollDeg;
    euler->pitch = g_pitchDeg;
    euler->yaw = g_yawDeg;
}

float attitude_get_gyro_z_dps(void)
{
    return g_gyroZDps;
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

static float attitude_clamp(
    float value,
    float minValue,
    float maxValue)
{
    if (value < minValue) {
        return minValue;
    }

    if (value > maxValue) {
        return maxValue;
    }

    return value;
}

static float attitude_apply_deadband(
    float value,
    float deadband)
{
    if ((value > -deadband) &&
        (value < deadband)) {
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

static bool attitude_raw_is_valid(
    const icm42688_raw_t *raw)
{
    if (raw == 0) {
        return false;
    }

    return raw->whoAmI ==
           ATTITUDE_EXPECTED_WHO_AM_I;
}

static void attitude_accel_to_euler(
    float ax,
    float ay,
    float az,
    float *roll,
    float *pitch)
{
    if (roll != 0) {
        *roll =
            atan2f(ay, az) *
            ATTITUDE_RAD_TO_DEG;
    }

    if (pitch != 0) {
        *pitch =
            atan2f(
                -ax,
                sqrtf(ay * ay + az * az)) *
            ATTITUDE_RAD_TO_DEG;
    }
}

static void attitude_set_zero_from_accel(
    float ax,
    float ay,
    float az)
{
    attitude_accel_to_euler(
        ax,
        ay,
        az,
        &g_rollZeroDeg,
        &g_pitchZeroDeg);

    g_rollDeg = 0.0f;
    g_pitchDeg = 0.0f;
    g_yawDeg = 0.0f;
    g_gyroZDps = 0.0f;
}

static void attitude_reset_calibration_accumulator(void)
{
    g_gyroCalCount = 0U;

    g_gyroCalSumX = 0;
    g_gyroCalSumY = 0;
    g_gyroCalSumZ = 0;

    g_accCalSumX = 0;
    g_accCalSumY = 0;
    g_accCalSumZ = 0;
}
