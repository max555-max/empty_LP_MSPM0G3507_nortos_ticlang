#include "attitude.h"

#include <math.h>

#include "vofa.h"

#define ATTITUDE_GYRO_DPS_PER_LSB      (2000.0f / 32768.0f)
#define ATTITUDE_DEG_TO_RAD            (0.017453292519943295f)
#define ATTITUDE_RAD_TO_DEG            (57.29577951308232f)
#define ATTITUDE_TWO_KP                (2.0f)
#define ATTITUDE_TWO_KI                (0.0f)

static float q0 = 1.0f;
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;
static float integralFbX = 0.0f;
static float integralFbY = 0.0f;
static float integralFbZ = 0.0f;

static float attitude_inv_sqrt(float x);
static float attitude_clamp(float x, float minValue, float maxValue);

void attitude_init(void)
{
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    integralFbX = 0.0f;
    integralFbY = 0.0f;
    integralFbZ = 0.0f;
}

bool attitude_update_from_icm42688(const icm42688_raw_t *raw, float dt)
{
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
    float recipNorm;
    float halfvx;
    float halfvy;
    float halfvz;
    float halfex;
    float halfey;
    float halfez;
    float qa;
    float qb;
    float qc;

    if ((raw == 0) || (dt <= 0.0f)) {
        return false;
    }

    gx = (float) raw->gyroX * ATTITUDE_GYRO_DPS_PER_LSB * ATTITUDE_DEG_TO_RAD;
    gy = (float) raw->gyroY * ATTITUDE_GYRO_DPS_PER_LSB * ATTITUDE_DEG_TO_RAD;
    gz = (float) raw->gyroZ * ATTITUDE_GYRO_DPS_PER_LSB * ATTITUDE_DEG_TO_RAD;

    ax = (float) raw->accelX;
    ay = (float) raw->accelY;
    az = (float) raw->accelZ;

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = attitude_inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        if (ATTITUDE_TWO_KI > 0.0f) {
            integralFbX += ATTITUDE_TWO_KI * halfex * dt;
            integralFbY += ATTITUDE_TWO_KI * halfey * dt;
            integralFbZ += ATTITUDE_TWO_KI * halfez * dt;
            gx += integralFbX;
            gy += integralFbY;
            gz += integralFbZ;
        } else {
            integralFbX = 0.0f;
            integralFbY = 0.0f;
            integralFbZ = 0.0f;
        }

        gx += ATTITUDE_TWO_KP * halfex;
        gy += ATTITUDE_TWO_KP * halfey;
        gz += ATTITUDE_TWO_KP * halfez;
    }

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    qa = q0;
    qb = q1;
    qc = q2;

    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    recipNorm = attitude_inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    return true;
}

void attitude_get_euler(attitude_euler_t *euler)
{
    float sinPitch;

    if (euler == 0) {
        return;
    }

    euler->roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                         1.0f - 2.0f * (q1 * q1 + q2 * q2)) *
                  ATTITUDE_RAD_TO_DEG;

    sinPitch = 2.0f * (q0 * q2 - q3 * q1);
    euler->pitch = asinf(attitude_clamp(sinPitch, -1.0f, 1.0f)) *
                   ATTITUDE_RAD_TO_DEG;

    euler->yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                        1.0f - 2.0f * (q2 * q2 + q3 * q3)) *
                 ATTITUDE_RAD_TO_DEG;
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
