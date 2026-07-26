#ifndef __MPU6050_H_
#define __MPU6050_H_

#include <stdbool.h>
#include <stdint.h>

#define MPU6050_WHO_AM_I_VALUE          (0x68U)
#define MPU6050_ADDRESS_INVALID         (0xFFU)

/*
 * Current register configuration follows the vendor example:
 *   clock source : PLL with Y gyro reference
 *   gyro range   : +/-2000 dps
 *   accel range  : +/-2 g
 *   sleep        : disabled
 *   AUX I2C      : master and bypass disabled
 */
#define MPU6050_ACCEL_G_PER_LSB         (1.0f / 16384.0f)
#define MPU6050_GYRO_DPS_PER_LSB        (2000.0f / 32768.0f)

typedef struct {
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int16_t temp;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
    uint8_t whoAmI;
} mpu6050_raw_t;

typedef struct {
    uint8_t currentAddress;
    uint8_t lastAddress;
    uint8_t lastWhoAmI;
    uint8_t lastError;
    uint32_t lastStatus;
    bool lastWhoReadOk;
    bool initialized;
} mpu6050_diag_t;

bool mpu6050_init(void);
bool mpu6050_read_raw(mpu6050_raw_t *raw);
uint8_t mpu6050_get_who_am_i(void);
void mpu6050_get_diag(mpu6050_diag_t *diag);
void mpu6050_recover_i2c_bus(void);

#endif
