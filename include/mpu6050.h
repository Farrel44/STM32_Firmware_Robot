#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * MPU6050 driver for STM32 HAL I2C.
 *
 * Configuration:
 * - Gyro: +/-250 deg/s (sensitivity 131 LSB/deg/s)
 * - Accel: +/-2g (sensitivity 16384 LSB/g)
 * - Sample rate: ~100Hz (internal DLPF enabled)
 */

/* Raw sensor data (signed 16-bit, straight from registers). */
typedef struct
{
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
} Mpu6050_RawData;

/* Converted sensor data in physical units. */
typedef struct
{
  float accel_x;  /* m/s^2 */
  float accel_y;  /* m/s^2 */
  float accel_z;  /* m/s^2 */
  float gyro_x;   /* rad/s */
  float gyro_y;   /* rad/s */
  float gyro_z;   /* rad/s */
} Mpu6050_Data;

/*
 * Initialize MPU6050.
 * Returns true if WHO_AM_I check passes and config succeeds.
 */
bool Mpu6050_Init(I2C_HandleTypeDef *hi2c);

/*
 * Read all 6 axes (accel + gyro) in one burst.
 * Returns true on successful I2C transaction.
 */
bool Mpu6050_ReadRaw(I2C_HandleTypeDef *hi2c, Mpu6050_RawData *out);

/*
 * Convert raw data to physical units.
 */
void Mpu6050_ConvertToPhysical(const Mpu6050_RawData *raw, Mpu6050_Data *out);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
