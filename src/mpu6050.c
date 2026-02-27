#include "mpu6050.h"
#include <math.h>

/* MPU6050 I2C address (AD0 pin low). */
#define MPU6050_ADDR            (0x68 << 1)

/* Register addresses. */
#define REG_WHO_AM_I            0x75
#define REG_PWR_MGMT_1          0x6B
#define REG_CONFIG              0x1A
#define REG_GYRO_CONFIG         0x1B
#define REG_ACCEL_CONFIG        0x1C
#define REG_ACCEL_XOUT_H        0x3B

/* Expected WHO_AM_I value. */
#define WHO_AM_I_EXPECTED       0x68

/* Sensitivity scaling factors.
 *
 * Gyro +/-250 deg/s: 131 LSB per deg/s -> convert to rad/s.
 * Accel +/-2g: 16384 LSB per g -> convert to m/s^2.
 */
#define GYRO_SCALE              (1.0f / 131.0f * (3.14159265f / 180.0f))  /* LSB to rad/s */
#define ACCEL_SCALE             (9.80665f / 16384.0f)                     /* LSB to m/s^2 */

/* I2C timeout (ms). */
#define I2C_TIMEOUT             100

static bool write_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value)
{
  uint8_t buf[2] = {reg, value};
  return (HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, buf, 2, I2C_TIMEOUT) == HAL_OK);
}

static bool read_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *value)
{
  if (HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, &reg, 1, I2C_TIMEOUT) != HAL_OK)
  {
    return false;
  }
  return (HAL_I2C_Master_Receive(hi2c, MPU6050_ADDR, value, 1, I2C_TIMEOUT) == HAL_OK);
}

static bool read_burst(I2C_HandleTypeDef *hi2c, uint8_t start_reg, uint8_t *buf, uint16_t len)
{
  if (HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, &start_reg, 1, I2C_TIMEOUT) != HAL_OK)
  {
    return false;
  }
  return (HAL_I2C_Master_Receive(hi2c, MPU6050_ADDR, buf, len, I2C_TIMEOUT) == HAL_OK);
}

bool Mpu6050_Init(I2C_HandleTypeDef *hi2c)
{
  /* Verify device identity. */
  uint8_t who_am_i = 0;
  if (!read_reg(hi2c, REG_WHO_AM_I, &who_am_i))
  {
    return false;
  }
  if (who_am_i != WHO_AM_I_EXPECTED)
  {
    return false;
  }

  /* Wake up from sleep (clear SLEEP bit, use internal 8MHz oscillator). */
  if (!write_reg(hi2c, REG_PWR_MGMT_1, 0x00))
  {
    return false;
  }

  /* Small delay for oscillator startup. */
  HAL_Delay(10);

  /* DLPF config: bandwidth ~44Hz for both accel/gyro.
   * Reduces high-frequency motor vibration noise.
   */
  if (!write_reg(hi2c, REG_CONFIG, 0x03))
  {
    return false;
  }

  /* Gyro: +/-250 deg/s (most sensitive, best for slow rotation). */
  if (!write_reg(hi2c, REG_GYRO_CONFIG, 0x00))
  {
    return false;
  }

  /* Accel: +/-2g (most sensitive, best for tilt detection). */
  if (!write_reg(hi2c, REG_ACCEL_CONFIG, 0x00))
  {
    return false;
  }

  return true;
}

bool Mpu6050_ReadRaw(I2C_HandleTypeDef *hi2c, Mpu6050_RawData *out)
{
  /* Read 14 bytes: ACCEL (6) + TEMP (2) + GYRO (6), all in one burst.
   * We skip the temperature bytes (indices 6-7).
   */
  uint8_t buf[14];
  if (!read_burst(hi2c, REG_ACCEL_XOUT_H, buf, 14))
  {
    return false;
  }

  /* Parse big-endian 16-bit values. */
  out->accel_x = (int16_t)((buf[0] << 8) | buf[1]);
  out->accel_y = (int16_t)((buf[2] << 8) | buf[3]);
  out->accel_z = (int16_t)((buf[4] << 8) | buf[5]);
  /* buf[6..7] = temperature (ignored) */
  out->gyro_x = (int16_t)((buf[8] << 8) | buf[9]);
  out->gyro_y = (int16_t)((buf[10] << 8) | buf[11]);
  out->gyro_z = (int16_t)((buf[12] << 8) | buf[13]);

  return true;
}

void Mpu6050_ConvertToPhysical(const Mpu6050_RawData *raw, Mpu6050_Data *out)
{
  out->accel_x = (float)raw->accel_x * ACCEL_SCALE;
  out->accel_y = (float)raw->accel_y * ACCEL_SCALE;
  out->accel_z = (float)raw->accel_z * ACCEL_SCALE;
  out->gyro_x = (float)raw->gyro_x * GYRO_SCALE;
  out->gyro_y = (float)raw->gyro_y * GYRO_SCALE;
  out->gyro_z = (float)raw->gyro_z * GYRO_SCALE;
}
