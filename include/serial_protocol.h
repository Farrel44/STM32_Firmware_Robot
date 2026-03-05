#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PACKET_HEADER          0xA5
#define CMD_PACKET_SIZE        8
#define WATCHDOG_TIMEOUT_MS    250   /* ~12 missed 50 Hz packets */

/*
 * Feedback packet layout (STM32 → Pi): 26 bytes
 *
 * Byte  Field            Type        Units (on wire)
 * ----  -----            ----        ---------------
 *   0   Header           uint8       0xA5
 *  1-4  Tick1            int32 BE    delta encoder ticks
 *  5-8  Tick2            int32 BE    delta encoder ticks
 * 9-12  Tick3            int32 BE    delta encoder ticks
 * 13-14 GyroX            int16 BE    milli-rad/s  (EMA-filtered)
 * 15-16 GyroY            int16 BE    milli-rad/s  (EMA-filtered)
 * 17-18 GyroZ            int16 BE    milli-rad/s  (EMA-filtered)
 * 19-20 AccelX           int16 BE    milli-m/s²   (raw, hardware DLPF only)
 * 21-22 AccelY           int16 BE    milli-m/s²   (raw, hardware DLPF only)
 * 23-24 AccelZ           int16 BE    milli-m/s²   (raw, hardware DLPF only)
 *   25  XOR checksum     uint8       XOR of bytes 0..24
 *
 * Previous 18-byte format carried only GyroZ + AngleX (filtered pitch).
 * This expanded format provides all 6 axes so the ROS2 side can publish
 * a fully-populated sensor_msgs/Imu and enable EKF sensor fusion.
 */
#define FEEDBACK_PACKET_SIZE   26

typedef struct
{
  int16_t rpm1;
  int16_t rpm2;
  int16_t rpm3;
  bool valid;
} CommandPacket;

typedef struct
{
  int32_t tick1; /* delta ticks */
  int32_t tick2;
  int32_t tick3;
  int16_t gyro_x;  /* milli-rad/s (EMA-filtered) */
  int16_t gyro_y;  /* milli-rad/s (EMA-filtered) */
  int16_t gyro_z;  /* milli-rad/s (EMA-filtered) */
  int16_t accel_x;  /* milli-m/s² (raw) */
  int16_t accel_y;  /* milli-m/s² (raw) */
  int16_t accel_z;  /* milli-m/s² (raw) */
} FeedbackPacket;

typedef struct
{
  uint8_t buf[CMD_PACKET_SIZE];
  uint8_t idx;
} SerialProtoRx;

void SerialProtoRx_Init(SerialProtoRx *rx);

/*
 * Feed bytes into the command parser.
 * Returns true if at least one valid command packet was parsed.
 * If multiple packets are found, the last valid one is returned in out_cmd.
 */
bool SerialProtoRx_ParseBytes(SerialProtoRx *rx,
                             const uint8_t *data,
                             size_t len,
                             CommandPacket *out_cmd);

/* Build feedback packet (big-endian fields + XOR checksum). out_buf must be >= FEEDBACK_PACKET_SIZE. */
void SerialProto_BuildFeedback(const FeedbackPacket *fb, uint8_t out_buf[FEEDBACK_PACKET_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_PROTOCOL_H */
