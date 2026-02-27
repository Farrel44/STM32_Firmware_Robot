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
#define FEEDBACK_PACKET_SIZE   18
#define WATCHDOG_TIMEOUT_MS    1000

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
  int16_t gyro_z;
  int16_t accel_z;
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
