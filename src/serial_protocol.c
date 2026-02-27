#include "serial_protocol.h"

static uint8_t checksum_xor(const uint8_t *data, size_t len)
{
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++)
  {
    checksum ^= data[i];
  }
  return checksum;
}

static int16_t parse_int16_be(const uint8_t *data)
{
  return (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

void SerialProtoRx_Init(SerialProtoRx *rx)
{
  rx->idx = 0;
}

bool SerialProtoRx_ParseBytes(SerialProtoRx *rx,
                             const uint8_t *data,
                             size_t len,
                             CommandPacket *out_cmd)
{
  bool got_valid = false;

  if (out_cmd != NULL)
  {
    out_cmd->valid = false;
  }

  for (size_t n = 0; n < len; n++)
  {
    const uint8_t byte = data[n];

    if (rx->idx == 0)
    {
      if (byte == PACKET_HEADER)
      {
        rx->buf[rx->idx++] = byte;
      }
      continue;
    }

    rx->buf[rx->idx++] = byte;

    if (rx->idx >= CMD_PACKET_SIZE)
    {
      const uint8_t received = rx->buf[CMD_PACKET_SIZE - 1];
      const uint8_t calculated = checksum_xor(rx->buf, CMD_PACKET_SIZE - 1);

      if ((received == calculated) && (out_cmd != NULL))
      {
        out_cmd->rpm1 = parse_int16_be(&rx->buf[1]);
        out_cmd->rpm2 = parse_int16_be(&rx->buf[3]);
        out_cmd->rpm3 = parse_int16_be(&rx->buf[5]);
        out_cmd->valid = true;
        got_valid = true;
      }

      rx->idx = 0;
    }
  }

  return got_valid;
}

void SerialProto_BuildFeedback(const FeedbackPacket *fb, uint8_t out_buf[FEEDBACK_PACKET_SIZE])
{
  size_t idx = 0;
  out_buf[idx++] = PACKET_HEADER;

  /* Tick1 */
  out_buf[idx++] = (uint8_t)((fb->tick1 >> 24) & 0xFF);
  out_buf[idx++] = (uint8_t)((fb->tick1 >> 16) & 0xFF);
  out_buf[idx++] = (uint8_t)((fb->tick1 >> 8) & 0xFF);
  out_buf[idx++] = (uint8_t)(fb->tick1 & 0xFF);

  /* Tick2 */
  out_buf[idx++] = (uint8_t)((fb->tick2 >> 24) & 0xFF);
  out_buf[idx++] = (uint8_t)((fb->tick2 >> 16) & 0xFF);
  out_buf[idx++] = (uint8_t)((fb->tick2 >> 8) & 0xFF);
  out_buf[idx++] = (uint8_t)(fb->tick2 & 0xFF);

  /* Tick3 */
  out_buf[idx++] = (uint8_t)((fb->tick3 >> 24) & 0xFF);
  out_buf[idx++] = (uint8_t)((fb->tick3 >> 16) & 0xFF);
  out_buf[idx++] = (uint8_t)((fb->tick3 >> 8) & 0xFF);
  out_buf[idx++] = (uint8_t)(fb->tick3 & 0xFF);

  /* Gyro Z */
  out_buf[idx++] = (uint8_t)((fb->gyro_z >> 8) & 0xFF);
  out_buf[idx++] = (uint8_t)(fb->gyro_z & 0xFF);

  /* Accel Z */
  out_buf[idx++] = (uint8_t)((fb->accel_z >> 8) & 0xFF);
  out_buf[idx++] = (uint8_t)(fb->accel_z & 0xFF);

  /* Checksum */
  out_buf[idx++] = checksum_xor(out_buf, FEEDBACK_PACKET_SIZE - 1);
}
