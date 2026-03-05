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

static void serialize_int32_be(uint8_t *out, int32_t val)
{
  out[0] = (uint8_t)((val >> 24) & 0xFF);
  out[1] = (uint8_t)((val >> 16) & 0xFF);
  out[2] = (uint8_t)((val >> 8) & 0xFF);
  out[3] = (uint8_t)(val & 0xFF);
}

static void serialize_int16_be(uint8_t *out, int16_t val)
{
  out[0] = (uint8_t)((val >> 8) & 0xFF);
  out[1] = (uint8_t)(val & 0xFF);
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
  serialize_int32_be(&out_buf[idx], fb->tick1);
  idx += 4;

  /* Tick2 */
  serialize_int32_be(&out_buf[idx], fb->tick2);
  idx += 4;

  /* Tick3 */
  serialize_int32_be(&out_buf[idx], fb->tick3);
  idx += 4;

  /* Gyro X/Y/Z (EMA-filtered, milli-rad/s) */
  serialize_int16_be(&out_buf[idx], fb->gyro_x);
  idx += 2;
  serialize_int16_be(&out_buf[idx], fb->gyro_y);
  idx += 2;
  serialize_int16_be(&out_buf[idx], fb->gyro_z);
  idx += 2;

  /* Accel X/Y/Z (raw, milli-m/s²) */
  serialize_int16_be(&out_buf[idx], fb->accel_x);
  idx += 2;
  serialize_int16_be(&out_buf[idx], fb->accel_y);
  idx += 2;
  serialize_int16_be(&out_buf[idx], fb->accel_z);
  idx += 2;

  /* XOR checksum over bytes 0..N-2 */
  out_buf[idx++] = checksum_xor(out_buf, FEEDBACK_PACKET_SIZE - 1);
}
