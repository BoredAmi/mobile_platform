/* rover_link framing: CRC-16/CCITT-FALSE + COBS. See rover_link.h. */
#include "rover_link.h"

#include <string.h>

uint16_t rl_crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

/* COBS encode in -> out, returns encoded length (without the trailing 0x00). */
static size_t cobs_encode(const uint8_t *in, size_t len, uint8_t *out) {
  size_t code_pos = 0;
  size_t o = 1;
  uint8_t code = 1;
  for (size_t i = 0; i < len; ++i) {
    if (in[i] == 0) {
      out[code_pos] = code;
      code_pos = o++;
      code = 1;
    } else {
      out[o++] = in[i];
      if (++code == 0xFF) {
        out[code_pos] = code;
        code_pos = o++;
        code = 1;
      }
    }
  }
  out[code_pos] = code;
  return o;
}

/* COBS decode in -> out, returns decoded length or 0 on a malformed block. */
static size_t cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t out_max) {
  size_t i = 0;
  size_t o = 0;
  while (i < len) {
    uint8_t code = in[i++];
    if (code == 0 || i + code - 1 > len) {
      return 0;
    }
    for (uint8_t k = 1; k < code; ++k) {
      if (o >= out_max) {
        return 0;
      }
      out[o++] = in[i++];
    }
    if (code != 0xFF && i < len) {
      if (o >= out_max) {
        return 0;
      }
      out[o++] = 0;
    }
  }
  return o;
}

size_t rl_encode(uint8_t type, const void *payload, size_t len, uint8_t *out) {
  if (len > RL_MAX_PAYLOAD) {
    return 0;
  }
  uint8_t raw[RL_MAX_RAW];
  raw[0] = type;
  if (len) {
    memcpy(raw + 1, payload, len);
  }
  uint16_t crc = rl_crc16(raw, len + 1);
  raw[len + 1] = (uint8_t)(crc & 0xFF);
  raw[len + 2] = (uint8_t)(crc >> 8);
  size_t n = cobs_encode(raw, len + 3, out);
  out[n++] = 0;
  return n;
}

void rl_decoder_init(rl_decoder_t *d) {
  memset(d, 0, sizeof(*d));
}

bool rl_decoder_feed(rl_decoder_t *d, uint8_t byte, uint8_t *type, const uint8_t **payload,
                     size_t *len) {
  if (byte != 0) {
    if (d->len < sizeof(d->buf)) {
      d->buf[d->len++] = byte;
    } else {
      d->overflow = true;
    }
    return false;
  }
  /* end of frame */
  size_t enc_len = d->len;
  bool overflow = d->overflow;
  d->len = 0;
  d->overflow = false;
  if (enc_len == 0) {
    return false;  /* empty frame: idle delimiter, not an error */
  }
  size_t n = overflow ? 0 : cobs_decode(d->buf, enc_len, d->frame, sizeof(d->frame));
  if (n < 3) {
    d->errors++;
    return false;
  }
  uint16_t crc = (uint16_t)(d->frame[n - 2] | (d->frame[n - 1] << 8));
  if (crc != rl_crc16(d->frame, n - 2)) {
    d->errors++;
    return false;
  }
  *type = d->frame[0];
  *payload = d->frame + 1;
  *len = n - 3;
  return true;
}
