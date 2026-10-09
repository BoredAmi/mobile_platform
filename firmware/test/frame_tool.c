/* Host helper for test_firmware.py: runs the C rover_link framing on hex input.
 *   frame_tool enc <type> <payload-hex>   -> encoded frame as hex
 *   frame_tool dec <stream-hex>           -> one "type payload-hex" line per valid frame, then
 *                                            "errors N"
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rover_link.h"

static size_t from_hex(const char *hex, uint8_t *out, size_t max) {
  size_t n = strlen(hex) / 2;
  if (n > max) n = max;
  for (size_t i = 0; i < n; ++i) {
    unsigned v;
    sscanf(hex + 2 * i, "%2x", &v);
    out[i] = (uint8_t)v;
  }
  return n;
}

static void print_hex(const uint8_t *p, size_t n) {
  for (size_t i = 0; i < n; ++i) printf("%02x", p[i]);
}

int main(int argc, char **argv) {
  if (argc == 4 && strcmp(argv[1], "enc") == 0) {
    uint8_t payload[1024];
    size_t n = from_hex(argv[3], payload, sizeof(payload));
    uint8_t out[RL_MAX_ENCODED];
    size_t len = rl_encode((uint8_t)strtol(argv[2], NULL, 0), payload, n, out);
    print_hex(out, len);
    printf("\n");
    return 0;
  }
  if (argc == 3 && strcmp(argv[1], "dec") == 0) {
    static uint8_t stream[65536];
    size_t n = from_hex(argv[2], stream, sizeof(stream));
    rl_decoder_t d;
    rl_decoder_init(&d);
    for (size_t i = 0; i < n; ++i) {
      uint8_t type;
      const uint8_t *payload;
      size_t len;
      if (rl_decoder_feed(&d, stream[i], &type, &payload, &len)) {
        printf("%02x ", type);
        print_hex(payload, len);
        printf("\n");
      }
    }
    printf("errors %u\n", (unsigned)d.errors);
    return 0;
  }
  fprintf(stderr, "usage: frame_tool enc <type> <hex> | dec <hex>\n");
  return 2;
}
