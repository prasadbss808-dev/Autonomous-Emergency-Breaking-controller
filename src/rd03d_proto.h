#ifndef RD03D_PROTO_H
#define RD03D_PROTO_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#define RD03D_FRAME_LEN 30
#define RD03D_HDR_LEN 4
#define RD03D_TARGET_LEN 8
#define RD03D_NUM_SLOTS 3
#define RD03D_CMD_SINGLE 0x0080u
#define RD03D_CMD_MULTI 0x0090u
#define RD03D_CMD_FRAME_LEN 12
typedef struct {
    bool present;
    bool speed_sentinel;
    int16_t x_mm;
    int16_t y_mm;
    int16_t speed_cms;
    uint16_t res_mm;
} rd03d_slot_t;
typedef struct {
    rd03d_slot_t slot[RD03D_NUM_SLOTS];
} rd03d_frame_t;
typedef struct {
    uint8_t buf[RD03D_FRAME_LEN];
    uint8_t len;
    uint32_t frames_ok;
    uint32_t bad_footer;
    uint32_t bytes_discarded;
} rd03d_parser_t;
void rd03d_parser_init(rd03d_parser_t *ps);
bool rd03d_parser_feed(rd03d_parser_t *ps, uint8_t byte, rd03d_frame_t *out);
int16_t rd03d_decode_value(uint8_t lo, uint8_t hi);
void rd03d_encode_value(int16_t v, uint8_t *lo, uint8_t *hi);
void rd03d_decode_frame(const uint8_t *frame30, rd03d_frame_t *out);
void rd03d_encode_frame(const rd03d_frame_t *in, uint8_t *frame30);
size_t rd03d_build_cmd(uint16_t cmd, uint8_t *out, size_t out_sz);
#endif
