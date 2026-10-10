#include "rd03d_proto.h"
#include <string.h>
static const uint8_t k_hdr[RD03D_HDR_LEN] = {0xAA, 0xFF, 0x03, 0x00};
static const uint8_t k_ftr[2] = {0x55, 0xCC};
static const uint8_t k_cmd_hdr[4] = {0xFD, 0xFC, 0xFB, 0xFA};
static const uint8_t k_cmd_ftr[4] = {0x04, 0x03, 0x02, 0x01};
void rd03d_parser_init(rd03d_parser_t *ps)
{
    memset(ps, 0, sizeof(*ps));
}
int16_t rd03d_decode_value(uint8_t lo, uint8_t hi)
{
    int16_t mag = (int16_t)(((uint16_t)(hi & 0x7Fu) << 8) | lo);
    return (hi & 0x80u) ? mag : (int16_t)(-mag);
}
void rd03d_encode_value(int16_t v, uint8_t *lo, uint8_t *hi)
{
    uint16_t mag = (uint16_t)(v < 0 ? -v : v) & 0x7FFFu;
    *lo = (uint8_t)(mag & 0xFFu);
    *hi = (uint8_t)((mag >> 8) & 0x7Fu);
    if (v >= 0) *hi |= 0x80u;
}
void rd03d_decode_frame(const uint8_t *f, rd03d_frame_t *out)
{
    for (int i = 0; i < RD03D_NUM_SLOTS; i++) {
        const uint8_t *t = f + RD03D_HDR_LEN + i * RD03D_TARGET_LEN;
        rd03d_slot_t *s = &out->slot[i];
        s->x_mm = rd03d_decode_value(t[0], t[1]);
        s->y_mm = rd03d_decode_value(t[2], t[3]);
        s->speed_cms = rd03d_decode_value(t[4], t[5]);
        s->res_mm = (uint16_t)(((uint16_t)t[7] << 8) | t[6]);
        s->present = (s->x_mm != 0 || s->y_mm != 0);
        int16_t a = (int16_t)(s->speed_cms < 0 ? -s->speed_cms : s->speed_cms);
        s->speed_sentinel = (a == 248 || a == 256);
    }
}
void rd03d_encode_frame(const rd03d_frame_t *in, uint8_t *f)
{
    memset(f, 0, RD03D_FRAME_LEN);
    memcpy(f, k_hdr, RD03D_HDR_LEN);
    for (int i = 0; i < RD03D_NUM_SLOTS; i++) {
        uint8_t *t = f + RD03D_HDR_LEN + i * RD03D_TARGET_LEN;
        const rd03d_slot_t *s = &in->slot[i];
        if (!s->present) continue;
        rd03d_encode_value(s->x_mm, &t[0], &t[1]);
        rd03d_encode_value(s->y_mm, &t[2], &t[3]);
        rd03d_encode_value(s->speed_cms, &t[4], &t[5]);
        t[6] = (uint8_t)(s->res_mm & 0xFF);
        t[7] = (uint8_t)(s->res_mm >> 8);
    }
    f[28] = k_ftr[0];
    f[29] = k_ftr[1];
}
static void resync(rd03d_parser_t *ps)
{
    for (uint8_t start = 1; start < ps->len; start++) {
        uint8_t n = (uint8_t)(ps->len - start);
        uint8_t cmp = n < RD03D_HDR_LEN ? n : RD03D_HDR_LEN;
        if (memcmp(&ps->buf[start], k_hdr, cmp) == 0) {
            memmove(ps->buf, &ps->buf[start], n);
            ps->bytes_discarded += start;
            ps->len = n;
            return;
        }
    }
    ps->bytes_discarded += ps->len;
    ps->len = 0;
}
bool rd03d_parser_feed(rd03d_parser_t *ps, uint8_t b, rd03d_frame_t *out)
{
    if (ps->len < RD03D_HDR_LEN) {
        if (b == k_hdr[ps->len]) {
            ps->buf[ps->len++] = b;
        } else if (b == k_hdr[0]) {
            ps->bytes_discarded += ps->len;
            ps->buf[0] = b;
            ps->len = 1;
        } else {
            ps->bytes_discarded += (uint32_t)ps->len + 1u;
            ps->len = 0;
        }
        return false;
    }
    ps->buf[ps->len++] = b;
    if (ps->len < RD03D_FRAME_LEN) return false;
    if (ps->buf[28] == k_ftr[0] && ps->buf[29] == k_ftr[1]) {
        rd03d_decode_frame(ps->buf, out);
        ps->frames_ok++;
        ps->len = 0;
        return true;
    }
    ps->bad_footer++;
    resync(ps);
    return false;
}
size_t rd03d_build_cmd(uint16_t cmd, uint8_t *out, size_t out_sz)
{
    if (out_sz < RD03D_CMD_FRAME_LEN) return 0;
    size_t n = 0;
    memcpy(out + n, k_cmd_hdr, 4);
    n += 4;
    out[n++] = 0x02;
    out[n++] = 0x00;
    out[n++] = (uint8_t)(cmd & 0xFF);
    out[n++] = (uint8_t)(cmd >> 8);
    memcpy(out + n, k_cmd_ftr, 4);
    n += 4;
    return n;
}
