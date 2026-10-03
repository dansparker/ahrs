#include "ubx.h"

#include <string.h>

enum { S_SYNC1, S_SYNC2, S_CLASS, S_ID, S_LEN1, S_LEN2, S_PAYLOAD, S_CKA, S_CKB };

static uint32_t u32(const uint8_t* b) { return (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 24; }
static int32_t i32(const uint8_t* b) { return (int32_t)u32(b); }
static uint16_t u16(const uint8_t* b) { return (uint16_t)(b[0] | b[1] << 8); }

void ubx_init(ubx_parser_t* p) { memset(p, 0, sizeof(*p)); }

static void decode_pvt(const uint8_t* b, ubx_pvt_t* o) {
    o->itow_ms = u32(b + 0);
    o->year = u16(b + 4);
    o->month = b[6];
    o->day = b[7];
    o->hour = b[8];
    o->min = b[9];
    o->sec = b[10];
    o->valid = b[11];
    o->nano = i32(b + 16);
    o->fix_type = b[20];
    o->flags = b[21];
    o->num_sv = b[23];
    o->lon_deg = i32(b + 24) * 1e-7;
    o->lat_deg = i32(b + 28) * 1e-7;
    o->height_m = (float)i32(b + 32) * 1e-3f;
    o->hmsl_m = (float)i32(b + 36) * 1e-3f;
    o->h_acc_m = (float)u32(b + 40) * 1e-3f;
    o->v_acc_m = (float)u32(b + 44) * 1e-3f;
    for (int i = 0; i < 3; ++i) o->vel_ned[i] = (float)i32(b + 48 + 4 * i) * 1e-3f;
    o->g_speed_ms = (float)i32(b + 60) * 1e-3f;
    o->head_mot_deg = (float)i32(b + 64) * 1e-5f;
    o->s_acc_ms = (float)u32(b + 68) * 1e-3f;
    o->head_acc_deg = (float)u32(b + 72) * 1e-5f;
    o->pdop = (float)u16(b + 76) * 0.01f;
}

int ubx_feed(ubx_parser_t* p, uint8_t c, ubx_pvt_t* out) {
    switch (p->state) {
    case S_SYNC1:
        if (c == 0xB5) p->state = S_SYNC2;
        return 0;
    case S_SYNC2:
        p->state = (c == 0x62) ? S_CLASS : (c == 0xB5 ? S_SYNC2 : S_SYNC1);
        return 0;
    case S_CLASS:
        p->cls = c;
        p->ck_a = c;
        p->ck_b = c;
        p->state = S_ID;
        return 0;
    default:
        break;
    }
    if (p->state <= S_PAYLOAD) {
        p->ck_a = (uint8_t)(p->ck_a + c);
        p->ck_b = (uint8_t)(p->ck_b + p->ck_a);
    }
    switch (p->state) {
    case S_ID:
        p->id = c;
        p->state = S_LEN1;
        break;
    case S_LEN1:
        p->len = c;
        p->state = S_LEN2;
        break;
    case S_LEN2:
        p->len |= (uint16_t)(c << 8);
        p->pos = 0;
        if (p->len > 512) { /* nothing we need is that long: resync */
            p->n_bad++;
            p->state = S_SYNC1;
        } else {
            p->state = p->len ? S_PAYLOAD : S_CKA;
        }
        break;
    case S_PAYLOAD:
        if (p->pos < sizeof(p->buf)) p->buf[p->pos] = c;
        if (++p->pos >= p->len) p->state = S_CKA;
        break;
    case S_CKA:
        p->state = (c == p->ck_a) ? S_CKB : S_SYNC1;
        if (c != p->ck_a) p->n_bad++;
        break;
    case S_CKB:
        p->state = S_SYNC1;
        if (c != p->ck_b) {
            p->n_bad++;
            return 0;
        }
        p->n_ok++;
        if (p->cls == 0x01 && p->id == 0x07 && p->len == 92) {
            decode_pvt(p->buf, out);
            return 1;
        }
        break;
    default:
        p->state = S_SYNC1;
        break;
    }
    return 0;
}

size_t ubx_frame(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len, uint8_t* out) {
    out[0] = 0xB5;
    out[1] = 0x62;
    out[2] = cls;
    out[3] = id;
    out[4] = (uint8_t)(len & 0xFF);
    out[5] = (uint8_t)(len >> 8);
    if (len) memcpy(out + 6, payload, len);
    uint8_t a = 0, b = 0;
    for (size_t i = 2; i < 6u + len; ++i) {
        a = (uint8_t)(a + out[i]);
        b = (uint8_t)(b + a);
    }
    out[6 + len] = a;
    out[7 + len] = b;
    return 8u + len;
}

static uint8_t* put32(uint8_t* p, uint32_t v) {
    for (int i = 0; i < 4; ++i) *p++ = (uint8_t)(v >> (8 * i));
    return p;
}
static uint8_t* put16(uint8_t* p, uint16_t v) {
    *p++ = (uint8_t)v;
    *p++ = (uint8_t)(v >> 8);
    return p;
}

size_t ubx_config_msg(unsigned n, uint32_t baud, unsigned rate_hz, uint8_t* out) {
    uint8_t pl[64];
    uint8_t* p = pl;
    memset(pl, 0, sizeof(pl));
    const uint16_t meas_ms = (uint16_t)(1000u / (rate_hz ? rate_hz : 1u));
    switch (n) {
    case 0: /* UBX-CFG-VALSET (M9/M10), RAM layer */
        *p++ = 0;
        *p++ = 0x01;
        p += 2;
        p = put32(p, 0x10740001u); *p++ = 1;          /* CFG-UART1OUTPROT-UBX */
        p = put32(p, 0x10740002u); *p++ = 0;          /* CFG-UART1OUTPROT-NMEA */
        p = put32(p, 0x20910007u); *p++ = 1;          /* CFG-MSGOUT-UBX_NAV_PVT_UART1 */
        p = put32(p, 0x20110021u); *p++ = 8;          /* CFG-NAVSPG-DYNMODEL: airborne <4g */
        return ubx_frame(0x06, 0x8A, pl, (uint16_t)(p - pl), out);
    case 4: /* VALSET: rate on its own - a receiver that cannot do it rejects only this message */
        *p++ = 0;
        *p++ = 0x01;
        p += 2;
        p = put32(p, 0x30210001u);
        p = put16(p, meas_ms); /* CFG-RATE-MEAS */
        return ubx_frame(0x06, 0x8A, pl, (uint16_t)(p - pl), out);
    case 1: /* UBX-CFG-MSG: NAV-PVT on UART1 every solution */
        pl[0] = 0x01;
        pl[1] = 0x07;
        pl[3] = 1; /* I2C, UART1, UART2, USB, SPI, reserved */
        return ubx_frame(0x06, 0x01, pl, 8, out);
    case 2: /* UBX-CFG-RATE */
        p = put16(p, meas_ms);
        p = put16(p, 1);
        p = put16(p, 0); /* UTC */
        return ubx_frame(0x06, 0x08, pl, 6, out);
    case 3: /* UBX-CFG-NAV5: dynamic model only */
        put16(pl, 0x0001);
        pl[2] = 8;
        return ubx_frame(0x06, 0x24, pl, 36, out);
    case 5: /* VALSET: UART1 baud rate */
        *p++ = 0;
        *p++ = 0x01;
        p += 2;
        p = put32(p, 0x40520001u);
        p = put32(p, baud);
        return ubx_frame(0x06, 0x8A, pl, (uint16_t)(p - pl), out);
    case 6: /* UBX-CFG-PRT (M8): UART1, 8N1, UBX in/out only */
        pl[0] = 1;
        put32(pl + 4, 0x000008C0u);
        put32(pl + 8, baud);
        put16(pl + 12, 0x0001);
        put16(pl + 14, 0x0001);
        return ubx_frame(0x06, 0x00, pl, 20, out);
    default:
        return 0;
    }
}
