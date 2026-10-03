/*
 * ubx.h - u-blox UBX protocol: streaming parser for UBX-NAV-PVT and receiver configuration.
 */
#ifndef UBX_H
#define UBX_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t itow_ms;
    uint16_t year;
    uint8_t month, day, hour, min, sec;
    uint8_t valid;     /* bit0 validDate, bit1 validTime, bit2 fullyResolved */
    int32_t nano;      /* fraction of second, ns (may be negative) */
    uint8_t fix_type;  /* 0 none, 2 = 2D, 3 = 3D, 4 = GNSS+DR, 5 = time only */
    uint8_t flags;     /* bit0 gnssFixOK */
    uint8_t num_sv;
    double lat_deg, lon_deg;
    float height_m;    /* above ellipsoid */
    float hmsl_m;
    float h_acc_m, v_acc_m;
    float vel_ned[3];  /* m/s */
    float g_speed_ms;
    float head_mot_deg;
    float s_acc_ms;
    float head_acc_deg;
    float pdop;
} ubx_pvt_t;

typedef struct {
    uint8_t state;
    uint8_t cls, id;
    uint16_t len, pos;
    uint8_t ck_a, ck_b;
    uint8_t buf[100];
    uint32_t n_ok, n_bad;
} ubx_parser_t;

#define UBX_TIME_VALID_DATE 0x01u
#define UBX_TIME_VALID_TIME 0x02u
#define UBX_TIME_RESOLVED 0x04u

void ubx_init(ubx_parser_t* p);
/* Feeds one byte; returns 1 when a complete NAV-PVT was decoded into *out. */
int ubx_feed(ubx_parser_t* p, uint8_t byte, ubx_pvt_t* out);

/* Builds a UBX frame (sync, class, id, length, payload, checksum) into out; returns its length. */
size_t ubx_frame(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len, uint8_t* out);

/*
 * Configuration messages for the receiver (both legacy M8 style and M9/M10 VALSET keys; a
 * receiver ignores what it does not know). Fills `out` (>= 128 bytes) with the n-th message and
 * returns its length, 0 when n is past the end.
 *   NAV-PVT at `rate_hz` on UART1, NMEA off on UART1, airborne <4g dynamic model, UART1 baud.
 */
size_t ubx_config_msg(unsigned n, uint32_t baud, unsigned rate_hz, uint8_t* out);

#endif
