/* Unit tests: CANaerospace encoding, UBX parser, MS5611, magnetometer calibration. */
#include <string.h>

#include "../core/baro.h"
#include "../core/can_out.h"
#include "../core/canas.h"
#include "../core/magcal.h"
#include "../core/ms4525.h"
#include "../core/ubx.h"
#include "test.h"

static void test_canas(void) {
    canas_tx_t tx;
    canas_init(&tx, 7);
    canas_frame_t f;
    canas_float(&tx, &f, CANAS_ID_ROLL, 12.5f);
    CHECK(f.id == 312 && f.dlc == 8);
    CHECK(f.data[0] == 7 && f.data[1] == CANAS_FLOAT && f.data[2] == 0 && f.data[3] == 0);
    CHECK(f.data[4] == 0x41 && f.data[5] == 0x48 && f.data[6] == 0 && f.data[7] == 0); /* 12.5f big endian */
    CHECK_NEAR(canas_get_float(&f), 12.5f, 0.0f);
    canas_float(&tx, &f, CANAS_ID_ROLL, -1.0f);
    CHECK(f.data[3] == 1); /* message code counts per identifier */
    canas_float(&tx, &f, CANAS_ID_PITCH, 0.0f);
    CHECK(f.data[3] == 0);
    for (int i = 0; i < 300; ++i) canas_float(&tx, &f, CANAS_ID_ROLL, 0.0f);
    CHECK(f.data[3] == (uint8_t)(301 % 256)); /* wraps 255 -> 0 */

    canas_short(&tx, &f, CANAS_ID_GPS_MODE, 2);
    CHECK(f.dlc == 6 && f.data[1] == CANAS_SHORT && f.data[4] == 0 && f.data[5] == 2);
    canas_uchar(&tx, &f, CANAS_ID_GPS_SATS, 11);
    CHECK(f.dlc == 5 && f.data[1] == CANAS_UCHAR && f.data[4] == 11);

    /* identification service: addressed to us, to all, to someone else */
    const uint8_t req_all[4] = {0, 0, 0, 42}, req_us[4] = {7, 0, 0, 3}, req_other[4] = {9, 0, 0, 1};
    canas_frame_t r;
    CHECK(canas_node_service(&tx, 128, req_all, 4, 1, 2, &r) == 1);
    CHECK(r.id == 129 && r.dlc == 8 && r.data[0] == 7 && r.data[1] == CANAS_UCHAR4 && r.data[3] == 42);
    CHECK(r.data[4] == 1 && r.data[5] == 2 && r.data[6] == 0 && r.data[7] == 0);
    CHECK(canas_node_service(&tx, 128, req_us, 4, 1, 2, &r) == 1);
    CHECK(canas_node_service(&tx, 128, req_other, 4, 1, 2, &r) == 0);
    CHECK(canas_node_service(&tx, 300, req_all, 4, 1, 2, &r) == 0);
    const uint8_t req_level[4] = {7, CANAS_NODATA, CANAS_SERVICE_ALIGN_LEVEL, 5};
    CHECK(canas_node_service(&tx, 128, req_level, 4, 1, 2, &r) == 0); /* not IDS */
    const uint8_t pl[4] = {0x00, 0xC8, 0xFE, 0xD4};                   /* +2.00, -3.00 deg */
    canas_service_response(&tx, CANAS_SERVICE_ALIGN_LEVEL, 5, CANAS_SHORT2, pl, 4, &r);
    CHECK(r.id == 129 && r.dlc == 8 && r.data[0] == 7 && r.data[1] == CANAS_SHORT2 && r.data[2] == 100 && r.data[3] == 5);
    CHECK(r.data[4] == 0x00 && r.data[5] == 0xC8 && r.data[6] == 0xFE && r.data[7] == 0xD4);
    canas_service_response(&tx, CANAS_SERVICE_ALIGN_RESET, 6, CANAS_UCHAR, pl, 1, &r);
    CHECK(r.dlc == 5 && r.data[4] == 0x00);
}

static void test_can_out(void) {
    canas_tx_t tx;
    canas_init(&tx, 7);
    ahrs_out_t o;
    memset(&o, 0, sizeof(o));
    canas_frame_t fr[CAN_OUT_MAX_FRAMES];
    CHECK(can_out_build(&tx, &o, CAN_OUT_FAST | CAN_OUT_GNSS | CAN_OUT_TIME, fr) == 0);

    o.valid = AHRS_OUT_ATTITUDE | AHRS_OUT_HEADING | AHRS_OUT_RATES | AHRS_OUT_ALTITUDE | AHRS_OUT_CLIMB |
              AHRS_OUT_GNSS | AHRS_OUT_VARIATION | AHRS_OUT_TIME;
    o.roll_deg = 10.0f;
    o.pitch_deg = -2.0f;
    o.gps_mode = 2;
    o.year = 2026;
    o.month = 10;
    o.day = 3;
    o.hour = 14;
    o.min = 5;
    o.sec = 59;
    unsigned n = can_out_build(&tx, &o, CAN_OUT_FAST, fr);
    CHECK(n == 8);
    CHECK(fr[0].id == CANAS_ID_PITCH && canas_get_float(&fr[0]) == -2.0f);
    CHECK(fr[1].id == CANAS_ID_ROLL && canas_get_float(&fr[1]) == 10.0f);
    n = can_out_build(&tx, &o, CAN_OUT_GNSS, fr);
    CHECK(n == 7); /* variation is not sent: it comes from the display */
    for (unsigned i = 0; i < n; ++i) CHECK(fr[i].id != CANAS_ID_MAG_VAR);
    n = can_out_build(&tx, &o, CAN_OUT_TIME, fr);
    CHECK(n == 2);
    CHECK(fr[0].id == CANAS_ID_UTC && fr[0].data[1] == CANAS_UCHAR4);
    CHECK(fr[0].data[4] == 14 && fr[0].data[5] == 5 && fr[0].data[6] == 59);
    CHECK(fr[1].id == CANAS_ID_DATE && fr[1].data[4] == 3 && fr[1].data[5] == 10 && fr[1].data[6] == 26 && fr[1].data[7] == 20);
    CHECK(n <= CAN_OUT_MAX_FRAMES);
    CHECK(can_out_build(&tx, &o, CAN_OUT_FAST | CAN_OUT_GNSS | CAN_OUT_TIME, fr) <= CAN_OUT_MAX_FRAMES);
}

static void put32(uint8_t* b, uint32_t v) {
    for (int i = 0; i < 4; ++i) b[i] = (uint8_t)(v >> (8 * i));
}

static void test_ubx(void) {
    uint8_t pl[92], frame[128];
    memset(pl, 0, sizeof(pl));
    put32(pl + 0, 123456);
    pl[4] = 0xEA; /* 2026 */
    pl[5] = 0x07;
    pl[6] = 10;
    pl[7] = 3;
    pl[8] = 14;
    pl[9] = 5;
    pl[10] = 59;
    pl[11] = 0x07;
    pl[20] = 3;
    pl[21] = 0x01;
    pl[23] = 12;
    put32(pl + 24, (uint32_t)(int32_t)163456789);  /* lon 16.3456789 */
    put32(pl + 28, (uint32_t)(int32_t)-481234567); /* lat -48.1234567 */
    put32(pl + 32, 456789);
    put32(pl + 40, 1500);
    put32(pl + 48, (uint32_t)(int32_t)-12345);
    put32(pl + 56, 250);
    put32(pl + 60, 50000);
    put32(pl + 64, (uint32_t)(int32_t)27000000);
    put32(pl + 68, 200);
    const size_t len = ubx_frame(0x01, 0x07, pl, 92, frame);
    CHECK(len == 100);

    ubx_parser_t p;
    ubx_init(&p);
    ubx_pvt_t out;
    memset(&out, 0, sizeof(out));
    int got = 0;
    const uint8_t noise[] = {0x24, 0x47, 0xB5, 0x00, 0xB5};
    for (size_t i = 0; i < sizeof(noise); ++i) got += ubx_feed(&p, noise[i], &out);
    for (size_t i = 0; i < len; ++i) got += ubx_feed(&p, frame[i], &out);
    CHECK(got == 1);
    CHECK(out.year == 2026 && out.month == 10 && out.day == 3 && out.hour == 14 && out.min == 5 && out.sec == 59);
    CHECK(out.fix_type == 3 && out.num_sv == 12 && (out.flags & 1));
    CHECK_NEAR(out.lon_deg, 16.3456789, 1e-9);
    CHECK_NEAR(out.lat_deg, -48.1234567, 1e-9);
    CHECK_NEAR(out.height_m, 456.789f, 1e-3f);
    CHECK_NEAR(out.vel_ned[0], -12.345f, 1e-4f);
    CHECK_NEAR(out.vel_ned[2], 0.25f, 1e-4f);
    CHECK_NEAR(out.head_mot_deg, 270.0f, 1e-3f);
    CHECK_NEAR(out.h_acc_m, 1.5f, 1e-4f);

    frame[50] ^= 0x10; /* corrupt payload */
    got = 0;
    for (size_t i = 0; i < len; ++i) got += ubx_feed(&p, frame[i], &out);
    CHECK(got == 0 && p.n_bad >= 1);

    /* configuration messages carry valid checksums */
    for (unsigned n = 0;; ++n) {
        uint8_t m[128];
        const size_t l = ubx_config_msg(n, 115200, 5, m);
        if (!l) {
            CHECK(n == 7);
            break;
        }
        uint8_t a = 0, b = 0;
        for (size_t i = 2; i < l - 2; ++i) {
            a = (uint8_t)(a + m[i]);
            b = (uint8_t)(b + a);
        }
        CHECK(m[0] == 0xB5 && m[1] == 0x62 && m[2] == 0x06 && m[l - 2] == a && m[l - 1] == b);
        CHECK((size_t)(m[4] | m[5] << 8) + 8 == l);
    }
}

static void test_baro(void) {
    /* MS5611-01BA03 datasheet example */
    const uint16_t prom[8] = {0, 40127, 36924, 23317, 23282, 33464, 28312, 0};
    int32_t p, t;
    ms5611_compensate(prom, 9085466, 8569150, &p, &t);
    CHECK(t == 2007);
    CHECK(p == 100009);
    CHECK_NEAR(pressure_altitude_m(101325.0f), 0.0f, 0.01f);
    CHECK_NEAR(pressure_altitude_m(89874.6f), 1000.0f, 1.0f);
    CHECK_NEAR(pressure_altitude_m(70108.5f), 3000.0f, 2.0f);
    /* AN520 CRC-4: exactly one of the 16 possible nibbles matches */
    const uint16_t prom2[8] = {0x3132, 0x3334, 0x3536, 0x3738, 0x3940, 0x4142, 0x4344, 0x4500};
    uint16_t p2[8];
    memcpy(p2, prom2, sizeof(p2));
    int ok_any = 0;
    for (uint16_t c = 0; c < 16; ++c) {
        p2[7] = (uint16_t)(0x4500 | c);
        ok_any += ms5611_prom_crc_ok(p2);
    }
    CHECK(ok_any == 1); /* exactly one CRC nibble matches */
}

/* body field for random attitudes, distorted by soft iron S and hard iron o */
static void distort(const float S[3][3], const float o[3], const float b[3], float noise, float raw[3]) {
    m3_mul_v(S, b, raw);
    for (int i = 0; i < 3; ++i) raw[i] += o[i] + noise * randn();
}

static void test_magcal_ellipsoid(void) {
    const float S[3][3] = {{1.10f, 0.04f, -0.02f}, {0.04f, 0.93f, 0.03f}, {-0.02f, 0.03f, 1.02f}};
    const float o[3] = {25.0f, -40.0f, 12.0f};
    const float Bn[3] = {21.0f, 1.7f, 43.0f};
    magcal_t c;
    magcal_init(&c);
    int accepted = 0;
    for (int k = 0; k < 20000; ++k) {
        const quat_t q = quat_normalize((quat_t){randn(), randn(), randn(), randn()});
        float R[3][3], b[3], raw[3];
        quat_to_dcm(q, R);
        m3t_mul_v(R, Bn, b);
        distort(S, o, b, 0.3f, raw);
        accepted += magcal_add(&c, raw);
    }
    CHECK(accepted >= 1 && c.active.valid);
    CHECK(magcal_covered_bins(&c) == MAGCAL_BINS);
    for (int i = 0; i < 3; ++i) CHECK_NEAR(c.active.offset[i], o[i], 0.5f);
    /* calibrated magnitude must be constant over attitude */
    float mn = 1e9f, mx = 0.0f;
    for (int k = 0; k < 500; ++k) {
        const quat_t q = quat_normalize((quat_t){randn(), randn(), randn(), randn()});
        float R[3][3], b[3], raw[3], cal[3];
        quat_to_dcm(q, R);
        m3t_mul_v(R, Bn, b);
        distort(S, o, b, 0.0f, raw);
        magcal_apply(&c, raw, cal);
        const float n = v3_norm(cal);
        mn = n < mn ? n : mn;
        mx = n > mx ? n : mx;
    }
    printf("  magcal: offset %.2f %.2f %.2f, |m| %.2f..%.2f uT, residual %.4f\n", c.active.offset[0], c.active.offset[1],
           c.active.offset[2], mn, mx, c.last_residual);
    CHECK((mx - mn) / mn < 0.02f);

    /* limited attitude diversity (level flight) must not produce a solution */
    magcal_t d;
    magcal_init(&d);
    int acc2 = 0;
    for (int k = 0; k < 20000; ++k) {
        float R[3][3], b[3], raw[3];
        quat_to_dcm(quat_from_euler(0.1f * randn(), 0.05f * randn(), k * 0.01f), R);
        m3t_mul_v(R, Bn, b);
        distort(S, o, b, 0.3f, raw);
        acc2 += magcal_add(&d, raw);
    }
    CHECK(acc2 == 0 && !d.active.valid);
}

static void test_magcal_aided(void) {
    const float Bn[3] = {21.0f, 1.7f, 43.0f};
    const float o_true[3] = {6.0f, -4.0f, 3.0f};
    magcal_t c;
    magcal_init(&c); /* active offset 0 -> residual hard iron o_true */
    int changes = 0;
    for (int k = 0; k < 3000; ++k) {
        /* flight: heading sweeps through turns, banks up to 30 deg */
        const float yaw = 0.02f * (float)k, roll = 0.5f * sinf(0.013f * (float)k);
        float R[3][3], b[3], raw[3], cal[3];
        quat_to_dcm(quat_from_euler(roll, 0.03f * randn(), yaw), R);
        m3t_mul_v(R, Bn, b);
        for (int i = 0; i < 3; ++i) raw[i] = b[i] + o_true[i] + 0.3f * randn();
        magcal_apply(&c, raw, cal);
        changes += magcal_aided_add(&c, R, cal);
    }
    printf("  aided: offset %.2f %.2f %.2f (true %.1f %.1f %.1f), %d changes\n", c.active.offset[0], c.active.offset[1],
           c.active.offset[2], o_true[0], o_true[1], o_true[2], changes);
    CHECK(changes >= 1);
    CHECK_NEAR(c.active.offset[0], o_true[0], 1.0f);
    CHECK_NEAR(c.active.offset[1], o_true[1], 1.0f);
}

static void test_eig(void) {
    const float A[3][3] = {{4, 1, 0.5f}, {1, 3, 0.2f}, {0.5f, 0.2f, 2}};
    float e[3], V[3][3];
    sym3_eig(A, e, V);
    for (int k = 0; k < 3; ++k) {
        const float v[3] = {V[0][k], V[1][k], V[2][k]};
        float Av[3];
        m3_mul_v(A, v, Av);
        for (int i = 0; i < 3; ++i) CHECK_NEAR(Av[i], e[k] * v[i], 1e-4f);
    }
}

static void test_ms4525(void) {
    const ms4525_range_t r = {-1.0f, 1.0f, 0};
    float dp, tc;
    /* mid scale (8192 counts) = 0 Pa; 10 % = -1 psi; 90 % = +1 psi; status bits */
    const uint8_t zero[4] = {0x20, 0x00, 0x7F, 0xE0}; /* p = 8192, t = 1023 */
    CHECK(ms4525_decode(zero, &r, &dp, &tc) == MS4525_OK);
    CHECK_NEAR(dp, 0.0f, 1.0f);
    CHECK_NEAR(tc, 1023.0f * 200.0f / 2047.0f - 50.0f, 1e-3f);
    const uint8_t lo[4] = {0x06, 0x66, 0x00, 0x00}; /* p = 1638 */
    ms4525_decode(lo, &r, &dp, &tc);
    CHECK_NEAR(dp, -6894.757f, 3.0f);
    CHECK_NEAR(tc, -50.0f, 1e-3f);
    const uint8_t hi[4] = {0x39, 0x9A, 0xFF, 0xE0}; /* p = 14746, t = 2047 */
    ms4525_decode(hi, &r, &dp, &tc);
    CHECK_NEAR(dp, 6894.757f, 3.0f);
    CHECK_NEAR(tc, 150.0f, 1e-3f);
    const uint8_t stale[4] = {0xA0, 0x00, 0x7F, 0xE0};
    CHECK(ms4525_decode(stale, &r, &dp, &tc) == MS4525_STALE);
    CHECK_NEAR(dp, 0.0f, 1.0f);
    const uint8_t fault[4] = {0xC0, 0, 0, 0};
    CHECK(ms4525_decode(fault, &r, &dp, &tc) == MS4525_FAULT);
}

void run_unit_tests(void) {
    RUN(test_ms4525);
    RUN(test_canas);
    RUN(test_can_out);
    RUN(test_ubx);
    RUN(test_baro);
    RUN(test_eig);
    RUN(test_magcal_ellipsoid);
    RUN(test_magcal_aided);
}
