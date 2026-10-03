/*
 * Closed-loop simulation: a light aircraft taxis, takes off, climbs and flies coordinated turns
 * in wind. Synthetic IMU (with biases and noise), magnetometer (hard and soft iron), barometer,
 * GNSS (5 Hz, with an outage) and optional airspeed drive the AHRS; outputs are compared with truth.
 */
#include <stdlib.h>
#include <string.h>

#include "../core/ahrs.h"
#include "../core/baro.h"
#include "test.h"

#define FS 208.0f
#define DEC_TRUE (4.7f * DEG2RAD)
#define LAT0 47.0
#define LON0 15.4

typedef struct {
    int use_gnss;
    int use_airspeed;
    float outage_from, outage_to; /* no GNSS messages in [from, mid), fix lost in [mid, to) */
    float duration;
} scenario_t;

typedef struct {
    float t_max_tilt, t_max_hdg;
    float max_tilt, rms_tilt, max_tilt_outage, max_hdg, max_climb_err;
    int att_invalid, hdg_invalid;
    gnss_state_t st_coast, st_lost, st_recovered;
    float dec_err;
    int time_ok;
} result_t;

static float smoothstep(float x) {
    if (x <= 0.0f) return 0.0f;
    if (x >= 1.0f) return 1.0f;
    return x * x * (3.0f - 2.0f * x);
}

static float pulse(float t, float t0, float t1, float ramp) { return smoothstep((t - t0) / ramp) * smoothstep((t1 - t) / ramp); }

/* flight profile: airspeed, bank, vertical speed (NED down), angle of attack, wind */
static void profile(float t, float* V, float* bank, float* vz, float* alpha, float wind[3]) {
    *V = 50.0f * smoothstep((t - 60.0f) / 30.0f);
    const float air = smoothstep((t - 90.0f) / 5.0f);
    *alpha = 3.0f * DEG2RAD * air;
    *vz = -3.0f * pulse(t, 100.0f, 160.0f, 5.0f);
    *bank = DEG2RAD * (30.0f * pulse(t, 180.0f, 240.0f, 4.0f) - 25.0f * pulse(t, 270.0f, 300.0f, 4.0f) +
                       30.0f * pulse(t, 350.0f, 400.0f, 4.0f) - 20.0f * pulse(t, 420.0f, 440.0f, 3.0f) +
                       30.0f * pulse(t, 480.0f, 560.0f, 4.0f));
    const float w = smoothstep((t - 90.0f) / 30.0f);
    wind[0] = 4.0f * w;
    wind[1] = -7.0f * w;
    wind[2] = 0.0f;
}

static float angdiff_deg(float a, float b) { return fabsf(wrap_pi((a - b) * DEG2RAD)) * RAD2DEG; }

static void run(const scenario_t* sc, result_t* res) {
    memset(res, 0, sizeof(*res));
    rand_seed(12345);
    ahrs_config_t cfg;
    ahrs_default_config(&cfg);
    ahrs_t* a = (ahrs_t*)calloc(1, sizeof(ahrs_t));
    ahrs_init(a, &cfg);

    /* magnetometer distortion and its (known) calibration */
    const float S[3] = {1.08f, 0.95f, 1.0f};
    const float o[3] = {20.0f, -35.0f, 10.0f};
    magcal_params_t mp;
    memset(&mp, 0, sizeof(mp));
    for (int i = 0; i < 3; ++i) {
        mp.offset[i] = o[i];
        mp.W[i][i] = 1.0f / S[i];
    }
    mp.radius = 47.9f;
    mp.valid = 1;
    ahrs_set_magcal(a, &mp);
    const float Bn[3] = {21.0f * cosf(DEC_TRUE), 21.0f * sinf(DEC_TRUE), 43.0f};

    const float bg[3] = {0.5f * DEG2RAD, -0.3f * DEG2RAD, 0.2f * DEG2RAD};
    const float ba[3] = {0.08f, -0.05f, 0.1f};
    const float sg = 0.0005f * sqrtf(FS), sa = 0.003f * sqrtf(FS);
    const float dt = 1.0f / FS;

    double p[3] = {0.0, 0.0, -300.0};
    float psi = 30.0f * DEG2RAD, v_prev[3] = {0, 0, 0};
    quat_t q_prev = quat_from_euler(0.0f, 0.0f, psi);
    double next_gnss = 0.0, sum_tilt2 = 0.0;
    long n_tilt = 0;
    const long steps = (long)(sc->duration * FS);
    for (long k = 1; k <= steps; ++k) {
        const float t = (float)k * dt;
        float V, bank, vz, alpha, wind[3];
        profile(t, &V, &bank, &vz, &alpha, wind);
        if (V > 20.0f) psi += GRAVITY * tanf(bank) / V * dt;
        const float gam = V > 1.0f ? asinf(-vz / V) : 0.0f;
        float v[3] = {V * cosf(gam) * cosf(psi) + wind[0], V * cosf(gam) * sinf(psi) + wind[1], vz + wind[2]};
        for (int i = 0; i < 3; ++i) p[i] += 0.5 * (double)(v[i] + v_prev[i]) * dt;
        const quat_t q = quat_from_euler(bank, gam + alpha, psi);

        /* IMU: rotation increment and specific force over the step */
        const quat_t qi = {q_prev.w, -q_prev.x, -q_prev.y, -q_prev.z};
        const quat_t dq = quat_normalize(quat_mul(qi, q));
        const float s = sqrtf(dq.x * dq.x + dq.y * dq.y + dq.z * dq.z);
        const float ang = 2.0f * atan2f(s, dq.w);
        float gyro[3] = {0, 0, 0}, acc[3], fn[3], Rp[3][3], R[3][3];
        if (s > 1e-9f) {
            gyro[0] = dq.x / s * ang / dt;
            gyro[1] = dq.y / s * ang / dt;
            gyro[2] = dq.z / s * ang / dt;
        }
        for (int i = 0; i < 3; ++i) fn[i] = (v[i] - v_prev[i]) / dt;
        fn[2] -= GRAVITY;
        quat_to_dcm(q_prev, Rp);
        m3t_mul_v(Rp, fn, acc);
        for (int i = 0; i < 3; ++i) {
            gyro[i] += bg[i] + sg * randn();
            acc[i] += ba[i] + sa * randn();
        }
        memcpy(v_prev, v, sizeof(v));
        q_prev = q;
        quat_to_dcm(q, R);

        ahrs_imu(a, gyro, acc, dt);

        if (k % 4 == 0) { /* 52 Hz magnetometer and barometer */
            float b[3], raw[3];
            m3t_mul_v(R, Bn, b);
            for (int i = 0; i < 3; ++i) raw[i] = S[i] * b[i] + o[i] + 0.3f * randn();
            ahrs_mag(a, raw);
            const float h = (float)-p[2] + 0.3f * randn();
            ahrs_baro(a, 101325.0f * powf(1.0f - h / 44330.77f, 1.0f / 0.190263f));
        }
        if (sc->use_airspeed && k % 10 == 0 && V > 20.0f) ahrs_airspeed(a, V, V);

        const int in_outage = t >= sc->outage_from && t < sc->outage_to;
        const int silent = in_outage && t < 0.5f * (sc->outage_from + sc->outage_to);
        if (sc->use_gnss && t >= next_gnss && !silent) {
            next_gnss += 0.2;
            ubx_pvt_t g;
            memset(&g, 0, sizeof(g));
            const double s0 = sin(LAT0 * AHRS_PI / 180.0), w0 = sqrt(1.0 - 6.69437999014e-3 * s0 * s0);
            const double rn = 6378137.0 / w0, rm = 6378137.0 * (1.0 - 6.69437999014e-3) / (w0 * w0 * w0);
            g.lat_deg = LAT0 + (p[0] + 1.5 * randn()) / rm * (180.0 / AHRS_PI);
            g.lon_deg = LON0 + (p[1] + 1.5 * randn()) / (rn * cos(LAT0 * AHRS_PI / 180.0)) * (180.0 / AHRS_PI);
            g.height_m = (float)-p[2] + 48.0f + 2.0f * randn();
            for (int i = 0; i < 3; ++i) g.vel_ned[i] = v[i] + 0.1f * randn();
            g.g_speed_ms = sqrtf(v[0] * v[0] + v[1] * v[1]);
            g.head_mot_deg = wrap_360(atan2f(v[1], v[0]) * RAD2DEG);
            g.h_acc_m = 1.5f;
            g.v_acc_m = 2.5f;
            g.s_acc_ms = 0.2f;
            g.fix_type = in_outage ? 0 : 3;
            g.flags = in_outage ? 0 : 1;
            g.num_sv = in_outage ? 2 : 12;
            const int secs = 14 * 3600 + (int)t;
            g.year = 2026;
            g.month = 10;
            g.day = 3;
            g.hour = (uint8_t)(secs / 3600);
            g.min = (uint8_t)(secs / 60 % 60);
            g.sec = (uint8_t)(secs % 60);
            g.valid = 0x07;
            ahrs_gnss(a, &g);
        }

        ahrs_out_t out;
        ahrs_output(a, &out);
        float tr, tp, ty;
        dcm_to_euler(R, &tr, &tp, &ty);
        if (t > 5.0f) {
            if (!(out.valid & AHRS_OUT_ATTITUDE)) res->att_invalid++;
            if (!(out.valid & AHRS_OUT_HEADING)) res->hdg_invalid++;
            const float e_tilt = fmaxf(angdiff_deg(out.roll_deg, tr * RAD2DEG), angdiff_deg(out.pitch_deg, tp * RAD2DEG));
            if (e_tilt > res->max_tilt) {
                res->max_tilt = e_tilt;
                res->t_max_tilt = t;
            }
            if (in_outage || (sc->outage_to > 0 && t >= sc->outage_to && t < sc->outage_to + 30.0f))
                res->max_tilt_outage = fmaxf(res->max_tilt_outage, e_tilt);
            sum_tilt2 += e_tilt * e_tilt;
            n_tilt++;
            const float e_hdg = angdiff_deg(out.heading_mag_deg, (ty - DEC_TRUE) * RAD2DEG);
            if (t > 10.0f && e_hdg > res->max_hdg) {
                res->max_hdg = e_hdg;
                res->t_max_hdg = t;
            }
            if (!sc->use_gnss && !sc->use_airspeed && t > 7.0f && t < 13.0f && k % (long)(FS / 4) == 0) {
                const float(*P)[ESKF_N] = (const float(*)[ESKF_N])a->kf.P;
                printf("    t=%5.2f r %6.2f p %6.2f sth %.3f %.3f %.3f sba %.3f %.3f corr(thy,bax) %.4f sdec %.2f nis %.1f magrej %u bb %.2f\n",
                       t, out.roll_deg, out.pitch_deg, sqrtf(P[6][6]) * RAD2DEG, sqrtf(P[7][7]) * RAD2DEG,
                       sqrtf(P[8][8]) * RAD2DEG, sqrtf(P[9][9]), sqrtf(P[10][10]), P[7][9] / sqrtf(P[7][7] * P[9][9]),
                       sqrtf(P[16][16]) * RAD2DEG, a->kf.last_nis, (unsigned)a->mag_rejects, a->kf.bbaro);
            }
            if (!sc->use_gnss && t < 95.0f && k % (long)(4 * FS) == 0)
                printf("    t=%3.0f r %6.1f p %5.1f v %6.1f %6.1f %6.1f bg %5.2f %5.2f %5.2f ba %5.2f %5.2f %5.2f st %d mv %d Pv %.1f\n",
                       t, out.roll_deg, out.pitch_deg, a->kf.v[0], a->kf.v[1], a->kf.v[2], a->kf.bg[0] * RAD2DEG,
                       a->kf.bg[1] * RAD2DEG, a->kf.bg[2] * RAD2DEG, a->kf.ba[0], a->kf.ba[1], a->kf.ba[2], a->stationary,
                       a->ever_moved, sqrtf(a->kf.P[ES_V][ES_V]));
            if (k % (long)(30 * FS) == 0)
                printf("    t=%3.0f roll %6.2f/%6.2f pitch %5.2f/%5.2f hdg %6.2f/%6.2f gnss %d stat %d\n", t, out.roll_deg,
                       tr * RAD2DEG, out.pitch_deg, tp * RAD2DEG, out.heading_mag_deg, wrap_360((ty - DEC_TRUE) * RAD2DEG),
                       out.gnss_state, a->stationary);
            if (t > 110.0f && t < 150.0f && (out.valid & AHRS_OUT_CLIMB))
                res->max_climb_err = fmaxf(res->max_climb_err, fabsf(out.climb_ms + vz));
        }
        if (sc->use_gnss && sc->outage_to > 0) {
            if (fabsf(t - (sc->outage_from + 3.0f)) < 0.5f * dt) res->st_coast = out.gnss_state;
            if (fabsf(t - (sc->outage_from + 10.0f)) < 0.5f * dt) res->st_lost = out.gnss_state;
            if (fabsf(t - (sc->outage_to + 1.0f)) < 0.5f * dt) res->st_recovered = out.gnss_state;
        }
        if (k == steps) {
            res->dec_err = fabsf(a->kf.dec - DEC_TRUE) * RAD2DEG;
            const int secs = 14 * 3600 + (int)t;
            res->time_ok = (out.valid & AHRS_OUT_TIME) && out.hour == secs / 3600 && out.min == secs / 60 % 60 &&
                           abs(out.sec - secs % 60) <= 1 && out.year == 2026;
        }
    }
    res->rms_tilt = (float)sqrt(sum_tilt2 / (double)(n_tilt ? n_tilt : 1));
    printf("  wind est %.1f %.1f (true 4 -7), airspeed est %.1f (true 50), alpha est %.2f deg\n", a->wx[1], a->wx[2],
           a->wx[0], a->alpha_est * RAD2DEG);
    printf("  max tilt at t=%.1f, max heading at t=%.1f\n", res->t_max_tilt, res->t_max_hdg);
    printf("  tilt max %.2f rms %.2f (outage max %.2f) deg, mag heading max %.2f deg, climb err %.2f m/s, "
           "dec err %.2f deg, invalid att/hdg %d/%d, gyro bias est %.3f %.3f %.3f deg/s\n",
           res->max_tilt, res->rms_tilt, res->max_tilt_outage, res->max_hdg, res->max_climb_err, res->dec_err,
           res->att_invalid, res->hdg_invalid, a->kf.bg[0] * RAD2DEG, a->kf.bg[1] * RAD2DEG, a->kf.bg[2] * RAD2DEG);
    free(a);
}

static void test_sim_gnss_outage(void) {
    const scenario_t sc = {1, 0, 330.0f, 450.0f, 600.0f};
    result_t r;
    run(&sc, &r);
    CHECK(r.att_invalid == 0);
    CHECK(r.max_tilt < 2.5f);
    CHECK(r.rms_tilt < 0.7f);
    CHECK(r.max_tilt_outage < 4.0f);
    CHECK(r.max_hdg < 5.0f);
    CHECK(r.max_climb_err < 0.5f);
    CHECK(r.st_coast == GNSS_COAST);
    CHECK(r.st_lost == GNSS_LOST);
    CHECK(r.st_recovered == GNSS_OK);
    CHECK(r.dec_err < 3.0f);
    CHECK(r.time_ok);
}

static void test_sim_no_gnss_airspeed(void) {
    const scenario_t sc = {0, 1, 0.0f, 0.0f, 600.0f};
    result_t r;
    run(&sc, &r);
    CHECK(r.att_invalid == 0);
    CHECK(r.max_tilt < 4.0f);
    CHECK(r.max_hdg < 6.0f);
}

static void test_sim_no_gnss_no_airspeed(void) {
    /* worst case, informational: only the body-velocity constraint is left */
    const scenario_t sc = {0, 0, 0.0f, 0.0f, 600.0f};
    result_t r;
    run(&sc, &r);
    CHECK(r.att_invalid == 0);
    CHECK(r.max_tilt < 20.0f);
}

void run_sim_tests(void) {
    RUN(test_sim_gnss_outage);
    RUN(test_sim_no_gnss_airspeed);
    RUN(test_sim_no_gnss_no_airspeed);
}
