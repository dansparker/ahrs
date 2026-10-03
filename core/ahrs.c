#include "ahrs.h"

#include <string.h>

#include "baro.h"

#define ALIGN_TIME_S 1.0f
#define PSEUDO_PERIOD_S 0.1
#define BARO_TIMEOUT_S 0.5
#define MAG_TIMEOUT_S 5.0
#define AIRSPEED_TIMEOUT_S 0.5
#define EARTH_A 6378137.0

void ahrs_default_config(ahrs_config_t* c) {
    memset(c, 0, sizeof(*c));
    c->dec0_deg = 0.0f;
    c->gnss_timeout_s = 1.5f;
    c->gnss_coast_s = 5.0f;
    c->gnss_reset_s = 30.0f;
    c->mag_sigma_deg = 5.0f;
    c->mag_rate_hz = 10.0f;
    c->alpha0_deg = 2.0f;
    /* LSM6DSOX class MEMS IMU, deliberately pessimistic for vibration */
    c->noise.acc_noise = 0.05f;
    c->noise.gyro_noise = 0.005f;
    c->noise.acc_bias_rw = 0.002f;
    c->noise.gyro_bias_rw = 0.0002f;
    c->noise.baro_bias_rw = 0.05f;
    c->noise.dec_rw = 1e-5f;
    c->noise.acc_bias_max = 1.0f;
    c->noise.gyro_bias_max = 5.0f * DEG2RAD;
}

void ahrs_init(ahrs_t* a, const ahrs_config_t* cfg) {
    memset(a, 0, sizeof(*a));
    a->cfg = *cfg;
    magcal_init(&a->mc);
    a->gnss = GNSS_NONE;
    a->alpha_est = cfg->alpha0_deg * DEG2RAD;
    a->wP[0][0] = sq(30.0f);
    a->wP[1][1] = a->wP[2][2] = sq(20.0f);
}

void ahrs_set_magcal(ahrs_t* a, const magcal_params_t* p) { magcal_set(&a->mc, p); }

int ahrs_take_magcal_changed(ahrs_t* a) {
    const int c = a->magcal_changed;
    a->magcal_changed = 0;
    return c;
}

/* Magnetic heading of a calibrated body vector using roll/pitch from R (yaw cancels out). */
static float mag_heading(const float R[3][3], const float m_b[3], float* dip) {
    float mn[3];
    m3_mul_v(R, m_b, mn);
    const float yaw = atan2f(R[1][0], R[0][0]);
    const float h = sqrtf(mn[0] * mn[0] + mn[1] * mn[1]);
    if (dip) *dip = atan2f(mn[2], h);
    return wrap_pi(yaw - atan2f(mn[1], mn[0]));
}

static void align(ahrs_t* a) {
    const float n = (float)a->align_n;
    float f[3], g[3];
    for (int i = 0; i < 3; ++i) {
        f[i] = a->acc_sum[i] / n;
        g[i] = a->gyro_sum[i] / n;
    }
    const float roll = atan2f(-f[1], -f[2]);
    const float pitch = atan2f(f[0], sqrtf(f[1] * f[1] + f[2] * f[2]));
    const int still = a->gyro_max < 0.05f && fabsf(v3_norm(f) - GRAVITY) < 0.5f;
    const float dec = a->cfg.dec0_deg * DEG2RAD;
    float yaw = 0.0f, sig_yaw = AHRS_PI;
    if (a->have_mag) {
        float R[3][3];
        quat_to_dcm(quat_from_euler(roll, pitch, 0.0f), R);
        yaw = mag_heading(R, a->mag_cal, 0) + dec;
        sig_yaw = 20.0f * DEG2RAD;
    }
    eskf_init(&a->kf, &a->cfg.noise, quat_from_euler(roll, pitch, yaw), (still ? 2.0f : 15.0f) * DEG2RAD, sig_yaw, dec,
              10.0f * DEG2RAD);
    if (still) {
        for (int i = 0; i < 3; ++i) {
            a->kf.bg[i] = g[i];
            a->kf.P[ES_BG + i][ES_BG + i] = sq(0.1f * DEG2RAD);
        }
    }
    if (a->have_baro) a->kf.p[2] = -a->baro_alt;
    a->aligned = 1;
}

static int wind_known(const ahrs_t* a) { return a->wP[1][1] < sq(3.0f) && a->wP[2][2] < sq(3.0f); }

/* Wind triangle: v_gnss = Va (cos psi, sin psi) + w (zero sideslip). Observable in turns. */
static void wind_update(ahrs_t* a, const float vel[3]) {
    const float psi = eskf_yaw(&a->kf);
    const float h[2][3] = {{cosf(psi), 1.0f, 0.0f}, {sinf(psi), 0.0f, 1.0f}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) a->wP[i][j] /= 0.995f;
    for (int r = 0; r < 2; ++r) {
        float Ph[3], den = sq(0.5f), pred = 0.0f;
        for (int i = 0; i < 3; ++i) {
            Ph[i] = a->wP[i][0] * h[r][0] + a->wP[i][1] * h[r][1] + a->wP[i][2] * h[r][2];
            den += h[r][i] * Ph[i];
            pred += h[r][i] * a->wx[i];
        }
        const float e = vel[r] - pred;
        for (int i = 0; i < 3; ++i) {
            a->wx[i] += Ph[i] / den * e;
            for (int j = 0; j < 3; ++j) a->wP[i][j] -= Ph[i] * Ph[j] / den;
        }
    }
    for (int i = 0; i < 3; ++i) { /* forgetting must not inflate unobservable directions without bound */
        const float lim = sq(i == 0 ? 30.0f : 20.0f);
        if (a->wP[i][i] > lim) {
            const float k = sqrtf(lim / a->wP[i][i]);
            for (int j = 0; j < 3; ++j) {
                a->wP[i][j] *= k;
                a->wP[j][i] *= k;
            }
        }
    }
    if (wind_known(a)) {
        const float w[3] = {a->wx[1], a->wx[2], 0.0f};
        float va[3], vb[3];
        for (int i = 0; i < 3; ++i) va[i] = a->kf.v[i] - w[i];
        m3t_mul_v(a->kf.R, va, vb);
        if (vb[0] > 15.0f) a->alpha_est += 0.01f * (atan2f(vb[2], vb[0]) - a->alpha_est);
    }
}

static void pseudo_measurements(ahrs_t* a) {
    eskf_t* f = &a->kf;
    const int gnss_ok = a->gnss == GNSS_OK;
    const int zupt_ok = (gnss_ok && a->last_gs < 0.5f) || (!a->ever_moved && a->gnss == GNSS_NONE);
    if (a->stationary && zupt_ok) {
        for (int i = 0; i < 3; ++i) eskf_update_body_vel(f, i, 0.0f, sq(0.1f), 0.0f, 0u, 0);
        return;
    }
    if (gnss_ok || a->gnss == GNSS_COAST) return;
    /* GNSS denied: the air-relative velocity lies along the body x axis (no sideslip, angle of
     * attack as learned before the outage). This lets the filter separate centripetal acceleration
     * from gravity in turns. Sensor biases are "consider" states here: the constraint is only
     * approximate and must not be learned as an accelerometer or gyro bias. */
    const uint32_t cm = ESKF_CONSIDER_BIASES;
    const int wk = a->ever_fused && wind_known(a);
    const float w[3] = {wk ? a->wx[1] : 0.0f, wk ? a->wx[2] : 0.0f, 0.0f};
    float vb[3], va[3];
    for (int i = 0; i < 3; ++i) va[i] = f->v[i] - w[i];
    m3t_mul_v(f->R, va, vb);
    eskf_update_body_vel(f, 1, 0.0f, sq(2.0f), 0.0f, cm, w);
    eskf_update_body_vel(f, 2, vb[0] * tanf(a->alpha_est), sq(2.0f), 0.0f, cm, w);
    if (a->t - a->t_airspeed < AIRSPEED_TIMEOUT_S) {
        eskf_update_body_vel(f, 0, a->tas, sq(3.0f), 0.0f, cm, w);
    } else if (wk && a->wP[0][0] < sq(3.0f)) {
        eskf_update_body_vel(f, 0, a->wx[0], sq(4.0f), 0.0f, cm, w);
    } else if (a->ever_fused) {
        eskf_update_body_vel(f, 0, a->last_gs, sq(15.0f), 0.0f, cm, w);
    } else {
        /* no speed reference at all: keep the velocity bounded (degraded, accelerometer-only tilt) */
        eskf_update_body_vel(f, 0, 0.0f, sq(40.0f), 0.0f, cm, w);
    }
}

void ahrs_imu(ahrs_t* a, const float gyro[3], const float acc[3], float dt) {
    a->t += dt;
    if (!a->aligned) {
        for (int i = 0; i < 3; ++i) {
            a->acc_sum[i] += acc[i];
            a->gyro_sum[i] += gyro[i];
        }
        const float gn = v3_norm(gyro);
        if (gn > a->gyro_max) a->gyro_max = gn;
        a->align_n++;
        if (a->t >= ALIGN_TIME_S && (a->have_mag || a->t >= 3.0 * ALIGN_TIME_S)) align(a);
        return;
    }
    eskf_t* f = &a->kf;
    eskf_predict(f, gyro, acc, dt);

    for (int i = 0; i < 3; ++i) {
        a->gyro_c[i] = gyro[i] - f->bg[i];
        a->acc_c[i] = acc[i] - f->ba[i];
    }
    const float k_rate = dt / (0.05f + dt), k_latg = dt / (0.2f + dt), k_act = dt / (0.5f + dt);
    for (int i = 0; i < 3; ++i) a->rate_lpf[i] += k_rate * (a->gyro_c[i] - a->rate_lpf[i]);
    a->latg_lpf += k_latg * (a->acc_c[1] / GRAVITY - a->latg_lpf);
    a->gyro_act += k_act * (v3_norm(a->gyro_c) - a->gyro_act);
    a->acc_act += k_act * (fabsf(v3_norm(acc) - GRAVITY) - a->acc_act);
    /* Stationary: calm gyro and accelerometer, and the specific force keeps its direction.
     * A direction change without rotation is linear acceleration (e.g. take-off roll): moving. */
    const int calm = a->gyro_act < 0.03f && a->acc_act < 0.3f;
    for (int i = 0; i < 3; ++i) a->acc_lpf[i] += k_act * (acc[i] - a->acc_lpf[i]); /* raw: bias estimates move */
    const float fn = v3_norm(a->acc_lpf);
    if (calm && !a->stationary && fn > 1.0f) {
        for (int i = 0; i < 3; ++i) a->f_anchor[i] = a->acc_lpf[i] / fn;
        a->stationary = 1;
    } else if (a->stationary) {
        const float c = fn > 1.0f ? v3_dot(a->acc_lpf, a->f_anchor) / fn : 0.0f;
        if (!calm || c < 0.99985f) { /* > 1 deg */
            if (calm) a->ever_moved = 1;
            a->stationary = 0;
        }
    }

    /* GNSS supervision */
    if (a->gnss == GNSS_OK && a->t - a->t_gnss_ok > a->cfg.gnss_timeout_s) a->gnss = GNSS_COAST;
    if (a->gnss == GNSS_COAST && a->t - a->t_gnss_ok > a->cfg.gnss_timeout_s + a->cfg.gnss_coast_s) a->gnss = GNSS_LOST;

    if (a->t - a->t_pseudo >= PSEUDO_PERIOD_S) {
        a->t_pseudo = a->t;
        pseudo_measurements(a);
    }
}

void ahrs_mag(ahrs_t* a, const float raw[3]) {
    if (magcal_add(&a->mc, raw)) a->magcal_changed = 1;
    magcal_apply(&a->mc, raw, a->mag_cal);
    a->have_mag = 1;
    if (!a->aligned) return;

    eskf_t* f = &a->kf;
    float dip;
    const float psi = mag_heading(f->R, a->mag_cal, &dip);
    const float norm = v3_norm(a->mag_cal);
    if (a->mag_norm_lpf <= 0.0f) {
        a->mag_norm_lpf = norm;
        a->mag_dip_lpf = dip;
    }
    a->mag_norm_lpf += 0.002f * (norm - a->mag_norm_lpf);
    a->mag_dip_lpf += 0.002f * (dip - a->mag_dip_lpf);

    if (a->t - a->t_mag_upd < 1.0f / a->cfg.mag_rate_hz) return;
    a->t_mag_upd = a->t;
    /* disturbance gating: field strength and inclination must match the long-term values */
    const int plausible = fabsf(norm / a->mag_norm_lpf - 1.0f) < 0.12f && fabsf(dip - a->mag_dip_lpf) < 8.0f * DEG2RAD;
    if (plausible && eskf_update_mag_heading(f, psi, sq(a->cfg.mag_sigma_deg * DEG2RAD), 16.0f)) {
        a->t_mag_ok = a->t;
    } else {
        a->mag_rejects++;
    }

    /* attitude-aided hard-iron refinement: only with a GNSS-observed yaw and calm flight */
    if (a->gnss == GNSS_OK && a->t - a->t_mag_aided >= 1.0 && plausible && sqrtf(f->P[ES_TH + 2][ES_TH + 2]) < 2.0f * DEG2RAD &&
        v3_norm(a->gyro_c) < 0.3f && a->last_gs > 15.0f) {
        a->t_mag_aided = a->t;
        if (magcal_aided_add(&a->mc, f->R, a->mag_cal)) a->magcal_changed = 1;
    }
}

void ahrs_baro(ahrs_t* a, float p_pa) {
    if (!(p_pa > 20000.0f && p_pa < 110000.0f)) return;
    a->baro_p = p_pa;
    a->baro_alt = pressure_altitude_m(p_pa);
    a->have_baro = 1;
    a->t_baro = a->t;
    if (a->aligned) eskf_update_baro(&a->kf, a->baro_alt, sq(0.5f), 25.0f);
}

void ahrs_airspeed(ahrs_t* a, float ias_ms, float tas_ms) {
    a->ias = ias_ms;
    a->tas = tas_ms;
    a->t_airspeed = a->t;
    if (ias_ms > 15.0f) a->ever_moved = 1;
}

static void lla_to_ned(const ahrs_t* a, double lat, double lon, float h, float ned[3]) {
    const double lat0 = a->lat0 * (AHRS_PI / 180.0);
    const double e2 = 6.69437999014e-3, s = sin(lat0);
    const double w = sqrt(1.0 - e2 * s * s);
    const double rn = EARTH_A / w, rm = EARTH_A * (1.0 - e2) / (w * w * w);
    ned[0] = (float)((lat - a->lat0) * (AHRS_PI / 180.0) * rm);
    ned[1] = (float)((lon - a->lon0) * (AHRS_PI / 180.0) * rn * cos(lat0));
    ned[2] = -h;
}

void ahrs_gnss(ahrs_t* a, const ubx_pvt_t* p) {
    a->pvt = *p;
    const uint8_t tv = UBX_TIME_VALID_DATE | UBX_TIME_VALID_TIME | UBX_TIME_RESOLVED;
    if ((p->valid & tv) == tv) a->t_gnss_time = a->t + 1e-9; /* +eps: valid even at t = 0 */

    const int ok = p->fix_type >= 3 && p->fix_type <= 4 && (p->flags & 1u) && p->num_sv >= 5 && p->h_acc_m < 15.0f &&
                   p->s_acc_ms < 2.0f;
    if (!ok || !a->aligned) return;
    eskf_t* f = &a->kf;

    if (!a->have_origin) {
        a->lat0 = p->lat_deg;
        a->lon0 = p->lon_deg;
        a->have_origin = 1;
    }
    float ned[3];
    lla_to_ned(a, p->lat_deg, p->lon_deg, p->height_m, ned);
    if (sqrtf(ned[0] * ned[0] + ned[1] * ned[1]) > 20000.0f) { /* keep the flat-earth area small */
        f->p[0] -= ned[0];
        f->p[1] -= ned[1];
        a->lat0 = p->lat_deg;
        a->lon0 = p->lon_deg;
        ned[0] = ned[1] = 0.0f;
    }

    const float sh = p->h_acc_m > 1.5f ? p->h_acc_m : 1.5f;
    const float sv = 1.5f * p->v_acc_m > 3.0f ? 1.5f * p->v_acc_m : 3.0f;
    const float ss = p->s_acc_ms > 0.3f ? p->s_acc_ms : 0.3f;
    const int reset = !a->ever_fused || a->gnss_rejects >= 10 ||
                      (a->gnss == GNSS_LOST && a->t - a->t_gnss_ok > a->cfg.gnss_reset_s);
    if (reset) {
        eskf_reset_pos(f, ned, sh);
        f->P[ES_P + 2][ES_P + 2] = sv * sv;
        eskf_reset_vel(f, p->vel_ned, 2.0f * ss);
        if (a->have_baro) eskf_reset_baro_bias(f, a->baro_alt - p->height_m, 10.0f);
        if (a->ever_fused) { /* after an outage the tilt may carry pseudo-measurement errors: let GNSS fix
                              * the attitude rather than the accelerometer bias */
            f->P[ES_TH][ES_TH] += sq(2.0f * DEG2RAD);
            f->P[ES_TH + 1][ES_TH + 1] += sq(2.0f * DEG2RAD);
        }
        a->gnss_rejects = 0;
        a->ever_fused = 1;
    } else {
        int rej = 0;
        rej |= !eskf_update_pos(f, 0, ned[0], sh * sh, 25.0f);
        rej |= !eskf_update_pos(f, 1, ned[1], sh * sh, 25.0f);
        eskf_update_pos(f, 2, ned[2], sv * sv, 25.0f);
        for (int i = 0; i < 3; ++i) rej |= !eskf_update_vel(f, i, p->vel_ned[i], (i == 2 ? 2.0f : 1.0f) * ss * ss, 25.0f);
        a->gnss_rejects = rej ? a->gnss_rejects + 1 : 0;
    }
    a->last_gs = p->g_speed_ms;
    if (p->g_speed_ms > 10.0f) a->ever_moved = 1;
    if (p->g_speed_ms > 15.0f) wind_update(a, p->vel_ned);
    a->t_gnss_ok = a->t;
    a->gnss = GNSS_OK;
}

void ahrs_output(const ahrs_t* a, ahrs_out_t* o) {
    memset(o, 0, sizeof(*o));
    o->gnss_state = a->gnss;
    if (!a->aligned) return;
    const eskf_t* f = &a->kf;
    float roll, pitch, yaw;
    dcm_to_euler(f->R, &roll, &pitch, &yaw);
    o->roll_deg = roll * RAD2DEG;
    o->pitch_deg = pitch * RAD2DEG;
    o->heading_true_deg = wrap_360(yaw * RAD2DEG);
    o->heading_mag_deg = wrap_360((yaw - f->dec) * RAD2DEG);
    o->pitch_rate_dps = a->rate_lpf[1] * RAD2DEG;
    o->yaw_rate_dps = a->rate_lpf[2] * RAD2DEG;
    o->lateral_g = a->latg_lpf;

    const float sig_tilt = sqrtf(f->P[ES_TH][ES_TH] > f->P[ES_TH + 1][ES_TH + 1] ? f->P[ES_TH][ES_TH] : f->P[ES_TH + 1][ES_TH + 1]);
    if (sig_tilt < 5.0f * DEG2RAD) o->valid |= AHRS_OUT_ATTITUDE | AHRS_OUT_RATES;
    const float var_hdg = f->P[ES_TH + 2][ES_TH + 2] + f->P[ES_DEC][ES_DEC] - 2.0f * f->P[ES_TH + 2][ES_DEC];
    const int mag_recent = a->t - a->t_mag_ok < MAG_TIMEOUT_S;
    if ((o->valid & AHRS_OUT_ATTITUDE) && var_hdg < sq(4.0f * DEG2RAD) &&
        (mag_recent || f->P[ES_DEC][ES_DEC] < sq(2.0f * DEG2RAD)))
        o->valid |= AHRS_OUT_HEADING;

    if (a->have_baro && a->t - a->t_baro < BARO_TIMEOUT_S) {
        o->pressure_alt_m = -f->p[2] + f->bbaro;
        o->static_pressure_hpa = a->baro_p * 0.01f;
        o->climb_ms = -f->v[2];
        o->valid |= AHRS_OUT_ALTITUDE | AHRS_OUT_CLIMB;
    } else if (a->gnss == GNSS_OK) {
        o->climb_ms = -f->v[2];
        o->valid |= AHRS_OUT_CLIMB;
    }
    if (a->t - a->t_airspeed < AIRSPEED_TIMEOUT_S) {
        o->ias_ms = a->ias;
        o->valid |= AHRS_OUT_IAS;
    }

    const ubx_pvt_t* p = &a->pvt;
    if (a->gnss == GNSS_OK) {
        o->lat_deg = p->lat_deg;
        o->lon_deg = p->lon_deg;
        o->height_m = p->height_m;
        o->gs_kmh = p->g_speed_ms * 3.6f;
        o->track_deg = wrap_360(p->head_mot_deg);
        o->gps_mode = p->fix_type >= 3 ? 2 : (p->fix_type == 2 ? 1 : 0);
        o->sats = p->num_sv;
        o->valid |= AHRS_OUT_GNSS;
    }
    if (a->ever_fused && f->P[ES_DEC][ES_DEC] < sq(2.0f * DEG2RAD)) {
        o->variation_deg = f->dec * RAD2DEG;
        o->valid |= AHRS_OUT_VARIATION;
    }
    if (a->t_gnss_time > 0.0 && a->t - a->t_gnss_time < 2.0) {
        o->year = p->year;
        o->month = p->month;
        o->day = p->day;
        o->hour = p->hour;
        o->min = p->min;
        o->sec = p->sec;
        o->valid |= AHRS_OUT_TIME;
    }
}
