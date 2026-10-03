#include "eskf.h"

#include <string.h>

static void refresh_dcm(eskf_t* f) { quat_to_dcm(f->q, f->R); }

static void symmetrize(eskf_t* f) {
    for (int i = 0; i < ESKF_N; ++i) {
        for (int j = i + 1; j < ESKF_N; ++j) {
            const float m = 0.5f * (f->P[i][j] + f->P[j][i]);
            f->P[i][j] = m;
            f->P[j][i] = m;
        }
        if (f->P[i][i] < 1e-12f) f->P[i][i] = 1e-12f;
    }
}

static float clampf(float x, float lim) { return x > lim ? lim : (x < -lim ? -lim : x); }

void eskf_init(eskf_t* f, const eskf_noise_t* n, quat_t q0, float sig_tilt, float sig_yaw, float dec0, float sig_dec) {
    memset(f, 0, sizeof(*f));
    f->n = *n;
    f->q = quat_normalize(q0);
    f->dec = dec0;
    refresh_dcm(f);
    for (int i = 0; i < 3; ++i) {
        f->P[ES_P + i][ES_P + i] = sq(100.0f);
        f->P[ES_V + i][ES_V + i] = sq(5.0f);
        f->P[ES_BA + i][ES_BA + i] = sq(0.3f);
        f->P[ES_BG + i][ES_BG + i] = sq(1.0f * DEG2RAD);
    }
    f->P[ES_TH + 0][ES_TH + 0] = sq(sig_tilt);
    f->P[ES_TH + 1][ES_TH + 1] = sq(sig_tilt);
    f->P[ES_TH + 2][ES_TH + 2] = sq(sig_yaw);
    /* small on purpose: without GNSS bbaro and p_d are only observable as a difference, and a large
     * common variance would make P ill-conditioned in single precision (reset at the first fix) */
    f->P[ES_BB][ES_BB] = sq(1.0f);
    f->P[ES_DEC][ES_DEC] = sq(sig_dec);
}

void eskf_predict(eskf_t* f, const float gyro[3], const float acc[3], float dt) {
    static float F[ESKF_N][ESKF_N];
    static float T[ESKF_N][ESKF_N];
    float w[3], a[3], an[3];
    for (int i = 0; i < 3; ++i) {
        w[i] = gyro[i] - f->bg[i];
        a[i] = acc[i] - f->ba[i];
    }
    const float (*R)[3] = (const float (*)[3])f->R;

    /* error-state transition F = I + A dt (built with the attitude at the start of the step) */
    memset(F, 0, sizeof(F));
    for (int i = 0; i < ESKF_N; ++i) F[i][i] = 1.0f;
    for (int i = 0; i < 3; ++i) {
        F[ES_P + i][ES_V + i] = dt;
        F[ES_TH + i][ES_BG + i] = -dt;
        for (int j = 0; j < 3; ++j) F[ES_V + i][ES_BA + j] = -R[i][j] * dt;
    }
    /* dv/dtheta = -R [a]x */
    const float ax[3][3] = {{0.0f, -a[2], a[1]}, {a[2], 0.0f, -a[0]}, {-a[1], a[0], 0.0f}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            F[ES_V + i][ES_TH + j] = -(R[i][0] * ax[0][j] + R[i][1] * ax[1][j] + R[i][2] * ax[2][j]) * dt;
    /* dtheta/dtheta = I - [w]x dt */
    F[ES_TH + 0][ES_TH + 1] = w[2] * dt;
    F[ES_TH + 0][ES_TH + 2] = -w[1] * dt;
    F[ES_TH + 1][ES_TH + 0] = -w[2] * dt;
    F[ES_TH + 1][ES_TH + 2] = w[0] * dt;
    F[ES_TH + 2][ES_TH + 0] = w[1] * dt;
    F[ES_TH + 2][ES_TH + 1] = -w[0] * dt;

    /* nominal state */
    m3_mul_v(f->R, a, an);
    an[2] += GRAVITY;
    for (int i = 0; i < 3; ++i) {
        f->p[i] += f->v[i] * dt + 0.5f * an[i] * dt * dt;
        f->v[i] += an[i] * dt;
    }
    const float rv[3] = {w[0] * dt, w[1] * dt, w[2] * dt};
    f->q = quat_normalize(quat_mul(f->q, quat_from_rotvec(rv)));
    refresh_dcm(f);

    /* P = F P F^T + Q */
    for (int i = 0; i < ESKF_N; ++i)
        for (int j = 0; j < ESKF_N; ++j) {
            float s = 0.0f;
            for (int k = 0; k < ESKF_N; ++k)
                if (F[i][k] != 0.0f) s += F[i][k] * f->P[k][j];
            T[i][j] = s;
        }
    for (int i = 0; i < ESKF_N; ++i)
        for (int j = i; j < ESKF_N; ++j) {
            float s = 0.0f;
            for (int k = 0; k < ESKF_N; ++k)
                if (F[j][k] != 0.0f) s += T[i][k] * F[j][k];
            f->P[i][j] = s;
            f->P[j][i] = s;
        }
    const eskf_noise_t* n = &f->n;
    for (int i = 0; i < 3; ++i) {
        f->P[ES_V + i][ES_V + i] += sq(n->acc_noise) * dt;
        f->P[ES_TH + i][ES_TH + i] += sq(n->gyro_noise) * dt;
        f->P[ES_BA + i][ES_BA + i] += sq(n->acc_bias_rw) * dt;
        f->P[ES_BG + i][ES_BG + i] += sq(n->gyro_bias_rw) * dt;
    }
    f->P[ES_BB][ES_BB] += sq(n->baro_bias_rw) * dt;
    f->P[ES_DEC][ES_DEC] += sq(n->dec_rw) * dt;
    /* bound the variance of unobserved states (horizontal position without GNSS) */
    for (int i = ES_P; i < ES_P + 2; ++i) {
        if (f->P[i][i] > 1e8f) {
            const float k = sqrtf(1e8f / f->P[i][i]);
            for (int j = 0; j < ESKF_N; ++j) {
                f->P[i][j] *= k;
                f->P[j][i] *= k;
            }
        }
    }
    symmetrize(f);
}

static void inject(eskf_t* f, const float dx[ESKF_N]) {
    for (int i = 0; i < 3; ++i) {
        f->p[i] += dx[ES_P + i];
        f->v[i] += dx[ES_V + i];
        f->ba[i] = clampf(f->ba[i] + dx[ES_BA + i], f->n.acc_bias_max);
        f->bg[i] = clampf(f->bg[i] + dx[ES_BG + i], f->n.gyro_bias_max);
    }
    f->q = quat_normalize(quat_mul(f->q, quat_from_rotvec(&dx[ES_TH])));
    f->bbaro += dx[ES_BB];
    f->dec = wrap_pi(f->dec + dx[ES_DEC]);
    refresh_dcm(f);
}

int eskf_update(eskf_t* f, float innov, const float H[ESKF_N], float R, float gate, float* nis) {
    const int ok = eskf_update_consider(f, innov, H, R, gate, 0u);
    if (nis) *nis = f->last_nis;
    return ok;
}

int eskf_update_consider(eskf_t* f, float innov, const float H[ESKF_N], float R, float gate, uint32_t cm) {
    float PHt[ESKF_N], dx[ESKF_N];
    float S = R;
    for (int i = 0; i < ESKF_N; ++i) {
        float s = 0.0f;
        for (int j = 0; j < ESKF_N; ++j)
            if (H[j] != 0.0f) s += f->P[i][j] * H[j];
        PHt[i] = s;
        S += H[i] * s;
    }
    if (!(S > 0.0f) || !isfinite(innov)) return 0;
    const float d2 = innov * innov / S;
    f->last_nis = d2;
    if (gate > 0.0f && d2 > gate) return 0;
    for (int i = 0; i < ESKF_N; ++i) {
        const int ci = (cm >> i) & 1u;
        dx[i] = ci ? 0.0f : PHt[i] / S * innov;
        for (int j = 0; j < ESKF_N; ++j)
            if (!ci || !((cm >> j) & 1u)) f->P[i][j] -= PHt[i] * PHt[j] / S;
    }
    symmetrize(f);
    inject(f, dx);
    return 1;
}

int eskf_update_pos(eskf_t* f, int axis, float z, float R, float gate) {
    float H[ESKF_N] = {0};
    H[ES_P + axis] = 1.0f;
    return eskf_update(f, z - f->p[axis], H, R, gate, 0);
}

int eskf_update_vel(eskf_t* f, int axis, float z, float R, float gate) {
    float H[ESKF_N] = {0};
    H[ES_V + axis] = 1.0f;
    return eskf_update(f, z - f->v[axis], H, R, gate, 0);
}

int eskf_update_baro(eskf_t* f, float alt, float R, float gate) {
    float H[ESKF_N] = {0};
    H[ES_P + 2] = -1.0f;
    H[ES_BB] = 1.0f;
    return eskf_update(f, alt - (-f->p[2] + f->bbaro), H, R, gate, 0);
}

int eskf_update_mag_heading(eskf_t* f, float psi_m, float R, float gate) {
    const float (*M)[3] = (const float (*)[3])f->R;
    const float d = sq(M[0][0]) + sq(M[1][0]);
    if (d < 1e-4f) return 0; /* pitch near +-90 deg: heading undefined */
    float H[ESKF_N] = {0};
    H[ES_TH + 1] = (M[1][0] * M[0][2] - M[0][0] * M[1][2]) / d;
    H[ES_TH + 2] = (M[0][0] * M[1][1] - M[1][0] * M[0][1]) / d;
    H[ES_DEC] = -1.0f;
    const float innov = wrap_pi(psi_m - (eskf_yaw(f) - f->dec));
    return eskf_update(f, innov, H, R, gate, 0);
}

int eskf_update_body_vel(eskf_t* f, int axis, float z, float R, float gate, uint32_t cm, const float wind[3]) {
    float vb[3], va[3];
    for (int i = 0; i < 3; ++i) va[i] = f->v[i] - (wind ? wind[i] : 0.0f);
    m3t_mul_v(f->R, va, vb);
    float H[ESKF_N] = {0};
    for (int j = 0; j < 3; ++j) H[ES_V + j] = f->R[j][axis];
    /* d(vb)/d(theta) = [vb]x */
    const float vx[3][3] = {{0.0f, -vb[2], vb[1]}, {vb[2], 0.0f, -vb[0]}, {-vb[1], vb[0], 0.0f}};
    for (int j = 0; j < 3; ++j) H[ES_TH + j] = vx[axis][j];
    return eskf_update_consider(f, z - vb[axis], H, R, gate, cm);
}

static void reset_block(eskf_t* f, int base, int len, float sig) {
    for (int i = base; i < base + len; ++i) {
        for (int j = 0; j < ESKF_N; ++j) {
            f->P[i][j] = 0.0f;
            f->P[j][i] = 0.0f;
        }
        f->P[i][i] = sig * sig;
    }
}

void eskf_reset_pos(eskf_t* f, const float p[3], float sig) {
    memcpy(f->p, p, sizeof(f->p));
    reset_block(f, ES_P, 3, sig);
}

void eskf_reset_vel(eskf_t* f, const float v[3], float sig) {
    memcpy(f->v, v, sizeof(f->v));
    reset_block(f, ES_V, 3, sig);
}

void eskf_reset_baro_bias(eskf_t* f, float bbaro, float sig) {
    f->bbaro = bbaro;
    reset_block(f, ES_BB, 1, sig);
}
