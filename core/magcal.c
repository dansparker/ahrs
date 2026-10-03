#include "magcal.h"

#include <math.h>
#include <string.h>

#include "ahrs_math.h"

#define MAGCAL_SCALE 50.0f /* normalisation of the RLS regressor, uT */

void sym3_eig(const float A_in[3][3], float e[3], float V[3][3]) {
    float A[3][3];
    memcpy(A, A_in, sizeof(A));
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) V[i][j] = (i == j) ? 1.0f : 0.0f;
    static const int pairs[3][2] = {{0, 1}, {0, 2}, {1, 2}};
    for (int sweep = 0; sweep < 30; ++sweep) {
        const float off = sq(A[0][1]) + sq(A[0][2]) + sq(A[1][2]);
        if (off < 1e-24f) break;
        for (int k = 0; k < 3; ++k) {
            const int p = pairs[k][0], q = pairs[k][1];
            if (fabsf(A[p][q]) < 1e-30f) continue;
            const float theta = (A[q][q] - A[p][p]) / (2.0f * A[p][q]);
            const float t = (theta >= 0.0f ? 1.0f : -1.0f) / (fabsf(theta) + sqrtf(theta * theta + 1.0f));
            const float c = 1.0f / sqrtf(t * t + 1.0f), s = t * c;
            for (int i = 0; i < 3; ++i) {
                const float aip = A[i][p], aiq = A[i][q];
                A[i][p] = c * aip - s * aiq;
                A[i][q] = s * aip + c * aiq;
            }
            for (int i = 0; i < 3; ++i) {
                const float api = A[p][i], aqi = A[q][i];
                A[p][i] = c * api - s * aqi;
                A[q][i] = s * api + c * aqi;
            }
            for (int i = 0; i < 3; ++i) {
                const float vip = V[i][p], viq = V[i][q];
                V[i][p] = c * vip - s * viq;
                V[i][q] = s * vip + c * viq;
            }
        }
    }
    for (int i = 0; i < 3; ++i) e[i] = A[i][i];
}

static void reset_aided(magcal_t* c) {
    memset(c->ax, 0, sizeof(c->ax));
    memset(c->aP, 0, sizeof(c->aP));
    for (int i = 0; i < 3; ++i) {
        c->aP[i][i] = sq(60.0f);
        c->aP[3 + i][3 + i] = sq(10.0f);
    }
    c->aided_updates = 0;
}

void magcal_init(magcal_t* c) {
    memset(c, 0, sizeof(*c));
    for (int i = 0; i < 3; ++i) c->active.W[i][i] = 1.0f;
    c->active.radius = 50.0f;
    for (int i = 0; i < 9; ++i) c->P[i][i] = 100.0;
    c->bin_quota = 40;
    c->min_bins = 14;
    c->min_angle_rad = 4.0f * DEG2RAD;
    c->max_residual = 0.03f;
    c->aided_lambda = 0.9995f;
    reset_aided(c);
}

void magcal_set(magcal_t* c, const magcal_params_t* p) {
    c->active = *p;
    reset_aided(c);
}

void magcal_apply(const magcal_t* c, const float raw[3], float out[3]) {
    const float d[3] = {raw[0] - c->active.offset[0], raw[1] - c->active.offset[1], raw[2] - c->active.offset[2]};
    m3_mul_v(c->active.W, d, out);
}

static int dir_bin(const float d[3], float n) {
    int idx = 0;
    for (int i = 0; i < 3; ++i) {
        const float u = d[i] / n;
        const int q = (u > 0.38f) - (u < -0.38f);
        idx = idx * 3 + (q + 1);
    }
    return idx > 13 ? idx - 1 : idx; /* 13 = (0,0,0) cannot occur for a unit vector */
}

int magcal_covered_bins(const magcal_t* c) {
    int n = 0;
    for (int i = 0; i < MAGCAL_BINS; ++i) n += c->bin_count[i] > 0;
    return n;
}

int magcal_solve(const double th[9], float scale, magcal_params_t* out) {
    /* quadric x^2 + b y^2 + c z^2 + 2d xy + 2e xz + 2f yz + 2g x + 2h y + 2i z + j = 0 (x^2 coefficient
     * fixed to 1: always well defined, also when the ellipsoid passes near the origin) */
    const float A[3][3] = {{1.0f, (float)th[2], (float)th[3]},
                           {(float)th[2], (float)th[0], (float)th[4]},
                           {(float)th[3], (float)th[4], (float)th[1]}};
    const float g[3] = {(float)th[5], (float)th[6], (float)th[7]};
    const float jc = (float)th[8];
    float e[3], V[3][3];
    sym3_eig(A, e, V);
    for (int i = 0; i < 3; ++i)
        if (!(e[i] > 1e-6f)) return 0;
    /* centre o = -A^-1 g */
    float o[3] = {0, 0, 0};
    for (int k = 0; k < 3; ++k) {
        const float proj = (V[0][k] * g[0] + V[1][k] * g[1] + V[2][k] * g[2]) / e[k];
        for (int i = 0; i < 3; ++i) o[i] -= V[i][k] * proj;
    }
    float Ao[3];
    m3_mul_v(A, o, Ao);
    const float kk = v3_dot(o, Ao) - jc;
    if (!(kk > 1e-6f)) return 0;
    float s[3], emax = 0.0f, emin = 1e30f;
    for (int i = 0; i < 3; ++i) {
        const float em = e[i] / kk;
        emax = em > emax ? em : emax;
        emin = em < emin ? em : emin;
        s[i] = sqrtf(em);
    }
    if (emax / emin > 4.0f) return 0; /* axis ratio > 2: implausible */
    const float r_u = 1.0f / cbrtf(s[0] * s[1] * s[2]);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            out->W[i][j] = r_u * (V[i][0] * s[0] * V[j][0] + V[i][1] * s[1] * V[j][1] + V[i][2] * s[2] * V[j][2]);
    for (int i = 0; i < 3; ++i) out->offset[i] = o[i] * scale;
    out->radius = r_u * scale;
    out->valid = 1;
    if (v3_norm(out->offset) > 300.0f || out->radius < 10.0f || out->radius > 120.0f) return 0;
    return 1;
}

static void rls9(magcal_t* c, const float raw[3]) {
    const double x = raw[0] / MAGCAL_SCALE, y = raw[1] / MAGCAL_SCALE, z = raw[2] / MAGCAL_SCALE;
    /* target x^2, regressor of the remaining quadric terms */
    const double phi[9] = {-y * y, -z * z, -2 * x * y, -2 * x * z, -2 * y * z, -2 * x, -2 * y, -2 * z, -1.0};
    double Pphi[9], den = 1.0, pred = 0.0;
    for (int i = 0; i < 9; ++i) {
        double s = 0.0;
        for (int k = 0; k < 9; ++k) s += c->P[i][k] * phi[k];
        Pphi[i] = s;
        den += phi[i] * s;
        pred += phi[i] * c->th[i];
    }
    const double err = x * x - pred;
    for (int i = 0; i < 9; ++i) {
        c->th[i] += Pphi[i] / den * err;
        for (int k = 0; k < 9; ++k) c->P[i][k] -= Pphi[i] * Pphi[k] / den;
    }
}

static float residual(const magcal_t* c, const magcal_params_t* p) {
    const uint32_t n = c->ring_n < 32 ? c->ring_n : 32;
    if (n == 0) return 1.0f;
    float acc = 0.0f;
    for (uint32_t k = 0; k < n; ++k) {
        float d[3], m[3];
        for (int i = 0; i < 3; ++i) d[i] = c->ring[k][i] - p->offset[i];
        m3_mul_v(p->W, d, m);
        acc += sq(v3_norm(m) / p->radius - 1.0f);
    }
    return sqrtf(acc / (float)n);
}

int magcal_add(magcal_t* c, const float raw[3]) {
    c->n_seen++;
    const float w = 1.0f / (float)(c->n_seen < 500 ? c->n_seen : 500);
    for (int i = 0; i < 3; ++i) c->mean[i] += (raw[i] - c->mean[i]) * w;

    const float* ctr = c->active.valid ? c->active.offset : c->mean;
    const float d[3] = {raw[0] - ctr[0], raw[1] - ctr[1], raw[2] - ctr[2]};
    const float nd = v3_norm(d);
    if (nd < 5.0f) return 0;
    if (c->n_used > 0) {
        const float dl[3] = {c->last[0] - ctr[0], c->last[1] - ctr[1], c->last[2] - ctr[2]};
        const float nl = v3_norm(dl);
        if (nl > 1e-3f && v3_dot(d, dl) / (nd * nl) > cosf(c->min_angle_rad)) return 0;
    }
    const int b = dir_bin(d, nd);
    if (c->bin_count[b] >= c->bin_quota) return 0;
    c->bin_count[b]++;
    memcpy(c->last, raw, sizeof(c->last));
    memcpy(c->ring[c->ring_n % 32], raw, sizeof(c->ring[0]));
    c->ring_n++;
    c->n_used++;
    rls9(c, raw);

    if (++c->n_since_solve < 20 || magcal_covered_bins(c) < c->min_bins) return 0;
    c->n_since_solve = 0;
    magcal_params_t cand;
    if (!magcal_solve(c->th, MAGCAL_SCALE, &cand)) return 0;
    c->last_residual = residual(c, &cand);
    if (c->last_residual > c->max_residual) return 0;
    magcal_set(c, &cand);
    return 1;
}

static int inv3(const float M[3][3], float out[3][3]) {
    const float det = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
                      M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
                      M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    if (fabsf(det) < 1e-9f) return 0;
    const float id = 1.0f / det;
    out[0][0] = (M[1][1] * M[2][2] - M[1][2] * M[2][1]) * id;
    out[0][1] = (M[0][2] * M[2][1] - M[0][1] * M[2][2]) * id;
    out[0][2] = (M[0][1] * M[1][2] - M[0][2] * M[1][1]) * id;
    out[1][0] = (M[1][2] * M[2][0] - M[1][0] * M[2][2]) * id;
    out[1][1] = (M[0][0] * M[2][2] - M[0][2] * M[2][0]) * id;
    out[1][2] = (M[0][2] * M[1][0] - M[0][0] * M[1][2]) * id;
    out[2][0] = (M[1][0] * M[2][1] - M[1][1] * M[2][0]) * id;
    out[2][1] = (M[0][1] * M[2][0] - M[0][0] * M[2][1]) * id;
    out[2][2] = (M[0][0] * M[1][1] - M[0][1] * M[1][0]) * id;
    return 1;
}

int magcal_aided_add(magcal_t* c, const float R[3][3], const float m_cal[3]) {
    if (c->aided_updates == 0) {
        float b[3];
        m3_mul_v(R, m_cal, b);
        for (int i = 0; i < 3; ++i) c->ax[i] = b[i];
    }
    /* forgetting once per sample */
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j) c->aP[i][j] /= c->aided_lambda;
    for (int axis = 0; axis < 3; ++axis) {
        float h[6] = {R[0][axis], R[1][axis], R[2][axis], 0, 0, 0};
        h[3 + axis] = 1.0f;
        float Ph[6], den = sq(0.5f), pred = 0.0f; /* 0.5 uT measurement noise */
        for (int i = 0; i < 6; ++i) {
            float s = 0.0f;
            for (int j = 0; j < 6; ++j) s += c->aP[i][j] * h[j];
            Ph[i] = s;
            den += h[i] * s;
            pred += h[i] * c->ax[i];
        }
        const float err = m_cal[axis] - pred;
        for (int i = 0; i < 6; ++i) {
            c->ax[i] += Ph[i] / den * err;
            for (int j = 0; j < 6; ++j) c->aP[i][j] -= Ph[i] * Ph[j] / den;
        }
    }
    /* bound the covariance of unobservable directions (forgetting would let it grow) */
    for (int i = 0; i < 6; ++i) {
        const float lim = i < 3 ? sq(60.0f) : sq(10.0f);
        if (c->aP[i][i] > lim) {
            const float k = sqrtf(lim / c->aP[i][i]);
            for (int j = 0; j < 6; ++j) {
                c->aP[i][j] *= k;
                c->aP[j][i] *= k;
            }
        }
    }
    c->aided_updates++;
    if (c->aided_updates < 200) return 0;

    float dcal[3] = {0, 0, 0};
    int any = 0;
    for (int i = 0; i < 3; ++i) {
        if (c->aP[3 + i][3 + i] < sq(0.7f) && fabsf(c->ax[3 + i]) > 0.5f) {
            dcal[i] = c->ax[3 + i];
            any = 1;
        }
    }
    if (!any) return 0;
    float Wi[3][3], draw[3];
    if (!inv3(c->active.W, Wi)) return 0;
    m3_mul_v(Wi, dcal, draw);
    for (int i = 0; i < 3; ++i) {
        c->active.offset[i] += draw[i];
        c->ax[3 + i] -= dcal[i];
    }
    c->active.valid = 1;
    return 1;
}
