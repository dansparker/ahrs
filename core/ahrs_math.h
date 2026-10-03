/*
 * ahrs_math.h - small vector, matrix and quaternion helpers (single precision).
 *
 * Conventions: NED navigation frame, body frame x forward / y right / z down.
 * Quaternion q rotates body vectors into NED: v_n = R(q) v_b.
 */
#ifndef AHRS_MATH_H
#define AHRS_MATH_H

#include <math.h>

#define AHRS_PI 3.14159265358979f
#define DEG2RAD (AHRS_PI / 180.0f)
#define RAD2DEG (180.0f / AHRS_PI)
#define GRAVITY 9.80665f

typedef struct {
    float w, x, y, z;
} quat_t;

static inline float sq(float a) { return a * a; }

static inline float v3_dot(const float a[3], const float b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

static inline float v3_norm(const float a[3]) { return sqrtf(v3_dot(a, a)); }

static inline void v3_cross(const float a[3], const float b[3], float out[3]) {
    const float x = a[1] * b[2] - a[2] * b[1];
    const float y = a[2] * b[0] - a[0] * b[2];
    const float z = a[0] * b[1] - a[1] * b[0];
    out[0] = x;
    out[1] = y;
    out[2] = z;
}

/* out = M v */
static inline void m3_mul_v(const float M[3][3], const float v[3], float out[3]) {
    float r[3];
    for (int i = 0; i < 3; ++i) r[i] = M[i][0] * v[0] + M[i][1] * v[1] + M[i][2] * v[2];
    out[0] = r[0];
    out[1] = r[1];
    out[2] = r[2];
}

/* out = M^T v */
static inline void m3t_mul_v(const float M[3][3], const float v[3], float out[3]) {
    float r[3];
    for (int i = 0; i < 3; ++i) r[i] = M[0][i] * v[0] + M[1][i] * v[1] + M[2][i] * v[2];
    out[0] = r[0];
    out[1] = r[1];
    out[2] = r[2];
}

static inline quat_t quat_mul(quat_t a, quat_t b) {
    quat_t r;
    r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return r;
}

static inline quat_t quat_normalize(quat_t q) {
    float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n < 1e-9f) {
        quat_t id = {1.0f, 0.0f, 0.0f, 0.0f};
        return id;
    }
    if (q.w < 0.0f) n = -n; /* keep w >= 0 */
    q.w /= n;
    q.x /= n;
    q.y /= n;
    q.z /= n;
    return q;
}

/* Quaternion of a rotation vector (axis * angle). */
static inline quat_t quat_from_rotvec(const float r[3]) {
    const float a = v3_norm(r);
    quat_t q;
    if (a < 1e-6f) {
        q.w = 1.0f;
        q.x = 0.5f * r[0];
        q.y = 0.5f * r[1];
        q.z = 0.5f * r[2];
        return quat_normalize(q);
    }
    const float s = sinf(0.5f * a) / a;
    q.w = cosf(0.5f * a);
    q.x = r[0] * s;
    q.y = r[1] * s;
    q.z = r[2] * s;
    return q;
}

static inline void quat_to_dcm(quat_t q, float R[3][3]) {
    const float ww = q.w * q.w, xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    R[0][0] = ww + xx - yy - zz;
    R[0][1] = 2.0f * (q.x * q.y - q.w * q.z);
    R[0][2] = 2.0f * (q.x * q.z + q.w * q.y);
    R[1][0] = 2.0f * (q.x * q.y + q.w * q.z);
    R[1][1] = ww - xx + yy - zz;
    R[1][2] = 2.0f * (q.y * q.z - q.w * q.x);
    R[2][0] = 2.0f * (q.x * q.z - q.w * q.y);
    R[2][1] = 2.0f * (q.y * q.z + q.w * q.x);
    R[2][2] = ww - xx - yy + zz;
}

/* ZYX Euler angles (yaw, pitch, roll) in rad. */
static inline quat_t quat_from_euler(float roll, float pitch, float yaw) {
    const float cr = cosf(0.5f * roll), sr = sinf(0.5f * roll);
    const float cp = cosf(0.5f * pitch), sp = sinf(0.5f * pitch);
    const float cy = cosf(0.5f * yaw), sy = sinf(0.5f * yaw);
    quat_t q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return quat_normalize(q);
}

static inline void dcm_to_euler(const float R[3][3], float* roll, float* pitch, float* yaw) {
    *roll = atan2f(R[2][1], R[2][2]);
    float s = -R[2][0];
    if (s > 1.0f) s = 1.0f;
    if (s < -1.0f) s = -1.0f;
    *pitch = asinf(s);
    *yaw = atan2f(R[1][0], R[0][0]);
}

static inline float wrap_pi(float a) {
    while (a > AHRS_PI) a -= 2.0f * AHRS_PI;
    while (a < -AHRS_PI) a += 2.0f * AHRS_PI;
    return a;
}

static inline float wrap_360(float deg) {
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f) deg += 360.0f;
    return deg;
}

#endif
