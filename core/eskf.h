/*
 * eskf.h - error-state extended Kalman filter (strapdown INS + aiding).
 *
 * Nominal state: position p (NED, m), velocity v (NED, m/s), attitude q (body->NED),
 * accelerometer bias, gyro bias, barometer bias and magnetic declination.
 * Error state (17): dp, dv, dtheta (body frame, local), dba, dbg, dbbaro, ddec.
 *
 * All measurement updates are scalar (sequential), so no matrix inversion is needed.
 */
#ifndef ESKF_H
#define ESKF_H

#include "ahrs_math.h"

#define ESKF_N 17
enum { ES_P = 0, ES_V = 3, ES_TH = 6, ES_BA = 9, ES_BG = 12, ES_BB = 15, ES_DEC = 16 };

typedef struct {
    float acc_noise;     /* accelerometer white noise, m/s^2/sqrt(Hz) */
    float gyro_noise;    /* gyro white noise, rad/s/sqrt(Hz) */
    float acc_bias_rw;   /* accelerometer bias random walk, m/s^3/sqrt(Hz) */
    float gyro_bias_rw;  /* gyro bias random walk, rad/s^2/sqrt(Hz) */
    float baro_bias_rw;  /* barometer bias random walk, m/s/sqrt(Hz) */
    float dec_rw;        /* declination random walk, rad/s/sqrt(Hz) */
    float acc_bias_max;  /* clamp, m/s^2 */
    float gyro_bias_max; /* clamp, rad/s */
} eskf_noise_t;

typedef struct {
    float p[3], v[3];
    quat_t q;
    float ba[3], bg[3];
    float bbaro; /* baro altitude - geometric altitude (m) */
    float dec;   /* magnetic declination, rad, east positive */
    float P[ESKF_N][ESKF_N];
    eskf_noise_t n;
    float R[3][3]; /* cached DCM of q */
    float last_nis;
} eskf_t;

void eskf_init(eskf_t* f, const eskf_noise_t* n, quat_t q0, float sig_tilt, float sig_yaw, float dec0, float sig_dec);

/* Strapdown propagation with bias-free body rates/specific force still containing biases. */
void eskf_predict(eskf_t* f, const float gyro[3], const float acc[3], float dt);

/*
 * Generic scalar update: innov = z - h(x), H = dense Jacobian row, R = variance.
 * gate: reject when innov^2 / S > gate (0 = no gate). Returns 1 when applied.
 * *nis receives the normalised innovation squared (may be NULL).
 */
int eskf_update(eskf_t* f, float innov, const float H[ESKF_N], float R, float gate, float* nis);

/* Schmidt ("consider") update: states >= first_consider are neither corrected nor learn from it.
 * Used for approximate pseudo measurements that must not bias the sensor error estimates. */
int eskf_update_consider(eskf_t* f, float innov, const float H[ESKF_N], float R, float gate, int first_consider);

int eskf_update_pos(eskf_t* f, int axis, float z, float R, float gate);
int eskf_update_vel(eskf_t* f, int axis, float z, float R, float gate);
int eskf_update_baro(eskf_t* f, float alt, float R, float gate);
/* Magnetic heading measurement psi_m (rad): h(x) = yaw(q) - dec. */
int eskf_update_mag_heading(eskf_t* f, float psi_m, float R, float gate);
/* Body-frame component of the air-relative velocity R^T (v - wind) (pseudo measurement,
 * e.g. zero sideslip). wind may be NULL. */
int eskf_update_body_vel(eskf_t* f, int axis, float z, float R, float gate, int first_consider, const float wind[3]);

/* Re-initialise position/velocity (e.g. after a long GNSS outage). */
void eskf_reset_pos(eskf_t* f, const float p[3], float sig);
void eskf_reset_vel(eskf_t* f, const float v[3], float sig);
void eskf_reset_baro_bias(eskf_t* f, float bbaro, float sig);

static inline float eskf_yaw(const eskf_t* f) { return atan2f(f->R[1][0], f->R[0][0]); }

#endif
