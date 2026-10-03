/*
 * magcal.h - online magnetometer calibration.
 *
 * Calibrated field: m_cal = W (m_raw - offset), W symmetric (soft iron, volume preserving),
 * offset = hard iron. Two estimators run side by side:
 *
 * 1. Ellipsoid fit by recursive least squares (RLS), as in Cao/Xu/Xu, Sensors 2020, 20, 535,
 *    using magnetometer data only. Needs 3D attitude diversity (bench or ground "swing").
 *    Samples are binned by direction (26 bins) with a quota per bin so that long straight
 *    flight cannot dominate the fit; a solution is only accepted when enough bins are covered,
 *    the ellipsoid is plausible and the residual is small.
 *
 * 2. Attitude-aided hard-iron refinement: with a good attitude from the GNSS-aided ESKF,
 *    m_cal = R^T B_ned + d_offset is linear in (B_ned, d_offset). A 6-state RLS with forgetting
 *    estimates the residual offset in flight, where heading changes alone make the horizontal
 *    offset observable (the ellipsoid fit would be ill-conditioned there).
 */
#ifndef MAGCAL_H
#define MAGCAL_H

#include <stdint.h>

#define MAGCAL_BINS 26

typedef struct {
    float offset[3]; /* raw units (uT) */
    float W[3][3];
    float radius;    /* expected |m_cal| (uT) */
    int32_t valid;
} magcal_params_t;

typedef struct {
    magcal_params_t active;

    /* ellipsoid RLS */
    double th[9];
    double P[9][9];
    uint16_t bin_count[MAGCAL_BINS];
    float last[3];
    float mean[3];
    uint32_t n_seen, n_used, n_since_solve;
    float ring[32][3]; /* recent accepted raw samples for the residual check */
    uint32_t ring_n;
    float last_residual;
    float dd[3][3]; /* sum of accepted unit directions d d^T: geometric spread */

    /* attitude-aided RLS: x = [B_ned(3), d_offset(3)] */
    float ax[6];
    float aP[6][6];
    uint32_t aided_updates;

    /* tunables */
    uint16_t bin_quota;
    uint8_t min_bins;
    float min_angle_rad;
    float max_residual;
    float aided_lambda;
} magcal_t;

void magcal_init(magcal_t* c);
/* Use a stored calibration (e.g. from flash) as the active one. */
void magcal_set(magcal_t* c, const magcal_params_t* p);
/* Feed a raw sample (uT). Returns 1 when a new ellipsoid solution became active. */
int magcal_add(magcal_t* c, const float raw[3]);
/* Feed a calibrated sample with the current body->NED DCM. Returns 1 when the offset changed. */
int magcal_aided_add(magcal_t* c, const float R[3][3], const float m_cal[3]);
void magcal_apply(const magcal_t* c, const float raw[3], float out[3]);
int magcal_covered_bins(const magcal_t* c);

/* exposed for tests */
int magcal_solve(const double th[9], float scale, magcal_params_t* out);
void sym3_eig(const float A[3][3], float eval[3], float V[3][3]);

#endif
