/*
 * ahrs.h - sensor fusion front end: alignment, GNSS supervision (outage / recovery),
 * magnetometer calibration and gating, pseudo measurements, output for the EFIS.
 *
 * Hardware independent; the firmware feeds body-frame samples, the host tests feed a simulation.
 */
#ifndef AHRS_H
#define AHRS_H

#include <stdint.h>

#include "eskf.h"
#include "magcal.h"
#include "ubx.h"

typedef enum { GNSS_NONE = 0, GNSS_OK, GNSS_COAST, GNSS_LOST } gnss_state_t;

typedef struct {
    float dec0_deg;          /* initial magnetic declination (east +): last stored value or 0 */
    float dec0_sigma_deg;    /* its uncertainty (large when unknown) */
    float gnss_timeout_s;    /* no valid PVT for this long -> coast */
    float gnss_coast_s;      /* pure inertial coasting before the GNSS-denied mode */
    float gnss_reset_s;      /* outage longer than this -> reset position/velocity on recovery */
    float mag_sigma_deg;     /* magnetic heading measurement noise */
    float mag_rate_hz;       /* magnetic heading updates per second */
    float alpha0_deg;        /* typical cruise angle of attack until learned with GNSS */
    eskf_noise_t noise;
} ahrs_config_t;

#define AHRS_OUT_ATTITUDE (1u << 0)
#define AHRS_OUT_HEADING (1u << 1)
#define AHRS_OUT_RATES (1u << 2)   /* body rates and lateral acceleration */
#define AHRS_OUT_ALTITUDE (1u << 3)
#define AHRS_OUT_CLIMB (1u << 4)
#define AHRS_OUT_GNSS (1u << 5)
#define AHRS_OUT_VARIATION (1u << 6)
#define AHRS_OUT_TIME (1u << 7)
#define AHRS_OUT_IAS (1u << 8)

typedef struct {
    uint32_t valid;
    float roll_deg, pitch_deg, heading_mag_deg, heading_true_deg;
    float lateral_g, pitch_rate_dps, yaw_rate_dps;
    float pressure_alt_m, climb_ms, static_pressure_hpa;
    float ias_ms;
    double lat_deg, lon_deg;
    float height_m, gs_kmh, track_deg;
    int16_t gps_mode;
    uint8_t sats;
    float variation_deg;
    uint16_t year;
    uint8_t month, day, hour, min, sec;
    gnss_state_t gnss_state;
} ahrs_out_t;

typedef struct {
    ahrs_config_t cfg;
    eskf_t kf;
    magcal_t mc;
    int aligned;
    double t;

    /* alignment */
    float acc_sum[3], gyro_sum[3], gyro_max;
    uint32_t align_n;

    /* last corrected IMU sample, filtered outputs */
    float gyro_c[3], acc_c[3];
    float rate_lpf[3], latg_lpf;
    float gyro_act, acc_act; /* activity measures for the stationary detector */
    int stationary;
    float f_anchor[3]; /* specific force direction when the stationary period began */
    float acc_lpf[3];

    /* wind triangle from GNSS in turns: x = [horizontal airspeed, wind north, wind east] */
    float wx[3];
    float wP[3][3];
    float alpha_est; /* mean angle of attack (body x vs air-relative velocity), rad */
    double t_pseudo;

    /* magnetometer */
    float mag_cal[3];
    float mag_norm_lpf, mag_dip_lpf;
    double t_mag_upd, t_mag_ok, t_mag_aided;
    int have_mag;
    int magcal_changed;
    uint32_t mag_rejects;

    /* barometer */
    float baro_alt, baro_p;
    double t_baro;
    int have_baro;

    /* declination from the display */
    float ext_dec;
    double t_ext_dec;
    int have_ext_dec;

    /* airspeed (optional) */
    float ias, tas;
    double t_airspeed;

    /* GNSS */
    gnss_state_t gnss;
    ubx_pvt_t pvt;     /* latest PVT (time) */
    ubx_pvt_t pvt_fix; /* latest PVT with a valid fix (position output) */
    double t_gnss_ok, t_gnss_time;
    double lat0, lon0;
    int have_origin, ever_fused, ever_moved;
    uint32_t gnss_rejects;
    float last_gs;
} ahrs_t;

void ahrs_default_config(ahrs_config_t* c);
void ahrs_init(ahrs_t* a, const ahrs_config_t* cfg);
void ahrs_set_magcal(ahrs_t* a, const magcal_params_t* p);

void ahrs_imu(ahrs_t* a, const float gyro[3], const float acc[3], float dt); /* rad/s, m/s^2 */
void ahrs_mag(ahrs_t* a, const float raw_ut[3]);
void ahrs_baro(ahrs_t* a, float p_pa);
void ahrs_gnss(ahrs_t* a, const ubx_pvt_t* pvt);
void ahrs_airspeed(ahrs_t* a, float ias_ms, float tas_ms);
/* Magnetic variation from the display (WMM there), deg east positive. */
void ahrs_declination(ahrs_t* a, float dec_deg);

void ahrs_output(const ahrs_t* a, ahrs_out_t* o);
/* Returns 1 once after the magnetometer calibration changed (to store it). */
int ahrs_take_magcal_changed(ahrs_t* a);

#endif
