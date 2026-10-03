#include "can_out.h"

unsigned can_out_build(canas_tx_t* tx, const ahrs_out_t* o, unsigned groups, canas_frame_t* fr) {
    unsigned n = 0;
    const uint32_t v = o->valid;
    if (groups & CAN_OUT_FAST) {
        if (v & AHRS_OUT_ATTITUDE) {
            canas_float(tx, &fr[n++], CANAS_ID_PITCH, o->pitch_deg);
            canas_float(tx, &fr[n++], CANAS_ID_ROLL, o->roll_deg);
        }
        if (v & AHRS_OUT_HEADING) canas_float(tx, &fr[n++], CANAS_ID_MAG_HEADING, o->heading_mag_deg);
        if (v & AHRS_OUT_RATES) {
            canas_float(tx, &fr[n++], CANAS_ID_LATERAL_ACCEL, o->lateral_g);
            canas_float(tx, &fr[n++], CANAS_ID_PITCH_RATE, o->pitch_rate_dps);
            canas_float(tx, &fr[n++], CANAS_ID_YAW_RATE, o->yaw_rate_dps);
        }
        if (v & AHRS_OUT_ALTITUDE) canas_float(tx, &fr[n++], CANAS_ID_STD_ALT, o->pressure_alt_m);
        if (v & AHRS_OUT_CLIMB) canas_float(tx, &fr[n++], CANAS_ID_ALT_RATE, o->climb_ms);
        if (v & AHRS_OUT_IAS) canas_float(tx, &fr[n++], CANAS_ID_IAS, o->ias_ms);
    }
    if (groups & CAN_OUT_GNSS) {
        if (v & AHRS_OUT_GNSS) {
            canas_float(tx, &fr[n++], CANAS_ID_GPS_LAT, (float)o->lat_deg);
            canas_float(tx, &fr[n++], CANAS_ID_GPS_LON, (float)o->lon_deg);
            canas_float(tx, &fr[n++], CANAS_ID_GPS_HEIGHT, o->height_m);
            canas_float(tx, &fr[n++], CANAS_ID_GPS_GS, o->gs_kmh);
            canas_float(tx, &fr[n++], CANAS_ID_GPS_TRACK, o->track_deg);
            canas_short(tx, &fr[n++], CANAS_ID_GPS_MODE, o->gps_mode);
            canas_uchar(tx, &fr[n++], CANAS_ID_GPS_SATS, o->sats);
        }
        if (v & AHRS_OUT_VARIATION) canas_float(tx, &fr[n++], CANAS_ID_MAG_VAR, o->variation_deg);
    }
    if ((groups & CAN_OUT_TIME) && (v & AHRS_OUT_TIME)) {
        const uint8_t t[4] = {o->hour, o->min, o->sec, 0};
        const uint8_t d[4] = {o->day, o->month, (uint8_t)(o->year % 100u), (uint8_t)(o->year / 100u)};
        canas_uchar4(tx, &fr[n++], CANAS_ID_UTC, t);
        canas_uchar4(tx, &fr[n++], CANAS_ID_DATE, d);
    }
    return n;
}
