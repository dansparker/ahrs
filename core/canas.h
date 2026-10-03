/*
 * canas.h - CANaerospace V1.6 encoder for the OpenEFIS AHRS interface
 * (open-efis docs/adr/0002-canaerospace.md): CAN 2.0A, 4 header bytes + up to 4 data bytes,
 * big endian, message code counting per identifier.
 */
#ifndef CANAS_H
#define CANAS_H

#include <stdint.h>

#define CANAS_FLOAT 2u
#define CANAS_SHORT 6u
#define CANAS_UCHAR 10u
#define CANAS_UCHAR4 16u

#define CANAS_ID_NODE_SERVICE_REQ 128u
#define CANAS_ID_NODE_SERVICE_RESP 129u
#define CANAS_ID_LATERAL_ACCEL 301u  /* g */
#define CANAS_ID_PITCH_RATE 303u     /* deg/s */
#define CANAS_ID_YAW_RATE 305u       /* deg/s */
#define CANAS_ID_PITCH 311u          /* deg */
#define CANAS_ID_ROLL 312u           /* deg */
#define CANAS_ID_ALT_RATE 314u       /* m/s */
#define CANAS_ID_IAS 315u            /* m/s */
#define CANAS_ID_STD_ALT 322u        /* m */
#define CANAS_ID_DIFF_PRESSURE 325u  /* hPa */
#define CANAS_ID_STATIC_PRESSURE 326u /* hPa */
#define CANAS_ID_GPS_LAT 1036u       /* deg */
#define CANAS_ID_GPS_LON 1037u       /* deg */
#define CANAS_ID_GPS_HEIGHT 1038u    /* m above ellipsoid */
#define CANAS_ID_GPS_GS 1039u        /* km/h */
#define CANAS_ID_GPS_TRACK 1040u     /* deg true */
#define CANAS_ID_GPS_MODE 1048u      /* SHORT 0/1/2 */
#define CANAS_ID_MAG_HEADING 1069u   /* deg */
#define CANAS_ID_MAG_VAR 1121u       /* deg, east positive: received from the display (WMM) */
#define CANAS_ID_UTC 1200u           /* UCHAR4: hours, minutes, seconds, 0 */
#define CANAS_ID_DATE 1201u          /* UCHAR4: day, month, year % 100, year / 100 */
#define CANAS_ID_GPS_SATS 1800u      /* UCHAR (UDL) */

typedef struct {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
} canas_frame_t;

#define CANAS_CODE_SLOTS 32u

typedef struct {
    uint8_t node_id;
    uint16_t ids[CANAS_CODE_SLOTS];
    uint8_t codes[CANAS_CODE_SLOTS];
} canas_tx_t;

void canas_init(canas_tx_t* tx, uint8_t node_id);
void canas_float(canas_tx_t* tx, canas_frame_t* f, uint16_t id, float v);
void canas_short(canas_tx_t* tx, canas_frame_t* f, uint16_t id, int16_t v);
void canas_uchar(canas_tx_t* tx, canas_frame_t* f, uint16_t id, uint8_t v);
void canas_uchar4(canas_tx_t* tx, canas_frame_t* f, uint16_t id, const uint8_t v[4]);

/* Answers the identification service (IDS) on 128 with a frame on 129; returns 1 if *reply is set. */
int canas_node_service(const canas_tx_t* tx, uint16_t id, const uint8_t* data, uint8_t dlc, uint8_t hw_rev,
                       uint8_t sw_rev, canas_frame_t* reply);

/* --- decoding (tests, bus monitors) --- */
float canas_get_float(const canas_frame_t* f);

#endif
