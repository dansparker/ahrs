/* can_out.h - maps the AHRS output onto CANaerospace frames (OpenEFIS ADR 0002 + UTC/date). */
#ifndef CAN_OUT_H
#define CAN_OUT_H

#include "ahrs.h"
#include "canas.h"

#define CAN_OUT_FAST (1u << 0) /* attitude, rates, air data: 50 Hz */
#define CAN_OUT_GNSS (1u << 1) /* position, speed, track, mode, satellites, variation: 10 Hz */
#define CAN_OUT_TIME (1u << 2) /* UTC time and date: 1 Hz */
#define CAN_OUT_MAX_FRAMES 20u

/* Builds the frames of the selected groups; values without a valid bit are left out. */
unsigned can_out_build(canas_tx_t* tx, const ahrs_out_t* o, unsigned groups, canas_frame_t* frames);

#endif
