/* comms.h - CAN bus, GNSS serial port, calibration storage. */
#ifndef COMMS_H
#define COMMS_H

#include <stddef.h>
#include <stdint.h>

#include "canas.h"
#include "magcal.h"
#include "stm32f4xx_hal.h"
#include "ubx.h"

int can_init(void);
void can_send(const canas_frame_t* f);
uint32_t can_dropped_frames(void);

void gnss_uart_irq(void);
int gnss_getc(uint8_t* c);
void gnss_configure(void);

/* Magnetic variation received from the display (deg east); returns 1 once per new value. */
int can_take_variation(float* deg);

/* Non-volatile settings: magnetometer calibration and last magnetic variation. */
typedef struct {
    magcal_params_t mag;
    float declination_deg;
    int32_t declination_valid;
} store_data_t;
int store_load(store_data_t* d);
int store_save(const store_data_t* d);

#endif
