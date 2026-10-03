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

int store_load(magcal_params_t* p);
int store_save(const magcal_params_t* p);

#endif
