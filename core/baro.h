/* baro.h - MS5611 compensation and ISA pressure altitude. */
#ifndef BARO_H
#define BARO_H

#include <stdint.h>

/* prom[1..6] = C1..C6. Returns pressure in Pa and temperature in 0.01 degC. */
void ms5611_compensate(const uint16_t prom[8], uint32_t d1, uint32_t d2, int32_t* p_pa, int32_t* t_cdeg);
/* CRC-4 of the PROM (AN520); returns 1 when it matches the low nibble of word 7. */
int ms5611_prom_crc_ok(const uint16_t prom[8]);
/* Standard (pressure) altitude in m for a static pressure in Pa (ISA, 1013.25 hPa). */
float pressure_altitude_m(float p_pa);

#endif
