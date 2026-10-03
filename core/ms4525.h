/* ms4525.h - TE MS4525DO differential pressure sensor (I2C, 14-bit pressure, 11-bit temperature). */
#ifndef MS4525_H
#define MS4525_H

#include <stdint.h>

#define MS4525_OK 0
#define MS4525_STALE 2 /* data already read (no new conversion) */
#define MS4525_FAULT 3

typedef struct {
    float pmin_psi, pmax_psi; /* range of the part, e.g. -1/+1 for ...001D */
    int type_b;               /* 0 = output type A (10..90 %), 1 = type B (5..95 %) */
} ms4525_range_t;

/* Decodes the 4-byte read-out. Returns the status bits (MS4525_*); pressure in Pa
 * (positive = pitot above static, port P1 at pitot), temperature in degC. */
int ms4525_decode(const uint8_t b[4], const ms4525_range_t* r, float* dp_pa, float* temp_c);

#endif
