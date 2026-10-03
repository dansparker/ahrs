#include "ms4525.h"

#define PSI_TO_PA 6894.757f

int ms4525_decode(const uint8_t b[4], const ms4525_range_t* r, float* dp_pa, float* temp_c) {
    const int status = b[0] >> 6;
    const uint16_t p = (uint16_t)(((b[0] & 0x3Fu) << 8) | b[1]);
    const uint16_t t = (uint16_t)((b[2] << 3) | (b[3] >> 5));
    const float lo = r->type_b ? 0.05f : 0.10f, span = r->type_b ? 0.90f : 0.80f;
    const float psi = ((float)p - lo * 16383.0f) * (r->pmax_psi - r->pmin_psi) / (span * 16383.0f) + r->pmin_psi;
    *dp_pa = psi * PSI_TO_PA;
    *temp_c = (float)t * 200.0f / 2047.0f - 50.0f;
    return status;
}
