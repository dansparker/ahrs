#include "baro.h"

#include <math.h>

void ms5611_compensate(const uint16_t C[8], uint32_t d1, uint32_t d2, int32_t* p_pa, int32_t* t_cdeg) {
    const int64_t dT = (int64_t)d2 - ((int64_t)C[5] << 8);
    int64_t temp = 2000 + dT * C[6] / 8388608;
    int64_t off = ((int64_t)C[2] << 16) + (C[4] * dT) / 128;
    int64_t sens = ((int64_t)C[1] << 15) + (C[3] * dT) / 256;
    if (temp < 2000) {
        const int64_t t2 = dT * dT / 2147483648LL;
        int64_t off2 = 5 * (temp - 2000) * (temp - 2000) / 2;
        int64_t sens2 = 5 * (temp - 2000) * (temp - 2000) / 4;
        if (temp < -1500) {
            off2 += 7 * (temp + 1500) * (temp + 1500);
            sens2 += 11 * (temp + 1500) * (temp + 1500) / 2;
        }
        temp -= t2;
        off -= off2;
        sens -= sens2;
    }
    *p_pa = (int32_t)((((int64_t)d1 * sens) / 2097152 - off) / 32768);
    *t_cdeg = (int32_t)temp;
}

int ms5611_prom_crc_ok(const uint16_t prom[8]) {
    uint16_t n[8];
    for (int i = 0; i < 8; ++i) n[i] = prom[i];
    const uint16_t crc_read = n[7] & 0x000F;
    n[7] &= 0xFF00;
    uint16_t rem = 0;
    for (int cnt = 0; cnt < 16; ++cnt) {
        rem ^= (cnt & 1) ? (uint16_t)(n[cnt >> 1] & 0x00FF) : (uint16_t)(n[cnt >> 1] >> 8);
        for (int b = 8; b > 0; --b) rem = (rem & 0x8000) ? (uint16_t)((rem << 1) ^ 0x3000) : (uint16_t)(rem << 1);
    }
    rem = (rem >> 12) & 0x000F;
    return rem == crc_read;
}

float pressure_altitude_m(float p_pa) { return 44330.77f * (1.0f - powf(p_pa / 101325.0f, 0.190263f)); }
