/* sensors.h - IMU, magnetometer and barometer drivers (body frame, SI units). */
#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

int imu_init(void);
/* 1 = new sample (rad/s, m/s^2 specific force), 0 = none yet, -1 = bus error */
int imu_read(float gyro[3], float acc[3]);
int mag_init(void);
int mag_read(float ut[3]);
int baro_init(void);
/* Runs the MS5611 conversion state machine; returns 1 with a new pressure (Pa). */
int baro_poll(uint32_t now_us, float* p_pa);
/* MS4525DO on I2C2: 1 = new value (Pa, pitot - static; degC), 0 = no new conversion, -1 = error */
int airspeed_read(float* dp_pa, float* temp_c);

#endif
