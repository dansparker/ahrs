/*
 * board.h - pin assignment (KiCad schematic "ahrs" + WeAct STM32F446RE core board V1.1)
 * and installation settings.
 *
 *   I2C1  PB6 SCL / PB7 SDA      LSM6DSOX (0x6A/0x6B) + LIS3MDL (0x1C/0x1E), J9/J8
 *   SPI1  PA5 SCK / PA6 MISO / PA7 MOSI, PA4 CS   MS5611 (GY-63, PS low = SPI), J3
 *   USART1 PA9 TX / PA10 RX      u-blox GNSS (UBX), J5/J7
 *   CAN1  PB8 RX / PB9 TX, PB4 STB (low = normal)  ATA6561, J10/J12
 *   I2C2  PB10 SCL / PC12 SDA    MS4525DO differential pressure (J1): read out only, not fused
 *   LEDs  PC13 ER_GPS, PC14 ER_AHRS, PC0 SC_OK, PC15 ER_ARINC (active high), PB2 blue LED on the core board
 *   INT1 PA15, INT2 PD2, INT_M PA8, DRDY PB12 (not used: the IMU is polled)
 */
#ifndef BOARD_H
#define BOARD_H

#define HW_REVISION 1u
#define SW_REVISION 1u

/* CANaerospace node ID (1..255), as configured in the OpenEFIS profile */
#define CAN_NODE_ID 7u

/* Initial magnetic declination guess, deg east (Austria ~ +5). Refined in flight with GNSS. */
#define MAG_DECLINATION_DEG 5.0f

/*
 * Sensor axes -> aircraft body axes (x forward, y right, z down): body = M * sensor.
 * Default: the breakout lies flat, component side up, its x arrow pointing forward
 * (sensor z up -> body z down, so y and z flip). CHECK THE ARROWS ON YOUR BOARD:
 * the LIS3MDL and LSM6DSOX axes printed on the breakout must both be mapped.
 */
#define IMU_MOUNT {{1, 0, 0}, {0, -1, 0}, {0, 0, -1}}
#define MAG_MOUNT {{1, 0, 0}, {0, -1, 0}, {0, 0, -1}}

/* GNSS navigation rate (UBX-NAV-PVT). u-blox M8 does 10 Hz only with a single constellation
 * (otherwise it rejects the rate and keeps 1 Hz); M9/M10 handle 10 Hz with several. */
#define GNSS_RATE_HZ 10u

/* MS4525DO on I2C2: address 0x28 (I2C address code I), range and output type of the part
 * (e.g. MS4525DO-DS5AI001DP: +-1 psi, type A) */
#define MS4525_ADDR 0x28u
#define MS4525_RANGE {-1.0f, 1.0f, 0}

#endif
