/* sensors.c - LSM6DSOX + LIS3MDL on I2C1, MS5611 on SPI1. */
#include "sensors.h"

#include "baro.h"
#include "board.h"

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;

#define I2C_TIMEOUT_MS 3u

/* LSM6DSOX */
#define LSM_WHO_AM_I 0x0Fu
#define LSM_CTRL1_XL 0x10u
#define LSM_CTRL2_G 0x11u
#define LSM_CTRL3_C 0x12u
#define LSM_STATUS 0x1Eu
#define LSM_OUTX_L_G 0x22u
#define LSM_ID 0x6Cu
#define LSM_GYRO_SCALE (0.0175f * 0.01745329f) /* 500 dps: 17.5 mdps/LSB -> rad/s */
#define LSM_ACC_SCALE (0.000244f * 9.80665f)   /* +-8 g: 0.244 mg/LSB -> m/s^2 */

/* LIS3MDL */
#define LIS_WHO_AM_I 0x0Fu
#define LIS_CTRL_REG1 0x20u
#define LIS_STATUS 0x27u
#define LIS_OUT_X_L 0x28u
#define LIS_ID 0x3Du
#define LIS_SCALE (100.0f / 6842.0f) /* +-4 gauss: 6842 LSB/gauss -> uT */

/* MS5611 */
#define MS_RESET 0x1Eu
#define MS_CONV_D1_4096 0x48u
#define MS_CONV_D2_4096 0x58u
#define MS_ADC_READ 0x00u
#define MS_PROM 0xA0u
#define MS_CONV_TIME_US 9500u

static uint8_t lsm_addr, lis_addr;
static const float imu_mount[3][3] = IMU_MOUNT;
static const float mag_mount[3][3] = MAG_MOUNT;
static uint16_t ms_prom[8];
static uint32_t ms_d2, ms_t_start;
static uint8_t ms_phase, ms_count;
static int ms_ok;

static int wr(uint8_t addr, uint8_t reg, uint8_t val) {
    return HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(addr << 1), reg, I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT_MS) == HAL_OK;
}

static int rd(uint8_t addr, uint8_t reg, uint8_t* buf, uint16_t n) {
    return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr << 1), reg, I2C_MEMADD_SIZE_8BIT, buf, n, I2C_TIMEOUT_MS) == HAL_OK;
}

static int probe(uint8_t a0, uint8_t a1, uint8_t who_reg, uint8_t id, uint8_t* found) {
    const uint8_t addrs[2] = {a0, a1};
    for (int i = 0; i < 2; ++i) {
        uint8_t v = 0;
        if (rd(addrs[i], who_reg, &v, 1) && v == id) {
            *found = addrs[i];
            return 1;
        }
    }
    return 0;
}

static void mount(const float M[3][3], const float s[3], float out[3]) {
    for (int i = 0; i < 3; ++i) out[i] = M[i][0] * s[0] + M[i][1] * s[1] + M[i][2] * s[2];
}

int imu_init(void) {
    if (!probe(0x6A, 0x6B, LSM_WHO_AM_I, LSM_ID, &lsm_addr)) return 0;
    wr(lsm_addr, LSM_CTRL3_C, 0x01); /* software reset */
    HAL_Delay(10);
    int ok = wr(lsm_addr, LSM_CTRL3_C, 0x44);  /* BDU, register auto increment */
    ok &= wr(lsm_addr, LSM_CTRL1_XL, 0x5C);    /* 208 Hz, +-8 g */
    ok &= wr(lsm_addr, LSM_CTRL2_G, 0x54);     /* 208 Hz, 500 dps */
    return ok;
}

int imu_read(float gyro[3], float acc[3]) {
    uint8_t st = 0, b[12];
    if (!rd(lsm_addr, LSM_STATUS, &st, 1)) return -1;
    if (!(st & 0x02)) return 0; /* no new gyro sample */
    if (!rd(lsm_addr, LSM_OUTX_L_G, b, 12)) return -1;
    float g[3], a[3];
    for (int i = 0; i < 3; ++i) {
        g[i] = (float)(int16_t)(b[2 * i] | b[2 * i + 1] << 8) * LSM_GYRO_SCALE;
        a[i] = (float)(int16_t)(b[6 + 2 * i] | b[7 + 2 * i] << 8) * LSM_ACC_SCALE;
    }
    mount(imu_mount, g, gyro);
    /* specific force: level and at rest the body z axis (down) reads -1 g */
    mount(imu_mount, a, acc);
    return 1;
}

int mag_init(void) {
    if (!probe(0x1C, 0x1E, LIS_WHO_AM_I, LIS_ID, &lis_addr)) return 0;
    int ok = wr(lis_addr, LIS_CTRL_REG1, 0x7C);  /* ultra-high performance X/Y, 80 Hz */
    ok &= wr(lis_addr, LIS_CTRL_REG1 + 1, 0x00); /* +-4 gauss */
    ok &= wr(lis_addr, LIS_CTRL_REG1 + 3, 0x0C); /* ultra-high performance Z */
    ok &= wr(lis_addr, LIS_CTRL_REG1 + 4, 0x40); /* BDU */
    ok &= wr(lis_addr, LIS_CTRL_REG1 + 2, 0x00); /* continuous conversion */
    return ok;
}

int mag_read(float ut[3]) {
    uint8_t st = 0, b[6];
    if (!rd(lis_addr, LIS_STATUS, &st, 1)) return -1;
    if (!(st & 0x08)) return 0;
    if (!rd(lis_addr, LIS_OUT_X_L | 0x80u, b, 6)) return -1;
    float m[3];
    for (int i = 0; i < 3; ++i) m[i] = (float)(int16_t)(b[2 * i] | b[2 * i + 1] << 8) * LIS_SCALE;
    mount(mag_mount, m, ut);
    return 1;
}

/* --- MS5611 over SPI1, chip select PA4 --- */
static void cs(int on) { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, on ? GPIO_PIN_RESET : GPIO_PIN_SET); }

static int ms_cmd(uint8_t c) {
    cs(1);
    const int ok = HAL_SPI_Transmit(&hspi1, &c, 1, 2) == HAL_OK;
    cs(0);
    return ok;
}

static int ms_read(uint8_t c, uint8_t* rx, uint16_t n) {
    uint8_t tx[4] = {c, 0, 0, 0}, r[4] = {0};
    cs(1);
    const int ok = HAL_SPI_TransmitReceive(&hspi1, tx, r, (uint16_t)(n + 1), 2) == HAL_OK;
    cs(0);
    for (uint16_t i = 0; i < n; ++i) rx[i] = r[i + 1];
    return ok;
}

int baro_init(void) {
    cs(0);
    ms_cmd(MS_RESET);
    HAL_Delay(5);
    for (int i = 0; i < 8; ++i) {
        uint8_t b[2];
        if (!ms_read((uint8_t)(MS_PROM + 2 * i), b, 2)) return 0;
        ms_prom[i] = (uint16_t)(b[0] << 8 | b[1]);
    }
    ms_ok = ms5611_prom_crc_ok(ms_prom) && ms_prom[1] != 0 && ms_prom[1] != 0xFFFF;
    ms_phase = 0;
    ms_count = 0;
    return ms_ok;
}

int baro_poll(uint32_t now_us, float* p_pa) {
    if (!ms_ok) return 0;
    if (ms_phase != 0 && now_us - ms_t_start < MS_CONV_TIME_US) return 0;
    int got = 0;
    if (ms_phase != 0) {
        uint8_t b[3];
        ms_read(MS_ADC_READ, b, 3);
        const uint32_t adc = (uint32_t)b[0] << 16 | (uint32_t)b[1] << 8 | b[2];
        if (ms_phase == 2) {
            ms_d2 = adc;
        } else if (ms_d2 != 0 && adc != 0) {
            int32_t p, t;
            ms5611_compensate(ms_prom, adc, ms_d2, &p, &t);
            *p_pa = (float)p;
            got = 1;
        }
    }
    /* temperature every 20th conversion, pressure otherwise */
    const int temp_next = ms_d2 == 0 || ++ms_count >= 20;
    if (temp_next) ms_count = 0;
    ms_cmd(temp_next ? MS_CONV_D2_4096 : MS_CONV_D1_4096);
    ms_phase = temp_next ? 2 : 1;
    ms_t_start = now_us;
    return got;
}
