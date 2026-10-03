/*
 * main.c - AHRS firmware for the STM32F446RE (WeAct core board).
 *
 * Loop: poll the IMU at 208 Hz -> ESKF; magnetometer every 4th sample (52 Hz); MS5611 ~95 Hz;
 * UBX NAV-PVT 10 Hz (GNSS_RATE_HZ); CANaerospace output 50 Hz (attitude/air data), 10 Hz (GNSS, UTC/date);
 * MS4525DO read at 20 Hz (not fused).
 */
#include <string.h>

#include "ahrs.h"
#include "board.h"
#include "can_out.h"
#include "comms.h"
#include "sensors.h"

CAN_HandleTypeDef hcan1;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;
IWDG_HandleTypeDef hiwdg;
canas_tx_t g_canas;

static ahrs_t ahrs;
static ubx_parser_t ubx;

/* MS4525DO read-out (debugger / future use); deliberately not fed into the fusion */
volatile float g_diff_pressure_pa, g_pitot_temp_c;
volatile uint32_t g_diff_pressure_ok;

#define LED_ER_GPS GPIOC, GPIO_PIN_13
#define LED_ER_AHRS GPIOC, GPIO_PIN_14
#define LED_SC_OK GPIOC, GPIO_PIN_0
#define LED_ER_ARINC GPIOC, GPIO_PIN_15
#define LED_BOARD GPIOB, GPIO_PIN_2

static void fatal(void) {
    for (;;) { /* the watchdog resets */
    }
}

static void clock_config(void) {
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSI;
    osc.HSEState = RCC_HSE_ON;
    osc.LSIState = RCC_LSI_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 4; /* 8 MHz / 4 * 180 / 2 = 180 MHz */
    osc.PLL.PLLN = 180;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 8;
    osc.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) fatal();
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) fatal();
    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4; /* 45 MHz: CAN */
    clk.APB2CLKDivider = RCC_HCLK_DIV2; /* 90 MHz: SPI1, USART1 */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) fatal();
}

static void gpio_af(GPIO_TypeDef* port, uint32_t pins, uint32_t mode, uint32_t pull, uint32_t af) {
    GPIO_InitTypeDef g = {0};
    g.Pin = pins;
    g.Mode = mode;
    g.Pull = pull;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = af;
    HAL_GPIO_Init(port, &g);
}

static void periph_init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();

    /* LEDs and outputs */
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0;
    HAL_GPIO_Init(GPIOC, &g);
    g.Pin = GPIO_PIN_2 | GPIO_PIN_4; /* board LED, CAN STB */
    HAL_GPIO_Init(GPIOB, &g);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); /* ATA6561 normal mode */
    g.Pin = GPIO_PIN_4; /* MS5611 CS */
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    gpio_af(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_AF_OD, GPIO_PULLUP, GPIO_AF4_I2C1);
    gpio_af(GPIOB, GPIO_PIN_10, GPIO_MODE_AF_OD, GPIO_PULLUP, GPIO_AF4_I2C2);
    gpio_af(GPIOC, GPIO_PIN_12, GPIO_MODE_AF_OD, GPIO_PULLUP, GPIO_AF4_I2C2);
    gpio_af(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF5_SPI1);
    gpio_af(GPIOA, GPIO_PIN_9 | GPIO_PIN_10, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART1);
    gpio_af(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF9_CAN1);

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) fatal();
    hi2c2.Instance = I2C2;
    hi2c2.Init = hi2c1.Init;
    hi2c2.Init.ClockSpeed = 100000;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) fatal();

    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; /* 5.6 MHz */
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) fatal();

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) fatal();
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    /* CAN1: 45 MHz / 5 / (1 + 15 + 2) = 500 kbit/s, sample point 88.9 % */
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 5;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.TransmitFifoPriority = ENABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK) fatal();
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
}

static void watchdog_init(void) {
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32; /* 1 ms tick */
    hiwdg.Init.Reload = 1000;                 /* 1 s: covers a flash sector erase */
    HAL_IWDG_Init(&hiwdg);
}

static uint32_t micros(void) { return DWT->CYCCNT / (SystemCoreClock / 1000000u); }

static void i2c_recover(void) {
    HAL_I2C_DeInit(&hi2c1);
    HAL_I2C_Init(&hi2c1);
    imu_init();
    mag_init();
}

int main(void) {
    HAL_Init();
    clock_config();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    periph_init();
    watchdog_init();

    HAL_GPIO_WritePin(LED_ER_AHRS, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_ER_GPS, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_ER_ARINC, GPIO_PIN_RESET);

    canas_init(&g_canas, CAN_NODE_ID);
    if (!can_init()) fatal();

    int imu_ok = imu_init();
    int mag_ok = mag_init();
    baro_init();
    HAL_IWDG_Refresh(&hiwdg);
    gnss_configure();
    HAL_IWDG_Refresh(&hiwdg);
    ubx_init(&ubx);

    ahrs_config_t cfg;
    ahrs_default_config(&cfg);
    cfg.dec0_deg = MAG_DECLINATION_DEG;
    ahrs_init(&ahrs, &cfg);
    magcal_params_t mp;
    if (store_load(&mp)) ahrs_set_magcal(&ahrs, &mp);

    uint32_t t_imu = micros(), t_fast = HAL_GetTick(), t_gnss_out = t_fast, t_pvt = t_fast;
    uint32_t imu_errors = 0, imu_n = 0, t_imu_ok = t_fast;
    uint32_t t_airspeed = t_fast;
    int save_pending = 0;
    canas_frame_t fr[CAN_OUT_MAX_FRAMES];

    for (;;) {
        HAL_IWDG_Refresh(&hiwdg);
        const uint32_t now_ms = HAL_GetTick(), now_us = micros();

        float gyro[3], acc[3], mag[3], p;
        const int r = imu_ok ? imu_read(gyro, acc) : -1;
        if (r > 0) {
            float dt = (float)(now_us - t_imu) * 1e-6f;
            t_imu = now_us;
            if (dt < 0.003f || dt > 0.008f) dt = 1.0f / 208.0f; /* first sample, missed sample */
            ahrs_imu(&ahrs, gyro, acc, dt);
            imu_errors = 0;
            t_imu_ok = now_ms;
            if (++imu_n % 4 == 0 && mag_ok && mag_read(mag) > 0) ahrs_mag(&ahrs, mag);
        } else if (r < 0 && ++imu_errors >= 3) {
            i2c_recover();
            imu_ok = imu_init();
            mag_ok = mag_init();
            imu_errors = 0;
        }
        if (baro_poll(now_us, &p)) ahrs_baro(&ahrs, p);
        if (now_ms - t_airspeed >= 50u) { /* 20 Hz */
            t_airspeed = now_ms;
            float dp, tc;
            if (airspeed_read(&dp, &tc) > 0) {
                g_diff_pressure_pa = dp;
                g_pitot_temp_c = tc;
                g_diff_pressure_ok++;
            }
        }

        uint8_t c;
        ubx_pvt_t pvt;
        while (gnss_getc(&c)) {
            if (ubx_feed(&ubx, c, &pvt)) {
                ahrs_gnss(&ahrs, &pvt);
                t_pvt = now_ms;
            }
        }
        /* receiver silent: configure again (baud rate, messages). This blocks for ~0.7 s, so only
         * before the first movement, never in flight. */
        if (now_ms - t_pvt > 10000u && !ahrs.ever_moved) {
            gnss_configure();
            t_pvt = HAL_GetTick();
        }

        unsigned groups = 0;
        if (now_ms - t_fast >= 20u) {
            t_fast += 20u;
            groups |= CAN_OUT_FAST;
        }
        if (now_ms - t_gnss_out >= 100u) { /* OpenEFIS drops values older than 500 ms: time at 10 Hz too */
            t_gnss_out += 100u;
            groups |= CAN_OUT_GNSS | CAN_OUT_TIME;
        }
        if (groups) {
            ahrs_out_t o;
            ahrs_output(&ahrs, &o);
            if (now_ms - t_imu_ok > 50u) o.valid &= ~(AHRS_OUT_ATTITUDE | AHRS_OUT_HEADING | AHRS_OUT_RATES | AHRS_OUT_CLIMB);
            const unsigned n = can_out_build(&g_canas, &o, groups, fr);
            for (unsigned i = 0; i < n; ++i) can_send(&fr[i]);

            if (groups & CAN_OUT_FAST) {
                HAL_GPIO_WritePin(LED_ER_AHRS, (o.valid & AHRS_OUT_ATTITUDE) ? GPIO_PIN_RESET : GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED_ER_GPS, (o.valid & AHRS_OUT_GNSS) ? GPIO_PIN_RESET : GPIO_PIN_SET);
                const int all_ok = (o.valid & (AHRS_OUT_ATTITUDE | AHRS_OUT_HEADING | AHRS_OUT_GNSS)) ==
                                   (AHRS_OUT_ATTITUDE | AHRS_OUT_HEADING | AHRS_OUT_GNSS);
                HAL_GPIO_WritePin(LED_SC_OK, (all_ok && (now_ms / 500u) % 2u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_BOARD, (now_ms / 1000u) % 2u ? GPIO_PIN_SET : GPIO_PIN_RESET);
            }
            /* store a new magnetometer calibration only while standing still on the ground:
             * erasing the flash sector blocks the CPU for up to 0.5 s */
            if (ahrs_take_magcal_changed(&ahrs)) save_pending = 1;
            const int on_ground = ahrs.stationary && ((ahrs.gnss == GNSS_OK && ahrs.last_gs < 1.0f) || !ahrs.ever_moved);
            if (save_pending && on_ground && ahrs.mc.active.valid) {
                HAL_IWDG_Refresh(&hiwdg);
                store_save(&ahrs.mc.active);
                save_pending = 0;
                t_imu = micros();
            }
        }
    }
}

/* ---------------- interrupts ---------------- */
void SysTick_Handler(void) { HAL_IncTick(); }
void USART1_IRQHandler(void) { gnss_uart_irq(); }
void CAN1_TX_IRQHandler(void) { HAL_CAN_IRQHandler(&hcan1); }
void CAN1_RX0_IRQHandler(void) { HAL_CAN_IRQHandler(&hcan1); }
void HardFault_Handler(void) { fatal(); }
