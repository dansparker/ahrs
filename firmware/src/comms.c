/* comms.c - CAN1 transmit queue + node service, USART1 GNSS receive ring, calibration storage. */
#include "comms.h"

#include <string.h>

#include "board.h"

extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart1;
extern canas_tx_t g_canas;
extern IWDG_HandleTypeDef hiwdg;

/* ---------------- CAN ---------------- */
#define CAN_QUEUE 64u
static canas_frame_t q[CAN_QUEUE];
static volatile uint32_t q_head, q_tail;
static volatile uint32_t can_dropped;

static int mailbox_put(const canas_frame_t* f) {
    CAN_TxHeaderTypeDef h = {0};
    uint32_t mb;
    h.StdId = f->id;
    h.IDE = CAN_ID_STD;
    h.RTR = CAN_RTR_DATA;
    h.DLC = f->dlc;
    return HAL_CAN_AddTxMessage(&hcan1, &h, (uint8_t*)f->data, &mb) == HAL_OK;
}

static void refill(void) {
    while (q_tail != q_head && HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
        if (!mailbox_put(&q[q_tail % CAN_QUEUE])) break;
        q_tail++;
    }
}

int can_init(void) {
    CAN_FilterTypeDef fl = {0};
    fl.FilterBank = 0;
    fl.FilterMode = CAN_FILTERMODE_IDMASK;
    fl.FilterScale = CAN_FILTERSCALE_32BIT;
    fl.FilterIdHigh = (uint16_t)(CANAS_ID_NODE_SERVICE_REQ << 5);
    fl.FilterMaskIdHigh = (uint16_t)(0x7FFu << 5);
    fl.FilterFIFOAssignment = CAN_RX_FIFO0;
    fl.FilterActivation = ENABLE;
    fl.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan1, &fl) != HAL_OK) return 0;
    fl.FilterBank = 1; /* magnetic variation from the display */
    fl.FilterIdHigh = (uint16_t)(CANAS_ID_MAG_VAR << 5);
    if (HAL_CAN_ConfigFilter(&hcan1, &fl) != HAL_OK) return 0;
    if (HAL_CAN_Start(&hcan1) != HAL_OK) return 0;
    return HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK;
}

void can_send(const canas_frame_t* f) {
    __disable_irq();
    if (q_head - q_tail < CAN_QUEUE) {
        q[q_head % CAN_QUEUE] = *f;
        q_head++;
    } else {
        can_dropped++;
    }
    refill();
    __enable_irq();
}

uint32_t can_dropped_frames(void) { return can_dropped; }

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* h) { (void)h; refill(); }
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef* h) { (void)h; refill(); }
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef* h) { (void)h; refill(); }
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef* h) { (void)h; refill(); }
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef* h) { (void)h; refill(); }
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef* h) { (void)h; refill(); }

static volatile service_req_t rx_req;
static volatile int rx_req_new;

int can_take_service_request(service_req_t* r) {
    if (!rx_req_new) return 0;
    __disable_irq();
    r->service = rx_req.service;
    r->msg_code = rx_req.msg_code;
    r->type = rx_req.type;
    for (int i = 0; i < 4; ++i) r->data[i] = rx_req.data[i];
    rx_req_new = 0;
    __enable_irq();
    return 1;
}

static volatile float rx_variation;
static volatile int rx_variation_new;

int can_take_variation(float* deg) {
    if (!rx_variation_new) return 0;
    __disable_irq();
    *deg = rx_variation;
    rx_variation_new = 0;
    __enable_irq();
    return 1;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* h) {
    CAN_RxHeaderTypeDef rh;
    uint8_t d[8];
    if (HAL_CAN_GetRxMessage(h, CAN_RX_FIFO0, &rh, d) != HAL_OK || rh.IDE != CAN_ID_STD) return;
    if (rh.StdId == CANAS_ID_MAG_VAR) {
        const int from_display = (EFIS_NODE_ID == 0u || d[0] == EFIS_NODE_ID) && d[0] != g_canas.node_id;
        if (from_display && rh.DLC == 8 && d[1] == CANAS_FLOAT) {
            canas_frame_t f = {CANAS_ID_MAG_VAR, 8, {0}};
            memcpy(f.data, d, 8);
            rx_variation = canas_get_float(&f);
            rx_variation_new = 1;
        }
        return;
    }
    /* alignment services: only when addressed to this node explicitly (no broadcast) */
    if (rh.StdId == CANAS_ID_NODE_SERVICE_REQ && rh.DLC >= 4 && d[0] == g_canas.node_id &&
        d[2] >= CANAS_SERVICE_ALIGN_LEVEL && d[2] <= CANAS_SERVICE_ALIGN_RESET) {
        rx_req.service = d[2];
        rx_req.msg_code = d[3];
        rx_req.type = d[1];
        for (int i = 0; i < 4; ++i) rx_req.data[i] = rh.DLC >= 8 ? d[4 + i] : 0;
        rx_req_new = 1;
        return;
    }
    canas_frame_t reply;
    if (canas_node_service(&g_canas, (uint16_t)rh.StdId, d, (uint8_t)rh.DLC, HW_REVISION, SW_REVISION, &reply)) {
        if (q_head - q_tail < CAN_QUEUE) {
            q[q_head % CAN_QUEUE] = reply;
            q_head++;
        }
        refill();
    }
}

/* ---------------- GNSS UART ---------------- */
#define RX_RING 1024u
static volatile uint8_t rx[RX_RING];
static volatile uint32_t rx_head, rx_tail;

void gnss_uart_irq(void) {
    USART_TypeDef* u = huart1.Instance;
    const uint32_t sr = u->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
        const uint8_t c = (uint8_t)u->DR; /* reading DR also clears the error flags */
        if ((sr & USART_SR_RXNE) && rx_head - rx_tail < RX_RING) {
            rx[rx_head % RX_RING] = c;
            rx_head++;
        }
    }
}

int gnss_getc(uint8_t* c) {
    if (rx_tail == rx_head) return 0;
    *c = rx[rx_tail % RX_RING];
    rx_tail++;
    return 1;
}

static void set_baud(uint32_t baud) {
    HAL_UART_DeInit(&huart1);
    huart1.Init.BaudRate = baud;
    HAL_UART_Init(&huart1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

void gnss_configure(void) {
    /* The receiver may be at its factory baud rate (9600 for M8/M9, 38400 for some M10) or
     * already at 115200: send the configuration at each rate, then listen at 115200. */
    static const uint32_t rates[3] = {115200u, 38400u, 9600u};
    uint8_t m[128];
    for (int r = 0; r < 3; ++r) {
        set_baud(rates[r]);
        for (unsigned n = 0;; ++n) {
            const size_t len = ubx_config_msg(n, 115200u, GNSS_RATE_HZ, m);
            if (!len) break;
            HAL_UART_Transmit(&huart1, m, (uint16_t)len, 200);
            HAL_Delay(30);
        }
        HAL_IWDG_Refresh(&hiwdg);
    }
    set_baud(115200u);
}

/* ---------------- calibration storage: flash sector 1 (0x08004000, 16 KB) ---------------- */
#define STORE_ADDR 0x08004000u
#define STORE_MAGIC 0x4D414733u /* "MAG3" */

typedef struct {
    uint32_t magic;
    store_data_t d;
    uint32_t sum;
} store_t;

static uint32_t checksum(const store_t* s) {
    const uint32_t* w = (const uint32_t*)s;
    uint32_t c = 0x12345678u;
    for (size_t i = 0; i < offsetof(store_t, sum) / 4u; ++i) c = (c ^ w[i]) * 16777619u;
    return c;
}

int store_load(store_data_t* d) {
    store_t s;
    memcpy(&s, (const void*)STORE_ADDR, sizeof(s));
    if (s.magic != STORE_MAGIC || s.sum != checksum(&s)) return 0;
    *d = s.d;
    return 1;
}

int store_save(const store_data_t* d) {
    store_t s;
    memset(&s, 0, sizeof(s));
    s.magic = STORE_MAGIC;
    s.d = *d;
    s.sum = checksum(&s);
    FLASH_EraseInitTypeDef e = {0};
    uint32_t err = 0;
    e.TypeErase = FLASH_TYPEERASE_SECTORS;
    e.Sector = FLASH_SECTOR_1;
    e.NbSectors = 1;
    e.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_FLASH_Unlock();
    int ok = HAL_FLASHEx_Erase(&e, &err) == HAL_OK;
    const uint32_t* w = (const uint32_t*)&s;
    for (size_t i = 0; ok && i < sizeof(s) / 4u; ++i)
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, STORE_ADDR + 4u * i, w[i]) == HAL_OK;
    HAL_FLASH_Lock();
    return ok;
}
