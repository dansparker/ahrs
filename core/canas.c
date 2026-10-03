#include "canas.h"

#include <string.h>

void canas_init(canas_tx_t* tx, uint8_t node_id) {
    memset(tx, 0, sizeof(*tx));
    tx->node_id = node_id;
}

static uint8_t next_code(canas_tx_t* tx, uint16_t id) {
    for (unsigned i = 0; i < CANAS_CODE_SLOTS; ++i) {
        if (tx->ids[i] == id) return ++tx->codes[i];
        if (tx->ids[i] == 0) {
            tx->ids[i] = id;
            tx->codes[i] = 0;
            return 0;
        }
    }
    return 0; /* table full: code stays 0 (receivers then see no sequence) */
}

static void header(canas_tx_t* tx, canas_frame_t* f, uint16_t id, uint8_t type, uint8_t n_data) {
    memset(f, 0, sizeof(*f));
    f->id = id;
    f->dlc = (uint8_t)(4u + n_data);
    f->data[0] = tx->node_id;
    f->data[1] = type;
    f->data[2] = 0;
    f->data[3] = next_code(tx, id);
}

void canas_float(canas_tx_t* tx, canas_frame_t* f, uint16_t id, float v) {
    uint32_t u;
    memcpy(&u, &v, 4);
    header(tx, f, id, CANAS_FLOAT, 4);
    for (int i = 0; i < 4; ++i) f->data[4 + i] = (uint8_t)(u >> (24 - 8 * i));
}

void canas_short(canas_tx_t* tx, canas_frame_t* f, uint16_t id, int16_t v) {
    header(tx, f, id, CANAS_SHORT, 2);
    f->data[4] = (uint8_t)((uint16_t)v >> 8);
    f->data[5] = (uint8_t)v;
}

void canas_uchar(canas_tx_t* tx, canas_frame_t* f, uint16_t id, uint8_t v) {
    header(tx, f, id, CANAS_UCHAR, 1);
    f->data[4] = v;
}

void canas_uchar4(canas_tx_t* tx, canas_frame_t* f, uint16_t id, const uint8_t v[4]) {
    header(tx, f, id, CANAS_UCHAR4, 4);
    memcpy(&f->data[4], v, 4);
}

int canas_node_service(const canas_tx_t* tx, uint16_t id, const uint8_t* data, uint8_t dlc, uint8_t hw_rev,
                       uint8_t sw_rev, canas_frame_t* reply) {
    if (id != CANAS_ID_NODE_SERVICE_REQ || dlc < 4) return 0;
    if (data[0] != 0 && data[0] != tx->node_id) return 0; /* addressed node, 0 = all */
    if (data[2] != 0) return 0;                           /* service code 0 = IDS */
    memset(reply, 0, sizeof(*reply));
    reply->id = CANAS_ID_NODE_SERVICE_RESP;
    reply->dlc = 8;
    reply->data[0] = tx->node_id;
    reply->data[1] = CANAS_UCHAR4;
    reply->data[2] = 0;
    reply->data[3] = data[3]; /* message code of the request */
    reply->data[4] = hw_rev;
    reply->data[5] = sw_rev;
    reply->data[6] = 0; /* identifier distribution: standard */
    reply->data[7] = 0; /* header type: standard */
    return 1;
}

float canas_get_float(const canas_frame_t* f) {
    uint32_t u = (uint32_t)f->data[4] << 24 | (uint32_t)f->data[5] << 16 | (uint32_t)f->data[6] << 8 | f->data[7];
    float v;
    memcpy(&v, &u, 4);
    return v;
}
