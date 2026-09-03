/*
 * ant_embedded.c - ANT serial protocol front end for ant_mac. See the header.
 */
#include "ant_embedded.h"
#include <string.h>

#define ANT_EMBEDDED_VERSION  "ESP32-ANT 1.0"

/* ------------------------------ ANT -> host ------------------------------ */

static void out_push(ant_embedded_t *e, const uint8_t *frame, size_t n)
{
    if (n == 0) return;
    if (e->out_count + n > ANT_EMBEDDED_OUT_SIZE) {
        e->out_overflow++;
        return;
    }
    for (size_t i = 0; i < n; i++) {
        e->out[(e->out_head + e->out_count) % ANT_EMBEDDED_OUT_SIZE] = frame[i];
        e->out_count++;
    }
}

static void emit(ant_embedded_t *e, uint8_t msg_id, const uint8_t *data, uint8_t len)
{
    uint8_t frame[ANT_MAX_FRAME_LEN];
    size_t n = ant_message_encode(msg_id, data, len, frame, sizeof(frame));
    out_push(e, frame, n);
}

static void emit_response(ant_embedded_t *e, uint8_t ch, uint8_t msg_id, uint8_t code)
{
    uint8_t d[3] = { ch, msg_id, code };
    emit(e, ANT_MSG_CHANNEL_RESPONSE, d, 3);
}

static void emit_startup(ant_embedded_t *e)
{
    uint8_t d[1] = { 0x20 };     /* startup reason: command reset */
    emit(e, ANT_MSG_STARTUP, d, 1);
}

static void mac_on_data(ant_mac_t *mac, uint8_t ch, uint8_t type, uint8_t seq,
                        const uint8_t data[8], void *user)
{
    (void)mac;
    ant_embedded_t *e = user;
    uint8_t d[9];
    /* Burst: the ANT sequence field rides in the top 3 bits of the channel byte. */
    d[0] = (type == ANT_MSG_BURST_DATA) ? (uint8_t)((seq << 5) | (ch & 0x1F)) : ch;
    memcpy(&d[1], data, 8);
    emit(e, type, d, 9);
}

static void mac_on_event(ant_mac_t *mac, uint8_t ch, uint8_t event, void *user)
{
    (void)mac;
    emit_response((ant_embedded_t *)user, ch, 0x01, event);
}

/* ------------------------------ host -> ANT ------------------------------ */

static void handle_request(ant_embedded_t *e, uint8_t ch, uint8_t what)
{
    switch (what) {
    case ANT_MSG_CHANNEL_STATUS: {
        uint8_t d[2] = { ch, ant_mac_channel_status(&e->mac, ch) };
        emit(e, ANT_MSG_CHANNEL_STATUS, d, 2);
        break;
    }
    case ANT_MSG_CHANNEL_ID: {
        uint16_t dev = 0; uint8_t dt = 0, tt = 0;
        ant_mac_get_channel_id(&e->mac, ch, &dev, &dt, &tt);
        uint8_t d[5] = { ch, (uint8_t)(dev & 0xFF), (uint8_t)(dev >> 8), dt, tt };
        emit(e, ANT_MSG_CHANNEL_ID, d, 5);
        break;
    }
    case ANT_MSG_CAPABILITIES: {
        /* [max channels, max networks, standard opts, advanced opts, adv2, ...] */
        uint8_t d[6] = { ANT_MAC_MAX_CHANNELS, ANT_MAC_MAX_NETWORKS,
                         0x00,   /* standard: nothing disabled */
                         0x02,   /* advanced: network enabled */
                         0x00, 0x00 };
        emit(e, ANT_MSG_CAPABILITIES, d, 6);
        break;
    }
    case ANT_MSG_VERSION: {
        uint8_t d[ANT_MAX_MSG_DATA];
        size_t n = strlen(ANT_EMBEDDED_VERSION) + 1;
        if (n > sizeof(d)) n = sizeof(d);
        memcpy(d, ANT_EMBEDDED_VERSION, n);
        emit(e, ANT_MSG_VERSION, d, (uint8_t)n);
        break;
    }
    default:
        emit_response(e, ch, ANT_MSG_REQUEST, ANT_RESPONSE_INVALID_MESSAGE);
        break;
    }
}

static void handle_burst(ant_embedded_t *e, const ant_message_t *msg)
{
    uint8_t ch  = msg->data[0] & 0x1F;
    uint8_t seq = msg->data[0] >> 5;
    bool last   = (seq & 0x04) != 0;
    seq &= 0x03;
    if (ch >= ANT_MAC_MAX_CHANNELS) {
        emit_response(e, ch, ANT_MSG_BURST_DATA, ANT_RESPONSE_INVALID_PARAMETER);
        return;
    }
    ant_embedded_hburst_t *hb = &e->hburst[ch];

    if (!hb->active) {
        if (seq != 0) {
            emit_response(e, ch, ANT_MSG_BURST_DATA, ANT_RESPONSE_TRANSFER_SEQUENCE_ERROR);
            return;
        }
        hb->active = true;
        hb->len = 0;
        hb->next_seq = 0;
    } else if (seq != hb->next_seq) {
        hb->active = false;
        emit_response(e, ch, ANT_MSG_BURST_DATA, ANT_RESPONSE_TRANSFER_SEQUENCE_ERROR);
        return;
    }
    if (hb->len + 8u > ANT_MAC_BURST_MAX_BYTES) {
        hb->active = false;
        emit_response(e, ch, ANT_MSG_BURST_DATA, ANT_RESPONSE_INVALID_PARAMETER);
        return;
    }
    memcpy(&hb->buf[hb->len], &msg->data[1], 8);
    hb->len = (uint8_t)(hb->len + 8u);
    hb->next_seq = (seq == 0 || seq == 3) ? 1u : (uint8_t)(seq + 1u);

    if (last) {
        hb->active = false;
        uint8_t rc = ant_mac_send_burst(&e->mac, ch, hb->buf, hb->len);
        if (rc != ANT_RESPONSE_NO_ERROR) emit_response(e, ch, ANT_MSG_BURST_DATA, rc);
        /* Success is reported by EVENT_TRANSFER_TX_COMPLETED, as on a real chip. */
    }
}

void ant_embedded_handle(ant_embedded_t *e, const ant_message_t *msg)
{
    const uint8_t *d = msg->data;
    uint8_t n = msg->data_len;
    uint8_t ch = n ? d[0] : 0;
    uint8_t rc;

#define NEED(k) do { if (n < (k)) { emit_response(e, ch, msg->msg_id, ANT_RESPONSE_INVALID_MESSAGE); return; } } while (0)

    switch (msg->msg_id) {
    case ANT_MSG_RESET_SYSTEM:
        ant_mac_reset(&e->mac);
        memset(e->hburst, 0, sizeof(e->hburst));
        emit_startup(e);
        return;

    case ANT_MSG_SET_NETWORK_KEY:
        NEED(9);
        rc = ant_mac_set_network_key(&e->mac, d[0], &d[1]);
        emit_response(e, 0, msg->msg_id, rc);
        return;

    case ANT_MSG_ASSIGN_CHANNEL:
        NEED(3);
        rc = ant_mac_assign_channel(&e->mac, ch, d[1], d[2]);
        break;
    case ANT_MSG_UNASSIGN_CHANNEL:
        NEED(1);
        rc = ant_mac_unassign_channel(&e->mac, ch);
        break;
    case ANT_MSG_SET_CHANNEL_ID:
        NEED(5);
        rc = ant_mac_set_channel_id(&e->mac, ch, (uint16_t)(d[1] | (d[2] << 8)), d[3], d[4]);
        break;
    case ANT_MSG_SET_CHANNEL_PERIOD:
        NEED(3);
        rc = ant_mac_set_channel_period(&e->mac, ch, (uint16_t)(d[1] | (d[2] << 8)));
        break;
    case ANT_MSG_SET_CHANNEL_RF_FREQ:
        NEED(2);
        rc = ant_mac_set_channel_rf_freq(&e->mac, ch, d[1]);
        break;
    case ANT_MSG_SET_SEARCH_TIMEOUT:
        NEED(2);
        rc = ant_mac_set_search_timeout(&e->mac, ch, d[1]);
        break;
    case ANT_MSG_SET_CHANNEL_TX_POWER:
        NEED(2);
        rc = ant_mac_set_tx_power(&e->mac, ch, d[1]);
        break;
    case ANT_MSG_OPEN_CHANNEL:
        NEED(1);
        rc = ant_mac_open_channel(&e->mac, ch);
        break;
    case ANT_MSG_CLOSE_CHANNEL:
        NEED(1);
        rc = ant_mac_close_channel(&e->mac, ch);
        break;

    case ANT_MSG_BROADCAST_DATA:
        NEED(9);
        rc = ant_mac_send_broadcast(&e->mac, ch, &d[1]);
        /* ANT chips do not respond to a successful broadcast load. */
        if (rc != ANT_RESPONSE_NO_ERROR) emit_response(e, ch, msg->msg_id, rc);
        return;
    case ANT_MSG_ACKNOWLEDGED_DATA:
        NEED(9);
        rc = ant_mac_send_acknowledged(&e->mac, ch, &d[1]);
        if (rc != ANT_RESPONSE_NO_ERROR) emit_response(e, ch, msg->msg_id, rc);
        return;
    case ANT_MSG_BURST_DATA:
        NEED(9);
        handle_burst(e, msg);
        return;

    case ANT_MSG_REQUEST:
        NEED(2);
        handle_request(e, ch, d[1]);
        return;

    default:
        emit_response(e, ch, msg->msg_id, ANT_RESPONSE_INVALID_MESSAGE);
        return;
    }
#undef NEED
    emit_response(e, ch, msg->msg_id, rc);
}

/* ------------------------------- transport ------------------------------- */

static int emb_write(ant_radio_t *r, const uint8_t *data, size_t len)
{
    ant_embedded_t *e = (ant_embedded_t *)r;
    ant_message_t msg;
    for (size_t i = 0; i < len; i++) {
        if (ant_parser_push(&e->parser, data[i], &msg)) {
            ant_embedded_handle(e, &msg);
        }
    }
    return (int)len;
}

static int emb_read(ant_radio_t *r, uint8_t *buf, size_t cap)
{
    ant_embedded_t *e = (ant_embedded_t *)r;
    size_t n = 0;
    while (n < cap && e->out_count > 0) {
        buf[n++] = e->out[e->out_head];
        e->out_head = (uint16_t)((e->out_head + 1u) % ANT_EMBEDDED_OUT_SIZE);
        e->out_count--;
    }
    return (int)n;
}

static void emb_poll(ant_radio_t *r)
{
    (void)r;   /* the MAC is driven by ant_embedded_tick() */
}

void ant_radio_embedded_init(ant_embedded_t *e, ant_phy_t *phy)
{
    memset(e, 0, sizeof(*e));
    e->radio.write = emb_write;
    e->radio.read  = emb_read;
    e->radio.poll  = emb_poll;
    e->radio.close = NULL;
    e->radio.ctx   = e;
    ant_parser_init(&e->parser);
    ant_mac_init(&e->mac, phy, mac_on_data, mac_on_event, e);
}

ant_radio_t *ant_radio_embedded(ant_embedded_t *e)
{
    return &e->radio;
}

uint32_t ant_embedded_tick(ant_embedded_t *e, uint32_t now)
{
    return ant_mac_tick(&e->mac, now);
}
