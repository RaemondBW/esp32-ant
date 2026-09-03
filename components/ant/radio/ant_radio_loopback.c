/*
 * ant_radio_loopback.c - Software radio + simulated ANT chip (see header).
 */
#include "ant_radio_loopback.h"
#include <string.h>

static void rx_push(ant_loopback_t *lb, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        size_t next = (lb->rx_head + 1) % ANT_LOOPBACK_BUF;
        if (next == lb->rx_tail) {
            return; /* full - drop (mirrors a real overrun) */
        }
        lb->rx[lb->rx_head] = data[i];
        lb->rx_head = next;
    }
}

static size_t rx_available(const ant_loopback_t *lb)
{
    return (lb->rx_head + ANT_LOOPBACK_BUF - lb->rx_tail) % ANT_LOOPBACK_BUF;
}

/* Enqueue a channel response message (0x40): [chan, resp_id, code]. */
static void emit_channel_response(ant_loopback_t *lb, uint8_t chan,
                                  uint8_t resp_id, uint8_t code)
{
    uint8_t data[3] = { chan, resp_id, code };
    uint8_t frame[ANT_MAX_FRAME_LEN];
    size_t n = ant_message_encode(ANT_MSG_CHANNEL_RESPONSE, data, 3,
                                  frame, sizeof(frame));
    rx_push(lb, frame, n);
}

/* Enqueue the startup message the chip sends after reset. */
static void emit_startup(ant_loopback_t *lb)
{
    uint8_t data[1] = { 0x20 }; /* power-on reset flag */
    uint8_t frame[ANT_MAX_FRAME_LEN];
    size_t n = ant_message_encode(ANT_MSG_STARTUP, data, 1, frame, sizeof(frame));
    rx_push(lb, frame, n);
}

static void emit_broadcast(ant_loopback_t *lb, const uint8_t payload[8])
{
    uint8_t frame[ANT_MAX_FRAME_LEN];
    size_t n = ant_build_broadcast_data(lb->channel, payload, frame, sizeof(frame));
    rx_push(lb, frame, n);
}

/* Process one fully-parsed command from the host. */
static void handle_command(ant_loopback_t *lb, const ant_message_t *msg)
{
    lb->last_cmd_id = msg->msg_id;
    lb->cmd_count++;

    uint8_t code = ANT_RESPONSE_NO_ERROR;
    if (lb->fail_next_command) {
        code = 0x15; /* transfer in progress / arbitrary error */
        lb->fail_next_command = false;
    }

    switch (msg->msg_id) {
    case ANT_MSG_RESET_SYSTEM:
        lb->assigned = false;
        lb->opened = false;
        lb->tx_events = 0;
        emit_startup(lb);
        return;

    case ANT_MSG_ASSIGN_CHANNEL:
        if (msg->data_len >= 1) {
            lb->channel = msg->data[0];
            lb->assigned = true;
        }
        emit_channel_response(lb, lb->channel, msg->msg_id, code);
        return;

    case ANT_MSG_SET_CHANNEL_PERIOD:
        if (msg->data_len >= 3) {
            lb->period = (uint16_t)(msg->data[1] | (msg->data[2] << 8));
        }
        emit_channel_response(lb, lb->channel, msg->msg_id, code);
        return;

    case ANT_MSG_OPEN_CHANNEL:
        lb->opened = true;
        emit_channel_response(lb, lb->channel, msg->msg_id, code);
        return;

    case ANT_MSG_CLOSE_CHANNEL:
        lb->opened = false;
        emit_channel_response(lb, lb->channel, msg->msg_id, code);
        /* channel closed event */
        emit_channel_response(lb, lb->channel, 0x01, ANT_EVENT_CHANNEL_CLOSED);
        return;

    /* All other config commands just get an ack. */
    case ANT_MSG_SET_NETWORK_KEY:
    case ANT_MSG_SET_CHANNEL_ID:
    case ANT_MSG_SET_CHANNEL_RF_FREQ:
    case ANT_MSG_SET_SEARCH_TIMEOUT:
    case ANT_MSG_UNASSIGN_CHANNEL:
        emit_channel_response(lb, lb->channel, msg->msg_id, code);
        return;

    case ANT_MSG_BROADCAST_DATA:
    case ANT_MSG_ACKNOWLEDGED_DATA:
        /* Host is a master transmitting; ack acknowledged data. */
        if (msg->msg_id == ANT_MSG_ACKNOWLEDGED_DATA) {
            emit_channel_response(lb, lb->channel, 0x01,
                                  ANT_EVENT_TRANSFER_TX_COMPLETED);
        }
        return;

    default:
        emit_channel_response(lb, lb->channel, msg->msg_id, code);
        return;
    }
}

/* ---- ant_radio_t vtable ---- */

static int lb_write(ant_radio_t *self, const uint8_t *data, size_t len)
{
    ant_loopback_t *lb = (ant_loopback_t *)self->ctx;
    ant_message_t msg;
    for (size_t i = 0; i < len; i++) {
        if (ant_parser_push(&lb->cmd_parser, data[i], &msg)) {
            handle_command(lb, &msg);
        }
    }
    return (int)len;
}

static int lb_read(ant_radio_t *self, uint8_t *buf, size_t cap)
{
    ant_loopback_t *lb = (ant_loopback_t *)self->ctx;
    size_t avail = rx_available(lb);
    size_t n = (avail < cap) ? avail : cap;
    for (size_t i = 0; i < n; i++) {
        buf[i] = lb->rx[lb->rx_tail];
        lb->rx_tail = (lb->rx_tail + 1) % ANT_LOOPBACK_BUF;
    }
    return (int)n;
}

void ant_loopback_init(ant_loopback_t *lb, ant_sim_page_gen_t page_gen,
                       void *page_user)
{
    memset(lb, 0, sizeof(*lb));
    ant_parser_init(&lb->cmd_parser);
    lb->page_gen = page_gen;
    lb->page_user = page_user;
    lb->radio.write = lb_write;
    lb->radio.read = lb_read;
    lb->radio.poll = NULL;
    lb->radio.close = NULL;
    lb->radio.ctx = lb;
}

ant_radio_t *ant_loopback_radio(ant_loopback_t *lb)
{
    return &lb->radio;
}

void ant_loopback_tick(ant_loopback_t *lb)
{
    if (!lb->opened || lb->page_gen == NULL) {
        return;
    }
    uint8_t page[8];
    if (!lb->page_gen(page, lb->page_user)) {
        return;
    }
    lb->tx_events++;
    if (lb->drop_every_n_rx > 0 &&
        (lb->tx_events % (uint32_t)lb->drop_every_n_rx) == 0) {
        return; /* simulate a dropped broadcast */
    }
    emit_broadcast(lb, page);
}
