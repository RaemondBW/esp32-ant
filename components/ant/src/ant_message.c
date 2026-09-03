/*
 * ant_message.c - ANT serial message framing (see ant_message.h).
 */
#include "ant_message.h"
#include <string.h>

uint8_t ant_checksum(const uint8_t *buf, size_t len)
{
    uint8_t xor_acc = 0;
    for (size_t i = 0; i < len; i++) {
        xor_acc ^= buf[i];
    }
    return xor_acc;
}

size_t ant_message_encode(uint8_t msg_id, const uint8_t *data, uint8_t data_len,
                          uint8_t *out, size_t out_cap)
{
    if (out == NULL || data_len > ANT_MAX_MSG_DATA) {
        return 0;
    }
    if (data_len > 0 && data == NULL) {
        return 0;
    }
    size_t total = (size_t)data_len + 4u; /* sync,len,id,...,checksum */
    if (out_cap < total) {
        return 0;
    }
    out[0] = ANT_SYNC_TX;
    out[1] = data_len;
    out[2] = msg_id;
    if (data_len > 0) {
        memcpy(&out[3], data, data_len);
    }
    out[3 + data_len] = ant_checksum(out, 3u + data_len);
    return total;
}

bool ant_message_decode(const uint8_t *buf, size_t len,
                        ant_message_t *msg, size_t *consumed)
{
    if (buf == NULL || msg == NULL || len < 4) {
        return false;
    }
    if (buf[0] != ANT_SYNC_TX && buf[0] != 0xA5u) {
        return false;
    }
    uint8_t data_len = buf[1];
    if (data_len > ANT_MAX_MSG_DATA) {
        return false;
    }
    size_t total = (size_t)data_len + 4u;
    if (len < total) {
        return false; /* need more bytes */
    }
    uint8_t expected = ant_checksum(buf, 3u + data_len);
    if (expected != buf[3 + data_len]) {
        return false;
    }
    msg->msg_id = buf[2];
    msg->data_len = data_len;
    if (data_len > 0) {
        memcpy(msg->data, &buf[3], data_len);
    }
    if (consumed) {
        *consumed = total;
    }
    return true;
}

/* Incremental parser state machine. */
enum {
    ST_SYNC = 0,
    ST_LEN,
    ST_ID,
    ST_DATA,
    ST_CHECKSUM
};

void ant_parser_init(ant_parser_t *p)
{
    memset(p, 0, sizeof(*p));
    p->state = ST_SYNC;
}

bool ant_parser_push(ant_parser_t *p, uint8_t byte, ant_message_t *out_msg)
{
    switch (p->state) {
    case ST_SYNC:
        if (byte == ANT_SYNC_TX || byte == 0xA5u) {
            p->running_xor = byte;
            p->state = ST_LEN;
        }
        break;

    case ST_LEN:
        if (byte > ANT_MAX_MSG_DATA) {
            /* invalid length - resync */
            p->state = ST_SYNC;
            break;
        }
        p->length = byte;
        p->running_xor ^= byte;
        p->idx = 0;
        p->state = ST_ID;
        break;

    case ST_ID:
        p->msg_id = byte;
        p->running_xor ^= byte;
        p->state = (p->length == 0) ? ST_CHECKSUM : ST_DATA;
        break;

    case ST_DATA:
        p->buf[p->idx++] = byte;
        p->running_xor ^= byte;
        if (p->idx >= p->length) {
            p->state = ST_CHECKSUM;
        }
        break;

    case ST_CHECKSUM:
        if (byte == p->running_xor) {
            out_msg->msg_id = p->msg_id;
            out_msg->data_len = p->length;
            if (p->length > 0) {
                memcpy(out_msg->data, p->buf, p->length);
            }
            p->state = ST_SYNC;
            return true;
        }
        /* checksum error - resync. If this byte is itself a SYNC, restart. */
        p->state = ST_SYNC;
        if (byte == ANT_SYNC_TX) {
            p->running_xor = byte;
            p->state = ST_LEN;
        }
        break;

    default:
        p->state = ST_SYNC;
        break;
    }
    return false;
}

/* ---- Convenience builders. ---- */

size_t ant_build_reset(uint8_t *out, size_t cap)
{
    uint8_t d[1] = { 0x00 };
    return ant_message_encode(ANT_MSG_RESET_SYSTEM, d, 1, out, cap);
}

size_t ant_build_set_network_key(uint8_t net, const uint8_t key[8],
                                 uint8_t *out, size_t cap)
{
    uint8_t d[9];
    d[0] = net;
    memcpy(&d[1], key, 8);
    return ant_message_encode(ANT_MSG_SET_NETWORK_KEY, d, 9, out, cap);
}

size_t ant_build_assign_channel(uint8_t chan, uint8_t type, uint8_t net,
                                uint8_t *out, size_t cap)
{
    uint8_t d[3] = { chan, type, net };
    return ant_message_encode(ANT_MSG_ASSIGN_CHANNEL, d, 3, out, cap);
}

size_t ant_build_set_channel_id(uint8_t chan, uint16_t dev_num,
                                uint8_t dev_type, uint8_t trans_type,
                                uint8_t *out, size_t cap)
{
    uint8_t d[5] = {
        chan,
        (uint8_t)(dev_num & 0xFF),
        (uint8_t)((dev_num >> 8) & 0xFF),
        dev_type,
        trans_type
    };
    return ant_message_encode(ANT_MSG_SET_CHANNEL_ID, d, 5, out, cap);
}

size_t ant_build_set_rf_freq(uint8_t chan, uint8_t freq, uint8_t *out, size_t cap)
{
    uint8_t d[2] = { chan, freq };
    return ant_message_encode(ANT_MSG_SET_CHANNEL_RF_FREQ, d, 2, out, cap);
}

size_t ant_build_set_period(uint8_t chan, uint16_t period, uint8_t *out, size_t cap)
{
    uint8_t d[3] = {
        chan,
        (uint8_t)(period & 0xFF),
        (uint8_t)((period >> 8) & 0xFF)
    };
    return ant_message_encode(ANT_MSG_SET_CHANNEL_PERIOD, d, 3, out, cap);
}

size_t ant_build_set_search_timeout(uint8_t chan, uint8_t timeout,
                                    uint8_t *out, size_t cap)
{
    uint8_t d[2] = { chan, timeout };
    return ant_message_encode(ANT_MSG_SET_SEARCH_TIMEOUT, d, 2, out, cap);
}

size_t ant_build_open_channel(uint8_t chan, uint8_t *out, size_t cap)
{
    uint8_t d[1] = { chan };
    return ant_message_encode(ANT_MSG_OPEN_CHANNEL, d, 1, out, cap);
}

size_t ant_build_close_channel(uint8_t chan, uint8_t *out, size_t cap)
{
    uint8_t d[1] = { chan };
    return ant_message_encode(ANT_MSG_CLOSE_CHANNEL, d, 1, out, cap);
}

size_t ant_build_broadcast_data(uint8_t chan, const uint8_t payload[8],
                                uint8_t *out, size_t cap)
{
    uint8_t d[9];
    d[0] = chan;
    memcpy(&d[1], payload, 8);
    return ant_message_encode(ANT_MSG_BROADCAST_DATA, d, 9, out, cap);
}

size_t ant_build_acknowledged_data(uint8_t chan, const uint8_t payload[8],
                                   uint8_t *out, size_t cap)
{
    uint8_t d[9];
    d[0] = chan;
    memcpy(&d[1], payload, 8);
    return ant_message_encode(ANT_MSG_ACKNOWLEDGED_DATA, d, 9, out, cap);
}

size_t ant_build_request(uint8_t chan, uint8_t requested_msg_id,
                         uint8_t *out, size_t cap)
{
    uint8_t d[2] = { chan, requested_msg_id };
    return ant_message_encode(ANT_MSG_REQUEST, d, 2, out, cap);
}
