/*
 * ant_channel.c - ANT channel state machine (see ant_channel.h).
 */
#include "ant_channel.h"
#include <string.h>

/* Published ANT+ managed network key. */
const uint8_t ANTPLUS_NETWORK_KEY[8] =
    { 0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45 };

static int send_frame(ant_stack_t *st, const uint8_t *frame, size_t len)
{
    if (len == 0) {
        return -1;
    }
    return ant_radio_write(st->radio, frame, len);
}

void ant_stack_init(ant_stack_t *st, ant_radio_t *radio,
                    const ant_channel_config_t *cfg)
{
    memset(st, 0, sizeof(*st));
    st->radio = radio;
    st->cfg = *cfg;
    if (st->cfg.network_key == NULL) {
        st->cfg.network_key = ANTPLUS_NETWORK_KEY;
    }
    st->state = ANT_CH_IDLE;
    ant_parser_init(&st->parser);
}

void ant_stack_set_callbacks(ant_stack_t *st, ant_data_cb_t on_data,
                             ant_event_cb_t on_event, void *user)
{
    st->on_data = on_data;
    st->on_event = on_event;
    st->user = user;
}

/* Emit the control message for the *current* state and mark awaiting-response. */
static void emit_current_state_command(ant_stack_t *st)
{
    uint8_t f[ANT_MAX_FRAME_LEN];
    size_t n = 0;
    const ant_channel_config_t *c = &st->cfg;

    switch (st->state) {
    case ANT_CH_RESETTING:
        n = ant_build_reset(f, sizeof(f));
        st->last_cmd_msg_id = ANT_MSG_RESET_SYSTEM;
        break;
    case ANT_CH_SET_KEY:
        n = ant_build_set_network_key(c->network, c->network_key, f, sizeof(f));
        st->last_cmd_msg_id = ANT_MSG_SET_NETWORK_KEY;
        break;
    case ANT_CH_ASSIGN:
        n = ant_build_assign_channel(c->channel, c->type, c->network, f, sizeof(f));
        st->last_cmd_msg_id = ANT_MSG_ASSIGN_CHANNEL;
        break;
    case ANT_CH_SET_ID:
        n = ant_build_set_channel_id(c->channel, c->device_num, c->device_type,
                                     c->trans_type, f, sizeof(f));
        st->last_cmd_msg_id = ANT_MSG_SET_CHANNEL_ID;
        break;
    case ANT_CH_SET_FREQ:
        n = ant_build_set_rf_freq(c->channel, c->rf_freq, f, sizeof(f));
        st->last_cmd_msg_id = ANT_MSG_SET_CHANNEL_RF_FREQ;
        break;
    case ANT_CH_SET_PERIOD:
        n = ant_build_set_period(c->channel, c->period, f, sizeof(f));
        st->last_cmd_msg_id = ANT_MSG_SET_CHANNEL_PERIOD;
        break;
    case ANT_CH_SET_TIMEOUT:
        n = ant_build_set_search_timeout(c->channel, c->search_timeout, f, sizeof(f));
        st->last_cmd_msg_id = ANT_MSG_SET_SEARCH_TIMEOUT;
        break;
    case ANT_CH_OPEN:
        n = ant_build_open_channel(c->channel, f, sizeof(f));
        st->last_cmd_msg_id = ANT_MSG_OPEN_CHANNEL;
        break;
    default:
        return;
    }
    send_frame(st, f, n);
    st->awaiting_response = true;
}

void ant_stack_start(ant_stack_t *st)
{
    st->state = ANT_CH_RESETTING;
    ant_parser_init(&st->parser);
    emit_current_state_command(st);
}

/* Move to the next state in the bring-up sequence and emit its command. */
static void advance_state(ant_stack_t *st)
{
    switch (st->state) {
    case ANT_CH_RESETTING:  st->state = ANT_CH_SET_KEY;    break;
    case ANT_CH_SET_KEY:    st->state = ANT_CH_ASSIGN;     break;
    case ANT_CH_ASSIGN:     st->state = ANT_CH_SET_ID;     break;
    case ANT_CH_SET_ID:     st->state = ANT_CH_SET_FREQ;   break;
    case ANT_CH_SET_FREQ:   st->state = ANT_CH_SET_PERIOD; break;
    case ANT_CH_SET_PERIOD:
        /* search timeout only meaningful for slave channels */
        st->state = (st->cfg.type == ANT_CHANNEL_TYPE_MASTER_TX)
                    ? ANT_CH_OPEN : ANT_CH_SET_TIMEOUT;
        break;
    case ANT_CH_SET_TIMEOUT: st->state = ANT_CH_OPEN;      break;
    case ANT_CH_OPEN:
        st->state = ANT_CH_RUNNING;
        st->awaiting_response = false;
        return; /* running - nothing more to emit */
    default:
        return;
    }
    emit_current_state_command(st);
}

void ant_stack_handle_message(ant_stack_t *st, const ant_message_t *msg)
{
    switch (msg->msg_id) {
    case ANT_MSG_STARTUP:
        /* Reset notification. If we're waiting on reset, proceed. */
        if (st->state == ANT_CH_RESETTING && st->awaiting_response) {
            advance_state(st);
        }
        break;

    case ANT_MSG_CHANNEL_RESPONSE: {
        /* data: [channel, response_msg_id, code] */
        if (msg->data_len < 3) {
            break;
        }
        uint8_t chan = msg->data[0];
        uint8_t resp_id = msg->data[1];
        uint8_t code = msg->data[2];

        if (st->on_event) {
            st->on_event(st, chan, resp_id, code, st->user);
        }

        if (resp_id == 0x01) {
            /* This is a channel EVENT (not a command response). */
            switch (code) {
            case ANT_EVENT_TX:
                /* master: time to (re)load the broadcast buffer */
                if (st->has_tx_payload) {
                    ant_stack_send_broadcast(st, st->tx_payload);
                }
                break;
            case ANT_EVENT_CHANNEL_CLOSED:
                /* A search timeout is followed by CHANNEL_CLOSED; keep ERROR. */
                if (st->state != ANT_CH_ERROR) {
                    st->state = ANT_CH_CLOSED;
                }
                break;
            case ANT_EVENT_RX_SEARCH_TIMEOUT:
                st->state = ANT_CH_ERROR;
                break;
            default:
                break;
            }
            break;
        }

        /* Otherwise it's a response to a config command. */
        if (st->awaiting_response && resp_id == st->last_cmd_msg_id) {
            st->awaiting_response = false;
            if (code == ANT_RESPONSE_NO_ERROR) {
                advance_state(st);
            } else {
                st->state = ANT_CH_ERROR;
            }
        }
        break;
    }

    case ANT_MSG_BROADCAST_DATA:
    case ANT_MSG_ACKNOWLEDGED_DATA: {
        /* data: [channel, 8 bytes payload] */
        if (msg->data_len < 9) {
            break;
        }
        st->rx_count++;
        if (st->on_data) {
            st->on_data(st, msg->data[0], &msg->data[1], st->user);
        }
        break;
    }

    default:
        break;
    }
}

int ant_stack_step(ant_stack_t *st)
{
    uint8_t buf[64];
    ant_message_t msg;
    int frames = 0;

    ant_radio_poll(st->radio);

    int n;
    while ((n = ant_radio_read(st->radio, buf, sizeof(buf))) > 0) {
        for (int i = 0; i < n; i++) {
            if (ant_parser_push(&st->parser, buf[i], &msg)) {
                ant_stack_handle_message(st, &msg);
                frames++;
            }
        }
        if ((size_t)n < sizeof(buf)) {
            break; /* drained */
        }
    }
    return frames;
}

void ant_stack_set_tx_payload(ant_stack_t *st, const uint8_t payload[8])
{
    memcpy(st->tx_payload, payload, 8);
    st->has_tx_payload = true;
}

bool ant_stack_send_broadcast(ant_stack_t *st, const uint8_t payload[8])
{
    uint8_t f[ANT_MAX_FRAME_LEN];
    size_t n = ant_build_broadcast_data(st->cfg.channel, payload, f, sizeof(f));
    if (n == 0) {
        return false;
    }
    if (send_frame(st, f, n) < 0) {
        return false;
    }
    st->tx_count++;
    return true;
}

bool ant_stack_is_running(const ant_stack_t *st)
{
    return st->state == ANT_CH_RUNNING;
}

const char *ant_channel_state_name(ant_channel_state_t s)
{
    switch (s) {
    case ANT_CH_IDLE:       return "IDLE";
    case ANT_CH_RESETTING:  return "RESETTING";
    case ANT_CH_SET_KEY:    return "SET_KEY";
    case ANT_CH_ASSIGN:     return "ASSIGN";
    case ANT_CH_SET_ID:     return "SET_ID";
    case ANT_CH_SET_FREQ:   return "SET_FREQ";
    case ANT_CH_SET_PERIOD: return "SET_PERIOD";
    case ANT_CH_SET_TIMEOUT:return "SET_TIMEOUT";
    case ANT_CH_OPEN:       return "OPEN";
    case ANT_CH_RUNNING:    return "RUNNING";
    case ANT_CH_CLOSED:     return "CLOSED";
    case ANT_CH_ERROR:      return "ERROR";
    default:                return "?";
    }
}

void ant_channel_config_antplus_slave(ant_channel_config_t *cfg, uint8_t channel,
                                       uint8_t device_type, uint16_t period)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->channel = channel;
    cfg->type = ANT_CHANNEL_TYPE_SLAVE_RX;
    cfg->device_num = ANT_WILDCARD_DEVICE_NUM;
    cfg->device_type = device_type;
    cfg->trans_type = ANT_WILDCARD_TRANS_TYPE;
    cfg->rf_freq = ANTPLUS_RF_FREQ;
    cfg->period = period;
    cfg->search_timeout = 0xFF; /* infinite search */
    cfg->network_key = ANTPLUS_NETWORK_KEY;
    cfg->network = ANTPLUS_NETWORK;
}

void ant_channel_config_antplus_master(ant_channel_config_t *cfg, uint8_t channel,
                                        uint16_t device_num, uint8_t device_type,
                                        uint8_t trans_type, uint16_t period)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->channel = channel;
    cfg->type = ANT_CHANNEL_TYPE_MASTER_TX;
    cfg->device_num = device_num;
    cfg->device_type = device_type;
    cfg->trans_type = trans_type ? trans_type : 1;
    cfg->rf_freq = ANTPLUS_RF_FREQ;
    cfg->period = period;
    cfg->search_timeout = 0;
    cfg->network_key = ANTPLUS_NETWORK_KEY;
    cfg->network = ANTPLUS_NETWORK;
}
