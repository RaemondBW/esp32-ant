/*
 * ant_mac.c - the ANT air-interface MAC (see ant_mac.h).
 *
 * Structure:
 *   - commands          : validate + update channel configuration
 *   - radio ownership   : one channel at a time owns the PHY (tune, rx match)
 *   - slots             : master transmits on its grid; slave opens a window
 *   - handshakes (hold) : reply window / awaiting ack / awaiting next burst
 *   - ant_mac_tick      : drain RX, expire, arbitrate, act, compute deadline
 *
 * Pure, portable C; exercised end-to-end on the host over the virtual air.
 */
#include "ant_mac.h"
#include <string.h>

/* ---- wrap-safe tick arithmetic ---- */
static inline bool t_ge(uint32_t a, uint32_t b) { return (int32_t)(a - b) >= 0; }
static inline bool t_gt(uint32_t a, uint32_t b) { return (int32_t)(a - b) > 0; }
static inline uint32_t t_min(uint32_t a, uint32_t b) { return t_gt(a, b) ? b : a; }

#define FAR_FUTURE 0x40000000u

static inline bool is_master(const ant_mac_channel_t *c)
{
    return (c->type & 0x10u) != 0;
}
static inline bool is_bidir(const ant_mac_channel_t *c)
{
    return c->type == ANT_CHANNEL_TYPE_SLAVE_RX ||
           c->type == ANT_CHANNEL_TYPE_MASTER_TX;
}
static inline bool is_open(const ant_mac_channel_t *c)
{
    return c->state == ANT_MAC_CH_SEARCHING || c->state == ANT_MAC_CH_TRACKING;
}

static const uint8_t ZERO8[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

static void emit_event(ant_mac_t *m, uint8_t ch, uint8_t ev)
{
    if (m->on_event) m->on_event(m, ch, ev, m->user);
}

static void emit_data(ant_mac_t *m, uint8_t ch, uint8_t type, uint8_t seq,
                      const uint8_t *data)
{
    if (m->on_data) m->on_data(m, ch, type, seq, data, m->user);
}

/* ================================ radio ================================ */

static void radio_release(ant_mac_t *m)
{
    if (m->rx_on) {
        ant_phy_rx_enable(m->phy, false);
        m->rx_on = false;
    }
    m->owner = -1;
}

static void radio_tune(ant_mac_t *m, uint16_t mhz)
{
    if (m->tuned_mhz != mhz) {
        ant_phy_tune(m->phy, mhz);
        m->tuned_mhz = mhz;
    }
}

/* Put the radio in receive for channel `idx` with its current address/mask. */
static void radio_listen(ant_mac_t *m, int idx)
{
    ant_mac_channel_t *c = &m->ch[idx];
    ant_phy_rx_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    memcpy(cfg.address, c->link.address, c->link.addr_len);
    memcpy(cfg.mask, c->mask, c->link.addr_len);
    cfg.addr_len = c->link.addr_len;
    cfg.payload_len = c->link.payload_len;

    radio_tune(m, c->link.freq_mhz);
    if (m->owner != idx || memcmp(&cfg, &m->rx_cfg, sizeof(cfg)) != 0) {
        m->rx_cfg = cfg;
        ant_phy_rx_config(m->phy, &cfg);
    }
    if (!m->rx_on) {
        ant_phy_rx_enable(m->phy, true);
        m->rx_on = true;
    }
    m->owner = (int8_t)idx;
}

/* Transmit one frame on channel `idx`. */
static bool radio_send(ant_mac_t *m, int idx, uint8_t ctrl, const uint8_t *data)
{
    ant_mac_channel_t *c = &m->ch[idx];
    uint8_t frame[ANT_SB_FRAME_MAX];
    size_t n = ant_sb_link_build(&c->link, ctrl, data, frame, sizeof(frame));
    if (n == 0) return false;
    radio_tune(m, c->link.freq_mhz);
    m->owner = (int8_t)idx;
    if (!ant_phy_tx(m->phy, frame, n)) return false;
    c->tx_count++;
    return true;
}

/* ============================ channel setup ============================ */

static uint32_t window_for(uint16_t period)
{
    uint32_t w = (uint32_t)period / 32u;
    if (w < ANT_MAC_WINDOW_MIN) w = ANT_MAC_WINDOW_MIN;
    if (w > ANT_MAC_WINDOW_MAX) w = ANT_MAC_WINDOW_MAX;
    return w;
}

static void chan_build_link(ant_mac_t *m, ant_mac_channel_t *c)
{
    ant_sb_link_from_identity(&c->link, c->rf_freq, ANT_SB_LINK_ADDR_LEN,
                              c->device_num, c->device_type, c->trans_type);
    ant_sb_link_set_network(&c->link, m->net_key[c->network]);
    if (is_master(c)) {
        memset(c->mask, 0xFF, sizeof(c->mask));
    } else {
        ant_sb_link_search_mask(ANT_SB_LINK_ADDR_LEN, c->device_num,
                                c->device_type, c->trans_type, c->mask);
    }
}

static void chan_clear_transfers(ant_mac_channel_t *c)
{
    c->hold = ANT_MAC_HOLD_NONE;
    c->xfer_type = 0;
    c->xfer_len = 0;
    c->xfer_off = 0;
    c->bcast_pending = false;
    c->rxb_active = false;
    c->rxb_have_last = false;
}

static void chan_start_search(ant_mac_t *m, ant_mac_channel_t *c, uint32_t now)
{
    (void)m;
    c->state = ANT_MAC_CH_SEARCHING;
    c->hold = ANT_MAC_HOLD_NONE;
    c->slot_done = false;
    c->rxb_active = false;
    c->rxb_have_last = false;
    c->search_infinite = (c->search_timeout == 0xFF);
    c->search_deadline = now + (uint32_t)c->search_timeout * ANT_MAC_SEARCH_UNIT_TICKS;
}

/* Close: back to ASSIGNED, transfers dropped, radio freed. */
static void chan_stop(ant_mac_t *m, int idx)
{
    ant_mac_channel_t *c = &m->ch[idx];
    c->state = ANT_MAC_CH_ASSIGNED;
    chan_clear_transfers(c);
    if (m->owner == idx) radio_release(m);
}

/* ============================== transfers ============================== */

static uint8_t burst_next_seq(uint8_t s)
{
    return (s == 0 || s == 3) ? 1u : (uint8_t)(s + 1);
}

/* Send the current packet of the pending acknowledged/burst transfer and wait
 * for the peer's acknowledgement. */
static void xfer_send_packet(ant_mac_t *m, int idx)
{
    ant_mac_channel_t *c = &m->ch[idx];
    uint8_t ctrl = c->xfer_type;
    if (!is_master(c)) ctrl |= ANT_SB_CTRL_REVERSE;
    if (c->xfer_type == ANT_SB_CTRL_BURST) {
        ctrl |= (uint8_t)(c->xfer_seq & ANT_SB_CTRL_SEQ_MASK);
        if ((uint16_t)c->xfer_off + 8u >= c->xfer_len) ctrl |= ANT_SB_CTRL_LAST;
    }
    radio_send(m, idx, ctrl, &c->xfer_buf[c->xfer_off]);
    c->hold = ANT_MAC_HOLD_AWAIT_ACKRESP;
    c->hold_until = m->now + ANT_MAC_REPLY_WINDOW;
    radio_listen(m, idx);
}

static void xfer_done(ant_mac_t *m, int idx, bool ok)
{
    ant_mac_channel_t *c = &m->ch[idx];
    c->xfer_type = 0;
    c->xfer_len = 0;
    c->xfer_off = 0;
    c->hold = ANT_MAC_HOLD_NONE;
    emit_event(m, (uint8_t)idx, ok ? ANT_EVENT_TRANSFER_TX_COMPLETED
                                   : ANT_EVENT_TRANSFER_TX_FAILED);
}

/* The peer acknowledged our current packet. */
static void xfer_acked(ant_mac_t *m, int idx)
{
    ant_mac_channel_t *c = &m->ch[idx];
    if (c->hold != ANT_MAC_HOLD_AWAIT_ACKRESP || c->xfer_type == 0) return;
    if (c->xfer_type == ANT_SB_CTRL_ACK) {
        xfer_done(m, idx, true);
        return;
    }
    c->xfer_off = (uint8_t)(c->xfer_off + 8u);
    c->xfer_retries = 0;
    if (c->xfer_off >= c->xfer_len) {
        xfer_done(m, idx, true);
        return;
    }
    c->xfer_seq = burst_next_seq(c->xfer_seq);
    xfer_send_packet(m, idx);
}

/* Acknowledge the packet we just received (direction opposite to our role). */
static void send_ack_resp(ant_mac_t *m, int idx)
{
    ant_mac_channel_t *c = &m->ch[idx];
    uint8_t ctrl = ANT_SB_CTRL_ACK_RESP;
    if (!is_master(c)) ctrl |= ANT_SB_CTRL_REVERSE;
    radio_send(m, idx, ctrl, ZERO8);
}

/* A burst packet arrived for channel idx. */
static void burst_rx(ant_mac_t *m, int idx, uint8_t ctrl, const uint8_t *data)
{
    ant_mac_channel_t *c = &m->ch[idx];
    uint8_t seq = ctrl & ANT_SB_CTRL_SEQ_MASK;
    bool last = (ctrl & ANT_SB_CTRL_LAST) != 0;

    /* Retransmission of a packet we already delivered (our ack was lost):
     * acknowledge again, do not deliver twice. */
    if (c->rxb_have_last && seq == c->rxb_last) {
        send_ack_resp(m, idx);
        if (c->rxb_active) {
            c->hold = ANT_MAC_HOLD_AWAIT_BURST_NEXT;
            c->hold_until = m->now + ANT_MAC_BURST_RX_WINDOW;
            radio_listen(m, idx);
        }
        return;
    }
    if (!c->rxb_active) {
        if (seq != 0) return;            /* mid-burst packet of a lost transfer */
        c->rxb_active = true;
        c->rxb_expect = 0;
    } else if (seq != c->rxb_expect) {
        c->rxb_active = false;
        c->hold = ANT_MAC_HOLD_NONE;
        emit_event(m, (uint8_t)idx, ANT_EVENT_TRANSFER_RX_FAILED);
        return;
    }

    emit_data(m, (uint8_t)idx, ANT_MSG_BURST_DATA,
              (uint8_t)(seq | (last ? 4u : 0u)), data);
    c->rxb_last = seq;
    c->rxb_have_last = true;
    c->rxb_expect = burst_next_seq(seq);
    send_ack_resp(m, idx);

    if (last) {
        c->rxb_active = false;
        c->hold = ANT_MAC_HOLD_NONE;
    } else {
        c->hold = ANT_MAC_HOLD_AWAIT_BURST_NEXT;
        c->hold_until = m->now + ANT_MAC_BURST_RX_WINDOW;
        radio_listen(m, idx);
    }
}

/* ================================ slots ================================ */

/* Slave: a slot's forward frame arrived; answer in the reverse direction. */
static void slave_reverse(ant_mac_t *m, int idx)
{
    ant_mac_channel_t *c = &m->ch[idx];
    if (c->xfer_type) {
        xfer_send_packet(m, idx);
    } else if (c->bcast_pending) {
        c->bcast_pending = false;
        radio_send(m, idx, ANT_SB_CTRL_BROADCAST | ANT_SB_CTRL_REVERSE, c->bcast);
        emit_event(m, (uint8_t)idx, ANT_EVENT_TX);
    }
}

static void slave_rx(ant_mac_t *m, int idx, const uint8_t *body, size_t len,
                     uint32_t t, int8_t rssi)
{
    ant_mac_channel_t *c = &m->ch[idx];
    uint8_t ctrl, data[ANT_SB_LINK_DATA_LEN];
    if (!ant_sb_link_recv(&c->link, body, len, &ctrl, data)) {      /* CRC */
        c->crc_fail_count++;
        return;
    }
    if (ctrl & ANT_SB_CTRL_REVERSE) return;   /* another slave's reply */

    if (c->state == ANT_MAC_CH_SEARCHING) {
        /* Proximity search: too far away for a first acquisition. */
        if (c->proximity_armed && rssi < c->proximity_rssi) return;
        /* Acquired: adopt the master's identity (fills any wildcards). */
        memcpy(c->link.address, body, c->link.addr_len);
        ant_sb_link_identity_from_address(body, c->link.addr_len, &c->device_num,
                                          &c->device_type, &c->trans_type);
        memset(c->mask, 0xFF, sizeof(c->mask));
        c->proximity_armed = false;
        c->state = ANT_MAC_CH_TRACKING;
        c->slot_done = false;
        c->window = c->window_base;
    }
    c->last_rssi = rssi;
    if (!c->slot_done) {
        /* First frame of this slot: resynchronize the schedule to the master. */
        c->slot_done = true;
        c->next_slot = t + c->period;
        c->window = c->window_base;
        c->rxb_have_last = false;
    }
    c->last_rx = t;
    c->rx_count++;

    switch (ctrl & ANT_SB_CTRL_TYPE_MASK) {
    case ANT_SB_CTRL_BROADCAST:
        emit_data(m, (uint8_t)idx, ANT_MSG_BROADCAST_DATA, 0, data);
        if (is_bidir(c)) slave_reverse(m, idx);
        break;
    case ANT_SB_CTRL_ACK:
        emit_data(m, (uint8_t)idx, ANT_MSG_ACKNOWLEDGED_DATA, 0, data);
        if (is_bidir(c)) send_ack_resp(m, idx);
        break;
    case ANT_SB_CTRL_BURST:
        if (is_bidir(c)) burst_rx(m, idx, ctrl, data);
        break;
    case ANT_SB_CTRL_ACK_RESP:
        xfer_acked(m, idx);
        break;
    default:
        break;
    }
}

static void master_rx(ant_mac_t *m, int idx, const uint8_t *body, size_t len,
                      int8_t rssi)
{
    ant_mac_channel_t *c = &m->ch[idx];
    uint8_t ctrl, data[ANT_SB_LINK_DATA_LEN];
    if (!ant_sb_link_recv(&c->link, body, len, &ctrl, data)) {
        c->crc_fail_count++;
        return;
    }
    if (!(ctrl & ANT_SB_CTRL_REVERSE)) return;  /* not a reply to us */
    c->rx_count++;
    c->last_rssi = rssi;

    switch (ctrl & ANT_SB_CTRL_TYPE_MASK) {
    case ANT_SB_CTRL_BROADCAST:
        emit_data(m, (uint8_t)idx, ANT_MSG_BROADCAST_DATA, 0, data);
        break;
    case ANT_SB_CTRL_ACK:
        emit_data(m, (uint8_t)idx, ANT_MSG_ACKNOWLEDGED_DATA, 0, data);
        send_ack_resp(m, idx);
        break;
    case ANT_SB_CTRL_BURST:
        burst_rx(m, idx, ctrl, data);
        break;
    case ANT_SB_CTRL_ACK_RESP:
        xfer_acked(m, idx);
        break;
    default:
        break;
    }
}

/* Master: its slot came due - transmit. */
static void master_slot(ant_mac_t *m, int idx)
{
    ant_mac_channel_t *c = &m->ch[idx];
    c->rxb_have_last = false;
    if (c->xfer_type) {
        xfer_send_packet(m, idx);       /* ack'd or burst instead of broadcast */
        return;
    }
    radio_send(m, idx, ANT_SB_CTRL_BROADCAST, c->bcast);
    emit_event(m, (uint8_t)idx, ANT_EVENT_TX);
    if (is_bidir(c)) {
        c->hold = ANT_MAC_HOLD_REPLY_WINDOW;
        c->hold_until = m->now + ANT_MAC_REPLY_WINDOW;
        radio_listen(m, idx);
    } else {
        c->hold = ANT_MAC_HOLD_NONE;
    }
}

static void hold_expired(ant_mac_t *m, int idx)
{
    ant_mac_channel_t *c = &m->ch[idx];
    switch (c->hold) {
    case ANT_MAC_HOLD_REPLY_WINDOW:
        c->hold = ANT_MAC_HOLD_NONE;
        break;
    case ANT_MAC_HOLD_AWAIT_ACKRESP:
        if (c->xfer_type == ANT_SB_CTRL_ACK) {
            xfer_done(m, idx, false);       /* acknowledged data: one attempt */
        } else if (++c->xfer_retries > ANT_MAC_BURST_RETRIES) {
            xfer_done(m, idx, false);
        } else {
            xfer_send_packet(m, idx);       /* retransmit this burst packet */
        }
        break;
    case ANT_MAC_HOLD_AWAIT_BURST_NEXT:
        c->rxb_active = false;
        c->hold = ANT_MAC_HOLD_NONE;
        emit_event(m, (uint8_t)idx, ANT_EVENT_TRANSFER_RX_FAILED);
        break;
    default:
        c->hold = ANT_MAC_HOLD_NONE;
        break;
    }
}

/* ================================ tick ================================= */

uint32_t ant_mac_tick(ant_mac_t *m, uint32_t now)
{
    m->now = now;

    /* 1. Drain the receiver for whichever channel owns it. */
    if (m->owner >= 0 && m->rx_on) {
        uint8_t body[ANT_SB_FRAME_MAX];
        uint32_t t;
        int8_t rssi;
        size_t n;
        while ((n = ant_phy_rx_poll(m->phy, body, sizeof(body), &t, &rssi)) > 0) {
            int idx = m->owner;
            if (idx < 0 || !is_open(&m->ch[idx])) break;
            if (is_master(&m->ch[idx])) master_rx(m, idx, body, n, rssi);
            else                        slave_rx(m, idx, body, n, t, rssi);
        }
    }

    /* 2. Time-driven state changes. */
    for (int i = 0; i < (int)ANT_MAC_MAX_CHANNELS; i++) {
        ant_mac_channel_t *c = &m->ch[i];
        if (!is_open(c)) continue;

        /* A channel opened before this tick has no idea what time it is:
         * anchor its schedule (first slot / search timeout) now. */
        if (!c->armed) {
            c->armed = true;
            if (is_master(c)) c->next_slot = now;
            else chan_start_search(m, c, now);
        }

        if (c->hold && t_ge(now, c->hold_until)) hold_expired(m, i);

        if (is_master(c)) {
            if (t_gt(now, c->next_slot + c->window_base)) {
                if (c->tx_count == 0 || t_gt(now, c->next_slot + c->period)) {
                    /* Not started yet, or fell a whole period behind: resync
                     * the grid to now rather than report a pile of misses. */
                    c->next_slot = now;
                } else {
                    /* Too late for the slave's window: the slot is lost. */
                    while (t_gt(now, c->next_slot + c->window_base)) {
                        emit_event(m, (uint8_t)i, ANT_EVENT_CHANNEL_COLLISION);
                        c->next_slot += c->period;
                    }
                }
            }
        } else if (c->state == ANT_MAC_CH_SEARCHING) {
            if (!c->search_infinite && t_ge(now, c->search_deadline)) {
                emit_event(m, (uint8_t)i, ANT_EVENT_RX_SEARCH_TIMEOUT);
                chan_stop(m, i);
                emit_event(m, (uint8_t)i, ANT_EVENT_CHANNEL_CLOSED);
            }
        } else { /* slave tracking */
            if (c->slot_done && t_ge(now, c->next_slot - c->window)) {
                c->slot_done = false;   /* next window opens */
            }
            if (!c->slot_done && t_gt(now, c->next_slot + c->window)) {
                /* Missed this slot. */
                c->miss_count++;
                emit_event(m, (uint8_t)i, ANT_EVENT_RX_FAIL);
                c->next_slot += c->period;
                c->window *= 2;
                if (c->window > (uint32_t)c->period / 4u) c->window = (uint32_t)c->period / 4u;
                if (c->hold) { c->hold = ANT_MAC_HOLD_NONE; }
                if (t_gt(now, c->last_rx + ANT_MAC_DROP_TO_SEARCH_TICKS)) {
                    emit_event(m, (uint8_t)i, ANT_EVENT_RX_FAIL_GO_TO_SEARCH);
                    chan_start_search(m, c, now);  /* keeps the learned id */
                }
            }
        }
    }

    /* 3. Arbitrate the radio: hold > master TX due > tracking window > search. */
    int best = -1, best_pri = 0;
    uint32_t best_key = 0;
    for (int i = 0; i < (int)ANT_MAC_MAX_CHANNELS; i++) {
        ant_mac_channel_t *c = &m->ch[i];
        if (!is_open(c)) continue;
        int pri = 0;
        uint32_t key = 0;
        if (c->hold) {
            pri = 4; key = c->hold_until;
        } else if (is_master(c)) {
            if (t_ge(now, c->next_slot)) { pri = 3; key = c->next_slot; }
        } else if (c->state == ANT_MAC_CH_TRACKING) {
            if (!c->slot_done && t_ge(now, c->next_slot - c->window)) {
                pri = 2; key = c->next_slot;
            }
        } else {
            /* Searching (or assigned-idle) channels share the radio in turns
             * of ~250 ms (8192 ticks): the earlier "lowest index wins" rule
             * let one channel searching for an absent sensor hold the
             * receiver forever, starving every other search on the node
             * (seen 2026-09-02: an HR channel waiting for a strap that was
             * off kept a wildcard scan from ever hearing anything). */
            uint32_t rot = (now >> 13) % ANT_MAC_MAX_CHANNELS;
            pri = 1; key = (uint32_t)(((uint32_t)i + ANT_MAC_MAX_CHANNELS - rot) % ANT_MAC_MAX_CHANNELS);
        }
        if (pri > best_pri || (pri == best_pri && pri > 0 && t_gt(best_key, key))) {
            best = i; best_pri = pri; best_key = key;
        }
    }

    if (best < 0) {
        radio_release(m);
    } else if (best_pri == 3) {
        ant_mac_channel_t *c = &m->ch[best];
        c->next_slot += c->period;
        master_slot(m, best);
        if (!c->hold && m->owner == best) radio_release(m);
    } else {
        radio_listen(m, best);
    }

    /* 4. When must we be called again? */
    uint32_t next = now + FAR_FUTURE;
    bool wants_rx = m->rx_on;   /* someone needs the receiver drained/handed over */
    for (int i = 0; i < (int)ANT_MAC_MAX_CHANNELS; i++) {
        ant_mac_channel_t *c = &m->ch[i];
        if (!is_open(c)) continue;
        if (c->hold) next = t_min(next, c->hold_until);
        if (is_master(c)) {
            if (!c->hold) next = t_min(next, c->next_slot);
        } else if (c->state == ANT_MAC_CH_TRACKING) {
            if (c->slot_done) {
                next = t_min(next, c->next_slot - c->window);
            } else {
                next = t_min(next, c->next_slot + c->window + 1u);
                wants_rx = true;
            }
        } else {
            wants_rx = true;
            if (!c->search_infinite) next = t_min(next, c->search_deadline);
        }
    }
    if (wants_rx) next = t_min(next, now + ANT_MAC_POLL_INTERVAL);
    if (!t_gt(next, now)) next = now + 1;
    return next;
}

/* ============================== lifecycle ============================== */

void ant_mac_init(ant_mac_t *m, ant_phy_t *phy, ant_mac_data_cb_t on_data,
                  ant_mac_event_cb_t on_event, void *user)
{
    memset(m, 0, sizeof(*m));
    m->phy = phy;
    m->on_data = on_data;
    m->on_event = on_event;
    m->user = user;
    m->owner = -1;
}

void ant_mac_reset(ant_mac_t *m)
{
    radio_release(m);
    memset(m->ch, 0, sizeof(m->ch));
    memset(m->net_key, 0, sizeof(m->net_key));
    m->tuned_mhz = 0;
    memset(&m->rx_cfg, 0, sizeof(m->rx_cfg));
}

/* ============================== commands =============================== */

static ant_mac_channel_t *chan_of(ant_mac_t *m, uint8_t ch)
{
    return (ch < ANT_MAC_MAX_CHANNELS) ? &m->ch[ch] : NULL;
}

uint8_t ant_mac_set_network_key(ant_mac_t *m, uint8_t net, const uint8_t key[8])
{
    if (net >= ANT_MAC_MAX_NETWORKS || !key) return ANT_RESPONSE_INVALID_NETWORK_NUMBER;
    memcpy(m->net_key[net], key, 8);
    return ANT_RESPONSE_NO_ERROR;
}

uint8_t ant_mac_assign_channel(ant_mac_t *m, uint8_t ch, uint8_t type, uint8_t net)
{
    ant_mac_channel_t *c = chan_of(m, ch);
    if (!c) return ANT_RESPONSE_INVALID_PARAMETER;
    if (c->state != ANT_MAC_CH_UNASSIGNED) return ANT_RESPONSE_CHANNEL_IN_WRONG_STATE;
    if (net >= ANT_MAC_MAX_NETWORKS) return ANT_RESPONSE_INVALID_NETWORK_NUMBER;
    switch (type) {
    case ANT_CHANNEL_TYPE_SLAVE_RX:
    case ANT_CHANNEL_TYPE_MASTER_TX:
    case ANT_CHANNEL_TYPE_SLAVE_RX_ONLY:
    case ANT_CHANNEL_TYPE_MASTER_TX_ONLY:
        break;
    default:
        return ANT_RESPONSE_INVALID_PARAMETER;   /* shared channels unsupported */
    }
    memset(c, 0, sizeof(*c));
    c->state = ANT_MAC_CH_ASSIGNED;
    c->type = type;
    c->network = net;
    c->rf_freq = 66;            /* ANT default 2466 MHz */
    c->period = 8192;           /* ANT default 4 Hz */
    c->search_timeout = 10;     /* ANT default 25 s */
    c->trans_type = 1;
    return ANT_RESPONSE_NO_ERROR;
}

uint8_t ant_mac_unassign_channel(ant_mac_t *m, uint8_t ch)
{
    ant_mac_channel_t *c = chan_of(m, ch);
    if (!c) return ANT_RESPONSE_INVALID_PARAMETER;
    if (c->state != ANT_MAC_CH_ASSIGNED) return ANT_RESPONSE_CHANNEL_IN_WRONG_STATE;
    memset(c, 0, sizeof(*c));
    return ANT_RESPONSE_NO_ERROR;
}

uint8_t ant_mac_set_channel_id(ant_mac_t *m, uint8_t ch, uint16_t device_num,
                               uint8_t device_type, uint8_t trans_type)
{
    ant_mac_channel_t *c = chan_of(m, ch);
    if (!c) return ANT_RESPONSE_INVALID_PARAMETER;
    if (c->state != ANT_MAC_CH_ASSIGNED) return ANT_RESPONSE_CHANNEL_IN_WRONG_STATE;
    if (is_master(c) && device_num == 0) return ANT_RESPONSE_INVALID_PARAMETER;
    c->device_num = device_num;
    c->device_type = device_type;
    c->trans_type = trans_type;
    c->id_set = true;
    return ANT_RESPONSE_NO_ERROR;
}

uint8_t ant_mac_set_channel_period(ant_mac_t *m, uint8_t ch, uint16_t period)
{
    ant_mac_channel_t *c = chan_of(m, ch);
    if (!c) return ANT_RESPONSE_INVALID_PARAMETER;
    if (c->state == ANT_MAC_CH_UNASSIGNED) return ANT_RESPONSE_CHANNEL_IN_WRONG_STATE;
    if (period < 64) return ANT_RESPONSE_INVALID_PARAMETER;
    c->period = period;
    c->window_base = window_for(period);
    return ANT_RESPONSE_NO_ERROR;
}

uint8_t ant_mac_set_channel_rf_freq(ant_mac_t *m, uint8_t ch, uint8_t rf_freq)
{
    ant_mac_channel_t *c = chan_of(m, ch);
    if (!c) return ANT_RESPONSE_INVALID_PARAMETER;
    if (c->state == ANT_MAC_CH_UNASSIGNED) return ANT_RESPONSE_CHANNEL_IN_WRONG_STATE;
    if (rf_freq > 124) return ANT_RESPONSE_INVALID_PARAMETER;
    c->rf_freq = rf_freq;
    if (is_open(c)) c->link.freq_mhz = (uint16_t)(2400u + rf_freq);
    return ANT_RESPONSE_NO_ERROR;
}

uint8_t ant_mac_set_search_timeout(ant_mac_t *m, uint8_t ch, uint8_t timeout)
{
    ant_mac_channel_t *c = chan_of(m, ch);
    if (!c) return ANT_RESPONSE_INVALID_PARAMETER;
    if (c->state == ANT_MAC_CH_UNASSIGNED) return ANT_RESPONSE_CHANNEL_IN_WRONG_STATE;
    c->search_timeout = timeout;
    return ANT_RESPONSE_NO_ERROR;
}

uint8_t ant_mac_set_proximity_search(ant_mac_t *m, uint8_t ch, int8_t min_rssi)
{
    ant_mac_channel_t *c = chan_of(m, ch);
    if (!c) return ANT_RESPONSE_INVALID_PARAMETER;
    if (c->state == ANT_MAC_CH_UNASSIGNED) return ANT_RESPONSE_CHANNEL_IN_WRONG_STATE;
    c->proximity_rssi = min_rssi;
    return ANT_RESPONSE_NO_ERROR;
}

uint8_t ant_mac_set_tx_power(ant_mac_t *m, uint8_t ch, uint8_t power)
{
    ant_mac_channel_t *c = chan_of(m, ch);
    if (!c) return ANT_RESPONSE_INVALID_PARAMETER;
    if (c->state == ANT_MAC_CH_UNASSIGNED) return ANT_RESPONSE_CHANNEL_IN_WRONG_STATE;
    if (power > 4) return ANT_RESPONSE_INVALID_PARAMETER;
    c->tx_power = power;
    return ANT_RESPONSE_NO_ERROR;
}

uint8_t ant_mac_open_channel(ant_mac_t *m, uint8_t ch)
{
    ant_mac_channel_t *c = chan_of(m, ch);
    if (!c) return ANT_RESPONSE_INVALID_PARAMETER;
    if (c->state != ANT_MAC_CH_ASSIGNED) return ANT_RESPONSE_CHANNEL_IN_WRONG_STATE;
    if (!c->id_set) return ANT_RESPONSE_CHANNEL_ID_NOT_SET;

    chan_build_link(m, c);
    chan_clear_transfers(c);
    c->window_base = window_for(c->period);
    c->window = c->window_base;
    c->miss_count = 0;
    c->armed = false;
    c->proximity_armed = (c->proximity_rssi != 0);
    if (is_master(c)) {
        c->state = ANT_MAC_CH_TRACKING;
        c->next_slot = m->now;          /* first slot right away */
    } else {
        chan_start_search(m, c, m->now);
    }
    return ANT_RESPONSE_NO_ERROR;
}

uint8_t ant_mac_close_channel(ant_mac_t *m, uint8_t ch)
{
    ant_mac_channel_t *c = chan_of(m, ch);
    if (!c) return ANT_RESPONSE_INVALID_PARAMETER;
    if (!is_open(c)) return ANT_RESPONSE_CHANNEL_IN_WRONG_STATE;
    if (c->xfer_type) emit_event(m, ch, ANT_EVENT_TRANSFER_TX_FAILED);
    chan_stop(m, ch);
    emit_event(m, ch, ANT_EVENT_CHANNEL_CLOSED);
    return ANT_RESPONSE_NO_ERROR;
}

uint8_t ant_mac_send_broadcast(ant_mac_t *m, uint8_t ch, const uint8_t data[8])
{
    ant_mac_channel_t *c = chan_of(m, ch);
    if (!c || !data) return ANT_RESPONSE_INVALID_PARAMETER;
    if (c->state == ANT_MAC_CH_UNASSIGNED) return ANT_RESPONSE_CHANNEL_IN_WRONG_STATE;
    memcpy(c->bcast, data, 8);
    if (is_master(c)) c->bcast_loaded = true;
    else if (is_bidir(c)) c->bcast_pending = true;
    else return ANT_RESPONSE_CHANNEL_IN_WRONG_STATE;   /* RX-only slave */
    return ANT_RESPONSE_NO_ERROR;
}

uint8_t ant_mac_send_acknowledged(ant_mac_t *m, uint8_t ch, const uint8_t data[8])
{
    ant_mac_channel_t *c = chan_of(m, ch);
    if (!c || !data) return ANT_RESPONSE_INVALID_PARAMETER;
    if (c->state == ANT_MAC_CH_UNASSIGNED) return ANT_RESPONSE_CHANNEL_IN_WRONG_STATE;
    if (!is_bidir(c)) return ANT_RESPONSE_CHANNEL_IN_WRONG_STATE;
    if (!is_open(c)) return ANT_RESPONSE_CHANNEL_NOT_OPENED;
    if (c->xfer_type) return ANT_RESPONSE_TRANSFER_IN_PROGRESS;
    memcpy(c->xfer_buf, data, 8);
    c->xfer_type = ANT_SB_CTRL_ACK;
    c->xfer_len = 8;
    c->xfer_off = 0;
    c->xfer_retries = 0;
    return ANT_RESPONSE_NO_ERROR;
}

uint8_t ant_mac_send_burst(ant_mac_t *m, uint8_t ch, const uint8_t *data, size_t len)
{
    ant_mac_channel_t *c = chan_of(m, ch);
    if (!c || !data) return ANT_RESPONSE_INVALID_PARAMETER;
    if (len == 0 || len > ANT_MAC_BURST_MAX_BYTES) return ANT_RESPONSE_INVALID_PARAMETER;
    if (c->state == ANT_MAC_CH_UNASSIGNED) return ANT_RESPONSE_CHANNEL_IN_WRONG_STATE;
    if (!is_bidir(c)) return ANT_RESPONSE_CHANNEL_IN_WRONG_STATE;
    if (!is_open(c)) return ANT_RESPONSE_CHANNEL_NOT_OPENED;
    if (c->xfer_type) return ANT_RESPONSE_TRANSFER_IN_PROGRESS;
    memset(c->xfer_buf, 0, sizeof(c->xfer_buf));
    memcpy(c->xfer_buf, data, len);
    c->xfer_type = ANT_SB_CTRL_BURST;
    c->xfer_len = (uint8_t)((len + 7u) & ~7u);   /* whole 8-byte packets */
    c->xfer_off = 0;
    c->xfer_seq = 0;
    c->xfer_retries = 0;
    return ANT_RESPONSE_NO_ERROR;
}

/* =============================== queries =============================== */

uint8_t ant_mac_channel_status(const ant_mac_t *m, uint8_t ch)
{
    if (ch >= ANT_MAC_MAX_CHANNELS) return 0;
    const ant_mac_channel_t *c = &m->ch[ch];
    return (uint8_t)((c->state & ANT_STATUS_STATE_MASK) |
                     ((c->network & 0x03u) << 2) | (c->type & 0xF0u));
}

bool ant_mac_get_channel_id(const ant_mac_t *m, uint8_t ch, uint16_t *device_num,
                            uint8_t *device_type, uint8_t *trans_type)
{
    if (ch >= ANT_MAC_MAX_CHANNELS) return false;
    const ant_mac_channel_t *c = &m->ch[ch];
    if (c->state == ANT_MAC_CH_UNASSIGNED) return false;
    if (device_num)  *device_num  = c->device_num;
    if (device_type) *device_type = c->device_type;
    if (trans_type)  *trans_type  = c->trans_type;
    return true;
}

const char *ant_mac_event_name(uint8_t ev)
{
    switch (ev) {
    case ANT_EVENT_RX_SEARCH_TIMEOUT:      return "RX_SEARCH_TIMEOUT";
    case ANT_EVENT_RX_FAIL:                return "RX_FAIL";
    case ANT_EVENT_TX:                     return "TX";
    case ANT_EVENT_TRANSFER_RX_FAILED:     return "TRANSFER_RX_FAILED";
    case ANT_EVENT_TRANSFER_TX_COMPLETED:  return "TRANSFER_TX_COMPLETED";
    case ANT_EVENT_TRANSFER_TX_FAILED:     return "TRANSFER_TX_FAILED";
    case ANT_EVENT_CHANNEL_CLOSED:         return "CHANNEL_CLOSED";
    case ANT_EVENT_RX_FAIL_GO_TO_SEARCH:   return "RX_FAIL_GO_TO_SEARCH";
    case ANT_EVENT_CHANNEL_COLLISION:      return "CHANNEL_COLLISION";
    default:                               return "?";
    }
}

const char *ant_mac_state_name(ant_mac_ch_state_t s)
{
    switch (s) {
    case ANT_MAC_CH_UNASSIGNED: return "UNASSIGNED";
    case ANT_MAC_CH_ASSIGNED:   return "ASSIGNED";
    case ANT_MAC_CH_SEARCHING:  return "SEARCHING";
    case ANT_MAC_CH_TRACKING:   return "TRACKING";
    default:                    return "?";
    }
}
