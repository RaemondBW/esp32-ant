/*
 * ant_air.c - virtual air medium implementing ant_phy_t. See ant_air.h.
 */
#include "ant_air.h"
#include <string.h>

static ant_air_radio_t *R(ant_phy_t *p) { return (ant_air_radio_t *)p; }

static bool air_tune(ant_phy_t *p, uint16_t mhz)
{
    R(p)->mhz = mhz;
    return true;
}

static void air_rx_config(ant_phy_t *p, const ant_phy_rx_cfg_t *cfg)
{
    if (cfg) R(p)->cfg = *cfg;
}

static void air_rx_enable(ant_phy_t *p, bool on)
{
    ant_air_radio_t *r = R(p);
    r->rx_on = on;
    if (!on) { r->q_head = 0; r->q_count = 0; }   /* receiver off: FIFO flushed */
}

static size_t air_rx_poll(ant_phy_t *p, uint8_t *body, size_t cap, uint32_t *rx_time,
                          int8_t *rssi)
{
    ant_air_radio_t *r = R(p);
    if (r->q_count == 0) return 0;
    size_t n = r->q[r->q_head].len;
    if (n > cap) n = cap;
    memcpy(body, r->q[r->q_head].body, n);
    if (rx_time) *rx_time = r->q[r->q_head].time;
    if (rssi)    *rssi    = r->q[r->q_head].rssi;
    r->q_head = (uint8_t)((r->q_head + 1u) % ANT_AIR_RX_QUEUE);
    r->q_count--;
    return n;
}

static bool addr_matches(const ant_phy_rx_cfg_t *cfg, const uint8_t *addr, size_t alen)
{
    if (cfg->addr_len != alen) return false;
    for (size_t i = 0; i < alen; i++) {
        if (((addr[i] ^ cfg->address[i]) & cfg->mask[i]) != 0) return false;
    }
    return true;
}

static bool air_tx(ant_phy_t *p, const uint8_t *frame, size_t len)
{
    ant_air_radio_t *src = R(p);
    ant_air_t *air = src->air;
    if (!air || len < 2 || len > ANT_SB_FRAME_MAX) return false;

    src->tx_count++;
    air->frames++;

    /* Fault injection is counted per transmitted frame. */
    if (air->drop_every_n && (air->frames % air->drop_every_n) == 0) {
        air->dropped++;
        return true;   /* the transmitter never knows */
    }
    uint8_t body[ANT_SB_FRAME_MAX];
    size_t blen = len - 1;                      /* strip the preamble */
    memcpy(body, frame + 1, blen);
    if (air->corrupt_every_n && (air->frames % air->corrupt_every_n) == 0) {
        air->corrupted++;
        body[blen - 1] ^= 0x5A;                 /* smash the CRC */
    }

    for (unsigned i = 0; i < ANT_AIR_MAX_RADIOS; i++) {
        ant_air_radio_t *dst = &air->radio[i];
        if (!dst->attached || dst == src) continue;
        if (!dst->rx_on || dst->mhz != src->mhz ||
            !addr_matches(&dst->cfg, body, dst->cfg.addr_len)) {
            air->not_listening++;
            continue;
        }
        if (dst->q_count >= ANT_AIR_RX_QUEUE) {
            dst->rx_overrun++;
            continue;
        }
        unsigned slot = (dst->q_head + dst->q_count) % ANT_AIR_RX_QUEUE;
        memcpy(dst->q[slot].body, body, blen);
        dst->q[slot].len = (uint8_t)blen;
        dst->q[slot].rssi = src->tx_rssi;
        dst->q[slot].time = air->now;
        dst->q_count++;
        dst->rx_count++;
        air->delivered++;
    }
    return true;
}

void ant_air_init(ant_air_t *air)
{
    memset(air, 0, sizeof(*air));
}

ant_phy_t *ant_air_attach(ant_air_t *air)
{
    for (unsigned i = 0; i < ANT_AIR_MAX_RADIOS; i++) {
        ant_air_radio_t *r = &air->radio[i];
        if (r->attached) continue;
        memset(r, 0, sizeof(*r));
        r->attached = true;
        r->air = air;
        r->tx_rssi = ANT_AIR_DEFAULT_RSSI;
        r->phy.tune      = air_tune;
        r->phy.tx        = air_tx;
        r->phy.rx_config = air_rx_config;
        r->phy.rx_enable = air_rx_enable;
        r->phy.rx_poll   = air_rx_poll;
        r->phy.ctx       = r;
        return &r->phy;
    }
    return NULL;
}

void ant_air_detach(ant_air_t *air, ant_phy_t *phy)
{
    (void)air;
    if (phy) R(phy)->attached = false;
}
