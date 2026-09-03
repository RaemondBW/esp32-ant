/*
 * ant_air.h - a virtual 2.4 GHz air shared by several ant_phy radios.
 *
 * Every radio attached to one ant_air_t is an ant_phy_t. A frame transmitted
 * by one radio is delivered, minus its preamble, to every other radio that is
 * listening on the same frequency with a matching address (after the receive
 * mask) at the air's current time. It models the things the MAC has to cope
 * with - the receiver being off, a different channel, an address mismatch,
 * a lost frame, a corrupted frame - exactly and deterministically, so the
 * whole ANT protocol can be exercised on the host and as an on-target
 * self-test. Each radio has a fixed level at which the others hear it
 * (tx_rssi), enough to exercise proximity search; propagation delay and
 * capture effect are not modelled.
 *
 * Time is supplied by the caller: set air->now before ticking the MACs.
 */
#ifndef ANT_AIR_H
#define ANT_AIR_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "ant_phy.h"
#include "ant_phy_shockburst.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ANT_AIR_MAX_RADIOS   8u
#define ANT_AIR_RX_QUEUE     4u
#define ANT_AIR_DEFAULT_RSSI (-50)

typedef struct ant_air ant_air_t;

typedef struct {
    ant_phy_t   phy;          /* the ant_phy_t handed to the MAC (must be first) */
    ant_air_t  *air;
    bool        attached;
    uint16_t    mhz;
    bool        rx_on;
    ant_phy_rx_cfg_t cfg;
    int8_t      tx_rssi;      /* level (dBm) at which the others hear this radio;
                                 ANT_AIR_DEFAULT_RSSI after attach */
    struct {
        uint8_t  body[ANT_SB_FRAME_MAX];
        uint8_t  len;
        int8_t   rssi;
        uint32_t time;
    } q[ANT_AIR_RX_QUEUE];
    uint8_t     q_head, q_count;
    /* statistics */
    uint32_t    tx_count;
    uint32_t    rx_count;      /* bodies queued for this radio */
    uint32_t    rx_overrun;    /* bodies dropped because the queue was full */
} ant_air_radio_t;

struct ant_air {
    ant_air_radio_t radio[ANT_AIR_MAX_RADIOS];
    uint32_t now;
    /* Fault injection: every n-th transmitted frame is lost / corrupted
     * (0 = never). Counted over all radios. */
    uint32_t drop_every_n;
    uint32_t corrupt_every_n;
    /* statistics */
    uint32_t frames;          /* frames transmitted */
    uint32_t delivered;       /* (radio, frame) deliveries */
    uint32_t dropped;
    uint32_t corrupted;
    uint32_t not_listening;   /* deliveries skipped: rx off / other freq / addr */
};

void       ant_air_init(ant_air_t *air);
/* Attach a radio; returns its ant_phy_t or NULL if the air is full. */
ant_phy_t *ant_air_attach(ant_air_t *air);
void       ant_air_detach(ant_air_t *air, ant_phy_t *phy);
/* Back from the ant_phy_t to its radio record (for stats). */
static inline ant_air_radio_t *ant_air_radio(ant_phy_t *phy)
{
    return (ant_air_radio_t *)phy;
}

#ifdef __cplusplus
}
#endif

#endif /* ANT_AIR_H */
