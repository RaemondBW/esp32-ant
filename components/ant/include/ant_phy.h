/*
 * ant_phy.h - the radio interface the ANT MAC drives.
 *
 * ant_mac runs the whole ANT air protocol (channel timing, search/track,
 * broadcast/acknowledged/burst, networks) on top of this small vtable. Two
 * implementations exist:
 *
 *   - ant_air      : a virtual air medium shared by several MACs in one process
 *                    (host tests and on-target self-test). Exact, deterministic.
 *   - ant_espphy   : the ESP32's own 2.4 GHz radio (the pure-ESP32 PHY).
 *
 * All calls are non-blocking. Time is in ANT ticks (1/32768 s). A frame handed
 * to tx() is the complete on-air byte stream (preamble|address|payload|CRC);
 * a body returned by rx_poll() is that frame minus the preamble, exactly as a
 * ShockBurst receiver hands it up (the MAC does the CRC in software).
 */
#ifndef ANT_PHY_H
#define ANT_PHY_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ANT_TICKS_PER_SEC   32768u

typedef struct ant_phy ant_phy_t;

/* Longest ShockBurst address a PHY must match (== ANT_SB_ADDR_MAX). */
#define ANT_PHY_ADDR_MAX    6u

/* Receiver match configuration: accept a frame whose address, ANDed with
 * `mask`, equals `address & mask`. addr_len bytes of each. A hardware
 * receiver with a shorter sync word (the ESP32's BLE core locks on 32 bits)
 * matches what it can in hardware and the rest in software. */
typedef struct {
    uint8_t address[ANT_PHY_ADDR_MAX];
    uint8_t mask[ANT_PHY_ADDR_MAX];
    uint8_t addr_len;
    uint8_t payload_len;
} ant_phy_rx_cfg_t;

struct ant_phy {
    /* Tune the synthesizer to `mhz`. Returns false if that cannot be done. */
    bool   (*tune)(ant_phy_t *self, uint16_t mhz);
    /* Key one complete frame onto the carrier now. Returns false on failure. */
    bool   (*tx)(ant_phy_t *self, const uint8_t *frame, size_t len);
    /* Program the receiver's address match; does not enable it. */
    void   (*rx_config)(ant_phy_t *self, const ant_phy_rx_cfg_t *cfg);
    /* Turn the receiver on/off. */
    void   (*rx_enable)(ant_phy_t *self, bool on);
    /* Fetch one received body (address|payload|crc) if available. Returns its
     * length, 0 if none. *rx_time gets the tick at which it arrived, *rssi
     * its signal strength in dBm (0 if the radio cannot tell). */
    size_t (*rx_poll)(ant_phy_t *self, uint8_t *body, size_t cap, uint32_t *rx_time,
                      int8_t *rssi);
    void *ctx;
};

static inline bool ant_phy_tune(ant_phy_t *p, uint16_t mhz)
{
    return (p && p->tune) ? p->tune(p, mhz) : false;
}
static inline bool ant_phy_tx(ant_phy_t *p, const uint8_t *f, size_t n)
{
    return (p && p->tx) ? p->tx(p, f, n) : false;
}
static inline void ant_phy_rx_config(ant_phy_t *p, const ant_phy_rx_cfg_t *c)
{
    if (p && p->rx_config) p->rx_config(p, c);
}
static inline void ant_phy_rx_enable(ant_phy_t *p, bool on)
{
    if (p && p->rx_enable) p->rx_enable(p, on);
}
static inline size_t ant_phy_rx_poll(ant_phy_t *p, uint8_t *b, size_t cap, uint32_t *t,
                                     int8_t *rssi)
{
    return (p && p->rx_poll) ? p->rx_poll(p, b, cap, t, rssi) : 0;
}

#ifdef __cplusplus
}
#endif

#endif /* ANT_PHY_H */
