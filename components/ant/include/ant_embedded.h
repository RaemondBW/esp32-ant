/*
 * ant_embedded.h - the ANT MAC presented as an ANT serial network processor.
 *
 * An ant_radio_t transport whose far end is not a UART to an ANT chip but
 * ant_mac running in this same process. Host-side ANT serial messages written
 * to it (assign channel, set id, open, broadcast data, request ...) are parsed
 * and executed on the MAC; the MAC's responses, channel events and received
 * data come back as ANT serial messages readable through it. The existing
 * ant_stack / ant_channel code therefore runs unchanged on top of the ESP32's
 * own radio: the ESP32 *is* the ANT chip.
 *
 * Messages implemented (host -> ANT): RESET_SYSTEM, SET_NETWORK_KEY,
 * ASSIGN_CHANNEL, UNASSIGN_CHANNEL, SET_CHANNEL_ID, SET_CHANNEL_PERIOD,
 * SET_CHANNEL_RF_FREQ, SET_SEARCH_TIMEOUT, SET_CHANNEL_TX_POWER, OPEN_CHANNEL,
 * CLOSE_CHANNEL, BROADCAST_DATA, ACKNOWLEDGED_DATA, BURST_DATA (packets are
 * collected until the last one, then sent as one burst), REQUEST (for
 * CHANNEL_STATUS, CHANNEL_ID, CAPABILITIES, VERSION).
 * Messages emitted (ANT -> host): STARTUP, CHANNEL_RESPONSE (command responses
 * and channel events), BROADCAST_DATA, ACKNOWLEDGED_DATA, BURST_DATA,
 * CHANNEL_STATUS, CHANNEL_ID, CAPABILITIES, VERSION.
 *
 * Time: call ant_embedded_tick(now) at or before the deadline it returns
 * (it forwards to ant_mac_tick). The transport calls are non-blocking.
 */
#ifndef ANT_EMBEDDED_H
#define ANT_EMBEDDED_H

#include "ant_radio.h"
#include "ant_mac.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ANT_EMBEDDED_OUT_SIZE   1024u   /* ANT -> host byte FIFO */

typedef struct {
    uint8_t  buf[ANT_MAC_BURST_MAX_BYTES];
    uint8_t  len;
    uint8_t  next_seq;
    bool     active;
} ant_embedded_hburst_t;

typedef struct {
    ant_radio_t  radio;       /* the transport handed to ant_stack (first) */
    ant_mac_t    mac;
    ant_parser_t parser;      /* host -> ANT byte stream */
    /* ANT -> host FIFO */
    uint8_t      out[ANT_EMBEDDED_OUT_SIZE];
    uint16_t     out_head, out_count;
    uint32_t     out_overflow;   /* messages dropped: host did not read */
    /* Host burst assembly, per channel. */
    ant_embedded_hburst_t hburst[ANT_MAC_MAX_CHANNELS];
    uint32_t     serial_errors;
} ant_embedded_t;

/* Bind to a PHY. The transport is &e->radio. */
void         ant_radio_embedded_init(ant_embedded_t *e, ant_phy_t *phy);
ant_radio_t *ant_radio_embedded(ant_embedded_t *e);
/* Run the MAC; returns the next tick it must be called at. */
uint32_t     ant_embedded_tick(ant_embedded_t *e, uint32_t now);
/* Feed one host->ANT message directly (what write() does after framing). */
void         ant_embedded_handle(ant_embedded_t *e, const ant_message_t *msg);

#ifdef __cplusplus
}
#endif

#endif /* ANT_EMBEDDED_H */
