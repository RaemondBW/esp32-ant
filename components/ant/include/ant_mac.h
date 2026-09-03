/*
 * ant_mac.h - the ANT air-interface MAC: what an ANT network processor does.
 *
 * This is the protocol engine that sits between the application (or the ANT
 * serial-message bridge, ant_embedded) and a radio (ant_phy). It implements the
 * ANT channel model over the air, independent of which radio it drives:
 *
 *   - up to ANT_MAC_MAX_CHANNELS channels time-sharing one radio, each with
 *     its own network, RF frequency, channel id, period and type;
 *   - ANT networks: 3 network keys; nodes on different networks never see
 *     each other (the key is folded into the sync word by ant_sb_link);
 *   - master channels: isochronous transmit on the 32768 Hz grid, exactly
 *     every `period` ticks, rebroadcasting the last page, EVENT_TX after each;
 *   - slave channels: search (wildcard or specific id, with the ANT search
 *     timeout and optional proximity search), acquire, track the master's
 *     slot with a tolerance window, EVENT_RX_FAIL per miss, drop to search
 *     after ~2 s of misses (EVENT_RX_FAIL_GO_TO_SEARCH), learn the master's
 *     channel id (the ANT "pairing": open with wildcards, keep what you
 *     acquired, reopen with it next time);
 *   - the pairing bit (device type bit 7): masters may set it, a slave
 *     searching with it set only acquires such masters;
 *   - bidirectional channels: a slave answers in the master's slot (reverse
 *     broadcast, acknowledged, burst);
 *   - acknowledged data both ways (EVENT_TRANSFER_TX_COMPLETED / _FAILED);
 *   - burst transfers both ways: 8-byte packets, sequence numbers, per-packet
 *     acknowledgement, retries, EVENT_TRANSFER_RX_FAILED on a broken sequence;
 *   - the ANT command validity rules and response codes (channel in wrong
 *     state, id not set, transfer in progress, invalid network ...).
 *
 * Time base: ANT ticks, 1/32768 s, as a wrapping uint32_t supplied by the
 * caller. The MAC is poll-driven: call ant_mac_tick(now) at or before the
 * deadline it returns, and it does everything due (transmits, opens/closes
 * receive windows, expires handshakes). It never blocks and never allocates.
 *
 * Not implemented (documented gaps): shared channels, background scanning /
 * continuous scan, frequency agility, extended messaging.
 */
#ifndef ANT_MAC_H
#define ANT_MAC_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "ant_phy.h"
#include "ant_sb_link.h"
#include "ant_message.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ANT_MAC_MAX_CHANNELS      8u
#define ANT_MAC_MAX_NETWORKS      3u
#define ANT_MAC_BURST_MAX_BYTES   128u   /* one burst transfer: 16 packets */

/* --- Air timing (ticks of 1/32768 s) --- */
/* 18-byte frame at 1 Mbit/s = 144 us (a real strap burst is ~174 us). */
/* Frame air time in ticks, rounded up. */
#define ANT_MAC_FRAME_TICKS        6u
/* After a transmit, how long the sender listens for the peer's reply. */
#define ANT_MAC_REPLY_WINDOW       32u    /* ~1 ms */
/* How often a MAC with the receiver on must be ticked to drain it. */
#define ANT_MAC_POLL_INTERVAL      8u     /* ~244 us */
/* Burst packet retransmissions before EVENT_TRANSFER_TX_FAILED. */
#define ANT_MAC_BURST_RETRIES      5u
/* A burst receiver waits this long for the next packet: the sender's whole
 * retry budget plus slack, so a lost packet or a lost ack does not desync. */
#define ANT_MAC_BURST_RX_WINDOW    (ANT_MAC_REPLY_WINDOW * (ANT_MAC_BURST_RETRIES + 2u))
/* Search timeout unit (ANT: 2.5 s). */
#define ANT_MAC_SEARCH_UNIT_TICKS  81920u
/* A tracking slave drops to search after this much time without a frame. */
#define ANT_MAC_DROP_TO_SEARCH_TICKS 65536u /* 2 s */
/* Tracking window half-width: period/32 clamped to this range. */
#define ANT_MAC_WINDOW_MIN         16u
#define ANT_MAC_WINDOW_MAX         1024u
/* Nominal ANT+ HRM period for reference: 8070 -> window 252 (~7.7 ms). */

typedef struct ant_mac ant_mac_t;

/*
 * Received data. `msg_type` is ANT_MSG_BROADCAST_DATA, ANT_MSG_ACKNOWLEDGED_DATA
 * or ANT_MSG_BURST_DATA. For burst, `burst_seq` is the 3-bit ANT sequence
 * field (0 first, then 1,2,3 rotating; +4 marks the last packet).
 */
typedef void (*ant_mac_data_cb_t)(ant_mac_t *mac, uint8_t channel,
                                  uint8_t msg_type, uint8_t burst_seq,
                                  const uint8_t data[8], void *user);
/* Channel events: ANT_EVENT_* codes from ant_message.h. */
typedef void (*ant_mac_event_cb_t)(ant_mac_t *mac, uint8_t channel,
                                   uint8_t event, void *user);

typedef enum {
    ANT_MAC_CH_UNASSIGNED = ANT_STATUS_UNASSIGNED,
    ANT_MAC_CH_ASSIGNED   = ANT_STATUS_ASSIGNED,
    ANT_MAC_CH_SEARCHING  = ANT_STATUS_SEARCHING,
    ANT_MAC_CH_TRACKING   = ANT_STATUS_TRACKING,
} ant_mac_ch_state_t;

/* Sub-slot handshake the channel is in (it holds the radio while non-zero). */
enum {
    ANT_MAC_HOLD_NONE = 0,
    ANT_MAC_HOLD_REPLY_WINDOW,     /* sent a broadcast, listening for a reply */
    ANT_MAC_HOLD_AWAIT_ACKRESP,    /* sent ack/burst packet, waiting for ack */
    ANT_MAC_HOLD_AWAIT_BURST_NEXT, /* acked a burst packet, waiting for next */
};

typedef struct {
    /* configuration */
    ant_mac_ch_state_t state;
    uint8_t  type;            /* ANT_CHANNEL_TYPE_* */
    uint8_t  network;
    uint16_t device_num;      /* learned on wildcard acquisition */
    uint8_t  device_type;
    uint8_t  trans_type;
    uint8_t  rf_freq;
    uint16_t period;
    uint8_t  search_timeout;  /* 2.5 s units; 0xFF infinite; 0 none */
    uint8_t  tx_power;
    bool     id_set;
    int8_t   proximity_rssi;  /* slave: acquire only at >= this dBm; 0 = off */
    bool     proximity_armed; /* applies to the first search after open only */

    /* air parameters */
    ant_sb_link_t link;
    uint8_t  mask[ANT_SB_ADDR_MAX];

    /* schedule */
    uint32_t next_slot;       /* master: next TX; slave: next expected RX */
    bool     slot_done;       /* slave: this slot's frame already received */
    uint32_t window;          /* slave: current tracking half-window */
    uint32_t window_base;
    uint32_t last_rx;
    uint32_t search_deadline;
    bool     search_infinite;
    bool     armed;           /* schedule anchored to real time (first tick after open) */
    uint8_t  hold;
    uint32_t hold_until;

    /* transmit data */
    uint8_t  bcast[ANT_SB_LINK_DATA_LEN];
    bool     bcast_loaded;    /* master: has a page to (re)broadcast */
    bool     bcast_pending;   /* slave: one-shot reverse broadcast queued */
    uint8_t  xfer_type;       /* 0, ANT_SB_CTRL_ACK or ANT_SB_CTRL_BURST */
    uint8_t  xfer_buf[ANT_MAC_BURST_MAX_BYTES];
    uint8_t  xfer_len;
    uint8_t  xfer_off;
    uint8_t  xfer_seq;
    uint8_t  xfer_retries;

    /* burst receive */
    bool     rxb_active;
    uint8_t  rxb_expect;
    uint8_t  rxb_last;
    bool     rxb_have_last;

    /* stats */
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t miss_count;
    uint32_t crc_fail_count;  /* frames addressed to this channel that failed CRC */
    int8_t   last_rssi;       /* of the last frame accepted on this channel */
} ant_mac_channel_t;

struct ant_mac {
    ant_phy_t *phy;
    ant_mac_channel_t ch[ANT_MAC_MAX_CHANNELS];
    uint8_t  net_key[ANT_MAC_MAX_NETWORKS][8];

    /* radio ownership */
    int8_t   owner;           /* channel index currently using the radio, -1 */
    bool     rx_on;
    uint16_t tuned_mhz;
    ant_phy_rx_cfg_t rx_cfg;

    uint32_t now;
    ant_mac_data_cb_t  on_data;
    ant_mac_event_cb_t on_event;
    void    *user;
};

/* --- lifecycle --- */
void ant_mac_init(ant_mac_t *mac, ant_phy_t *phy,
                  ant_mac_data_cb_t on_data, ant_mac_event_cb_t on_event,
                  void *user);
/* Reset everything (all channels unassigned, keys cleared, radio released). */
void ant_mac_reset(ant_mac_t *mac);

/* --- ANT commands. Each returns an ANT response code (ANT_RESPONSE_*). --- */
uint8_t ant_mac_set_network_key(ant_mac_t *mac, uint8_t net, const uint8_t key[8]);
uint8_t ant_mac_assign_channel(ant_mac_t *mac, uint8_t ch, uint8_t type, uint8_t net);
uint8_t ant_mac_unassign_channel(ant_mac_t *mac, uint8_t ch);
uint8_t ant_mac_set_channel_id(ant_mac_t *mac, uint8_t ch, uint16_t device_num,
                               uint8_t device_type, uint8_t trans_type);
uint8_t ant_mac_set_channel_period(ant_mac_t *mac, uint8_t ch, uint16_t period);
uint8_t ant_mac_set_channel_rf_freq(ant_mac_t *mac, uint8_t ch, uint8_t rf_freq);
uint8_t ant_mac_set_search_timeout(ant_mac_t *mac, uint8_t ch, uint8_t timeout);
/* Proximity search: while searching after open, ignore masters heard below
 * `min_rssi` dBm (0 = off). As in ANT, it applies to the first acquisition
 * only; a channel that drops to search re-acquires its master at any level. */
uint8_t ant_mac_set_proximity_search(ant_mac_t *mac, uint8_t ch, int8_t min_rssi);
uint8_t ant_mac_set_tx_power(ant_mac_t *mac, uint8_t ch, uint8_t power);
uint8_t ant_mac_open_channel(ant_mac_t *mac, uint8_t ch);
uint8_t ant_mac_close_channel(ant_mac_t *mac, uint8_t ch);

/* Master: page to broadcast every slot from now on. Slave (bidirectional):
 * one reverse broadcast in the next slot. */
uint8_t ant_mac_send_broadcast(ant_mac_t *mac, uint8_t ch, const uint8_t data[8]);
/* Acknowledged data: sent once in the next slot; completes with
 * EVENT_TRANSFER_TX_COMPLETED or EVENT_TRANSFER_TX_FAILED. */
uint8_t ant_mac_send_acknowledged(ant_mac_t *mac, uint8_t ch, const uint8_t data[8]);
/* Burst: 1..ANT_MAC_BURST_MAX_BYTES bytes (padded to 8-byte packets), sent in
 * the next slot with per-packet acknowledgement and retries. */
uint8_t ant_mac_send_burst(ant_mac_t *mac, uint8_t ch, const uint8_t *data, size_t len);

/* --- queries --- */
uint8_t ant_mac_channel_status(const ant_mac_t *mac, uint8_t ch);  /* ANT status byte */
bool    ant_mac_get_channel_id(const ant_mac_t *mac, uint8_t ch, uint16_t *device_num,
                               uint8_t *device_type, uint8_t *trans_type);
static inline const ant_mac_channel_t *ant_mac_channel(const ant_mac_t *mac, uint8_t ch)
{
    return (ch < ANT_MAC_MAX_CHANNELS) ? &mac->ch[ch] : NULL;
}

/*
 * Run the MAC at time `now` (ticks). Performs every action due, then returns
 * the tick by which it must be called again. Call it earlier if you like; call
 * it late and slots are missed the way a real radio would miss them.
 */
uint32_t ant_mac_tick(ant_mac_t *mac, uint32_t now);

const char *ant_mac_event_name(uint8_t event);
const char *ant_mac_state_name(ant_mac_ch_state_t s);

#ifdef __cplusplus
}
#endif

#endif /* ANT_MAC_H */
