/*
 * ant_sb_link.h - ANT channel identity -> on-air PHY parameters + air payload.
 *
 * The ESP32 runs the entire ANT MAC itself; this module turns an ANT channel
 * identity (network key, RF frequency, device number/type, transmission type)
 * into the things any radio needs to put an ANT frame on the air:
 *
 *   - the RF frequency (MHz),
 *   - the ShockBurst sync word (the 6-byte "address"), and
 *   - the ShockBurst payload: one flags byte + the 8-byte ANT data page.
 *
 * It is pure and radio-independent: it produces the bytes/frequency that must
 * appear on the air. The ANT MAC (ant_mac.*) consumes it to drive whatever PHY
 * it is given (the internal ESP32 radio via ant_espphy, or the virtual air used
 * by the host tests). Nothing here touches hardware.
 *
 * This IS the format commercial ANT+ sensors use. It was confirmed on this
 * bench by SDR-decoding a heart-rate strap (device 0x6941) and by receiving
 * that strap on an ESP32-S3, and it matches the ESPwn32 reverse engineering
 * (Cayre & Cauquil, WOOT 2023, incl. their nRF52 SoftDevice analysis of the
 * network-key -> marker function). See docs/SHOCKBURST_LINK.md.
 *
 * What is ours: the meaning of the flags byte beyond broadcasts. A real sensor
 * sends 0x0A in every broadcast; the acknowledged/burst/reverse encodings in
 * the ANT_SB_CTRL_* bits below are our documented convention for ESP32 <->
 * ESP32 links (a commercial receiver only needs the broadcast form).
 */
#ifndef ANT_SB_LINK_H
#define ANT_SB_LINK_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "ant_phy_shockburst.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ANT broadcast/acknowledged/burst data is always 8 bytes. */
#define ANT_SB_LINK_DATA_LEN     8u
/* On-air ShockBurst payload: flags/control byte + 8 data bytes. */
#define ANT_SB_LINK_PAYLOAD_LEN  (1u + ANT_SB_LINK_DATA_LEN)
/* Network marker (2) + channel id (4): the real ANT address length. */
#define ANT_SB_LINK_MARKER_LEN   2u
#define ANT_SB_LINK_ADDR_LEN     6u
/* Full on-air frame: preamble + 6 + 9 + 2 = 18 bytes. */
#define ANT_SB_LINK_FRAME_LEN    (1u + ANT_SB_LINK_ADDR_LEN + ANT_SB_LINK_PAYLOAD_LEN + ANT_SB_CRC_LEN)

/*
 * Payload control byte (payload[0]).
 *   bits 7..6  message type   (broadcast / acknowledged / ack-response / burst)
 *   bit  5     direction      0 = master -> slave (forward), 1 = slave -> master
 *   bit  4     last           burst: final packet of the transfer
 *   bits 3..0  burst sequence 0 for the first packet, then 1,2,3,1,2,3..;
 *              for every other type the nibble is ANT_SB_CTRL_TAG (0xA), the
 *              value real ANT+ sensors put here (so a forward broadcast is
 *              0x0A on the air, exactly like a commercial sensor).
 * ant_sb_link_build() fills the low nibble; callers pass type|direction|last.
 */
#define ANT_SB_CTRL_TYPE_MASK    0xC0u
#define ANT_SB_CTRL_BROADCAST    0x00u
#define ANT_SB_CTRL_ACK          0x40u
#define ANT_SB_CTRL_ACK_RESP     0x80u
#define ANT_SB_CTRL_BURST        0xC0u
#define ANT_SB_CTRL_REVERSE      0x20u
#define ANT_SB_CTRL_LAST         0x10u
#define ANT_SB_CTRL_SEQ_MASK     0x0Fu
#define ANT_SB_CTRL_TAG          0x0Au

/* Well-known ANT network keys, in the byte order they appear in the ANT
 * "Set Network Key" message (the order the Garmin SDK prints them). */
#define ANT_NETWORK_KEY_ANTPLUS  { 0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45 }
#define ANT_NETWORK_KEY_PUBLIC   { 0xE8, 0xE4, 0x21, 0x3B, 0x55, 0x7A, 0x67, 0xC1 }
#define ANT_NETWORK_KEY_ANTFS    { 0xA8, 0xA4, 0x23, 0xB9, 0xF5, 0x5E, 0x63, 0xC1 }

/* On-air PHY parameters derived from an ANT channel identity. */
typedef struct {
    uint16_t freq_mhz;                  /* RF frequency in MHz (e.g. 2457) */
    uint8_t  addr_len;                  /* ANT_SB_LINK_ADDR_LEN */
    uint8_t  address[ANT_SB_ADDR_MAX];  /* MSB-first sync word, addr_len bytes */
    uint8_t  payload_len;               /* ANT_SB_LINK_PAYLOAD_LEN */
} ant_sb_link_t;

/*
 * Map an ANT channel identity onto on-air PHY parameters (ANT+ network
 * marker by default; ant_sb_link_set_network() rebinds it).
 *   rf_freq     : ANT rf_freq field (e.g. ANTPLUS_RF_FREQ = 57). The absolute
 *                 frequency is 2400 + rf_freq MHz (ANT+ = 57 -> 2457 MHz).
 *   addr_len    : must be ANT_SB_LINK_ADDR_LEN (6).
 *   device_num  : ANT device number.
 *   device_type : ANT+ device profile type (e.g. 120 = HRM).
 *   trans_type  : transmission type.
 * Returns false (and leaves *link zeroed) on a bad addr_len.
 *
 * Address layout (the real one):
 *   address[0..1] = network marker (ANT+ = a6 c5)
 *   address[2]    = device_num high
 *   address[3]    = device_num low
 *   address[4]    = device_type
 *   address[5]    = trans_type
 */
bool ant_sb_link_from_identity(ant_sb_link_t *link, uint8_t rf_freq,
                               uint8_t addr_len, uint16_t device_num,
                               uint8_t device_type, uint8_t trans_type);

/*
 * Bind the link to an ANT network: derive the 2-byte marker from the 8-byte
 * network key. Nodes on different networks use different markers and never
 * see each other - the ANT "network" semantic.
 *
 * The derivation is the one the nRF52 ANT SoftDevice uses (recovered by
 * ESPwn32): ANT+ key -> a6 c5, ANT-FS key -> 3b a3, public key -> 5b 25.
 * The all-zero key means "the public network" (ANT's network 0 default).
 * A key the SoftDevice would reject (ant_sb_network_key_valid() false) gets a
 * documented fallback marker (XOR-fold, MSB set) so ESP32 <-> ESP32 links on
 * private made-up keys still work; only SoftDevice-valid keys interoperate
 * with commercial ANT devices.
 */
void ant_sb_link_set_network(ant_sb_link_t *link, const uint8_t key[8]);
void ant_sb_link_network_marker(const uint8_t key[8], uint8_t marker[ANT_SB_LINK_MARKER_LEN]);
bool ant_sb_network_key_valid(const uint8_t key[8]);

/*
 * Fill a receive-match mask for a slave search: 0xFF for address bytes that
 * must match (marker + every non-wildcard identity field), 0x00 for wildcard
 * fields (device_num 0, device_type 0, trans_type 0). The device type's
 * pairing bit (bit 7) is ignored unless the search itself sets it. `mask`
 * has addr_len bytes.
 */
void ant_sb_link_search_mask(uint8_t addr_len, uint16_t device_num,
                             uint8_t device_type, uint8_t trans_type,
                             uint8_t mask[ANT_SB_ADDR_MAX]);

/* Recover the channel identity carried by a received address. */
void ant_sb_link_identity_from_address(const uint8_t *address, uint8_t addr_len,
                                       uint16_t *device_num, uint8_t *device_type,
                                       uint8_t *trans_type);

/*
 * Build a full on-air ANT frame (preamble|address|ctrl+8 data|CRC16) from a
 * link, a control byte (ANT_SB_CTRL_* type|direction|last|seq) and 8 data
 * bytes. Returns the frame length, or 0 on error. This is the complete byte
 * stream a transmitter keys onto the carrier.
 */
size_t ant_sb_link_build(const ant_sb_link_t *link, uint8_t ctrl,
                         const uint8_t data[ANT_SB_LINK_DATA_LEN],
                         uint8_t *out, size_t out_cap);

/*
 * Verify a received {address|ctrl+data|CRC} body (no preamble). The CRC is
 * checked over the body's own address+payload, so a corrupted or foreign frame
 * fails. On success copies out the control byte and the 8 data bytes (either
 * pointer may be NULL) and returns true. `body_len` must equal
 * addr_len + payload_len + 2.
 */
bool ant_sb_link_recv(const ant_sb_link_t *link,
                      const uint8_t *body, size_t body_len,
                      uint8_t *ctrl_out, uint8_t data_out[ANT_SB_LINK_DATA_LEN]);

/* Convenience: forward (master -> slave) broadcast build / receive. */
size_t ant_sb_link_build_broadcast(const ant_sb_link_t *link,
                                   const uint8_t page[ANT_SB_LINK_DATA_LEN],
                                   uint8_t *out, size_t out_cap);
bool ant_sb_link_recv_broadcast(const ant_sb_link_t *link,
                                const uint8_t *body, size_t body_len,
                                uint8_t page_out[ANT_SB_LINK_DATA_LEN]);

#ifdef __cplusplus
}
#endif

#endif /* ANT_SB_LINK_H */
