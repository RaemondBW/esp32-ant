/*
 * ant_sb_link.c - ANT channel identity <-> ShockBurst PHY parameters.
 * See ant_sb_link.h for the format and its provenance.
 *
 * Pure, portable C: no ESP dependencies, exercised by the host test suite.
 */
#include "ant_sb_link.h"
#include <string.h>

static const uint8_t ANTPLUS_KEY[8] = ANT_NETWORK_KEY_ANTPLUS;
static const uint8_t PUBLIC_KEY[8]  = ANT_NETWORK_KEY_PUBLIC;

/*
 * Network key validation and marker derivation, as performed by the nRF52 ANT
 * SoftDevice (recovered by ESPwn32's ant_network_keys artifact). The key is
 * the 8 bytes of the ANT "Set Network Key" message, in message order.
 *
 * Validation: eight expressions, each the XOR of a growing set of key bytes
 * (key[2], key[2..3], ... key[2..7], then also key[1], then key[0]), masked
 * and compared against a constant; all must match.
 */
static const uint8_t VALID_AND[8] = { 0xec, 0x3f, 0xd7, 0xdb, 0x79, 0xf7, 0xbe, 0xef };
static const uint8_t VALID_XOR[8] = { 0x20, 0x1a, 0x47, 0x11, 0x50, 0x93, 0x36, 0x8f };
/* Marker: 4 masked XOR-combinations per byte; bit j of KEYSEL selects key[j]. */
static const uint8_t MARK_KEYSEL[8] = { 0xfe, 0xff, 0x1c, 0x7c, 0xfc, 0x0c, 0x04, 0x3c };
static const uint8_t MARK_AND[8]    = { 0x41, 0x10, 0x28, 0x86, 0x08, 0xc0, 0x13, 0x24 };

bool ant_sb_network_key_valid(const uint8_t key[8])
{
    if (!key) return false;
    uint8_t acc = 0;
    for (int i = 0; i < 8; i++) {
        uint8_t t = 0;
        for (int j = 0; j <= i; j++) {
            t ^= (j < 6) ? key[2 + j] : (j == 6 ? key[1] : key[0]);
        }
        acc |= (uint8_t)((t & VALID_AND[i]) ^ VALID_XOR[i]);
    }
    return acc == 0;
}

static bool key_is_zero(const uint8_t key[8])
{
    for (int i = 0; i < 8; i++) if (key[i]) return false;
    return true;
}

void ant_sb_link_network_marker(const uint8_t key[8], uint8_t marker[ANT_SB_LINK_MARKER_LEN])
{
    if (!key || key_is_zero(key)) key = PUBLIC_KEY;   /* ANT network 0 default */

    if (!ant_sb_network_key_valid(key)) {
        /* Not a real ANT key: documented fallback so both ESP32 ends agree.
         * XOR-fold to 7 bits, keep the MSB set (preamble stays 0xAA). */
        uint8_t f = 0;
        for (int i = 0; i < 8; i++) f ^= key[i];
        marker[0] = (uint8_t)(0x80u | ((0xA6u ^ f) & 0x7Fu));
        marker[1] = (uint8_t)(0xC5u ^ key[0] ^ key[7]);
        return;
    }
    uint8_t lo = 0, hi = 0;
    for (int i = 0; i < 8; i++) {
        uint8_t t = 0;
        for (int j = 0; j < 8; j++) {
            if (MARK_KEYSEL[i] & (1u << j)) t ^= key[j];
        }
        t &= MARK_AND[i];
        if (i < 4) lo |= t; else hi |= t;
    }
    /* The SoftDevice keeps the marker as a little-endian uint16 (ANT+ =
     * 0xc5a6); on the air the low byte goes first: a6 c5. */
    marker[0] = lo;
    marker[1] = hi;
}

bool ant_sb_link_from_identity(ant_sb_link_t *link, uint8_t rf_freq,
                               uint8_t addr_len, uint16_t device_num,
                               uint8_t device_type, uint8_t trans_type)
{
    if (!link) {
        return false;
    }
    memset(link, 0, sizeof(*link));
    if (addr_len != ANT_SB_LINK_ADDR_LEN) {
        return false;
    }

    /* ANT's rf_freq field is the frequency offset above 2400 MHz, so the
     * absolute RF frequency is 2400 + rf_freq MHz. ANTPLUS_RF_FREQ 57 -> 2457
     * MHz. The internal-radio PHY (ant_espphy) converts this to its synth word. */
    link->freq_mhz = (uint16_t)(2400u + rf_freq);
    link->payload_len = ANT_SB_LINK_PAYLOAD_LEN;
    link->addr_len = addr_len;

    ant_sb_link_network_marker(ANTPLUS_KEY, link->address);
    link->address[2] = (uint8_t)(device_num >> 8);
    link->address[3] = (uint8_t)(device_num & 0xFF);
    link->address[4] = device_type;
    link->address[5] = trans_type;
    return true;
}

void ant_sb_link_set_network(ant_sb_link_t *link, const uint8_t key[8])
{
    if (!link) return;
    ant_sb_link_network_marker(key, link->address);
}

void ant_sb_link_search_mask(uint8_t addr_len, uint16_t device_num,
                             uint8_t device_type, uint8_t trans_type,
                             uint8_t mask[ANT_SB_ADDR_MAX])
{
    uint8_t full[ANT_SB_ADDR_MAX];
    full[0] = full[1] = 0xFF;                         /* network marker */
    full[2] = full[3] = device_num ? 0xFF : 0x00;     /* device number */
    /* Device type: bit 7 is the pairing request. Searching without it
     * accepts masters either way; searching with it set demands it. */
    full[4] = (device_type & 0x7Fu) == 0 ? 0x00
            : (device_type & 0x80u)      ? 0xFF : 0x7F;
    full[5] = trans_type ? 0xFF : 0x00;
    memset(mask, 0, ANT_SB_ADDR_MAX);
    if (addr_len > ANT_SB_ADDR_MAX) addr_len = ANT_SB_ADDR_MAX;
    memcpy(mask, full, addr_len);
}

void ant_sb_link_identity_from_address(const uint8_t *address, uint8_t addr_len,
                                       uint16_t *device_num, uint8_t *device_type,
                                       uint8_t *trans_type)
{
    uint8_t full[ANT_SB_ADDR_MAX] = { 0, 0, 0, 0, 0, 0 };
    if (address) {
        if (addr_len > ANT_SB_ADDR_MAX) addr_len = ANT_SB_ADDR_MAX;
        memcpy(full, address, addr_len);
    }
    if (device_num)  *device_num  = (uint16_t)((full[2] << 8) | full[3]);
    if (device_type) *device_type = full[4];
    if (trans_type)  *trans_type  = full[5];
}

size_t ant_sb_link_build(const ant_sb_link_t *link, uint8_t ctrl,
                         const uint8_t data[ANT_SB_LINK_DATA_LEN],
                         uint8_t *out, size_t out_cap)
{
    if (!link || !data || !out) {
        return 0;
    }
    /* Only bursts carry a sequence number; everything else gets the nibble a
     * commercial sensor sends (broadcast -> 0x0A on the air). */
    if ((ctrl & ANT_SB_CTRL_TYPE_MASK) != ANT_SB_CTRL_BURST) {
        ctrl = (uint8_t)((ctrl & ~ANT_SB_CTRL_SEQ_MASK) | ANT_SB_CTRL_TAG);
    }
    uint8_t payload[ANT_SB_LINK_PAYLOAD_LEN];
    payload[0] = ctrl;
    memcpy(&payload[1], data, ANT_SB_LINK_DATA_LEN);
    return ant_sb_build_frame(link->address, link->addr_len,
                              payload, ANT_SB_LINK_PAYLOAD_LEN, out, out_cap);
}

bool ant_sb_link_recv(const ant_sb_link_t *link,
                      const uint8_t *body, size_t body_len,
                      uint8_t *ctrl_out, uint8_t data_out[ANT_SB_LINK_DATA_LEN])
{
    if (!link || !body) {
        return false;
    }
    uint8_t payload[ANT_SB_LINK_PAYLOAD_LEN];
    if (!ant_sb_verify_frame(body, body_len, link->addr_len,
                             ANT_SB_LINK_PAYLOAD_LEN, payload)) {
        return false;
    }
    if (ctrl_out) *ctrl_out = payload[0];
    if (data_out) memcpy(data_out, &payload[1], ANT_SB_LINK_DATA_LEN);
    return true;
}

size_t ant_sb_link_build_broadcast(const ant_sb_link_t *link,
                                   const uint8_t page[ANT_SB_LINK_DATA_LEN],
                                   uint8_t *out, size_t out_cap)
{
    return ant_sb_link_build(link, ANT_SB_CTRL_BROADCAST, page, out, out_cap);
}

bool ant_sb_link_recv_broadcast(const ant_sb_link_t *link,
                                const uint8_t *body, size_t body_len,
                                uint8_t page_out[ANT_SB_LINK_DATA_LEN])
{
    uint8_t ctrl;
    if (!ant_sb_link_recv(link, body, body_len, &ctrl, page_out)) {
        return false;
    }
    return (ctrl & ANT_SB_CTRL_TYPE_MASK) == ANT_SB_CTRL_BROADCAST;
}
