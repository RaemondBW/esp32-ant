/*
 * ant_phy_shockburst.c - ANT over-the-air PHY frame (see header).
 *
 * The CRC and frame layout here reproduce, byte for byte, the format used by
 * the public ANT ShockBurst demodulator (antfs-poc-defcon24/sniff/packets.h),
 * which is the same format an nRF24L01+ produces in hardware.
 */
#include "ant_phy_shockburst.h"
#include <string.h>

uint16_t ant_crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int i = 0; i < 8; i++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

uint8_t ant_sb_preamble_for(uint8_t first_address_byte)
{
    /* If the first transmitted bit (MSB) is 1, preamble is 0xAA, else 0x55.
     * This keeps the preamble/address boundary a clean alternating run. */
    return (first_address_byte & 0x80u) ? 0xAAu : 0x55u;
}

size_t ant_sb_build_frame(const uint8_t *address, uint8_t addr_len,
                          const uint8_t *payload, uint8_t payload_len,
                          uint8_t *out, size_t out_cap)
{
    if (!address || !payload || !out) {
        return 0;
    }
    if (addr_len < ANT_SB_ADDR_MIN || addr_len > ANT_SB_ADDR_MAX) {
        return 0;
    }
    if (payload_len == 0 || payload_len > ANT_SB_PAYLOAD_MAX) {
        return 0;
    }
    size_t total = (size_t)1u + addr_len + payload_len + ANT_SB_CRC_LEN;
    if (out_cap < total) {
        return 0;
    }

    size_t o = 0;
    out[o++] = ant_sb_preamble_for(address[0]);
    memcpy(&out[o], address, addr_len); o += addr_len;
    memcpy(&out[o], payload, payload_len); o += payload_len;

    /* CRC is over address+payload (i.e. everything after the preamble). */
    uint16_t crc = ant_crc16_ccitt(&out[1], (size_t)addr_len + payload_len);
    out[o++] = (uint8_t)(crc >> 8);   /* big-endian on the wire */
    out[o++] = (uint8_t)(crc & 0xFF);
    return o;
}

bool ant_sb_verify_frame(const uint8_t *buf, size_t buf_len,
                         uint8_t addr_len, uint8_t payload_len,
                         uint8_t *payload_out)
{
    if (!buf) {
        return false;
    }
    if (addr_len < ANT_SB_ADDR_MIN || addr_len > ANT_SB_ADDR_MAX) {
        return false;
    }
    size_t body = (size_t)addr_len + payload_len;
    if (buf_len != body + ANT_SB_CRC_LEN) {
        return false;
    }
    uint16_t calc = ant_crc16_ccitt(buf, body);
    uint16_t got = (uint16_t)((buf[body] << 8) | buf[body + 1]);
    if (calc != got) {
        return false;
    }
    if (payload_out) {
        memcpy(payload_out, &buf[addr_len], payload_len);
    }
    return true;
}

uint64_t ant_sb_address_to_u64(const uint8_t *address, uint8_t addr_len)
{
    uint64_t a = 0;
    for (uint8_t i = 0; i < addr_len; i++) {
        a |= ((uint64_t)address[i]) << ((addr_len - 1 - i) * 8);
    }
    return a;
}

void ant_sb_address_from_u64(uint64_t addr, uint8_t addr_len, uint8_t *out)
{
    for (uint8_t i = 0; i < addr_len; i++) {
        out[i] = (uint8_t)(addr >> ((addr_len - 1 - i) * 8));
    }
}
