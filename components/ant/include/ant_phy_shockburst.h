/*
 * ant_phy_shockburst.h - ANT over-the-air PHY frame (Enhanced ShockBurst).
 *
 * ANT's PHY is Nordic ShockBurst, and the exact frame a commercial ANT+
 * sensor puts on the air has been confirmed on this bench with an SDR
 * (a heart-rate strap, device number 0x6941) and matches the ESPwn32/esperanto
 * reverse engineering (Cayre & Cauquil, WOOT 2023):
 *
 *   +--------------+----------------------------+----------------+---------+
 *   | Preamble (1) | Address (6)                | Payload (9)    | CRC (2) |
 *   |    0xAA      | mk_hi mk_lo dn_hi dn_lo    | flags + page   |         |
 *   |              |             devtype trans  |                |         |
 *   +--------------+----------------------------+----------------+---------+
 *
 *   - Modulation : GFSK, 1 Mbit/s, ~+-220 kHz deviation, bit 1 = +f.
 *   - Preamble   : 1 byte, 0xAA if MSB of first address byte is 1, else 0x55.
 *   - Address    : 2-byte network marker derived from the 8-byte network key
 *                  (ANT+ key -> a6 c5; see ant_sb_link_network_marker) followed
 *                  by the 4-byte channel id: device number (big-endian), device
 *                  type, transmission type. The whole 6 bytes are MSB-first.
 *   - Payload    : one flags byte (0x0A on every broadcast a real sensor
 *                  sends) + the 8-byte ANT data page.
 *   - CRC        : CRC-16/CCITT-FALSE, poly 0x1021, init 0xFFFF, no reflection,
 *                  no final xor, computed over Address+Payload, MSB-first,
 *                  appended big-endian. NO data whitening (unlike BLE).
 *   - RF freq    : ANT+ broadcasts on 2457 MHz; general ANT default 2466.
 *
 * This module is the generic frame builder/verifier (any 3..6-byte address,
 * 1..32-byte payload); ant_sb_link.* maps an ANT channel identity onto it.
 * Pure, portable, host-tested. On the ESP32 the frames are generated and
 * received by the internal radio (ant_espphy); the nRF24L01+'s ShockBurst
 * engine is why the format is known, but it is not part of the build.
 */
#ifndef ANT_PHY_SHOCKBURST_H
#define ANT_PHY_SHOCKBURST_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ANT_SB_ADDR_MIN      3u
#define ANT_SB_ADDR_MAX      6u
#define ANT_SB_PAYLOAD_MAX   32u
#define ANT_SB_CRC_LEN       2u
/* preamble + max address + max payload + crc */
#define ANT_SB_FRAME_MAX     (1u + ANT_SB_ADDR_MAX + ANT_SB_PAYLOAD_MAX + 2u)

/* ANT+ single-broadcast RF frequency, MHz. */
#define ANT_SB_ANTPLUS_FREQ_MHZ  2457u
/* General ANT default base frequency, MHz. */
#define ANT_SB_ANT_BASE_FREQ_MHZ 2466u

/*
 * CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no input/output reflection,
 * no final xor. This is the CRC the ANT ShockBurst frame carries. Standard
 * check value: crc of ASCII "123456789" == 0x29B1.
 */
uint16_t ant_crc16_ccitt(const uint8_t *data, size_t len);

/* Return the ShockBurst preamble byte for a given first address byte. */
uint8_t ant_sb_preamble_for(uint8_t first_address_byte);

/*
 * Build a full on-air frame: preamble | address | payload | crc(2, big-endian).
 * `address` is `addr_len` bytes MSB-first (as transmitted). Returns total
 * frame length, or 0 on invalid arguments / insufficient capacity.
 */
size_t ant_sb_build_frame(const uint8_t *address, uint8_t addr_len,
                          const uint8_t *payload, uint8_t payload_len,
                          uint8_t *out, size_t out_cap);

/*
 * Verify and unpack a received {address | payload | crc} buffer (no preamble;
 * an nRF24L01+ strips the preamble and hands you address+payload, and here we
 * do the ANT CRC in software). `buf_len` must equal addr_len+payload_len+2.
 * On CRC match, copies the payload out (if `payload_out` != NULL) and returns
 * true. `payload_out` must hold at least `payload_len` bytes.
 */
bool ant_sb_verify_frame(const uint8_t *buf, size_t buf_len,
                         uint8_t addr_len, uint8_t payload_len,
                         uint8_t *payload_out);

/* Pack/unpack an address between a big-endian byte array and a uint64_t,
 * matching the layout used by the reference decoder. */
uint64_t ant_sb_address_to_u64(const uint8_t *address, uint8_t addr_len);
void     ant_sb_address_from_u64(uint64_t addr, uint8_t addr_len,
                                 uint8_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ANT_PHY_SHOCKBURST_H */
