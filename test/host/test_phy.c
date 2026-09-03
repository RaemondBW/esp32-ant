/* Tests for the ANT ShockBurst PHY frame (ant_phy_shockburst). */
#include "test.h"
#include "ant_phy_shockburst.h"

static int test_crc_standard_vector(void)
{
    /* CRC-16/CCITT-FALSE check value for "123456789" is 0x29B1. This proves
     * our CRC matches the algorithm in the reverse-engineered ANT decoder. */
    const uint8_t s[] = { '1','2','3','4','5','6','7','8','9' };
    CHECK_EQ(ant_crc16_ccitt(s, sizeof(s)), 0x29B1);
    return 0;
}

static int test_crc_empty_is_init(void)
{
    CHECK_EQ(ant_crc16_ccitt((const uint8_t *)"", 0), 0xFFFF);
    return 0;
}

static int test_preamble_selection(void)
{
    /* MSB set -> 0xAA, MSB clear -> 0x55 (Enhanced ShockBurst rule). */
    CHECK_EQ(ant_sb_preamble_for(0x80), 0xAA);
    CHECK_EQ(ant_sb_preamble_for(0xC5), 0xAA);
    CHECK_EQ(ant_sb_preamble_for(0x3B), 0x55);
    CHECK_EQ(ant_sb_preamble_for(0x01), 0x55);
    return 0;
}

static int test_address_pack_roundtrip(void)
{
    /* A real ANT-FS beacon address observed in the DEF CON capture. */
    uint8_t addr[5] = { 0x3B, 0xA3, 0x47, 0x24, 0x01 };
    uint64_t u = ant_sb_address_to_u64(addr, 5);
    CHECK_EQ(u, 0x3BA3472401ULL);
    uint8_t back[5];
    ant_sb_address_from_u64(0x3BA3472401ULL, 5, back);
    for (int i = 0; i < 5; i++) CHECK_EQ(back[i], addr[i]);
    return 0;
}

static int test_build_frame_layout(void)
{
    uint8_t addr[5] = { 0x3B, 0xA3, 0x47, 0x24, 0x01 };
    uint8_t payload[10] = { 0x43,0x24,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
    uint8_t frame[ANT_SB_FRAME_MAX];
    size_t n = ant_sb_build_frame(addr, 5, payload, 10, frame, sizeof(frame));

    /* preamble(1) + addr(5) + payload(10) + crc(2) = 18 */
    CHECK_EQ(n, 18);
    CHECK_EQ(frame[0], 0x55);      /* first addr byte 0x3B -> MSB 0 -> 0x55 */
    CHECK_EQ(frame[1], 0x3B);      /* address follows preamble */
    CHECK_EQ(frame[5], 0x01);      /* last address byte */
    CHECK_EQ(frame[6], 0x43);      /* payload starts */

    /* CRC is over address+payload (bytes 1..15), big-endian at the tail. */
    uint16_t expect = ant_crc16_ccitt(&frame[1], 15);
    CHECK_EQ(frame[16], (uint8_t)(expect >> 8));
    CHECK_EQ(frame[17], (uint8_t)(expect & 0xFF));
    return 0;
}

static int test_build_then_verify(void)
{
    uint8_t addr[5] = { 0xC5, 0x11, 0x22, 0x33, 0x44 };
    uint8_t payload[8] = { 0x84, 0x00, 0x10, 0x20, 0x30, 0x40, 0x02, 0x48 };
    uint8_t frame[ANT_SB_FRAME_MAX];
    size_t n = ant_sb_build_frame(addr, 5, payload, 8, frame, sizeof(frame));
    CHECK_EQ(frame[0], 0xAA);      /* 0xC5 MSB set -> 0xAA preamble */

    /* A receiver (nRF24L01+) strips the preamble; verify addr+payload+crc. */
    uint8_t body_out[8];
    CHECK(ant_sb_verify_frame(&frame[1], n - 1, 5, 8, body_out));
    for (int i = 0; i < 8; i++) CHECK_EQ(body_out[i], payload[i]);
    return 0;
}

static int test_verify_rejects_corruption(void)
{
    uint8_t addr[3] = { 0x71, 0x02, 0x99 };
    uint8_t payload[8] = { 1,2,3,4,5,6,7,8 };
    uint8_t frame[ANT_SB_FRAME_MAX];
    size_t n = ant_sb_build_frame(addr, 3, payload, 8, frame, sizeof(frame));
    /* flip a payload bit; CRC must now fail */
    frame[4] ^= 0x01;
    CHECK(!ant_sb_verify_frame(&frame[1], n - 1, 3, 8, NULL));
    return 0;
}

static int test_build_rejects_bad_args(void)
{
    uint8_t addr[7] = {0}; uint8_t pl[8] = {0}; uint8_t out[ANT_SB_FRAME_MAX];
    CHECK_EQ(ant_sb_build_frame(addr, 2, pl, 8, out, sizeof(out)), 0); /* addr too short */
    CHECK_EQ(ant_sb_build_frame(addr, 7, pl, 8, out, sizeof(out)), 0); /* addr too long */
    CHECK_EQ(ant_sb_build_frame(addr, 5, pl, 0, out, sizeof(out)), 0); /* empty payload */
    CHECK_EQ(ant_sb_build_frame(addr, 5, pl, 8, out, 4), 0);           /* no capacity */
    return 0;
}

void run_phy_tests(void)
{
    RUN(test_crc_standard_vector);
    RUN(test_crc_empty_is_init);
    RUN(test_preamble_selection);
    RUN(test_address_pack_roundtrip);
    RUN(test_build_frame_layout);
    RUN(test_build_then_verify);
    RUN(test_verify_rejects_corruption);
    RUN(test_build_rejects_bad_args);
}
