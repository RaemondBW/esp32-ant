/* Tests for the ANT+ identity <-> ShockBurst PHY mapping (ant_sb_link). */
#include "test.h"
#include "ant_sb_link.h"
#include "ant_message.h"

/* ANT+ HRM identity used across these tests. */
#define HRM_DEVTYPE   120u
#define ANTPLUS_FREQ  57u   /* 2457 MHz */
#define ADDR          ((int)ANT_SB_LINK_ADDR_LEN)

static const uint8_t ANTPLUS_KEY[8] = ANT_NETWORK_KEY_ANTPLUS;
static const uint8_t PUBLIC_KEY[8]  = ANT_NETWORK_KEY_PUBLIC;
static const uint8_t ANTFS_KEY[8]   = ANT_NETWORK_KEY_ANTFS;

static int test_rf_frequency_is_exact(void)
{
    /* ANT rf_freq is the offset above 2400 MHz. ANT+ freq 57 must map to the
     * absolute frequency 2457 MHz that the internal-radio PHY then tunes to. */
    ant_sb_link_t link;
    CHECK(ant_sb_link_from_identity(&link, ANTPLUS_FREQ, ADDR, 0x1234, HRM_DEVTYPE, 1));
    CHECK_EQ(link.freq_mhz, 2457);
    CHECK_EQ(link.payload_len, ANT_SB_LINK_PAYLOAD_LEN);
    CHECK_EQ(link.addr_len, 6);
    return 0;
}

static int test_address_layout(void)
{
    /* The real layout: marker(2) | devnum_hi | devnum_lo | devtype | trans. */
    ant_sb_link_t link;
    CHECK(ant_sb_link_from_identity(&link, ANTPLUS_FREQ, ADDR, 0xBEEF, HRM_DEVTYPE, 0x05));
    CHECK_EQ(link.address[0], 0xA6);
    CHECK_EQ(link.address[1], 0xC5);
    CHECK_EQ(link.address[2], 0xBE);
    CHECK_EQ(link.address[3], 0xEF);
    CHECK_EQ(link.address[4], HRM_DEVTYPE);
    CHECK_EQ(link.address[5], 0x05);
    /* Marker MSB set -> ShockBurst preamble will be 0xAA. */
    CHECK_EQ(ant_sb_preamble_for(link.address[0]), 0xAA);
    return 0;
}

static int test_network_markers(void)
{
    /* SoftDevice derivation: the three well-known keys and their markers
     * (ANT+ a6c5 is also what the bench strap transmits). */
    uint8_t m[2];
    CHECK(ant_sb_network_key_valid(ANTPLUS_KEY));
    ant_sb_link_network_marker(ANTPLUS_KEY, m);
    CHECK_EQ(m[0], 0xA6); CHECK_EQ(m[1], 0xC5);
    CHECK(ant_sb_network_key_valid(ANTFS_KEY));
    ant_sb_link_network_marker(ANTFS_KEY, m);
    CHECK_EQ(m[0], 0x3B); CHECK_EQ(m[1], 0xA3);
    CHECK(ant_sb_network_key_valid(PUBLIC_KEY));
    ant_sb_link_network_marker(PUBLIC_KEY, m);
    CHECK_EQ(m[0], 0x5B); CHECK_EQ(m[1], 0x25);
    /* The all-zero key is "network 0 default" = the public network. */
    uint8_t zero[8] = { 0 };
    uint8_t mz[2];
    ant_sb_link_network_marker(zero, mz);
    CHECK_EQ(mz[0], 0x5B); CHECK_EQ(mz[1], 0x25);
    /* A made-up key is rejected by the SoftDevice check but still yields a
     * deterministic marker with the preamble bit set. */
    uint8_t junk[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    CHECK(!ant_sb_network_key_valid(junk));
    ant_sb_link_network_marker(junk, m);
    CHECK(m[0] & 0x80);
    uint8_t m2[2];
    ant_sb_link_network_marker(junk, m2);
    CHECK_EQ(m[0], m2[0]); CHECK_EQ(m[1], m2[1]);
    return 0;
}

static int test_set_network_rebinds_marker(void)
{
    ant_sb_link_t link;
    CHECK(ant_sb_link_from_identity(&link, ANTPLUS_FREQ, ADDR, 0x6941, HRM_DEVTYPE, 1));
    ant_sb_link_set_network(&link, ANTFS_KEY);
    CHECK_EQ(link.address[0], 0x3B);
    CHECK_EQ(link.address[1], 0xA3);
    CHECK_EQ(link.address[2], 0x69);   /* identity untouched */
    ant_sb_link_set_network(&link, ANTPLUS_KEY);
    CHECK_EQ(link.address[0], 0xA6);
    CHECK_EQ(link.address[1], 0xC5);
    return 0;
}

static int test_same_identity_same_address(void)
{
    /* Two endpoints given the same identity must derive identical PHY params,
     * otherwise they could never rendezvous on the air. */
    ant_sb_link_t a, b;
    CHECK(ant_sb_link_from_identity(&a, ANTPLUS_FREQ, ADDR, 0x2211, HRM_DEVTYPE, 1));
    CHECK(ant_sb_link_from_identity(&b, ANTPLUS_FREQ, ADDR, 0x2211, HRM_DEVTYPE, 1));
    CHECK_EQ(a.freq_mhz, b.freq_mhz);
    CHECK_EQ(a.addr_len, b.addr_len);
    for (int i = 0; i < ADDR; i++) CHECK_EQ(a.address[i], b.address[i]);
    return 0;
}

static int test_distinct_identity_distinct_address(void)
{
    /* Different device numbers must yield different sync words so two nearby
     * sensors don't collide. */
    ant_sb_link_t a, b;
    CHECK(ant_sb_link_from_identity(&a, ANTPLUS_FREQ, ADDR, 0x0001, HRM_DEVTYPE, 1));
    CHECK(ant_sb_link_from_identity(&b, ANTPLUS_FREQ, ADDR, 0x0002, HRM_DEVTYPE, 1));
    int differs = 0;
    for (int i = 0; i < ADDR; i++) if (a.address[i] != b.address[i]) differs = 1;
    CHECK(differs);
    return 0;
}

static int test_addr_len_bounds(void)
{
    /* Only the real 6-byte address is an ANT link. */
    ant_sb_link_t link;
    CHECK(!ant_sb_link_from_identity(&link, ANTPLUS_FREQ, 5, 1, HRM_DEVTYPE, 1));
    CHECK(!ant_sb_link_from_identity(&link, ANTPLUS_FREQ, 7, 1, HRM_DEVTYPE, 1));
    CHECK(ant_sb_link_from_identity(&link, ANTPLUS_FREQ, 6, 1, HRM_DEVTYPE, 1));
    CHECK_EQ(link.addr_len, 6);
    return 0;
}

static int test_master_slave_roundtrip(void)
{
    /* Master builds a broadcast; a slave with the SAME identity recovers the
     * exact 8-byte page. This is the end-to-end air path (minus the radio). */
    ant_sb_link_t master, slave;
    CHECK(ant_sb_link_from_identity(&master, ANTPLUS_FREQ, ADDR, 0xABCD, HRM_DEVTYPE, 1));
    CHECK(ant_sb_link_from_identity(&slave,  ANTPLUS_FREQ, ADDR, 0xABCD, HRM_DEVTYPE, 1));

    uint8_t page[8] = { 0x00, 0x12, 0x34, 0x56, 0x11, 0x22, 0x33, 72 };
    uint8_t frame[ANT_SB_FRAME_MAX];
    size_t n = ant_sb_link_build_broadcast(&master, page, frame, sizeof(frame));
    /* preamble(1) + addr(6) + ctrl(1)+data(8) + crc(2) = 18 */
    CHECK_EQ(n, 18);
    CHECK_EQ(n, ANT_SB_LINK_FRAME_LEN);
    CHECK_EQ(frame[0], 0xAA);   /* marker 0xA6 MSB set */
    CHECK_EQ(frame[7], 0x0A);   /* the flags byte a commercial sensor sends */

    /* The radio strips the preamble; the slave verifies addr+payload+crc. */
    uint8_t got[8];
    CHECK(ant_sb_link_recv_broadcast(&slave, &frame[1], n - 1, got));
    for (int i = 0; i < 8; i++) CHECK_EQ(got[i], page[i]);
    return 0;
}

static int test_real_strap_frame(void)
{
    /* A frame captured off the air from a commercial ANT+ heart-rate strap
     * (device 0x6941, HRM, trans 1; page 0, 67 bpm) and received by the
     * ESP32-S3 - the ground truth this module must reproduce. */
    static const uint8_t air[17] = {
        0xa6, 0xc5, 0x69, 0x41, 0x78, 0x01,
        0x0a, 0x00, 0xff, 0xff, 0xff, 0x9a, 0x47, 0xc7, 0x43,
        0xbf, 0xd2
    };
    ant_sb_link_t link;
    CHECK(ant_sb_link_from_identity(&link, ANTPLUS_FREQ, ADDR, 0x6941, 0x78, 1));
    ant_sb_link_set_network(&link, ANTPLUS_KEY);
    uint8_t page[8];
    CHECK(ant_sb_link_recv_broadcast(&link, air, sizeof(air), page));
    CHECK_EQ(page[7], 67);              /* computed heart rate */
    CHECK_EQ(page[6], 0xC7);            /* beat count */

    /* And we would put exactly those bytes on the air ourselves. */
    uint8_t frame[ANT_SB_FRAME_MAX];
    size_t n = ant_sb_link_build_broadcast(&link, page, frame, sizeof(frame));
    CHECK_EQ(n, 18);
    CHECK_EQ(frame[0], 0xAA);
    CHECK_MEM(&frame[1], air, 17);

    uint16_t dev; uint8_t type, trans;
    ant_sb_link_identity_from_address(air, ADDR, &dev, &type, &trans);
    CHECK_EQ(dev, 0x6941); CHECK_EQ(type, 0x78); CHECK_EQ(trans, 1);
    return 0;
}

static int test_search_mask(void)
{
    uint8_t mask[ANT_SB_ADDR_MAX];
    ant_sb_link_search_mask(ADDR, 0, HRM_DEVTYPE, 0, mask);
    CHECK_EQ(mask[0], 0xFF); CHECK_EQ(mask[1], 0xFF);   /* marker always */
    CHECK_EQ(mask[2], 0x00); CHECK_EQ(mask[3], 0x00);   /* wildcard device */
    CHECK_EQ(mask[4], 0x7F);                            /* device type given, pairing bit free */
    CHECK_EQ(mask[5], 0x00);                            /* wildcard trans */
    ant_sb_link_search_mask(ADDR, 0x6941, HRM_DEVTYPE, 1, mask);
    for (int i = 0; i < ADDR; i++) CHECK_EQ(mask[i], i == 4 ? 0x7F : 0xFF);
    return 0;
}

static int test_search_mask_pairing_bit(void)
{
    /* A plain type search must not care about the master's pairing bit; a
     * search that sets the bit demands it; a wildcard type ignores the byte. */
    uint8_t mask[ANT_SB_ADDR_MAX];
    ant_sb_link_search_mask(ADDR, 0, HRM_DEVTYPE, 0, mask);
    CHECK_EQ(mask[4], 0x7F);
    ant_sb_link_search_mask(ADDR, 0, HRM_DEVTYPE | ANT_DEVICE_TYPE_PAIRING_BIT, 0, mask);
    CHECK_EQ(mask[4], 0xFF);
    ant_sb_link_search_mask(ADDR, 0, 0, 0, mask);
    CHECK_EQ(mask[4], 0x00);
    return 0;
}

static int test_wrong_identity_rejected(void)
{
    /* A frame from one identity must NOT verify against another: the CRC is over
     * address+payload, so a different device number breaks the check even if the
     * raw bytes were handed up. The address is a filter. */
    ant_sb_link_t master, other;
    CHECK(ant_sb_link_from_identity(&master, ANTPLUS_FREQ, ADDR, 0x1000, HRM_DEVTYPE, 1));
    CHECK(ant_sb_link_from_identity(&other,  ANTPLUS_FREQ, ADDR, 0x2000, HRM_DEVTYPE, 1));

    uint8_t page[8] = { 1,2,3,4,5,6,7,8 };
    uint8_t frame[ANT_SB_FRAME_MAX];
    CHECK(ant_sb_link_build_broadcast(&master, page, frame, sizeof(frame)) != 0);

    /* Feed the master's frame body but verify with the master's own address
     * substituted out: reconstruct as if 'other' received these raw bytes.
     * Since verify recomputes CRC over other.address||payload, it must fail. */
    uint8_t body[ANT_SB_ADDR_MAX + 9 + 2];
    memcpy(body, other.address, other.addr_len);           /* other's address */
    memcpy(body + other.addr_len, &frame[1 + master.addr_len], 9 + 2); /* ctrl+page+crc from master */
    CHECK(!ant_sb_link_recv_broadcast(&other, body, other.addr_len + 9 + 2, NULL));
    return 0;
}

static int test_burst_ctrl_keeps_sequence(void)
{
    ant_sb_link_t link;
    CHECK(ant_sb_link_from_identity(&link, ANTPLUS_FREQ, ADDR, 0x3042, HRM_DEVTYPE, 1));
    uint8_t data[8] = { 0 };
    uint8_t frame[ANT_SB_FRAME_MAX];
    CHECK(ant_sb_link_build(&link, ANT_SB_CTRL_BURST | ANT_SB_CTRL_LAST | 3, data,
                            frame, sizeof(frame)) == 18);
    CHECK_EQ(frame[7], ANT_SB_CTRL_BURST | ANT_SB_CTRL_LAST | 3);
    CHECK(ant_sb_link_build(&link, ANT_SB_CTRL_ACK | ANT_SB_CTRL_REVERSE, data,
                            frame, sizeof(frame)) == 18);
    CHECK_EQ(frame[7], ANT_SB_CTRL_ACK | ANT_SB_CTRL_REVERSE | ANT_SB_CTRL_TAG);
    return 0;
}

void run_link_tests(void)
{
    RUN(test_rf_frequency_is_exact);
    RUN(test_address_layout);
    RUN(test_network_markers);
    RUN(test_set_network_rebinds_marker);
    RUN(test_same_identity_same_address);
    RUN(test_distinct_identity_distinct_address);
    RUN(test_addr_len_bounds);
    RUN(test_master_slave_roundtrip);
    RUN(test_real_strap_frame);
    RUN(test_search_mask);
    RUN(test_search_mask_pairing_bit);
    RUN(test_wrong_identity_rejected);
    RUN(test_burst_ctrl_keeps_sequence);
}
