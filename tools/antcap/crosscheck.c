/*
 * crosscheck.c - prints the same reference vectors as `python3 antframe.py`,
 * computed by the firmware's own C (ant_phy_shockburst.c, ant_sb_link.c,
 * antplus_profiles.c). `make check` diffs the two outputs byte for byte.
 */
#include <stdio.h>
#include <string.h>
#include "ant_phy_shockburst.h"
#include "ant_sb_link.h"
#include "antplus_profiles.h"

static const uint8_t ANTPLUS_KEY[8] = ANT_NETWORK_KEY_ANTPLUS;
static const uint8_t PUBLIC_KEY[8]  = ANT_NETWORK_KEY_PUBLIC;
static const uint8_t ANTFS_KEY[8]   = ANT_NETWORK_KEY_ANTFS;
static const uint8_t ZERO_KEY[8]    = { 0 };
static const uint8_t JUNK_KEY[8]    = { 1, 2, 3, 4, 5, 6, 7, 8 };

static void print_hex(const char *name, const uint8_t *b, size_t n)
{
    printf("%s", name);
    for (size_t i = 0; i < n; i++) printf(" %02x", b[i]);
    printf("\n");
}

static void print_marker(const char *name, const uint8_t *key)
{
    uint8_t m[ANT_SB_LINK_MARKER_LEN];
    ant_sb_link_network_marker(key, m);
    print_hex(name, m, sizeof(m));
}

static void print_link_frame(const char *name, uint16_t dev, uint8_t type, uint8_t trans,
                             const uint8_t *key, uint8_t ctrl, const uint8_t data[8])
{
    ant_sb_link_t link;
    uint8_t frame[ANT_SB_FRAME_MAX];
    ant_sb_link_from_identity(&link, 57, ANT_SB_LINK_ADDR_LEN, dev, type, trans);
    ant_sb_link_set_network(&link, key);
    size_t n = ant_sb_link_build(&link, ctrl, data, frame, sizeof(frame));
    print_hex(name, frame, n);
}

int main(void)
{
    printf("crc123456789 %04x\n", ant_crc16_ccitt((const uint8_t *)"123456789", 9));
    print_marker("marker_public", PUBLIC_KEY);
    print_marker("marker_zero", ZERO_KEY);
    print_marker("marker_antplus", ANTPLUS_KEY);
    print_marker("marker_antfs", ANTFS_KEY);
    print_marker("marker_junk", JUNK_KEY);
    printf("valid %d%d%d%d%d\n",
           ant_sb_network_key_valid(ANTPLUS_KEY), ant_sb_network_key_valid(PUBLIC_KEY),
           ant_sb_network_key_valid(ANTFS_KEY), ant_sb_network_key_valid(ZERO_KEY),
           ant_sb_network_key_valid(JUNK_KEY));

    antplus_hrm_data_t hr = { .computed_heart_rate = 72, .heart_beat_count = 1,
                              .heart_beat_event_time = 1024 };
    uint8_t page[8];
    antplus_hrm_encode_page0(&hr, false, page);
    print_link_frame("frame_public", 0x1234, ANTPLUS_DEVTYPE_HRM, 1, PUBLIC_KEY, 0, page);
    print_link_frame("frame_antplus", 0x1234, ANTPLUS_DEVTYPE_HRM, 1, ANTPLUS_KEY, 0, page);
    print_link_frame("frame_ack", 0x1234, ANTPLUS_DEVTYPE_HRM, 1, ANTPLUS_KEY,
                     ANT_SB_CTRL_ACK | ANT_SB_CTRL_REVERSE, page);

    uint8_t seq[8];
    for (int i = 0; i < 8; i++) seq[i] = (uint8_t)i;
    print_link_frame("frame_burst", 0x3042, ANTPLUS_DEVTYPE_HRM, 1, ANTPLUS_KEY,
                     ANT_SB_CTRL_BURST | ANT_SB_CTRL_REVERSE | ANT_SB_CTRL_LAST | 3, seq);

    /* The strap frame the S3 received off the air (device 0x6941, 67 bpm). */
    static const uint8_t strap_page[8] = { 0x00, 0xff, 0xff, 0xff, 0x9a, 0x47, 0xc7, 0x43 };
    print_link_frame("frame_strap", 0x6941, 0x78, 1, ANTPLUS_KEY, 0, strap_page);

    /* 3-byte address, generic builder */
    static const uint8_t a3[3] = { 0xE7, 0xE7, 0xE7 };
    uint8_t payload[9] = { 0x40, 0, 1, 2, 3, 4, 5, 6, 7 };
    uint8_t frame[ANT_SB_FRAME_MAX];
    size_t n = ant_sb_build_frame(a3, 3, payload, 9, frame, sizeof(frame));
    print_hex("frame_addr3", frame, n);
    return 0;
}
