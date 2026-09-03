/* Tests for the ANT message framing layer. */
#include "test.h"
#include "ant_message.h"

static int test_checksum(void)
{
    uint8_t buf[] = { 0xA4, 0x01, 0x4A, 0x00 };
    /* XOR of A4 ^ 01 ^ 4A ^ 00 */
    uint8_t expect = 0xA4 ^ 0x01 ^ 0x4A ^ 0x00;
    CHECK_EQ(ant_checksum(buf, 4), expect);
    return 0;
}

static int test_encode_reset(void)
{
    uint8_t out[ANT_MAX_FRAME_LEN];
    size_t n = ant_build_reset(out, sizeof(out));
    /* reset system: A4 01 4A 00 <cs> */
    CHECK_EQ(n, 5);
    CHECK_EQ(out[0], ANT_SYNC_TX);
    CHECK_EQ(out[1], 0x01);
    CHECK_EQ(out[2], ANT_MSG_RESET_SYSTEM);
    CHECK_EQ(out[3], 0x00);
    CHECK_EQ(out[4], (uint8_t)(0xA4 ^ 0x01 ^ 0x4A ^ 0x00));
    return 0;
}

static int test_encode_set_network_key(void)
{
    uint8_t out[ANT_MAX_FRAME_LEN];
    uint8_t key[8] = { 0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45 };
    size_t n = ant_build_set_network_key(0, key, out, sizeof(out));
    /* len = 9 (net + 8 key), total = 13 */
    CHECK_EQ(n, 13);
    CHECK_EQ(out[1], 9);
    CHECK_EQ(out[2], ANT_MSG_SET_NETWORK_KEY);
    CHECK_EQ(out[3], 0); /* network number */
    CHECK_EQ(out[4], 0xB9);
    CHECK_EQ(out[11], 0x45);
    /* verify checksum validity by decoding */
    ant_message_t m; size_t used;
    CHECK(ant_message_decode(out, n, &m, &used));
    CHECK_EQ(used, n);
    return 0;
}

static int test_encode_channel_id(void)
{
    uint8_t out[ANT_MAX_FRAME_LEN];
    size_t n = ant_build_set_channel_id(0, 0xBEEF, 120, 5, out, sizeof(out));
    CHECK_EQ(n, 9);
    CHECK_EQ(out[3], 0);        /* channel */
    CHECK_EQ(out[4], 0xEF);     /* dev num LSB */
    CHECK_EQ(out[5], 0xBE);     /* dev num MSB */
    CHECK_EQ(out[6], 120);      /* dev type */
    CHECK_EQ(out[7], 5);        /* trans type */
    return 0;
}

static int test_encode_period_le(void)
{
    uint8_t out[ANT_MAX_FRAME_LEN];
    size_t n = ant_build_set_period(0, 8070, out, sizeof(out));
    CHECK(n > 0);
    CHECK_EQ(out[4], (uint8_t)(8070 & 0xFF));  /* 0x86 */
    CHECK_EQ(out[5], (uint8_t)(8070 >> 8));    /* 0x1F */
    return 0;
}

static int test_roundtrip_decode(void)
{
    uint8_t payload[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    uint8_t out[ANT_MAX_FRAME_LEN];
    size_t n = ant_build_broadcast_data(0, payload, out, sizeof(out));
    ant_message_t m; size_t used = 0;
    CHECK(ant_message_decode(out, n, &m, &used));
    CHECK_EQ(used, n);
    CHECK_EQ(m.msg_id, ANT_MSG_BROADCAST_DATA);
    CHECK_EQ(m.data_len, 9);
    CHECK_EQ(m.data[0], 0); /* channel */
    for (int i = 0; i < 8; i++) CHECK_EQ(m.data[1 + i], payload[i]);
    return 0;
}

static int test_decode_bad_checksum(void)
{
    uint8_t out[ANT_MAX_FRAME_LEN];
    size_t n = ant_build_reset(out, sizeof(out));
    out[n - 1] ^= 0xFF; /* corrupt checksum */
    ant_message_t m; size_t used;
    CHECK(!ant_message_decode(out, n, &m, &used));
    return 0;
}

static int test_decode_partial(void)
{
    uint8_t out[ANT_MAX_FRAME_LEN];
    size_t n = ant_build_reset(out, sizeof(out));
    ant_message_t m; size_t used;
    /* one byte short - should report not-yet-complete */
    CHECK(!ant_message_decode(out, n - 1, &m, &used));
    return 0;
}

static int test_parser_stream(void)
{
    /* Two frames back to back, plus leading garbage, fed byte by byte. */
    uint8_t f1[ANT_MAX_FRAME_LEN], f2[ANT_MAX_FRAME_LEN];
    size_t n1 = ant_build_reset(f1, sizeof(f1));
    uint8_t pl[8] = { 0x10, 20, 30, 40, 50, 60, 70, 80 };
    size_t n2 = ant_build_broadcast_data(1, pl, f2, sizeof(f2));

    uint8_t stream[80];
    size_t s = 0;
    stream[s++] = 0x00; stream[s++] = 0xFF; /* garbage before sync */
    memcpy(&stream[s], f1, n1); s += n1;
    stream[s++] = 0x11;                     /* garbage between frames */
    memcpy(&stream[s], f2, n2); s += n2;

    ant_parser_t p; ant_parser_init(&p);
    ant_message_t m;
    int got = 0;
    uint8_t ids[4] = {0};
    for (size_t i = 0; i < s; i++) {
        if (ant_parser_push(&p, stream[i], &m)) {
            if (got < 4) ids[got] = m.msg_id;
            got++;
        }
    }
    CHECK_EQ(got, 2);
    CHECK_EQ(ids[0], ANT_MSG_RESET_SYSTEM);
    CHECK_EQ(ids[1], ANT_MSG_BROADCAST_DATA);
    return 0;
}

static int test_parser_resync_after_corruption(void)
{
    uint8_t f[ANT_MAX_FRAME_LEN];
    uint8_t pl[8] = { 1,2,3,4,5,6,7,8 };
    size_t n = ant_build_broadcast_data(0, pl, f, sizeof(f));

    ant_parser_t p; ant_parser_init(&p);
    ant_message_t m;
    /* corrupt the checksum of first copy, then send a clean copy */
    uint8_t corrupt = f[n - 1] ^ 0xAA;
    int got = 0;
    for (size_t i = 0; i < n - 1; i++) ant_parser_push(&p, f[i], &m);
    ant_parser_push(&p, corrupt, &m); /* bad checksum, should resync */
    for (size_t i = 0; i < n; i++) {
        if (ant_parser_push(&p, f[i], &m)) got++;
    }
    CHECK_EQ(got, 1);
    return 0;
}

void run_message_tests(void)
{
    RUN(test_checksum);
    RUN(test_encode_reset);
    RUN(test_encode_set_network_key);
    RUN(test_encode_channel_id);
    RUN(test_encode_period_le);
    RUN(test_roundtrip_decode);
    RUN(test_decode_bad_checksum);
    RUN(test_decode_partial);
    RUN(test_parser_stream);
    RUN(test_parser_resync_after_corruption);
}
