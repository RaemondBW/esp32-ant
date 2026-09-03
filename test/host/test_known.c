/*
 * test_known.c - the paired-device table and its blob form.
 */
#include "test.h"
#include "ant_known.h"
#include "ant_message.h"

static int test_put_find_replace(void)
{
    ant_known_t k;
    ant_known_init(&k);
    CHECK_EQ(k.count, 0);
    CHECK(ant_known_find(&k, 120) == NULL);

    ant_known_device_t hrm = { 0x6941, 120, 1 };
    ant_known_device_t pwr = { 0x0102, 11, 5 };
    CHECK(ant_known_put(&k, &hrm));
    CHECK(ant_known_put(&k, &pwr));
    CHECK_EQ(k.count, 2);
    const ant_known_device_t *d = ant_known_find(&k, 120);
    CHECK(d != NULL);
    CHECK_EQ(d->device_num, 0x6941); CHECK_EQ(d->trans_type, 1);
    CHECK(ant_known_find(&k, 11) != NULL);
    /* The pairing bit is not part of the key. */
    CHECK(ant_known_find(&k, 120 | ANT_DEVICE_TYPE_PAIRING_BIT) == d);

    /* A new strap replaces the old one: one device per type. */
    ant_known_device_t hrm2 = { 0x1234, 120 | ANT_DEVICE_TYPE_PAIRING_BIT, 1 };
    CHECK(ant_known_put(&k, &hrm2));
    CHECK_EQ(k.count, 2);
    d = ant_known_find(&k, 120);
    CHECK_EQ(d->device_num, 0x1234);
    CHECK_EQ(d->device_type, 120);          /* stored without the bit */

    /* Re-putting the same device is a no-op. */
    CHECK(ant_known_put(&k, &hrm2));
    CHECK_EQ(k.count, 2);
    return 0;
}

static int test_rejects_non_identities_and_full(void)
{
    ant_known_t k;
    ant_known_init(&k);
    ant_known_device_t wild = { 0, 120, 1 };
    ant_known_device_t notype = { 0x1234, 0, 1 };
    CHECK(!ant_known_put(&k, &wild));
    CHECK(!ant_known_put(&k, &notype));
    CHECK(!ant_known_put(&k, NULL));
    CHECK_EQ(k.count, 0);
    for (unsigned i = 0; i < ANT_KNOWN_MAX; i++) {
        ant_known_device_t d = { (uint16_t)(0x100 + i), (uint8_t)(1 + i), 1 };
        CHECK(ant_known_put(&k, &d));
    }
    ant_known_device_t extra = { 0x999, 100, 1 };
    CHECK(!ant_known_put(&k, &extra));
    CHECK_EQ(k.count, ANT_KNOWN_MAX);
    /* Replacing an existing type still works when full. */
    ant_known_device_t repl = { 0x777, 3, 2 };
    CHECK(ant_known_put(&k, &repl));
    CHECK_EQ(ant_known_find(&k, 3)->device_num, 0x777);
    return 0;
}

static int test_remove_keeps_order(void)
{
    ant_known_t k;
    ant_known_init(&k);
    ant_known_device_t a = { 1, 10, 1 }, b = { 2, 20, 1 }, c = { 3, 30, 1 };
    ant_known_put(&k, &a); ant_known_put(&k, &b); ant_known_put(&k, &c);
    CHECK(ant_known_remove(&k, 20));
    CHECK(!ant_known_remove(&k, 20));
    CHECK_EQ(k.count, 2);
    CHECK_EQ(k.dev[0].device_num, 1);
    CHECK_EQ(k.dev[1].device_num, 3);
    CHECK_EQ(k.dev[2].device_num, 0);
    ant_known_clear(&k);
    CHECK_EQ(k.count, 0);
    return 0;
}

static int test_blob_round_trip(void)
{
    ant_known_t k, back;
    ant_known_init(&k);
    ant_known_device_t hrm = { 0x6941, 120, 1 }, pwr = { 0xABCD, 11, 5 };
    ant_known_put(&k, &hrm); ant_known_put(&k, &pwr);

    uint8_t blob[ANT_KNOWN_MAX * 4];
    CHECK_EQ(ant_known_to_blob(&k, blob, sizeof(blob)), 8);
    static const uint8_t expect[8] = { 0x41, 0x69, 120, 1, 0xCD, 0xAB, 11, 5 };
    CHECK_MEM(blob, expect, 8);

    CHECK_EQ(ant_known_from_blob(&back, blob, 8), 2);
    CHECK_EQ(back.count, 2);
    CHECK_EQ(ant_known_find(&back, 120)->device_num, 0x6941);
    CHECK_EQ(ant_known_find(&back, 11)->device_num, 0xABCD);
    CHECK_EQ(ant_known_find(&back, 11)->trans_type, 5);

    /* Truncated / garbage entries are dropped, not trusted. */
    uint8_t bad[12] = { 0x41, 0x69, 120, 1,  0, 0, 33, 1,  0x11, 0x22, 0, 1 };
    CHECK_EQ(ant_known_from_blob(&back, bad, sizeof(bad)), 1);
    CHECK_EQ(ant_known_from_blob(&back, bad, 7), 1);
    CHECK_EQ(ant_known_from_blob(&back, NULL, 0), 0);
    /* A too-small buffer writes whole entries only. */
    CHECK_EQ(ant_known_to_blob(&k, blob, 5), 4);
    return 0;
}

void run_known_tests(void)
{
    RUN(test_put_find_replace);
    RUN(test_rejects_non_identities_and_full);
    RUN(test_remove_keeps_order);
    RUN(test_blob_round_trip);
}
