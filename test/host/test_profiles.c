/* Tests for ANT+ device profile encode/decode. */
#include "test.h"
#include "antplus_profiles.h"

/* ---- Heart Rate ---- */

static int test_hrm_decode_page0(void)
{
    /* page 0, toggle set, event time 0x1234, count 42, hr 75 */
    uint8_t page[8] = { 0x80, 0xFF, 0xFF, 0xFF, 0x34, 0x12, 42, 75 };
    antplus_hrm_data_t d;
    CHECK(antplus_hrm_decode(page, &d));
    CHECK_EQ(d.page_number, 0);
    CHECK(d.toggle);
    CHECK_EQ(d.heart_beat_event_time, 0x1234);
    CHECK_EQ(d.heart_beat_count, 42);
    CHECK_EQ(d.computed_heart_rate, 75);
    return 0;
}

static int test_hrm_decode_page2_manufacturer(void)
{
    uint8_t page[8] = { 0x02, 0x0F, 0xCD, 0xAB, 0x00, 0x00, 1, 60 };
    antplus_hrm_data_t d;
    CHECK(antplus_hrm_decode(page, &d));
    CHECK_EQ(d.page_number, 2);
    CHECK_EQ(d.manufacturer_id, 0x0F);
    CHECK_EQ(d.serial_number, 0xABCD);
    CHECK_EQ(d.computed_heart_rate, 60);
    return 0;
}

static int test_hrm_encode_roundtrip(void)
{
    antplus_hrm_data_t in = {0};
    in.heart_beat_event_time = 0x2222;
    in.heart_beat_count = 100;
    in.computed_heart_rate = 145;
    uint8_t page[8];
    antplus_hrm_encode_page0(&in, true, page);

    antplus_hrm_data_t out;
    CHECK(antplus_hrm_decode(page, &out));
    CHECK_EQ(out.page_number, 0);
    CHECK(out.toggle);
    CHECK_EQ(out.heart_beat_event_time, 0x2222);
    CHECK_EQ(out.heart_beat_count, 100);
    CHECK_EQ(out.computed_heart_rate, 145);
    return 0;
}

static int test_hrm_bpm_calc(void)
{
    /* 1 beat in 1024 ticks (1 second) -> 60 bpm */
    CHECK_EQ(antplus_hrm_bpm_from_events(0, 1024, 1), 60);
    /* 2 beats in 1024 ticks -> 120 bpm */
    CHECK_EQ(antplus_hrm_bpm_from_events(1000, 1000 + 1024, 2), 120);
    /* rollover: prev near top, now wrapped */
    CHECK_EQ(antplus_hrm_bpm_from_events(65000, (uint16_t)(65000 + 1024), 1), 60);
    /* zero dt guarded */
    CHECK_EQ(antplus_hrm_bpm_from_events(500, 500, 1), 0);
    return 0;
}

/* ---- Bicycle Power ---- */

static int test_power_decode(void)
{
    /* page 0x10, event 5, pedal 0xFF, cadence 90, accum 0x0100, instant 250 */
    uint8_t page[8] = { 0x10, 5, 0xFF, 90, 0x00, 0x01, 0xFA, 0x00 };
    antplus_power_data_t d;
    CHECK(antplus_power_decode(page, &d));
    CHECK_EQ(d.event_count, 5);
    CHECK_EQ(d.instantaneous_cadence, 90);
    CHECK_EQ(d.accumulated_power, 0x0100);
    CHECK_EQ(d.instantaneous_power, 250);
    return 0;
}

static int test_power_wrong_page(void)
{
    uint8_t page[8] = { 0x20, 0,0,0,0,0,0,0 };
    antplus_power_data_t d;
    CHECK(!antplus_power_decode(page, &d));
    return 0;
}

static int test_power_encode_roundtrip(void)
{
    antplus_power_data_t in = {0};
    in.event_count = 200;
    in.instantaneous_cadence = 85;
    in.accumulated_power = 12345;
    in.instantaneous_power = 305;
    in.pedal_power = 0xFF;
    uint8_t page[8];
    antplus_power_encode(&in, page);
    antplus_power_data_t out;
    CHECK(antplus_power_decode(page, &out));
    CHECK_EQ(out.event_count, 200);
    CHECK_EQ(out.instantaneous_cadence, 85);
    CHECK_EQ(out.accumulated_power, 12345);
    CHECK_EQ(out.instantaneous_power, 305);
    return 0;
}

static int test_power_average(void)
{
    /* accum went from 1000 to 2000 over 4 events -> 250 W avg */
    CHECK_EQ(antplus_power_average(1000, 2000, 10, 14), 250);
    /* 16-bit accum rollover: 65500 -> 964 = +1000 over 4 -> 250 */
    CHECK_EQ(antplus_power_average(65500, 964, 100, 104), 250);
    /* 8-bit count rollover: 254 -> 2 = 4 events, +1000 -> 250 */
    CHECK_EQ(antplus_power_average(0, 1000, 254, 2), 250);
    /* no events */
    CHECK_EQ(antplus_power_average(100, 200, 5, 5), 0);
    return 0;
}

/* ---- Speed & Cadence ---- */

static int test_spdcad_decode_roundtrip(void)
{
    antplus_spdcad_data_t in = {0};
    in.cadence_event_time = 0x1111;
    in.cumulative_cadence = 0x2222;
    in.speed_event_time = 0x3333;
    in.cumulative_speed = 0x4444;
    uint8_t page[8];
    antplus_spdcad_encode(&in, page);
    antplus_spdcad_data_t out;
    CHECK(antplus_spdcad_decode(page, &out));
    CHECK_EQ(out.cadence_event_time, 0x1111);
    CHECK_EQ(out.cumulative_cadence, 0x2222);
    CHECK_EQ(out.speed_event_time, 0x3333);
    CHECK_EQ(out.cumulative_speed, 0x4444);
    return 0;
}

static int test_spdcad_rpm(void)
{
    /* 1 rev in 1024 ticks -> 60 rpm */
    CHECK_EQ(antplus_spdcad_rpm(0, 1024, 0, 1), 60);
    /* 2 revs in 1024 ticks -> 120 rpm */
    CHECK_EQ(antplus_spdcad_rpm(500, 500 + 1024, 100, 102), 120);
    /* rollover on both fields: dt=2048 ticks, drev=3 -> 90 rpm */
    CHECK_EQ(antplus_spdcad_rpm(65500, 2012, 65534, 1), 90);
    return 0;
}

static int test_spdcad_speed(void)
{
    /* 1 wheel rev of 2100 mm in 1 s -> 2100 mm/s */
    CHECK_EQ(antplus_spdcad_speed_mm_s(0, 1024, 0, 1, 2100), 2100);
    /* 2 revs in 1 s -> 4200 mm/s */
    CHECK_EQ(antplus_spdcad_speed_mm_s(0, 1024, 0, 2, 2100), 4200);
    return 0;
}

void run_profile_tests(void)
{
    RUN(test_hrm_decode_page0);
    RUN(test_hrm_decode_page2_manufacturer);
    RUN(test_hrm_encode_roundtrip);
    RUN(test_hrm_bpm_calc);
    RUN(test_power_decode);
    RUN(test_power_wrong_page);
    RUN(test_power_encode_roundtrip);
    RUN(test_power_average);
    RUN(test_spdcad_decode_roundtrip);
    RUN(test_spdcad_rpm);
    RUN(test_spdcad_speed);
}
