/*
 * antplus_profiles.c - ANT+ device profile encode/decode (see header).
 */
#include "antplus_profiles.h"
#include <string.h>

/* Little-endian 16-bit helpers (ANT is little-endian on the wire). */
static inline uint16_t rd16(const uint8_t *p) {
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}
static inline void wr16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

/* Difference with modular (rollover) arithmetic. */
static inline uint16_t diff16(uint16_t now, uint16_t prev) {
    return (uint16_t)(now - prev); /* wraps naturally */
}
static inline uint8_t diff8(uint8_t now, uint8_t prev) {
    return (uint8_t)(now - prev);
}

/* ============================ Heart Rate ============================ */

bool antplus_hrm_decode(const uint8_t page[8], antplus_hrm_data_t *out)
{
    if (page == NULL || out == NULL) {
        return false;
    }
    memset(out, 0, sizeof(*out));

    uint8_t b0 = page[0];
    out->toggle = (b0 & 0x80u) != 0;
    out->page_number = (uint8_t)(b0 & 0x7Fu);

    /* Bytes 4-7 are common to every HR page. */
    out->heart_beat_event_time = rd16(&page[4]);
    out->heart_beat_count = page[6];
    out->computed_heart_rate = page[7];

    switch (out->page_number) {
    case 1: /* cumulative operating time: 3 bytes, unit 2 s */
        out->operating_time_s =
            ((uint32_t)page[1] | ((uint32_t)page[2] << 8) |
             ((uint32_t)page[3] << 16)) * 2u;
        break;
    case 2: /* manufacturer id + serial (lower 16 bits) */
        out->manufacturer_id = page[1];
        out->serial_number = rd16(&page[2]);
        break;
    case 3: /* hw version / sw version / model */
        out->hw_version = page[1];
        out->sw_version = page[2];
        out->model_number = page[3];
        break;
    case 4: /* previous heartbeat event time */
        out->prev_beat_event_time = rd16(&page[2]);
        break;
    default:
        break;
    }
    return true;
}

void antplus_hrm_encode_page0(const antplus_hrm_data_t *in, bool toggle,
                              uint8_t page[8])
{
    memset(page, 0, 8);
    page[0] = (uint8_t)(toggle ? 0x80u : 0x00u); /* page 0 */
    page[1] = 0xFF;  /* reserved */
    page[2] = 0xFF;
    page[3] = 0xFF;
    wr16(&page[4], in->heart_beat_event_time);
    page[6] = in->heart_beat_count;
    page[7] = in->computed_heart_rate;
}

uint16_t antplus_hrm_bpm_from_events(uint16_t t_prev, uint16_t t_now,
                                     uint8_t beats_between)
{
    uint16_t dt = diff16(t_now, t_prev); /* in 1/1024 s */
    if (dt == 0 || beats_between == 0) {
        return 0;
    }
    /* bpm = beats / (dt/1024 s) * 60 = beats * 1024 * 60 / dt */
    uint32_t bpm = ((uint32_t)beats_between * 1024u * 60u) / dt;
    return (uint16_t)bpm;
}

/* ============================ Bicycle Power ============================ */

bool antplus_power_decode(const uint8_t page[8], antplus_power_data_t *out)
{
    if (page == NULL || out == NULL || page[0] != 0x10u) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    out->event_count = page[1];
    out->pedal_power = page[2];
    out->instantaneous_cadence = page[3];
    out->accumulated_power = rd16(&page[4]);
    out->instantaneous_power = rd16(&page[6]);
    return true;
}

void antplus_power_encode(const antplus_power_data_t *in, uint8_t page[8])
{
    memset(page, 0, 8);
    page[0] = 0x10;
    page[1] = in->event_count;
    page[2] = in->pedal_power;
    page[3] = in->instantaneous_cadence;
    wr16(&page[4], in->accumulated_power);
    wr16(&page[6], in->instantaneous_power);
}

uint16_t antplus_power_average(uint16_t accum_prev, uint16_t accum_now,
                               uint8_t count_prev, uint8_t count_now)
{
    uint8_t d_count = diff8(count_now, count_prev);
    if (d_count == 0) {
        return 0;
    }
    uint16_t d_power = diff16(accum_now, accum_prev);
    return (uint16_t)(d_power / d_count);
}

/* ====================== Bicycle Speed & Cadence ====================== */

bool antplus_spdcad_decode(const uint8_t page[8], antplus_spdcad_data_t *out)
{
    if (page == NULL || out == NULL) {
        return false;
    }
    out->cadence_event_time = rd16(&page[0]);
    out->cumulative_cadence = rd16(&page[2]);
    out->speed_event_time   = rd16(&page[4]);
    out->cumulative_speed   = rd16(&page[6]);
    return true;
}

void antplus_spdcad_encode(const antplus_spdcad_data_t *in, uint8_t page[8])
{
    wr16(&page[0], in->cadence_event_time);
    wr16(&page[2], in->cumulative_cadence);
    wr16(&page[4], in->speed_event_time);
    wr16(&page[6], in->cumulative_speed);
}

uint16_t antplus_spdcad_rpm(uint16_t t_prev, uint16_t t_now,
                            uint16_t rev_prev, uint16_t rev_now)
{
    uint16_t dt = diff16(t_now, t_prev); /* 1/1024 s */
    uint16_t drev = diff16(rev_now, rev_prev);
    if (dt == 0 || drev == 0) {
        return 0;
    }
    /* rpm = revs / (dt/1024 s) * 60 = revs * 1024 * 60 / dt */
    uint32_t rpm = ((uint32_t)drev * 1024u * 60u) / dt;
    return (uint16_t)rpm;
}

uint32_t antplus_spdcad_speed_mm_s(uint16_t t_prev, uint16_t t_now,
                                   uint16_t rev_prev, uint16_t rev_now,
                                   uint32_t wheel_circumference_mm)
{
    uint16_t dt = diff16(t_now, t_prev); /* 1/1024 s */
    uint16_t drev = diff16(rev_now, rev_prev);
    if (dt == 0 || drev == 0) {
        return 0;
    }
    /* distance = drev * circumference (mm) over dt/1024 s */
    /* speed(mm/s) = drev * circ * 1024 / dt */
    uint64_t num = (uint64_t)drev * wheel_circumference_mm * 1024u;
    return (uint32_t)(num / dt);
}
