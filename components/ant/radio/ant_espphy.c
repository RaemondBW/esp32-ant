/*
 * ant_espphy.c - ANT on the ESP32-S3 / ESP32-C3 BLE controller.
 *
 * See ant_espphy.h for the idea. The controller-internals part (hook indices,
 * exchange-memory layout, control-structure fields)
 * is derived from ESPwn32 / esperanto by Romain Cayre et al. (WOOT 2023) and
 * was re-verified against the ESP-IDF v5.4 libbtdm_app.a for the S3.
 *
 * Execution contexts:
 *   - hook_test_evt_start / hook_test_rx_isr run inside the BT controller
 *     (interrupt or its high-priority task): IRAM, no logging, no blocking,
 *     plain stores into the device struct.
 *   - the ant_phy_t entry points run in the ANT task; mode changes issue HCI
 *     commands and wait for the command-complete event.
 */
#include "ant_espphy.h"

#if defined(ESP_PLATFORM)
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_attr.h"

static const char *TAG = "ant_espphy";

uint32_t ant_espphy_ticks(void)
{
    /* us * 32768 / 1e6 == us * 512 / 15625, exact in 64-bit */
    return (uint32_t)(((uint64_t)esp_timer_get_time() * 512u) / 15625u);
}

const char *ant_espphy_status_str(ant_espphy_status_t s)
{
    switch (s) {
    case ANT_ESPPHY_OK:              return "ok";
    case ANT_ESPPHY_ERR_UNSUPPORTED: return "unsupported-target";
    case ANT_ESPPHY_ERR_BT:          return "bt-controller-failed";
    case ANT_ESPPHY_ERR_ARG:         return "bad-arg";
    default:                         return "?";
    }
}

const char *ant_espphy_mode_str(ant_espphy_mode_t m)
{
    switch (m) {
    case ANT_ESPPHY_MODE_IDLE: return "idle";
    case ANT_ESPPHY_MODE_RX:   return "rx";
    case ANT_ESPPHY_MODE_TX:   return "tx";
    default:                   return "?";
    }
}

ant_phy_t *ant_espphy_phy(ant_espphy_t *dev)
{
    return &dev->phy;
}

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_bt.h"

/* ------------------- RW-BLE controller internals (ESPwn32) ---------------- */

/* r_ip_funcs_p: the controller's writable table of link-layer function
 * pointers (ROM code calls through it, so patching an entry hooks the ROM). */
extern void **r_ip_funcs_p;
extern void  *p_lld_env;
extern void  *r_emi_get_mem_addr_by_offset(uint16_t offset);

/* Indices into r_ip_funcs_p (from the relocations of ip_funcs.o in the
 * ESP-IDF v5.4 libbtdm_app.a for the S3; the C3 table is laid out the same). */
#define IPF_LLD_TEST_EVT_START_CBK  127   /* void (sch_arb_elt *evt)   - event programmed */
#define IPF_LLD_TEST_RX_ISR         132   /* void (timestamp)          - RX interrupt      */

/* Scan-path indices, for the coexist (shared-radio) mode. Same table, verified
 * against ip_funcs.o in the ESP-IDF v5.4 libbtdm_app.a for the S3; used by
 * ESPwn32 / esperanto for the C3/S3 scan hooks. */
#define IPF_LLD_SCAN_PROCESS_PKT_RX 258   /* void (scan_id)            - RX in scan mode    */
#define IPF_LLD_SCAN_SCHED          268   /* (scan_id, ts, resched)    - scan (re)scheduled */
/* Window boundaries (same table, ip_funcs.o of the Arduino 2.0.14 / IDF 4.4.6
 * libbtdm_app.a for the S3): the event-start callback runs after the LL has
 * programmed the window's CS (channel included), so a patch made there is
 * what the radio actually uses; the end-of-frame ISR runs when the window is
 * over, normally or aborted by the arbiter; canceled/skip cover a window that
 * never ran. Inside a window no other activity touches the radio, so a global
 * controller bit (CRC checking off) can be flipped for its duration only. */
#define IPF_LLD_SCAN_EVT_CANCELED   250   /* void (sch_arb_elt *evt) */
#define IPF_LLD_SCAN_EVT_START      251   /* void (sch_arb_elt *evt)  (_eco) */
#define IPF_LLD_SCAN_FRM_EOF_ISR    253   /* void (act_id, timestamp, abort) (_eco) */
#define IPF_LLD_SCAN_FRM_SKIP_ISR   255   /* void (act_id) (_eco) */
#define IPF_LLD_SCAN_FRM_RX_ISR     254   /* void (act_id) - a packet landed in a window */
/* Tracers only (pass-through hooks that time-stamp the trace ring). */
#define IPF_LLD_ADV_END             104
#define IPF_LLD_ADV_EVT_START       106
#define IPF_LLD_CON_EVT_START       210

/* Trace ring in RTC memory: survives the interrupt-watchdog reset, so the
 * sequence of radio events right before a hang can be read on the next boot
 * (ant_espphy_dump_trace). */
#include "esp_attr.h"
#define TRACE_N 96
typedef struct { uint32_t t; uint8_t type; uint8_t a; uint16_t b; } trace_ent_t;
RTC_NOINIT_ATTR static struct { uint32_t magic; uint32_t head; trace_ent_t e[TRACE_N]; } s_trace;
#define TRACE_MAGIC 0xA7A7C0DEu
enum { TR_WIN_START = 1, TR_ABORT, TR_EOF, TR_CANCEL, TR_SKIP, TR_SCHED, TR_ADV_START, TR_ADV_END,
       TR_CON_START, TR_RX, TR_DEINIT, TR_CFG, TR_RXEN, TR_TUNE };
static inline void IRAM_ATTR trace(uint8_t type, uint8_t a, uint16_t b)
{
#ifndef ANT_ESPPHY_TRACE
    /* Compiled out unless asked for: it writes RTC memory from the BT
     * controller's interrupt, which is a bring-up aid, not a shipping path. */
    (void)type; (void)a; (void)b; return;
#endif
    if (s_trace.magic != TRACE_MAGIC) return;
    trace_ent_t *e = &s_trace.e[s_trace.head % TRACE_N];
    e->t = (uint32_t)esp_timer_get_time(); e->type = type; e->a = a; e->b = b;
    s_trace.head++;
}

#define EM_FREQ_TABLE   0x100u   /* byte[ch] = MHz - 2402 */
#define EM_CS           0x400u   /* control structure 0: the test event's */
/* Control structures are EM_CS_STRIDE apart and one activity each (byte 2 is
 * the activity index). Which slot the scan gets depends on what else is up:
 * on a device that advertises and holds connections it was slot 2 (slot 0
 * unused, slot 1 the phone's slave connection), so coexist mode finds the
 * scan's CS by its format byte rather than assuming slot 0. */
#define EM_CS_STRIDE    0x5Au
#define EM_CS_SLOTS     12u
#define BLE_CH39_FREQ   2480u    /* what the channel-39 frequency entry means to BLE */
/* Coexist windows: the host's scan window sets their length (see
 * CS_MINEVTIME in hook_scan_evt_start). 10 ms windows gave about half of an
 * ANT+ sensor's 4 Hz page rate on the bench, 25 ms windows the full rate;
 * connections and advertising are served between windows. */
#define EM_RX_DESC      0x1000u  /* RX descriptors, 20 bytes each */

#define CS_CNTL                 0    /* event format */
#define CS_THRCNTL_RATECNTL     4
#define CS_SYNC                 12   /* 4 bytes, bit-swapped, first-on-air last */
#define CS_HOPCTRL              22
#define CS_TXDESCPTR            28
#define CS_MINEVTIME            32   /* scan: the window's programmed length (see below) */
#define CS_RXMAXBUF             40

#define CS_FORMAT_MASK          0x1F
#define CS_FORMAT_PASSIVE_SCAN  0x08
#define CS_FORMAT_ACTIVE_SCAN   0x09
#define CS_FORMAT_TEST_RX       0x1D
#define CS_FORMAT_TEST_TX       0x1E

#define RWBLECNTL               (*(volatile uint32_t *)0x60031000u)
#define RWBLECNTL_CRC_OFF       ((1u << 16) | (1u << 17))
#define RWBLECNTL_WHIT_OFF      (1u << 18)
#define RWBLECNTL_SCAN_ABORT    (1u << 26)
#define RWBLECNTL_ADVERT_ABORT  (1u << 27)
#define RWBLECNTL_RFTEST_ABORT  (1u << 28)

static inline void IRAM_ATTR coexist_abort(ant_espphy_t *d, uint8_t where);

#define LLD_ENV_RX_FIFO_IDX     0xd8
#define BLE_CH                  39u
#define DTM_PAYLOAD_LEN         0xFFu
#define TX_CLEAR_US             5000     /* ~2 DTM packets carry the frame */

typedef struct {
    uint32_t unknown_1;
    uint32_t header;          /* opcode | len << 8 | rssi << 16 */
    uint32_t unknown_2;
    uint32_t unknown_3;
    uint16_t unknown_4;
    uint16_t buffer_offset;   /* EM offset of the received bytes */
} em_rx_desc_t;

typedef struct {
    uint16_t txptr;
    uint16_t txheader;
    uint16_t txdataptr;       /* EM offset of the DTM payload */
    uint16_t txdle;
} em_tx_desc_t;

typedef uint32_t (*fn_u32_t)(uint32_t arg);
typedef uint32_t (*fn_scan_sched_t)(uint32_t scan_id, uint32_t ts, uint32_t resched);

static fn_u32_t         s_orig_evt_start;
static fn_u32_t         s_orig_rx_isr;
static fn_scan_sched_t  s_orig_scan_sched;
static fn_u32_t         s_orig_scan_rx;
static ant_espphy_t    *s_dev;         /* the single instance the hooks serve */

/* ------------------------------ bit helpers ------------------------------- */

static inline uint8_t IRAM_ATTR bit_swap(uint8_t v)
{
    v = (uint8_t)((v & 0xF0) >> 4 | (v & 0x0F) << 4);
    v = (uint8_t)((v & 0xCC) >> 2 | (v & 0x33) << 2);
    v = (uint8_t)((v & 0xAA) >> 1 | (v & 0x55) << 1);
    return v;
}

/* BLE data-whitening LFSR (x^7 + x^4 + 1) keyed by channel index, over bytes as
 * the core stores them (LSB-first). XOR is self-inverse, so this undoes the
 * hardware dewhitening the controller applies to a received PDU. Used only in
 * coexist mode, where we leave hardware whitening on (so BLE keeps working) and
 * recover the never-whitened ANT payload in software. From ESPwn32/esperanto. */
static void IRAM_ATTR ble_dewhiten(uint8_t *data, size_t len, uint8_t channel)
{
    uint8_t lfsr = (uint8_t)(bit_swap(channel) | 2);
    for (size_t i = 0; i < len; i++) {
        uint8_t c = bit_swap(data[i]);
        for (int j = 7; j >= 0; j--) {
            if (lfsr & 0x80) { lfsr ^= 0x11; c ^= (uint8_t)(1u << j); }
            lfsr <<= 1;
        }
        data[i] = bit_swap(c);
    }
}

/* ------------------------------- CS patching ------------------------------ */

static inline uint8_t *IRAM_ATTR em_ptr(uint16_t off)
{
    return (uint8_t *)r_emi_get_mem_addr_by_offset(off);
}

/* Channel index the retuned event runs on: fixed 39 for the test event,
 * configurable in coexist mode (see coexist_ch). */
static inline uint8_t IRAM_ATTR rx_ch(const ant_espphy_t *d)
{
    return (d->coexist && d->coexist_ch) ? d->coexist_ch : (uint8_t)BLE_CH;
}

static void IRAM_ATTR cs_set_frequency(ant_espphy_t *d, uint16_t mhz)
{
    em_ptr(EM_FREQ_TABLE)[rx_ch(d)] = (uint8_t)(mhz - 2402u);
}

/* The control structure this instance drives: slot 0 for the test event, the
 * scan's slot (found by coexist_find_cs) in coexist mode. */
static inline uint8_t *IRAM_ATTR cs_base(const ant_espphy_t *d)
{
    return em_ptr(d->cs_off ? d->cs_off : EM_CS);
}

/* Coexist: locate the scan activity's control structure. A slot whose format
 * is a passive/active scan is the live one; once patched it reads as our
 * test-RX format with our sync word, which is also accepted so the search is
 * stable across windows. The fields we overwrite are saved the first time. */
static void IRAM_ATTR coexist_find_cs(ant_espphy_t *d)
{
    for (unsigned n = 0; n < EM_CS_SLOTS; n++) {
        uint16_t off = (uint16_t)(EM_CS + n * EM_CS_STRIDE);
        const uint8_t *cs = em_ptr(off);
        uint8_t fmt = cs[CS_CNTL] & CS_FORMAT_MASK;
        if (fmt == CS_FORMAT_PASSIVE_SCAN || fmt == CS_FORMAT_ACTIVE_SCAN) {
            if (d->cs_off != off) {
                d->cs_off = off;
                memcpy(d->cs_saved, cs, sizeof(d->cs_saved));
                d->cs_have_saved = true;
            }
            return;
        }
    }
}

/* Coexist deinit: give the scan its control structure back (format, rate,
 * sync, hop, rx buffer) and put BLE channel 39 back on its own frequency. */
static void coexist_restore_cs(ant_espphy_t *d)
{
    if (d->cs_have_saved && d->cs_off) {
        uint8_t *cs = em_ptr(d->cs_off);
        cs[CS_CNTL]                 = d->cs_saved[CS_CNTL];
        cs[CS_THRCNTL_RATECNTL]     = d->cs_saved[CS_THRCNTL_RATECNTL];
        cs[CS_THRCNTL_RATECNTL + 1] = d->cs_saved[CS_THRCNTL_RATECNTL + 1];
        memcpy(cs + CS_SYNC, d->cs_saved + CS_SYNC, 4);
        cs[CS_HOPCTRL]              = d->cs_saved[CS_HOPCTRL];
        cs[CS_HOPCTRL + 1]          = d->cs_saved[CS_HOPCTRL + 1];
        cs[CS_RXMAXBUF]             = d->cs_saved[CS_RXMAXBUF];
    }
    /* The table is linear (entry i = 2402 + 2i MHz), so any entry restores to 2i. */
    em_ptr(EM_FREQ_TABLE)[rx_ch(d)] = (uint8_t)(2u * rx_ch(d));
    em_ptr(EM_FREQ_TABLE)[BLE_CH]   = (uint8_t)(BLE_CH39_FREQ - 2402u);
    d->cs_off = 0;
    d->cs_have_saved = false;
}

static void IRAM_ATTR cs_set_sync(ant_espphy_t *d)
{
    uint8_t *cs = cs_base(d);
    const uint8_t *sy = d->sync_override_on ? d->sync_override : d->sync;
    for (int i = 0; i < 4; i++) cs[CS_SYNC + i] = bit_swap(sy[i]);
    memcpy(d->cs_sync, d->sync, 4);
    d->cs_sync_skip = d->sync_skip;
}

/* The receiver-test event the controller programmed is already a continuous
 * 1 Mbit/s receive (format 0x1D); retarget its sync word, frequency and
 * framing. */
static void IRAM_ATTR cs_program_rx(ant_espphy_t *d)
{
    uint8_t *cs = cs_base(d);
    /* Coexist keeps the scan's own event format. Writing the receiver-test
     * format into the scan's slot resets the device by interrupt watchdog
     * within seconds when other activities (a slave connection to a phone,
     * advertising) are live - a test event never ends, and the scheduler
     * spins in its ISR trying to close the window (2026-09-02, T5S3 bike
     * computer, scan in slot 2). UNRESOLVED: with the scan format kept the
     * sync/frequency retune takes (BLE advertisements stop arriving), but
     * every window then yields one descriptor with length 0 and header byte
     * 0x01/0x0d - the advertising-channel receive path validates the byte
     * after the sync word as a PDU length, and an ANT device number / type
     * byte fails that check, so no ANT frame is stored. Needs either a CS
     * field that lifts the adv length check in scan format, or a way to end
     * a test-format window cleanly. */
    /* Both modes run the window as a receiver-test event: it is the only
     * format whose RX path hands over a frame that fails the BLE CRC / length
     * rules (an ANT frame always does - the scan formats drop it in hardware,
     * verified on the S3 2026-09-02). Such an event never ends by itself; in
     * coexist mode coexist_win_timer_cb pulses SCAN_ABORT after win_len_ms so
     * the scheduler gets the slot back for connections and advertising. */
    cs[CS_CNTL] = CS_FORMAT_TEST_RX;
    uint8_t pm = (d->coexist && d->patch_mask) ? d->patch_mask : 0xFF;
    if (pm & 1) cs_set_sync(d);
    if (pm & 2) {
        cs_set_frequency(d, d->mhz_override ? d->mhz_override : d->mhz);
        cs[CS_HOPCTRL]     = rx_ch(d);       /* no hopping, stay on this index */
        cs[CS_HOPCTRL + 1] = 0;
    }
    if (pm & 8) cs[CS_RXMAXBUF] = 0xFF;
    /* Exclusive (test) mode owns the radio, so turn whitening/CRC off in
     * hardware and just bit-swap the bytes. Coexist mode shares the radio with
     * BLE, whose connection events need whitening/CRC on, so it leaves these
     * global bits alone and undoes the whitening in software (see decode). */
    if (!d->coexist) RWBLECNTL |= RWBLECNTL_CRC_OFF | RWBLECNTL_WHIT_OFF;
}

/* Force 1 Mbit/s in the control structure. The test event already runs at 1M,
 * but a scan event we hijack for coexist mode is re-asserted each window. */
static void IRAM_ATTR cs_set_rate_1m(ant_espphy_t *d)
{
    uint8_t *cs = cs_base(d);
    cs[CS_THRCNTL_RATECNTL]     = 0;         /* rate 0 = LE 1M */
    cs[CS_THRCNTL_RATECNTL + 1] = 0x10;
}

static uint8_t *IRAM_ATTR dtm_payload(void)
{
    uint8_t *cs = em_ptr(EM_CS);
    uint16_t descptr = (uint16_t)(cs[CS_TXDESCPTR] | (cs[CS_TXDESCPTR + 1] << 8));
    if (descptr == 0) return NULL;
    em_tx_desc_t *desc = (em_tx_desc_t *)em_ptr(descptr);
    if (desc->txdataptr == 0) return NULL;
    return em_ptr(desc->txdataptr);
}

/* --------------------------------- hooks ---------------------------------- */

/* The controller has programmed the DTM event (RX or TX test) and is about to
 * start it: rewrite the control structure for ANT. */
static uint32_t IRAM_ATTR hook_test_evt_start(uint32_t evt)
{
    uint32_t r = s_orig_evt_start(evt);
    ant_espphy_t *d = s_dev;
    if (d && d->mode == ANT_ESPPHY_MODE_RX) {
        cs_program_rx(d);
        d->sched_hook_calls++;
    } else if (d && d->mode == ANT_ESPPHY_MODE_TX) {
        cs_set_frequency(d, d->mhz);
        em_ptr(EM_CS)[CS_CNTL] = CS_FORMAT_TEST_TX;
        uint8_t *p = dtm_payload();
        if (p) memset(p, 0, DTM_PAYLOAD_LEN);
        d->sched_hook_calls++;
    }
    return r;
}

/* Pull one received packet out of the RX descriptor at the current FIFO index,
 * rebuild the ANT body from the sync word + raw bytes, run the software address
 * match, and queue matches for rx_poll(). Shared by the receiver-test RX
 * interrupt (test mode) and the scan RX callback (coexist mode). Runs in the
 * controller's interrupt context: IRAM, no logging, no blocking. */
static void IRAM_ATTR decode_rx_desc(ant_espphy_t *d)
{
    {
        em_rx_desc_t *desc = (em_rx_desc_t *)em_ptr(EM_RX_DESC);
        uint8_t idx = ((uint8_t *)p_lld_env)[LLD_ENV_RX_FIFO_IDX];
        uint32_t hdr = desc[idx].header;
        if (hdr != 0) {
            d->rx_last_hdr = hdr;
            d->rx_last_stat = desc[idx].unknown_1;
            uint32_t tick = ant_espphy_ticks();
            int8_t  rssi = (int8_t)((hdr >> 16) & 0xff);
            uint8_t size = (uint8_t)((hdr >> 8) & 0xff);
            const uint8_t *pdu = em_ptr(desc[idx].buffer_offset);
            d->rx_hook_calls++;
            d->last_rssi = rssi;

            /* The core stores [opcode][length][length bytes]; opcode+length
             * are the first two air bytes after the sync word. */
            uint8_t raw[2 + ANT_SB_FRAME_MAX];
            size_t body_len = (size_t)d->rx_cfg.addr_len + d->rx_cfg.payload_len + 2u;
            size_t lead = 4u - d->cs_sync_skip;         /* body bytes taken from the sync */
            size_t need = body_len > lead ? body_len - lead : 0;
            size_t have = 2u + size;
            if (d->tap_on && have < need && have >= 2 &&
                d->tap_head - d->tap_tail < ANT_ESPPHY_TAP_RING) {
                /* Bring-up: a packet the core cut short. Show what it stored,
                 * decoded the same way, so the length/whitening question can
                 * be settled from bytes rather than counters. */
                size_t n = have > ANT_SB_FRAME_MAX - 2 ? ANT_SB_FRAME_MAX - 2 : have;
                raw[0] = (uint8_t)(hdr & 0xff);
                raw[1] = size;
                if (n > 2) memcpy(raw + 2, pdu, n - 2);
                if (d->coexist) ble_dewhiten(raw, n, rx_ch(d));
                for (size_t i = 0; i < n; i++) raw[i] = bit_swap(raw[i]);
                ant_espphy_rx_t *t = &d->tap[d->tap_head % ANT_ESPPHY_TAP_RING];
                memcpy(t->body, d->cs_sync + d->cs_sync_skip, lead);
                memcpy(t->body + lead, raw, n);
                t->len = (uint8_t)(lead + n);
                t->rssi = rssi;
                t->tick = tick;
                t->matched = false;
                d->tap_head++;
            }
            if (body_len <= ANT_SB_FRAME_MAX && have >= need && need >= 2) {
                raw[0] = (uint8_t)(hdr & 0xff);
                raw[1] = size;
                memcpy(raw + 2, pdu, need - 2);
                /* Coexist mode leaves hardware whitening on (for BLE), so undo
                 * the dewhitening the core applied to this PDU. Test mode ran
                 * with whitening off, so this is skipped there. The whitener
                 * starts at the first PDU byte, which is raw[0]. */
                if (d->coexist) ble_dewhiten(raw, need, rx_ch(d));
                /* The core stores LSB-first bytes; put them MSB-first. */
                for (size_t i = 0; i < need; i++) raw[i] = bit_swap(raw[i]);

                uint8_t body[ANT_SB_FRAME_MAX];
                memcpy(body, d->cs_sync + d->cs_sync_skip, lead);
                memcpy(body + lead, raw, need);
                d->rx_frames++;
                memcpy(d->last_frame, body, body_len);
                d->last_len = (uint8_t)body_len;

                bool ok = true;
                for (size_t i = 0; i < d->rx_cfg.addr_len; i++) {
                    if ((body[i] ^ d->rx_cfg.address[i]) & d->rx_cfg.mask[i]) { ok = false; break; }
                }
                if (d->tap_on && d->tap_head - d->tap_tail < ANT_ESPPHY_TAP_RING) {
                    ant_espphy_rx_t *t = &d->tap[d->tap_head % ANT_ESPPHY_TAP_RING];
                    memcpy(t->body, body, body_len);
                    t->len  = (uint8_t)body_len;
                    t->rssi = rssi;
                    t->tick = tick;
                    t->matched = ok;
                    d->tap_head++;
                }
                /* Only while the MAC is listening. In coexist mode the windows
                 * are open all the time, so a frame can land while the MAC has
                 * its receiver off (between a tracked channel's slots, or after
                 * a channel closed). Queuing it then leaves the ring non-empty
                 * with nobody draining it, and the node task - which wakes on a
                 * non-empty ring - spins at top priority until the next slot,
                 * or forever after a close: a task watchdog reset on the bike
                 * computer (2026-09-02). A receiver that is not listening does
                 * not receive; drop it. */
                if (ok && !d->rx_on) { d->rx_off_dropped++; ok = false; }
                if (ok) {
                    d->rx_matched++;
                    uint32_t head = d->ring_head;
                    if (head - d->ring_tail < ANT_ESPPHY_RX_RING) {
                        ant_espphy_rx_t *slot = &d->ring[head % ANT_ESPPHY_RX_RING];
                        memcpy(slot->body, body, body_len);
                        slot->len  = (uint8_t)body_len;
                        slot->rssi = rssi;
                        slot->tick = tick;
                        d->ring_head = head + 1;
                        if (d->waiter) vTaskNotifyGiveFromISR((TaskHandle_t)d->waiter, NULL);
                    } else {
                        d->rx_dropped++;
                    }
                }
            }
            /* Consumed: make the controller see an empty packet. In coexist
             * mode this also stops the BLE scan from parsing the ANT frame as
             * an advertising report. */
            if (!d->keep_rx_hdr) desc[idx].header = 0;
        } else {
            d->rx_hook_empty++;
        }
    }
}

/* Receiver-test RX interrupt (test mode): decode, then let the controller count
 * and free the descriptor as usual. */
static uint32_t IRAM_ATTR hook_test_rx_isr(uint32_t timestamp)
{
    ant_espphy_t *d = s_dev;
    if (d && !d->coexist && d->mode == ANT_ESPPHY_MODE_RX) decode_rx_desc(d);
    return s_orig_rx_isr(timestamp);
}

/* -------------------------- coexist (scan) hooks -------------------------- */

/* The scan activity has been (re)scheduled: retarget its control structure to
 * ANT (2457 MHz / 1 Mbit/s / ANT sync word). The CS persists between windows,
 * so this need not fire every window; whitening/CRC stay on in hardware and the
 * PDU is de-whitened in software (see decode_rx_desc), which is what lets BLE
 * connection events keep running normally between the scan windows. */
static uint32_t IRAM_ATTR hook_scan_sched(uint32_t scan_id, uint32_t ts, uint32_t resched)
{
    coexist_abort(s_dev, 1);
    trace(TR_SCHED, (uint8_t)resched, (uint16_t)ts);
    uint32_t r = s_orig_scan_sched(scan_id, ts, resched);
    ant_espphy_t *d = s_dev;
    if (d && d->coexist && d->mode == ANT_ESPPHY_MODE_RX) {
        coexist_find_cs(d);         /* the scan's slot, not slot 0 */
        if (!d->cs_off) return r;
        cs_program_rx(d);           /* format / sync / freq / hop / rxmaxbuf */
        cs_set_rate_1m(d);
        {
            uint8_t *cs = cs_base(d);
            if (d->cs_fmt_override) cs[CS_CNTL] = d->cs_fmt_override;
            for (unsigned i = 0; i < d->cs_ovr_n && i < 4; i++) cs[d->cs_ovr[i].off] = d->cs_ovr[i].val;
        }
        d->sched_hook_calls++;
    }
    return r;
}

/* Scan RX callback: a packet matched the (ANT) sync word during the window. */
static uint32_t IRAM_ATTR hook_scan_rx(uint32_t scan_id)
{
    ant_espphy_t *d = s_dev;
    if (d && d->coexist && d->mode == ANT_ESPPHY_MODE_RX) decode_rx_desc(d);
    return s_orig_scan_rx(scan_id);
}

/* Window start: the LL has just programmed the scan's CS for this window
 * (its own channel, the BLE access address). Retarget it to ANT and switch
 * CRC checking off for the window - an ANT frame has no BLE CRC, and in a
 * scan-class event the core drops CRC failures before the LL (and our RX
 * hook) ever sees them. The test-event format would deliver them but a
 * window in that format never ends (interrupt-watchdog reset). */
static fn_u32_t s_orig_scan_evt_start, s_orig_scan_evt_canceled, s_orig_scan_frm_skip;
static uint32_t (*s_orig_scan_frm_eof)(uint32_t, uint32_t, uint32_t);

static inline void IRAM_ATTR coexist_abort(ant_espphy_t *d, uint8_t where)
{
    if (d && d->coexist && (d->abort_mode & where)) {
        RWBLECNTL |= (d->abort_mode & 0x20) ? RWBLECNTL_RFTEST_ABORT : RWBLECNTL_SCAN_ABORT;
        d->aborts++;
    }
}

/* Window length from the scan CS's MINEVTIME (312.5 us units; a 16-slot
 * window reads 25 = 7.8 ms) less a margin for the timer's task-dispatch
 * jitter. The scheduler REWRITES this for a window already running when a
 * higher-priority activity (advertising) turns up, so it is re-read on every
 * timer tick, not just at window start: a deadline taken at start went stale
 * and the late abort collided with the advertiser (interrupt watchdog). */
/* The abort pulse goes out on the first timer tick at least this long after
 * the window-start callback, i.e. once the core is certainly running the
 * event (see hook_scan_evt_start). With a 500 us tick that is 0.3-0.8 ms in,
 * ahead of the earliest next activity the scheduler books (MINEVTIME 3 =
 * 0.9 ms was the smallest seen). A pending abort does not shorten a quiet
 * window: it ends at the core's ~27 ms limit, the first received packet, or
 * the next activity. */
#define ANT_ESPPHY_WIN_US 1000u
static uint32_t coexist_win_len_us(const ant_espphy_t *d, uint16_t minevt)
{
    (void)minevt;
    /* MINEVTIME-derived deadlines (both 312.5 us and re-read per tick) still
     * collided with a freshly started advertiser, and so did a fixed 3 ms
     * window after ~20-60 s of advertising at the default interval: the
     * collision is probabilistic and scales with how long a window stays
     * open. 1 ms windows ran a full minute under default advertising and
     * still delivered ~3.4 pages/s from a 4 Hz sensor (an ANT frame is only
     * ~150 us on the air), so that is the default; win_len_ms overrides. */
    if (d->win_len_ms) return (uint32_t)d->win_len_ms * 1000u;
    return ANT_ESPPHY_WIN_US;
}

/* Periodic (esp_timer task) watchdog for a window in the test format: such an
 * event never ends on its own, so once it has run win_len_ms the radio is
 * told to abort it; the LL then sees a normal end of event, the scheduler
 * gets the slot back for connections / advertising, and the next window is
 * retargeted afresh by hook_scan_evt_start. */
static void coexist_win_timer_cb(void *arg)
{
    ant_espphy_t *d = (ant_espphy_t *)arg;
    if (!d->coexist || !d->win_active || d->mode != ANT_ESPPHY_MODE_RX) return;
    if (d->win_abort_sent) return;           /* one pulse per window */
    if (d->cs_off) {
        const uint8_t *cs = em_ptr(d->cs_off);
        uint16_t me = (uint16_t)(cs[CS_MINEVTIME] | (cs[CS_MINEVTIME + 1] << 8));
        if (me != d->win_minevt) { d->win_minevt = me; d->win_len_us = coexist_win_len_us(d, me); }
    }
    uint32_t age = (uint32_t)esp_timer_get_time() - d->win_start_us;
    if (age < d->win_len_us) return;
    d->win_abort_sent = true;
    trace(TR_ABORT, 0, (uint16_t)age);
    /* SCAN_ABORT, not RFTEST_ABORT: the latter ends the event too, but the
     * LL's end-of-event handling for the scan slot then spins in its ISR
     * (interrupt watchdog). The scan abort is the path the LL expects. */
    RWBLECNTL |= (d->abort_mode & 0x20) ? RWBLECNTL_RFTEST_ABORT : RWBLECNTL_SCAN_ABORT;
    d->aborts++;
}

static inline void IRAM_ATTR coexist_window_end(ant_espphy_t *d)
{
    if (d && d->win_active) {
        /* Ended by the LL, not by our abort: how long did it let it run? */
        uint32_t age = (uint32_t)esp_timer_get_time() - d->win_start_us;
        d->win_ll_ended++;
        if (!d->win_ll_min_us || age < d->win_ll_min_us) d->win_ll_min_us = age;
    }
    if (d) d->win_active = false;
    if (d && d->coexist && d->win_crc_off) {
        RWBLECNTL &= ~RWBLECNTL_CRC_OFF;
        d->win_crc_off = false;
    }
}

static uint32_t IRAM_ATTR hook_scan_evt_start(uint32_t evt)
{
    uint32_t r = s_orig_scan_evt_start(evt);
    ant_espphy_t *d = s_dev;
    if (d) d->scan_elt = (void *)evt;
    if (d && d->coexist && d->mode == ANT_ESPPHY_MODE_RX) {
        coexist_find_cs(d);
        if (d->cs_off) {
            cs_program_rx(d);
            if (!d->patch_mask || (d->patch_mask & 4)) cs_set_rate_1m(d);
            uint8_t *cs = cs_base(d);
            if (d->cs_fmt_override) cs[CS_CNTL] = d->cs_fmt_override;
            for (unsigned i = 0; i < d->cs_ovr_n && i < 4; i++) cs[d->cs_ovr[i].off] = d->cs_ovr[i].val;
            if (!d->win_keep_crc) {
                RWBLECNTL |= RWBLECNTL_CRC_OFF;
                d->win_crc_off = true;
            }
            /* How long this window is allowed to run. A receiver-test event
             * never ends on its own, and the scheduler places the next
             * activity (advertising, a connection event) right after the
             * window's programmed end - abort later than that and the LL
             * spins in its ISR (interrupt watchdog). So the abort goes a
             * little BEFORE the window's own length, read from MINEVTIME
             * (units of 312.5 us on this core: a 16-slot / 10 ms window reads
             * 25). The 8-ms-good / 20-ms-bad finding of 2026-09-02 fits. */
            d->win_minevt = (uint16_t)(cs[CS_MINEVTIME] | (cs[CS_MINEVTIME + 1] << 8));
            d->win_len_us = coexist_win_len_us(d, d->win_minevt);
            d->win_start_us = (uint32_t)esp_timer_get_time();
            d->win_active = true;
            trace(TR_WIN_START, 0, d->win_minevt);
            /* Arm the abort NOW rather than from the timer. The trace of
             * 2026-09-02 showed the window survives an abort request until
             * the core's own end (~27 ms), a received packet, or the next
             * activity - and it is only when the next activity (advertising)
             * arrives with NO abort pending that the LL stalls. An abort
             * that is already pending turns that arrival into a normal end
             * of event 0.7 ms later. Packets still arrive after the pulse. */
            /* ...but only when the scheduler has squeezed this window (a
             * small MINEVTIME means another activity is due within ~2 ms,
             * sooner than the 1 ms timer can be trusted to fire). A pulse
             * issued here for every window - before the hardware has started
             * the event - coincided with interrupt-watchdog resets on a
             * device holding a live connection, which the devkit does not
             * have: the pulse can land while the previous connection event
             * is still on the air. Quiet windows get theirs from the timer. */
            /* NOT here. A pulse issued from this callback - before the core
             * has started the event - is discarded at event start, so the
             * window then runs with no abort pending; the trace of
             * 2026-09-02 23:01 shows such a window (MINEVTIME 7) receiving a
             * packet 18 ms in with no end of frame after it, then the stall.
             * The timer issues the pulse once the event is running. */
            /* The configuration that ran multi-minute soaks next to a live
             * phone connection (commit 77a57f2 + the ring fix): request the
             * abort here AND again from the timer ~1 ms in. Every variation
             * tried since (hook only, timer only, earlier timer) coincided
             * with resets within seconds of a strap reconnect. */
            d->win_abort_sent = false;
            if (!(d->abort_mode & 0x40)) {
                RWBLECNTL |= RWBLECNTL_SCAN_ABORT;
                d->aborts++;
                trace(TR_ABORT, 1, 0);
            }
            d->win_hook_calls++;
        }
    }
    return r;
}

static fn_u32_t s_orig_adv_end, s_orig_adv_start, s_orig_con_start;
static uint32_t IRAM_ATTR hook_adv_start(uint32_t evt) { trace(TR_ADV_START, 0, 0); return s_orig_adv_start(evt); }
static uint32_t IRAM_ATTR hook_adv_end(uint32_t a, uint32_t b, uint32_t c) { trace(TR_ADV_END, (uint8_t)b, (uint16_t)c); return ((uint32_t (*)(uint32_t,uint32_t,uint32_t))s_orig_adv_end)(a, b, c); }
static uint32_t IRAM_ATTR hook_con_start(uint32_t evt) { trace(TR_CON_START, 0, 0); return s_orig_con_start(evt); }

static fn_u32_t s_orig_scan_frm_rx;
static uint32_t IRAM_ATTR hook_scan_frm_rx(uint32_t act_id)
{
    ant_espphy_t *d = s_dev;
    if (d && d->coexist) {
        trace(TR_RX, 0, 0);
        d->scan_rx_isr_calls++;
        em_rx_desc_t *desc = (em_rx_desc_t *)em_ptr(EM_RX_DESC);
        uint8_t idx = ((uint8_t *)p_lld_env)[LLD_ENV_RX_FIFO_IDX];
        d->scan_rx_isr_hdr  = desc[idx].header;
        d->scan_rx_isr_stat = desc[idx].unknown_1;
    }
    return s_orig_scan_frm_rx(act_id);
}

static uint32_t IRAM_ATTR hook_scan_frm_eof(uint32_t act_id, uint32_t ts, uint32_t abort)
{
    trace(TR_EOF, (uint8_t)abort, (uint16_t)ts);
    coexist_abort(s_dev, 2);
    coexist_window_end(s_dev);
    return s_orig_scan_frm_eof(act_id, ts, abort);
}

static uint32_t IRAM_ATTR hook_scan_evt_canceled(uint32_t evt)
{
    trace(TR_CANCEL, 0, 0);
    coexist_abort(s_dev, 4);
    coexist_window_end(s_dev);
    return s_orig_scan_evt_canceled(evt);
}

static uint32_t IRAM_ATTR hook_scan_frm_skip(uint32_t act_id)
{
    trace(TR_SKIP, 0, 0);
    coexist_window_end(s_dev);
    return s_orig_scan_frm_skip(act_id);
}

size_t ant_espphy_trace_count(void)
{
    if (s_trace.magic != TRACE_MAGIC) return 0;
    return s_trace.head < TRACE_N ? s_trace.head : TRACE_N;
}

bool ant_espphy_trace_get(size_t i, uint32_t *t, const char **type, uint8_t *a, uint16_t *b)
{
    static const char *names[] = { "?", "WIN", "ABORT", "EOF", "CANCEL", "SKIP", "SCHED", "ADV", "ADVEND", "CON", "RX", "DEINIT", "CFG", "RXEN", "TUNE" };
    size_t n = ant_espphy_trace_count();
    if (i >= n) return false;
    const trace_ent_t *e = &s_trace.e[(s_trace.head - n + i) % TRACE_N];
    *t = e->t; *type = e->type < 15 ? names[e->type] : "?"; *a = e->a; *b = e->b;
    return true;
}

void ant_espphy_trace_reset(void)
{
    s_trace.magic = TRACE_MAGIC;
    s_trace.head = 0;
}

void ant_espphy_dump_trace(void)
{
    static const char *names[] = { "?", "WIN", "ABORT", "EOF", "CANCEL", "SKIP", "SCHED", "ADV", "ADVEND", "CON", "RX", "DEINIT", "CFG", "RXEN", "TUNE" };
    if (s_trace.magic != TRACE_MAGIC) { printf("[trace] empty\n"); s_trace.magic = TRACE_MAGIC; s_trace.head = 0; return; }
    uint32_t n = s_trace.head < TRACE_N ? s_trace.head : TRACE_N;
    uint32_t start = s_trace.head - n;
    printf("[trace] last %lu events before reset (us, delta, type, a, b):\n", (unsigned long)n);
    uint32_t prev = 0;
    for (uint32_t i = 0; i < n; i++) {
        const trace_ent_t *e = &s_trace.e[(start + i) % TRACE_N];
        printf("  %10lu %+7ld %-6s %3u %5u\n", (unsigned long)e->t, (long)(prev ? e->t - prev : 0),
               e->type < 15 ? names[e->type] : "?", e->a, e->b);
        prev = e->t;
    }
    s_trace.head = 0;
}

void ant_espphy_debug_cs(ant_espphy_t *d, uint8_t *out, size_t n)
{
    (void)d;
    memcpy(out, em_ptr(EM_CS), n);
    out[n - 1] = em_ptr(EM_FREQ_TABLE)[BLE_CH];
}

static void install_hooks(void)
{
    s_orig_evt_start = (fn_u32_t)r_ip_funcs_p[IPF_LLD_TEST_EVT_START_CBK];
    s_orig_rx_isr    = (fn_u32_t)r_ip_funcs_p[IPF_LLD_TEST_RX_ISR];
    r_ip_funcs_p[IPF_LLD_TEST_EVT_START_CBK] = (void *)hook_test_evt_start;
    r_ip_funcs_p[IPF_LLD_TEST_RX_ISR]        = (void *)hook_test_rx_isr;
}

static void install_scan_hooks(void)
{
    s_orig_scan_sched = (fn_scan_sched_t)r_ip_funcs_p[IPF_LLD_SCAN_SCHED];
    s_orig_scan_rx    = (fn_u32_t)r_ip_funcs_p[IPF_LLD_SCAN_PROCESS_PKT_RX];
    s_orig_scan_evt_start    = (fn_u32_t)r_ip_funcs_p[IPF_LLD_SCAN_EVT_START];
    s_orig_scan_evt_canceled = (fn_u32_t)r_ip_funcs_p[IPF_LLD_SCAN_EVT_CANCELED];
    s_orig_scan_frm_skip     = (fn_u32_t)r_ip_funcs_p[IPF_LLD_SCAN_FRM_SKIP_ISR];
    s_orig_scan_frm_eof      = (uint32_t (*)(uint32_t, uint32_t, uint32_t))r_ip_funcs_p[IPF_LLD_SCAN_FRM_EOF_ISR];
    s_orig_scan_frm_rx       = (fn_u32_t)r_ip_funcs_p[IPF_LLD_SCAN_FRM_RX_ISR];
    r_ip_funcs_p[IPF_LLD_SCAN_FRM_RX_ISR]     = (void *)hook_scan_frm_rx;
    r_ip_funcs_p[IPF_LLD_SCAN_SCHED]          = (void *)hook_scan_sched;
    r_ip_funcs_p[IPF_LLD_SCAN_PROCESS_PKT_RX] = (void *)hook_scan_rx;
    r_ip_funcs_p[IPF_LLD_SCAN_EVT_START]      = (void *)hook_scan_evt_start;
    r_ip_funcs_p[IPF_LLD_SCAN_EVT_CANCELED]   = (void *)hook_scan_evt_canceled;
    r_ip_funcs_p[IPF_LLD_SCAN_FRM_SKIP_ISR]   = (void *)hook_scan_frm_skip;
    r_ip_funcs_p[IPF_LLD_SCAN_FRM_EOF_ISR]    = (void *)hook_scan_frm_eof;
    /* Tracer hooks on the advertiser / connection paths: bring-up only,
     * and only when asked for (ANT_ESPPHY_TRACE_OTHERS) - patching the
     * table entries other activities call through is one more thing that
     * can go wrong on a device holding a live connection. */
#ifdef ANT_ESPPHY_TRACE_OTHERS
    s_orig_adv_start = (fn_u32_t)r_ip_funcs_p[IPF_LLD_ADV_EVT_START];
    s_orig_adv_end   = (fn_u32_t)r_ip_funcs_p[IPF_LLD_ADV_END];
    s_orig_con_start = (fn_u32_t)r_ip_funcs_p[IPF_LLD_CON_EVT_START];
    r_ip_funcs_p[IPF_LLD_ADV_EVT_START] = (void *)hook_adv_start;
    r_ip_funcs_p[IPF_LLD_ADV_END]       = (void *)hook_adv_end;
    r_ip_funcs_p[IPF_LLD_CON_EVT_START] = (void *)hook_con_start;
#endif
    if (s_trace.magic != TRACE_MAGIC) { s_trace.magic = TRACE_MAGIC; s_trace.head = 0; }
}

/* ---------------------------------- HCI ----------------------------------- */

#define HCI_RESET               0x0C03
#define HCI_SET_EVT_MASK        0x0C01
#define HCI_LE_RX_TEST_V2       0x2033
#define HCI_LE_TX_TEST_V2       0x2034
#define HCI_LE_TEST_END         0x201F
#define HCI_EVT_CMD_COMPLETE    0x0E
#define HCI_TIMEOUT_MS          500

static SemaphoreHandle_t s_hci_done;
static volatile uint8_t  s_hci_status;
static volatile uint16_t s_hci_opcode;

static void hci_on_ready(void) {}

static int hci_on_event(uint8_t *data, uint16_t len)
{
    /* H4: [0x04][evt][plen][num_cmds][opcode lo][opcode hi][status ...] */
    if (len >= 7 && data[1] == HCI_EVT_CMD_COMPLETE) {
        s_hci_opcode = (uint16_t)(data[4] | (data[5] << 8));
        s_hci_status = data[6];
        xSemaphoreGive(s_hci_done);
    }
    return 0;
}

static bool hci_cmd(ant_espphy_t *d, uint16_t opcode, const uint8_t *params, uint8_t plen)
{
    uint8_t pkt[4 + 16];
    pkt[0] = 0x01;                                  /* H4 command */
    pkt[1] = (uint8_t)opcode;
    pkt[2] = (uint8_t)(opcode >> 8);
    pkt[3] = plen;
    if (plen) memcpy(pkt + 4, params, plen);

    for (int i = 0; i < 100 && !esp_vhci_host_check_send_available(); i++) vTaskDelay(1);
    xSemaphoreTake(s_hci_done, 0);                  /* drain a stale completion */
    esp_vhci_host_send_packet(pkt, (uint16_t)(4 + plen));
    if (xSemaphoreTake(s_hci_done, pdMS_TO_TICKS(HCI_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "HCI 0x%04x: no completion", opcode);
        d->hci_errors++;
        return false;
    }
    if (s_hci_opcode != opcode || s_hci_status != 0) {
        ESP_LOGE(TAG, "HCI 0x%04x -> opcode 0x%04x status 0x%02x", opcode, s_hci_opcode, s_hci_status);
        d->hci_errors++;
        return false;
    }
    return true;
}

static bool hci_rx_test(ant_espphy_t *d)
{
    /* channel 39 (re-targeted by the hook), LE 1M, standard modulation index */
    const uint8_t p[3] = { BLE_CH, 0x01, 0x00 };
    return hci_cmd(d, HCI_LE_RX_TEST_V2, p, sizeof(p));
}

static bool hci_tx_test(ant_espphy_t *d)
{
    /* channel 39 (re-targeted by the hook), 255-byte payload of 0x00, LE 1M */
    const uint8_t p[4] = { BLE_CH, DTM_PAYLOAD_LEN, 0x06, 0x01 };
    return hci_cmd(d, HCI_LE_TX_TEST_V2, p, sizeof(p));
}

static bool hci_test_end(ant_espphy_t *d)
{
    return hci_cmd(d, HCI_LE_TEST_END, NULL, 0);
}

/* ------------------------------ mode control ------------------------------ */

static bool enter_mode(ant_espphy_t *d, ant_espphy_mode_t mode)
{
    if (d->mode == mode) return true;
    if (d->mode != ANT_ESPPHY_MODE_IDLE) hci_test_end(d);
    d->mode = ANT_ESPPHY_MODE_IDLE;

    bool ok = true;
    if (mode == ANT_ESPPHY_MODE_RX) {
        d->ring_tail = d->ring_head;
        d->mode = ANT_ESPPHY_MODE_RX;       /* the event-start hook fires inside the command */
        ok = hci_rx_test(d);
    } else if (mode == ANT_ESPPHY_MODE_TX) {
        d->mode = ANT_ESPPHY_MODE_TX;
        ok = hci_tx_test(d);
    }
    if (!ok) d->mode = ANT_ESPPHY_MODE_IDLE;
    ESP_LOGI(TAG, "mode -> %s%s", ant_espphy_mode_str(d->mode), ok ? "" : " (HCI failed)");
    return ok;
}

/* ------------------------------ ant_phy_t --------------------------------- */

static esp_timer_handle_t s_tx_clear;

static void tx_clear_cb(void *arg)
{
    ant_espphy_t *d = (ant_espphy_t *)arg;
    if (d->mode != ANT_ESPPHY_MODE_TX) return;
    uint8_t *p = dtm_payload();
    if (p) memset(p, 0, DTM_PAYLOAD_LEN);
}

static bool espphy_tune(ant_phy_t *p, uint16_t mhz)
{
    ant_espphy_t *d = (ant_espphy_t *)p;
    if (mhz < 2402 || mhz > 2480) return false;
    d->mhz = mhz;                       /* picked up at the next CS rewrite */
    d->tunes++;
    return true;
}

static bool espphy_tx(ant_phy_t *p, const uint8_t *frame, size_t len)
{
    ant_espphy_t *d = (ant_espphy_t *)p;
    if (!frame || len == 0 || len > ANT_SB_FRAME_MAX) return false;
    d->tx_count++;
    memcpy(d->last_frame, frame, len);
    d->last_len = (uint8_t)len;

    if (d->coexist)                    { d->tx_refused++; return false; } /* RX-only mode */
    if (d->mode == ANT_ESPPHY_MODE_RX) { d->tx_refused++; return false; }
    if (d->mode != ANT_ESPPHY_MODE_TX && !enter_mode(d, ANT_ESPPHY_MODE_TX)) return false;

    uint8_t *buf = dtm_payload();
    if (!buf) return false;

    /* extra preamble so a real ANT receiver has bits to settle on, then the
     * frame; the core shifts LSB first so every byte is bit-reversed */
    uint8_t out[2 + ANT_SB_FRAME_MAX];
    size_t n = 0;
    out[n++] = frame[0];
    out[n++] = frame[0];
    for (size_t i = 0; i < len; i++) out[n++] = frame[i];
    for (size_t i = 0; i < n; i++) out[i] = bit_swap(out[i]);

    esp_timer_stop(s_tx_clear);
    memset(buf, 0, DTM_PAYLOAD_LEN);
    memcpy(buf, out, n);
    esp_timer_start_once(s_tx_clear, TX_CLEAR_US);
    d->tx_emitted++;
    return true;
}

static void espphy_rx_config(ant_phy_t *p, const ant_phy_rx_cfg_t *cfg)
{
    ant_espphy_t *d = (ant_espphy_t *)p;
    d->rx_cfg = *cfg;
    const uint8_t *a = cfg->address, *m = cfg->mask;
    uint8_t sync[4];
    uint8_t skip;
    if (cfg->addr_len >= 4 && m[0] == 0xFF && m[1] == 0xFF && m[2] == 0xFF && m[3] == 0xFF) {
        memcpy(sync, a, 4);                        /* marker + device number */
        skip = 0;
    } else {
        uint8_t pre = ant_sb_preamble_for(a[0]);   /* wildcard: preamble + marker */
        sync[0] = pre; sync[1] = pre; sync[2] = a[0]; sync[3] = a[1];
        skip = 2;
    }
    if (memcmp(d->sync, sync, 4) == 0 && d->sync_skip == skip) return;
    memcpy(d->sync, sync, 4);
    d->sync_skip = skip;
    trace(TR_CFG, skip, (uint16_t)((sync[2] << 8) | sync[3]));
    /* Coexist mode reprograms the CS at every scan window, so a new sync word
     * takes effect on its own; nothing to restart. */
    if (d->coexist) return;
    if (d->mode != ANT_ESPPHY_MODE_RX) return;

    /* The running event keeps its sync word. That is fine as long as it can
     * still hear the wanted address (a wildcard sync hears the whole network;
     * the software match above does the rest); otherwise restart the event. */
    bool hears = (d->cs_sync_skip == 2)
               ? (d->cs_sync[2] == a[0] && d->cs_sync[3] == a[1])
               : (skip == 0 && memcmp(d->cs_sync, sync, 4) == 0);
    if (!hears) {
        enter_mode(d, ANT_ESPPHY_MODE_IDLE);
        enter_mode(d, ANT_ESPPHY_MODE_RX);
        d->sync_rewrites++;
    }
}

bool ant_espphy_tap_poll(ant_espphy_t *dev, ant_espphy_rx_t *out, bool *matched)
{
    uint32_t tail = dev->tap_tail;
    if (tail == dev->tap_head) return false;
    *out = dev->tap[tail % ANT_ESPPHY_TAP_RING];
    if (matched) *matched = out->matched;
    dev->tap_tail = tail + 1;
    return true;
}

bool ant_espphy_wait_rx(ant_espphy_t *dev, uint32_t timeout_ms)
{
    dev->waiter = xTaskGetCurrentTaskHandle();
    /* Never spin on frames the MAC is not going to read (see decode_rx_desc). */
    if (!dev->rx_on) dev->ring_tail = dev->ring_head;
    if (dev->ring_head != dev->ring_tail) return true;
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms));
    return dev->ring_head != dev->ring_tail;
}

static void espphy_rx_enable(ant_phy_t *p, bool on)
{
    ant_espphy_t *d = (ant_espphy_t *)p;
    if (d->rx_on != on) trace(TR_RXEN, on, 0);
    d->rx_on = on;
    if (d->coexist) {
        /* No HCI: the scan windows the BLE host runs are the radio. Once armed
         * we keep grabbing them (the MAC toggles rx off between slots; the scan
         * windows are shared and cheap, so stay listening like the test path). */
        if (on) d->mode = ANT_ESPPHY_MODE_RX;
        return;
    }
    if (!on) return;                                /* stay in RX; cheap and keeps timing */
    if (d->mode == ANT_ESPPHY_MODE_TX) { d->rx_enables_ignored++; return; }
    if (d->mode != ANT_ESPPHY_MODE_RX) enter_mode(d, ANT_ESPPHY_MODE_RX);
}

static size_t espphy_rx_poll(ant_phy_t *p, uint8_t *body, size_t cap, uint32_t *rx_time,
                             int8_t *rssi)
{
    ant_espphy_t *d = (ant_espphy_t *)p;
    d->rx_polls++;
    uint32_t tail = d->ring_tail;
    if (tail == d->ring_head) return 0;
    ant_espphy_rx_t *slot = &d->ring[tail % ANT_ESPPHY_RX_RING];
    size_t n = slot->len;
    if (n > cap) n = cap;
    memcpy(body, slot->body, n);
    if (rx_time) *rx_time = slot->tick;
    if (rssi)    *rssi    = slot->rssi;
    d->ring_tail = tail + 1;
    return n;
}

/* --------------------------------- public --------------------------------- */

static void espphy_setup_phy(ant_espphy_t *dev);

/* A plausible S3/C3 code pointer: masked ROM, IRAM, or flash-.text (icache).
 * The controller installs IRAM "hack"/"eco" patches over some link-layer
 * entries at runtime (coexistence, ROM bug fixes), so a table entry being in
 * IRAM rather than ROM is expected, not a red flag. */
static bool is_code_ptr(uintptr_t p)
{
    return (p >= 0x40000000u && p < 0x40060000u) ||   /* masked ROM              */
           (p >= 0x40370000u && p < 0x403E0000u) ||   /* IRAM (runtime patches)  */
           (p >= 0x42000000u && p < 0x43000000u);     /* flash .text via icache  */
}

static void restore_hooks(void)
{
    if (s_orig_evt_start) r_ip_funcs_p[IPF_LLD_TEST_EVT_START_CBK] = (void *)s_orig_evt_start;
    if (s_orig_rx_isr)    r_ip_funcs_p[IPF_LLD_TEST_RX_ISR]        = (void *)s_orig_rx_isr;
    s_orig_evt_start = NULL;
    s_orig_rx_isr    = NULL;
}

static void restore_scan_hooks(void)
{
    if (s_orig_scan_sched) r_ip_funcs_p[IPF_LLD_SCAN_SCHED]          = (void *)s_orig_scan_sched;
    if (s_orig_scan_rx)    r_ip_funcs_p[IPF_LLD_SCAN_PROCESS_PKT_RX] = (void *)s_orig_scan_rx;
    s_orig_scan_sched = NULL;
    s_orig_scan_rx    = NULL;
    if (s_orig_scan_evt_start)    r_ip_funcs_p[IPF_LLD_SCAN_EVT_START]    = (void *)s_orig_scan_evt_start;
    if (s_orig_scan_evt_canceled) r_ip_funcs_p[IPF_LLD_SCAN_EVT_CANCELED] = (void *)s_orig_scan_evt_canceled;
    if (s_orig_scan_frm_skip)     r_ip_funcs_p[IPF_LLD_SCAN_FRM_SKIP_ISR] = (void *)s_orig_scan_frm_skip;
    if (s_orig_scan_frm_eof)      r_ip_funcs_p[IPF_LLD_SCAN_FRM_EOF_ISR]  = (void *)s_orig_scan_frm_eof;
    if (s_orig_scan_frm_rx)       r_ip_funcs_p[IPF_LLD_SCAN_FRM_RX_ISR]   = (void *)s_orig_scan_frm_rx;
    if (s_orig_adv_start) r_ip_funcs_p[IPF_LLD_ADV_EVT_START] = (void *)s_orig_adv_start;
    if (s_orig_adv_end)   r_ip_funcs_p[IPF_LLD_ADV_END]       = (void *)s_orig_adv_end;
    if (s_orig_con_start) r_ip_funcs_p[IPF_LLD_CON_EVT_START] = (void *)s_orig_con_start;
    trace(TR_DEINIT, 0, 0);
    RWBLECNTL &= ~RWBLECNTL_CRC_OFF;
}

ant_espphy_status_t ant_espphy_init(ant_espphy_t *dev)
{
    if (!dev) return ANT_ESPPHY_ERR_ARG;
    if (s_dev) return ANT_ESPPHY_ERR_ARG;           /* one instance only */
    if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_IDLE) {
        /* Bluedroid / NimBLE has the controller. The ANT modem needs it to
         * itself: stop that host (NimBLEDevice::deinit(true), btStop(), ...)
         * before starting ANT, and ant_espphy_deinit() before restarting it. */
        ESP_LOGE(TAG, "BT controller already in use by a BLE host; stop it first");
        return ANT_ESPPHY_ERR_BT;
    }
    memset(dev, 0, sizeof(*dev));
    espphy_setup_phy(dev);

    s_hci_done = xSemaphoreCreateBinary();
    const esp_timer_create_args_t targs = { .callback = tx_clear_cb, .arg = dev, .name = "ant_txclr" };
    if (!s_hci_done || esp_timer_create(&targs, &s_tx_clear) != ESP_OK) return ANT_ESPPHY_ERR_BT;

    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    cfg.sleep_mode = ESP_BT_SLEEP_MODE_NONE;        /* the modem is busy whenever ANT runs */
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    esp_err_t err = esp_bt_controller_init(&cfg);
    if (err == ESP_OK) err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BT controller: %s", esp_err_to_name(err));
        ant_espphy_deinit(dev);
        return ANT_ESPPHY_ERR_BT;
    }
    static const esp_vhci_host_callback_t vcb = { hci_on_ready, hci_on_event };
    esp_vhci_host_register_callback(&vcb);
    dev->bt_up = true;

    static const uint8_t evt_mask[8] = { 0, 0, 0, 0, 0, 0, 0, 0x20 };
    if (!hci_cmd(dev, HCI_RESET, NULL, 0) ||
        !hci_cmd(dev, HCI_SET_EVT_MASK, evt_mask, sizeof(evt_mask))) {
        ant_espphy_deinit(dev);
        return ANT_ESPPHY_ERR_BT;
    }

    /* The two entries we patch are ROM functions in every controller library
     * checked so far (IDF 4.4.6 = Arduino core 2.x, IDF 5.4). Anything else
     * means a table layout this code has not been verified against. */
    uintptr_t f0 = (uintptr_t)r_ip_funcs_p[IPF_LLD_TEST_EVT_START_CBK];
    uintptr_t f1 = (uintptr_t)r_ip_funcs_p[IPF_LLD_TEST_RX_ISR];
    if ((f0 >> 20) != 0x400 || (f1 >> 20) != 0x400) {
        ESP_LOGE(TAG, "unexpected controller function table (%p %p); refusing to hook",
                 (void *)f0, (void *)f1);
        ant_espphy_deinit(dev);
        return ANT_ESPPHY_ERR_UNSUPPORTED;
    }

    s_dev = dev;
    install_hooks();
    ESP_LOGI(TAG, "BLE core hooked for ANT: test evt_start %p (orig %p), rx_isr %p (orig %p)",
             (void *)hook_test_evt_start, (void *)s_orig_evt_start,
             (void *)hook_test_rx_isr, (void *)s_orig_rx_isr);
    return ANT_ESPPHY_OK;
}

/* Fill in the ant_phy_t vtable and the ANT defaults shared by both init paths. */
static void espphy_setup_phy(ant_espphy_t *dev)
{
    dev->phy.tune      = espphy_tune;
    dev->phy.tx        = espphy_tx;
    dev->phy.rx_config = espphy_rx_config;
    dev->phy.rx_enable = espphy_rx_enable;
    dev->phy.rx_poll   = espphy_rx_poll;
    dev->phy.ctx       = dev;
    dev->mhz           = 2457;
    dev->sync[0] = 0xAA; dev->sync[1] = 0xAA; dev->sync[2] = 0xA6; dev->sync[3] = 0xC5;
    dev->sync_skip = 2;
}

ant_espphy_status_t ant_espphy_init_coexist(ant_espphy_t *dev)
{
    if (!dev) return ANT_ESPPHY_ERR_ARG;
    if (s_dev) return ANT_ESPPHY_ERR_ARG;           /* one instance only */
    if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_ENABLED) {
        /* Coexist rides a BLE host's radio; that host must already be up and
         * running a scan. If nothing owns the controller, use ant_espphy_init()
         * (exclusive mode) instead. */
        ESP_LOGE(TAG, "coexist needs a running BLE host (controller not ENABLED)");
        return ANT_ESPPHY_ERR_BT;
    }
    memset(dev, 0, sizeof(*dev));
    espphy_setup_phy(dev);
    dev->coexist = true;
    dev->win_keep_crc = true;   /* the test format delivers CRC failures anyway */
    dev->bt_up   = false;                           /* the host owns the controller */

    /* The three scan entries must be ROM functions, as everywhere else this
     * table has been verified (IDF 4.4.6 / 5.4). Anything else is a layout we
     * have not checked. */
    uintptr_t f0 = (uintptr_t)r_ip_funcs_p[IPF_LLD_SCAN_SCHED];
    uintptr_t f1 = (uintptr_t)r_ip_funcs_p[IPF_LLD_SCAN_PROCESS_PKT_RX];
    if (!is_code_ptr((uintptr_t)r_ip_funcs_p[IPF_LLD_SCAN_EVT_START]) ||
        !is_code_ptr((uintptr_t)r_ip_funcs_p[IPF_LLD_SCAN_EVT_CANCELED]) ||
        !is_code_ptr((uintptr_t)r_ip_funcs_p[IPF_LLD_SCAN_FRM_SKIP_ISR]) ||
        !is_code_ptr((uintptr_t)r_ip_funcs_p[IPF_LLD_SCAN_FRM_EOF_ISR]))
        f0 = 0;
    if (!is_code_ptr(f0) || !is_code_ptr(f1)) {
        ESP_LOGE(TAG, "unexpected controller function table (%p %p); refusing to hook",
                 (void *)f0, (void *)f1);
        return ANT_ESPPHY_ERR_UNSUPPORTED;
    }

    s_dev = dev;
    install_scan_hooks();
    {
        esp_timer_create_args_t a = { .callback = coexist_win_timer_cb, .arg = dev,
                                      .dispatch_method = ESP_TIMER_TASK, .name = "antwin" };
        esp_timer_handle_t t = NULL;
        if (esp_timer_create(&a, &t) == ESP_OK && esp_timer_start_periodic(t, 1000) == ESP_OK)
            dev->win_timer = t;
        else
            ESP_LOGW(TAG, "window timer unavailable");
    }
    ESP_LOGI(TAG, "BLE scan path hooked for ANT coexist: sched %p rx %p",
             (void *)hook_scan_sched, (void *)hook_scan_rx);
    return ANT_ESPPHY_OK;
}

void ant_espphy_deinit(ant_espphy_t *dev)
{
    if (!dev) return;
    if (dev->coexist) {
        dev->mode = ANT_ESPPHY_MODE_IDLE;
        /* A window in the test format that is running right now would never
         * end once the timer is gone, and the host's next scan stop would
         * wait on it forever: end it first. */
        if (dev->win_active) {
            dev->win_active = false;
            RWBLECNTL |= RWBLECNTL_SCAN_ABORT;
            dev->aborts++;
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        if (dev->win_timer) {
            esp_timer_stop((esp_timer_handle_t)dev->win_timer);
            esp_timer_delete((esp_timer_handle_t)dev->win_timer);
            dev->win_timer = NULL;
        }
        if (s_dev == dev) { restore_scan_hooks(); s_dev = NULL; }
        coexist_restore_cs(dev);
        ESP_LOGI(TAG, "scan path released (controller stays with the BLE host)");
        return;
    }
    if (dev->bt_up && dev->mode != ANT_ESPPHY_MODE_IDLE) enter_mode(dev, ANT_ESPPHY_MODE_IDLE);
    if (s_dev == dev) {
        restore_hooks();
        s_dev = NULL;
    }
    if (s_tx_clear) { esp_timer_stop(s_tx_clear); esp_timer_delete(s_tx_clear); s_tx_clear = NULL; }
    if (dev->bt_up) {
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        dev->bt_up = false;
    } else if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
        esp_bt_controller_deinit();
    }
    if (s_hci_done) { vSemaphoreDelete(s_hci_done); s_hci_done = NULL; }
    dev->mode = ANT_ESPPHY_MODE_IDLE;
    ESP_LOGI(TAG, "radio released");
}

#else /* not S3/C3: keep the API, refuse to start */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static bool   stub_tune(ant_phy_t *p, uint16_t mhz) { (void)p; (void)mhz; return false; }
static bool   stub_tx(ant_phy_t *p, const uint8_t *f, size_t n) { (void)p; (void)f; (void)n; return false; }
static void   stub_rx_config(ant_phy_t *p, const ant_phy_rx_cfg_t *c) { ((ant_espphy_t *)p)->rx_cfg = *c; }
static void   stub_rx_enable(ant_phy_t *p, bool on) { ((ant_espphy_t *)p)->rx_on = on; }
static size_t stub_rx_poll(ant_phy_t *p, uint8_t *b, size_t cap, uint32_t *t, int8_t *rssi)
{
    (void)b; (void)cap; (void)t; (void)rssi; ((ant_espphy_t *)p)->rx_polls++; return 0;
}

bool ant_espphy_tap_poll(ant_espphy_t *dev, ant_espphy_rx_t *out, bool *matched)
{
    uint32_t tail = dev->tap_tail;
    if (tail == dev->tap_head) return false;
    *out = dev->tap[tail % ANT_ESPPHY_TAP_RING];
    if (matched) *matched = out->matched;
    dev->tap_tail = tail + 1;
    return true;
}

bool ant_espphy_wait_rx(ant_espphy_t *dev, uint32_t timeout_ms)
{
    (void)dev;
    vTaskDelay(pdMS_TO_TICKS(timeout_ms ? timeout_ms : 1));
    return false;
}

ant_espphy_status_t ant_espphy_init(ant_espphy_t *dev)
{
    if (!dev) return ANT_ESPPHY_ERR_ARG;
    memset(dev, 0, sizeof(*dev));
    dev->phy.tune      = stub_tune;
    dev->phy.tx        = stub_tx;
    dev->phy.rx_config = stub_rx_config;
    dev->phy.rx_enable = stub_rx_enable;
    dev->phy.rx_poll   = stub_rx_poll;
    dev->phy.ctx       = dev;
    dev->mhz           = 2457;
    ESP_LOGE(TAG, "no ANT radio backend for " CONFIG_IDF_TARGET " yet (S3/C3 only)");
    return ANT_ESPPHY_ERR_UNSUPPORTED;
}

ant_espphy_status_t ant_espphy_init_coexist(ant_espphy_t *dev)
{
    if (dev) { memset(dev, 0, sizeof(*dev)); }
    ESP_LOGE(TAG, "no ANT radio backend for " CONFIG_IDF_TARGET " yet (S3/C3 only)");
    return ANT_ESPPHY_ERR_UNSUPPORTED;
}

void ant_espphy_deinit(ant_espphy_t *dev) { (void)dev; }

void ant_espphy_debug_cs(ant_espphy_t *dev, uint8_t *out, size_t n)
{
    (void)dev;
    memset(out, 0, n);
}

#endif /* target */
#endif /* ESP_PLATFORM */
