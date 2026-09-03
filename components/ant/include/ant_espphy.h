/*
 * ant_espphy.h - ANT on the ESP32's own 2.4 GHz radio (the pure-ESP32 PHY).
 *
 * Implements ant_phy_t on the BLE controller of an ESP32-S3 / ESP32-C3 with no
 * external chip. The trick (from ESPwn32 / esperanto, Cayre et al., WOOT 2023)
 * is that the RW-BLE link-layer core is a generic GFSK modem whose per-event
 * control structure (CS) lives in a RAM the CPU can write, and whose
 * link-layer functions are called through a writable table (r_ip_funcs_p).
 * We run the controller's own LE test mode (DTM, "channel 39") and hook two
 * of its functions: after the controller has programmed the test event we
 * rewrite the CS so the modem runs at 1 Mbit/s on 2457 MHz with the ANT sync
 * word and CRC/whitening off, and in the RX interrupt we lift the raw bytes
 * out of the RX descriptor. That is exactly ANT's ShockBurst air format, so
 * the ANT MAC (ant_mac) runs unchanged on top.
 *
 *   RX  : HCI LE receiver test. The event never ends, the evt_start hook
 *         reprograms the CS, the rx_isr hook hands every packet that matched
 *         the sync word to a lock-free ring for rx_poll(). Test mode runs with
 *         whitening off, so the bytes only need bit-reversal. The sync word is
 *         chosen from the MAC's address match: a known device number gives
 *         `a6 c5 dev_hi dev_lo`, a wildcard search gives `aa aa a6 c5`
 *         (preamble + network marker), and the rest of the address is matched
 *         in software - which is what lets several slave channels share the
 *         one receiver. Wildcard caveat: the core reads the byte after the
 *         sync word as a length, so ~5% of device numbers (those whose low
 *         byte bit-reverses to < 13) cannot be found by wildcard search;
 *         give the device number explicitly for those.
 *   TX  : HCI LE transmitter test; tx() drops the frame into the DTM payload
 *         buffer, so the next test packet carries it, and clears it again a
 *         few ms later. DTM transmits back-to-back, so the frame goes out a
 *         few times per slot; receivers ignore the repeats.
 *
 * The controller runs one test event at a time and switching takes
 * milliseconds of HCI traffic, so the backend is "sticky": rx_enable(true)
 * puts it in RX mode (a slave/display), the first tx() puts it in TX mode (a
 * master/sensor). Once in a mode the other direction is refused and counted.
 * A bidirectional master therefore transmits but never hears replies; the ANT
 * MAC copes (broadcasts do not need one).
 *
 * In the default (exclusive) mode the controller is used bare (no BLE host,
 * our own VHCI callback), so a BLE host cannot be up at the same time: init()
 * refuses unless the controller is idle, deinit() shuts it down again for the
 * host.
 *
 * There is also a coexist (shared-radio) mode - ant_espphy_init_coexist() -
 * that runs ANT receive *alongside* a live BLE host (NimBLE/Bluedroid) instead
 * of taking the controller. It hooks the controller's scan path rather than
 * test mode: while an ANT receive is open, the BLE host's passive-scan radio
 * windows are retuned to ANT, frames are lifted out, and the never-whitened
 * ANT payload is de-whitened in software - so no global radio register is
 * changed and BLE connection events between the scan windows keep working.
 * The scan windows are consumed by ANT (so BLE scanning pauses while an ANT
 * receive is open), and this mode is receive-only. See that function.
 *
 * Supported: ESP32-S3, ESP32-C3 (same controller; the function-table indices
 * are identical on ESP-IDF 4.4 and 5.x, so the Arduino core works too).
 * ESP32-C6 (a different controller) returns ANT_ESPPHY_ERR_UNSUPPORTED from
 * init for now.
 */
#ifndef ANT_ESPPHY_H
#define ANT_ESPPHY_H

#include "ant_phy.h"
#include "ant_phy_shockburst.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ANT_ESPPHY_OK = 0,
    ANT_ESPPHY_ERR_UNSUPPORTED,   /* this chip's controller is not supported yet */
    ANT_ESPPHY_ERR_BT,            /* BT controller init/enable or HCI failed */
    ANT_ESPPHY_ERR_ARG,
} ant_espphy_status_t;

typedef enum {
    ANT_ESPPHY_MODE_IDLE = 0,     /* controller up, no radio event running */
    ANT_ESPPHY_MODE_RX,           /* LE receiver test: raw ANT receive */
    ANT_ESPPHY_MODE_TX,           /* DTM: raw ANT transmit */
} ant_espphy_mode_t;

/* One received frame, as parked by the RX interrupt hook. */
#define ANT_ESPPHY_RX_RING  8u
typedef struct {
    uint8_t  body[ANT_SB_FRAME_MAX];  /* address|payload|crc, preamble stripped */
    uint8_t  len;
    int8_t   rssi;
    bool     matched;                 /* address passed the MAC's mask (tap only) */
    uint32_t tick;
} ant_espphy_rx_t;

#define ANT_ESPPHY_TAP_RING 16

typedef struct {
    ant_phy_t phy;                    /* must be first */
    volatile ant_espphy_mode_t mode;
    bool     bt_up;                   /* this driver brought the controller up */
    bool     coexist;                 /* shared-radio mode: a BLE host owns the
                                         controller, ANT rides its passive scan */
    uint16_t mhz;
    /* coexist: EM offset of the scan activity's control structure (0 = not
     * found yet), and the fields we overwrite in it, kept for deinit. */
    uint16_t cs_off;
    uint8_t  cs_saved[48];
    bool     cs_have_saved;
    /* coexist bring-up knobs, applied to the scan CS at every (re)schedule
     * after the normal retune: a format byte to force (0 = keep the scan's),
     * and up to four raw byte overrides. */
    uint8_t  coexist_ch;              /* channel index the retuned windows run on
                                         (0 = 39). It is also the whitening seed the
                                         core applies, so it decides how the byte after
                                         the sync word reads as a packet length. */
    uint8_t  cs_fmt_override;
    struct { uint8_t off, val; } cs_ovr[4];
    uint8_t  cs_ovr_n;
    bool     win_crc_off;             /* CRC checking is off for the current window */
    bool     win_keep_crc;            /* knob: leave CRC checking on (experiment) */
    uint32_t win_hook_calls;          /* scan windows retargeted at start */
    uint8_t  abort_mode;              /* knob (default 0 = timer + SCAN_ABORT): 1/2/4 also
                                         pulse in the reschedule hook / end-of-frame ISR
                                         / canceled callback; 0x20 = RFTEST_ABORT instead */
    uint32_t aborts;                  /* abort pulses issued */
    uint16_t win_len_ms;              /* window length override, ms (0 = derive it from
                                         the scan's own MINEVTIME each window) */
    uint16_t win_minevt;              /* MINEVTIME read from the scan CS at window start */
    uint32_t win_len_us;              /* length in use for the current window */
    uint32_t win_ll_ended;            /* windows the LL ended before our abort */
    uint32_t win_ll_min_us;           /* ... the shortest such window (0 = none yet) */
    volatile uint32_t win_start_us;   /* esp_timer time of the current window's start */
    volatile bool     win_active;
    void    *win_timer;               /* periodic esp_timer driving the abort */
    uint16_t mhz_override;            /* knob: window frequency instead of the MAC's (0 = off) */
    uint8_t  sync_override[4];        /* knob: on-air sync bytes to program instead of
                                         the MAC's (sync_override_on) */
    bool     sync_override_on;
    bool     keep_rx_hdr;             /* knob: do not zero consumed RX descriptor headers */
    uint8_t  patch_mask;              /* knob: which CS patches to apply at window
                                         start (0 = all): 1 sync, 2 freq+hop, 4 rate,
                                         8 rxmaxbuf */
    uint32_t scan_rx_isr_calls;       /* scan RX ISR entries (a packet in a window) */
    uint32_t scan_rx_isr_hdr;         /* ... last descriptor header word seen there */
    uint32_t scan_rx_isr_stat;        /* ... and its cntl|stat word */
    uint32_t rx_last_hdr;             /* last non-empty RX descriptor header word */
    uint32_t rx_last_stat;            /* ... and the word before it (cntl|stat) */

    /* receiver programming */
    ant_phy_rx_cfg_t rx_cfg;
    uint8_t  sync[4];                 /* wanted: on-air order, first byte first */
    uint8_t  sync_skip;               /* sync bytes that are NOT part of the body (2 for aa aa) */
    uint8_t  cs_sync[4];              /* what the control structure currently holds */
    uint8_t  cs_sync_skip;
    bool     rx_on;
    void    *waiter;                  /* task to notify when a frame is queued */

    /* ISR -> task ring (single producer, single consumer) */
    ant_espphy_rx_t ring[ANT_ESPPHY_RX_RING];
    volatile uint32_t ring_head, ring_tail;

    /* Diagnostic tap: every packet the hook decodes, matched or not, when
     * `tap_on` is set. Drain with ant_espphy_tap_poll() from a task. */
    bool tap_on;
    ant_espphy_rx_t tap[ANT_ESPPHY_TAP_RING];
    volatile uint32_t tap_head, tap_tail;

    /* counters (read by the app log; written from hooks, plain stores) */
    uint32_t tx_count;                /* frames handed to tx() */
    uint32_t tx_emitted;              /* ... written into the DTM buffer */
    uint32_t tx_refused;              /* ... refused because we are in RX mode */
    uint32_t rx_enables_ignored;      /* rx_enable(true) while in TX mode */
    uint32_t tunes;
    uint32_t rx_polls;
    uint32_t rx_hook_calls;           /* RX interrupt entries with a packet */
    uint32_t rx_hook_empty;           /* ... entries with an empty header */
    uint32_t rx_frames;               /* packets carrying at least a whole body */
    uint32_t rx_matched;              /* ... whose address passed the MAC's mask */
    uint32_t rx_dropped;              /* ... lost because the ring was full */
    uint32_t sched_hook_calls;        /* test-event start hook (CS rewrites) */
    uint32_t sync_rewrites;           /* live sync-word changes (acquire / lose) */
    uint32_t hci_errors;
    int8_t   last_rssi;
    uint8_t  last_frame[ANT_SB_FRAME_MAX]; /* last body sent or received */
    uint8_t  last_len;
} ant_espphy_t;

/* Bring up the BT controller, install the hooks, leave the radio idle. Only
 * one instance can exist (the hooks are global). */
ant_espphy_status_t ant_espphy_init(ant_espphy_t *dev);

/* Shared-radio variant: coexist with a running BLE host (NimBLE/Bluedroid)
 * instead of taking the controller. The BLE host must already be initialised
 * AND running a passive scan (that scan's radio windows are what ANT rides
 * on); the controller stays owned by the host. This installs hooks on the
 * *scan* path (r_lld_scan_sched / r_lld_scan_process_pkt_rx): while an ANT
 * receive is open, each scan window is retuned to 2457 MHz / 1 Mbit/s / the
 * ANT sync word with CRC and whitening disabled, ANT frames are lifted out of
 * the RX descriptor, and the controls are restored at the end of the window so
 * BLE connection events in between run normally. RX only - tx() is refused in
 * this mode (the scan activity has no transmit slot). Because the scan windows
 * are consumed by ANT, real BLE scanning does not run while an ANT receive is
 * open; BLE connections are unaffected. Returns ANT_ESPPHY_ERR_BT if no BLE
 * host has the controller enabled. Call ant_espphy_deinit() to remove the
 * hooks; it leaves the controller with the host. */
ant_espphy_status_t ant_espphy_init_coexist(ant_espphy_t *dev);

/* Diagnostics: with dev->tap_on set, every packet the RX hook decodes (address
 * matched or not) is copied into a side ring; this pops the oldest one. The
 * body is {address | payload | crc}; `matched` tells whether it also went to
 * the MAC. Returns false when the ring is empty. */
bool ant_espphy_tap_poll(ant_espphy_t *dev, ant_espphy_rx_t *out, bool *matched);

/* Stop the radio event, unhook the controller and shut it down, so a BLE host
 * (NimBLE, Bluedroid) can take the controller again. The ANT MAC must not be
 * ticked after this. */
void ant_espphy_deinit(ant_espphy_t *dev);
const char *ant_espphy_status_str(ant_espphy_status_t s);
const char *ant_espphy_mode_str(ant_espphy_mode_t m);

/* The ant_phy_t to hand to ant_radio_embedded_init(). */
ant_phy_t *ant_espphy_phy(ant_espphy_t *dev);

/* ANT ticks (1/32768 s) from the high-resolution timer, for ant_mac_tick(). */
uint32_t ant_espphy_ticks(void);

/* Block the calling task up to `timeout_ms` or until the receiver queues a
 * frame, whichever is first. Returns true if a frame is waiting. Lets the ANT
 * task sleep instead of polling rx_poll() every few hundred microseconds. */
bool ant_espphy_wait_rx(ant_espphy_t *dev, uint32_t timeout_ms);

/* Debug: copy the first n-1 bytes of the radio control structure and, as the
 * last byte, the frequency-table entry the receiver uses. */
void ant_espphy_debug_cs(ant_espphy_t *dev, uint8_t *out, size_t n);

#ifdef __cplusplus
}
#endif
#endif /* ANT_ESPPHY_H */
