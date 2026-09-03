/*
 * ant_radio_loopback.h - Software radio backend + simulated ANT chip.
 *
 * Provides an ant_radio_t whose "other end" is a simulated ANT network
 * processor. The simulated chip understands the bring-up command sequence and
 * replies exactly as real hardware would (startup message after reset, a
 * channel response for each config command), then emits broadcast data pages
 * supplied by a page generator. This lets the entire ANT stack + ANT+ profile
 * code be exercised end-to-end on the host with no hardware.
 */
#ifndef ANT_RADIO_LOOPBACK_H
#define ANT_RADIO_LOOPBACK_H

#include "ant_radio.h"
#include "ant_message.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Called when the chip needs the next broadcast page to emit (may be NULL). */
typedef bool (*ant_sim_page_gen_t)(uint8_t out_page[8], void *user);

#define ANT_LOOPBACK_BUF 1024

typedef struct {
    ant_radio_t radio;

    /* Bytes queued for the stack to read (chip -> host). */
    uint8_t  rx[ANT_LOOPBACK_BUF];
    size_t   rx_head, rx_tail;

    /* Parser for host -> chip commands. */
    ant_parser_t cmd_parser;

    /* Simulated chip state. */
    bool     assigned;
    bool     opened;
    uint8_t  channel;
    uint16_t period;
    uint32_t tx_events;    /* number of broadcast pages emitted so far */

    /* Page source for emitted broadcasts. */
    ant_sim_page_gen_t page_gen;
    void    *page_user;

    /* Fault injection for tests. */
    bool     fail_next_command; /* respond with error code to next cmd */
    int      drop_every_n_rx;   /* if >0, drop every Nth emitted page */

    /* Log of received commands (for assertions). */
    uint8_t  last_cmd_id;
    uint32_t cmd_count;
} ant_loopback_t;

/* Initialize a loopback radio. `page_gen` (optional) supplies broadcast pages. */
void ant_loopback_init(ant_loopback_t *lb, ant_sim_page_gen_t page_gen,
                       void *page_user);

/* Get the ant_radio_t to hand to the stack. */
ant_radio_t *ant_loopback_radio(ant_loopback_t *lb);

/*
 * Advance the simulated chip by one "radio period": if a channel is open,
 * generate and enqueue one broadcast page (subject to drop injection) and a
 * matching TX event (for masters). Call once per simulated channel period.
 */
void ant_loopback_tick(ant_loopback_t *lb);

#ifdef __cplusplus
}
#endif

#endif /* ANT_RADIO_LOOPBACK_H */
