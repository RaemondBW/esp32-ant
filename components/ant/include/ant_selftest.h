/*
 * ant_selftest.h - portable ANT protocol-engine self-test.
 *
 * Runs an ANT+ HRM sensor and an HRM display, both complete ANT MACs, against
 * each other over the virtual air (ant_air) inside one process: acquisition,
 * broadcast pages, an acknowledged transfer and a 40-byte burst. It is the
 * same code on the host (test suite) and on the ESP32 at boot, so a target
 * that passes it is known to run the whole protocol engine correctly; only
 * the PHY underneath differs.
 */
#ifndef ANT_SELFTEST_H
#define ANT_SELFTEST_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int      rx_pages;       /* broadcast pages the display received */
    uint16_t learned_dev;    /* id the wildcard display learned */
    uint8_t  learned_type;
    uint8_t  heart_rate;     /* decoded from the last page (expect 72) */
    int      ack_rx;         /* acknowledged pages received by the display */
    int      ack_tx_done;    /* TRANSFER_TX_COMPLETED at the sensor */
    int      burst_pkts;     /* burst packets received by the sensor */
    int      burst_tx_done;  /* TRANSFER_TX_COMPLETED at the display */
    int      tx_failed;      /* TRANSFER_TX_FAILED anywhere */
    uint32_t air_frames, air_delivered;
    bool     pass;
} ant_selftest_result_t;

/* Run it (takes no real time: the virtual air's clock is stepped). */
void ant_selftest_run(ant_selftest_result_t *r);

#ifdef __cplusplus
}
#endif

#endif /* ANT_SELFTEST_H */
