/*
 * ant_selftest.c - see the header. Two full ANT MACs over ant_air.
 */
#include "ant_selftest.h"
#include "ant_mac.h"
#include "ant_air.h"
#include "ant_channel.h"       /* ANTPLUS_NETWORK_KEY, ANTPLUS_RF_FREQ */
#include "antplus_profiles.h"
#include <string.h>

typedef struct {
    ant_mac_t mac;
    int rx_pages, ack_rx, burst_pkts, tx_done, tx_fail;
    uint8_t last[8];
} st_node_t;

static void st_data(ant_mac_t *m, uint8_t ch, uint8_t type, uint8_t seq,
                    const uint8_t d[8], void *u)
{
    (void)m; (void)ch; (void)seq;
    st_node_t *n = u;
    if (type == ANT_MSG_BROADCAST_DATA) n->rx_pages++;
    else if (type == ANT_MSG_ACKNOWLEDGED_DATA) n->ack_rx++;
    else n->burst_pkts++;
    memcpy(n->last, d, 8);
}

static void st_event(ant_mac_t *m, uint8_t ch, uint8_t ev, void *u)
{
    (void)m; (void)ch;
    st_node_t *n = u;
    if (ev == ANT_EVENT_TRANSFER_TX_COMPLETED) n->tx_done++;
    if (ev == ANT_EVENT_TRANSFER_TX_FAILED) n->tx_fail++;
}

static void st_run(ant_air_t *air, st_node_t *a, st_node_t *b, uint32_t ticks)
{
    uint32_t end = air->now + ticks;
    uint32_t da = ant_mac_tick(&a->mac, air->now);
    uint32_t db = ant_mac_tick(&b->mac, air->now);
    for (;;) {
        uint32_t next = (int32_t)(da - db) < 0 ? da : db;
        if ((int32_t)(next - end) >= 0) break;
        air->now = next;
        if ((int32_t)(da - next) <= 0) da = ant_mac_tick(&a->mac, next);
        if ((int32_t)(db - next) <= 0) db = ant_mac_tick(&b->mac, next);
    }
    air->now = end;
}

/* Static: two MACs plus the air are a few KB, too much for a small stack. */
static ant_air_t  g_air;
static st_node_t  g_sensor, g_display;

void ant_selftest_run(ant_selftest_result_t *r)
{
    memset(r, 0, sizeof(*r));
    memset(&g_sensor, 0, sizeof(g_sensor));
    memset(&g_display, 0, sizeof(g_display));
    ant_air_init(&g_air);
    ant_mac_init(&g_sensor.mac,  ant_air_attach(&g_air), st_data, st_event, &g_sensor);
    ant_mac_init(&g_display.mac, ant_air_attach(&g_air), st_data, st_event, &g_display);

    ant_mac_t *s = &g_sensor.mac, *d = &g_display.mac;
    bool ok = true;
    ok &= ant_mac_set_network_key(s, 0, ANTPLUS_NETWORK_KEY) == ANT_RESPONSE_NO_ERROR;
    ok &= ant_mac_assign_channel(s, 0, ANT_CHANNEL_TYPE_MASTER_TX, 0) == ANT_RESPONSE_NO_ERROR;
    ok &= ant_mac_set_channel_id(s, 0, 0x1234, ANTPLUS_DEVTYPE_HRM, 1) == ANT_RESPONSE_NO_ERROR;
    ok &= ant_mac_set_channel_rf_freq(s, 0, ANTPLUS_RF_FREQ) == ANT_RESPONSE_NO_ERROR;
    ok &= ant_mac_set_channel_period(s, 0, ANTPLUS_PERIOD_HRM) == ANT_RESPONSE_NO_ERROR;
    ok &= ant_mac_open_channel(s, 0) == ANT_RESPONSE_NO_ERROR;

    ok &= ant_mac_set_network_key(d, 0, ANTPLUS_NETWORK_KEY) == ANT_RESPONSE_NO_ERROR;
    ok &= ant_mac_assign_channel(d, 0, ANT_CHANNEL_TYPE_SLAVE_RX, 0) == ANT_RESPONSE_NO_ERROR;
    ok &= ant_mac_set_channel_id(d, 0, 0, ANTPLUS_DEVTYPE_HRM, 0) == ANT_RESPONSE_NO_ERROR;
    ok &= ant_mac_set_channel_rf_freq(d, 0, ANTPLUS_RF_FREQ) == ANT_RESPONSE_NO_ERROR;
    ok &= ant_mac_set_channel_period(d, 0, ANTPLUS_PERIOD_HRM) == ANT_RESPONSE_NO_ERROR;
    ok &= ant_mac_set_search_timeout(d, 0, 4) == ANT_RESPONSE_NO_ERROR;
    ok &= ant_mac_open_channel(d, 0) == ANT_RESPONSE_NO_ERROR;
    if (!ok) { r->pass = false; return; }

    antplus_hrm_data_t hr = {0};
    hr.computed_heart_rate = 72; hr.heart_beat_count = 1; hr.heart_beat_event_time = 1024;
    uint8_t page[8];
    antplus_hrm_encode_page0(&hr, false, page);
    ant_mac_send_broadcast(s, 0, page);

    /* 2 s of broadcast: the display must acquire and receive ~8 pages */
    st_run(&g_air, &g_sensor, &g_display, 2 * ANT_TICKS_PER_SEC);
    uint8_t tt = 0;
    ant_mac_get_channel_id(d, 0, &r->learned_dev, &r->learned_type, &tt);
    antplus_hrm_data_t got = {0};
    bool decoded = antplus_hrm_decode(g_display.last, &got);
    r->rx_pages = g_display.rx_pages;
    r->heart_rate = got.computed_heart_rate;
    ok &= g_display.rx_pages >= 6 && r->learned_dev == 0x1234 && r->learned_type == ANTPLUS_DEVTYPE_HRM;
    ok &= decoded && got.computed_heart_rate == 72;

    /* acknowledged sensor -> display, and a 40-byte burst display -> sensor */
    static const uint8_t ack[8] = { 0x46, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x01, 0x01 };
    ant_mac_send_acknowledged(s, 0, ack);
    uint8_t burst[40];
    for (int i = 0; i < 40; i++) burst[i] = (uint8_t)i;
    ant_mac_send_burst(d, 0, burst, sizeof(burst));
    st_run(&g_air, &g_sensor, &g_display, ANT_TICKS_PER_SEC);

    r->ack_rx = g_display.ack_rx;
    r->ack_tx_done = g_sensor.tx_done;
    r->burst_pkts = g_sensor.burst_pkts;
    r->burst_tx_done = g_display.tx_done;
    r->tx_failed = g_sensor.tx_fail + g_display.tx_fail;
    r->air_frames = g_air.frames;
    r->air_delivered = g_air.delivered;
    ok &= r->ack_rx == 1 && r->ack_tx_done == 1;
    ok &= r->burst_pkts == 5 && r->burst_tx_done == 1;
    ok &= r->tx_failed == 0;
    /* the burst's last packet must be intact */
    ok &= g_sensor.last[7] == 39;
    r->pass = ok;
}
