/*
 * test_mac.c - the ANT air-interface MAC, end to end over the virtual air.
 *
 * Several ant_mac instances (nodes), each with its own radio on one ant_air,
 * are ticked event-driven: every node is called exactly at the deadline it
 * asked for, in time order, the way the ESP32 main loop calls it. Nothing here
 * peeks inside the MAC beyond its public queries; the assertions are about
 * what the peer receives and what events the application sees.
 */
#include "test.h"
#include "ant_mac.h"
#include "ant_air.h"
#include "antplus_profiles.h"

#define MAX_NODES   4
#define LOG_MAX     512

#define ANTPLUS_FREQ 57u
#define HRM_TYPE     120u
#define HRM_PERIOD   8070u
#define PWR_TYPE     11u
#define PWR_PERIOD   8182u

static const uint8_t ANTPLUS_KEY[8] = { 0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45 };
static const uint8_t ZERO_KEY[8]    = { 0 };

typedef struct {
    uint32_t t;
    uint8_t  ch;
    uint8_t  code;
} ev_rec_t;

typedef struct {
    uint32_t t;
    uint8_t  ch;
    uint8_t  type;
    uint8_t  seq;
    uint8_t  data[8];
} data_rec_t;

typedef struct {
    ant_mac_t  mac;
    ant_phy_t *phy;
    uint32_t   deadline;
    ev_rec_t   ev[LOG_MAX];
    int        n_ev;
    data_rec_t rx[LOG_MAX];
    int        n_rx;
    uint32_t   rx_total;   /* all data deliveries, even past the log */
} node_t;

typedef struct {
    ant_air_t air;
    node_t    node[MAX_NODES];
    int       n_nodes;
    uint32_t  now;
} sim_t;

static sim_t S;

static void on_data(ant_mac_t *mac, uint8_t ch, uint8_t type, uint8_t seq,
                    const uint8_t data[8], void *user)
{
    (void)mac;
    node_t *n = user;
    n->rx_total++;
    if (n->n_rx < LOG_MAX) {
        data_rec_t *r = &n->rx[n->n_rx++];
        r->t = S.now; r->ch = ch; r->type = type; r->seq = seq;
        memcpy(r->data, data, 8);
    }
}

static void on_event(ant_mac_t *mac, uint8_t ch, uint8_t event, void *user)
{
    (void)mac;
    node_t *n = user;
    if (n->n_ev < LOG_MAX) {
        ev_rec_t *e = &n->ev[n->n_ev++];
        e->t = S.now; e->ch = ch; e->code = event;
    }
}

static void sim_init(int n_nodes, uint32_t t0)
{
    memset(&S, 0, sizeof(S));
    ant_air_init(&S.air);
    S.now = t0;
    S.air.now = t0;
    S.n_nodes = n_nodes;
    for (int i = 0; i < n_nodes; i++) {
        node_t *n = &S.node[i];
        n->phy = ant_air_attach(&S.air);
        ant_mac_init(&n->mac, n->phy, on_data, on_event, n);
        n->deadline = t0;
    }
}

/* Event-driven run: each node is ticked exactly at its own deadline. Commands
 * may have been issued since the last tick, so (as an application's main loop
 * does after issuing commands) every node is ticked once up front. */
static void sim_run(uint32_t ticks)
{
    uint32_t end = S.now + ticks;
    for (int i = 0; i < S.n_nodes; i++) {
        S.node[i].deadline = ant_mac_tick(&S.node[i].mac, S.now);
    }
    for (;;) {
        uint32_t next = end;
        for (int i = 0; i < S.n_nodes; i++) {
            if ((int32_t)(S.node[i].deadline - next) < 0) next = S.node[i].deadline;
        }
        if ((int32_t)(next - end) >= 0) break;
        S.now = next;
        S.air.now = next;
        for (int i = 0; i < S.n_nodes; i++) {
            node_t *n = &S.node[i];
            if ((int32_t)(n->deadline - S.now) <= 0) {
                n->deadline = ant_mac_tick(&n->mac, S.now);
            }
        }
    }
    S.now = end;
    S.air.now = end;
}

static void log_clear(node_t *n) { n->n_ev = 0; n->n_rx = 0; n->rx_total = 0; }

static int count_events(const node_t *n, uint8_t ch, uint8_t code)
{
    int k = 0;
    for (int i = 0; i < n->n_ev; i++) if (n->ev[i].ch == ch && n->ev[i].code == code) k++;
    return k;
}

static int count_data(const node_t *n, uint8_t ch, uint8_t type)
{
    int k = 0;
    for (int i = 0; i < n->n_rx; i++) if (n->rx[i].ch == ch && n->rx[i].type == type) k++;
    return k;
}

static const ev_rec_t *first_event(const node_t *n, uint8_t ch, uint8_t code)
{
    for (int i = 0; i < n->n_ev; i++) if (n->ev[i].ch == ch && n->ev[i].code == code) return &n->ev[i];
    return NULL;
}

static int event_seq_ok(const node_t *n, uint8_t ch, uint8_t first, uint8_t second)
{
    /* `first` must occur, and `second` must occur after it. */
    int i = 0;
    for (; i < n->n_ev; i++) if (n->ev[i].ch == ch && n->ev[i].code == first) break;
    for (; i < n->n_ev; i++) if (n->ev[i].ch == ch && n->ev[i].code == second) return 1;
    return 0;
}

static uint8_t state_of(const node_t *n, uint8_t ch)
{
    return ant_mac_channel_status(&n->mac, ch) & ANT_STATUS_STATE_MASK;
}

/* Configure an ANT+ style channel; returns the first non-zero response code. */
static uint8_t setup_channel(node_t *n, uint8_t ch, uint8_t type, uint8_t net,
                             uint16_t dev, uint8_t devtype, uint8_t trans,
                             uint16_t period, uint8_t timeout)
{
    uint8_t r;
    if ((r = ant_mac_assign_channel(&n->mac, ch, type, net))) return r;
    if ((r = ant_mac_set_channel_id(&n->mac, ch, dev, devtype, trans))) return r;
    if ((r = ant_mac_set_channel_rf_freq(&n->mac, ch, ANTPLUS_FREQ))) return r;
    if ((r = ant_mac_set_channel_period(&n->mac, ch, period))) return r;
    if ((r = ant_mac_set_search_timeout(&n->mac, ch, timeout))) return r;
    return 0;
}

static void hrm_page(uint8_t bpm, uint8_t count, uint8_t page[8])
{
    antplus_hrm_data_t d;
    memset(&d, 0, sizeof(d));
    d.page_number = 0;
    d.computed_heart_rate = bpm;
    d.heart_beat_count = count;
    d.heart_beat_event_time = (uint16_t)(count * 500u);
    antplus_hrm_encode_page0(&d, (count & 4) != 0, page);
}

/* Standard 2-node fixture: node0 = HRM master 0x1234, node1 = wildcard slave. */
static void pair_hrm(uint32_t t0, uint8_t slave_type, uint8_t timeout)
{
    sim_init(2, t0);
    ant_mac_set_network_key(&S.node[0].mac, 0, ANTPLUS_KEY);
    ant_mac_set_network_key(&S.node[1].mac, 0, ANTPLUS_KEY);
    setup_channel(&S.node[0], 0, ANT_CHANNEL_TYPE_MASTER_TX, 0, 0x1234, HRM_TYPE, 1, HRM_PERIOD, 0);
    setup_channel(&S.node[1], 0, slave_type, 0, 0, 0, 0, HRM_PERIOD, timeout);
    uint8_t page[8];
    hrm_page(72, 1, page);
    ant_mac_send_broadcast(&S.node[0].mac, 0, page);
    ant_mac_open_channel(&S.node[0].mac, 0);
    ant_mac_open_channel(&S.node[1].mac, 0);
}

/* ------------------------------------------------------------------------ */

static int test_master_schedule_is_exact(void)
{
    sim_init(1, 1000);
    node_t *m = &S.node[0];
    CHECK_EQ(setup_channel(m, 0, ANT_CHANNEL_TYPE_MASTER_TX, 0, 0x1234, HRM_TYPE, 1, HRM_PERIOD, 0), 0);
    uint8_t page[8]; hrm_page(60, 0, page);
    CHECK_EQ(ant_mac_send_broadcast(&m->mac, 0, page), ANT_RESPONSE_NO_ERROR);
    CHECK_EQ(ant_mac_open_channel(&m->mac, 0), ANT_RESPONSE_NO_ERROR);
    CHECK_EQ(state_of(m, 0), ANT_STATUS_TRACKING);   /* masters go straight to tracking */

    sim_run(10 * ANT_TICKS_PER_SEC);

    /* 10 s at 8070 ticks: 40 or 41 transmissions, each announced by EVENT_TX,
     * consecutive ones exactly one period apart - no drift, no jitter. */
    int n_tx = count_events(m, 0, ANT_EVENT_TX);
    CHECK(n_tx >= 40 && n_tx <= 41);
    CHECK_EQ(ant_air_radio(m->phy)->tx_count, (uint32_t)n_tx);
    uint32_t prev = 0; int first = 1;
    for (int i = 0; i < m->n_ev; i++) {
        if (m->ev[i].code != ANT_EVENT_TX) continue;
        if (!first) CHECK_EQ(m->ev[i].t - prev, HRM_PERIOD);
        prev = m->ev[i].t; first = 0;
    }
    CHECK_EQ(count_events(m, 0, ANT_EVENT_CHANNEL_COLLISION), 0);
    return 0;
}

static int test_master_without_page_broadcasts_zeros(void)
{
    /* ANT: a master transmits from the moment it is opened; with no page
     * loaded yet it sends all-zero data (and reports EVENT_TX for each). */
    sim_init(2, 0);
    node_t *m = &S.node[0], *s = &S.node[1];
    CHECK_EQ(setup_channel(m, 0, ANT_CHANNEL_TYPE_MASTER_TX, 0, 0x1234, HRM_TYPE, 1, HRM_PERIOD, 0), 0);
    CHECK_EQ(setup_channel(s, 0, ANT_CHANNEL_TYPE_SLAVE_RX, 0, 0x1234, HRM_TYPE, 1, HRM_PERIOD, 0xFF), 0);
    CHECK_EQ(ant_mac_open_channel(&m->mac, 0), ANT_RESPONSE_NO_ERROR);
    CHECK_EQ(ant_mac_open_channel(&s->mac, 0), ANT_RESPONSE_NO_ERROR);
    sim_run(2 * ANT_TICKS_PER_SEC);
    CHECK(count_events(m, 0, ANT_EVENT_TX) >= 8);
    CHECK(s->n_rx >= 7);
    static const uint8_t zeros[8] = { 0 };
    CHECK(memcmp(s->rx[0].data, zeros, 8) == 0);
    return 0;
}

static int test_wildcard_slave_acquires_and_learns_id(void)
{
    pair_hrm(5000, ANT_CHANNEL_TYPE_SLAVE_RX, 0xFF);
    node_t *m = &S.node[0], *s = &S.node[1];
    CHECK_EQ(state_of(s, 0), ANT_STATUS_SEARCHING);

    sim_run(3 * ANT_TICKS_PER_SEC);

    CHECK_EQ(state_of(s, 0), ANT_STATUS_TRACKING);
    uint16_t dev; uint8_t dt, tt;
    CHECK(ant_mac_get_channel_id(&s->mac, 0, &dev, &dt, &tt));
    CHECK_EQ(dev, 0x1234);
    CHECK_EQ(dt, HRM_TYPE);
    CHECK_EQ(tt, 1);

    /* Every frame the master sent after the slave's first receive arrived. */
    int n_tx = count_events(m, 0, ANT_EVENT_TX);
    int n_rx = count_data(s, 0, ANT_MSG_BROADCAST_DATA);
    CHECK(n_rx >= n_tx - 1 && n_rx <= n_tx);
    CHECK_EQ(count_events(s, 0, ANT_EVENT_RX_FAIL), 0);

    /* And the payload is the HRM page, decodable by the profile layer. */
    antplus_hrm_data_t hr;
    CHECK(antplus_hrm_decode(s->rx[0].data, &hr));
    CHECK_EQ(hr.computed_heart_rate, 72);
    return 0;
}

static int test_slave_receive_is_on_the_master_grid(void)
{
    pair_hrm(0, ANT_CHANNEL_TYPE_SLAVE_RX, 0xFF);
    node_t *s = &S.node[1];
    sim_run(5 * ANT_TICKS_PER_SEC);
    /* Consecutive receives are one period apart (to within the receiver
     * polling interval): the master is exact and the slave is on its grid. */
    CHECK(s->n_rx >= 15);
    for (int i = 1; i < s->n_rx; i++) {
        int32_t d = (int32_t)(s->rx[i].t - s->rx[i - 1].t) - (int32_t)HRM_PERIOD;
        CHECK(d >= -(int32_t)ANT_MAC_POLL_INTERVAL && d <= (int32_t)ANT_MAC_POLL_INTERVAL);
    }
    /* The receiver is not left on all the time: with the slot known, it is
     * only open for a window around each slot. */
    ant_air_radio_t *r = ant_air_radio(s->phy);
    CHECK_EQ(r->rx_count, (uint32_t)s->n_rx);
    return 0;
}

static int test_specific_id_slave_ignores_other_master(void)
{
    sim_init(3, 0);
    node_t *a = &S.node[0], *b = &S.node[1], *s = &S.node[2];
    for (int i = 0; i < 3; i++) ant_mac_set_network_key(&S.node[i].mac, 0, ANTPLUS_KEY);
    setup_channel(a, 0, ANT_CHANNEL_TYPE_MASTER_TX, 0, 0x1111, HRM_TYPE, 1, HRM_PERIOD, 0);
    setup_channel(b, 0, ANT_CHANNEL_TYPE_MASTER_TX, 0, 0x2222, HRM_TYPE, 1, HRM_PERIOD, 0);
    setup_channel(s, 0, ANT_CHANNEL_TYPE_SLAVE_RX, 0, 0x2222, HRM_TYPE, 1, HRM_PERIOD, 0xFF);
    uint8_t pa[8], pb[8];
    hrm_page(100, 0, pa); hrm_page(150, 0, pb);
    ant_mac_send_broadcast(&a->mac, 0, pa);
    ant_mac_send_broadcast(&b->mac, 0, pb);
    ant_mac_open_channel(&a->mac, 0);
    /* Stagger the two masters so their slots never coincide. */
    S.now += 3000; S.air.now = S.now;
    ant_mac_open_channel(&b->mac, 0);
    ant_mac_open_channel(&s->mac, 0);
    sim_run(4 * ANT_TICKS_PER_SEC);
    CHECK_EQ(state_of(s, 0), ANT_STATUS_TRACKING);
    CHECK(s->n_rx > 10);
    for (int i = 0; i < s->n_rx; i++) {
        antplus_hrm_data_t hr;
        CHECK(antplus_hrm_decode(s->rx[i].data, &hr));
        CHECK_EQ(hr.computed_heart_rate, 150);
    }
    return 0;
}

static int test_dropped_frames_give_rx_fail_but_keep_tracking(void)
{
    pair_hrm(0, ANT_CHANNEL_TYPE_SLAVE_RX, 0xFF);
    node_t *m = &S.node[0], *s = &S.node[1];
    sim_run(2 * ANT_TICKS_PER_SEC);
    CHECK_EQ(state_of(s, 0), ANT_STATUS_TRACKING);
    log_clear(m); log_clear(s);

    S.air.drop_every_n = 4;          /* lose every 4th frame on the air */
    sim_run(10 * ANT_TICKS_PER_SEC);

    int n_tx   = count_events(m, 0, ANT_EVENT_TX);
    int n_rx   = count_data(s, 0, ANT_MSG_BROADCAST_DATA);
    int n_fail = count_events(s, 0, ANT_EVENT_RX_FAIL);
    CHECK(n_tx >= 40);
    CHECK_EQ(n_rx + n_fail, n_tx);                   /* every slot accounted for */
    CHECK(n_fail >= n_tx / 4 - 1 && n_fail <= n_tx / 4 + 1);
    CHECK_EQ(count_events(s, 0, ANT_EVENT_RX_FAIL_GO_TO_SEARCH), 0);
    CHECK_EQ(state_of(s, 0), ANT_STATUS_TRACKING);

    /* Recovery: with the air clean again the slave misses nothing. */
    S.air.drop_every_n = 0;
    log_clear(m); log_clear(s);
    sim_run(3 * ANT_TICKS_PER_SEC);
    CHECK_EQ(count_events(s, 0, ANT_EVENT_RX_FAIL), 0);
    CHECK_EQ(count_data(s, 0, ANT_MSG_BROADCAST_DATA), count_events(m, 0, ANT_EVENT_TX));
    return 0;
}

static int test_corrupted_frames_are_rejected(void)
{
    pair_hrm(0, ANT_CHANNEL_TYPE_SLAVE_RX, 0xFF);
    node_t *m = &S.node[0], *s = &S.node[1];
    sim_run(2 * ANT_TICKS_PER_SEC);
    log_clear(m); log_clear(s);
    S.air.corrupt_every_n = 3;
    sim_run(6 * ANT_TICKS_PER_SEC);
    int n_tx = count_events(m, 0, ANT_EVENT_TX);
    int n_rx = count_data(s, 0, ANT_MSG_BROADCAST_DATA);
    int n_fail = count_events(s, 0, ANT_EVENT_RX_FAIL);
    CHECK_EQ(n_rx + n_fail, n_tx);
    CHECK(n_fail >= n_tx / 3 - 1);
    CHECK(n_rx >= 2 * n_tx / 3 - 1);
    /* Every delivered page is intact (decodes with the expected rate). */
    for (int i = 0; i < s->n_rx; i++) {
        antplus_hrm_data_t hr;
        CHECK(antplus_hrm_decode(s->rx[i].data, &hr));
        CHECK_EQ(hr.computed_heart_rate, 72);
    }
    return 0;
}

static int test_master_gone_drops_to_search_and_reacquires(void)
{
    pair_hrm(0, ANT_CHANNEL_TYPE_SLAVE_RX, 0xFF);
    node_t *m = &S.node[0], *s = &S.node[1];
    sim_run(2 * ANT_TICKS_PER_SEC);
    CHECK_EQ(state_of(s, 0), ANT_STATUS_TRACKING);
    log_clear(s);

    CHECK_EQ(ant_mac_close_channel(&m->mac, 0), ANT_RESPONSE_NO_ERROR);
    CHECK_EQ(count_events(m, 0, ANT_EVENT_CHANNEL_CLOSED), 1);
    CHECK_EQ(state_of(m, 0), ANT_STATUS_ASSIGNED);
    uint32_t t_close = S.now;

    sim_run(4 * ANT_TICKS_PER_SEC);
    CHECK(count_events(s, 0, ANT_EVENT_RX_FAIL) >= 7);
    const ev_rec_t *gts = first_event(s, 0, ANT_EVENT_RX_FAIL_GO_TO_SEARCH);
    CHECK(gts != NULL);
    /* ~2 s of misses (ANT_MAC_DROP_TO_SEARCH_TICKS) after the last frame. */
    CHECK(gts->t - t_close >= ANT_MAC_DROP_TO_SEARCH_TICKS);
    CHECK(gts->t - t_close <= ANT_MAC_DROP_TO_SEARCH_TICKS + 2 * HRM_PERIOD);
    CHECK_EQ(state_of(s, 0), ANT_STATUS_SEARCHING);   /* infinite timeout: keeps searching */
    /* The learned id survives the drop to search. */
    uint16_t dev;
    CHECK(ant_mac_get_channel_id(&s->mac, 0, &dev, NULL, NULL));
    CHECK_EQ(dev, 0x1234);

    /* Master comes back at an unrelated phase: slave reacquires. */
    log_clear(s);
    CHECK_EQ(ant_mac_open_channel(&m->mac, 0), ANT_RESPONSE_NO_ERROR);
    sim_run(3 * ANT_TICKS_PER_SEC);
    CHECK_EQ(state_of(s, 0), ANT_STATUS_TRACKING);
    CHECK(count_data(s, 0, ANT_MSG_BROADCAST_DATA) >= 8);
    return 0;
}

static int test_search_timeout_closes_channel(void)
{
    sim_init(1, 777);
    node_t *s = &S.node[0];
    CHECK_EQ(setup_channel(s, 0, ANT_CHANNEL_TYPE_SLAVE_RX, 0, 0, 0, 0, HRM_PERIOD, 1), 0);
    CHECK_EQ(ant_mac_open_channel(&s->mac, 0), ANT_RESPONSE_NO_ERROR);
    uint32_t t_open = S.now;
    sim_run(4 * ANT_TICKS_PER_SEC);
    const ev_rec_t *to = first_event(s, 0, ANT_EVENT_RX_SEARCH_TIMEOUT);
    CHECK(to != NULL);
    CHECK_EQ(to->t - t_open, ANT_MAC_SEARCH_UNIT_TICKS);      /* timeout 1 = 2.5 s */
    CHECK(event_seq_ok(s, 0, ANT_EVENT_RX_SEARCH_TIMEOUT, ANT_EVENT_CHANNEL_CLOSED));
    CHECK_EQ(state_of(s, 0), ANT_STATUS_ASSIGNED);
    /* Receiver is off afterwards. */
    CHECK_EQ(ant_air_radio(s->phy)->rx_on, 0);
    /* And the channel can be reopened. */
    CHECK_EQ(ant_mac_open_channel(&s->mac, 0), ANT_RESPONSE_NO_ERROR);
    CHECK_EQ(state_of(s, 0), ANT_STATUS_SEARCHING);
    return 0;
}

static int test_search_timeout_zero_closes_immediately(void)
{
    sim_init(1, 0);
    node_t *s = &S.node[0];
    CHECK_EQ(setup_channel(s, 0, ANT_CHANNEL_TYPE_SLAVE_RX, 0, 0, 0, 0, HRM_PERIOD, 0), 0);
    CHECK_EQ(ant_mac_open_channel(&s->mac, 0), ANT_RESPONSE_NO_ERROR);
    sim_run(ANT_TICKS_PER_SEC / 10);
    CHECK_EQ(count_events(s, 0, ANT_EVENT_RX_SEARCH_TIMEOUT), 1);
    CHECK_EQ(state_of(s, 0), ANT_STATUS_ASSIGNED);
    return 0;
}

static int test_acknowledged_master_to_slave(void)
{
    pair_hrm(0, ANT_CHANNEL_TYPE_SLAVE_RX, 0xFF);
    node_t *m = &S.node[0], *s = &S.node[1];
    sim_run(2 * ANT_TICKS_PER_SEC);
    log_clear(m); log_clear(s);

    uint8_t ack[8] = { 0x46, 0xFF, 0xFF, 0xFF, 0x80, 0x01, 0x50, 0x01 }; /* request page */
    CHECK_EQ(ant_mac_send_acknowledged(&m->mac, 0, ack), ANT_RESPONSE_NO_ERROR);
    CHECK_EQ(ant_mac_send_acknowledged(&m->mac, 0, ack), ANT_RESPONSE_TRANSFER_IN_PROGRESS);
    sim_run(ANT_TICKS_PER_SEC);

    CHECK_EQ(count_events(m, 0, ANT_EVENT_TRANSFER_TX_COMPLETED), 1);
    CHECK_EQ(count_events(m, 0, ANT_EVENT_TRANSFER_TX_FAILED), 0);
    CHECK_EQ(count_data(s, 0, ANT_MSG_ACKNOWLEDGED_DATA), 1);
    int i = 0;
    while (i < s->n_rx && s->rx[i].type != ANT_MSG_ACKNOWLEDGED_DATA) i++;
    CHECK(memcmp(s->rx[i].data, ack, 8) == 0);
    /* Normal broadcasts resume in the following slots, and the slave stayed
     * on the grid throughout. */
    CHECK(count_data(s, 0, ANT_MSG_BROADCAST_DATA) >= 2);
    CHECK_EQ(count_events(s, 0, ANT_EVENT_RX_FAIL), 0);
    CHECK_EQ(state_of(s, 0), ANT_STATUS_TRACKING);
    return 0;
}

static int test_acknowledged_slave_to_master(void)
{
    pair_hrm(0, ANT_CHANNEL_TYPE_SLAVE_RX, 0xFF);
    node_t *m = &S.node[0], *s = &S.node[1];
    sim_run(2 * ANT_TICKS_PER_SEC);
    log_clear(m); log_clear(s);

    uint8_t ack[8] = { 0x46, 0xFF, 0xFF, 0xFF, 0x80, 0x02, 0x50, 0x01 };
    CHECK_EQ(ant_mac_send_acknowledged(&s->mac, 0, ack), ANT_RESPONSE_NO_ERROR);
    sim_run(ANT_TICKS_PER_SEC);

    CHECK_EQ(count_events(s, 0, ANT_EVENT_TRANSFER_TX_COMPLETED), 1);
    CHECK_EQ(count_data(m, 0, ANT_MSG_ACKNOWLEDGED_DATA), 1);
    CHECK(memcmp(m->rx[0].data, ack, 8) == 0);
    /* The master's own schedule is undisturbed. */
    CHECK_EQ(count_events(m, 0, ANT_EVENT_CHANNEL_COLLISION), 0);
    CHECK_EQ(count_events(s, 0, ANT_EVENT_RX_FAIL), 0);
    return 0;
}

static int test_reverse_broadcast_slave_to_master(void)
{
    pair_hrm(0, ANT_CHANNEL_TYPE_SLAVE_RX, 0xFF);
    node_t *m = &S.node[0], *s = &S.node[1];
    sim_run(2 * ANT_TICKS_PER_SEC);
    log_clear(m); log_clear(s);
    uint8_t rb[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    CHECK_EQ(ant_mac_send_broadcast(&s->mac, 0, rb), ANT_RESPONSE_NO_ERROR);
    sim_run(ANT_TICKS_PER_SEC);
    /* One-shot: exactly one reverse broadcast reaches the master. */
    CHECK_EQ(count_data(m, 0, ANT_MSG_BROADCAST_DATA), 1);
    CHECK(memcmp(m->rx[0].data, rb, 8) == 0);
    return 0;
}

static int test_acknowledged_fails_without_peer(void)
{
    sim_init(1, 0);
    node_t *m = &S.node[0];
    setup_channel(m, 0, ANT_CHANNEL_TYPE_MASTER_TX, 0, 0x1234, HRM_TYPE, 1, HRM_PERIOD, 0);
    uint8_t page[8]; hrm_page(60, 0, page);
    ant_mac_send_broadcast(&m->mac, 0, page);
    ant_mac_open_channel(&m->mac, 0);
    uint8_t ack[8] = { 0 };
    CHECK_EQ(ant_mac_send_acknowledged(&m->mac, 0, ack), ANT_RESPONSE_NO_ERROR);
    sim_run(ANT_TICKS_PER_SEC);
    CHECK_EQ(count_events(m, 0, ANT_EVENT_TRANSFER_TX_FAILED), 1);
    CHECK_EQ(count_events(m, 0, ANT_EVENT_TRANSFER_TX_COMPLETED), 0);
    /* The channel carries on broadcasting afterwards. */
    CHECK(count_events(m, 0, ANT_EVENT_TX) >= 3);
    return 0;
}

static int test_acknowledged_lost_on_air_fails(void)
{
    pair_hrm(0, ANT_CHANNEL_TYPE_SLAVE_RX, 0xFF);
    node_t *m = &S.node[0], *s = &S.node[1];
    sim_run(2 * ANT_TICKS_PER_SEC);
    log_clear(m); log_clear(s);
    S.air.drop_every_n = 1;              /* nothing gets through */
    uint8_t ack[8] = { 9, 9, 9, 9, 9, 9, 9, 9 };
    CHECK_EQ(ant_mac_send_acknowledged(&m->mac, 0, ack), ANT_RESPONSE_NO_ERROR);
    sim_run(ANT_TICKS_PER_SEC / 2);
    CHECK_EQ(count_events(m, 0, ANT_EVENT_TRANSFER_TX_FAILED), 1);
    CHECK_EQ(count_data(s, 0, ANT_MSG_ACKNOWLEDGED_DATA), 0);
    return 0;
}

static int check_burst_sequence(const node_t *rx, int n_packets, const uint8_t *payload)
{
    /* ANT sequence: 0 first, then 1,2,3,1,2,3,...; +4 on the last packet. */
    int k = 0;
    uint8_t expect = 0;
    for (int i = 0; i < rx->n_rx; i++) {
        if (rx->rx[i].type != ANT_MSG_BURST_DATA) continue;
        uint8_t want = expect;
        if (k == n_packets - 1) want |= 4;
        CHECK_EQ(rx->rx[i].seq, want);
        CHECK(memcmp(rx->rx[i].data, payload + 8 * k, 8) == 0);
        expect = (expect == 0 || expect == 3) ? 1 : (uint8_t)(expect + 1);
        k++;
    }
    CHECK_EQ(k, n_packets);
    return 0;
}

static int test_burst_master_to_slave(void)
{
    pair_hrm(0, ANT_CHANNEL_TYPE_SLAVE_RX, 0xFF);
    node_t *m = &S.node[0], *s = &S.node[1];
    sim_run(2 * ANT_TICKS_PER_SEC);
    log_clear(m); log_clear(s);

    uint8_t burst[40];
    for (int i = 0; i < 40; i++) burst[i] = (uint8_t)(0xB0 + i);
    CHECK_EQ(ant_mac_send_burst(&m->mac, 0, burst, sizeof(burst)), ANT_RESPONSE_NO_ERROR);
    uint32_t t0 = S.now;
    sim_run(ANT_TICKS_PER_SEC);

    CHECK_EQ(count_events(m, 0, ANT_EVENT_TRANSFER_TX_COMPLETED), 1);
    CHECK_EQ(count_events(s, 0, ANT_EVENT_TRANSFER_RX_FAILED), 0);
    CHECK_EQ(count_data(s, 0, ANT_MSG_BURST_DATA), 5);
    if (check_burst_sequence(s, 5, burst)) return -1;
    /* The whole 5-packet burst goes back-to-back inside one slot, not one
     * packet per channel period. */
    const ev_rec_t *done = first_event(m, 0, ANT_EVENT_TRANSFER_TX_COMPLETED);
    CHECK(done->t - t0 < 2 * HRM_PERIOD);
    /* Broadcasting resumes and the slave is still locked. */
    CHECK_EQ(state_of(s, 0), ANT_STATUS_TRACKING);
    CHECK_EQ(count_events(s, 0, ANT_EVENT_RX_FAIL), 0);
    return 0;
}

static int test_burst_slave_to_master(void)
{
    pair_hrm(0, ANT_CHANNEL_TYPE_SLAVE_RX, 0xFF);
    node_t *m = &S.node[0], *s = &S.node[1];
    sim_run(2 * ANT_TICKS_PER_SEC);
    log_clear(m); log_clear(s);

    uint8_t burst[ANT_MAC_BURST_MAX_BYTES];
    for (int i = 0; i < (int)sizeof(burst); i++) burst[i] = (uint8_t)(i * 7);
    CHECK_EQ(ant_mac_send_burst(&s->mac, 0, burst, sizeof(burst)), ANT_RESPONSE_NO_ERROR);
    sim_run(ANT_TICKS_PER_SEC);

    CHECK_EQ(count_events(s, 0, ANT_EVENT_TRANSFER_TX_COMPLETED), 1);
    CHECK_EQ(count_data(m, 0, ANT_MSG_BURST_DATA), 16);
    if (check_burst_sequence(m, 16, burst)) return -1;
    CHECK_EQ(count_events(m, 0, ANT_EVENT_CHANNEL_COLLISION), 0);
    CHECK_EQ(count_events(s, 0, ANT_EVENT_RX_FAIL), 0);
    return 0;
}

static int test_burst_short_is_padded(void)
{
    pair_hrm(0, ANT_CHANNEL_TYPE_SLAVE_RX, 0xFF);
    node_t *m = &S.node[0], *s = &S.node[1];
    sim_run(2 * ANT_TICKS_PER_SEC);
    log_clear(m); log_clear(s);
    uint8_t burst[11] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };
    uint8_t padded[16] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 0, 0, 0, 0 };
    CHECK_EQ(ant_mac_send_burst(&m->mac, 0, burst, sizeof(burst)), ANT_RESPONSE_NO_ERROR);
    sim_run(ANT_TICKS_PER_SEC);
    CHECK_EQ(count_data(s, 0, ANT_MSG_BURST_DATA), 2);
    if (check_burst_sequence(s, 2, padded)) return -1;
    return 0;
}

static int test_burst_survives_lost_packets(void)
{
    pair_hrm(0, ANT_CHANNEL_TYPE_SLAVE_RX, 0xFF);
    node_t *m = &S.node[0], *s = &S.node[1];
    sim_run(2 * ANT_TICKS_PER_SEC);
    log_clear(m); log_clear(s);

    uint8_t burst[64];
    for (int i = 0; i < 64; i++) burst[i] = (uint8_t)(0x40 + i);
    S.air.drop_every_n = 3;     /* lose every 3rd frame: data packets AND acks */
    CHECK_EQ(ant_mac_send_burst(&m->mac, 0, burst, sizeof(burst)), ANT_RESPONSE_NO_ERROR);
    sim_run(ANT_TICKS_PER_SEC);
    S.air.drop_every_n = 0;

    CHECK_EQ(count_events(m, 0, ANT_EVENT_TRANSFER_TX_COMPLETED), 1);
    CHECK_EQ(count_events(m, 0, ANT_EVENT_TRANSFER_TX_FAILED), 0);
    CHECK_EQ(count_events(s, 0, ANT_EVENT_TRANSFER_RX_FAILED), 0);
    /* Exactly the 8 packets, in order, no duplicates delivered to the app
     * even though the air lost acks and packets were retransmitted. */
    CHECK_EQ(count_data(s, 0, ANT_MSG_BURST_DATA), 8);
    if (check_burst_sequence(s, 8, burst)) return -1;
    CHECK(S.air.dropped > 0);
    return 0;
}

static int test_burst_corrupted_packets_retransmitted(void)
{
    pair_hrm(0, ANT_CHANNEL_TYPE_SLAVE_RX, 0xFF);
    node_t *m = &S.node[0], *s = &S.node[1];
    sim_run(2 * ANT_TICKS_PER_SEC);
    log_clear(m); log_clear(s);
    uint8_t burst[32];
    for (int i = 0; i < 32; i++) burst[i] = (uint8_t)(0xC0 ^ i);
    /* Every 3rd frame on the air is corrupted (a burst exchange is 2 frames,
     * so packets and acks are hit alternately). */
    S.air.corrupt_every_n = 3;
    CHECK_EQ(ant_mac_send_burst(&s->mac, 0, burst, sizeof(burst)), ANT_RESPONSE_NO_ERROR);
    sim_run(ANT_TICKS_PER_SEC);
    S.air.corrupt_every_n = 0;
    CHECK_EQ(count_events(s, 0, ANT_EVENT_TRANSFER_TX_COMPLETED), 1);
    CHECK_EQ(count_data(m, 0, ANT_MSG_BURST_DATA), 4);
    if (check_burst_sequence(m, 4, burst)) return -1;
    CHECK(S.air.corrupted > 0);
    /* More frames than the ideal 4 packets + 4 acks went out: retries. */
    CHECK(ant_air_radio(s->phy)->tx_count > 4);
    return 0;
}

static int test_burst_fails_when_air_is_dead(void)
{
    pair_hrm(0, ANT_CHANNEL_TYPE_SLAVE_RX, 0xFF);
    node_t *m = &S.node[0], *s = &S.node[1];
    sim_run(2 * ANT_TICKS_PER_SEC);
    log_clear(m); log_clear(s);
    uint8_t burst[24] = { 0 };
    S.air.drop_every_n = 1;
    CHECK_EQ(ant_mac_send_burst(&m->mac, 0, burst, sizeof(burst)), ANT_RESPONSE_NO_ERROR);
    sim_run(ANT_TICKS_PER_SEC);
    CHECK_EQ(count_events(m, 0, ANT_EVENT_TRANSFER_TX_FAILED), 1);
    CHECK_EQ(count_events(m, 0, ANT_EVENT_TRANSFER_TX_COMPLETED), 0);
    /* Retries were made: more than one frame left the master for the burst
     * beyond its regular broadcasts... (one first packet + up to 5 retries). */
    CHECK(ant_air_radio(m->phy)->tx_count > (uint32_t)count_events(m, 0, ANT_EVENT_TX) + 1);
    /* A new transfer is accepted afterwards. */
    S.air.drop_every_n = 0;
    CHECK_EQ(ant_mac_send_burst(&m->mac, 0, burst, sizeof(burst)), ANT_RESPONSE_NO_ERROR);
    return 0;
}

static int test_burst_interrupted_reports_rx_failed(void)
{
    pair_hrm(0, ANT_CHANNEL_TYPE_SLAVE_RX, 0xFF);
    node_t *m = &S.node[0], *s = &S.node[1];
    sim_run(2 * ANT_TICKS_PER_SEC);
    log_clear(m); log_clear(s);
    uint8_t burst[48] = { 0 };
    CHECK_EQ(ant_mac_send_burst(&m->mac, 0, burst, sizeof(burst)), ANT_RESPONSE_NO_ERROR);
    /* Let the first packet(s) through, then kill the master mid-burst. */
    while (count_data(s, 0, ANT_MSG_BURST_DATA) == 0) sim_run(ANT_MAC_POLL_INTERVAL);
    CHECK(count_data(s, 0, ANT_MSG_BURST_DATA) >= 1);
    CHECK(count_data(s, 0, ANT_MSG_BURST_DATA) < 6);
    ant_mac_close_channel(&m->mac, 0);
    CHECK_EQ(count_events(m, 0, ANT_EVENT_TRANSFER_TX_FAILED), 1);
    sim_run(ANT_TICKS_PER_SEC / 4);
    CHECK_EQ(count_events(s, 0, ANT_EVENT_TRANSFER_RX_FAILED), 1);
    return 0;
}

static int test_rx_only_slave_never_transmits(void)
{
    pair_hrm(0, ANT_CHANNEL_TYPE_SLAVE_RX_ONLY, 0xFF);
    node_t *m = &S.node[0], *s = &S.node[1];
    sim_run(2 * ANT_TICKS_PER_SEC);
    CHECK_EQ(state_of(s, 0), ANT_STATUS_TRACKING);
    uint8_t d[8] = { 0 };
    CHECK_EQ(ant_mac_send_acknowledged(&s->mac, 0, d), ANT_RESPONSE_CHANNEL_IN_WRONG_STATE);
    CHECK_EQ(ant_mac_send_burst(&s->mac, 0, d, 8), ANT_RESPONSE_CHANNEL_IN_WRONG_STATE);
    CHECK_EQ(ant_mac_send_broadcast(&s->mac, 0, d), ANT_RESPONSE_CHANNEL_IN_WRONG_STATE);
    /* A master's acknowledged send to it fails: nobody answers. */
    log_clear(m);
    CHECK_EQ(ant_mac_send_acknowledged(&m->mac, 0, d), ANT_RESPONSE_NO_ERROR);
    sim_run(ANT_TICKS_PER_SEC);
    CHECK_EQ(count_events(m, 0, ANT_EVENT_TRANSFER_TX_FAILED), 1);
    CHECK_EQ(ant_air_radio(s->phy)->tx_count, 0);
    return 0;
}

static int test_network_key_separates_nodes(void)
{
    /* Master on the ANT+ network, slave with no key: must never pair. */
    sim_init(2, 0);
    node_t *m = &S.node[0], *s = &S.node[1];
    ant_mac_set_network_key(&m->mac, 0, ANTPLUS_KEY);
    ant_mac_set_network_key(&s->mac, 0, ZERO_KEY);
    setup_channel(m, 0, ANT_CHANNEL_TYPE_MASTER_TX, 0, 0x1234, HRM_TYPE, 1, HRM_PERIOD, 0);
    setup_channel(s, 0, ANT_CHANNEL_TYPE_SLAVE_RX, 0, 0, 0, 0, HRM_PERIOD, 0xFF);
    uint8_t page[8]; hrm_page(72, 0, page);
    ant_mac_send_broadcast(&m->mac, 0, page);
    ant_mac_open_channel(&m->mac, 0);
    ant_mac_open_channel(&s->mac, 0);
    sim_run(5 * ANT_TICKS_PER_SEC);
    CHECK_EQ(state_of(s, 0), ANT_STATUS_SEARCHING);
    CHECK_EQ(s->rx_total, 0);
    CHECK(count_events(m, 0, ANT_EVENT_TX) > 15);

    /* Now give the slave the key (on network 1, used by its channel): pairs. */
    ant_mac_close_channel(&s->mac, 0);
    ant_mac_unassign_channel(&s->mac, 0);
    ant_mac_set_network_key(&s->mac, 1, ANTPLUS_KEY);
    setup_channel(s, 0, ANT_CHANNEL_TYPE_SLAVE_RX, 1, 0, 0, 0, HRM_PERIOD, 0xFF);
    ant_mac_open_channel(&s->mac, 0);
    sim_run(3 * ANT_TICKS_PER_SEC);
    CHECK_EQ(state_of(s, 0), ANT_STATUS_TRACKING);
    CHECK(s->rx_total > 5);
    return 0;
}

static int test_rf_frequency_separates_nodes(void)
{
    sim_init(2, 0);
    node_t *m = &S.node[0], *s = &S.node[1];
    setup_channel(m, 0, ANT_CHANNEL_TYPE_MASTER_TX, 0, 0x1234, HRM_TYPE, 1, HRM_PERIOD, 0);
    setup_channel(s, 0, ANT_CHANNEL_TYPE_SLAVE_RX, 0, 0, 0, 0, HRM_PERIOD, 0xFF);
    ant_mac_set_channel_rf_freq(&s->mac, 0, 66);       /* slave on 2466, master on 2457 */
    uint8_t page[8]; hrm_page(72, 0, page);
    ant_mac_send_broadcast(&m->mac, 0, page);
    ant_mac_open_channel(&m->mac, 0);
    ant_mac_open_channel(&s->mac, 0);
    sim_run(3 * ANT_TICKS_PER_SEC);
    CHECK_EQ(state_of(s, 0), ANT_STATUS_SEARCHING);
    CHECK_EQ(s->rx_total, 0);
    return 0;
}

static int test_multi_channel_mixed_roles(void)
{
    /* Node A: ch0 HRM master (0x0A0A), ch1 power slave (wildcard).
     * Node B: ch0 power master (0x0B0B), ch1 HRM slave for 0x0A0A.
     * Each node runs a master and a slave on one radio at the same time. */
    sim_init(2, 0);
    node_t *a = &S.node[0], *b = &S.node[1];
    ant_mac_set_network_key(&a->mac, 0, ANTPLUS_KEY);
    ant_mac_set_network_key(&b->mac, 0, ANTPLUS_KEY);
    CHECK_EQ(setup_channel(a, 0, ANT_CHANNEL_TYPE_MASTER_TX, 0, 0x0A0A, HRM_TYPE, 1, HRM_PERIOD, 0), 0);
    CHECK_EQ(setup_channel(a, 1, ANT_CHANNEL_TYPE_SLAVE_RX,  0, 0, PWR_TYPE, 0, PWR_PERIOD, 0xFF), 0);
    CHECK_EQ(setup_channel(b, 0, ANT_CHANNEL_TYPE_MASTER_TX, 0, 0x0B0B, PWR_TYPE, 5, PWR_PERIOD, 0), 0);
    CHECK_EQ(setup_channel(b, 1, ANT_CHANNEL_TYPE_SLAVE_RX,  0, 0x0A0A, HRM_TYPE, 1, HRM_PERIOD, 0xFF), 0);

    uint8_t hp[8]; hrm_page(99, 0, hp);
    antplus_power_data_t pd; memset(&pd, 0, sizeof(pd));
    pd.instantaneous_power = 250; pd.pedal_power = 0xFF; pd.instantaneous_cadence = 90;
    uint8_t pp[8]; antplus_power_encode(&pd, pp);
    ant_mac_send_broadcast(&a->mac, 0, hp);
    ant_mac_send_broadcast(&b->mac, 0, pp);
    ant_mac_open_channel(&a->mac, 0);
    ant_mac_open_channel(&a->mac, 1);
    S.now += 1234; S.air.now = S.now;
    ant_mac_open_channel(&b->mac, 0);
    ant_mac_open_channel(&b->mac, 1);

    sim_run(6 * ANT_TICKS_PER_SEC);

    CHECK_EQ(state_of(a, 1), ANT_STATUS_TRACKING);
    CHECK_EQ(state_of(b, 1), ANT_STATUS_TRACKING);
    uint16_t dev; uint8_t dt, tt;
    CHECK(ant_mac_get_channel_id(&a->mac, 1, &dev, &dt, &tt));
    CHECK_EQ(dev, 0x0B0B); CHECK_EQ(dt, PWR_TYPE); CHECK_EQ(tt, 5);

    int a_rx = count_data(a, 1, ANT_MSG_BROADCAST_DATA);
    int b_rx = count_data(b, 1, ANT_MSG_BROADCAST_DATA);
    CHECK(a_rx >= 15);
    CHECK(b_rx >= 15);
    /* Payloads are the right profile on the right channel. */
    for (int i = 0; i < a->n_rx; i++) {
        CHECK_EQ(a->rx[i].ch, 1);
        antplus_power_data_t p;
        CHECK(antplus_power_decode(a->rx[i].data, &p));
        CHECK_EQ(p.instantaneous_power, 250);
    }
    for (int i = 0; i < b->n_rx; i++) {
        CHECK_EQ(b->rx[i].ch, 1);
        antplus_hrm_data_t h;
        CHECK(antplus_hrm_decode(b->rx[i].data, &h));
        CHECK_EQ(h.computed_heart_rate, 99);
    }
    /* Both masters keep their grid: the periods are coprime so the two slots
     * do overlap now and then; those are the only misses allowed. */
    int a_tx = count_events(a, 0, ANT_EVENT_TX), b_tx = count_events(b, 0, ANT_EVENT_TX);
    CHECK(a_tx >= 22 && b_tx >= 22);
    CHECK(count_events(a, 1, ANT_EVENT_RX_FAIL) <= 3);
    CHECK(count_events(b, 1, ANT_EVENT_RX_FAIL) <= 3);

    /* Acknowledged data on both channels of both nodes, concurrently. */
    log_clear(a); log_clear(b);
    uint8_t x[8] = { 0xAA, 1, 2, 3, 4, 5, 6, 7 }, y[8] = { 0xBB, 1, 2, 3, 4, 5, 6, 7 };
    CHECK_EQ(ant_mac_send_acknowledged(&a->mac, 0, x), ANT_RESPONSE_NO_ERROR);
    CHECK_EQ(ant_mac_send_acknowledged(&b->mac, 0, y), ANT_RESPONSE_NO_ERROR);
    sim_run(ANT_TICKS_PER_SEC);
    CHECK_EQ(count_events(a, 0, ANT_EVENT_TRANSFER_TX_COMPLETED), 1);
    CHECK_EQ(count_events(b, 0, ANT_EVENT_TRANSFER_TX_COMPLETED), 1);
    CHECK_EQ(count_data(b, 1, ANT_MSG_ACKNOWLEDGED_DATA), 1);
    CHECK_EQ(count_data(a, 1, ANT_MSG_ACKNOWLEDGED_DATA), 1);
    return 0;
}

static int test_two_slaves_one_master_in_one_node(void)
{
    /* A display tracking two sensors (HRM + power) from two ESP32 sensors. */
    sim_init(3, 0);
    node_t *hrm = &S.node[0], *pwr = &S.node[1], *disp = &S.node[2];
    setup_channel(hrm, 0, ANT_CHANNEL_TYPE_MASTER_TX, 0, 0x1001, HRM_TYPE, 1, HRM_PERIOD, 0);
    setup_channel(pwr, 0, ANT_CHANNEL_TYPE_MASTER_TX, 0, 0x2002, PWR_TYPE, 5, PWR_PERIOD, 0);
    setup_channel(disp, 0, ANT_CHANNEL_TYPE_SLAVE_RX, 0, 0, HRM_TYPE, 0, HRM_PERIOD, 0xFF);
    setup_channel(disp, 1, ANT_CHANNEL_TYPE_SLAVE_RX, 0, 0, PWR_TYPE, 0, PWR_PERIOD, 0xFF);
    uint8_t hp[8]; hrm_page(120, 0, hp);
    uint8_t pp[8] = { 0x10, 1, 0xFF, 90, 0, 0, 0x2C, 0x01 };
    ant_mac_send_broadcast(&hrm->mac, 0, hp);
    ant_mac_send_broadcast(&pwr->mac, 0, pp);
    ant_mac_open_channel(&hrm->mac, 0);
    S.now += 4000; S.air.now = S.now;
    ant_mac_open_channel(&pwr->mac, 0);
    ant_mac_open_channel(&disp->mac, 0);
    ant_mac_open_channel(&disp->mac, 1);
    sim_run(6 * ANT_TICKS_PER_SEC);
    CHECK_EQ(state_of(disp, 0), ANT_STATUS_TRACKING);
    CHECK_EQ(state_of(disp, 1), ANT_STATUS_TRACKING);
    uint16_t dev;
    ant_mac_get_channel_id(&disp->mac, 0, &dev, NULL, NULL); CHECK_EQ(dev, 0x1001);
    ant_mac_get_channel_id(&disp->mac, 1, &dev, NULL, NULL); CHECK_EQ(dev, 0x2002);
    CHECK(count_data(disp, 0, ANT_MSG_BROADCAST_DATA) >= 15);
    CHECK(count_data(disp, 1, ANT_MSG_BROADCAST_DATA) >= 15);
    /* Wildcard-by-type: the HRM channel never got a power page and vice versa. */
    for (int i = 0; i < disp->n_rx; i++) {
        CHECK_EQ(disp->rx[i].data[0] & 0x7F, disp->rx[i].ch == 0 ? 0x00 : 0x10);
    }
    return 0;
}

static int test_pairing_bit_is_transparent_and_selective(void)
{
    /* Master 0x1234 sets the pairing bit. A slave searching for plain HRM
     * still acquires it and reports the bit in the learned type. */
    sim_init(2, 0);
    node_t *m = &S.node[0], *s = &S.node[1];
    ant_mac_set_network_key(&m->mac, 0, ANTPLUS_KEY);
    ant_mac_set_network_key(&s->mac, 0, ANTPLUS_KEY);
    setup_channel(m, 0, ANT_CHANNEL_TYPE_MASTER_TX, 0, 0x1234,
                  HRM_TYPE | ANT_DEVICE_TYPE_PAIRING_BIT, 1, HRM_PERIOD, 0);
    setup_channel(s, 0, ANT_CHANNEL_TYPE_SLAVE_RX, 0, 0, HRM_TYPE, 0, HRM_PERIOD, 0xFF);
    ant_mac_open_channel(&m->mac, 0);
    ant_mac_open_channel(&s->mac, 0);
    sim_run(2 * ANT_TICKS_PER_SEC);
    CHECK_EQ(state_of(s, 0), ANT_STATUS_TRACKING);
    uint16_t dev; uint8_t dt, tt;
    CHECK(ant_mac_get_channel_id(&s->mac, 0, &dev, &dt, &tt));
    CHECK_EQ(dev, 0x1234);
    CHECK_EQ(dt, HRM_TYPE | ANT_DEVICE_TYPE_PAIRING_BIT);
    CHECK_EQ(ANT_DEVICE_TYPE_OF(dt), HRM_TYPE);

    /* A slave that asks for pairing masters only ignores a plain one... */
    sim_init(3, 0);
    node_t *plain = &S.node[0], *pairing = &S.node[1];
    s = &S.node[2];
    for (int i = 0; i < 3; i++) ant_mac_set_network_key(&S.node[i].mac, 0, ANTPLUS_KEY);
    setup_channel(plain, 0, ANT_CHANNEL_TYPE_MASTER_TX, 0, 0x1111, HRM_TYPE, 1, HRM_PERIOD, 0);
    setup_channel(pairing, 0, ANT_CHANNEL_TYPE_MASTER_TX, 0, 0x2222,
                  HRM_TYPE | ANT_DEVICE_TYPE_PAIRING_BIT, 1, HRM_PERIOD, 0);
    setup_channel(s, 0, ANT_CHANNEL_TYPE_SLAVE_RX, 0, 0,
                  HRM_TYPE | ANT_DEVICE_TYPE_PAIRING_BIT, 0, HRM_PERIOD, 0xFF);
    ant_mac_open_channel(&plain->mac, 0);
    ant_mac_open_channel(&s->mac, 0);
    sim_run(2 * ANT_TICKS_PER_SEC);
    CHECK_EQ(state_of(s, 0), ANT_STATUS_SEARCHING);
    CHECK_EQ(s->n_rx, 0);
    /* ... and takes the pairing one as soon as it appears. */
    S.now += 3000; S.air.now = S.now;
    ant_mac_open_channel(&pairing->mac, 0);
    sim_run(2 * ANT_TICKS_PER_SEC);
    CHECK_EQ(state_of(s, 0), ANT_STATUS_TRACKING);
    CHECK(ant_mac_get_channel_id(&s->mac, 0, &dev, &dt, &tt));
    CHECK_EQ(dev, 0x2222);
    return 0;
}

static int test_rssi_is_reported_per_channel(void)
{
    pair_hrm(0, ANT_CHANNEL_TYPE_SLAVE_RX, 0xFF);
    node_t *m = &S.node[0], *s = &S.node[1];
    ant_air_radio(m->phy)->tx_rssi = -63;
    sim_run(2 * ANT_TICKS_PER_SEC);
    CHECK_EQ(state_of(s, 0), ANT_STATUS_TRACKING);
    CHECK_EQ(ant_mac_channel(&s->mac, 0)->last_rssi, -63);
    return 0;
}

static int test_proximity_search_picks_the_near_master(void)
{
    /* Two HRM straps: a far one (-85 dBm) that transmits first, a near one
     * (-40 dBm). Without proximity search the slave adopts the far one; with
     * a -60 dBm threshold it waits for the near one. */
    for (int prox = 0; prox < 2; prox++) {
        sim_init(3, 0);
        node_t *far = &S.node[0], *near = &S.node[1], *s = &S.node[2];
        for (int i = 0; i < 3; i++) ant_mac_set_network_key(&S.node[i].mac, 0, ANTPLUS_KEY);
        setup_channel(far,  0, ANT_CHANNEL_TYPE_MASTER_TX, 0, 0x1111, HRM_TYPE, 1, HRM_PERIOD, 0);
        setup_channel(near, 0, ANT_CHANNEL_TYPE_MASTER_TX, 0, 0x2222, HRM_TYPE, 1, HRM_PERIOD, 0);
        setup_channel(s,    0, ANT_CHANNEL_TYPE_SLAVE_RX,  0, 0, HRM_TYPE, 0, HRM_PERIOD, 0xFF);
        ant_air_radio(far->phy)->tx_rssi  = -85;
        ant_air_radio(near->phy)->tx_rssi = -40;
        if (prox) CHECK_EQ(ant_mac_set_proximity_search(&s->mac, 0, -60), ANT_RESPONSE_NO_ERROR);
        ant_mac_open_channel(&far->mac, 0);
        ant_mac_open_channel(&s->mac, 0);
        S.now += 3000; S.air.now = S.now;     /* the far one is heard first */
        ant_mac_open_channel(&near->mac, 0);
        sim_run(3 * ANT_TICKS_PER_SEC);
        CHECK_EQ(state_of(s, 0), ANT_STATUS_TRACKING);
        uint16_t dev; uint8_t dt, tt;
        CHECK(ant_mac_get_channel_id(&s->mac, 0, &dev, &dt, &tt));
        CHECK_EQ(dev, prox ? 0x2222 : 0x1111);
        CHECK_EQ(ant_mac_channel(&s->mac, 0)->last_rssi, prox ? -40 : -85);

        if (prox) {
            /* Proximity applies to the first acquisition only: once paired,
             * the strap is re-acquired after a drop at any level. */
            ant_air_radio(near->phy)->tx_rssi = -90;
            ant_mac_close_channel(&near->mac, 0);
            sim_run(4 * ANT_TICKS_PER_SEC);
            CHECK_EQ(state_of(s, 0), ANT_STATUS_SEARCHING);
            ant_mac_open_channel(&near->mac, 0);
            sim_run(3 * ANT_TICKS_PER_SEC);
            CHECK_EQ(state_of(s, 0), ANT_STATUS_TRACKING);
            CHECK(ant_mac_get_channel_id(&s->mac, 0, &dev, &dt, &tt));
            CHECK_EQ(dev, 0x2222);
        }
    }
    return 0;
}

static int test_proximity_search_with_nobody_near_keeps_searching(void)
{
    pair_hrm(0, ANT_CHANNEL_TYPE_SLAVE_RX, 2);   /* 5 s search timeout */
    node_t *m = &S.node[0], *s = &S.node[1];
    ant_air_radio(m->phy)->tx_rssi = -80;
    ant_mac_close_channel(&s->mac, 0);
    ant_mac_set_proximity_search(&s->mac, 0, -60);
    ant_mac_open_channel(&s->mac, 0);
    sim_run(6 * ANT_TICKS_PER_SEC);
    CHECK_EQ(s->n_rx, 0);
    CHECK_EQ(count_events(s, 0, ANT_EVENT_RX_SEARCH_TIMEOUT), 1);
    return 0;
}

static int test_command_validation(void)
{
    sim_init(1, 0);
    ant_mac_t *m = &S.node[0].mac;
    uint8_t d[8] = { 0 };

    CHECK_EQ(ant_mac_assign_channel(m, 8, ANT_CHANNEL_TYPE_SLAVE_RX, 0), ANT_RESPONSE_INVALID_PARAMETER);
    CHECK_EQ(ant_mac_assign_channel(m, 0, ANT_CHANNEL_TYPE_SLAVE_RX, 3), ANT_RESPONSE_INVALID_NETWORK_NUMBER);
    CHECK_EQ(ant_mac_assign_channel(m, 0, ANT_CHANNEL_TYPE_SHARED_SLAVE, 0), ANT_RESPONSE_INVALID_PARAMETER);
    CHECK_EQ(ant_mac_set_network_key(m, 3, ANTPLUS_KEY), ANT_RESPONSE_INVALID_NETWORK_NUMBER);

    /* Unassigned channel: nothing else is valid. */
    CHECK_EQ(ant_mac_set_channel_id(m, 0, 1, 1, 1), ANT_RESPONSE_CHANNEL_IN_WRONG_STATE);
    CHECK_EQ(ant_mac_set_channel_period(m, 0, 8070), ANT_RESPONSE_CHANNEL_IN_WRONG_STATE);
    CHECK_EQ(ant_mac_open_channel(m, 0), ANT_RESPONSE_CHANNEL_IN_WRONG_STATE);
    CHECK_EQ(ant_mac_close_channel(m, 0), ANT_RESPONSE_CHANNEL_IN_WRONG_STATE);
    CHECK_EQ(ant_mac_send_broadcast(m, 0, d), ANT_RESPONSE_CHANNEL_IN_WRONG_STATE);
    CHECK_EQ(ant_mac_unassign_channel(m, 0), ANT_RESPONSE_CHANNEL_IN_WRONG_STATE);
    CHECK_EQ(ant_mac_channel_status(m, 0), ANT_STATUS_UNASSIGNED);

    CHECK_EQ(ant_mac_assign_channel(m, 0, ANT_CHANNEL_TYPE_MASTER_TX, 2), ANT_RESPONSE_NO_ERROR);
    CHECK_EQ(ant_mac_assign_channel(m, 0, ANT_CHANNEL_TYPE_MASTER_TX, 2), ANT_RESPONSE_CHANNEL_IN_WRONG_STATE);
    CHECK_EQ(ant_mac_channel_status(m, 0), ANT_STATUS_ASSIGNED | (2 << 2) | ANT_CHANNEL_TYPE_MASTER_TX);
    /* Defaults per the ANT spec. */
    const ant_mac_channel_t *c = ant_mac_channel(m, 0);
    CHECK_EQ(c->rf_freq, 66); CHECK_EQ(c->period, 8192); CHECK_EQ(c->search_timeout, 10);

    CHECK_EQ(ant_mac_open_channel(m, 0), ANT_RESPONSE_CHANNEL_ID_NOT_SET);
    CHECK_EQ(ant_mac_set_channel_id(m, 0, 0, HRM_TYPE, 1), ANT_RESPONSE_INVALID_PARAMETER); /* master needs a number */
    CHECK_EQ(ant_mac_set_channel_id(m, 0, 77, HRM_TYPE, 1), ANT_RESPONSE_NO_ERROR);
    CHECK_EQ(ant_mac_set_channel_rf_freq(m, 0, 125), ANT_RESPONSE_INVALID_PARAMETER);
    CHECK_EQ(ant_mac_set_channel_period(m, 0, 10), ANT_RESPONSE_INVALID_PARAMETER);
    CHECK_EQ(ant_mac_set_tx_power(m, 0, 5), ANT_RESPONSE_INVALID_PARAMETER);
    CHECK_EQ(ant_mac_set_tx_power(m, 0, 3), ANT_RESPONSE_NO_ERROR);
    CHECK_EQ(ant_mac_send_acknowledged(m, 0, d), ANT_RESPONSE_CHANNEL_NOT_OPENED);
    CHECK_EQ(ant_mac_send_burst(m, 0, d, 8), ANT_RESPONSE_CHANNEL_NOT_OPENED);

    CHECK_EQ(ant_mac_open_channel(m, 0), ANT_RESPONSE_NO_ERROR);
    CHECK_EQ(ant_mac_channel_status(m, 0) & ANT_STATUS_STATE_MASK, ANT_STATUS_TRACKING);
    CHECK_EQ(ant_mac_open_channel(m, 0), ANT_RESPONSE_CHANNEL_IN_WRONG_STATE);
    CHECK_EQ(ant_mac_set_channel_id(m, 0, 78, HRM_TYPE, 1), ANT_RESPONSE_CHANNEL_IN_WRONG_STATE);
    CHECK_EQ(ant_mac_unassign_channel(m, 0), ANT_RESPONSE_CHANNEL_IN_WRONG_STATE);
    CHECK_EQ(ant_mac_send_burst(m, 0, d, 0), ANT_RESPONSE_INVALID_PARAMETER);
    CHECK_EQ(ant_mac_send_burst(m, 0, d, 129), ANT_RESPONSE_INVALID_PARAMETER);
    CHECK_EQ(ant_mac_send_burst(m, 0, d, 8), ANT_RESPONSE_NO_ERROR);
    CHECK_EQ(ant_mac_send_acknowledged(m, 0, d), ANT_RESPONSE_TRANSFER_IN_PROGRESS);

    CHECK_EQ(ant_mac_close_channel(m, 0), ANT_RESPONSE_NO_ERROR);
    /* Closing with a transfer pending fails it. */
    CHECK_EQ(count_events(&S.node[0], 0, ANT_EVENT_TRANSFER_TX_FAILED), 1);
    CHECK_EQ(count_events(&S.node[0], 0, ANT_EVENT_CHANNEL_CLOSED), 1);
    CHECK_EQ(ant_mac_unassign_channel(m, 0), ANT_RESPONSE_NO_ERROR);
    CHECK_EQ(ant_mac_channel_status(m, 0), ANT_STATUS_UNASSIGNED);

    /* TX-only master rejects acknowledged data. */
    CHECK_EQ(ant_mac_assign_channel(m, 1, ANT_CHANNEL_TYPE_MASTER_TX_ONLY, 0), ANT_RESPONSE_NO_ERROR);
    CHECK_EQ(ant_mac_set_channel_id(m, 1, 5, 5, 5), ANT_RESPONSE_NO_ERROR);
    CHECK_EQ(ant_mac_open_channel(m, 1), ANT_RESPONSE_NO_ERROR);
    CHECK_EQ(ant_mac_send_acknowledged(m, 1, d), ANT_RESPONSE_CHANNEL_IN_WRONG_STATE);

    /* Reset clears the lot. */
    ant_mac_reset(m);
    CHECK_EQ(ant_mac_channel_status(m, 1), ANT_STATUS_UNASSIGNED);
    CHECK_EQ(ant_air_radio(S.node[0].phy)->rx_on, 0);
    return 0;
}

static int test_tick_wraps_around_32bit_time(void)
{
    /* Start 1.5 s before the 32-bit tick counter wraps; everything must sail
     * through the wrap without a hiccup. */
    uint32_t t0 = 0u - (3u * ANT_TICKS_PER_SEC / 2u);
    pair_hrm(t0, ANT_CHANNEL_TYPE_SLAVE_RX, 0xFF);
    node_t *m = &S.node[0], *s = &S.node[1];
    sim_run(4 * ANT_TICKS_PER_SEC);
    CHECK((int32_t)S.now > 0);                          /* we did wrap */
    CHECK_EQ(state_of(s, 0), ANT_STATUS_TRACKING);
    CHECK_EQ(count_events(s, 0, ANT_EVENT_RX_FAIL), 0);
    CHECK_EQ(count_events(m, 0, ANT_EVENT_CHANNEL_COLLISION), 0);
    int n_tx = count_events(m, 0, ANT_EVENT_TX);
    CHECK(n_tx >= 15);
    CHECK(count_data(s, 0, ANT_MSG_BROADCAST_DATA) >= n_tx - 1);
    return 0;
}

static int test_late_ticks_are_tolerated(void)
{
    /* A host that is late (ticks every 3 ms instead of at the deadline) still
     * keeps the link: the tracking window absorbs the lateness. */
    pair_hrm(0, ANT_CHANNEL_TYPE_SLAVE_RX, 0xFF);
    node_t *m = &S.node[0], *s = &S.node[1];
    for (uint32_t t = 0; t < 5 * ANT_TICKS_PER_SEC; t += 100) {
        S.now = t; S.air.now = t;
        ant_mac_tick(&m->mac, t);
        ant_mac_tick(&s->mac, t);
    }
    CHECK_EQ(state_of(s, 0), ANT_STATUS_TRACKING);
    int n_tx = count_events(m, 0, ANT_EVENT_TX);
    CHECK(n_tx >= 19);
    CHECK(count_data(s, 0, ANT_MSG_BROADCAST_DATA) >= n_tx - 2);
    CHECK_EQ(count_events(s, 0, ANT_EVENT_RX_FAIL_GO_TO_SEARCH), 0);
    return 0;
}

void run_mac_tests(void)
{
    RUN(test_master_schedule_is_exact);
    RUN(test_master_without_page_broadcasts_zeros);
    RUN(test_wildcard_slave_acquires_and_learns_id);
    RUN(test_slave_receive_is_on_the_master_grid);
    RUN(test_specific_id_slave_ignores_other_master);
    RUN(test_dropped_frames_give_rx_fail_but_keep_tracking);
    RUN(test_corrupted_frames_are_rejected);
    RUN(test_master_gone_drops_to_search_and_reacquires);
    RUN(test_search_timeout_closes_channel);
    RUN(test_search_timeout_zero_closes_immediately);
    RUN(test_acknowledged_master_to_slave);
    RUN(test_acknowledged_slave_to_master);
    RUN(test_reverse_broadcast_slave_to_master);
    RUN(test_acknowledged_fails_without_peer);
    RUN(test_acknowledged_lost_on_air_fails);
    RUN(test_burst_master_to_slave);
    RUN(test_burst_slave_to_master);
    RUN(test_burst_short_is_padded);
    RUN(test_burst_survives_lost_packets);
    RUN(test_burst_corrupted_packets_retransmitted);
    RUN(test_burst_fails_when_air_is_dead);
    RUN(test_burst_interrupted_reports_rx_failed);
    RUN(test_rx_only_slave_never_transmits);
    RUN(test_network_key_separates_nodes);
    RUN(test_rf_frequency_separates_nodes);
    RUN(test_multi_channel_mixed_roles);
    RUN(test_two_slaves_one_master_in_one_node);
    RUN(test_pairing_bit_is_transparent_and_selective);
    RUN(test_rssi_is_reported_per_channel);
    RUN(test_proximity_search_picks_the_near_master);
    RUN(test_proximity_search_with_nobody_near_keeps_searching);
    RUN(test_command_validation);
    RUN(test_tick_wraps_around_32bit_time);
    RUN(test_late_ticks_are_tolerated);
}
