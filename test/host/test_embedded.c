/*
 * Tests for ant_embedded: the ANT serial network-processor front end on top
 * of ant_mac. Two "ESP32s", each running the unchanged ant_stack host code
 * over ant_radio_embedded(), talk over the virtual air.
 */
#include "test.h"
#include "ant_embedded.h"
#include "ant_channel.h"
#include "ant_air.h"
#include "antplus_profiles.h"
#include "ant_selftest.h"

/* ------------------------------ harness ---------------------------------- */

#define EMB_NODES 2

typedef struct {
    ant_embedded_t emb;
    ant_stack_t    st;
    uint32_t       deadline;
    /* captured host-side traffic */
    int      n_data;
    uint8_t  last_data[8];
    int      n_burst;                  /* burst packets seen by the host */
    uint8_t  burst_ch_byte[32];
    uint8_t  burst_body[ANT_MAC_BURST_MAX_BYTES];
    int      n_events;
    uint8_t  events[64];
    int      n_resp;                   /* command responses (resp_id != 1) */
    uint8_t  resp_id[64], resp_code[64];
    /* raw frames captured through a side parser (for request replies) */
    ant_parser_t   side;
    int      n_raw;
    ant_message_t  raw[32];
} emb_node_t;

static struct {
    ant_air_t  air;
    emb_node_t node[EMB_NODES];
} E;

static void on_data(ant_stack_t *st, uint8_t ch, const uint8_t payload[8], void *user)
{
    (void)st; (void)ch;
    emb_node_t *n = user;
    n->n_data++;
    memcpy(n->last_data, payload, 8);
}

static void on_event(ant_stack_t *st, uint8_t ch, uint8_t msg_id, uint8_t code, void *user)
{
    (void)st; (void)ch;
    emb_node_t *n = user;
    if (msg_id == 0x01) {
        if (n->n_events < 64) n->events[n->n_events] = code;
        n->n_events++;
    } else {
        if (n->n_resp < 64) { n->resp_id[n->n_resp] = msg_id; n->resp_code[n->n_resp] = code; }
        n->n_resp++;
    }
}

static void emb_init(void)
{
    memset(&E, 0, sizeof(E));
    ant_air_init(&E.air);
    for (int i = 0; i < EMB_NODES; i++) {
        ant_radio_embedded_init(&E.node[i].emb, ant_air_attach(&E.air));
        ant_parser_init(&E.node[i].side);
    }
}

static void emb_stack(int i, const ant_channel_config_t *cfg)
{
    emb_node_t *n = &E.node[i];
    ant_stack_init(&n->st, ant_radio_embedded(&n->emb), cfg);
    ant_stack_set_callbacks(&n->st, on_data, on_event, n);
}

/* Read everything the "chip" has for the host, feeding ant_stack and also a
 * side parser so tests can look at messages ant_stack itself ignores. */
static void emb_service(emb_node_t *n)
{
    uint8_t buf[256];
    int k;
    while ((k = ant_radio_read(ant_radio_embedded(&n->emb), buf, sizeof(buf))) > 0) {
        for (int i = 0; i < k; i++) {
            ant_message_t m;
            if (ant_parser_push(&n->st.parser, buf[i], &m)) ant_stack_handle_message(&n->st, &m);
            if (ant_parser_push(&n->side, buf[i], &m)) {
                if (m.msg_id == ANT_MSG_BURST_DATA && m.data_len >= 9) {
                    if (n->n_burst < 32) n->burst_ch_byte[n->n_burst] = m.data[0];
                    if ((n->n_burst + 1) * 8 <= (int)ANT_MAC_BURST_MAX_BYTES)
                        memcpy(&n->burst_body[n->n_burst * 8], &m.data[1], 8);
                    n->n_burst++;
                }
                if (n->n_raw < 32) n->raw[n->n_raw] = m;
                n->n_raw++;
            }
        }
    }
}

/* Run the two nodes for `ticks`, event-driven at each node's MAC deadline. */
static void emb_run(uint32_t ticks)
{
    uint32_t end = E.air.now + ticks;
    for (int i = 0; i < EMB_NODES; i++) {
        E.node[i].deadline = ant_embedded_tick(&E.node[i].emb, E.air.now);
        emb_service(&E.node[i]);
    }
    for (;;) {
        uint32_t next = end;
        for (int i = 0; i < EMB_NODES; i++)
            if ((int32_t)(E.node[i].deadline - next) < 0) next = E.node[i].deadline;
        if ((int32_t)(next - end) >= 0) break;
        E.air.now = next;
        for (int i = 0; i < EMB_NODES; i++) {
            if ((int32_t)(E.node[i].deadline - E.air.now) <= 0) {
                E.node[i].deadline = ant_embedded_tick(&E.node[i].emb, E.air.now);
                emb_service(&E.node[i]);
            }
        }
    }
    E.air.now = end;
}

/* Bring a stack up: it only needs serial round trips, no air time. */
static bool emb_bring_up(emb_node_t *n)
{
    ant_stack_start(&n->st);
    for (int i = 0; i < 50 && !ant_stack_is_running(&n->st); i++) emb_service(n);
    return ant_stack_is_running(&n->st);
}

static void emb_clear(emb_node_t *n)
{
    n->n_data = n->n_burst = n->n_events = n->n_resp = n->n_raw = 0;
}

static void hrm_pair(void)
{
    ant_channel_config_t cfg;
    emb_init();
    ant_channel_config_antplus_master(&cfg, 0, 0x1234, ANTPLUS_DEVTYPE_HRM, 1, ANTPLUS_PERIOD_HRM);
    emb_stack(0, &cfg);
    ant_channel_config_antplus_slave(&cfg, 0, ANTPLUS_DEVTYPE_HRM, ANTPLUS_PERIOD_HRM);
    emb_stack(1, &cfg);
}

static const ant_message_t *find_raw(emb_node_t *n, uint8_t msg_id)
{
    for (int i = 0; i < n->n_raw && i < 32; i++)
        if (n->raw[i].msg_id == msg_id) return &n->raw[i];
    return NULL;
}

static void send_raw(emb_node_t *n, uint8_t msg_id, const uint8_t *d, uint8_t len)
{
    uint8_t f[ANT_MAX_FRAME_LEN];
    size_t k = ant_message_encode(msg_id, d, len, f, sizeof(f));
    ant_radio_write(ant_radio_embedded(&n->emb), f, k);
}

/* -------------------------------- tests ---------------------------------- */

static int test_bringup_reaches_running_both_roles(void)
{
    hrm_pair();
    CHECK(emb_bring_up(&E.node[0]));
    CHECK(emb_bring_up(&E.node[1]));
    /* every config command got a NO_ERROR response */
    for (int i = 0; i < E.node[0].n_resp; i++) CHECK_EQ(E.node[0].resp_code[i], ANT_RESPONSE_NO_ERROR);
    for (int i = 0; i < E.node[1].n_resp; i++) CHECK_EQ(E.node[1].resp_code[i], ANT_RESPONSE_NO_ERROR);
    /* master: reset,key,assign,id,freq,period,open = 6 responses (reset -> STARTUP) */
    CHECK_EQ(E.node[0].n_resp, 6);
    CHECK_EQ(E.node[1].n_resp, 7);   /* + search timeout */
    /* MAC state agrees */
    CHECK_EQ(ant_mac_channel_status(&E.node[0].emb.mac, 0) & 0x03, ANT_MAC_CH_TRACKING);
    CHECK_EQ(ant_mac_channel_status(&E.node[1].emb.mac, 0) & 0x03, ANT_MAC_CH_SEARCHING);
    return 0;
}

static int test_hrm_pages_flow_master_to_slave(void)
{
    hrm_pair();
    CHECK(emb_bring_up(&E.node[0]));
    CHECK(emb_bring_up(&E.node[1]));

    antplus_hrm_data_t hr = {0};
    hr.computed_heart_rate = 77; hr.heart_beat_count = 3; hr.heart_beat_event_time = 1000;
    uint8_t page[8];
    antplus_hrm_encode_page0(&hr, false, page);
    ant_stack_set_tx_payload(&E.node[0].st, page);
    ant_stack_send_broadcast(&E.node[0].st, page);

    emb_run(4 * ANT_TICKS_PER_SEC);

    /* ~4 s at 4.06 Hz -> 16 pages; allow the acquisition slot */
    CHECK(E.node[1].n_data >= 14);
    CHECK(E.node[1].n_data <= 17);
    antplus_hrm_data_t got;
    CHECK(antplus_hrm_decode(E.node[1].last_data, &got));
    CHECK_EQ(got.computed_heart_rate, 77);
    /* master got an EVENT_TX for every transmission and reloaded each time */
    int tx = 0;
    for (int i = 0; i < E.node[0].n_events && i < 64; i++) if (E.node[0].events[i] == ANT_EVENT_TX) tx++;
    CHECK(tx >= 15);
    CHECK_EQ(E.node[0].st.tx_count, (uint32_t)tx + 1);
    /* slave learned the master's id */
    uint16_t dev; uint8_t dt, tt;
    ant_mac_get_channel_id(&E.node[1].emb.mac, 0, &dev, &dt, &tt);
    CHECK_EQ(dev, 0x1234); CHECK_EQ(dt, ANTPLUS_DEVTYPE_HRM); CHECK_EQ(tt, 1);
    CHECK_EQ(E.node[1].st.rx_count, (uint32_t)E.node[1].n_data);
    return 0;
}

static int test_page_updates_are_picked_up(void)
{
    hrm_pair();
    CHECK(emb_bring_up(&E.node[0]));
    CHECK(emb_bring_up(&E.node[1]));
    uint8_t page[8] = { 0, 0, 0, 0, 0, 0, 0, 60 };
    ant_stack_set_tx_payload(&E.node[0].st, page);
    emb_run(2 * ANT_TICKS_PER_SEC);
    CHECK_EQ(E.node[1].last_data[7], 60);
    page[7] = 61;
    ant_stack_set_tx_payload(&E.node[0].st, page);
    emb_run(ANT_TICKS_PER_SEC);
    CHECK_EQ(E.node[1].last_data[7], 61);
    return 0;
}

static int test_requests_answer_status_id_caps_version(void)
{
    hrm_pair();
    CHECK(emb_bring_up(&E.node[0]));
    emb_node_t *n = &E.node[0];
    emb_clear(n);

    uint8_t req[2] = { 0, ANT_MSG_CHANNEL_STATUS };
    send_raw(n, ANT_MSG_REQUEST, req, 2);
    req[1] = ANT_MSG_CHANNEL_ID;      send_raw(n, ANT_MSG_REQUEST, req, 2);
    req[1] = ANT_MSG_CAPABILITIES;    send_raw(n, ANT_MSG_REQUEST, req, 2);
    req[1] = ANT_MSG_VERSION;         send_raw(n, ANT_MSG_REQUEST, req, 2);
    req[1] = 0x77;                    send_raw(n, ANT_MSG_REQUEST, req, 2);
    emb_service(n);

    const ant_message_t *m = find_raw(n, ANT_MSG_CHANNEL_STATUS);
    CHECK(m != NULL);
    CHECK_EQ(m->data[0], 0);
    CHECK_EQ(m->data[1] & 0x03, ANT_MAC_CH_TRACKING);
    CHECK_EQ(m->data[1] & 0xF0, ANT_CHANNEL_TYPE_MASTER_TX & 0xF0);

    m = find_raw(n, ANT_MSG_CHANNEL_ID);
    CHECK(m != NULL);
    CHECK_EQ(m->data_len, 5);
    CHECK_EQ(m->data[1] | (m->data[2] << 8), 0x1234);
    CHECK_EQ(m->data[3], ANTPLUS_DEVTYPE_HRM);
    CHECK_EQ(m->data[4], 1);

    m = find_raw(n, ANT_MSG_CAPABILITIES);
    CHECK(m != NULL);
    CHECK_EQ(m->data[0], ANT_MAC_MAX_CHANNELS);
    CHECK_EQ(m->data[1], ANT_MAC_MAX_NETWORKS);

    m = find_raw(n, ANT_MSG_VERSION);
    CHECK(m != NULL);
    CHECK(strncmp((const char *)m->data, "ESP32-ANT", 9) == 0);

    /* unknown request id -> INVALID_MESSAGE response to REQUEST */
    CHECK_EQ(n->n_resp, 1);
    CHECK_EQ(n->resp_id[0], ANT_MSG_REQUEST);
    CHECK_EQ(n->resp_code[0], ANT_RESPONSE_INVALID_MESSAGE);
    return 0;
}

static int test_host_burst_is_assembled_and_delivered(void)
{
    hrm_pair();
    CHECK(emb_bring_up(&E.node[0]));
    CHECK(emb_bring_up(&E.node[1]));
    emb_run(2 * ANT_TICKS_PER_SEC);             /* let the slave acquire */
    emb_clear(&E.node[0]); emb_clear(&E.node[1]);

    /* 3 host packets: seq 0, 1, 2|LAST (ANT serial burst sequencing) */
    uint8_t pk[9];
    static const uint8_t seqs[3] = { 0x00, 0x20, 0x40 | 0x80 };
    for (int p = 0; p < 3; p++) {
        pk[0] = (uint8_t)(seqs[p] | 0);
        for (int i = 0; i < 8; i++) pk[1 + i] = (uint8_t)(p * 8 + i + 1);
        send_raw(&E.node[0], ANT_MSG_BURST_DATA, pk, 9);
    }
    emb_service(&E.node[0]);
    CHECK_EQ(E.node[0].n_resp, 0);              /* accepted silently */

    emb_run(ANT_TICKS_PER_SEC);

    CHECK_EQ(E.node[1].n_burst, 3);
    /* channel bytes carry the air sequence: 0, 1, 2|LAST */
    CHECK_EQ(E.node[1].burst_ch_byte[0], 0x00);
    CHECK_EQ(E.node[1].burst_ch_byte[1], 0x20);
    CHECK_EQ(E.node[1].burst_ch_byte[2], 0x40 | 0x80);
    for (int i = 0; i < 24; i++) CHECK_EQ(E.node[1].burst_body[i], i + 1);
    /* master host sees TRANSFER_TX_COMPLETED */
    int done = 0;
    for (int i = 0; i < E.node[0].n_events && i < 64; i++)
        if (E.node[0].events[i] == ANT_EVENT_TRANSFER_TX_COMPLETED) done++;
    CHECK_EQ(done, 1);
    return 0;
}

static int test_host_burst_sequence_errors(void)
{
    hrm_pair();
    CHECK(emb_bring_up(&E.node[0]));
    emb_node_t *n = &E.node[0];
    emb_clear(n);
    uint8_t pk[9] = {0};

    /* first packet must be seq 0 */
    pk[0] = 0x20;
    send_raw(n, ANT_MSG_BURST_DATA, pk, 9);
    emb_service(n);
    CHECK_EQ(n->n_resp, 1);
    CHECK_EQ(n->resp_id[0], ANT_MSG_BURST_DATA);
    CHECK_EQ(n->resp_code[0], ANT_RESPONSE_TRANSFER_SEQUENCE_ERROR);

    /* seq 0 then seq 3 (skipped 1) */
    emb_clear(n);
    pk[0] = 0x00; send_raw(n, ANT_MSG_BURST_DATA, pk, 9);
    pk[0] = 0x60; send_raw(n, ANT_MSG_BURST_DATA, pk, 9);
    emb_service(n);
    CHECK_EQ(n->n_resp, 1);
    CHECK_EQ(n->resp_code[0], ANT_RESPONSE_TRANSFER_SEQUENCE_ERROR);
    CHECK(!n->emb.hburst[0].active);

    /* sequence 0,1,2,3,1,2,3,1... up to 16 packets is fine; a 17th overflows */
    emb_clear(n);
    uint8_t seq = 0;
    for (int p = 0; p < 17; p++) {
        pk[0] = (uint8_t)(seq << 5);
        send_raw(n, ANT_MSG_BURST_DATA, pk, 9);
        seq = (seq == 0 || seq == 3) ? 1 : (uint8_t)(seq + 1);
    }
    emb_service(n);
    CHECK_EQ(n->n_resp, 1);
    CHECK_EQ(n->resp_code[0], ANT_RESPONSE_INVALID_PARAMETER);
    return 0;
}

static int test_acknowledged_data_over_serial(void)
{
    hrm_pair();
    CHECK(emb_bring_up(&E.node[0]));
    CHECK(emb_bring_up(&E.node[1]));
    emb_run(2 * ANT_TICKS_PER_SEC);
    emb_clear(&E.node[0]); emb_clear(&E.node[1]);

    uint8_t pk[9] = { 0, 9, 8, 7, 6, 5, 4, 3, 2 };
    send_raw(&E.node[0], ANT_MSG_ACKNOWLEDGED_DATA, pk, 9);
    emb_run(ANT_TICKS_PER_SEC);

    const ant_message_t *m = find_raw(&E.node[1], ANT_MSG_ACKNOWLEDGED_DATA);
    CHECK(m != NULL);
    CHECK_EQ(m->data[1], 9); CHECK_EQ(m->data[8], 2);
    int done = 0;
    for (int i = 0; i < E.node[0].n_events && i < 64; i++)
        if (E.node[0].events[i] == ANT_EVENT_TRANSFER_TX_COMPLETED) done++;
    CHECK_EQ(done, 1);
    /* ant_stack delivers acknowledged pages through on_data too */
    CHECK(E.node[1].n_data >= 1);
    return 0;
}

static int test_invalid_and_short_messages(void)
{
    hrm_pair();
    emb_node_t *n = &E.node[0];
    emb_bring_up(n);
    emb_clear(n);

    uint8_t d[2] = { 0, 0 };
    send_raw(n, 0x7B, d, 2);                          /* unknown id */
    send_raw(n, ANT_MSG_SET_CHANNEL_ID, d, 2);        /* too short */
    send_raw(n, ANT_MSG_BROADCAST_DATA, d, 1);        /* too short */
    emb_service(n);
    CHECK_EQ(n->n_resp, 3);
    CHECK_EQ(n->resp_id[0], 0x7B);
    CHECK_EQ(n->resp_code[0], ANT_RESPONSE_INVALID_MESSAGE);
    CHECK_EQ(n->resp_id[1], ANT_MSG_SET_CHANNEL_ID);
    CHECK_EQ(n->resp_code[1], ANT_RESPONSE_INVALID_MESSAGE);
    CHECK_EQ(n->resp_id[2], ANT_MSG_BROADCAST_DATA);
    CHECK_EQ(n->resp_code[2], ANT_RESPONSE_INVALID_MESSAGE);

    /* a bad checksum on the wire is dropped by the parser, no response */
    emb_clear(n);
    uint8_t f[ANT_MAX_FRAME_LEN];
    size_t k = ant_message_encode(ANT_MSG_OPEN_CHANNEL, d, 1, f, sizeof(f));
    f[k - 1] ^= 0xFF;
    ant_radio_write(ant_radio_embedded(&n->emb), f, k);
    emb_service(n);
    CHECK_EQ(n->n_resp, 0);
    return 0;
}

static int test_reset_gives_startup_and_clears_everything(void)
{
    hrm_pair();
    CHECK(emb_bring_up(&E.node[0]));
    CHECK(emb_bring_up(&E.node[1]));
    emb_run(ANT_TICKS_PER_SEC);
    emb_node_t *n = &E.node[0];
    emb_clear(n);

    /* leave a half-assembled host burst behind, then reset */
    uint8_t pk[9] = {0};
    send_raw(n, ANT_MSG_BURST_DATA, pk, 9);
    send_raw(n, ANT_MSG_RESET_SYSTEM, NULL, 0);
    emb_service(n);
    CHECK(find_raw(n, ANT_MSG_STARTUP) != NULL);
    CHECK_EQ(find_raw(n, ANT_MSG_STARTUP)->data[0], 0x20);
    CHECK(!n->emb.hburst[0].active);
    CHECK_EQ(ant_mac_channel_status(&n->emb.mac, 0) & 0x03, ANT_MAC_CH_UNASSIGNED);

    /* the master is gone: the slave loses it and drops back to search */
    emb_clear(&E.node[1]);
    emb_run(4 * ANT_TICKS_PER_SEC);
    int gts = 0;
    for (int i = 0; i < E.node[1].n_events && i < 64; i++)
        if (E.node[1].events[i] == ANT_EVENT_RX_FAIL_GO_TO_SEARCH) gts++;
    CHECK_EQ(gts, 1);
    CHECK(ant_stack_is_running(&E.node[1].st));   /* infinite search: still up */

    /* bringing the master back up through ant_stack again reacquires */
    ant_channel_config_t cfg;
    ant_channel_config_antplus_master(&cfg, 0, 0x1234, ANTPLUS_DEVTYPE_HRM, 1, ANTPLUS_PERIOD_HRM);
    emb_stack(0, &cfg);
    CHECK(emb_bring_up(n));
    emb_clear(&E.node[1]);
    emb_run(3 * ANT_TICKS_PER_SEC);
    CHECK(E.node[1].n_data >= 8);
    return 0;
}

static int test_search_timeout_reaches_host_as_error(void)
{
    ant_channel_config_t cfg;
    emb_init();
    ant_channel_config_antplus_slave(&cfg, 0, ANTPLUS_DEVTYPE_HRM, ANTPLUS_PERIOD_HRM);
    cfg.search_timeout = 1;          /* 2.5 s, nobody transmitting */
    emb_stack(1, &cfg);
    CHECK(emb_bring_up(&E.node[1]));
    emb_run(3 * ANT_TICKS_PER_SEC);
    CHECK_EQ(E.node[1].st.state, ANT_CH_ERROR);
    int seen = 0;
    for (int i = 0; i < E.node[1].n_events && i < 64; i++) {
        if (E.node[1].events[i] == ANT_EVENT_RX_SEARCH_TIMEOUT) seen |= 1;
        if (E.node[1].events[i] == ANT_EVENT_CHANNEL_CLOSED) seen |= 2;
    }
    CHECK_EQ(seen, 3);
    return 0;
}

static int test_close_channel_over_serial(void)
{
    hrm_pair();
    CHECK(emb_bring_up(&E.node[0]));
    emb_node_t *n = &E.node[0];
    emb_clear(n);
    uint8_t d[1] = { 0 };
    send_raw(n, ANT_MSG_CLOSE_CHANNEL, d, 1);
    emb_service(n);
    CHECK_EQ(n->n_resp, 1);
    CHECK_EQ(n->resp_id[0], ANT_MSG_CLOSE_CHANNEL);
    CHECK_EQ(n->resp_code[0], ANT_RESPONSE_NO_ERROR);
    emb_run(ANT_MAC_FRAME_TICKS * 4);
    CHECK_EQ(n->st.state, ANT_CH_CLOSED);
    CHECK_EQ(ant_mac_channel_status(&n->emb.mac, 0) & 0x03, ANT_MAC_CH_ASSIGNED);
    /* nothing goes out on the air any more */
    uint32_t tx = ant_air_radio(n->emb.mac.phy)->tx_count;
    emb_run(ANT_TICKS_PER_SEC);
    CHECK_EQ(ant_air_radio(n->emb.mac.phy)->tx_count, tx);
    return 0;
}

static int test_output_fifo_overflow_is_counted(void)
{
    hrm_pair();
    CHECK(emb_bring_up(&E.node[0]));
    CHECK(emb_bring_up(&E.node[1]));
    /* nobody services node 1 for a long time: its FIFO fills with data msgs */
    uint32_t end = E.air.now + 40 * ANT_TICKS_PER_SEC;
    while ((int32_t)(E.air.now - end) < 0) {
        uint32_t next = end;
        for (int i = 0; i < EMB_NODES; i++) {
            uint32_t dl = ant_embedded_tick(&E.node[i].emb, E.air.now);
            if ((int32_t)(dl - next) < 0) next = dl;
        }
        E.air.now = next;
    }
    CHECK(E.node[1].emb.out_overflow > 0);
    CHECK(E.node[1].emb.out_count <= ANT_EMBEDDED_OUT_SIZE);
    /* once drained, delivery resumes */
    emb_service(&E.node[1]);
    CHECK_EQ(E.node[1].emb.out_count, 0);
    int before = E.node[1].n_data;
    emb_run(ANT_TICKS_PER_SEC);
    CHECK(E.node[1].n_data > before);
    return 0;
}

static int test_mixed_api_interop(void)
{
    /* Node 0 drives ant_mac directly, node 1 runs ant_stack over the serial
     * bridge: the bridge adds nothing the raw MAC does not do. */
    ant_channel_config_t cfg;
    emb_init();
    ant_mac_t *m = &E.node[0].emb.mac;
    CHECK_EQ(ant_mac_set_network_key(m, 0, ANTPLUS_NETWORK_KEY), ANT_RESPONSE_NO_ERROR);
    CHECK_EQ(ant_mac_assign_channel(m, 0, ANT_CHANNEL_TYPE_MASTER_TX, 0), ANT_RESPONSE_NO_ERROR);
    CHECK_EQ(ant_mac_set_channel_id(m, 0, 0x4321, ANTPLUS_DEVTYPE_BIKE_POWER, 5), ANT_RESPONSE_NO_ERROR);
    CHECK_EQ(ant_mac_set_channel_rf_freq(m, 0, ANTPLUS_RF_FREQ), ANT_RESPONSE_NO_ERROR);
    CHECK_EQ(ant_mac_set_channel_period(m, 0, ANTPLUS_PERIOD_BIKE_POWER), ANT_RESPONSE_NO_ERROR);
    CHECK_EQ(ant_mac_open_channel(m, 0), ANT_RESPONSE_NO_ERROR);
    uint8_t page[8] = { 0x10, 1, 2, 3, 4, 5, 6, 7 };
    ant_mac_send_broadcast(m, 0, page);

    ant_channel_config_antplus_slave(&cfg, 0, ANTPLUS_DEVTYPE_BIKE_POWER, ANTPLUS_PERIOD_BIKE_POWER);
    emb_stack(1, &cfg);
    CHECK(emb_bring_up(&E.node[1]));
    emb_run(3 * ANT_TICKS_PER_SEC);
    CHECK(E.node[1].n_data >= 8);
    CHECK_EQ(E.node[1].last_data[0], 0x10);
    CHECK_EQ(E.node[1].last_data[7], 7);
    return 0;
}

/* The boot-time self-test the ESP32 firmware runs is the same code. */
static int test_on_target_selftest_passes_on_host(void)
{
    ant_selftest_result_t r;
    ant_selftest_run(&r);
    CHECK(r.pass);
    CHECK(r.rx_pages >= 6);
    CHECK_EQ(r.learned_dev, 0x1234);
    CHECK_EQ(r.heart_rate, 72);
    CHECK_EQ(r.ack_rx, 1);
    CHECK_EQ(r.burst_pkts, 5);
    CHECK_EQ(r.tx_failed, 0);
    CHECK(r.air_delivered + 1 >= r.air_frames);   /* first page before the display listened */
    return 0;
}

void run_embedded_tests(void)
{
    RUN(test_bringup_reaches_running_both_roles);
    RUN(test_hrm_pages_flow_master_to_slave);
    RUN(test_page_updates_are_picked_up);
    RUN(test_requests_answer_status_id_caps_version);
    RUN(test_host_burst_is_assembled_and_delivered);
    RUN(test_host_burst_sequence_errors);
    RUN(test_acknowledged_data_over_serial);
    RUN(test_invalid_and_short_messages);
    RUN(test_reset_gives_startup_and_clears_everything);
    RUN(test_search_timeout_reaches_host_as_error);
    RUN(test_close_channel_over_serial);
    RUN(test_output_fifo_overflow_is_counted);
    RUN(test_mixed_api_interop);
    RUN(test_on_target_selftest_passes_on_host);
}
