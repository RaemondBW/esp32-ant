/* Tests for the ANT channel state machine and end-to-end stack behavior. */
#include "test.h"
#include "ant_channel.h"
#include "ant_radio_loopback.h"
#include "antplus_profiles.h"

/* Pump the stack + simulated chip until running or a step budget is hit. */
static void bring_up(ant_stack_t *st, ant_loopback_t *lb, int max_steps)
{
    (void)lb;
    ant_stack_start(st);
    for (int i = 0; i < max_steps && !ant_stack_is_running(st); i++) {
        ant_stack_step(st);
    }
}

static int test_slave_bringup_reaches_running(void)
{
    ant_loopback_t lb;
    ant_loopback_init(&lb, NULL, NULL);
    ant_channel_config_t cfg;
    ant_channel_config_antplus_slave(&cfg, 0, ANTPLUS_DEVTYPE_HRM,
                                     ANTPLUS_PERIOD_HRM);
    ant_stack_t st;
    ant_stack_init(&st, ant_loopback_radio(&lb), &cfg);

    bring_up(&st, &lb, 50);
    CHECK(ant_stack_is_running(&st));
    /* The chip should have seen the full config command sequence. */
    CHECK(lb.cmd_count >= 7);
    CHECK(lb.opened);
    return 0;
}

static int test_master_bringup_skips_search_timeout(void)
{
    ant_loopback_t lb;
    ant_loopback_init(&lb, NULL, NULL);
    ant_channel_config_t cfg;
    ant_channel_config_antplus_master(&cfg, 0, 0x1234,
                                      ANTPLUS_DEVTYPE_HRM, 1,
                                      ANTPLUS_PERIOD_HRM);
    ant_stack_t st;
    ant_stack_init(&st, ant_loopback_radio(&lb), &cfg);
    bring_up(&st, &lb, 50);
    CHECK(ant_stack_is_running(&st));
    return 0;
}

static int test_config_error_goes_to_error_state(void)
{
    ant_loopback_t lb;
    ant_loopback_init(&lb, NULL, NULL);
    ant_channel_config_t cfg;
    ant_channel_config_antplus_slave(&cfg, 0, ANTPLUS_DEVTYPE_HRM,
                                     ANTPLUS_PERIOD_HRM);
    ant_stack_t st;
    ant_stack_init(&st, ant_loopback_radio(&lb), &cfg);
    ant_stack_start(&st);
    ant_stack_step(&st); /* consume startup, send set-key */
    /* Make the chip reject the NEXT command (set network key). */
    lb.fail_next_command = true;
    for (int i = 0; i < 20 && ant_stack_is_running(&st) == false; i++) {
        ant_stack_step(&st);
        if (st.state == ANT_CH_ERROR) break;
    }
    CHECK_EQ(st.state, ANT_CH_ERROR);
    CHECK(!ant_stack_is_running(&st));
    return 0;
}

/* Data-reception path: feed HRM pages via the simulated chip. */

struct hrm_capture {
    int count;
    uint8_t last_hr;
    uint16_t last_event;
};

static void on_hrm_data(ant_stack_t *st, uint8_t ch, const uint8_t payload[8],
                        void *user)
{
    (void)st; (void)ch;
    struct hrm_capture *cap = (struct hrm_capture *)user;
    antplus_hrm_data_t d;
    if (antplus_hrm_decode(payload, &d)) {
        cap->count++;
        cap->last_hr = d.computed_heart_rate;
        cap->last_event = d.heart_beat_event_time;
    }
}

/* Page generator: produces an HRM page with an increasing HR. */
struct hrm_source { uint8_t hr; uint16_t event; };
static bool gen_hrm_page(uint8_t out[8], void *user)
{
    struct hrm_source *src = (struct hrm_source *)user;
    antplus_hrm_data_t in = {0};
    src->event += 1024;      /* one beat per second-ish */
    in.heart_beat_event_time = src->event;
    in.heart_beat_count++;
    in.computed_heart_rate = src->hr++;
    antplus_hrm_encode_page0(&in, (src->hr & 4) != 0, out);
    return true;
}

static int test_slave_receives_hrm_pages(void)
{
    struct hrm_source src = { .hr = 70, .event = 0 };
    ant_loopback_t lb;
    ant_loopback_init(&lb, gen_hrm_page, &src);
    ant_channel_config_t cfg;
    ant_channel_config_antplus_slave(&cfg, 0, ANTPLUS_DEVTYPE_HRM,
                                     ANTPLUS_PERIOD_HRM);
    ant_stack_t st;
    ant_stack_init(&st, ant_loopback_radio(&lb), &cfg);
    struct hrm_capture cap = {0};
    ant_stack_set_callbacks(&st, on_hrm_data, NULL, &cap);

    bring_up(&st, &lb, 50);
    CHECK(ant_stack_is_running(&st));

    /* Simulate 10 radio periods; each should deliver one page. */
    for (int i = 0; i < 10; i++) {
        ant_loopback_tick(&lb);
        ant_stack_step(&st);
    }
    CHECK_EQ(cap.count, 10);
    CHECK_EQ(cap.last_hr, 79);        /* 70..79 */
    CHECK_EQ(st.rx_count, 10);
    return 0;
}

static int test_slave_tolerates_dropped_pages(void)
{
    struct hrm_source src = { .hr = 100, .event = 0 };
    ant_loopback_t lb;
    ant_loopback_init(&lb, gen_hrm_page, &src);
    lb.drop_every_n_rx = 3; /* drop every 3rd page */
    ant_channel_config_t cfg;
    ant_channel_config_antplus_slave(&cfg, 0, ANTPLUS_DEVTYPE_HRM,
                                     ANTPLUS_PERIOD_HRM);
    ant_stack_t st;
    ant_stack_init(&st, ant_loopback_radio(&lb), &cfg);
    struct hrm_capture cap = {0};
    ant_stack_set_callbacks(&st, on_hrm_data, NULL, &cap);
    bring_up(&st, &lb, 50);

    for (int i = 0; i < 9; i++) {
        ant_loopback_tick(&lb);
        ant_stack_step(&st);
    }
    /* 9 ticks, every 3rd dropped -> 6 delivered */
    CHECK_EQ(cap.count, 6);
    return 0;
}

/* Master transmit path: verify broadcasts are emitted on TX events. */
static int test_master_transmits_on_tx_event(void)
{
    ant_loopback_t lb;
    ant_loopback_init(&lb, NULL, NULL);
    ant_channel_config_t cfg;
    ant_channel_config_antplus_master(&cfg, 0, 0xABCD,
                                      ANTPLUS_DEVTYPE_HRM, 1,
                                      ANTPLUS_PERIOD_HRM);
    ant_stack_t st;
    ant_stack_init(&st, ant_loopback_radio(&lb), &cfg);
    bring_up(&st, &lb, 50);
    CHECK(ant_stack_is_running(&st));

    uint8_t page[8] = { 0x80, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 1, 65 };
    ant_stack_set_tx_payload(&st, page);
    CHECK(ant_stack_send_broadcast(&st, page));
    CHECK_EQ(st.tx_count, 1);
    return 0;
}

void run_channel_tests(void)
{
    RUN(test_slave_bringup_reaches_running);
    RUN(test_master_bringup_skips_search_timeout);
    RUN(test_config_error_goes_to_error_state);
    RUN(test_slave_receives_hrm_pages);
    RUN(test_slave_tolerates_dropped_pages);
    RUN(test_master_transmits_on_tx_event);
}
