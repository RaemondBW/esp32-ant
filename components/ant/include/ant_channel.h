/*
 * ant_channel.h - ANT channel state machine and stack driver.
 *
 * Sits on top of ant_message (framing) and ant_radio (transport). It:
 *   - runs the standard bring-up sequence for a channel (reset, network key,
 *     assign, channel id, RF freq, period, open) as a non-blocking state
 *     machine driven by ant_stack_step();
 *   - parses inbound frames and routes broadcast/ack data to a user callback;
 *   - lets a master channel push broadcast pages out.
 *
 * The design is intentionally poll-driven (no threads, no blocking) so it runs
 * identically in a host test loop and in an ESP32 FreeRTOS task.
 */
#ifndef ANT_CHANNEL_H
#define ANT_CHANNEL_H

#include "ant_message.h"
#include "ant_radio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The ANT+ managed network key (public, published in the ANT+ docs). */
extern const uint8_t ANTPLUS_NETWORK_KEY[8];
#define ANTPLUS_RF_FREQ   57u   /* 2457 MHz */
#define ANTPLUS_NETWORK   0u    /* network number used for the ANT+ key */

/* Wildcards for open search on a slave channel. */
#define ANT_WILDCARD_DEVICE_NUM   0x0000u
#define ANT_WILDCARD_DEVICE_TYPE  0x00u
#define ANT_WILDCARD_TRANS_TYPE   0x00u

typedef enum {
    ANT_CH_IDLE = 0,
    ANT_CH_RESETTING,
    ANT_CH_SET_KEY,
    ANT_CH_ASSIGN,
    ANT_CH_SET_ID,
    ANT_CH_SET_FREQ,
    ANT_CH_SET_PERIOD,
    ANT_CH_SET_TIMEOUT,
    ANT_CH_OPEN,
    ANT_CH_RUNNING,
    ANT_CH_CLOSED,
    ANT_CH_ERROR
} ant_channel_state_t;

typedef struct ant_stack ant_stack_t;

/* Callback for received data pages (broadcast or acknowledged). */
typedef void (*ant_data_cb_t)(ant_stack_t *st, uint8_t channel,
                              const uint8_t payload[8], void *user);
/* Callback for raw channel events / responses (optional, may be NULL). */
typedef void (*ant_event_cb_t)(ant_stack_t *st, uint8_t channel,
                               uint8_t msg_id, uint8_t code, void *user);

typedef struct {
    uint8_t  channel;      /* channel number */
    uint8_t  type;         /* ANT_CHANNEL_TYPE_* */
    uint16_t device_num;   /* 0 = wildcard (search) */
    uint8_t  device_type;  /* ANT+ device profile type, 0 = wildcard */
    uint8_t  trans_type;   /* transmission type, 0 = wildcard */
    uint8_t  rf_freq;      /* usually ANTPLUS_RF_FREQ */
    uint16_t period;       /* channel period in 1/32768 s counts */
    uint8_t  search_timeout; /* in 2.5s units, 0xFF = infinite, 0 = disable */
    const uint8_t *network_key; /* 8 bytes; defaults to ANT+ key if NULL */
    uint8_t  network;      /* network number */
} ant_channel_config_t;

struct ant_stack {
    ant_radio_t        *radio;
    ant_channel_config_t cfg;
    ant_channel_state_t  state;
    ant_parser_t         parser;
    bool                 awaiting_response; /* waiting for cmd ack */
    uint8_t              last_cmd_msg_id;
    uint32_t             tx_count;          /* broadcasts sent (master) */
    uint32_t             rx_count;          /* data pages received */
    ant_data_cb_t        on_data;
    ant_event_cb_t       on_event;
    void                *user;
    /* Master mode: payload the stack rebroadcasts on each TX event. */
    uint8_t              tx_payload[8];
    bool                 has_tx_payload;
};

/* Initialize the stack over a radio transport with the given config. */
void ant_stack_init(ant_stack_t *st, ant_radio_t *radio,
                    const ant_channel_config_t *cfg);

void ant_stack_set_callbacks(ant_stack_t *st, ant_data_cb_t on_data,
                             ant_event_cb_t on_event, void *user);

/* Begin bring-up: transitions IDLE -> RESETTING and sends the reset. */
void ant_stack_start(ant_stack_t *st);

/*
 * Advance the stack: drains the radio, parses frames, advances the config
 * state machine, and dispatches data. Call frequently (e.g. every few ms).
 * Returns the number of frames processed this step.
 */
int ant_stack_step(ant_stack_t *st);

/* Feed a single response/event message into the state machine (used by step,
 * exposed for testing). */
void ant_stack_handle_message(ant_stack_t *st, const ant_message_t *msg);

/* Master mode: set the 8-byte page rebroadcast on each channel period. */
void ant_stack_set_tx_payload(ant_stack_t *st, const uint8_t payload[8]);

/* Send one broadcast page immediately (master). Returns true on success. */
bool ant_stack_send_broadcast(ant_stack_t *st, const uint8_t payload[8]);

bool ant_stack_is_running(const ant_stack_t *st);
const char *ant_channel_state_name(ant_channel_state_t s);

/* Helper: fill a config with ANT+ slave (receiver) defaults for a profile. */
void ant_channel_config_antplus_slave(ant_channel_config_t *cfg, uint8_t channel,
                                       uint8_t device_type, uint16_t period);
/* Helper: fill a config with ANT+ master (sensor) defaults for a profile. */
void ant_channel_config_antplus_master(ant_channel_config_t *cfg, uint8_t channel,
                                        uint16_t device_num, uint8_t device_type,
                                        uint8_t trans_type, uint16_t period);

#ifdef __cplusplus
}
#endif

#endif /* ANT_CHANNEL_H */
