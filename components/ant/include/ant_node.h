/*
 * ant_node.h - the ESP32 as a complete ANT node: its own radio (ant_espphy),
 * the ANT protocol engine (ant_mac) and the FreeRTOS task that ticks it on the
 * 32768 Hz ANT grid. This is the API an application (an Arduino sketch, a
 * PlatformIO project, an ESP-IDF app) uses; everything below it is portable
 * and host-tested.
 *
 *   static ant_node_t node;
 *   ant_node_start(&node, &(ant_node_config_t){ .on_data = on_page,
 *                                               .store = &ant_node_store_nvs });
 *   ant_node_open_antplus_slave(&node, 0, ANTPLUS_DEVTYPE_HRM, 0, ANTPLUS_PERIOD_HRM);
 *   ... on_page() is called from the ANT task with every received page ...
 *   ant_node_stop(&node);          // hands the radio back (e.g. to NimBLE)
 *
 * Pairing: ANT pairing is remembering a channel id. A slave channel opened
 * with device number 0 takes the first master of its type it hears; the node
 * remembers that device (one per device type, persisted through the
 * configured store) and, next time a channel of that type is opened with 0,
 * opens it with the remembered id instead - so the head unit reconnects to
 * its own strap and ignores the neighbour's. ant_node_pair() runs a fresh
 * search for a new device of a type (optionally proximity-limited), which
 * then replaces the remembered one; ant_node_forget_device() drops it.
 *
 * Threading: every call here is safe from any task. Callbacks run on the ANT
 * task while it holds the node lock, so they may call back into this API but
 * must not block (copy the page out, notify your own task).
 *
 * Radio: the ESP32-S3/C3 BLE controller is driven as a raw ANT modem, so it is
 * exclusive - a BLE host (NimBLE, Bluedroid) cannot be up at the same time.
 * Stop it before ant_node_start(), ant_node_stop() before restarting it. The
 * radio is also one-directional per start (see ant_espphy.h): a node with
 * slave channels receives, a node with a master channel transmits.
 */
#ifndef ANT_NODE_H
#define ANT_NODE_H

#include "ant_mac.h"
#include "ant_known.h"
#include "ant_espphy.h"
#include "ant_channel.h"        /* ANTPLUS_NETWORK_KEY, ANTPLUS_RF_FREQ */
#include "antplus_profiles.h"   /* ANTPLUS_DEVTYPE_* / ANTPLUS_PERIOD_* for the callers */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ant_node ant_node_t;

/* A paired master: device number / type (no pairing bit) / transmission type. */
typedef ant_known_device_t ant_node_device_t;

/* One received page and where it came from. `device_num/type/trans` is the
 * channel's identity (learned on wildcard acquisition; the type without its
 * pairing bit, which is reported in `pairing_request`). `rssi` is the
 * radio's reading for the frame (dBm, 0 if unknown). */
typedef struct {
    uint8_t  channel;
    uint16_t device_num;
    uint8_t  device_type;
    uint8_t  trans_type;
    uint8_t  msg_type;       /* ANT_MSG_BROADCAST_DATA / ACKNOWLEDGED_DATA / BURST_DATA */
    uint8_t  burst_seq;      /* burst: ANT sequence field, +4 marks the last packet */
    int8_t   rssi;
    bool     pairing_request;/* the master sets the pairing bit */
} ant_node_rx_t;

typedef void (*ant_node_data_cb_t)(ant_node_t *node, const ant_node_rx_t *rx,
                                   const uint8_t page[8], void *user);
/* ANT_EVENT_* (ant_message.h): RX_SEARCH_TIMEOUT, RX_FAIL, TX,
 * RX_FAIL_GO_TO_SEARCH, TRANSFER_TX_COMPLETED, ... */
typedef void (*ant_node_event_cb_t)(ant_node_t *node, uint8_t channel, uint8_t event, void *user);
/* A slave channel acquired a master (`remembered`: it was a fresh search and
 * the device is now the known one of its type). */
typedef void (*ant_node_paired_cb_t)(ant_node_t *node, uint8_t channel,
                                     const ant_node_device_t *dev, bool remembered, void *user);

/* Where the known-device table lives between runs. `load` fills `k` (return
 * false = nothing stored), `save` writes it. Both run on the caller's task
 * (`load` in ant_node_start, `save` on the ANT task when a device is paired
 * or forgotten). */
typedef struct ant_node_store {
    bool (*load)(const struct ant_node_store *s, ant_known_t *k);
    bool (*save)(const struct ant_node_store *s, const ant_known_t *k);
    void *user;
} ant_node_store_t;

/* Built-in stores: NVS (namespace "ant", key "known"; needs nvs_flash_init(),
 * which the Arduino core does at boot) and a process-lifetime RAM table
 * (survives ant_node_stop()/start(), e.g. BLE hand-offs, not a reboot). */
extern const ant_node_store_t ant_node_store_nvs;
extern const ant_node_store_t ant_node_store_ram;

typedef struct {
    ant_node_data_cb_t   on_data;
    ant_node_event_cb_t  on_event;    /* optional */
    ant_node_paired_cb_t on_paired;   /* optional */
    void                *user;
    const ant_node_store_t *store;    /* NULL = known devices live in the node only */
    int8_t               proximity_rssi; /* fresh searches acquire only masters heard at
                                            >= this dBm (0 = any). -70 is "on my bike". */
    uint32_t             task_stack;  /* 0 = 4096 bytes */
    uint8_t              task_priority;/* 0 = configMAX_PRIORITIES - 2 (it must win) */
    uint8_t              task_core;   /* 0 = default (core 1 on dual-core parts; the BT
                                         controller lives on core 0), or ANT_NODE_CORE_* */
    bool                 coexist;     /* shared-radio mode: a BLE host (NimBLE/Bluedroid)
                                         already owns the controller and is running a
                                         passive scan; ANT receives on that scan's radio
                                         windows instead of taking the controller. RX only
                                         (no ANT transmit). The host must be up and
                                         scanning before ant_node_start(). See
                                         ant_espphy_init_coexist(). */
} ant_node_config_t;

#define ANT_NODE_CORE_0    0x80u
#define ANT_NODE_CORE_1    0x81u
#define ANT_NODE_CORE_ANY  0xFFu

/* A channel to open. Zero fields take ANT+ defaults: rf_freq 57 (2457 MHz),
 * the ANT+ network key, search timeout infinite. On a slave, device_num 0
 * means the known device of `device_type` if there is one (auto-reconnect),
 * else any device of that type, which is then remembered. */
typedef struct {
    uint8_t        type;            /* ANT_CHANNEL_TYPE_SLAVE_RX, _MASTER_TX, _SLAVE_RX_ONLY, ... */
    uint16_t       device_num;
    uint8_t        device_type;
    uint8_t        trans_type;
    uint8_t        rf_freq;         /* 2400 + rf_freq MHz */
    uint16_t       period;          /* ticks (1/32768 s), e.g. ANTPLUS_PERIOD_HRM */
    uint8_t        search_timeout;  /* 2.5 s units, 0xFF infinite; 0 here = infinite */
    const uint8_t *network_key;     /* 8 bytes, NULL = ANT+ */
    uint8_t        network;         /* 0..2 */
    uint8_t        flags;           /* ANT_NODE_CH_* */
    int8_t         proximity_rssi;  /* fresh search threshold; 0 = the node's default */
} ant_node_channel_cfg_t;

#define ANT_NODE_CH_IGNORE_KNOWN  0x01u  /* device_num 0 searches afresh even if a device
                                            of that type is known (what ant_node_pair does) */
#define ANT_NODE_CH_NO_REMEMBER   0x02u  /* do not remember what this channel acquires */

struct ant_node {
    ant_espphy_t phy;
    ant_mac_t    mac;
    ant_node_config_t cfg;
    ant_known_t  known;              /* the paired-device table */
    struct {
        uint8_t flags;
        bool    fresh;               /* opened as a wildcard search */
        bool    acquired;            /* first page seen since open */
    } chs[ANT_MAC_MAX_CHANNELS];
    void  *lock;                     /* recursive mutex */
    void  *task;
    volatile bool running;
    uint32_t pages;                  /* pages delivered to on_data */
    uint32_t ticks;                  /* MAC ticks run */
};

/* Bring up the radio and the engine and start the ANT task. Returns the radio
 * status; on anything but ANT_ESPPHY_OK nothing is running. */
ant_espphy_status_t ant_node_start(ant_node_t *node, const ant_node_config_t *cfg);

/* Close every channel, stop the task, release the radio. */
void ant_node_stop(ant_node_t *node);

/* Channel control. Return ANT response codes (ant_message.h): 0 = no error,
 * ANT_RESPONSE_CHANNEL_IN_WRONG_STATE, ANT_RESPONSE_INVALID_* ... */
uint8_t ant_node_open(ant_node_t *node, uint8_t ch, const ant_node_channel_cfg_t *cfg);
uint8_t ant_node_close(ant_node_t *node, uint8_t ch);

/* ANT+ receiver for a profile (device_num 0 = the known device of that type,
 * else any device of that type - see ant_node_channel_cfg_t). */
uint8_t ant_node_open_antplus_slave(ant_node_t *node, uint8_t ch, uint8_t device_type,
                                    uint16_t device_num, uint16_t period);
/* Pair a new ANT+ device of a type: (re)open `ch` as a fresh search, ignoring
 * the known device; the master acquired (within the node's proximity
 * threshold, if set) replaces it. Reports through on_paired. */
uint8_t ant_node_pair(ant_node_t *node, uint8_t ch, uint8_t device_type, uint16_t period);
/* ANT+ sensor. */
uint8_t ant_node_open_antplus_master(ant_node_t *node, uint8_t ch, uint16_t device_num,
                                     uint8_t device_type, uint8_t trans_type, uint16_t period);

/* Master: the page broadcast every period from now on. Slave: one reverse
 * broadcast in the next slot. */
uint8_t ant_node_send_broadcast(ant_node_t *node, uint8_t ch, const uint8_t page[8]);
uint8_t ant_node_send_acknowledged(ant_node_t *node, uint8_t ch, const uint8_t page[8]);
uint8_t ant_node_send_burst(ant_node_t *node, uint8_t ch, const uint8_t *data, size_t len);

/* Queries. */
uint8_t ant_node_channel_status(ant_node_t *node, uint8_t ch);   /* ANT_STATUS_* */
bool    ant_node_channel_id(ant_node_t *node, uint8_t ch, uint16_t *device_num,
                            uint8_t *device_type, uint8_t *trans_type);
bool    ant_node_is_tracking(ant_node_t *node, uint8_t ch);

/* Known (paired) devices, one per device type. Changes are written to the
 * store at once. forget_* do not touch an open channel; close/pair it. */
bool   ant_node_known_device(ant_node_t *node, uint8_t device_type, ant_node_device_t *out);
size_t ant_node_known_devices(ant_node_t *node, ant_node_device_t *out, size_t cap);
bool   ant_node_remember_device(ant_node_t *node, const ant_node_device_t *dev);
bool   ant_node_forget_device(ant_node_t *node, uint8_t device_type);
void   ant_node_forget_all(ant_node_t *node);

/* Radio counters (ant_espphy_t fields) for diagnostics; read-only snapshot. */
static inline const ant_espphy_t *ant_node_radio(const ant_node_t *node) { return &node->phy; }

#ifdef __cplusplus
}
#endif
#endif /* ANT_NODE_H */
