/*
 * ant_node.c - radio + engine + task. See ant_node.h.
 */
#include "ant_node.h"

#ifdef ESP_PLATFORM

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "ant_node";

#define NODE_MAX_WAIT_MS   100u     /* re-tick at least this often */
#define NODE_STOP_WAIT_MS  500u

#define LOCK(n)   xSemaphoreTakeRecursive((SemaphoreHandle_t)(n)->lock, portMAX_DELAY)
#define UNLOCK(n) xSemaphoreGiveRecursive((SemaphoreHandle_t)(n)->lock)

static bool is_master_type(uint8_t type)
{
    return type == ANT_CHANNEL_TYPE_MASTER_TX || type == ANT_CHANNEL_TYPE_MASTER_TX_ONLY;
}

/* ----------------------------- known devices ------------------------------ */

static void store_save(ant_node_t *n)
{
    if (n->cfg.store && n->cfg.store->save && !n->cfg.store->save(n->cfg.store, &n->known))
        ESP_LOGW(TAG, "known-device store: save failed");
}

/* A slave channel just heard its master for the first time since open. */
static void channel_acquired(ant_node_t *n, uint8_t ch)
{
    ant_node_device_t dev;
    uint8_t type;
    if (!ant_mac_get_channel_id(&n->mac, ch, &dev.device_num, &type, &dev.trans_type)) return;
    dev.device_type = ANT_DEVICE_TYPE_OF(type);
    bool remembered = false;
    if (n->chs[ch].fresh && !(n->chs[ch].flags & ANT_NODE_CH_NO_REMEMBER)) {
        const ant_known_device_t *old = ant_known_find(&n->known, dev.device_type);
        bool same = old && old->device_num == dev.device_num && old->trans_type == dev.trans_type;
        if (!same && ant_known_put(&n->known, &dev)) {
            store_save(n);
            remembered = true;
            ESP_LOGI(TAG, "ch%u paired %u/%u/%u (rssi %d)%s", ch, dev.device_num, dev.device_type,
                     dev.trans_type, ant_mac_channel(&n->mac, ch)->last_rssi,
                     old ? ", replacing the previous one" : "");
        }
    }
    if (n->cfg.on_paired) n->cfg.on_paired(n, ch, &dev, remembered, n->cfg.user);
}

/* ------------------------------- callbacks -------------------------------- */

static void mac_on_data(ant_mac_t *mac, uint8_t ch, uint8_t msg_type, uint8_t burst_seq,
                        const uint8_t data[8], void *user)
{
    (void)mac;
    ant_node_t *n = (ant_node_t *)user;
    n->pages++;
    const ant_mac_channel_t *c = ant_mac_channel(&n->mac, ch);
    if (c && !is_master_type(c->type) && !n->chs[ch].acquired) {
        n->chs[ch].acquired = true;
        channel_acquired(n, ch);
    }
    if (!n->cfg.on_data) return;
    ant_node_rx_t rx = { .channel = ch, .msg_type = msg_type, .burst_seq = burst_seq,
                         .rssi = c ? c->last_rssi : 0 };
    ant_mac_get_channel_id(&n->mac, ch, &rx.device_num, &rx.device_type, &rx.trans_type);
    rx.pairing_request = (rx.device_type & ANT_DEVICE_TYPE_PAIRING_BIT) != 0;
    rx.device_type = ANT_DEVICE_TYPE_OF(rx.device_type);
    n->cfg.on_data(n, &rx, data, n->cfg.user);
}

static void mac_on_event(ant_mac_t *mac, uint8_t ch, uint8_t event, void *user)
{
    (void)mac;
    ant_node_t *n = (ant_node_t *)user;
    if (n->cfg.on_event) n->cfg.on_event(n, ch, event, n->cfg.user);
}

/* --------------------------------- task ----------------------------------- */

/* Sleep until `deadline` (ANT ticks). Wakes early when the radio queues a
 * frame (timestamped in the interrupt, so handling it a little later costs
 * nothing) or when an API call changed the schedule. In TX mode the last
 * millisecond is spun so master slots land on the tick. */
static void wait_until(ant_node_t *n, uint32_t deadline)
{
    for (;;) {
        int32_t left = (int32_t)(deadline - ant_espphy_ticks());
        if (left <= 0 || !n->running) return;
        uint32_t us = (uint32_t)((uint64_t)left * 15625u / 512u);
        if (us >= 1000u || n->phy.mode != ANT_ESPPHY_MODE_TX) {
            uint32_t ms = us / 1000u;
            if (ms < 1) ms = 1;
            if (ms > NODE_MAX_WAIT_MS) ms = NODE_MAX_WAIT_MS;
            /* returns on a frame, a notification from the API, or timeout */
            ant_espphy_wait_rx(&n->phy, ms);
            return;
        } else if (us > 50) {
            esp_rom_delay_us(us - 40);
        } else {
            esp_rom_delay_us(us);
        }
    }
}

static void node_task(void *arg)
{
    ant_node_t *n = (ant_node_t *)arg;
    while (n->running) {
        LOCK(n);
        uint32_t deadline = ant_mac_tick(&n->mac, ant_espphy_ticks());
        n->ticks++;
        UNLOCK(n);
        wait_until(n, deadline);
    }
    n->task = NULL;
    vTaskDelete(NULL);
}

/* An API call may have moved the next deadline: wake the task to re-tick. */
static void kick(ant_node_t *n)
{
    if (n->task && xTaskGetCurrentTaskHandle() != (TaskHandle_t)n->task)
        xTaskNotifyGive((TaskHandle_t)n->task);
}

/* -------------------------------- lifecycle ------------------------------- */

ant_espphy_status_t ant_node_start(ant_node_t *n, const ant_node_config_t *cfg)
{
    if (!n || !cfg) return ANT_ESPPHY_ERR_ARG;
    memset(n, 0, sizeof(*n));
    n->cfg = *cfg;
    n->lock = xSemaphoreCreateRecursiveMutex();
    if (!n->lock) return ANT_ESPPHY_ERR_ARG;

    ant_espphy_status_t s = ant_espphy_init(&n->phy);
    if (s != ANT_ESPPHY_OK) {
        ESP_LOGE(TAG, "radio: %s", ant_espphy_status_str(s));
        vSemaphoreDelete((SemaphoreHandle_t)n->lock);
        n->lock = NULL;
        return s;
    }
    ant_mac_init(&n->mac, ant_espphy_phy(&n->phy), mac_on_data, mac_on_event, n);
    ant_known_init(&n->known);
    if (n->cfg.store && n->cfg.store->load && n->cfg.store->load(n->cfg.store, &n->known)) {
        for (unsigned i = 0; i < n->known.count; i++)
            ESP_LOGI(TAG, "known device: %u/%u/%u", n->known.dev[i].device_num,
                     n->known.dev[i].device_type, n->known.dev[i].trans_type);
    }

    uint32_t stack = n->cfg.task_stack ? n->cfg.task_stack : 4096u;
    UBaseType_t prio = n->cfg.task_priority ? n->cfg.task_priority : (UBaseType_t)(configMAX_PRIORITIES - 2);
    BaseType_t core = tskNO_AFFINITY;
#if (portNUM_PROCESSORS > 1) || (defined(CONFIG_FREERTOS_NUMBER_OF_CORES) && CONFIG_FREERTOS_NUMBER_OF_CORES > 1)
    core = 1;
#endif
    if (n->cfg.task_core == ANT_NODE_CORE_ANY) core = tskNO_AFFINITY;
    else if (n->cfg.task_core & 0x80u) core = n->cfg.task_core & 0x01u;
    n->running = true;
    TaskHandle_t t = NULL;
    if (xTaskCreatePinnedToCore(node_task, "ant", stack, n, prio, &t, core) != pdPASS) {
        n->running = false;
        ant_espphy_deinit(&n->phy);
        vSemaphoreDelete((SemaphoreHandle_t)n->lock);
        n->lock = NULL;
        return ANT_ESPPHY_ERR_ARG;
    }
    n->task = t;
    ESP_LOGI(TAG, "ANT node up (task prio %u, core %d)", (unsigned)prio, (int)core);
    return ANT_ESPPHY_OK;
}

void ant_node_stop(ant_node_t *n)
{
    if (!n || !n->lock) return;
    LOCK(n);
    for (uint8_t ch = 0; ch < ANT_MAC_MAX_CHANNELS; ch++) {
        ant_mac_close_channel(&n->mac, ch);
        ant_mac_unassign_channel(&n->mac, ch);
    }
    n->running = false;
    UNLOCK(n);
    kick(n);
    for (int i = 0; i < (int)(NODE_STOP_WAIT_MS / 10) && n->task; i++) vTaskDelay(pdMS_TO_TICKS(10));
    ant_espphy_deinit(&n->phy);
    vSemaphoreDelete((SemaphoreHandle_t)n->lock);
    n->lock = NULL;
    ESP_LOGI(TAG, "ANT node stopped");
}

/* -------------------------------- channels -------------------------------- */

uint8_t ant_node_open(ant_node_t *n, uint8_t ch, const ant_node_channel_cfg_t *c)
{
    if (!n || !n->lock || !c || ch >= ANT_MAC_MAX_CHANNELS) return ANT_RESPONSE_INVALID_PARAMETER;
    uint8_t rf   = c->rf_freq ? c->rf_freq : ANTPLUS_RF_FREQ;
    uint8_t tmo  = c->search_timeout ? c->search_timeout : 0xFF;
    const uint8_t *key = c->network_key ? c->network_key : ANTPLUS_NETWORK_KEY;
    uint16_t dev = c->device_num;
    uint8_t  type = c->device_type, trans = c->trans_type;
    bool slave = !is_master_type(c->type);
    bool reconnect = false;

    LOCK(n);
    /* Auto-reconnect: a wildcard slave of a known type opens with its id. */
    if (slave && dev == 0 && ANT_DEVICE_TYPE_OF(type) != 0 && !(c->flags & ANT_NODE_CH_IGNORE_KNOWN)) {
        const ant_known_device_t *k = ant_known_find(&n->known, type);
        if (k) {
            dev = k->device_num;
            type = (uint8_t)(k->device_type | (type & ANT_DEVICE_TYPE_PAIRING_BIT));
            trans = k->trans_type;
            reconnect = true;
        }
    }
    bool fresh = slave && dev == 0;
    int8_t prox = c->proximity_rssi ? c->proximity_rssi : n->cfg.proximity_rssi;

    uint8_t r = ant_mac_set_network_key(&n->mac, c->network, key);
    if (r == ANT_RESPONSE_NO_ERROR) r = ant_mac_assign_channel(&n->mac, ch, c->type, c->network);
    if (r == ANT_RESPONSE_NO_ERROR) r = ant_mac_set_channel_id(&n->mac, ch, dev, type, trans);
    if (r == ANT_RESPONSE_NO_ERROR) r = ant_mac_set_channel_rf_freq(&n->mac, ch, rf);
    if (r == ANT_RESPONSE_NO_ERROR) r = ant_mac_set_channel_period(&n->mac, ch, c->period);
    if (r == ANT_RESPONSE_NO_ERROR) r = ant_mac_set_search_timeout(&n->mac, ch, tmo);
    if (r == ANT_RESPONSE_NO_ERROR && fresh) r = ant_mac_set_proximity_search(&n->mac, ch, prox);
    if (r == ANT_RESPONSE_NO_ERROR) r = ant_mac_open_channel(&n->mac, ch);
    if (r != ANT_RESPONSE_NO_ERROR) ant_mac_unassign_channel(&n->mac, ch);
    else {
        n->chs[ch].flags = c->flags;
        n->chs[ch].fresh = fresh;
        n->chs[ch].acquired = false;
    }
    UNLOCK(n);
    kick(n);
    if (r == ANT_RESPONSE_NO_ERROR)
        ESP_LOGI(TAG, "ch%u open: type 0x%02x id %u/%u/%u %u MHz period %u%s%s", ch, c->type,
                 dev, type, trans, 2400u + rf, c->period,
                 reconnect ? " (known device)" : "",
                 fresh && prox ? " proximity-limited" : "");
    else
        ESP_LOGW(TAG, "ch%u open failed: response 0x%02x", ch, r);
    return r;
}

uint8_t ant_node_close(ant_node_t *n, uint8_t ch)
{
    if (!n || !n->lock) return ANT_RESPONSE_INVALID_PARAMETER;
    LOCK(n);
    uint8_t r = ant_mac_close_channel(&n->mac, ch);
    ant_mac_unassign_channel(&n->mac, ch);
    UNLOCK(n);
    kick(n);
    return r;
}

uint8_t ant_node_open_antplus_slave(ant_node_t *n, uint8_t ch, uint8_t device_type,
                                    uint16_t device_num, uint16_t period)
{
    ant_node_channel_cfg_t c = { .type = ANT_CHANNEL_TYPE_SLAVE_RX, .device_num = device_num,
                                 .device_type = device_type, .period = period };
    return ant_node_open(n, ch, &c);
}

uint8_t ant_node_pair(ant_node_t *n, uint8_t ch, uint8_t device_type, uint16_t period)
{
    if (!n || !n->lock || ch >= ANT_MAC_MAX_CHANNELS) return ANT_RESPONSE_INVALID_PARAMETER;
    LOCK(n);
    uint8_t st = ant_mac_channel_status(&n->mac, ch) & ANT_STATUS_STATE_MASK;
    if (st != ANT_STATUS_UNASSIGNED) ant_node_close(n, ch);
    ant_node_channel_cfg_t c = { .type = ANT_CHANNEL_TYPE_SLAVE_RX, .device_type = device_type,
                                 .period = period, .flags = ANT_NODE_CH_IGNORE_KNOWN };
    uint8_t r = ant_node_open(n, ch, &c);
    UNLOCK(n);
    return r;
}

uint8_t ant_node_open_antplus_master(ant_node_t *n, uint8_t ch, uint16_t device_num,
                                     uint8_t device_type, uint8_t trans_type, uint16_t period)
{
    ant_node_channel_cfg_t c = { .type = ANT_CHANNEL_TYPE_MASTER_TX, .device_num = device_num,
                                 .device_type = device_type, .trans_type = trans_type,
                                 .period = period };
    return ant_node_open(n, ch, &c);
}

/* ---------------------------------- data ---------------------------------- */

uint8_t ant_node_send_broadcast(ant_node_t *n, uint8_t ch, const uint8_t page[8])
{
    if (!n || !n->lock) return ANT_RESPONSE_INVALID_PARAMETER;
    LOCK(n);
    uint8_t r = ant_mac_send_broadcast(&n->mac, ch, page);
    UNLOCK(n);
    kick(n);
    return r;
}

uint8_t ant_node_send_acknowledged(ant_node_t *n, uint8_t ch, const uint8_t page[8])
{
    if (!n || !n->lock) return ANT_RESPONSE_INVALID_PARAMETER;
    LOCK(n);
    uint8_t r = ant_mac_send_acknowledged(&n->mac, ch, page);
    UNLOCK(n);
    kick(n);
    return r;
}

uint8_t ant_node_send_burst(ant_node_t *n, uint8_t ch, const uint8_t *data, size_t len)
{
    if (!n || !n->lock) return ANT_RESPONSE_INVALID_PARAMETER;
    LOCK(n);
    uint8_t r = ant_mac_send_burst(&n->mac, ch, data, len);
    UNLOCK(n);
    kick(n);
    return r;
}

/* -------------------------------- queries --------------------------------- */

uint8_t ant_node_channel_status(ant_node_t *n, uint8_t ch)
{
    if (!n || !n->lock) return ANT_STATUS_UNASSIGNED;
    LOCK(n);
    uint8_t s = ant_mac_channel_status(&n->mac, ch);
    UNLOCK(n);
    return s;
}

bool ant_node_channel_id(ant_node_t *n, uint8_t ch, uint16_t *device_num,
                         uint8_t *device_type, uint8_t *trans_type)
{
    if (!n || !n->lock) return false;
    LOCK(n);
    bool ok = ant_mac_get_channel_id(&n->mac, ch, device_num, device_type, trans_type);
    UNLOCK(n);
    return ok;
}

bool ant_node_is_tracking(ant_node_t *n, uint8_t ch)
{
    return (ant_node_channel_status(n, ch) & ANT_STATUS_STATE_MASK) == ANT_STATUS_TRACKING;
}

/* ----------------------------- known devices ------------------------------ */

bool ant_node_known_device(ant_node_t *n, uint8_t device_type, ant_node_device_t *out)
{
    if (!n || !n->lock) return false;
    LOCK(n);
    const ant_known_device_t *k = ant_known_find(&n->known, device_type);
    if (k && out) *out = *k;
    UNLOCK(n);
    return k != NULL;
}

size_t ant_node_known_devices(ant_node_t *n, ant_node_device_t *out, size_t cap)
{
    if (!n || !n->lock) return 0;
    LOCK(n);
    size_t k = n->known.count;
    if (k > cap) k = cap;
    if (out) memcpy(out, n->known.dev, k * sizeof(*out));
    UNLOCK(n);
    return k;
}

bool ant_node_remember_device(ant_node_t *n, const ant_node_device_t *dev)
{
    if (!n || !n->lock || !dev) return false;
    LOCK(n);
    bool ok = ant_known_put(&n->known, dev);
    if (ok) store_save(n);
    UNLOCK(n);
    return ok;
}

bool ant_node_forget_device(ant_node_t *n, uint8_t device_type)
{
    if (!n || !n->lock) return false;
    LOCK(n);
    bool had = ant_known_remove(&n->known, device_type);
    if (had) store_save(n);
    UNLOCK(n);
    return had;
}

void ant_node_forget_all(ant_node_t *n)
{
    if (!n || !n->lock) return;
    LOCK(n);
    bool had = n->known.count > 0;
    ant_known_clear(&n->known);
    if (had) store_save(n);
    UNLOCK(n);
}

#endif /* ESP_PLATFORM */
