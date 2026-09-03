/*
 * app_main.c - pure-ESP32 ANT+. No external radio: the ESP32 runs the complete
 * ANT protocol engine (ant_mac) itself and drives its OWN 2.4 GHz radio as the
 * PHY (ant_espphy). ant_node bundles the two with the FreeRTOS task that ticks
 * the engine on the ANT 32768 Hz grid; this firmware is its reference user.
 *
 * Boot sequence:
 *   1. Self-test: an HRM sensor and an HRM display, both full ANT MACs, talk
 *      over the virtual air (ant_air) on this chip. Proves the protocol engine
 *      (timing, search/acquire, broadcast, acknowledged, burst) runs on target.
 *   2. Real run: ant_node on the S3/C3 BLE core driven as a raw ANT modem.
 *      Role: ANT+ HRM display (default, receives a real strap) or HRM sensor
 *      (-DANT_APP_ROLE_SENSOR, transmits to a real display). The radio is
 *      one-directional per boot, see ant_espphy.h.
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "ant_node.h"
#include "ant_phy_shockburst.h"
#include "ant_selftest.h"

static const char *TAG = "app";

#define ANT_APP_DEVICE_NUM   0x3042u
#define ANT_APP_CHANNEL      0u

/* ----------------------------- self-test ---------------------------------- */

static bool self_test(void)
{
    ant_selftest_result_t r;
    ant_selftest_run(&r);
    ESP_LOGI(TAG, "self-test: display got %d pages, learned id %04x/%u, HR=%u",
             r.rx_pages, r.learned_dev, r.learned_type, r.heart_rate);
    ESP_LOGI(TAG, "self-test: ack rx=%d (tx done=%d)  burst pkts=%d (tx done=%d) fails=%d",
             r.ack_rx, r.ack_tx_done, r.burst_pkts, r.burst_tx_done, r.tx_failed);
    ESP_LOGI(TAG, "self-test: air frames=%lu delivered=%lu -> %s",
             (unsigned long)r.air_frames, (unsigned long)r.air_delivered,
             r.pass ? "PASS" : "FAIL");
    return r.pass;
}

/* ------------------------------ real run ---------------------------------- */

static ant_node_t g_node;

static void on_data(ant_node_t *node, const ant_node_rx_t *rx, const uint8_t page[8], void *user)
{
    (void)node; (void)user;
    antplus_hrm_data_t hr;
    if (rx->device_type == ANTPLUS_DEVTYPE_HRM && antplus_hrm_decode(page, &hr)) {
        ESP_LOGI(TAG, "ch%u HRM #%u: %u bpm  beats=%u  t=%u  rssi=%d", rx->channel, rx->device_num,
                 hr.computed_heart_rate, hr.heart_beat_count, hr.heart_beat_event_time, rx->rssi);
    } else {
        ESP_LOGI(TAG, "ch%u #%u/%u page %02x %02x %02x %02x %02x %02x %02x %02x", rx->channel,
                 rx->device_num, rx->device_type,
                 page[0], page[1], page[2], page[3], page[4], page[5], page[6], page[7]);
    }
}

static void on_event(ant_node_t *node, uint8_t ch, uint8_t event, void *user)
{
    (void)node; (void)user;
    if (event != ANT_EVENT_TX)   /* TX every period is too chatty */
        ESP_LOGI(TAG, "ch%u event %s", ch, ant_mac_event_name(event));
}

static void on_paired(ant_node_t *node, uint8_t ch, const ant_node_device_t *dev,
                      bool remembered, void *user)
{
    (void)node; (void)user;
    ESP_LOGI(TAG, "ch%u %s %u/%u/%u%s", ch, remembered ? "PAIRED" : "connected",
             dev->device_num, dev->device_type, dev->trans_type,
             remembered ? " (saved to NVS; reconnects to it from now on)" : "");
}

static void log_radio(void)
{
    const ant_espphy_t *p = ant_node_radio(&g_node);
    ESP_LOGI(TAG, "%s  phy %s  tx=%lu emitted=%lu refused=%lu  rx hook=%lu/%lu frames=%lu "
                  "matched=%lu dropped=%lu polls=%lu  evt=%lu sync rw=%lu  hci err=%lu  rssi=%d  pages=%lu  "
                  "mac rx=%lu crcfail=%lu miss=%lu",
             ant_mac_state_name((ant_mac_ch_state_t)(ant_node_channel_status(&g_node, ANT_APP_CHANNEL) & ANT_STATUS_STATE_MASK)),
             ant_espphy_mode_str(p->mode),
             (unsigned long)p->tx_count, (unsigned long)p->tx_emitted, (unsigned long)p->tx_refused,
             (unsigned long)p->rx_hook_calls, (unsigned long)p->rx_hook_empty,
             (unsigned long)p->rx_frames, (unsigned long)p->rx_matched, (unsigned long)p->rx_dropped,
             (unsigned long)p->rx_polls, (unsigned long)p->sched_hook_calls,
             (unsigned long)p->sync_rewrites, (unsigned long)p->hci_errors, p->last_rssi,
             (unsigned long)g_node.pages,
             (unsigned long)g_node.mac.ch[ANT_APP_CHANNEL].rx_count,
             (unsigned long)g_node.mac.ch[ANT_APP_CHANNEL].crc_fail_count,
             (unsigned long)g_node.mac.ch[ANT_APP_CHANNEL].miss_count);
    if (p->last_len) {
        char hex[3 * ANT_SB_FRAME_MAX + 1]; size_t o = 0;
        for (size_t i = 0; i < p->last_len; i++)
            o += (size_t)snprintf(hex + o, sizeof(hex) - o, "%02x ", p->last_frame[i]);
        ESP_LOGI(TAG, "last frame (%u B @ %u MHz): %s", (unsigned)p->last_len, p->mhz, hex);
    }
}

#if !defined(ANT_APP_ROLE_SENSOR)
static uint8_t open_role(void)
{
    ant_node_device_t known;
    if (ant_node_known_device(&g_node, ANTPLUS_DEVTYPE_HRM, &known))
        ESP_LOGI(TAG, "role: ANT+ HRM display, reconnecting to strap #%u on %u MHz "
                      "(ANT_FORGET_KNOWN=1 rebuild to pair afresh)", known.device_num, 2400u + ANTPLUS_RF_FREQ);
    else
        ESP_LOGI(TAG, "role: ANT+ HRM display, pairing with the first HRM heard on %u MHz",
                 2400u + ANTPLUS_RF_FREQ);
    return ant_node_open_antplus_slave(&g_node, ANT_APP_CHANNEL, ANTPLUS_DEVTYPE_HRM, 0,
                                       ANTPLUS_PERIOD_HRM);
}
#else
static void sensor_task(void *arg)
{
    (void)arg;
    uint8_t bpm = 60; uint16_t beats = 0, event_time = 0;
    for (;;) {
        antplus_hrm_data_t hr = { .computed_heart_rate = bpm,
                                  .heart_beat_count = (uint8_t)beats,
                                  .heart_beat_event_time = event_time };
        uint8_t page[8];
        antplus_hrm_encode_page0(&hr, (beats & 4) != 0, page);
        ant_node_send_broadcast(&g_node, ANT_APP_CHANNEL, page);   /* repeats every slot */
        bpm = (uint8_t)(60 + ((bpm - 59) % 40));    /* sweep 60..99 */
        beats++;
        event_time += 1024;                         /* ~1 beat/s in 1/1024 s */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static uint8_t open_role(void)
{
    ESP_LOGI(TAG, "role: ANT+ HRM sensor #%u, %u MHz, period %u (%.2f Hz)",
             ANT_APP_DEVICE_NUM, 2400u + ANTPLUS_RF_FREQ, ANTPLUS_PERIOD_HRM,
             (double)ANT_TICKS_PER_SEC / ANTPLUS_PERIOD_HRM);
    uint8_t r = ant_node_open_antplus_master(&g_node, ANT_APP_CHANNEL, ANT_APP_DEVICE_NUM,
                                             ANTPLUS_DEVTYPE_HRM, 1, ANTPLUS_PERIOD_HRM);
    if (r == ANT_RESPONSE_NO_ERROR) xTaskCreate(sensor_task, "hrm", 2048, NULL, 5, NULL);
    return r;
}
#endif

void app_main(void)
{
    ESP_LOGI(TAG, "pure-ESP32 ANT+ starting (internal radio, no external chip)");

    ESP_LOGI(TAG, "self-test: two ANT MACs over the virtual air on this chip");
    if (!self_test()) {
        ESP_LOGE(TAG, "self-test FAILED - not starting the radio");
        return;
    }

    esp_err_t nv = nvs_flash_init();           /* RF calibration data lives in NVS */
    if (nv == ESP_ERR_NVS_NO_FREE_PAGES || nv == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nv = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nv);

    ant_node_config_t cfg = { .on_data = on_data, .on_event = on_event, .on_paired = on_paired,
                              .store = &ant_node_store_nvs };
#if defined(ANT_PROXIMITY_RSSI)
    cfg.proximity_rssi = ANT_PROXIMITY_RSSI;    /* pair only with devices at least this strong */
    ESP_LOGI(TAG, "proximity pairing: >= %d dBm", ANT_PROXIMITY_RSSI);
#endif
    ant_espphy_status_t s = ant_node_start(&g_node, &cfg);
    ESP_LOGI(TAG, "ant node: %s", ant_espphy_status_str(s));
    if (s != ANT_ESPPHY_OK) return;
#if defined(ANT_FORGET_KNOWN)
    ant_node_forget_all(&g_node);
    ESP_LOGI(TAG, "known devices forgotten");
#endif

    uint8_t r = open_role();
    if (r != ANT_RESPONSE_NO_ERROR) {
        ESP_LOGE(TAG, "open channel: response 0x%02x", r);
        return;
    }
#if defined(ANT_LOG_FRAMES)
    /* Diagnostics: every packet the radio hook decodes, with RSSI, CRC verdict
     * and the gap to the previous one (ticks, 32768 Hz; a slot is 8070). */
    ((ant_espphy_t *)ant_node_radio(&g_node))->tap_on = true;
    uint32_t last_tick = 0;
    for (int i = 0;; i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
        ant_espphy_rx_t f; bool matched;
        while (ant_espphy_tap_poll((ant_espphy_t *)ant_node_radio(&g_node), &f, &matched)) {
            bool crc = ant_sb_verify_frame(f.body, f.len, 5, (uint8_t)(f.len - 7), NULL);
            char hex[3 * ANT_SB_FRAME_MAX + 1]; size_t o = 0;
            for (size_t k = 0; k < f.len; k++) o += (size_t)snprintf(hex + o, sizeof(hex) - o, "%02x ", f.body[k]);
            ESP_LOGI(TAG, "F %+6ld %4d %s %s %s", (long)(f.tick - last_tick), f.rssi,
                     matched ? "M" : "-", crc ? "ok " : "CRC", hex);
            last_tick = f.tick;
        }
        if (i % 100 == 99) log_radio();
    }
#else
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        log_radio();
    }
#endif
}
