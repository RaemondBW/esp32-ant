/*
 * ANT+ heart-rate monitor display (or, with -DANT_EXAMPLE_SENSOR, a fake HRM
 * sensor) on a plain ESP32-S3 running the Arduino framework. The ESP32's own
 * BLE core is the ANT radio; nothing external.
 *
 * Pairing: the first strap heard is remembered in NVS and reconnected to on
 * every boot. Serial commands: 'p' pair a new strap (nearby ones only),
 * 'f' forget it, 'l' list known devices.
 *
 * The radio is exclusive: if the sketch also uses NimBLE / Bluedroid, stop it
 * (NimBLEDevice::deinit(true) / btStop()) before ant_node_start() and call
 * ant_node_stop() before bringing BLE back.
 */
#include <Arduino.h>
#include "ant_node.h"

static ant_node_t node;

/* Runs on the ANT task: keep it short, hand data to loop() through volatiles. */
static volatile uint8_t  g_bpm;
static volatile uint16_t g_device;
static volatile uint32_t g_pages;
static volatile int8_t   g_rssi;

static void on_data(ant_node_t *, const ant_node_rx_t *rx, const uint8_t page[8], void *)
{
    antplus_hrm_data_t hr;
    if (rx->device_type == ANTPLUS_DEVTYPE_HRM && antplus_hrm_decode(page, &hr)) {
        g_bpm    = hr.computed_heart_rate;
        g_device = rx->device_num;
        g_rssi   = rx->rssi;
        g_pages  = g_pages + 1;
    }
}

static void on_event(ant_node_t *, uint8_t ch, uint8_t event, void *)
{
    if (event != ANT_EVENT_TX)
        Serial.printf("ch%u: %s\n", ch, ant_mac_event_name(event));
}

static void on_paired(ant_node_t *, uint8_t ch, const ant_node_device_t *dev, bool remembered, void *)
{
    Serial.printf("ch%u: %s device %u type %u trans %u\n", ch,
                  remembered ? "PAIRED (saved)" : "connected to", dev->device_num,
                  dev->device_type, dev->trans_type);
}

static void list_known()
{
    ant_node_device_t devs[ANT_KNOWN_MAX];
    size_t n = ant_node_known_devices(&node, devs, ANT_KNOWN_MAX);
    Serial.printf("%u known device(s)\n", (unsigned)n);
    for (size_t i = 0; i < n; i++)
        Serial.printf("  #%u type %u trans %u\n", devs[i].device_num, devs[i].device_type,
                      devs[i].trans_type);
}

void setup()
{
    Serial.begin(115200);
    delay(2000);
    Serial.println("pure-ESP32 ANT+ example");

    ant_node_config_t cfg = {};
    cfg.on_data   = on_data;
    cfg.on_event  = on_event;
    cfg.on_paired = on_paired;
    cfg.store     = &ant_node_store_nvs;   /* the Arduino core has NVS up already */
    cfg.proximity_rssi = -70;             /* pair only with a strap that is close */
    ant_espphy_status_t s = ant_node_start(&node, &cfg);
    Serial.printf("ant_node_start: %s\n", ant_espphy_status_str(s));
    if (s != ANT_ESPPHY_OK) return;
    list_known();

#ifdef ANT_EXAMPLE_SENSOR
    uint8_t r = ant_node_open_antplus_master(&node, 0, 0x3042, ANTPLUS_DEVTYPE_HRM, 1,
                                             ANTPLUS_PERIOD_HRM);
    Serial.printf("HRM sensor #0x3042 open: 0x%02x\n", r);
#else
    /* device 0: the remembered strap if there is one, else the first HRM heard */
    uint8_t r = ant_node_open_antplus_slave(&node, 0, ANTPLUS_DEVTYPE_HRM, 0, ANTPLUS_PERIOD_HRM);
    Serial.printf("HRM display open: 0x%02x\n", r);
#endif
}

static void handle_serial()
{
    while (Serial.available()) {
        switch (Serial.read()) {
        case 'p':
            Serial.println("pairing: searching for a nearby HRM...");
            ant_node_pair(&node, 0, ANTPLUS_DEVTYPE_HRM, ANTPLUS_PERIOD_HRM);
            break;
        case 'f':
            Serial.printf("forget HRM: %s\n", ant_node_forget_device(&node, ANTPLUS_DEVTYPE_HRM) ? "done" : "none known");
            break;
        case 'l':
            list_known();
            break;
        default:
            break;
        }
    }
}

void loop()
{
    static uint32_t last_pages;
    for (int i = 0; i < 10; i++) { handle_serial(); delay(100); }

#ifdef ANT_EXAMPLE_SENSOR
    static uint8_t bpm = 60; static uint16_t beats, t;
    antplus_hrm_data_t hr = {};
    hr.computed_heart_rate  = bpm;
    hr.heart_beat_count     = (uint8_t)beats;
    hr.heart_beat_event_time = t;
    uint8_t page[8];
    antplus_hrm_encode_page0(&hr, (beats & 4) != 0, page);
    ant_node_send_broadcast(&node, 0, page);
    bpm = (uint8_t)(60 + ((bpm - 59) % 40));
    beats++; t += 1024;
#endif

    const ant_espphy_t *p = ant_node_radio(&node);
    if (g_pages != last_pages) {
        last_pages = g_pages;
        Serial.printf("HRM #%u: %u bpm  (rssi %d dBm, %lu pages)\n",
                      g_device, g_bpm, g_rssi, (unsigned long)g_pages);
    } else {
        Serial.printf("%s  tracking=%d  frames=%lu matched=%lu evt=%lu\n",
                      ant_espphy_mode_str(p->mode), ant_node_is_tracking(&node, 0),
                      (unsigned long)p->rx_frames, (unsigned long)p->rx_matched,
                      (unsigned long)p->sched_hook_calls);
    }
}
