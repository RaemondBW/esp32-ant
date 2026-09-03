/*
 * One radio, two protocols, in turns. The ESP32-S3 BLE core is either a BLE
 * controller under NimBLE or an ANT modem under ant_node - never both. This
 * sketch alternates: 20 s scanning for BLE heart-rate straps with NimBLE,
 * then 20 s searching for ANT+ straps, and back, printing what each hears.
 *
 * The rules a real application must follow are the three calls in
 * enter_ant() / enter_ble():
 *   NimBLEDevice::deinit(true)  before  ant_node_start()
 *   ant_node_stop()             before  NimBLEDevice::init()
 * ant_node_start() refuses (ANT_ESPPHY_ERR_BT) if a BLE host still owns the
 * controller, so a wrong order fails loudly rather than corrupting anything.
 */
#include <Arduino.h>
#include <NimBLEDevice.h>
#include "ant_node.h"

static const NimBLEUUID HR_SERVICE((uint16_t)0x180D);

static ant_node_t ant;
static bool       ant_running;
static volatile uint32_t ant_pages;
static volatile uint16_t ant_device;
static volatile uint8_t  ant_bpm;

static void on_ant_data(ant_node_t *, const ant_node_rx_t *rx, const uint8_t page[8], void *)
{
    antplus_hrm_data_t hr;
    if (rx->device_type == ANTPLUS_DEVTYPE_HRM && antplus_hrm_decode(page, &hr)) {
        ant_bpm = hr.computed_heart_rate;
        ant_device = rx->device_num;
        ant_pages = ant_pages + 1;
    }
}

class ScanLog : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice *dev) override {
        if (dev->isAdvertisingService(HR_SERVICE))
            Serial.printf("BLE HRM %s '%s' rssi %d\n", dev->getAddress().toString().c_str(),
                          dev->getName().c_str(), dev->getRSSI());
    }
} scan_log;

static void enter_ble()
{
    if (ant_running) { ant_node_stop(&ant); ant_running = false; }
    NimBLEDevice::init("ant-handoff");
    NimBLEScan *scan = NimBLEDevice::getScan();
    scan->setScanCallbacks(&scan_log, false);
    scan->setActiveScan(true);
    scan->start(0, false, true);          /* forever, until we switch */
    Serial.println("-- BLE: scanning for 0x180D --");
}

static void enter_ant()
{
    NimBLEDevice::getScan()->stop();
    NimBLEDevice::deinit(true);           /* controller -> IDLE */
    ant_node_config_t cfg = {};
    cfg.on_data = on_ant_data;
    cfg.store   = &ant_node_store_nvs;    /* the strap paired once stays paired across hand-offs */
    ant_espphy_status_t s = ant_node_start(&ant, &cfg);
    Serial.printf("-- ANT: start %s --\n", ant_espphy_status_str(s));
    if (s != ANT_ESPPHY_OK) return;
    ant_running = true;
    ant_pages = 0;
    ant_node_open_antplus_slave(&ant, 0, ANTPLUS_DEVTYPE_HRM, 0, ANTPLUS_PERIOD_HRM);
}

void setup()
{
    Serial.begin(115200);
    delay(2000);
    Serial.printf("free internal heap at boot: %u\n", ESP.getFreeHeap());
    enter_ble();
}

void loop()
{
    static uint32_t phase_start = millis();
    static bool on_ant = false;
    delay(1000);

    if (on_ant && ant_running && ant_pages)
        Serial.printf("ANT HRM #%u: %u bpm (%lu pages)\n", ant_device, ant_bpm, (unsigned long)ant_pages);

    if (millis() - phase_start > 20000) {
        phase_start = millis();
        on_ant = !on_ant;
        if (on_ant) enter_ant(); else enter_ble();
        Serial.printf("free internal heap: %u\n", ESP.getFreeHeap());
    }
}
