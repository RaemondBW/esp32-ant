/*
 * One radio, both protocols at once. The ESP32-S3 BLE controller normally
 * belongs to NimBLE; here NimBLE stays up - including a live BLE connection -
 * while the ANT stack rides the same radio to receive an ANT+ heart-rate
 * strap. No hand-off, no deinit: both are live together.
 *
 * How it works (see ant_espphy_init_coexist / ant_node coexist flag):
 *   - NimBLE owns the controller. ant_node_start() with cfg.coexist = true does
 *     NOT take it; it hooks the controller's scan path (r_lld_scan_sched /
 *     r_lld_scan_process_pkt_rx). While an ANT receive is open, the scan's
 *     radio windows are retuned to 2457 MHz / 1 Mbit/s / the ANT sync word,
 *     ANT frames are lifted out of the RX descriptor, and the never-whitened
 *     ANT payload is recovered in software - so NO global radio register is
 *     touched and BLE connection events between the scan windows keep working.
 *   - Trade-off: the scan windows are consumed by ANT, so BLE *scanning* pauses
 *     while an ANT receive is open (find/connect your BLE peers first). BLE
 *     *connections* are unaffected, which is what this sketch demonstrates.
 *   - RX only: coexist mode cannot transmit ANT.
 *
 * What it does:
 *   1. NimBLE scans for a BLE heart-rate peripheral (service 0x180D).
 *   2. Connects to the first one found and subscribes to HR measurement.
 *   3. Starts ANT+ in coexist mode and opens an ANT+ HRM channel.
 *   4. Prints both the BLE HR (from connection notifications) and the ANT+ HR
 *      (from the shared radio) as they arrive - both live at once.
 * If no BLE peripheral is found in ~15 s it starts ANT anyway, so the ANT side
 * still runs without a BLE peer.
 *
 * Order rule: NimBLE must be up before ant_node_start(coexist), and
 * ant_node_stop() before NimBLEDevice::deinit().
 */
#include <Arduino.h>
#include <NimBLEDevice.h>
#include "ant_node.h"

static const NimBLEUUID HR_SERVICE((uint16_t)0x180D);
static const NimBLEUUID HR_MEAS((uint16_t)0x2A37);

static ant_node_t ant;
static volatile uint32_t ant_pages;
static volatile uint16_t ant_device;
static volatile uint8_t  ant_bpm;
static volatile int8_t   ant_rssi;

static volatile uint8_t  ble_bpm;
static volatile uint32_t ble_notifs;
static bool              ble_connected;

/* ------------------------------- ANT+ side ------------------------------- */

static void on_ant_data(ant_node_t *, const ant_node_rx_t *rx, const uint8_t page[8], void *)
{
    antplus_hrm_data_t hr;
    if (rx->device_type == ANTPLUS_DEVTYPE_HRM && antplus_hrm_decode(page, &hr)) {
        ant_bpm = hr.computed_heart_rate;
        ant_device = rx->device_num;
        ant_rssi = rx->rssi;
        ant_pages = ant_pages + 1;
    }
}

static void on_ant_paired(ant_node_t *, uint8_t, const ant_node_device_t *dev, bool remembered, void *)
{
    Serial.printf("ANT+ %s HRM %u\n", remembered ? "reconnected" : "paired", dev->device_num);
}

static void start_ant()
{
    ant_node_config_t cfg = {};
    cfg.on_data   = on_ant_data;
    cfg.on_paired = on_ant_paired;
    cfg.store     = &ant_node_store_nvs;
    cfg.coexist   = true;                 /* share the radio with the live BLE host */
    ant_espphy_status_t s = ant_node_start(&ant, &cfg);
    Serial.printf("-- ANT: start %s --\n", ant_espphy_status_str(s));
    if (s == ANT_ESPPHY_OK)
        ant_node_open_antplus_slave(&ant, 0, ANTPLUS_DEVTYPE_HRM, 0, ANTPLUS_PERIOD_HRM);
}

/* ------------------------------- BLE side -------------------------------- */

static void on_hr_notify(NimBLERemoteCharacteristic *, uint8_t *data, size_t len, bool)
{
    if (len >= 2) ble_bpm = (data[0] & 0x01) ? (uint8_t)(data[1] | (data[2] << 8)) : data[1];
    ble_notifs = ble_notifs + 1;
}

class ClientCb : public NimBLEClientCallbacks {
    void onDisconnect(NimBLEClient *, int reason) override {
        Serial.printf("BLE disconnected (reason %d)\n", reason);
        ble_connected = false;
    }
};
static ClientCb client_cb;

static bool connect_hr(const NimBLEAdvertisedDevice *dev)
{
    NimBLEClient *cli = NimBLEDevice::createClient();
    cli->setClientCallbacks(&client_cb, false);
    if (!cli->connect(dev)) { NimBLEDevice::deleteClient(cli); return false; }
    NimBLERemoteService *svc = cli->getService(HR_SERVICE);
    NimBLERemoteCharacteristic *ch = svc ? svc->getCharacteristic(HR_MEAS) : nullptr;
    if (!ch || !ch->canNotify() || !ch->subscribe(true, on_hr_notify)) {
        cli->disconnect();
        NimBLEDevice::deleteClient(cli);
        return false;
    }
    ble_connected = true;
    Serial.printf("BLE connected to %s '%s', subscribed to HR\n",
                  dev->getAddress().toString().c_str(), dev->getName().c_str());
    return true;
}

void setup()
{
    Serial.begin(115200);
    delay(2000);
    Serial.printf("free internal heap at boot: %u\n", ESP.getFreeHeap());

    NimBLEDevice::init("ant-coexist");

    /* Find and connect a BLE HR peripheral first (scanning is consumed once ANT
     * is running, so do all BLE discovery/connecting up front). */
    Serial.println("-- BLE: scanning for a 0x180D peripheral (15 s) --");
    NimBLEScan *scan = NimBLEDevice::getScan();
    scan->setActiveScan(true);
    NimBLEScanResults res = scan->getResults(15 * 1000, false);
    for (int i = 0; i < res.getCount(); i++) {
        const NimBLEAdvertisedDevice *d = res.getDevice(i);
        if (d->isAdvertisingService(HR_SERVICE)) {
            Serial.printf("BLE HRM found: %s '%s' rssi %d\n", d->getAddress().toString().c_str(),
                          d->getName().c_str(), d->getRSSI());
            if (connect_hr(d)) break;
        }
    }
    if (!ble_connected) Serial.println("-- no BLE HR peripheral connected; ANT only --");

    /* ANT rides the scan's radio windows, so a scan must be running the whole
     * time ANT is open. Start a perpetual passive scan now; it coexists with
     * the BLE connection (connection events run between the scan windows). */
    scan->setActiveScan(false);
    scan->start(0, false, true);

    /* Now bring ANT up on the same radio - the BLE connection (if any) keeps
     * running through it. */
    start_ant();
    Serial.println(ble_connected ? "-- both live: BLE HR connection + ANT+ HRM --"
                                  : "-- ANT+ HRM live --");
}

void loop()
{
    static uint32_t last_pages, last_notifs;
    delay(3000);

    uint32_t pages = ant_pages, notifs = ble_notifs;
    Serial.printf("[%lus] ANT+ %u bpm dev %u rssi %d (+%lu)  |  BLE %s %u bpm (+%lu notif)  |  heap %u\n",
                  (unsigned long)(millis() / 1000), ant_bpm, ant_device, ant_rssi,
                  (unsigned long)(pages - last_pages),
                  ble_connected ? "conn" : "----", ble_bpm, (unsigned long)(notifs - last_notifs),
                  ESP.getFreeHeap());
    Serial.printf("        phy: sched=%lu rxhook=%lu frames=%lu matched=%lu mode=%s\n",
                  (unsigned long)ant.phy.sched_hook_calls, (unsigned long)ant.phy.rx_hook_calls,
                  (unsigned long)ant.phy.rx_frames, (unsigned long)ant.phy.rx_matched,
                  ant_espphy_mode_str(ant.phy.mode));
    last_pages = pages;
    last_notifs = notifs;
}
