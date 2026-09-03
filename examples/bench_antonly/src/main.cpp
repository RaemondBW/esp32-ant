/* Bench: ANT+ coexist on a bare devkit, node started BEFORE the scan so the
 * window hooks are captured, with console knobs mirroring the bike computer's
 * `ant` command. Optional BLE connection to a 0x180D peripheral: type "ble". */
#include <Arduino.h>
#include <NimBLEDevice.h>
#include "ant_node.h"

static ant_node_t ant;
static volatile uint32_t ant_pages; static volatile uint16_t ant_device;
static volatile uint8_t ant_bpm; static volatile int8_t ant_rssi;
static int tapLeft = 0;

static void on_ant_data(ant_node_t *, const ant_node_rx_t *rx, const uint8_t page[8], void *) {
    antplus_hrm_data_t hr;
    if (rx->device_type == ANTPLUS_DEVTYPE_HRM && antplus_hrm_decode(page, &hr)) {
        ant_bpm = hr.computed_heart_rate; ant_device = rx->device_num; ant_rssi = rx->rssi;
        ant_pages = ant_pages + 1;
    }
}

static int scanSlots = 16;
static void startScan() {
    NimBLEScan *scan = NimBLEDevice::getScan();
    scan->setActiveScan(false);
    scan->setInterval(scanSlots); scan->setWindow(scanSlots);
    scan->start(0, false, true);
}

static void connectBle() {
    NimBLEScan *scan = NimBLEDevice::getScan();
    ant.phy.mode = ANT_ESPPHY_MODE_IDLE;   /* bench: let a real BLE scan run */
    scan->stop();
    scan->setActiveScan(true);
    NimBLEScanResults res = scan->getResults(10 * 1000, false);
    for (int i = 0; i < res.getCount(); i++) {
        const NimBLEAdvertisedDevice *d = res.getDevice(i);
        if (!d->isAdvertisingService(NimBLEUUID((uint16_t)0x180D))) continue;
        NimBLEClient *cli = NimBLEDevice::createClient();
        if (cli->connect(d)) { Serial.printf("BLE connected %s\n", d->getAddress().toString().c_str()); break; }
        NimBLEDevice::deleteClient(cli);
    }
    startScan();
    ant.phy.mode = ANT_ESPPHY_MODE_RX;
}

void setup() {
    Serial.begin(115200); delay(1500);
    NimBLEDevice::init("ant-bench");
    ant_node_config_t cfg = {}; cfg.on_data = on_ant_data; cfg.store = &ant_node_store_ram; cfg.coexist = true;
    ant_espphy_status_t s = ant_node_start(&ant, &cfg);
    Serial.printf("-- ANT: start %s --\n", ant_espphy_status_str(s));
    startScan();                                   /* AFTER the node: window hooks captured */
    ant_node_open_antplus_slave(&ant, 0, ANTPLUS_DEVTYPE_HRM, 0, ANTPLUS_PERIOD_HRM);
    Serial.println("knobs: fmt XX | patch XX | crc keep|off | ch N | mhz N | tap N | ble | status");
}

static void status() {
    const ant_espphy_t *r = &ant.phy;
    Serial.printf("[%lus] ANT+ %u bpm dev %u rssi %d pages %lu | cs=0x%x fmt=%02x patch=%02x crc=%s ch=%u\n",
                  (unsigned long)(millis()/1000), ant_bpm, ant_device, ant_rssi, (unsigned long)ant_pages,
                  r->cs_off, r->cs_fmt_override, r->patch_mask, r->win_keep_crc ? "keep" : "off", r->coexist_ch);
    Serial.printf("   sched=%lu windows=%lu aborts=%lu(mode %02x win %ums minevt %u -> %luus llend %lu min %luus) scanrx=%lu rxhook=%lu frames=%lu matched=%lu lasthdr=%08lx\n",
                  (unsigned long)r->sched_hook_calls, (unsigned long)r->win_hook_calls, (unsigned long)r->aborts, r->abort_mode, r->win_len_ms, r->win_minevt, (unsigned long)r->win_len_us, (unsigned long)r->win_ll_ended, (unsigned long)r->win_ll_min_us,
                  (unsigned long)r->scan_rx_isr_calls, (unsigned long)r->rx_hook_calls,
                  (unsigned long)r->rx_frames, (unsigned long)r->rx_matched, (unsigned long)r->scan_rx_isr_hdr);
}

void loop() {
    static char line[48]; static int n = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            line[n] = 0; n = 0;
            char *cmd = strtok(line, " "); char *arg = strtok(nullptr, " ");
            if (!cmd) continue;
            if (!strcmp(cmd, "fmt")) ant.phy.cs_fmt_override = arg ? strtoul(arg, 0, 16) : 0;
            else if (!strcmp(cmd, "patch")) ant.phy.patch_mask = arg ? strtoul(arg, 0, 16) : 0;
            else if (!strcmp(cmd, "crc")) ant.phy.win_keep_crc = arg && !strcmp(arg, "keep");
            else if (!strcmp(cmd, "ch")) ant.phy.coexist_ch = arg ? atoi(arg) : 0;
            else if (!strcmp(cmd, "mhz")) ant.phy.mhz_override = arg ? atoi(arg) : 0;
            else if (!strcmp(cmd, "tap")) { tapLeft = arg ? atoi(arg) : 8; ant.phy.tap_on = true; }
            else if (!strcmp(cmd, "abort")) ant.phy.abort_mode = arg ? strtoul(arg, 0, 16) : 0;
            else if (!strcmp(cmd, "winms")) ant.phy.win_len_ms = arg ? atoi(arg) : 0;
            else if (!strcmp(cmd, "csw")) { char *v = strtok(nullptr, " "); if (arg && v && ant.phy.cs_ovr_n < 4) { ant.phy.cs_ovr[ant.phy.cs_ovr_n].off = atoi(arg); ant.phy.cs_ovr[ant.phy.cs_ovr_n].val = strtoul(v, 0, 16); ant.phy.cs_ovr_n++; } else ant.phy.cs_ovr_n = 0; }
            else if (!strcmp(cmd, "scan")) { scanSlots = arg ? atoi(arg) : 16; NimBLEDevice::getScan()->stop(); startScan(); }
            else if (!strcmp(cmd, "adv")) { NimBLEAdvertising *a = NimBLEDevice::getAdvertising(); a->setName("ant-bench"); if (arg) { a->setMinInterval(atoi(arg)); a->setMaxInterval(atoi(arg)); } Serial.printf("adv start %d\n", a->start()); }
            else if (!strcmp(cmd, "keephdr")) ant.phy.keep_rx_hdr = arg && !strcmp(arg, "on");
            else if (!strcmp(cmd, "advoff")) NimBLEDevice::getAdvertising()->stop();
            else if (!strcmp(cmd, "ble")) connectBle();
            status();
        } else if (n < (int)sizeof(line) - 1) line[n++] = c;
    }
    ant_espphy_rx_t f; bool m;
    while (tapLeft > 0 && ant_espphy_tap_poll(&ant.phy, &f, &m)) {
        tapLeft--; Serial.printf("[tap] len=%u rssi=%d %s:", f.len, f.rssi, m ? "MATCH" : "     ");
        for (int i = 0; i < f.len; i++) Serial.printf(" %02x", f.body[i]);
        Serial.println(); if (!tapLeft) ant.phy.tap_on = false;
    }
    static uint32_t last = 0;
    if (millis() - last > 5000) { last = millis(); status(); }
    delay(20);
}
