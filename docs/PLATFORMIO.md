# Using esp32-ant+ from PlatformIO / Arduino (and in the tdisplay bike computer)

`components/ant` is a PlatformIO library: `library.json` declares
`frameworks = arduino, espidf`, `platforms = espressif32`, builds `src/*.c` and
`radio/*.c`, and exports `include/`. Everything is C99 with `extern "C"`
headers, so a C++17 Arduino sketch includes `ant_node.h` directly.

Verified toolchain: `espressif32@6.5.0` (Arduino-ESP32 2.0.14 = ESP-IDF
4.4.6) — the same platform pin as `~/Documents/tdisplay`. The controller
hook-table indices the radio backend relies on were checked against that
core's `libbtdm_app.a` and are identical to ESP-IDF 5.4's; the backend also
sanity-checks them at runtime (the original entries must be ROM addresses)
and returns `ANT_ESPPHY_ERR_UNSUPPORTED` instead of hooking if they are not.

## Adding the library

```ini
lib_deps =
    h2zero/NimBLE-Arduino@^2.2.3
    symlink:///Users/raemond/Documents/esp32-ant+/components/ant
```

`symlink://` keeps the library live (edits in this repo are picked up by the
next `pio run`); `file://` copies it; a git URL works once it is pushed. No
`build_flags` are needed. The sketch's `-std=gnu++17` does not matter to the
C sources.

Footprint on the S3: ~30 KB of flash, `ant_node_t` is 2.5 KB of static RAM,
the ANT task stack is 4 KB (configurable), plus the BT controller's own
internal-RAM allocation while the radio is up — the same ~50 KB NimBLE's
controller half already costs, and it is only ever one or the other.

## The one rule: the radio is exclusive

The ANT backend drives the BLE controller bare (its own VHCI callback, LE test
mode, hooked link-layer functions). No BLE host can be up at the same time,
and the controller cannot serve BLE and ANT alternately within a session —
it is one or the other until you switch.

```cpp
NimBLEDevice::deinit(true);          // NimBLE: stop host, disable + deinit controller
ant_node_start(&node, &cfg);         // ANT owns the radio
...
ant_node_stop(&node);                // closes channels, stops the task, deinits the controller
NimBLEDevice::init("name");          // NimBLE again
```

`ant_node_start()` checks `esp_bt_controller_get_status() == IDLE` and returns
`ANT_ESPPHY_ERR_BT` otherwise, so the wrong order fails at the call rather
than corrupting the controller. `NimBLEDevice::deinit()` in NimBLE-Arduino
2.x calls `nimble_port_deinit()`, which disables and deinits the controller —
verified in its source (`nimble_port.c`), so no extra `btStop()` is needed.
`examples/platformio_ble_handoff/` links both stacks and alternates every
20 s; it builds against the bike computer's platform pin.

`ant_node_stop()` waits for the ANT task to exit (≤ 500 ms), so call it from
a normal task, not from the ANT callbacks.

## API in one screen

```cpp
#include "ant_node.h"

static ant_node_t node;                      // static or heap; not on a task stack

static void on_data(ant_node_t *, const ant_node_rx_t *rx, const uint8_t page[8], void *user)
{
    // ANT task context, node lock held: copy out and return. rx->channel,
    // rx->device_num/type/trans (learned on wildcard acquisition), rx->rssi.
}
static void on_event(ant_node_t *, uint8_t ch, uint8_t event, void *user)
{
    // ANT_EVENT_RX_SEARCH_TIMEOUT, ANT_EVENT_RX_FAIL, ANT_EVENT_RX_FAIL_GO_TO_SEARCH, ...
}

static void on_paired(ant_node_t *, uint8_t ch, const ant_node_device_t *dev, bool remembered, void *user)
{
    // a wildcard search locked on: dev->device_num/type/trans; remembered = saved to the store
}

ant_node_config_t cfg = {};
cfg.on_data = on_data; cfg.on_event = on_event; cfg.on_paired = on_paired; cfg.user = this;
cfg.store = &ant_node_store_nvs;         // remember paired sensors across reboots (NVS "ant"/"known")
cfg.proximity_rssi = -70;                // only pair with sensors this close (dBm); 0 = any
// cfg.task_core = ANT_NODE_CORE_1 (default on dual-core), task_stack, task_priority
ant_espphy_status_t s = ant_node_start(&node, &cfg);

// ANT+ receivers: device number 0 = the remembered sensor of that type, else
// the first one heard (which is then remembered)
ant_node_open_antplus_slave(&node, 0, ANTPLUS_DEVTYPE_HRM,         0, ANTPLUS_PERIOD_HRM);
ant_node_open_antplus_slave(&node, 1, ANTPLUS_DEVTYPE_BIKE_POWER,  0, ANTPLUS_PERIOD_BIKE_POWER);
ant_node_open_antplus_slave(&node, 2, ANTPLUS_DEVTYPE_BIKE_SPDCAD, 0, ANTPLUS_PERIOD_BIKE_SPDCAD);

// "pair new" button: drop the saved HRM and search for a nearby one
ant_node_pair(&node, 0, ANTPLUS_DEVTYPE_HRM, ANTPLUS_PERIOD_HRM);
// known-device table: ant_node_known_device(), ant_node_known_devices(),
//   ant_node_remember_device(), ant_node_forget_device(), ant_node_forget_all()

// anything else: ant_node_open() with an ant_node_channel_cfg_t
//   (flags ANT_NODE_CH_IGNORE_KNOWN / ANT_NODE_CH_NO_REMEMBER, per-channel proximity_rssi)
// queries: ant_node_is_tracking(), ant_node_channel_id(), ant_node_channel_status()
// diagnostics: ant_node_radio(&node)->rx_frames / rx_matched / last_rssi ...
```

### Pairing

ANT+ has no bonding: "pairing" is remembering a device number. The node does
this for you when a store is configured. A wildcard open (device number 0)
of a device type with a saved entry opens directly on that device — no
search, no chance of adopting a neighbour's strap on a group ride, and it
sidesteps the wildcard 5 % hole (ids whose low byte bit-reverses to < 13 are
invisible to the wildcard sync word). A wildcard open of an unknown type
searches, and the first device that locks is saved (`on_paired` with
`remembered = true`). One device per type, up to `ANT_KNOWN_MAX` (8).

Behaviour follows the head units: a saved sensor that is not there produces
`ANT_EVENT_RX_SEARCH_TIMEOUT` (if a search timeout is set) rather than a
silent switch to whatever else is around. Re-pairing is an explicit action —
`ant_node_pair()` — which ignores the saved entry, searches afresh and saves
the result. `cfg.proximity_rssi` (or the per-channel value) gates that first
acquisition by signal strength so "pair" picks the strap on your chest, not
the one across the room; ANT semantics apply, the threshold is not used when
reacquiring a sensor that dropped out.

Stores: `ant_node_store_nvs` (namespace `ant`, key `known`; the Arduino core
initialises NVS at boot, IDF projects call `nvs_flash_init()` first),
`ant_node_store_ram` (process lifetime; useful across BLE hand-offs when
flash writes are unwanted), or your own `ant_node_store_t {load, save, user}`
to keep the table in the bike computer's own settings file.

Pages decode with the profile helpers in `antplus_profiles.h`
(`antplus_hrm_decode`, `antplus_power_decode`, `antplus_spdcad_decode`, …).

## Fitting it into the bike computer

The bike computer today pairs BLE sensors by saved MAC (`ble_sensors`) and
serves the phone over BLE (`ble_server`, `ams_client`), all on NimBLE. ANT+ is
therefore a **mode**, not an addition:

1. **Sensor source setting**: `BLE` (current) or `ANT+`. In ANT+ mode the
   phone link is unavailable while sensors are live — that is a physical
   constraint of one radio, so the UI should say so rather than pretend.
2. **`ant_sensors` module** mirroring `ble_sensors`: `begin()` does
   `NimBLEDevice::deinit(true)` (if `ble_sensors`/`ble_server` are up) then
   `ant_node_start()` and opens one slave channel per kind (HR / power /
   speed-cadence). `on_data` decodes with `antplus_*_decode` and publishes
   into the same sensor-state structs the BLE path feeds, so the dashboard,
   FIT writer and ride recorder do not care which radio delivered a reading.
3. **Pairing**: built in (see above). Give the node a store — NVS, or a
   custom `ant_node_store_t` that writes into the same settings file as the
   BLE MACs — and wire the existing "pair sensor" UI to `ant_node_pair()`
   and "forget" to `ant_node_forget_device()`. Show `on_paired` /
   `ant_node_known_device()` in the sensor list the way the BLE MACs are
   shown today.
4. **Radio hand-back for the phone**: `ant_node_stop()` → `ble_server::begin()`
   when the user opens the phone-sync screen, and the reverse when leaving it.
   Both transitions are ~100 ms of controller init and the ANT search restarts
   (a sensor is reacquired within a couple of its periods, i.e. < 1 s).
5. **Memory**: nothing new to steer; the controller allocation reuses the
   internal RAM NimBLE's controller had. Keep the boot-order note in
   `main.cpp` (EPD fast buffer in PSRAM) as is.
6. **Sleep**: the ANT backend disables BT modem sleep and holds a continuous
   receive event, so `power_mgmt` should treat ANT-live like BLE-link-live
   (light sleep off).

What to expect on first bring-up: at desk range ~90 % of a strap's slots
decode; further away frames increasingly fail CRC (see the README's open
items) and the channel may briefly drop to search and reacquire. HR pages
still arrive several times per second, and the profile decoders and the ride
recorder do not need every slot. Power meters and speed/cadence sensors have
not been tested yet, only HRM.

## ESP-IDF projects

The same directory is an IDF component: add its parent to
`EXTRA_COMPONENT_DIRS`. `sdkconfig` needs `CONFIG_BT_ENABLED=y`; no host is
required (`CONFIG_BT_CONTROLLER_ONLY=y` is the smallest configuration). The
firmware in `main/` is the reference user.
