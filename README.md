# esp32-ant+

A **pure-ESP32 ANT / ANT+ implementation**: the ESP32 runs the complete ANT
protocol engine in portable C and drives its **own** 2.4 GHz radio as the PHY —
no external ANT module, no nRF24, no companion SPI radio, no add-on silicon.

It **receives a real, commercial ANT+ heart-rate strap on an ESP32-S3** and
tracks it through the full stack (search → acquire → HRM pages with RSSI).
Ships as an ESP-IDF component **and** a PlatformIO/Arduino library, with a
host-runnable test suite (100 tests / 1079 checks, clean under ASan+UBSan), a
boot-time on-target self-test, a flashable ESP-IDF firmware and two PlatformIO
examples.

## Status

| Layer | Status |
|---|---|
| ANT protocol engine (`ant_mac`): 8 channels, 3 networks, 32768 Hz TDMA timing, master/slave, wildcard search + id learning, tracking / `RX_FAIL` / `GO_TO_SEARCH` / reacquire, search timeouts, broadcast / acknowledged / burst both ways with retries | ✅ host-tested + on-target self-test |
| ANT serial bridge (`ant_embedded`) + host-side stack (`ant_stack`), ANT+ profiles (HRM, bike power, speed & cadence) | ✅ host-tested |
| Real ANT+ air format (`ant_sb_link`, `ant_phy_shockburst`): `AA \| a6 c5 \| dev_hi dev_lo \| type \| trans \| 0x0A + page \| CRC-16` | ✅ confirmed by SDR decode of a Garmin-compatible strap and by live reception |
| **Receive on the ESP32-S3/C3 radio** (`ant_espphy`) | ✅ **live**: a worn HRM strap (device 0x6941) tracked at 66–68 bpm, RSSI −56 dBm, the hook sees ~every slot; synthesised/replayed strap decodes to −76/−81 dBm; interference episodes flip bits after the sync word, see below |
| Transmit on the ESP32-S3/C3 radio | ✅ raw frames on the air at 2457 MHz (SDR-verified); end-to-end reception by a commercial display not yet tested |
| ESP32-C6 | ❌ different controller; `ant_espphy_init` returns `ANT_ESPPHY_ERR_UNSUPPORTED` (next step) |
| Application API (`ant_node`): radio + engine + FreeRTOS task, thread-safe, BLE hand-back | ✅ compiles on ESP-IDF 5.4 and on Arduino-ESP32 2.0.14 (IDF 4.4.6); the S3 run above used it |
| Pairing: known-device table saved to NVS (or a custom store), auto-reconnect to the saved sensor on wildcard open, explicit `ant_node_pair()` re-pair, ANT pairing bit, RSSI proximity search | ✅ **verified on the strap**: paired + saved on first search, reconnected from NVS 70 ms after the next boot with no search, −65 dBm proximity gate ignored the strap at −70 and paired it at −61 |

### How the radio works (short version)

The ESP32-S3/C3 BLE controller is an RW-BLE link-layer core: a generic
1 Mbit/s GFSK modem whose per-event control structure lives in CPU-writable
RAM and whose link-layer functions are called through a writable table
(`r_ip_funcs_p`). Following ESPwn32 / esperanto (Cayre et al., WOOT 2023) we
run the controller's own **LE test mode** (receiver test / transmitter test on
"channel 39") and hook two of its functions:

- **event start** — after the controller has programmed the test event we
  rewrite the control structure: 2457 MHz, 1 Mbit/s, ANT sync word, CRC and
  whitening off;
- **RX interrupt** — we lift the raw bytes out of the RX descriptor, bit-reverse
  them (test mode runs with whitening off; nothing else is needed) and queue
  them for the MAC.

That is exactly ANT's ShockBurst air format, so `ant_mac` runs on top
unchanged. A slave channel with a known device number programs the sync word
`a6 c5 dev_hi dev_lo`; a wildcard search programs `aa aa a6 c5` (preamble +
network marker) and the address is matched in software, which is also what
lets several slave channels (HRM + power + cadence) share the one receiver.

Two consequences of the mechanism:

- **The radio is exclusive — or shared.** By default the controller runs bare,
  with our own VHCI callback and no BLE host: a BLE host (NimBLE, Bluedroid)
  must be stopped before `ant_node_start()`, and `ant_node_stop()` releases the
  controller for it again (`ant_node_start()` refuses with `ANT_ESPPHY_ERR_BT`
  if a host still owns it). There is also a **coexist mode** (`cfg.coexist =
  true`, `ant_espphy_init_coexist()`) that receives ANT *alongside* a running
  BLE host by hooking the controller's scan path and de-whitening the ANT
  payload in software — no global radio state is touched, so BLE connections
  keep working; BLE scanning pauses while an ANT receive is open, and coexist
  is receive-only. Verified on the S3 with NimBLE scanning and an ANT+ strap
  tracking at the same time.
- **One direction per start.** Switching between the receiver and transmitter
  test costs milliseconds of HCI traffic, so the backend is sticky: a node with
  slave channels receives, a node with a master channel transmits. A master
  transmits but never hears replies (broadcasts do not need any).

Details: [`docs/SHOCKBURST_LINK.md`](docs/SHOCKBURST_LINK.md) (air format and
what was verified), [`docs/PHY_FINDINGS.md`](docs/PHY_FINDINGS.md) and
[`docs/TIER3_MODEM_RE.md`](docs/TIER3_MODEM_RE.md) (the reverse-engineering
trail, including the earlier libphy path).

## Architecture

```
  your app (Arduino sketch / PlatformIO / ESP-IDF)      ANT+ profiles (antplus_hrm_*, ...)
        │
  ant_node   (radio + engine + FreeRTOS task; thread-safe API; BLE hand-back)
        │
  ant_mac    (the ANT protocol engine: timing, search/track, ack/burst, networks)
        │  ant_phy_t: tune / tx / rx_config / rx_enable / rx_poll
  ┌─────┴──────────┐
  ant_air           ant_espphy  →  ESP32-S3/C3 BLE core in LE test mode (hooked)
  (virtual air:     (the ESP32's own 2.4 GHz radio)
   tests/self-test)

  ant_stack → ant_embedded → ant_mac : the same engine behind an ANT serial-chip
  interface, for code written against a classic ANT network processor.
```

## Using it

### Arduino / PlatformIO (e.g. the tdisplay bike computer)

`components/ant` is a PlatformIO library (`library.json`). Point `lib_deps` at
it:

```ini
[env:esp32s3]
platform = espressif32@6.5.0
board = esp32-s3-devkitc-1
framework = arduino
lib_deps =
    symlink:///path/to/esp32-ant+/components/ant      ; or file://, or a git URL
```

```cpp
#include "ant_node.h"

static ant_node_t node;

static void on_data(ant_node_t *, const ant_node_rx_t *rx, const uint8_t page[8], void *) {
    antplus_hrm_data_t hr;
    if (rx->device_type == ANTPLUS_DEVTYPE_HRM && antplus_hrm_decode(page, &hr))
        Serial.printf("HRM #%u: %u bpm (rssi %d)\n", rx->device_num, hr.computed_heart_rate, rx->rssi);
}

void setup() {
    Serial.begin(115200);
    ant_node_config_t cfg = {};
    cfg.on_data = on_data;
    ant_node_start(&node, &cfg);                                   // NimBLE must be deinit'd first
    ant_node_open_antplus_slave(&node, 0, ANTPLUS_DEVTYPE_HRM, 0, ANTPLUS_PERIOD_HRM);  // 0 = any HRM
    // more channels share the radio:
    // ant_node_open_antplus_slave(&node, 1, ANTPLUS_DEVTYPE_BIKE_POWER, 0, ANTPLUS_PERIOD_BIKE_POWER);
}
void loop() { delay(1000); }
```

Callbacks run on the ANT task (priority `configMAX_PRIORITIES-2`, core 1 by
default) under the node lock: copy the page out, do not block.

Examples, both compiled against `espressif32@6.5.0` / Arduino (the bike
computer's toolchain) with `~/.platformio/penv/bin/pio run`:

- [`examples/platformio_hrm/`](examples/platformio_hrm/) — HRM display (or
  `-e esp32s3-sensor`: a fake HRM sensor).
- [`examples/platformio_ble_handoff/`](examples/platformio_ble_handoff/) —
  NimBLE-Arduino and the ANT stack in one binary, alternating ownership of the
  radio (`NimBLEDevice::deinit(true)` → `ant_node_start()` … `ant_node_stop()`
  → `NimBLEDevice::init()`).
- [`examples/platformio_ble_coexist/`](examples/platformio_ble_coexist/) —
  NimBLE and ANT+ **live at the same time** on one radio: NimBLE stays up and
  scanning (and can hold BLE connections) while ANT+ receives the strap via
  `cfg.coexist = true`. This is the better shape for the bike-computer
  integration; see [`docs/PLATFORMIO.md`](docs/PLATFORMIO.md).

### ESP-IDF

Add `components/ant` to `EXTRA_COMPONENT_DIRS` (or copy it into `components/`),
`#include "ant_node.h"` and use the same API. The controller must be enabled
in `sdkconfig` (`CONFIG_BT_ENABLED=y`, `CONFIG_BT_CONTROLLER_ONLY=y` is fine —
no host is needed).

### Lower levels

`ant_mac` (`include/ant_mac.h`) is the engine itself — poll-driven,
allocation-free, portable — and `ant_stack` → `ant_embedded` presents it as an
ANT serial chip for code written against one. Both are documented in their
headers and exercised by `test/host`.

## Test it right now (no hardware)

```sh
cd test/host
make                                    # 100 tests / 1079 checks
CC="cc -fsanitize=address,undefined" make BIN=ant_tests_asan   # same, sanitized
```

Expected tail:

```
tests: 100   checks: 1079   failures: 0
RESULT: PASS
```

What the suite covers, by layer:

- **messages / profiles / legacy channel stack** — checksum, encode/parse,
  streaming-parser resync, HRM / Bike Power / Speed&Cadence pages including
  rollover math, channel bring-up against a simulated chip.
- **PHY frame** — the real ANT+ air format build/verify round-trip pinned to
  bytes decoded from a commercial strap, CRC, identity → frequency + sync
  word mapping.
- **MAC over virtual air (`test_mac.c`)** — exact master grid; wildcard
  acquisition + id learning; slave receive on the master's grid; specific-id
  filtering; dropped/corrupted frames → `RX_FAIL` but tracking kept; master
  gone → `GO_TO_SEARCH` → reacquire; search timeout closes at exactly 81920
  ticks; acknowledged both directions; bursts 40 B → 5 packets, 128 B → 16
  packets, surviving lost and corrupted packets; RX-only slaves never transmit;
  network key and RF frequency separation; 3 mixed-role channels on one radio;
  every command's response code; 32-bit time wrap; late ticks; pairing bit
  transparent on the air but selective in search; per-channel RSSI; proximity
  search picks the near master and is not applied when reacquiring.
- **Paired-device table (`test_known.c`)** — one device per type, replace on
  re-pair, full-table and wildcard rejection, remove order, 4-byte-per-entry
  blob round trip with truncated/garbage entries dropped.
- **Serial bridge (`test_embedded.c`)** — `ant_stack` master and slave bring-up
  with the real response sequence; HRM pages end to end with `EVENT_TX` reload;
  REQUEST replies; host burst assembly and `TRANSFER_TX_COMPLETED`; RESET →
  STARTUP; search timeout as `RX_SEARCH_TIMEOUT` + `CHANNEL_CLOSED`; FIFO
  overflow accounting; the boot self-test itself.

## Build & flash the ESP-IDF firmware

Requires ESP-IDF v5.4 (installed at `~/esp-idf` on this machine).

```sh
. ~/esp-idf/export.sh
cd esp32-ant+
idf.py set-target esp32s3            # or esp32c3
idf.py build
idf.py -p /dev/cu.usbmodemXXXX flash monitor
```

At boot the firmware runs the protocol self-test (sensor ↔ display over the
virtual air on the chip: acquisition, 8 pages, one acknowledged transfer, one
40-byte burst) and logs `PASS`/`FAIL`, then starts `ant_node` on the internal
radio:

- default: **HRM display** — wildcard slave, infinite search; logs every HRM
  page (`ch0 HRM #26945: 67 bpm beats=… rssi=-56`) and, every 5 s, the radio
  counters and the last on-air frame.
- `idf.py -DANT_ROLE=sensor build`: **HRM sensor** (device #0x3042, 2457 MHz,
  8070-tick period), HR sweeping 60..99 bpm.

The radio counters to watch: `evt` (test-event starts; 1 in RX mode, since the
receiver test never ends), `rx hook a/b` (RX interrupts with / without a
packet), `frames` / `matched` (bodies received / passing the address match),
`sync rw` (live sync-word changes on acquire / lose).

## Bench validation — with a HackRF

[`tools/antcap/`](tools/antcap/README.md) is the SDR side: a numpy-only
GFSK/ShockBurst decoder (`antdecode.py`), a frame synthesiser for
`hackrf_transfer -t` (`antgen.py`), a carrier locator (`tone.py`) and capture /
transmit wrappers. `make -C tools/antcap check` proves it without hardware: the
Python frame builder is diffed byte-for-byte against the firmware's C, and
synthetic captures with noise and carrier error decode back to the exact bytes
on the exact ANT grid.

What has been done with it: decoded a commercial strap's frames (which is how
the air format was pinned), verified the ESP32-S3's own transmissions at
2457 MHz, and played a synthesised "fake strap" at the S3 receiver.

## Open items

- **Interference, not sensitivity.** The receiver hook sees essentially every
  slot of a strap (gaps of exactly 8070 ticks), and the measured sensitivity
  is fine: a synthesised strap from the HackRF decodes 100 % (250 kHz
  deviation) / ~85 % (ANT's 165 kHz) down to −76 dBm, and the strap's own
  recorded waveform replayed at −68…−81 dBm decodes 92 %. What was first read
  as a "cliff at −60 dBm" turned out to be episodes of 5–10 s in which every
  frame arrived with one or two flipped bits **in the first four bytes after
  the sync word** (`78 01 0a 80` → `68 01 4a a0` …) while the rest of the
  frame was intact — the same damage a weak CW tone in the channel produces
  on this demodulator (its frequency-offset estimate is biased at sync and
  converges over the next ~30 bits; ANT has no whitening to help). The MAC
  rides through such episodes (`RX_FAIL` → `GO_TO_SEARCH` → reacquire) and
  `crc_fail_count` per channel counts them. Build with `-DANT_LOG_FRAMES=1`
  to print every hooked frame with RSSI, CRC verdict and inter-frame gap and
  see for yourself; the ±2 MHz around 2457 MHz at this desk also carries a
  saturating Bluetooth Classic audio link and WiFi.
- **Wildcard search cannot see ~5 % of device numbers** — in wildcard mode the
  core reads `dev_lo` as the packet length; values whose bit-reversal is < 13
  are dropped by hardware. Give those devices' numbers explicitly.
- **TX interop** with a commercial ANT+ display (raw frames are on the air;
  needs a display to confirm).
- **ESP32-C6** — a different (ESP-BLE) controller; the same idea needs its own
  hook points.

## Layout

```
components/ant/
  library.json  PlatformIO manifest (frameworks arduino + espidf)
  CMakeLists.txt ESP-IDF component
  include/      public headers (ant_node.h is the application API)
  src/          portable core (no ESP deps; host-tested):
                ant_message, ant_channel, antplus_profiles, ant_phy_shockburst,
                ant_sb_link, ant_mac, ant_air, ant_embedded, ant_selftest
  radio/        ant_espphy (ESP32-S3/C3 BLE core as ant_phy_t), ant_node
                (radio + engine + task), ant_radio_loopback (simulated chip
                for the legacy serial-stack tests)
examples/       PlatformIO/Arduino projects (HRM display/sensor, BLE handoff, BLE coexist)
main/           ESP-IDF firmware (self-test, then HRM display or sensor)
test/host/      zero-dependency unit tests (cc + make)
tools/antcap/   HackRF bench tooling (decoder, frame synthesiser, tone locator; make check)
docs/           PLATFORMIO (integration), SHOCKBURST_LINK, PHY_FINDINGS, TIER3_MODEM_RE
```
