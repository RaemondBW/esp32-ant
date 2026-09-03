# Pure-ESP32 ANT: identity → air, on the internal radio

This is the pure-ESP32 path: **no external radio chip.** The ESP32 runs the
entire ANT MAC itself and drives its *own* 2.4 GHz radio as the PHY. The
pieces:

| Piece | File | Role | Status |
|---|---|---|---|
| identity → PHY params | `ant_sb_link.*` | ANT channel id → RF frequency + 6-byte sync word + control byte | ✅ host-tested, matches a commercial strap |
| on-air frame | `ant_phy_shockburst.*` | build/verify `preamble\|address\|payload\|CRC16` | ✅ host-tested, pinned to SDR-decoded bytes |
| protocol engine | `ant_mac.*` | full ANT MAC (timing, search/track, ack/burst, networks) over `ant_phy_t` | ✅ host-tested over virtual air |
| serial bridge | `ant_embedded.*` | the engine as an ANT serial chip for `ant_stack` | ✅ host-tested |
| ESP32 radio PHY | `ant_espphy.*` | `ant_phy_t` on the ESP32-S3/C3 BLE core | ✅ RX of a real strap live on the S3; TX on air |
| application API | `ant_node.*` | radio + engine + task, thread-safe, BLE hand-back | ✅ IDF 5.4 and Arduino (IDF 4.4.6) |

## The air format (real ANT+, confirmed)

ANT is Nordic (Enhanced) ShockBurst at 1 Mbit/s GFSK, MSB-first, no
whitening, CRC-16/CCITT-FALSE over address + payload:

```
AA | a6 c5 | dev_hi dev_lo | dev_type | trans_type | 0A p0 p1 p2 p3 p4 p5 p6 p7 | crc_hi crc_lo
 ^   ^^^^^   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^   ^^^^^^^^^^^^^
 |   network  channel id (4 bytes)                   control byte + 8-byte page    CRC-16
 |   marker
 preamble (one byte, MSB of a6 set -> 0xAA)
```

18 bytes on the air, 17 without the preamble. The 2-byte **network marker**
`a6 c5` is derived from the ANT+ network key; the derivation is Garmin's and
was reverse-engineered by ESPwn32 (Cayre & Cauquil, WOOT 2023) from the nRF52
SoftDevice. We use their result: `ant_sb_link` maps the ANT+ key to `a6 c5`.
The whole layout was confirmed on this bench two ways: decoding a
Garmin-compatible heart-rate strap (device 0x6941, type 120, trans 1) with a
HackRF and `tools/antcap/antdecode.py`, and then receiving that strap on an
ESP32-S3 through this code.

| ANT identity field | PHY parameter | How |
|---|---|---|
| `rf_freq` (e.g. 57) | RF frequency (MHz) | exact: `2400 + rf_freq`. ANT+ 57 → **2457 MHz**. |
| network key | address[0..1] | network marker (ANT+ → `a6 c5`) |
| device number / type / trans type | address[2..5] | `dev_hi dev_lo type trans` |
| broadcast page | payload[1..8] | after the control byte `0x0A` |

The **control byte** (payload[0]) is `0x0A` in every commercial broadcast. Its
other encodings (acknowledged, burst with sequence, slave → master reverse
direction) are our documented convention for ESP32 ↔ ESP32 links, see
`ant_sb_link.h`; a commercial receiver only ever needs the broadcast form.

## Receiving it on the ESP32-S3/C3

`ant_espphy` puts the BLE controller into its own LE receiver test and hooks
two link-layer functions through the writable `r_ip_funcs_p` table (index 127
`r_lld_test_evt_start_cbk`, 132 `r_lld_test_rx_isr`): the first rewrites the
event's control structure to 2457 MHz / 1 Mbit/s / the ANT sync word / CRC and
whitening off, the second lifts the bytes out of the RX descriptor. The
receiver wants a 4-byte sync word, so:

- **known device number** → sync `a6 c5 dev_hi dev_lo`, body starts at
  `dev_type`;
- **wildcard search** → sync `aa aa a6 c5` (preamble + marker), body starts at
  `dev_hi`, and the rest of the address is matched in software against each
  open slave channel's mask — which is what lets HRM, power and cadence
  channels share the receiver. The mask covers bits 0–6 of the device type
  only, so a master advertising with the ANT pairing bit (bit 7) set is
  still matched by an ordinary search; a slave that sets the bit in its own
  channel id matches only such masters (ANT pairing-bit semantics).

Two hardware facts learned the hard way:

- **Test mode runs with whitening off.** The bytes come out bit-reversed (the
  core stores LSB-first) and nothing else — applying BLE dewhitening, as one
  would for a scan-mode capture, corrupts them.
- **The byte after the sync word is a length to the core.** In wildcard mode
  that is `dev_lo`; device numbers whose low byte bit-reverses to a value
  below 13 (≈ 5 % of them) are dropped before the hook. Open those channels
  with the device number given.

Result on the bench: a worn strap tracked through the full stack at 66–68 bpm,
RSSI −56 dBm, every hooked frame matched, id learned as 0x6941/120/1. With the
exact 4-byte sync the hook sees essentially every slot (consecutive frames
8070 ticks apart), and the receiver decodes a HackRF-synthesised strap down
to −76 dBm and the strap's own replayed waveform to −81 dBm. What limits
reception in practice is interference: a narrowband signal in the channel
flips bits in the first bytes after the sync word (see the README's open
items). `-DANT_LOG_FRAMES=1` in the reference firmware prints every hooked
frame with RSSI, CRC verdict and gap for diagnosing this.

## Transmitting

The LE transmitter test on "channel 39" is retargeted the same way; `tx()`
drops the frame into the DTM payload buffer, so the next test packet carries
it, and clears it again a few ms later. DTM transmits back-to-back, so the
frame goes out several times per slot; ANT receivers ignore repeats. Raw
frames were verified on the air at 2457 MHz with the HackRF; reception by a
commercial ANT+ display has not been tried yet.

Switching between the receiver and transmitter tests costs milliseconds of
HCI traffic, so the backend is sticky per start: slave channels receive, a
master channel transmits, and a master never hears replies (the MAC copes;
broadcasts do not need any).

## Using it

```c
#include "ant_node.h"

static ant_node_t node;
ant_node_start(&node, &(ant_node_config_t){ .on_data = on_page });
ant_node_open_antplus_slave(&node, 0, ANTPLUS_DEVTYPE_HRM, 0, ANTPLUS_PERIOD_HRM);
```

or drive the engine directly:

```c
#include "ant_mac.h"
#include "ant_espphy.h"

static ant_espphy_t phy; static ant_mac_t mac;
ant_espphy_init(&phy);
ant_mac_init(&mac, ant_espphy_phy(&phy), on_data, on_event, NULL);
ant_mac_set_network_key(&mac, 0, ANTPLUS_NETWORK_KEY);
ant_mac_assign_channel(&mac, 0, ANT_CHANNEL_TYPE_SLAVE_RX, 0);
ant_mac_set_channel_id(&mac, 0, 0, ANTPLUS_DEVTYPE_HRM, 0);   // wildcard
ant_mac_set_channel_rf_freq(&mac, 0, ANTPLUS_RF_FREQ);      // 57 -> 2457 MHz
ant_mac_set_channel_period(&mac, 0, ANTPLUS_PERIOD_HRM);    // 8070 ticks
ant_mac_open_channel(&mac, 0);
for (;;) { uint32_t next = ant_mac_tick(&mac, ant_espphy_ticks()); ant_espphy_wait_rx(&phy, ms_until(next)); }
```

## Tests

`test/host/test_link.c` covers the RF-frequency mapping, the address layout
pinned to the bytes decoded from the strap, same/distinct-identity behaviour,
a full master → slave page round-trip and wrong-identity rejection;
`test_phy.c` the ShockBurst builder/verifier and CRC. `test_mac.c` and `test_embedded.c` cover the engine
and the serial bridge over the virtual air. `make -C test/host` (100 tests /
1079 checks).
