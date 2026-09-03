# Figuring out the ANT PHY

You asked me to figure out the physical layer. Here is what the ANT radio
actually does on the air, how confident I am in each part, and what it means for
the ESP32.

## TL;DR

**ANT's PHY is Nordic Enhanced ShockBurst.** It is not an exotic custom
waveform — it is the same packet engine as the nRF24L01+, which is exactly why
Nordic's nRF24AP2 (an nRF24 die with ANT firmware) is *the* ANT chip. Once you
know that, "native ANT" stops being about cracking Espressif's radio and becomes
about driving a ShockBurst radio, which a **$1 nRF24L01+** does in hardware.

## The on-air frame

```
+--------------+---------------+----------------+-------------+
| Preamble (1) | Address (3-5) | Payload (1-32) | CRC-16 (2)  |
+--------------+---------------+----------------+-------------+
                \___________ CRC is computed over this _______/
```

| Parameter | Value | Confidence | Source |
|---|---|---|---|
| Modulation | GFSK | High | Nordic spec; multiple RE efforts |
| Bit rate | 1 Mbit/s | High | Nordic spec; rtl_433 thread |
| Deviation | ~220 kHz (measured on a commercial HRM strap with the HackRF; ~160 kHz per rtl_433) | **Verified** | this bench |
| Preamble | 1 byte, `0xAA` if first address bit = 1 else `0x55` | High | Enhanced ShockBurst rule (Nordic spec) |
| Address (sync word) | ANT: 6 bytes = 2-byte network marker + devnum(2) + devtype + trans, MSB-first | **Verified** | captured strap + ESPwn32 |
| Payload | 1–32 bytes | High | ShockBurst |
| CRC | CRC-16/CCITT-FALSE, poly `0x1021`, init `0xFFFF`, no reflection, no final xor, over address+payload, big-endian | **Verified** | `packets.h` `crc16()`, reproduced and unit-tested here |
| Whitening | **None** (unlike BLE) | High | The RE decoder CRC-checks raw bytes with no dewhitening |
| Bit order | MSB-first | High | `ExtractByte()` shifts `<< (7-c)` |
| ANT+ RF frequency | 2457 MHz (single channel, no hop) | High | ANT+ spec; rtl_433 thread |
| General ANT | frequency hopping, base 2466 MHz | High | rtl_433 thread |

The CRC is the one piece I could prove rather than just cite. The reverse-
engineered decoder uses CRC-16/CCITT-FALSE; our `ant_crc16_ccitt()` reproduces
it and the test suite pins it to that algorithm's standard check value
(`crc("123456789") == 0x29B1`). See `test/host/test_phy.c`.

## The channel-ID → address derivation (now known)

A commercial ANT+ sensor's address is **not** a secret hash: it is the channel
ID itself, prefixed by a 2-byte *network marker* derived from the 8-byte
network key. Our bench HRM strap (device 0x6941) sends

```
AA | a6 c5 | 69 41 | 78 | 01 | 0a <8-byte page> | crc16
```

and the DEF CON ANT-FS beacon `3B A3 47 24 01` is the same thing on the ANT-FS
network (marker `3b a3`). The marker function is the one inside the nRF52 ANT
SoftDevice, recovered by ESPwn32 (Cayre & Cauquil, WOOT 2023): ANT+ key →
`a6 c5`, ANT-FS → `3b a3`, public → `5b 25`; implemented in
`ant_sb_link_network_marker()` and mirrored in `tools/antcap/antframe.py`.
The byte after the address is `0x0a` in every broadcast a sensor sends.

## What's still proprietary

1. **Adaptive isochronous coexistence** — the exact hop schedule and TDMA slot
   timing general ANT uses across its 3 frequencies. ANT+ single-channel
   broadcast (HR, power, cadence) does not need the hop, so this matters less
   for the common profiles.

ANT+ single-channel broadcast is the case that matters, and there the format
above is complete: the ESP32-S3 both receives the commercial strap and
transmits frames the HackRF decodes as ANT+.

## Can the ESP32's *internal* radio do it?

This is the whole point of the pure-ESP32 build, and the answer is now **yes,
on the ESP32-S3/C3** — measured, not argued. The path that got there is not
the one first tried:

1. **The libphy path (original ESP32).** Reverse-engineering `libphy` (see
   [`docs/TIER3_MODEM_RE.md`](TIER3_MODEM_RE.md)) recovered an app-linkable,
   MHz-linear frequency setter (`set_chan_freq_sw_start`) that reaches
   2457 MHz. It gave a carrier, but no packet path: keying an arbitrary
   ShockBurst bit stream through the closed modem was never validated. That
   code has been removed from the build.
2. **The BLE-controller path (S3/C3), following ESPwn32 / esperanto.** The
   RW-BLE link-layer core is a generic 1 Mbit/s GFSK modem. Its per-event
   control structure lives in exchange memory the CPU can write, and its
   link-layer functions are called through a writable table (`r_ip_funcs_p`).
   Run the controller's own LE test mode, hook the event-start and RX-interrupt
   functions, rewrite the control structure (2457 MHz via the frequency table,
   1 Mbit/s, the ANT sync word, CRC and whitening off) and read the raw bytes
   out of the RX descriptor. The MAC above needs no change.

Measured with this repo's code: the commercial strap above is received and
tracked through the full stack on an S3 (66–68 bpm, RSSI −56 dBm), and S3
transmissions at 2457 MHz decode on the HackRF as ANT+ frames. Two
controller facts worth recording: LE test mode runs with **whitening off**
(bytes are only bit-reversed — dewhitening them corrupts the frame), and the
core treats the byte after the sync word as a **length**, so a wildcard sync
(`aa aa a6 c5`) cannot receive the ~5 % of device numbers whose low byte
bit-reverses to < 13.

## What the PHY discovery unlocks: pure-ESP32 ANT

The pipeline this repo ships:

- [`ant_phy_shockburst.c`](../components/ant/src/ant_phy_shockburst.c) — portable,
  host-tested builder/verifier for the exact ANT air format above (CRC, preamble,
  address packing).
- [`ant_sb_link.c`](../components/ant/src/ant_sb_link.c) — maps an ANT channel
  identity to the RF frequency (2457 MHz), the network marker and the sync
  word; pinned to the strap's bytes in the host tests.
- [`ant_mac.c`](../components/ant/src/ant_mac.c) — the complete ANT protocol
  engine (channel timing, search/track, broadcast/acknowledged/burst, networks,
  8 channels) over a small PHY vtable; proven over the virtual air
  [`ant_air.c`](../components/ant/src/ant_air.c) on the host and on target.
- [`ant_embedded.c`](../components/ant/src/ant_embedded.c) — presents the
  engine as an ANT serial network processor so the ordinary host stack and
  ANT+ profiles run unchanged.
- [`ant_espphy.c`](../components/ant/radio/ant_espphy.c) — the ESP32-S3/C3
  BLE core as that PHY, as described above; receive live, transmit on air.
- [`ant_node.c`](../components/ant/radio/ant_node.c) — radio + engine +
  FreeRTOS task behind a thread-safe application API, with the BLE hand-back
  (`ant_node_stop()` returns the controller to NimBLE/Bluedroid).

The nRF24L01+ (whose ShockBurst engine is *why* we know the ANT format) is a
useful reference only; it is not part of this build — the target is the
ESP32's own radio.

### How to take it to hardware

1. Flash the firmware (`idf.py set-target esp32s3 && idf.py build flash
   monitor`); it self-tests the protocol engine and then searches for any
   ANT+ HRM on 2457 MHz. Wear a strap: `ch0 HRM #26945: 67 bpm ...`.
2. `idf.py -DANT_ROLE=sensor build` transmits a fake HRM; capture it with
   `tools/antcap` (`antdecode.py`) or point a real ANT+ display at it.
3. Open more channels (power, speed/cadence) on the same node; they share the
   receiver through the wildcard sync word and software address matching.

## Sources

- ANT ShockBurst demodulator + `packets.h` (CRC, framing):
  `github.com/sghctoma/antfs-poc-defcon24` (DEF CON 24), building on
  `github.com/omriiluz/NRF24-BTLE-Decoder`.
- rtl_433 issue #1990 "Decoding ANT and ANT+ packets (1 Mbps, GFSK, 160 kHz
  deviation)".
- Nordic nRF24L01+ Product Specification (preamble rule, CRC polynomials,
  1 Mbit/s deviation).
- Wikipedia "ANT (network)" (GFSK 1 Mbit/s, ShockBurst lineage).
