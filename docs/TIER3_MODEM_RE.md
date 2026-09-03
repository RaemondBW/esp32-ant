# Tier 3: reverse-engineering the ESP32 radio/modem interface

Goal: find whether the ESP32's *own* radio can be driven to inject raw ANT
frames — specifically the piece the public state of the art had **not** cracked.
Tarlogic's 2025 work ("Liberating Bluetooth on the ESP32", 39C3) mapped the BT
link-layer controller (registers at `0x3ff71000`, exchange memory at
`0x3ffb0000`) but states explicitly: *"The interfaces used to talk to [the]
Radio, Modem... have not yet been reverse engineered."* That modem interface —
frequency, data rate, deviation — is exactly the ANT blocker. This is a
first-pass reverse-engineering of it.

## Method

Static analysis of the ESP-IDF v5.4 esp32 blobs with the xtensa toolchain
(`nm`, `objdump -dr`, `objdump -s`): `libphy.a`, `librftest.a`,
`libbttestmode.a`, `librtc.a`, `libbtdm_app.a`. No hardware, no Ghidra — just
symbol tables, disassembly, and relocation records. All of it reproducible from
the blobs in your IDF install (`components/esp_phy/lib/esp32/`,
`components/bt/controller/lib_esp32/esp32/`).

## Key findings

### 1. The RF synthesizer is programmed by an app-linkable symbol

The frequency-set call chain in `libphy.a` (`phy_chip_v7_ana.o`) is:

```
chip_v7_set_chan_offset
   -> set_channel_rfpll_freq(chan, ...)          [global T]
        -> set_chan_freq_sw_start(freq_byte, a3, a4)   [global T]  <-- the metal
```

`set_chan_freq_sw_start` is a **global text symbol**, so user firmware can call
it directly. We proved this: forcing the linker to resolve it
(`-u set_chan_freq_sw_start`) pulls it into the app image from
`libphy.a(phy_chip_v7_ana.o)` at address `0x400dc604`. It is not hidden or
controller-private — it is reachable from `app_main`.

### 2. The frequency argument is ~1-MHz-linear (odd MHz reachable)

From `chan_to_freq` and `set_channel_rfpll_freq`, the WiFi path computes the
freq word as `5 * (channel - 1)` (WiFi's 5 MHz channel spacing), i.e. the word
is linear in MHz with channel 1 (2412 MHz) at word 0. That means:

- The 2 MHz BLE channel grid is a *link-layer* constraint, not a synth
  constraint. The synth setter takes an arbitrary MHz-linear byte.
- ANT+'s **2457 MHz** — odd, unreachable via BLE test-mode channel indexing —
  maps to freq byte ~`45` and is representable here.

This defeats the single hardest ANT blocker (odd-MHz frequency) *in principle*.
The exact base/scale needs bench confirmation (see "What's left").

### 3. Concrete modem/RF registers (previously undocumented)

`set_chan_freq_sw_start` reads/writes real RF registers via `memw`-barriered
`s32i`/DPORT reads:

| Register | Role (inferred) |
|---|---|
| `0x3FF4E0C4` | RF freq/command data port — receives the frequency-derived bytes in sequence |
| `0x3FF4E168` | RF/DPORT status read during the freq set |
| `0x3FF5D008` | chip-rev/status; bits [31:29] select a calibration path |

It also calls `phy_dis_hw_set_freq`, `correct_rfpll_offset`,
`write_wifi_chan_data`. The synthesizer cap is additionally programmed over the
ESP32's **internal analog I²C** bus (`check_rfpll_write_i2c` -> `bt_get_i2c_data`
-> ROM `rom_i2c_writeReg`), confirmed by `i2c_rfpll_init` / `get_rf_freq_cap`.

### 4. What is stubbed out

`rf_rw_reg_rd` / `rf_rw_reg_wr` in `libbttestmode.a` — the generic "RF register
read/write" API one would hope to abuse — are shipped as **empty stubs**
(`rd` returns 0, `wr` is a `retw.n`). So the clean documented path is disabled;
the real access is the `libphy` internals above.

## What this repo ships from it

Nothing, any more — and that is the finding. The recovered
`set_chan_freq_sw_start` was wired up as `ant_native_phy` (strongly linked so
`libphy` really was in the ELF) and did put a carrier where asked, but there
was no validated way to key a raw bit stream through the closed modem. The
original-ESP32 libphy path was dropped in favour of the S3/C3 BLE controller,
where the packet engine itself is reachable: the RW-BLE link-layer core's
control structures live in CPU-writable exchange memory and its functions are
called through the writable `r_ip_funcs_p` table (ESPwn32 / esperanto, Cayre
et al., WOOT 2023). `ant_espphy.c` hooks the LE-test-mode event start and RX
interrupt, rewrites the control structure (frequency-table entry 39 → 2457 MHz,
1 Mbit/s, ANT sync word, CRC/whitening off) and reads the RX descriptors. See
`docs/SHOCKBURST_LINK.md` for the mechanism and the measurements.

Indices used (identical in ESP-IDF 4.4.6 and 5.4 `libbtdm_app.a` for the
S3): 127 `r_lld_test_evt_start_cbk`, 132 `r_lld_test_rx_isr` (also 128
`r_lld_test_freq2chnl`, 134/135 `r_lld_test_start/stop`, 258
`r_lld_scan_process_pkt_rx`, 268 `r_lld_scan_sched`). Exchange memory via
`r_emi_get_mem_addr_by_offset`: frequency table at 0x100, control structure at
0x400, RX descriptors at 0x1000 (stride 20), RX FIFO index at `p_lld_env`
byte 0xd8.

## What's left (honest)

1. **Original ESP32 (Xtensa LX6).** Its BT controller is a different RW core
   generation; the exchange-memory layout and hook table have not been mapped
   here. The libphy frequency setter above is the starting point if anyone
   wants to try the packet path on that chip.
2. **ESP32-C6 / H2.** Espressif's own ESP-BLE controller, not RW-BLE; the same
   idea (test mode + hooks) needs its own entry points.
3. **S3 receiver under interference** — sensitivity itself is not the
   limit (HackRF-synthesised strap decodes to −76 dBm at either deviation,
   the strap's replayed waveform to −81 dBm), but a narrowband signal in
   the channel flips bits in the first bytes after the sync word, where the
   demodulator's offset estimate has not yet converged; ANT's unwhitened
   frames give it nothing to help. See the README's open items for the
   measurements. Untried: the RW-BLE control structure's receiver options
   for the ANT event.

## Assessment

The modem/RF frequency interface on the original ESP32 — the part the public
work had *not* reverse engineered — was partially mapped: the frequency setter
identified, app-linkable, MHz-linear, and three RF register addresses
recovered. It turned out to be the wrong door. The S3/C3 controller route,
where the whole packet engine is exposed, is what carried ANT+ end to end.

## Reproduce

```sh
cd <scratch>/re
nm -A ~/esp-idf/components/esp_phy/lib/esp32/libphy.a | grep set_chan_freq_sw_start
# extract + disassemble
ar x libphy.a phy_chip_v7_ana.o
xtensa-esp32-elf-objdump -dr --disassemble=set_chan_freq_sw_start phy_chip_v7_ana.o
```
