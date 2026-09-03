# antcap — HackRF bench tooling for the ESP32 ANT PHY

Everything needed to *see* what the ESP32 puts on the air at 2457 MHz and to
put known-good ANT frames on the air for the ESP32 to receive. Pure Python +
numpy on the host; the HackRF only moves samples.

```
antframe.py   the on-air frame in Python (CRC, preamble, ANT+ address layout, ANT+ pages)
gfsk.py       GFSK modem: modulate / burst-detect / demodulate, HackRF file I/O
antdecode.py  capture -> decoded frames (+ timing grid, carrier offset); --selftest
antgen.py     synthesise an ANT+ sensor's bursts -> .cs8 for hackrf_transfer -t
tone.py       averaged spectrum: where did the carrier land? (+ SDR ppm calibration)
capture.sh    hackrf_transfer -r wrapper (4 MS/s at 2457 MHz)
transmit.sh   hackrf_transfer -t wrapper (low power)
crosscheck.c  prints frames using the firmware's C; `make check` diffs it against antframe.py
```

## Prerequisites

```sh
brew install hackrf            # hackrf_info / hackrf_transfer / hackrf_sweep
python3 -c "import numpy"      # numpy 2.x; nothing else needed (no scipy, no GNU Radio)
hackrf_info                    # should list your board
```

## Prove the tooling without hardware

```sh
cd tools/antcap
make check
```

`make check` runs three things and must end in `ANTCAP CHECK: PASS`:

1. **crosscheck** — compiles `crosscheck.c` against
   `components/ant/src/{ant_phy_shockburst,ant_sb_link,antplus_profiles}.c`,
   prints 13 reference vectors (CRC, network-key validity and markers, complete
   18-byte frames for public / ANT+ networks, ack and burst packets, the strap
   frame received on the S3, a 3-byte-address frame) and diffs them
   against `python3 antframe.py`. The Python tooling cannot drift from the
   firmware's bytes.
2. **selftest** — `antdecode.py --selftest`: GFSK-modulates ANT+ HRM frames at
   4 MS/s, adds noise and carrier offsets (up to ±95 kHz, more than the HackRF's
   worst-case TCXO error), random burst timing and inverted keying, and checks
   the decoder returns exactly the transmitted bytes with the right spacing and
   offset estimate. Also: unknown-format scan (3-byte address, 12-byte payload)
   and a noise-only capture yielding zero false frames.
3. **roundtrip** — `antgen.py` writes a real int8 HackRF file, `antdecode.py`
   reads it back: 11/11 frames, address, ctrl byte, +30 kHz offset and the
   246.277 ms grid all verified.

## The bench plan, stage by stage

Each stage has a firmware build, a capture command and an expected decoder
output. Stages 1-4 have all passed on the ESP32-S3; they remain the way to
localise a regression (stop at the first stage that does not match) and the
way to bring up the next chip (ESP32-C6).

### 0. Calibrate the HackRF once (optional, 1 minute)

The HackRF's crystal is ±20 ppm (±49 kHz at 2457 MHz). To separate its error
from the ESP32's, capture any signal of known frequency — the cleanest is
the ESP32 itself running plain WiFi (`idf.py create-project` + a beacon on
channel 1 = 2412.000 MHz) or a signal generator — and run

```sh
python3 tone.py cal.cs8 --center 2412000000 --calibrate 2412000000
# -> SDR clock error +5.02 ppm  (pass --ppm 5.02 next time)
```

### 1. Carrier / frequency: where does the S3 transmit?

Firmware: `idf.py -DANT_ROLE=sensor build` (the DTM transmitter test runs
back-to-back, so a plain capture shows the S3's frequency directly).

```sh
./capture.sh tone.cs8 2                 # 2 s
python3 tone.py tone.cs8 --ppm 5.02
#   peak 1:   -3.1 kHz  62.0 dB above floor -> 2456.996900 MHz
```

Pass: the energy within a few tens of kHz of 2457.000 MHz (the GFSK tones sit
±~200 kHz around it). If it lands a whole number of MHz away, the
frequency-table entry `ant_espphy` writes (MHz − 2402) is off.
`hackrf_sweep -f 2400:2500` shows the same thing across the whole band if you
would rather look than compute.

### 2. TX: do the logged frame bytes come out as ANT+?

Firmware: `idf.py -DANT_ROLE=sensor build`. The firmware logs `last frame
(17 B @ 2457 MHz): a6 c5 30 42 78 01 0a ...` every 5 s — those are the bytes
that must appear (the preamble `aa` is added by the radio).

```sh
./capture.sh raw.cs8 5
python3 antdecode.py raw.cs8            # expects the ANT+ 6-byte address, 9-byte payload
python3 antdecode.py raw.cs8 --scan     # if not: anything with a valid CRC, any lengths
python3 antdecode.py raw.cs8 --invert   # if not: maybe the radio keys 1 = -deviation
```

Pass (verified on the S3): bursts on the `246.277 ms` grid, each decoding to
the firmware's logged bytes, `dev ~160-220 kHz`. Because DTM transmits
continuously, each slot shows the frame a few times back-to-back followed by
the all-zero test payload; that is expected. If nothing decodes:

| decoder output | meaning |
|---|---|
| `0 bursts` | wrong frequency (rerun stage 1) or the controller refused the test command (`hci err` counter in the firmware log) |
| bursts, `no CRC-valid frame`, `dev ~` far from 160 kHz | not 1 Mbit/s GFSK: the control-structure rate bytes did not take |
| `--scan` finds a frame with other lengths | framing differs: sync-word length or a whitening/CRC stage is in the way |
| frames decode but bytes differ from the log | bit order (the controller stores LSB-first; the code bit-reverses) |

`--json out.json` keeps every decoded frame with its sample time for a
side-by-side against the firmware log.

### 3. RX: does the ESP32 hear a known-good frame?

Firmware: the default build (HRM display, wildcard search).

```sh
python3 antgen.py -o hrm.cs8 --seconds 30 --devnum 0x1234 --hr 72
./transmit.sh hrm.cs8
```

The HackRF now behaves as an ANT+ HRM sensor #0x1234 (ANT+ network key, the
real `a6 c5` marker, HRM page 0, 8070-tick grid). The firmware should log the
channel going to `TRACKING`, learn id `1234/120`, and print `ch0 HRM #4660:
72 bpm`. The same file, captured back with `capture.sh`, is what stage 2
should look like when it passes — a handy reference. `--raw HEX` transmits
any frame bytes (e.g. one copied from a real sensor capture), `--public`
uses the public network key, `--offset` bakes in a carrier error to test the
receiver's tolerance. Mind the wildcard hole: choose a device number whose
low byte does not bit-reverse to < 13 (0x1234 is fine), or open the channel
with the number given.

### 4. A real ANT+ sensor

Put a commercial HRM strap or power meter next to the HackRF:

```sh
./capture.sh real.cs8 10
python3 antdecode.py real.cs8 --scan
```

This is how the air format was pinned: the strap on this bench (device
0x6941) decodes as `aa | a6 c5 | 69 41 | 78 | 01 | 0a <page> | crc`, which
is exactly what `ant_sb_link` builds for that identity, so the firmware
receives it with no per-device configuration. If a sensor does not decode,
the bursts are still listed with their length, tone separation and timing;
the 4.06 Hz grid identifies an HRM immediately, and the bits can be sliced by
hand from `gfsk.slice_bits` to look at the preamble / sync word.

## Formats and rates

* HackRF: `-s 4000000` (4 samples per bit) is the default everywhere. 8 MS/s
  also works (`--rate 8e6`); the modem only requires an integer number of
  samples per bit.
* Files: `.cs8`/`.iq` int8 I,Q (HackRF), `.cu8` (rtl_sdr), `.cs16`, `.cf32`
  (GNU Radio / gqrx / SDR++), chosen by extension or `--format`.
* Gains: `LNA=24 VGA=20 ./capture.sh` — if antdecode prints `peak 1.000` the
  ADC is clipping; lower the VGA.
