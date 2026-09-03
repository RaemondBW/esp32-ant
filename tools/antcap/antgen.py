#!/usr/bin/env python3
"""
antgen.py - synthesise an ANT+ sensor's on-air traffic as an I/Q file.

    python3 antgen.py -o hrm.cs8 --seconds 10            # HRM #0x1234 at 72 bpm
    hackrf_transfer -t hrm.cs8 -f 2457000000 -s 4000000 -a 1 -x 20

Produces exactly the bytes ant_mac transmits (verified against the C code by
`make check`) as 1 Mbit/s GFSK bursts on the ANT TDMA grid - one broadcast
every `--period` ticks of 1/32768 s (8070 = 246.28 ms for HRM). Uses:

  * feed the ESP32's receive path from a HackRF while `espphy_rx_poll()` is
    being brought up (the firmware's display role should acquire it);
  * a known-good reference capture for antdecode.py (`antgen | antdecode`
    round-trips in `make check`);
  * `--raw HEX` to key any frame bytes you like.

Only numpy is required.
"""
import argparse
import sys

import numpy as np

import antframe as af
import gfsk


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("-o", "--output", required=True, help="output I/Q file (.cs8 for HackRF, .cf32, ...)")
    p.add_argument("--rate", type=float, default=4_000_000, help="sample rate, Hz (default 4e6)")
    p.add_argument("--seconds", type=float, default=5.0)
    p.add_argument("--devnum", type=lambda s: int(s, 0), default=0x1234)
    p.add_argument("--devtype", type=int, default=af.ANTPLUS_DEVTYPE_HRM)
    p.add_argument("--trans", type=int, default=1)
    p.add_argument("--period", type=int, default=af.ANTPLUS_PERIOD_HRM, help="channel period in ticks (8070)")
    p.add_argument("--hr", type=int, default=72, help="heart rate for HRM page 0")
    p.add_argument("--public", action="store_true", help="public network key instead of ANT+")
    p.add_argument("--raw", help="hex bytes of a complete frame to send instead (incl. preamble)")
    p.add_argument("--offset", type=float, default=0.0, help="carrier offset to bake in, Hz")
    p.add_argument("--deviation", type=float, default=gfsk.DEVIATION)
    p.add_argument("--start", type=float, default=0.020, help="time of the first burst, s")
    p.add_argument("--amplitude", type=float, default=0.8, help="peak |sample| as a fraction of full scale")
    p.add_argument("--invert", action="store_true", help="bit 1 = negative deviation")
    args = p.parse_args(argv)

    fs = args.rate
    key = af.PUBLIC_NETWORK_KEY if args.public else af.ANTPLUS_NETWORK_KEY
    period_s = args.period / af.TICKS_PER_SEC
    bursts = []
    k = 0
    beats, event_time = 0, 0
    while True:
        t = args.start + k * period_s
        if t + 200e-6 > args.seconds:
            break
        if args.raw:
            frame = bytes.fromhex(args.raw.replace(" ", ""))
        else:
            # one beat per ~60/hr s: advance the HRM counters on the ANT grid
            while (event_time + 1024 * 60 / args.hr) <= t * 1024:
                event_time += 1024 * 60 / args.hr
                beats += 1
            page = af.hrm_page0(args.hr, beats, int(event_time) & 0xFFFF, toggle=(k // 4) & 1 == 1)
            frame = af.link_frame(args.devnum, args.devtype, args.trans, page, key=key)
        bits = gfsk.bytes_to_bits(frame)
        if args.invert:
            bits = 1 - bits
        bursts.append((t, gfsk.modulate(bits, fs, deviation=args.deviation)))
        k += 1
    x, _ = gfsk.place_bursts(fs, args.seconds, bursts, freq_offset=args.offset)
    gfsk.write_iq(args.output, x, amplitude=args.amplitude)
    first = bursts[0] if bursts else None
    print(f"{args.output}: {len(x)} samples, {args.seconds:g} s at {fs / 1e6:g} MS/s, "
          f"{len(bursts)} bursts every {period_s * 1e3:.3f} ms ({args.period} ticks)")
    if not args.raw:
        print(f"identity #{args.devnum} (0x{args.devnum:04x}) type {args.devtype} trans {args.trans}, "
              f"{'public' if args.public else 'ANT+'} network, address {af.hexs(af.link_address(args.devnum, args.devtype, args.trans, key))}")
    return 0 if first else 1


if __name__ == "__main__":
    sys.exit(main())
