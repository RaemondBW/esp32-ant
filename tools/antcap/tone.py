#!/usr/bin/env python3
"""
tone.py - where is the carrier? Averaged spectrum of an I/Q capture, peak(s)
reported as an offset from the tuned centre.

    hackrf_transfer -r tone.cs8 -f 2457000000 -s 4000000 -n 8000000 -l 16 -g 16
    python3 tone.py tone.cs8                      # -> peak at +12.3 kHz (2457.0123 MHz)

Use it for stage 1 of the bench plan (a sensor-role build transmitting): the
ESP32's signal should land within a few tens of kHz of 2457.000 MHz. Two
contributions to what you read: the ESP32's frequency-table entry (MHz - 2402)
and the HackRF's own TCXO error (+-20 ppm = +-49 kHz at 2.457 GHz, but usually
a few ppm). Separate them once: run --calibrate against a signal of known
frequency (e.g. `--calibrate` on a capture of a WiFi channel you know, or a
signal generator) and pass the resulting `--ppm` here.

Only numpy is required.
"""
import argparse
import sys

import numpy as np

import gfsk


def spectrum(x, fs, nfft=65536):
    n = len(x) // nfft
    if n == 0:
        raise SystemExit("capture too short for the FFT size")
    blocks = x[: n * nfft].reshape(n, nfft) * np.hanning(nfft)
    p = (np.abs(np.fft.fft(blocks, axis=1)) ** 2).mean(axis=0)
    p = np.fft.fftshift(p) / (nfft ** 2)
    f = np.fft.fftshift(np.fft.fftfreq(nfft, 1 / fs))
    return f, 10 * np.log10(p + 1e-20)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("file")
    p.add_argument("--rate", type=float, default=4_000_000)
    p.add_argument("--format", choices=list(gfsk.FORMATS))
    p.add_argument("--center", type=float, default=2457e6, help="tuned centre frequency, Hz")
    p.add_argument("--ppm", type=float, default=0.0, help="SDR clock error to correct for (from --calibrate)")
    p.add_argument("--calibrate", type=float, metavar="HZ",
                   help="the strongest peak is really at this absolute frequency; print the SDR ppm")
    p.add_argument("--nfft", type=int, default=65536, help="FFT size (bin = rate/nfft: 61 Hz default)")
    p.add_argument("--peaks", type=int, default=5)
    p.add_argument("--seconds", type=float, help="only read the first N seconds")
    p.add_argument("--dc-skip", type=float, default=2e3, help="ignore +-this around DC (HackRF DC spike), Hz")
    args = p.parse_args(argv)

    fs = args.rate
    x = gfsk.read_iq(args.file, args.format, max_samples=int(args.seconds * fs) if args.seconds else None)
    f, db = spectrum(x, fs, args.nfft)
    floor = np.median(db)
    print(f"{args.file}: {len(x) / fs:.3f} s, {args.nfft}-point FFT (bin {fs / args.nfft:.1f} Hz), "
          f"noise floor {floor:.1f} dBFS/bin")

    valid = np.abs(f) > args.dc_skip
    order = np.argsort(db * valid - 1e9 * (~valid))[::-1]
    picked = []
    for i in order:
        if len(picked) >= args.peaks:
            break
        if db[i] - floor < 6:                                 # nothing left above the noise
            break
        if any(abs(f[i] - f[j]) < 20e3 for j in picked):     # one entry per 20 kHz neighbourhood
            continue
        picked.append(i)

    corr = 1 + args.ppm * 1e-6
    for rank, i in enumerate(picked):
        off = f[i]
        absolute = (args.center + off) / corr           # the SDR's Hz are (1+ppm) too big
        print(f"  peak {rank + 1}: {off / 1e3:+9.3f} kHz  {db[i] - floor:5.1f} dB above floor  "
              f"-> {absolute / 1e6:.6f} MHz{' (ppm-corrected)' if args.ppm else ''}")

    if args.calibrate is not None and picked:
        measured = args.center + f[picked[0]]
        ppm = (measured / args.calibrate - 1) * 1e6
        print(f"calibration: strongest peak read as {measured / 1e6:.6f} MHz, true {args.calibrate / 1e6:.6f} MHz "
              f"-> SDR clock error {ppm:+.2f} ppm  (pass --ppm {ppm:.2f} next time)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
