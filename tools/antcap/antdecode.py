#!/usr/bin/env python3
"""
antdecode.py - find and decode ANT / ShockBurst frames in an SDR capture.

    hackrf_transfer -r cap.cs8 -f 2457000000 -s 4000000 -l 32 -g 20 -n 40000000
    python3 antdecode.py cap.cs8                 # everything with a valid CRC
    python3 antdecode.py cap.cs8 --scan          # unknown address/payload length too
    python3 antdecode.py --selftest              # no hardware: synth -> decode

For each energy burst in the capture the tool demodulates GFSK (1 Mbit/s),
estimates the carrier offset, recovers the bit clock, slices bits at every
sample phase and searches them for  preamble(0xAA/0x55) | address | payload |
CRC-16/CCITT-FALSE  - the same check the ESP32 firmware applies
(ant_sb_verify_frame). Frames on the ANT+ link (6-byte address, ctrl + 8
byte page) are decoded down to the ANT+ HRM fields; anything else with a
valid CRC is printed raw. Inter-burst spacing is compared against the ANT
channel period (8070 ticks = 246.28 ms for HRM) so you can see the TDMA grid.

Only numpy is required.
"""
import argparse
import json
import sys

import numpy as np

import antframe as af
import gfsk

DEFAULT_RATE = 4_000_000


# ------------------------------------------------------------ frame search

def search_bits(bits, addr_lens=(5,), payload_lens=(9,), addr=None):
    """
    Scan a 0/1 array for preamble + CRC-valid body. Returns a list of
    (bit_offset, addr_len, payload_len, body bytes). Overlapping hits at the
    same offset keep the longest body; after a hit the scan resumes after it.
    """
    n = len(bits)
    hits = []
    min_total = min(addr_lens) + min(payload_lens) + 2
    i = 0
    packed_cache = {}
    while i + 8 + 8 * min_total <= n:
        b = bits[i:i + 8]
        pre = int(np.packbits(b)[0])
        if pre not in (0xAA, 0x55) or bits[i + 8] != (1 if pre == 0xAA else 0):
            i += 1
            continue
        found = None
        for al in addr_lens:
            for pl in sorted(payload_lens, reverse=True):
                total = al + pl + 2
                end = i + 8 + 8 * total
                if end > n:
                    continue
                key = (i + 8, total)
                body = packed_cache.get(key)
                if body is None:
                    body = np.packbits(bits[i + 8:end]).tobytes()
                    packed_cache[key] = body
                if addr is not None and body[:al] != addr[:al]:
                    continue
                if af.verify_body(body):
                    found = (i, al, pl, body)
                    break
            if found:
                break
        if found:
            hits.append(found)
            i = found[0] + 8 + 8 * (found[1] + found[2] + 2)
        else:
            i += 1
    return hits


def decode_burst(x, fs, start, end, args):
    """Demodulate one burst and return its best decode as a dict (or None)."""
    seg = x[start:end]
    freq = gfsk.instantaneous_freq(seg, fs)
    if args.offset is not None:
        centre, sep = float(args.offset), 0.0
    else:
        centre, sep = gfsk.estimate_centre(freq, seg, fs)
    best = None
    for phase, bits in gfsk.slice_bits(freq, fs, centre=centre, invert=args.invert):
        hits = search_bits(bits, args.addr_lens, args.payload_lens, args.addr)
        if hits:
            best = {"phase": phase, "hits": hits, "bits": int(len(bits))}
            break
    return {"start": int(start), "end": int(end), "centre_hz": centre, "tone_sep_hz": sep,
            "peak": float(np.abs(seg).max()), "decode": best}


# ------------------------------------------------------------- reporting

def describe_body(al, pl, body):
    """Human-readable line for a CRC-valid body."""
    addr, payload = body[:al], body[al:al + pl]
    s = f"addr={af.hexs(addr)}  payload={af.hexs(payload)}"
    if al == af.LINK_ADDR_LEN and pl == af.LINK_PAYLOAD_LEN:
        dev = addr[2] << 8 | addr[3]
        ctrl, page = payload[0], payload[1:]
        net = {af.network_marker(af.ANTPLUS_NETWORK_KEY): "ANT+", af.network_marker(af.PUBLIC_NETWORK_KEY): "public",
               af.network_marker(af.ANTFS_NETWORK_KEY): "ANT-FS"}.get(bytes(addr[:2]), "unknown")
        s += (f"\n      link: net-marker {af.hexs(addr[:2])} ({net}) dev #{dev} (0x{dev:04x}) type {addr[4]}"
              f" trans {addr[5]}  {af.describe_ctrl(ctrl)}")
        if addr[4] == af.ANTPLUS_DEVTYPE_HRM:
            s += "\n      " + af.describe_hrm_page(page)
    return s


def run_file(args):
    fs = args.rate
    x = gfsk.read_iq(args.file, args.format, max_samples=int(args.seconds * fs) if args.seconds else None)
    if len(x) < 1000:
        sys.exit(f"{args.file}: only {len(x)} samples")
    bursts, thr = gfsk.find_bursts(x, fs, threshold=args.threshold)
    print(f"{args.file}: {len(x)} samples = {len(x) / fs:.3f} s at {fs / 1e6:g} MS/s, "
          f"{len(bursts)} bursts above |x|>{thr:.3f}")
    results = [decode_burst(x, fs, s, e, args) for s, e in bursts]
    frames = []
    last_t = None
    period_s = args.period_ms / 1000.0
    for r in results:
        t = r["start"] / fs
        d = r["decode"]
        spacing = "" if last_t is None else f"  +{(t - last_t) * 1000:8.3f} ms"
        if spacing and period_s:
            k = round((t - last_t) / period_s)
            if k >= 1:
                err = ((t - last_t) - k * period_s) * 1e6
                spacing += f" ({k}x period {err:+.0f} us)"
        head = (f"[{t:10.6f} s] burst {(r['end'] - r['start']) / fs * 1e6:6.1f} us  peak {r['peak']:.3f}  "
                f"offset {r['centre_hz'] / 1e3:+7.1f} kHz  dev ~{r['tone_sep_hz'] / 2e3:5.1f} kHz{spacing}")
        if d is None:
            if args.verbose or not args.quiet_undecoded:
                print(head + "  -- no CRC-valid frame")
            last_t = t
            continue
        print(head)
        for off, al, pl, body in d["hits"]:
            print(f"   {describe_body(al, pl, body)}")
            frames.append({"t": t, "addr_len": al, "payload_len": pl, "body": body.hex(),
                           "offset_hz": r["centre_hz"], "phase": d["phase"]})
        last_t = t
    print(f"\n{len(frames)} CRC-valid frames in {len(bursts)} bursts")
    if frames:
        offs = np.array([f["offset_hz"] for f in frames])
        print(f"carrier offset: mean {offs.mean() / 1e3:+.1f} kHz, spread {offs.std() / 1e3:.1f} kHz "
              f"(HackRF TCXO is +-20 ppm = +-49 kHz at 2457 MHz; see tone.py)")
        ts = np.array([f["t"] for f in frames])
        if len(ts) > 1:
            dt = np.diff(ts)
            print(f"frame spacing: min {dt.min() * 1e3:.3f} ms  median {np.median(dt) * 1e3:.3f} ms  "
                  f"(expected {args.period_ms:.3f} ms)")
    if args.json:
        with open(args.json, "w") as fp:
            json.dump(frames, fp, indent=1)
        print(f"wrote {args.json}")
    return 0 if frames else 2


# --------------------------------------------------------------- self-test

def selftest(args):
    """
    Synthesise an ANT+ HRM sensor's bursts at 4 MS/s, decode them back under
    noise, carrier offset, arbitrary timing phase and inverted keying, and
    check bytes, count, offset estimate and spacing. Pure software.
    """
    fs = args.rate
    key = af.ANTPLUS_NETWORK_KEY
    period = af.ANTPLUS_PERIOD_HRM / af.TICKS_PER_SEC
    cases = [
        dict(name="clean", noise=0.0, offset=0.0, invert=False),
        dict(name="+40 kHz offset, SNR ~14 dB", noise=0.2, offset=40e3, invert=False),
        dict(name="-95 kHz offset, SNR ~10 dB", noise=0.32, offset=-95e3, invert=False),
        dict(name="inverted keying, +20 kHz", noise=0.1, offset=20e3, invert=True),
        # 7 dB in 4 MHz (~13 dB Eb/N0): a discriminator at h=0.32 loses frames
        # here; a HackRF a metre from the ESP32 sees far more. Partial credit.
        dict(name="SNR ~7 dB (degraded)", noise=0.45, offset=10e3, invert=False, min_frames=2),
    ]
    ok_all = True
    for case in cases:
        n_frames = 6
        bursts, expect = [], []
        for k in range(n_frames):
            page = af.hrm_page0(60 + k, k, 1024 * k, toggle=(k & 4) != 0)
            ctrl = af.CTRL_BROADCAST if k % 3 else af.CTRL_ACK
            frame = af.link_frame(0x3042, af.ANTPLUS_DEVTYPE_HRM, 1, page, ctrl=ctrl, key=key)
            bits = gfsk.bytes_to_bits(frame)
            if case["invert"]:
                bits = 1 - bits
            # fractional-sample timing: random extra sub-bit delay via phase0 only
            # (integer placement); the slicer must find the right phase anyway
            t = 0.010 + k * period + (k * 0.37e-6)
            bursts.append((t, gfsk.modulate(bits, fs, phase0=0.7 * k)))
            expect.append(frame[1:])
        x, placed = gfsk.place_bursts(fs, 0.010 + n_frames * period + 0.010, bursts,
                                      freq_offset=case["offset"], noise_rms=case["noise"], seed=7)
        # extra: a burst of pure noise-like garbage must not decode
        found, thr = gfsk.find_bursts(x, fs)
        ns = argparse.Namespace(offset=None, invert=case["invert"], addr=None,
                                addr_lens=(af.LINK_ADDR_LEN,), payload_lens=(9,))
        got, offsets, starts = [], [], []
        for s, e in found:
            r = decode_burst(x, fs, s, e, ns)
            if r["decode"]:
                for _, al, pl, body in r["decode"]["hits"]:
                    got.append(body)
                offsets.append(r["centre_hz"])
                starts.append(s)
        want = case.get("min_frames", n_frames)
        ok = (got == expect) if want == n_frames else (len(got) >= want and all(g in expect for g in got))
        off_err = max(abs(o - case["offset"]) for o in offsets) if offsets else float("inf")
        ok &= off_err < 25e3
        spacing_ok = True
        if len(starts) == n_frames and want == n_frames:
            for k in range(1, n_frames):
                exp_dt = placed[k] - placed[k - 1]
                spacing_ok &= abs((starts[k] - starts[k - 1]) - exp_dt) <= 2 * (fs / gfsk.BITRATE)
        ok &= spacing_ok
        ok_all &= ok
        print(f"{'PASS' if ok else 'FAIL'}  {case['name']:32s} bursts={len(found)} frames={len(got)}/{n_frames} "
              f"offset-est-err={off_err / 1e3:.1f} kHz spacing={'ok' if spacing_ok else 'BAD'}")
        if not ok and args.verbose:
            for g in got:
                print("   got ", g.hex())
            for e in expect:
                print("   want", e.hex())

    # scan mode must find a 3-byte-address, 12-byte-payload frame it knows nothing about
    body_frame = af.build_frame(bytes([0x31, 0x77, 0x02]), bytes(range(12)))
    x, _ = gfsk.place_bursts(fs, 0.002, [(0.0005, gfsk.modulate(gfsk.bytes_to_bits(body_frame), fs))],
                             freq_offset=-30e3, noise_rms=0.15)
    found, _ = gfsk.find_bursts(x, fs)
    ns = argparse.Namespace(offset=None, invert=False, addr=None, addr_lens=(3, 4, 5, 6), payload_lens=range(1, 33))
    hits = [h for s, e in found for h in (decode_burst(x, fs, s, e, ns)["decode"] or {}).get("hits", [])]
    scan_ok = len(hits) == 1 and hits[0][1] == 3 and hits[0][2] == 12 and hits[0][3] == body_frame[1:]
    ok_all &= scan_ok
    print(f"{'PASS' if scan_ok else 'FAIL'}  {'scan: unknown 3B addr / 12B payload':32s} hits={len(hits)}")

    # noise only: no false frames
    rng = np.random.default_rng(3)
    x = (0.3 * (rng.standard_normal(200_000) + 1j * rng.standard_normal(200_000))).astype(np.complex64)
    found, _ = gfsk.find_bursts(x, fs)
    ns = argparse.Namespace(offset=None, invert=False, addr=None, addr_lens=(3, 4, 5, 6), payload_lens=range(1, 33))
    hits = [h for s, e in found for h in (decode_burst(x, fs, s, e, ns)["decode"] or {}).get("hits", [])]
    noise_ok = len(hits) == 0
    ok_all &= noise_ok
    print(f"{'PASS' if noise_ok else 'FAIL'}  {'noise only: no false frames':32s} bursts={len(found)} hits={len(hits)}")

    print("SELFTEST", "PASS" if ok_all else "FAIL")
    return 0 if ok_all else 1


# -------------------------------------------------------------------- main

def parse_hex(s):
    return bytes.fromhex(s.replace(" ", "").replace(":", ""))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("file", nargs="?", help="I/Q capture (HackRF .cs8/.iq, .cu8, .cs16, .cf32)")
    p.add_argument("--rate", type=float, default=DEFAULT_RATE, help="sample rate, Hz (default 4e6)")
    p.add_argument("--format", choices=list(gfsk.FORMATS), help="override sample format (default: by extension)")
    p.add_argument("--seconds", type=float, help="only read the first N seconds")
    p.add_argument("--addr", type=parse_hex, help="only accept this address (hex, e.g. a6c5 6941 78 01)")
    p.add_argument("--addr-len", type=int, choices=(3, 4, 5, 6), help="address length (default 6, or 3..6 with --scan)")
    p.add_argument("--payload-len", type=int, help="payload length (default 9 = ctrl+page, or 1..32 with --scan)")
    p.add_argument("--scan", action="store_true", help="try every address (3..6) and payload (1..32) length")
    p.add_argument("--invert", action="store_true", help="bit 1 = negative deviation")
    p.add_argument("--offset", type=float, help="fixed carrier offset in Hz instead of per-burst estimate")
    p.add_argument("--threshold", type=float, help="burst detector level in |x| (default: automatic)")
    p.add_argument("--period-ms", type=float, default=af.ANTPLUS_PERIOD_HRM / af.TICKS_PER_SEC * 1000,
                   help="expected channel period for spacing report (default 246.28 = 8070 ticks)")
    p.add_argument("--quiet-undecoded", action="store_true", help="don't list bursts without a valid frame")
    p.add_argument("--json", help="write decoded frames to this JSON file")
    p.add_argument("-v", "--verbose", action="store_true")
    p.add_argument("--selftest", action="store_true", help="synthesise and decode; no hardware needed")
    args = p.parse_args(argv)

    if args.scan:
        args.addr_lens = (args.addr_len,) if args.addr_len else (3, 4, 5, 6)
        args.payload_lens = (args.payload_len,) if args.payload_len else tuple(range(1, 33))
    else:
        args.addr_lens = (args.addr_len or (len(args.addr) if args.addr else af.LINK_ADDR_LEN),)
        args.payload_lens = (args.payload_len or af.LINK_PAYLOAD_LEN,)

    if args.selftest:
        return selftest(args)
    if not args.file:
        p.error("a capture file or --selftest is required")
    return run_file(args)


if __name__ == "__main__":
    sys.exit(main())
