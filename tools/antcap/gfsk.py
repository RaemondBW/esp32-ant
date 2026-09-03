"""
gfsk.py - numpy-only GFSK modem for ANT / ShockBurst bursts (1 Mbit/s,
+-160 kHz, BT 0.5) at an SDR sample rate, plus HackRF I/Q file I/O.

Modulator: bits -> GFSK complex baseband (used to self-test the decoder and to
synthesise frames for `hackrf_transfer -t`).
Demodulator: I/Q -> instantaneous frequency -> burst detection -> per-burst
DC (carrier offset) estimate -> clock phase from zero crossings -> bit slicing.
The bit-level frame search lives in antdecode.py.

Convention: bit 1 = +deviation, bit 0 = -deviation (nRF24 / BLE). Pass
invert=True to the slicer if the radio turns out to key the other way.
"""
import numpy as np

BITRATE = 1_000_000.0
DEVIATION = 160_000.0
BT = 0.5

# ------------------------------------------------------------------ file I/O

FORMATS = {
    "cs8":  ("HackRF hackrf_transfer: int8 I,Q interleaved", np.int8, 127.0, 0.0),
    "cu8":  ("rtl_sdr: uint8 I,Q interleaved", np.uint8, 127.5, 127.5),
    "cs16": ("int16 I,Q interleaved", np.int16, 32767.0, 0.0),
    "cf32": ("complex64 (GNU Radio / gqrx / SDR++)", np.complex64, 1.0, 0.0),
}


def format_for(path, fmt=None):
    if fmt:
        return fmt
    ext = path.rsplit(".", 1)[-1].lower() if "." in path else ""
    aliases = {"iq": "cs8", "cs8": "cs8", "s8": "cs8", "cu8": "cu8", "u8": "cu8",
               "cs16": "cs16", "s16": "cs16", "cf32": "cf32", "fc32": "cf32", "raw": "cs8"}
    return aliases.get(ext, "cs8")


def read_iq(path, fmt=None, max_samples=None):
    fmt = format_for(path, fmt)
    _, dtype, scale, offset = FORMATS[fmt]
    count = -1 if max_samples is None else (max_samples if fmt == "cf32" else 2 * max_samples)
    raw = np.fromfile(path, dtype=dtype, count=count)
    if fmt == "cf32":
        return raw.astype(np.complex64)
    raw = raw[: len(raw) - (len(raw) % 2)].astype(np.float32)
    x = (raw[0::2] - offset) / scale + 1j * (raw[1::2] - offset) / scale
    return x.astype(np.complex64)


def write_iq(path, x, fmt=None, amplitude=0.8):
    """`x` complex baseband with |x| <= 1. HackRF wants int8; keep some headroom."""
    fmt = format_for(path, fmt)
    _, dtype, scale, offset = FORMATS[fmt]
    if fmt == "cf32":
        np.asarray(x, dtype=np.complex64).tofile(path)
        return
    out = np.empty(2 * len(x), dtype=np.float32)
    out[0::2] = np.real(x) * amplitude * scale + offset
    out[1::2] = np.imag(x) * amplitude * scale + offset
    lo, hi = (0, 255) if fmt == "cu8" else (-scale, scale)
    np.clip(np.round(out), lo, hi).astype(dtype).tofile(path)


# ---------------------------------------------------------------- modulator

def gaussian_taps(fs, bitrate=BITRATE, bt=BT, span_bits=3):
    sps = fs / bitrate
    n = int(round(span_bits * sps))
    t = (np.arange(-n, n + 1) / fs) * bitrate          # in bit periods
    a = np.sqrt(2 * np.pi / np.log(2)) * bt
    h = a * np.exp(-2 * (np.pi ** 2) * (bt ** 2) * (t ** 2) / np.log(2))
    return h / h.sum()


def modulate(bits, fs, bitrate=BITRATE, deviation=DEVIATION, bt=BT,
             ramp_bits=2, pad_bits=4, phase0=0.0):
    """
    GFSK-modulate `bits` (iterable of 0/1, MSB-first order as on air) at
    sample rate `fs`. Returns complex64 baseband of one burst with `pad_bits`
    of unmodulated carrier before/after and a raised-cosine amplitude ramp
    of `ramp_bits` at each edge, so the burst looks like a real PA keying up.
    Bit boundaries are exact only when fs/bitrate is an integer.
    """
    sps = fs / bitrate
    if abs(sps - round(sps)) > 1e-9:
        raise ValueError("fs must be an integer multiple of the bit rate")
    sps = int(round(sps))
    nrz = np.repeat(np.where(np.asarray(list(bits), dtype=np.int8) > 0, 1.0, -1.0), sps)
    nrz = np.concatenate([np.zeros(pad_bits * sps), nrz, np.zeros(pad_bits * sps)])
    shaped = np.convolve(nrz, gaussian_taps(fs, bitrate, bt), mode="same")
    phase = phase0 + np.cumsum(2 * np.pi * deviation * shaped / fs)
    x = np.exp(1j * phase)
    n_ramp = ramp_bits * sps
    ramp = 0.5 - 0.5 * np.cos(np.pi * np.arange(n_ramp) / n_ramp)
    env = np.ones(len(x))
    env[:n_ramp] = ramp
    env[-n_ramp:] = ramp[::-1]
    return (x * env).astype(np.complex64)


def bytes_to_bits(data):
    return np.unpackbits(np.frombuffer(bytes(data), dtype=np.uint8))   # MSB first


def bits_to_bytes(bits):
    bits = np.asarray(bits, dtype=np.uint8)
    n = len(bits) - len(bits) % 8
    return np.packbits(bits[:n]).tobytes()


def place_bursts(fs, total_seconds, bursts, freq_offset=0.0, noise_rms=0.0, seed=1):
    """
    Lay `bursts` = [(t_seconds, complex burst), ...] into a `total_seconds`
    long capture, shift them by `freq_offset` Hz (simulated crystal error) and
    add complex white noise of `noise_rms` (burst amplitude is 1.0). Bursts
    are placed to the nearest sample; the exact placed sample index is returned
    so tests can check timing.
    """
    n = int(round(total_seconds * fs))
    x = np.zeros(n, dtype=np.complex64)
    placed = []
    for t, b in bursts:
        i = int(round(t * fs))
        if i + len(b) > n:
            break
        x[i:i + len(b)] += b
        placed.append(i)
    if freq_offset:
        x *= np.exp(2j * np.pi * freq_offset * np.arange(n) / fs).astype(np.complex64)
    if noise_rms > 0:
        rng = np.random.default_rng(seed)
        x += (noise_rms / np.sqrt(2)) * (rng.standard_normal(n) + 1j * rng.standard_normal(n)).astype(np.complex64)
    return x, placed


# -------------------------------------------------------------- demodulator

def lowpass(x, fs, cutoff=600e3, taps=31):
    """Windowed-sinc channel filter: passes +-cutoff (tones + offset), drops the rest."""
    n = np.arange(taps) - (taps - 1) / 2
    h = np.sinc(2 * cutoff / fs * n) * np.hamming(taps)
    h /= h.sum()
    return np.convolve(x, h, mode="same").astype(np.complex64)


def instantaneous_freq(x, fs, prefilter=True):
    """Quadrature discriminator: Hz per sample (length len(x), first sample 0)."""
    if prefilter:
        x = lowpass(x, fs)
    d = np.angle(x[1:] * np.conj(x[:-1]))
    f = np.empty(len(x), dtype=np.float32)
    f[0] = 0.0
    f[1:] = d * (fs / (2 * np.pi))
    return f


def moving_average(v, n):
    if n <= 1:
        return np.asarray(v, dtype=np.float32)
    c = np.cumsum(np.concatenate([[0.0], np.asarray(v, dtype=np.float64)]))
    out = (c[n:] - c[:-n]) / n
    # centre the window: pad both ends so len(out) == len(v)
    pad_l = (n - 1) // 2
    pad_r = n - 1 - pad_l
    return np.concatenate([np.full(pad_l, out[0]), out, np.full(pad_r, out[-1])]).astype(np.float32)


def find_bursts(x, fs, bitrate=BITRATE, threshold=None, min_bits=40, gap_bits=8, pad_bits=6):
    """
    Energy-based burst detection. Returns [(start, end)] sample ranges (with
    `pad_bits` of margin) where the smoothed envelope exceeds `threshold`
    (absolute, |x| units). Default: 8 dB above the noise floor (10th
    percentile of the envelope), but at least 5% of the peak.
    """
    sps = fs / bitrate
    env = moving_average(np.abs(x), max(1, int(round(sps * 4))))
    if threshold is None:
        floor = float(np.percentile(env, 10))
        peak = float(env.max())
        threshold = max(2.5 * floor, 0.05 * peak, 1e-6)
    above = env > threshold
    if not above.any():
        return [], threshold
    edges = np.diff(above.astype(np.int8))
    starts = list(np.flatnonzero(edges == 1) + 1)
    ends = list(np.flatnonzero(edges == -1) + 1)
    if above[0]:
        starts.insert(0, 0)
    if above[-1]:
        ends.append(len(x))
    # merge segments separated by less than gap_bits
    merged = []
    gap = gap_bits * sps
    for s, e in zip(starts, ends):
        if merged and s - merged[-1][1] < gap:
            merged[-1] = (merged[-1][0], e)
        else:
            merged.append((s, e))
    pad = int(round(pad_bits * sps))
    out = []
    for s, e in merged:
        if e - s >= min_bits * sps:
            out.append((max(0, s - pad), min(len(x), e + pad)))
    return out, threshold


def estimate_centre(freq, x=None, fs=None):
    """
    Midpoint between the two FSK tones (= carrier offset) and their spacing
    (= 2 x deviation). Uses only samples where the envelope is within 6 dB of
    the burst peak, so the noise around the burst does not pull the estimate.
    Robust to data imbalance up to ~85/15.
    """
    if x is not None:
        env = moving_average(np.abs(lowpass(x, fs)), 4)
        strong = env > 0.5 * env.max()
        if strong.sum() >= 16:
            freq = freq[strong]
    lo, hi = np.percentile(freq, [15, 85])
    return float((lo + hi) / 2), float(hi - lo)


def slice_bits(freq, fs, bitrate=BITRATE, centre=None, invert=False, phases=None):
    """
    Slice a burst's instantaneous-frequency trace into bits at every sample
    phase (or the given `phases`), best phase first. Returns
    [(phase, bits ndarray)], where `phase` is the sample offset within a bit.
    Clock recovery: the mean zero-crossing position modulo sps gives the bit
    edge; sample half a bit later. Drift over a 17-byte burst is negligible.
    """
    sps = int(round(fs / bitrate))
    if centre is None:
        centre, _ = estimate_centre(freq)
    f = moving_average(freq - centre, sps)          # integrate over one bit
    if invert:
        f = -f
    sign = f > 0
    crossings = np.flatnonzero(sign[1:] != sign[:-1]) + 1
    if len(crossings):
        ang = np.exp(2j * np.pi * (crossings % sps) / sps).mean()
        edge = (np.angle(ang) / (2 * np.pi) * sps) % sps
        best = int(round(edge + sps / 2)) % sps
    else:
        best = sps // 2
    if phases is None:
        order = [best] + [p for p in range(sps) if p != best]
    else:
        order = list(phases)
    out = []
    for p in order:
        bits = (f[p::sps] > 0).astype(np.uint8)
        out.append((p, bits))
    return out
