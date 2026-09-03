#!/bin/sh
# capture.sh - record ANT+ air (2457 MHz) with a HackRF for antdecode.py / tone.py.
#
#   tools/antcap/capture.sh [out.cs8] [seconds] [freq_hz]
#
# Defaults: cap.cs8, 5 s, 2457000000. Sample rate 4 MS/s (4 samples per
# 1 Mbit/s bit; int8 I/Q = 8 MB/s). Gains are conservative for an ESP32 a
# metre away; if antdecode reports peak ~1.0 the ADC is clipping - lower -g.
set -eu
OUT=${1:-cap.cs8}
SECONDS_=${2:-5}
FREQ=${3:-2457000000}
RATE=4000000
LNA=${LNA:-24}     # 0..40 in steps of 8
VGA=${VGA:-20}     # 0..62 in steps of 2
command -v hackrf_transfer >/dev/null 2>&1 || { echo "hackrf_transfer not found: brew install hackrf" >&2; exit 1; }
N=$((SECONDS_ * RATE))
echo "capturing $SECONDS_ s at $FREQ Hz, $RATE S/s -> $OUT (LNA $LNA, VGA $VGA)"
hackrf_transfer -r "$OUT" -f "$FREQ" -s "$RATE" -l "$LNA" -g "$VGA" -n "$N"
echo "decode with: python3 $(dirname "$0")/antdecode.py $OUT"
