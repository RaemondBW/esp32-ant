#!/bin/sh
# transmit.sh - key an antgen.py I/Q file on the air with a HackRF.
#
#   python3 tools/antcap/antgen.py -o hrm.cs8 --seconds 30
#   tools/antcap/transmit.sh hrm.cs8 [freq_hz]
#
# Bench use only, at minimum power (TX VGA 0..47 dB; default low) with the
# HackRF next to the ESP32. 2457 MHz is inside the 2.4 GHz ISM band; keep
# the power to what the local rules allow.
set -eu
IN=${1:?usage: transmit.sh file.cs8 [freq_hz]}
FREQ=${2:-2457000000}
RATE=4000000
TXVGA=${TXVGA:-10}
command -v hackrf_transfer >/dev/null 2>&1 || { echo "hackrf_transfer not found: brew install hackrf" >&2; exit 1; }
echo "transmitting $IN at $FREQ Hz, $RATE S/s, TX VGA $TXVGA dB (ctrl-c to stop)"
hackrf_transfer -t "$IN" -f "$FREQ" -s "$RATE" -a 1 -x "$TXVGA" -R
