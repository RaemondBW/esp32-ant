#!/usr/bin/env python3
"""
phyrepl.py - drive the ESP-IDF `phy>` RF-test console (examples/phy/cert_test)
over serial so bench steps can be scripted together with the HackRF.

    python3 phyrepl.py -p /dev/cu.usbmodem21301 "bt_tx_tone -e 1 -c 27 -p 0"
    python3 phyrepl.py -p PORT "help"
    python3 phyrepl.py -p PORT cmdstop

Needs pyserial (present in the ESP-IDF python env).
"""
import argparse
import sys
import time

import serial


def repl(port, cmds, baud=115200, settle=0.4, quiet=False):
    with serial.Serial(port, baud, timeout=0.2) as s:
        s.dtr = False
        s.rts = False
        # Opening the USB-Serial-JTAG port resets the chip on some boards: wait
        # for the prompt (or 3 s of silence) before talking.
        deadline = time.time() + 4.0
        seen = b""
        while time.time() < deadline:
            chunk = s.read(4096)
            seen += chunk
            if b"phy>" in seen[-200:] and not chunk:
                break
        s.reset_input_buffer()
        s.write(b"\r\n")
        time.sleep(settle)
        s.read(65536)
        out = []
        for c in cmds:
            s.write(c.encode() + b"\r\n")
            time.sleep(settle)
            r = s.read(65536).decode(errors="replace")
            out.append(r)
            if not quiet:
                print(r.strip())
        return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("-p", "--port", required=True)
    p.add_argument("--settle", type=float, default=0.4, help="seconds to wait for each reply")
    p.add_argument("cmds", nargs="+")
    a = p.parse_args(argv)
    repl(a.port, a.cmds, settle=a.settle)
    return 0


if __name__ == "__main__":
    sys.exit(main())
