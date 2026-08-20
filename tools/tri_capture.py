"""Simultaneous 3-board ignition telemetry capture.

Opens all three ignition MCU serial ports at once and logs every line to its own
file, timestamped against a single shared clock so the three logs can be lined up
against each other after the fact.

This matters: the false half-lock was only diagnosable because three boards were
captured at the same time. Judging one channel alone hides which faults are common
mode and which are per-channel.

Usage:
    python tri_capture.py                  # default ports/tag
    python tri_capture.py --tag firststart
    python tri_capture.py --ports COM13,COM15,COM14

Opening a CH340 port asserts DTR and resets the Mega, so each board reboots as the
capture starts and prints its banner. That is deliberate -- the banner records which
cylinder each port actually is, since COM numbers renumber between sessions.
"""

import argparse
import os
import sys
import threading
import time
from datetime import datetime

try:
    import serial
except ImportError:
    sys.exit("pyserial not installed:  pip install pyserial")

DEFAULT_PORTS = ["COM13", "COM15", "COM14"]
BAUD = 115200
LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "bench_logs")

t0 = time.monotonic()
stop = threading.Event()
lock = threading.Lock()
stats = {}


def reader(port, path):
    """One thread per board. Never raises -- a dead port must not kill the capture."""
    try:
        ser = serial.Serial(port, BAUD, timeout=0.2)
    except Exception as exc:
        with lock:
            print(f"  {port}: OPEN FAILED: {exc}")
        stats[port] = {"lines": 0, "cyl": "?", "err": str(exc)}
        return

    st = {"lines": 0, "cyl": "?", "err": None}
    stats[port] = st

    with open(path, "w", encoding="utf-8", buffering=1) as fh:
        fh.write(f"# port={port} baud={BAUD} started={datetime.now().isoformat()}\n")
        buf = b""
        while not stop.is_set():
            try:
                chunk = ser.read(4096)
            except Exception as exc:
                fh.write(f"# READ ERROR: {exc}\n")
                st["err"] = str(exc)
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").rstrip("\r")
                ts = time.monotonic() - t0
                fh.write(f"{ts:9.3f} {line}\n")
                st["lines"] += 1
                # Self-identify from the boot banner: COM numbers are not identity.
                if "CYL=" in line:
                    st["cyl"] = line.split("CYL=")[-1].strip()[:1]
    try:
        ser.close()
    except Exception:
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ports", default=",".join(DEFAULT_PORTS))
    ap.add_argument("--tag", default="capture")
    args = ap.parse_args()

    ports = [p.strip() for p in args.ports.split(",") if p.strip()]
    os.makedirs(LOG_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y-%m-%d_%H.%M.%S")

    threads = []
    print(f"Capturing {len(ports)} boards -> {LOG_DIR}")
    for port in ports:
        path = os.path.join(LOG_DIR, f"{args.tag}_{port}_{stamp}.txt")
        print(f"  {port} -> {os.path.basename(path)}")
        th = threading.Thread(target=reader, args=(port, path), daemon=True)
        th.start()
        threads.append(th)

    time.sleep(2.5)          # let the reset-induced banners land
    print("\nREADY - crank when you want. Ctrl-C to stop.\n")

    last = 0
    try:
        while True:
            time.sleep(2.0)
            with lock:
                total = sum(s["lines"] for s in stats.values())
                parts = [f"{p}(cyl{stats[p]['cyl']})={stats[p]['lines']}"
                         for p in ports if p in stats]
                rate = (total - last) / 2.0
                last = total
                print(f"  [{time.monotonic()-t0:7.1f}s] " + "  ".join(parts)
                      + f"   {rate:.0f} lines/s")
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        for th in threads:
            th.join(timeout=1.0)
        print("\nStopped. Totals:")
        for p in ports:
            s = stats.get(p, {})
            print(f"  {p}: cyl={s.get('cyl','?')} lines={s.get('lines',0)}"
                  + (f" ERR={s['err']}" if s.get("err") else ""))


if __name__ == "__main__":
    main()
