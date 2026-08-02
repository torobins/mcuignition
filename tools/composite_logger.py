#!/usr/bin/env python3
"""Live composite-logger monitor for Speeduino's PRIMARY (USB) serial port.

Unlike speeduino_monitor.py (which polls the secondary/TTL port and can run
alongside TunerStudio), this talks to the PRIMARY serial port -- the same one
TunerStudio uses -- so TunerStudio MUST be closed while this runs. Only one
client can hold that port at a time (see comms_legacy.cpp / comms_secondary.cpp
in the speeduino firmware: the secondary port's restricted command set never
implements 'T'/'H'/'h'/'J'/'j', so tooth/composite logging is primary-only).

Protocol (traced from speeduino/speeduino comms_legacy.cpp, decoders.cpp):
  - send 'J' (0x4A)            -> start composite logger; board replies 1 byte (0x01 ack)
  - send 'T' (0x54) + 6 bytes  -> read back the composite log buffer. The 6
                                   bytes are a legacy page/offset/length header
                                   that the firmware ignores for this command;
                                   send zeros.
  - response: TOOTH_LOG_SIZE (127 on a standard, non-small-flash AVR build)
    entries of 5 bytes each = 635 bytes total:
        bytes[0:4]  u32 BE   running timestamp in microseconds
        bytes[4]    u8       status flags:
                                 bit0 COMPOSITE_LOG_PRI   primary trigger pin state
                                 bit1 COMPOSITE_LOG_SEC   secondary trigger pin state
                                 bit2 COMPOSITE_LOG_THIRD tertiary (cam2) pin state
                                 bit3 COMPOSITE_LOG_TRIG  which physical trigger fired this edge
                                 bit4 COMPOSITE_LOG_SYNC  full sync achieved at this edge
    If the buffer hasn't refilled since the last read, the firmware sends
    635 zero bytes instead of real data -- detect and skip these.
  - send 'j' (0x6A)            -> stop composite logger (send on exit)

Caveat: this was traced against the speeduino/speeduino GitHub `master` branch
source, not read off Todd's actual board firmware -- confirm TOOTH_LOG_SIZE
and the command bytes still match once the board responds in TunerStudio.

Requires: pyserial (already installed: `pip show pyserial`).
"""
import argparse
import struct
import sys
import time

import serial

TOOTH_LOG_SIZE = 127  # entries per buffer (non-small-flash-mode AVR build)
ENTRY_SIZE = 5  # 4-byte timestamp + 1-byte flags
RESPONSE_SIZE = TOOTH_LOG_SIZE * ENTRY_SIZE

CMD_START = b"J"
CMD_STOP = b"j"
CMD_READ = b"T"
READ_HEADER = b"\x00" * 6  # page/offset/length header the firmware ignores here

BIT_PRI = 0
BIT_SEC = 1
BIT_THIRD = 2
BIT_TRIG = 3
BIT_SYNC = 4


def start_logger(ser: serial.Serial):
    ser.reset_input_buffer()
    ser.write(CMD_START)
    ack = ser.read(1)
    if ack != b"\x01":
        print(f"[warn] unexpected start ack: {ack!r} (expected b'\\x01')", file=sys.stderr)


def stop_logger(ser: serial.Serial):
    ser.write(CMD_STOP)


def read_log(ser: serial.Serial):
    ser.write(CMD_READ + READ_HEADER)
    buf = bytearray()
    deadline = time.monotonic() + 0.3
    while len(buf) < RESPONSE_SIZE:
        if time.monotonic() > deadline:
            return None, f"timeout, got {len(buf)}/{RESPONSE_SIZE} bytes"
        chunk = ser.read(RESPONSE_SIZE - len(buf))
        if chunk:
            buf += chunk

    if buf == bytes(RESPONSE_SIZE):
        return [], None  # buffer hasn't refilled since last read

    entries = []
    for i in range(0, RESPONSE_SIZE, ENTRY_SIZE):
        ts_us = struct.unpack_from(">I", buf, i)[0]
        flags = buf[i + 4]
        entries.append({
            "ts_us": ts_us,
            "pri": bool(flags & (1 << BIT_PRI)),
            "sec": bool(flags & (1 << BIT_SEC)),
            "third": bool(flags & (1 << BIT_THIRD)),
            "trig": bool(flags & (1 << BIT_TRIG)),
            "sync": bool(flags & (1 << BIT_SYNC)),
        })
    return entries, None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", help="COM port of Speeduino's PRIMARY/USB connection (e.g. COM5). TunerStudio must be closed.")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--interval", type=float, default=0.1, help="seconds between 'T' polls (default 0.1)")
    ap.add_argument("--log", help="optional CSV file to append decoded edges to")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.3)
    time.sleep(2)  # let the board settle after DTR-triggered reset on open

    log_fh = open(args.log, "a", buffering=1) if args.log else None
    if log_fh and log_fh.tell() == 0:
        log_fh.write("epoch,ts_us,delta_us,pri,sec,third,trig,sync\n")

    print(f"Starting composite logger on {args.port} @ {args.baud} baud. Ctrl-C to stop.")
    start_logger(ser)

    last_ts = None
    try:
        while True:
            t0 = time.monotonic()
            entries, err = read_log(ser)
            now = time.time()
            if err:
                print(f"[no reply] {err}", file=sys.stderr)
            else:
                for e in entries:
                    delta = "" if last_ts is None else e["ts_us"] - last_ts
                    last_ts = e["ts_us"]
                    flags_str = "".join([
                        "P" if e["pri"] else "-",
                        "S" if e["sec"] else "-",
                        "T" if e["third"] else "-",
                        "X" if e["trig"] else "-",
                        "Y" if e["sync"] else "-",
                    ])
                    delta_str = f"{delta:6d}us" if delta != "" else "   -   "
                    print(f"t={e['ts_us']:10d}us  d={delta_str}  flags={flags_str}")
                    if log_fh:
                        log_fh.write(f"{now:.3f},{e['ts_us']},{delta},{int(e['pri'])},{int(e['sec'])},"
                                      f"{int(e['third'])},{int(e['trig'])},{int(e['sync'])}\n")

            elapsed = time.monotonic() - t0
            if elapsed < args.interval:
                time.sleep(args.interval - elapsed)
    except KeyboardInterrupt:
        pass
    finally:
        stop_logger(ser)
        ser.close()
        if log_fh:
            log_fh.close()


if __name__ == "__main__":
    main()
