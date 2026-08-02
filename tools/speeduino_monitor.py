#!/usr/bin/env python3
"""Live realtime-data monitor for Speeduino's secondary serial output.

Talks to Speeduino's Serial3 (secondarySerialProtocol = "Generic (Fixed
List)"), NOT the primary USB port TunerStudio uses -- this runs concurrently
with TunerStudio, same as the speeduino-dash ESP32 firmware's poll loop
(../../speeduino-dash/src/main.cpp), whose validated 'A'-command protocol and
byte offsets this script reuses:

  - send 0x41 ('A')
  - read 76 bytes back; byte 0 must echo 0x41 (confirm)
  - payload = bytes[1:76], offsets into payload:
      MAP     @4   u16 LE   kPa
      CLT     @7   u8       raw+40 = degC
      Battery @9   u8       volts x10
      RPM     @14  u16 LE
      Advance @23  i8       deg
      TPS     @24  u8       percent

Requires: pyserial (already installed: `pip show pyserial`).
"""
import argparse
import struct
import sys
import time

import serial

A_CMD_RESPONSE = 76  # 1 confirm byte + 75 data bytes

OFF_MAP = 4
OFF_CLT = 7
OFF_BATT = 9
OFF_RPM = 14
OFF_ADVANCE = 23
OFF_TPS = 24


def poll(ser: serial.Serial):
    ser.reset_input_buffer()
    ser.write(b"A")

    buf = bytearray()
    deadline = time.monotonic() + 0.2
    while len(buf) < A_CMD_RESPONSE:
        if time.monotonic() > deadline:
            return None, f"timeout, got {len(buf)}/{A_CMD_RESPONSE} bytes"
        chunk = ser.read(A_CMD_RESPONSE - len(buf))
        if chunk:
            buf += chunk

    if buf[0] != 0x41:
        return None, f"bad confirm byte: 0x{buf[0]:02X} (expected 0x41)"

    d = buf[1:]
    rpm = struct.unpack_from("<H", d, OFF_RPM)[0]
    map_kpa = struct.unpack_from("<H", d, OFF_MAP)[0]
    clt_c = d[OFF_CLT] - 40
    batt_v = d[OFF_BATT] / 10.0
    advance = struct.unpack_from("<b", d, OFF_ADVANCE)[0]
    tps = d[OFF_TPS]

    return {
        "rpm": rpm,
        "map_kpa": map_kpa,
        "clt_c": clt_c,
        "batt_v": batt_v,
        "advance_deg": advance,
        "tps_pct": tps,
    }, None


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("port", help="COM port of the USB-TTL adapter wired to Speeduino Serial3 (e.g. COM7)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--interval", type=float, default=0.15, help="seconds between polls (default 0.15, matches dash firmware)")
    ap.add_argument("--log", help="optional CSV file to append readings to")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.25)
    time.sleep(2)  # let the adapter/board settle

    log_fh = open(args.log, "a", buffering=1) if args.log else None
    if log_fh and log_fh.tell() == 0:
        log_fh.write("epoch,rpm,map_kpa,clt_c,batt_v,advance_deg,tps_pct\n")

    print(f"Polling Speeduino secondary serial on {args.port} @ {args.baud} baud. Ctrl-C to stop.")
    try:
        while True:
            t0 = time.monotonic()
            data, err = poll(ser)
            now = time.time()
            if data:
                line = (f"RPM={data['rpm']:5d}  MAP={data['map_kpa']:4d}kPa  "
                        f"CLT={data['clt_c']:4d}C  BATT={data['batt_v']:.1f}V  "
                        f"ADV={data['advance_deg']:3d}deg  TPS={data['tps_pct']:3d}%")
                print(line)
                if log_fh:
                    log_fh.write(f"{now:.3f},{data['rpm']},{data['map_kpa']},{data['clt_c']},"
                                 f"{data['batt_v']},{data['advance_deg']},{data['tps_pct']}\n")
            else:
                print(f"[no reply] {err}", file=sys.stderr)

            elapsed = time.monotonic() - t0
            if elapsed < args.interval:
                time.sleep(args.interval - elapsed)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if log_fh:
            log_fh.close()


if __name__ == "__main__":
    main()
