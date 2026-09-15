#!/usr/bin/env python3
"""Cut a small, scrubbed test fixture out of a real dataflash log.

    python tools/make_fixture.py flight.bin tests/fixtures/name.bin \
        --keep ESC,CTUN,RCOU,RCIN,ATT,BAT,GPS,GPA,UBX2,EV,MODE,ARM,PARM,MSG,VER,PM,DSF,MOTB \
        --decimate ESC=5 --origin 30.0,-140.0,100.0

A real flight log is the only honest test input for a segmenter or a motor check, but a
real log is 10-16 MB and carries the pilot's position. This tool writes a new .bin that
holds only the message types a test needs, optionally decimated per instance, with every
identifying value rewritten:

  * every Lat/Lng/Lon field in every kept message is shifted by one constant so the first
    3D fix lands on --origin (default: 30 N 140 W, open ocean). Relative motion is kept,
    absolute position is not. Zero (no-fix) coordinates stay zero.
  * every absolute altitude field (GPS.Alt, AltAMSL, ORGN/POS/AHR2 Alt) is shifted so the
    first 3D fix sits at the --origin altitude.
  * the MCU serial words in the board banner ("<board> 0032001A 3233510C 34373333") are
    zeroed. Every other MSG text must match an allow-list of known ArduPilot messages or it
    is dropped and listed, so nothing unreviewed ships.
  * FILE records (embedded @SYS files), and any parameter carrying an address or an
    identifier (NET_*ADDR*, ADSB_ICAO_ID), are dropped.

Every FMT/FMTU/UNIT/MULT record is kept so the fixture is self-describing exactly like the
original. The tool prints what it kept, what it dropped and every MSG text that survives,
so the scrub can be reviewed before the file is committed. It is not used by the toolkit.
"""
from __future__ import annotations

import argparse
import os
import re
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dflog.parser import (FMT_BODY_LEN, FMT_STRUCT, FMT_TYPE, HEADER, INSTANCE_FIELD_CANDIDATES,
                          MessageFormat, _cstr)                              # noqa: E402

META = {"FMT", "FMTU", "UNIT", "MULT"}
LATLNG = {"Lat", "Lng", "Lon", "Latitude", "Longitude"}
ALT = {"Alt", "AltAMSL"}
ALT_MSGS = {"GPS", "POS", "AHR2", "ORGN", "BARO", "CMD", "TERR"}     # absolute-altitude carriers
PARAM_DROP = re.compile(r"^(NET_.*(ADDR|IP).*|ADSB_ICAO_ID)$")
BOARD_BANNER = re.compile(r"^([A-Za-z0-9_\-]+)(\s+[0-9A-Fa-f]{8}){3}\b")

# MSG texts ArduPilot writes that carry nothing about the pilot or the place.
MSG_ALLOW = [
    r"^(ArduCopter|ArduPlane|ArduRover|Rover|ArduSub|Blimp|AntennaTracker) V",
    r"^ChibiOS: ", r"^Param space used: ", r"^RC Protocol: ", r"^RCOut: ", r"^New (mission|rally|fence)$",
    r"^Frame: ", r"^GPS \d: (probing|detected|specified)", r"^u-blox \d HW: ", r"^EKF3 IMU\d ",
    r"^PreArm: ", r"^Arm: ", r"^IMU\d: fast sampling", r"^Mode change to ", r"^GPS Glitch", r"^Glitch cleared",
    r"^Radio Failsafe", r"^Field Elevation Set: ", r"^GCS:", r"^Fence ", r"^Land complete", r"^Throttle ",
    r"^Disarming ", r"^Crash: ", r"^Vibration compensation ", r"^Bad ", r"^Compass ", r"^Battery \d ",
    r"^Autotune", r"^AutoTune", r"^Starting ", r"^SmartRTL ", r"^Reached ", r"^Terrain ", r"^EKF primary ",
    r"^EKF variance", r"^Yaw re-aligned", r"^Hover ", r"^MOT_THST_HOVER", r"^Learned ",
]


def _instance(mf, row):
    for c in INSTANCE_FIELD_CANDIDATES:
        if c in mf.columns:
            return row[mf.columns.index(c)]
    return None


def _shift(raw, ch, delta):
    """Add a physical delta to a raw (unscaled) field value of format char `ch`."""
    if ch == "L":
        return int(raw + round(delta * 1e7))
    if ch in "cCeE":
        return int(raw + round(delta * 100))
    if ch in "fd":
        return float(raw + delta)
    if ch in "hHiIqQ":
        return int(raw + round(delta))
    return raw


def _scaled(raw, ch):
    if ch == "L":
        return raw * 1e-7
    if ch in "cCeE":
        return raw * 0.01
    return raw


def walk(data):
    """Yield (offset, type_id, body) for every well-formed packet; stops at the first
    thing it cannot decode, exactly where the parser would report a truncated tail."""
    formats = {}
    i, n = 0, len(data)
    while i + 3 <= n:
        if data[i] != HEADER[0] or data[i + 1] != HEADER[1]:
            nxt = data.find(HEADER, i + 1)
            if nxt < 0:
                return
            i = nxt
            continue
        t = data[i + 2]
        if t == FMT_TYPE:
            if i + 3 + FMT_BODY_LEN > n:
                return
            body = data[i + 3:i + 3 + FMT_BODY_LEN]
            ft, length, name, fmt, cols = FMT_STRUCT.unpack(body)
            name, fmt = _cstr(name), _cstr(fmt)
            cols = [c for c in _cstr(cols).split(",") if c]
            try:
                formats[ft] = MessageFormat(ft, length, name, fmt, cols)
            except ValueError:
                pass
            yield i, t, body, None
            i += 3 + FMT_BODY_LEN
            continue
        mf = formats.get(t)
        if mf is None:
            nxt = data.find(HEADER, i + 1)
            if nxt < 0:
                return
            i = nxt
            continue
        end = i + 3 + mf.body_len
        if end > n:
            return
        yield i, t, data[i + 3:end], mf
        i = end


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--keep", required=True, help="comma-separated message names to keep (FMT/FMTU/UNIT/MULT always)")
    ap.add_argument("--decimate", default="", help="per-instance decimation, e.g. ESC=5,IMU=2")
    ap.add_argument("--origin", default="30.0,-140.0,100.0", help="fake lat,lng,alt the first 3D fix is moved to")
    ap.add_argument("--start", type=float, default=None, help="drop timestamped messages before this many seconds")
    ap.add_argument("--end", type=float, default=None, help="drop timestamped messages after this many seconds")
    args = ap.parse_args(argv)

    keep = set(x for x in args.keep.split(",") if x) | META
    decim = {}
    for item in args.decimate.split(","):
        if item:
            k, v = item.split("=")
            decim[k] = int(v)
    fake_lat, fake_lng, fake_alt = (float(x) for x in args.origin.split(","))

    with open(args.src, "rb") as fh:
        data = fh.read()

    # Pass 1: the origin of the shift - the first GPS row with a 3D fix.
    origin = None
    for _off, _t, body, mf in walk(data):
        if mf is None or mf.name != "GPS":
            continue
        vals = mf.struct.unpack(body)
        row = dict(zip(mf.columns, vals))
        if row.get("Status", 3) >= 3 and row.get("Lat", 0) != 0:
            lat = _scaled(row["Lat"], mf.format[mf.columns.index("Lat")])
            lng = _scaled(row["Lng"], mf.format[mf.columns.index("Lng")])
            alt = _scaled(row.get("Alt", 0), mf.format[mf.columns.index("Alt")]) if "Alt" in row else 0.0
            origin = (lat, lng, alt)
            break
    if origin is None:
        print("no GPS 3D fix in the source: positions are not shifted (none to shift)", file=sys.stderr)
        d_lat = d_lng = d_alt = 0.0
    else:
        d_lat, d_lng, d_alt = fake_lat - origin[0], fake_lng - origin[1], fake_alt - origin[2]

    # Pass 2: copy, filter, decimate, scrub.
    out = bytearray()
    counters = {}
    kept, dropped = {}, {}
    msgs_kept, msgs_dropped = [], []
    allow = [re.compile(p) for p in MSG_ALLOW]
    for _off, t, body, mf in walk(data):
        if mf is None:                                   # FMT: always kept
            out += HEADER + bytes([t]) + body
            continue
        name = mf.name
        if name not in keep:
            dropped[name] = dropped.get(name, 0) + 1
            continue
        vals = list(mf.struct.unpack(body))
        cols = mf.columns
        if "TimeUS" in cols and name not in META:
            ts = vals[cols.index("TimeUS")] / 1e6
            if (args.start is not None and ts < args.start) or (args.end is not None and ts > args.end):
                dropped[name] = dropped.get(name, 0) + 1
                continue
        step = decim.get(name)
        if step and step > 1:
            key = (name, _instance(mf, vals))
            c = counters.get(key, 0)
            counters[key] = c + 1
            if c % step:
                dropped[name] = dropped.get(name, 0) + 1
                continue
        # -- scrub
        if name == "PARM":
            pname = vals[cols.index("Name")]
            pname = pname.split(b"\x00", 1)[0].decode("ascii", "replace")
            if PARAM_DROP.match(pname):
                dropped["PARM(scrubbed)"] = dropped.get("PARM(scrubbed)", 0) + 1
                continue
        if name == "MSG":
            j = cols.index("Message")
            text = vals[j].split(b"\x00", 1)[0].decode("ascii", "replace")
            m = BOARD_BANNER.match(text)
            if m:
                text = m.group(1) + " 00000000 00000000 00000000"
            elif not any(p.match(text) for p in allow):
                msgs_dropped.append(text)
                continue
            msgs_kept.append(text)
            width = len(vals[j])
            vals[j] = text.encode("ascii")[:width].ljust(width, b"\x00")
        for j, (col, ch) in enumerate(zip(cols, mf.format)):
            if col in LATLNG and vals[j] != 0:
                vals[j] = _shift(vals[j], ch, d_lat if col in ("Lat", "Latitude") else d_lng)
            elif col in ALT and name in ALT_MSGS and vals[j] != 0:
                vals[j] = _shift(vals[j], ch, d_alt)
        out += HEADER + bytes([t]) + mf.struct.pack(*vals)
        kept[name] = kept.get(name, 0) + 1

    with open(args.dst, "wb") as fh:
        fh.write(out)

    print(f"wrote {args.dst}: {len(out)} bytes from {len(data)}")
    if origin is not None:
        print(f"positions shifted so the first 3D fix is at {fake_lat}, {fake_lng}, {fake_alt} m")
    print("kept:    " + ", ".join(f"{k}={v}" for k, v in sorted(kept.items())))
    print("dropped: " + ", ".join(f"{k}={v}" for k, v in sorted(dropped.items())))
    print("MSG texts kept:")
    for m in msgs_kept:
        print("   " + m)
    if msgs_dropped:
        print("MSG texts DROPPED (not on the allow-list):")
        for m in msgs_dropped:
            print("   " + m)
    return 0


if __name__ == "__main__":
    sys.exit(main())
