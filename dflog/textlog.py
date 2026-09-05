"""Reader for the text (.log) DataFlash export written by Mission Planner and friends.

The text form is the same message stream printed one record per line:

    FMT, 128, 89, FMT, BBnNZ, Type,Length,Name,Format,Columns
    FMT, 86, 40, ESC, QBffffcfcf, TimeUS,Instance,RPM,RawRPM,Volt,Curr,Temp,CTot,MotTemp,Err
    ESC, 175291234, 0, 4823, 4823, 0, 0, 21, 0, 0, 3.2

Two things make it a second-class source, and both are reported as diagnostics so a
reader cannot mistake it for the .bin:

* Values are already scaled by the exporter (c/C/e/E divided by 100, L by 1e7), so this
  reader applies no scaling of its own. pymavlink's DFReader_text does the same.
* The exporter may decimate or drop records (per-instance ESC telemetry in particular),
  and int16[32] array fields (`a`, i.e. ISBD) do not round-trip. Prefer the .bin whenever
  it exists - see reference/pitfalls.md.

Nothing here is silently repaired: an unknown message name, a line with the wrong field
count, or a non-numeric value in a numeric column is counted and reported with the line
number of its first occurrence.
"""

from __future__ import annotations

import math
from collections import defaultdict

from .parser import MessageFormat, FMT_TYPE, _METADATA_MSGS  # noqa: F401

__all__ = ["looks_like_text_log", "parse_text"]

_NUMERIC_INT = set("bBhHiIqQM")
_NUMERIC_FLOAT = set("fdgcCeEL")
_STRINGS = set("nNZ")


def looks_like_text_log(head: bytes) -> bool:
    """True if the first bytes read like the text export (an 'FMT,' line, ASCII)."""
    if head[:3] == b"\xa3\x95\x80":
        return False
    try:
        text = head[:8000].decode("ascii")
    except UnicodeDecodeError:
        return False
    return "FMT," in text or "FMT, " in text


def parse_text(log, data: bytes):
    """Populate `log` (a dflog.parser.Log) from text-export bytes."""
    diag = log.diagnostics
    log.source_format = "text"
    diag.add("TEXT_LOG", "warning",
             "this is a text (.log) export, not the .bin: values are pre-scaled by the exporter, "
             "records may be decimated (per-instance ESC telemetry especially), and int16[32] "
             "array fields (ISBD) do not round-trip. Prefer the .bin when it exists.")
    text = data.decode("utf-8", "replace")
    lines = text.splitlines()
    formats_by_name = log.formats_by_name
    msgs = defaultdict(list)
    unknown = defaultdict(int)
    unknown_first = {}
    bad_count = defaultdict(int)
    bad_first = {}
    bad_value = defaultdict(int)
    bad_value_first = {}
    seen_fmt = False
    n_records = 0

    for lineno, line in enumerate(lines, 1):
        line = line.strip()
        if not line:
            continue
        parts = [p.strip() for p in line.split(",")]
        name = parts[0]
        if name == "FMT":
            # FMT, type, length, name, format, col1, col2, ...  (columns re-split on ',')
            if len(parts) < 6:
                diag.add("TEXT_BAD_FMT", "error", f"FMT line with {len(parts)} fields", offset=lineno)
                continue
            seen_fmt = True
            try:
                t = int(parts[1])
                length = int(parts[2])
            except ValueError:
                diag.add("TEXT_BAD_FMT", "error", f"FMT line with non-numeric type/length: {line[:60]}",
                         offset=lineno)
                continue
            mname, fmt = parts[3], parts[4]
            cols = [c for c in parts[5:] if c]
            try:
                mf = MessageFormat(t, length, mname, fmt, cols)
            except ValueError as exc:
                diag.add("FMT_UNKNOWN_FORMAT_CHAR", "error",
                         f"{exc}; every {mname} line in this log is undecodable", subject=mname, offset=lineno)
                continue
            if len(cols) != len(fmt):
                diag.add("FMT_COLUMN_COUNT_MISMATCH", "error",
                         f"{mname}: {len(cols)} column names for {len(fmt)} format chars", subject=mname,
                         offset=lineno)
                while len(mf.columns) < len(fmt):
                    mf.columns.append(f"F{len(mf.columns)}")
                mf.columns = mf.columns[:len(fmt)]
            prev = formats_by_name.get(mname)
            if prev is not None and not prev.same_definition(mf):
                diag.add("FMT_REDEFINED", "warning", f"{mname} redefined at line {lineno}", subject=mname,
                         offset=lineno)
            formats_by_name[mname] = mf
            log.formats[t] = mf
            msgs["FMT"].append(dict(Type=t, Length=length, Name=mname, Format=fmt, Columns=",".join(cols)))
            continue

        mf = formats_by_name.get(name)
        if mf is None:
            unknown[name] += 1
            unknown_first.setdefault(name, lineno)
            continue
        vals = parts[1:]
        if "a" in mf.format:
            bad_count[name] += 1
            bad_first.setdefault(name, lineno)
            continue
        if len(vals) != len(mf.columns):
            last_is_str = mf.kinds and mf.kinds[-1] in ("str", "bytes")
            if last_is_str and len(vals) > len(mf.columns):
                # a trailing text field containing commas (MSG text, FILE data)
                head = vals[:len(mf.columns) - 1]
                vals = head + [", ".join(vals[len(mf.columns) - 1:])]
            elif last_is_str and len(vals) == len(mf.columns) - 1:
                vals = vals + [""]          # empty trailing string (UNIT '-' has label "")
            else:
                bad_count[name] += 1
                bad_first.setdefault(name, lineno)
                continue
        row = {}
        ok = True
        for col, ch, kind, raw in zip(mf.columns, mf.format, mf.kinds, vals):
            if kind in ("str", "bytes"):
                row[col] = raw
            elif ch == "M" and not raw.lstrip("-").isdigit():
                # pymavlink's text reader keeps the mode as a string; map it back to the
                # copter number when it is one, else record it as unknown.
                from .flight import MODES
                rev = {v.replace("_", ""): k for k, v in MODES.items()}
                key = raw.upper().replace("_", "").replace(" ", "")
                if key in rev:
                    row[col] = rev[key]
                else:
                    row[col] = None
                    ok = False
            elif ch in _NUMERIC_INT:
                try:
                    row[col] = int(raw)
                except ValueError:
                    try:
                        row[col] = int(float(raw))
                    except ValueError:
                        row[col] = None
                        ok = False
            else:
                try:
                    row[col] = float(raw)
                except ValueError:
                    row[col] = math.nan
                    ok = False
        if not ok:
            bad_value[name] += 1
            bad_value_first.setdefault(name, lineno)
        msgs[name].append(row)
        n_records += 1

    for name, cnt in unknown.items():
        diag.add("TEXT_UNKNOWN_TYPE", "error",
                 f"{cnt} line(s) of message {name!r} with no FMT definition; skipped",
                 subject=name, offset=unknown_first[name], lines=cnt)
    for name, cnt in bad_count.items():
        why = ("int16[32] array fields cannot be read from the text export" if "a" in formats_by_name[name].format
               else "field count does not match the FMT")
        diag.add("TEXT_FIELD_COUNT", "warning" if "a" in formats_by_name[name].format else "error",
                 f"{cnt} {name} line(s) skipped: {why}", subject=name, offset=bad_first[name], lines=cnt)
    for name, cnt in bad_value.items():
        diag.add("TEXT_BAD_VALUE", "warning",
                 f"{cnt} {name} line(s) had a non-numeric value in a numeric column (stored as NaN)",
                 subject=name, offset=bad_value_first[name], lines=cnt)
    if not seen_fmt:
        diag.add("NO_FMT", "error", "no FMT lines found; not a DataFlash text export")

    log.messages = dict(msgs)
    log.n_messages = sum(len(v) for v in msgs.values())
    log.bytes_parsed = len(data)
    log._attach_units()
    log._post_parse_checks()
