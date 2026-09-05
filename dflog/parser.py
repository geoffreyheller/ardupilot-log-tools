"""
ArduPilot DataFlash (.bin) log reader - pure stdlib + numpy, and it fails loudly.

This module parses the self-describing DataFlash format directly, with no dependency on
pymavlink. It exists for two reasons: pymavlink is awkward to vendor (see
reference/existing-tools.md), and a parser whose every decision is visible is easier to
trust than one whose recovery paths are hidden inside a library.

Typical use:

    from dflog import Log
    log = Log("flight.bin")                  # parses, then caches to .dfcache
    log.diagnostics.ok                       # False if anything in the file was wrong
    print(log.diagnostics.render())          # every structural problem, with byte offsets
    att  = log.df("ATT")                     # pandas DataFrame
    esc  = log.instances("ESC")              # {0: df, 1: df, 2: df, 3: df}
    print(log.types())                       # what's in this log

The parse is fast (about 1 s for a 16 MB log on a laptop). The first parse writes a
pickle cache beside the log (`<name>.dfcache`); later loads take well under a second.
Delete the cache or pass `use_cache=False` to force a re-parse.

Fail-loud contract
------------------
Nothing in the input is ever silently repaired. Every deviation from a well-formed log -
truncation, resync, an unknown message type, a malformed FMT, a cache that had to be
rebuilt - is recorded in `log.diagnostics` with a stable code, a severity, a byte offset
and a count. `Log(path, strict=True)` raises `LogIntegrityError` if any error-level
issue was found. `alog.py` prints the diagnostics at the top of every report and folds
them into the exit code. The codes are documented in reference/integrity-codes.md.
"""

from __future__ import annotations

import hashlib
import os
import pickle
import re
import struct
import sys
from collections import defaultdict

import numpy as np

try:
    import pandas as pd
except ImportError:  # pragma: no cover - pandas is a hard dependency in practice
    pd = None

__all__ = ["Log", "FORMAT_CHARS", "CACHE_VERSION", "Diagnostics", "Issue",
           "LogIntegrityError", "gps_to_unix"]

# Bump when the parser's output structure changes, so stale caches are ignored.
CACHE_VERSION = 4

HEAD1 = 0xA3
HEAD2 = 0x95
HEADER = b"\xa3\x95"
FMT_TYPE = 0x80          # 128
FMT_BODY_LEN = 86        # NOT 89 - the 3-byte packet header is not part of the body
FMT_STRUCT = struct.Struct("<BB4s16s64s")
FMT_SELF_FORMAT = "BBnNZ"

# DataFlash format characters (libraries/AP_Logger/LogStructure.h).
#   size  = bytes on the wire
#   pack  = struct code
#   scale = multiplier applied on decode (format-char scaling only; MULT is never applied)
FORMAT_CHARS = {
    "a": dict(size=64, pack="64s", scale=None, desc="int16_t[32]"),
    "b": dict(size=1,  pack="b",   scale=None, desc="int8_t"),
    "B": dict(size=1,  pack="B",   scale=None, desc="uint8_t"),
    "h": dict(size=2,  pack="h",   scale=None, desc="int16_t"),
    "H": dict(size=2,  pack="H",   scale=None, desc="uint16_t"),
    "i": dict(size=4,  pack="i",   scale=None, desc="int32_t"),
    "I": dict(size=4,  pack="I",   scale=None, desc="uint32_t"),
    "f": dict(size=4,  pack="f",   scale=None, desc="float"),
    "d": dict(size=8,  pack="d",   scale=None, desc="double"),
    "g": dict(size=2,  pack="e",   scale=None, desc="float16"),
    "n": dict(size=4,  pack="4s",  scale=None, desc="char[4]"),
    "N": dict(size=16, pack="16s", scale=None, desc="char[16]"),
    "Z": dict(size=64, pack="64s", scale=None, desc="char[64]"),
    "c": dict(size=2,  pack="h",   scale=0.01, desc="int16_t * 100"),
    "C": dict(size=2,  pack="H",   scale=0.01, desc="uint16_t * 100"),
    "e": dict(size=4,  pack="i",   scale=0.01, desc="int32_t * 100"),
    "E": dict(size=4,  pack="I",   scale=0.01, desc="uint32_t * 100"),
    "L": dict(size=4,  pack="i",   scale=1e-7, desc="int32_t degE7 (lat/lng)"),
    "M": dict(size=1,  pack="B",   scale=None, desc="uint8_t flight mode"),
    "q": dict(size=8,  pack="q",   scale=None, desc="int64_t"),
    "Q": dict(size=8,  pack="Q",   scale=None, desc="uint64_t"),
}

_STRING_CHARS = set("nNZ")
_ARRAY_CHARS = set("a")
_FLOAT_CHARS = set("fdg")

# Field names ArduPilot uses for the instance/core index, most specific first.
INSTANCE_FIELD_CANDIDATES = ("Instance", "Inst", "IMU", "Core", "C", "I", "Id", "N")

# Messages whose timestamps are allowed to run backwards without it meaning anything:
# metadata the logger emits on first use rather than in time order.
_METADATA_MSGS = {"FMT", "FMTU", "UNIT", "MULT", "PARM", "MSG", "FILE", "VER", "DFLT"}

# Fields that are NaN by design on some builds (LogAnalyzer TestNaN allow-list, plus
# fields that read NaN on boards without the sensor).
NAN_ALLOWED = {
    ("CTUN", "DSAlt"), ("CTUN", "TAlt"), ("POS", "RelOriginAlt"),
    ("POWR", "Vcc"), ("POWR", "VServo"), ("BAT", "Res"), ("BAT", "Temp"),
    ("ESC", "Temp"), ("ESC", "MotTemp"), ("MCU", "MTemp"),
    ("PARM", "Default"),     # ArduPilot writes NaN when it has no default for a parameter
    ("FCNS", "CF"), ("FCNS", "HF"), ("FCN", "CF1"),   # notch centre is NaN while its source has no data
}

_SEVERITY_RANK = {"info": 0, "warning": 1, "error": 2}
_MAX_EXAMPLES = 8


def gps_to_unix(week, ms, leap_seconds=18):
    """GPS week + milliseconds-of-week -> Unix epoch seconds (UTC).

    GPS time started 1980-01-06 and does not count leap seconds; UTC does. 18 s is right
    for 2017 onwards. pymavlink's DFReader uses the same constant.
    """
    gps_epoch = 315964800.0          # 1980-01-06T00:00:00Z
    return gps_epoch + float(week) * 604800.0 + float(ms) / 1000.0 - leap_seconds


def _cstr(raw: bytes) -> str:
    return raw.split(b"\x00", 1)[0].decode("ascii", "replace").strip()


class LogIntegrityError(Exception):
    """Raised by Log(strict=True) when the file has error-level integrity issues."""

    def __init__(self, diagnostics, path=""):
        self.diagnostics = diagnostics
        self.path = path
        super().__init__(f"{path}: {diagnostics.summary()}")


class Issue:
    """One kind of problem found in a log, aggregated over every occurrence."""

    __slots__ = ("code", "severity", "message", "subject", "count", "offsets", "detail")

    def __init__(self, code, severity, message, subject=None, offset=None, **detail):
        self.code = code
        self.severity = severity
        self.message = message
        self.subject = subject
        self.count = 1
        self.offsets = [] if offset is None else [int(offset)]
        self.detail = dict(detail)

    @property
    def key(self):
        return (self.code, self.subject)

    def to_dict(self):
        return dict(code=self.code, severity=self.severity, subject=self.subject,
                    message=self.message, count=self.count,
                    first_offset=self.offsets[0] if self.offsets else None,
                    offsets=list(self.offsets), detail=self.detail)

    def line(self):
        where = f" at byte {self.offsets[0]}" if self.offsets else ""
        times = f" (x{self.count})" if self.count > 1 else ""
        subj = f" [{self.subject}]" if self.subject is not None else ""
        return f"[{self.severity.upper()}] {self.code}{subj}: {self.message}{where}{times}"

    def __repr__(self):
        return f"<Issue {self.severity} {self.code} {self.subject!r} x{self.count}>"


class Diagnostics:
    """Everything the parser found wrong with a file. Never empty-by-omission: a check
    that ran and found nothing simply adds nothing, and `ok` is then True."""

    def __init__(self):
        self._issues = {}
        self.order = []

    def add(self, code, severity, message, subject=None, offset=None, **detail):
        assert severity in _SEVERITY_RANK, severity
        key = (code, subject)
        iss = self._issues.get(key)
        if iss is None:
            iss = Issue(code, severity, message, subject, offset, **detail)
            self._issues[key] = iss
            self.order.append(key)
        else:
            iss.count += 1
            if offset is not None and len(iss.offsets) < _MAX_EXAMPLES:
                iss.offsets.append(int(offset))
            for k, v in detail.items():
                # keep running totals for numeric detail, first value otherwise
                if isinstance(v, (int, float)) and isinstance(iss.detail.get(k), (int, float)):
                    iss.detail[k] += v
                else:
                    iss.detail.setdefault(k, v)
        return iss

    @property
    def issues(self):
        return [self._issues[k] for k in self.order]

    def by_severity(self, severity):
        return [i for i in self.issues if i.severity == severity]

    @property
    def errors(self):
        return self.by_severity("error")

    @property
    def warnings(self):
        return self.by_severity("warning")

    @property
    def infos(self):
        return self.by_severity("info")

    @property
    def ok(self):
        """True when there are no error-level issues. Warnings do not clear this."""
        return not self.errors

    @property
    def worst(self):
        return max((i.severity for i in self.issues), key=_SEVERITY_RANK.get, default="ok")

    def has(self, code):
        return any(i.code == code for i in self.issues)

    def summary(self):
        e, w, n = len(self.errors), len(self.warnings), len(self.infos)
        if not self.issues:
            return "log integrity OK: no structural issues found"
        return f"{e} error(s), {w} warning(s), {n} info in log integrity"

    def render(self, min_severity="info"):
        lines = [self.summary()]
        floor = _SEVERITY_RANK[min_severity]
        for i in sorted(self.issues, key=lambda i: -_SEVERITY_RANK[i.severity]):
            if _SEVERITY_RANK[i.severity] >= floor:
                lines.append("  " + i.line())
        return "\n".join(lines)

    def to_dict(self):
        return dict(ok=self.ok, worst=self.worst, n_errors=len(self.errors),
                    n_warnings=len(self.warnings), n_info=len(self.infos),
                    issues=[i.to_dict() for i in self.issues])

    def __getstate__(self):
        return {"issues": [(k, self._issues[k]) for k in self.order]}

    def __setstate__(self, state):
        self._issues = {}
        self.order = []
        for k, iss in state["issues"]:
            self._issues[k] = iss
            self.order.append(k)

    def __repr__(self):
        return f"<Diagnostics {self.summary()}>"


class MessageFormat:
    """One FMT definition: how to unpack every message of a given type."""

    __slots__ = ("type", "length", "name", "format", "columns",
                 "struct", "body_len", "scales", "kinds", "units", "mults")

    def __init__(self, type_id, length, name, fmt, columns):
        self.type = type_id
        self.length = length
        self.name = name
        self.format = fmt
        self.columns = columns
        self.units = None      # filled in from FMTU/UNIT if present
        self.mults = None

        pack = "<"
        scales = []
        kinds = []
        size = 0
        for idx, ch in enumerate(fmt):
            spec = FORMAT_CHARS.get(ch)
            if spec is None:
                raise ValueError(f"unknown format char {ch!r} in {name} ({fmt!r})")
            pack += spec["pack"]
            scales.append(spec["scale"])
            if ch == "Z" and name == "FILE" and idx < len(columns) and columns[idx] == "Data":
                kinds.append("bytes")     # embedded-file chunks are binary, not text
            elif ch in _STRING_CHARS:
                kinds.append("str")
            elif ch in _ARRAY_CHARS:
                kinds.append("arr")
            elif ch in _FLOAT_CHARS:
                kinds.append("float")
            else:
                kinds.append("num")
            size += spec["size"]

        self.struct = struct.Struct(pack)
        self.body_len = size
        self.scales = scales
        self.kinds = kinds

    def same_definition(self, other):
        return (self.name == other.name and self.format == other.format
                and self.columns == other.columns and self.length == other.length)

    # struct.Struct is not picklable, so the cache stores only the definition
    # and the derived fields are rebuilt on load.
    def __getstate__(self):
        return (self.type, self.length, self.name, self.format, self.columns,
                self.units, self.mults)

    def __setstate__(self, state):
        type_id, length, name, fmt, columns, units, mults = state
        self.__init__(type_id, length, name, fmt, columns)
        self.units, self.mults = units, mults

    def to_dict(self):
        return dict(type=self.type, name=self.name, format=self.format,
                    columns=list(self.columns), packet_length=self.length,
                    units=list(self.units) if self.units else None,
                    mults=list(self.mults) if self.mults else None)

    def __repr__(self):
        return f"<FMT {self.name} type={self.type} fmt={self.format!r} cols={self.columns}>"


class Log:
    """A parsed ArduPilot dataflash log."""

    def __init__(self, path, use_cache=True, verbose=False, progress=False, strict=False):
        self.path = str(path)
        self.verbose = verbose
        self.formats: dict[int, MessageFormat] = {}
        self.formats_by_name: dict[str, MessageFormat] = {}
        self.messages: dict[str, list] = {}
        self.diagnostics = Diagnostics()
        self.resync_bytes = 0
        self.resync_events = 0
        self.n_messages = 0
        self.file_size = 0
        self.bytes_parsed = 0
        self.source_format = "bin"
        self._df_cache: dict[str, "pd.DataFrame"] = {}
        self._quality = None

        if not os.path.exists(self.path):
            raise FileNotFoundError(self.path)
        self.file_size = os.path.getsize(self.path)

        cache = self._cache_path()
        loaded = False
        if use_cache and os.path.exists(cache):
            try:
                self._load_cache(cache)
                loaded = True
                if verbose:
                    print(f"[dflog] loaded cache {cache}", file=sys.stderr)
            except Exception as exc:
                # A stale cache is normal after editing the log or upgrading the parser;
                # a corrupt one is not. Either way, say so rather than silently re-parse.
                self.diagnostics.add("CACHE_REBUILT", "info",
                                     f"cache {os.path.basename(cache)} unusable "
                                     f"({type(exc).__name__}: {exc}); re-parsed from the .bin")
        if not loaded:
            self._parse(progress=progress)
            # Never cache a failed parse: the next run must look at the file again.
            if use_cache and self.n_messages > 0:
                try:
                    self._save_cache(cache)
                except OSError as exc:
                    self.diagnostics.add("CACHE_WRITE_FAILED", "warning",
                                         f"could not write cache {cache}: {exc}; every run "
                                         "will re-parse")
        if strict and not self.diagnostics.ok:
            raise LogIntegrityError(self.diagnostics, self.path)

    # ------------------------------------------------------------------ cache

    def _cache_path(self):
        return self.path + ".dfcache"

    def _save_cache(self, cache):
        blob = dict(
            version=CACHE_VERSION,
            src_mtime=os.path.getmtime(self.path),
            src_size=self.file_size,
            formats=self.formats,
            messages=self.messages,
            diagnostics=self.diagnostics,
            resync_bytes=self.resync_bytes,
            resync_events=self.resync_events,
            n_messages=self.n_messages,
            bytes_parsed=self.bytes_parsed,
            source_format=self.source_format,
        )
        tmp = cache + ".tmp"
        with open(tmp, "wb") as fh:
            pickle.dump(blob, fh, protocol=pickle.HIGHEST_PROTOCOL)
        os.replace(tmp, cache)

    def _load_cache(self, cache):
        with open(cache, "rb") as fh:
            blob = pickle.load(fh)
        if blob.get("version") != CACHE_VERSION:
            raise ValueError(f"cache version {blob.get('version')} != {CACHE_VERSION}")
        if blob.get("src_size") != self.file_size:
            raise ValueError("log size changed since the cache was written")
        if abs(blob.get("src_mtime", -1) - os.path.getmtime(self.path)) > 1.0:
            raise ValueError("log mtime changed since the cache was written")
        self.formats = blob["formats"]
        self.messages = blob["messages"]
        self.diagnostics = blob["diagnostics"]
        self.resync_bytes = blob["resync_bytes"]
        self.resync_events = blob["resync_events"]
        self.n_messages = blob["n_messages"]
        self.bytes_parsed = blob["bytes_parsed"]
        self.source_format = blob.get("source_format", "bin")
        self.formats_by_name = {f.name: f for f in self.formats.values()}

    # ------------------------------------------------------------------ parse

    def _parse(self, progress=False):
        with open(self.path, "rb") as fh:
            data = fh.read()
        diag = self.diagnostics
        n = len(data)
        if n == 0:
            diag.add("EMPTY_FILE", "error", "file is empty")
            return
        from .textlog import looks_like_text_log, parse_text
        if looks_like_text_log(data[:8000]):
            parse_text(self, data)
            return
        if n < 3 + FMT_BODY_LEN:
            diag.add("FILE_TOO_SHORT", "error",
                     f"{n} bytes is shorter than one FMT record ({3 + FMT_BODY_LEN} bytes)")

        i = 0
        msgs = defaultdict(list)
        formats = self.formats
        unparseable = {}          # type id -> name, for FMTs we could not use
        nonascii = defaultdict(int)
        seen_fmt = False
        tail_reason = None

        def resync(at, reason, subject=None):
            """Scan forward for the next header. Returns the new offset or -1 at EOF."""
            nxt = data.find(HEADER, at + 1)
            if nxt < 0:
                return -1
            skipped = nxt - at
            if at == 0:
                diag.add("LEADING_GARBAGE", "warning",
                         f"{skipped} bytes before the first packet header", offset=0,
                         bytes=skipped)
            else:
                self.resync_bytes += skipped
                self.resync_events += 1
                diag.add("RESYNC", "error",
                         "bytes skipped mid-file to find the next packet header "
                         f"({reason})", subject=subject, offset=at, bytes=skipped)
            return nxt

        while i + 3 <= n:
            if data[i] != HEAD1 or data[i + 1] != HEAD2:
                nxt = resync(i, "not on a packet boundary")
                if nxt < 0:
                    tail_reason = ("no packet header in the remaining bytes", None)
                    break
                i = nxt
                continue

            mtype = data[i + 2]

            if mtype == FMT_TYPE:
                if i + 3 + FMT_BODY_LEN > n:
                    tail_reason = ("FMT", "FMT")
                    break
                body = data[i + 3: i + 3 + FMT_BODY_LEN]
                t, length, name, fmt, cols = FMT_STRUCT.unpack(body)
                name = _cstr(name)
                fmt = _cstr(fmt)
                cols = [c for c in _cstr(cols).split(",") if c]
                seen_fmt = True
                if not name or not fmt:
                    diag.add("FMT_EMPTY", "error",
                             f"FMT for type {t} has an empty name or format string; "
                             "messages of that type cannot be decoded", subject=t, offset=i)
                    unparseable[t] = name or f"type{t}"
                    i += 3 + FMT_BODY_LEN
                    continue
                if t == FMT_TYPE and fmt != FMT_SELF_FORMAT:
                    diag.add("FMT_SELF_UNEXPECTED", "warning",
                             f"FMT's own definition is {fmt!r}, expected {FMT_SELF_FORMAT!r}",
                             offset=i)
                try:
                    mf = MessageFormat(t, length, name, fmt, cols)
                except ValueError as exc:
                    diag.add("FMT_UNKNOWN_FORMAT_CHAR", "error",
                             f"{exc}; every {name} message in this log is undecodable",
                             subject=name, offset=i)
                    unparseable[t] = name
                    i += 3 + FMT_BODY_LEN
                    continue
                if len(cols) != len(fmt):
                    diag.add("FMT_COLUMN_COUNT_MISMATCH", "error",
                             f"{name}: {len(cols)} column names for {len(fmt)} format chars; "
                             "decoding by format and padding names as F<n>",
                             subject=name, offset=i, columns=len(cols), format_len=len(fmt))
                    while len(mf.columns) < len(fmt):
                        mf.columns.append(f"F{len(mf.columns)}")
                    mf.columns = mf.columns[:len(fmt)]
                if length != mf.body_len + 3:
                    diag.add("FMT_LENGTH_MISMATCH", "error",
                             f"{name}: FMT.Length={length} but format {fmt!r} occupies "
                             f"{mf.body_len + 3} bytes with header; decoding by format",
                             subject=name, offset=i, declared=length, computed=mf.body_len + 3)
                prev = formats.get(t)
                if prev is not None:
                    if prev.same_definition(mf):
                        diag.add("FMT_DUPLICATE", "info",
                                 f"{name}: FMT for type {t} re-sent with an identical definition",
                                 subject=name, offset=i)
                    else:
                        diag.add("FMT_REDEFINED", "warning",
                                 f"type {t} redefined from {prev.name}/{prev.format!r} to "
                                 f"{name}/{fmt!r}; earlier messages keep the old definition",
                                 subject=t, offset=i)
                other = self.formats_by_name.get(name)
                if other is not None and other.type != t:
                    diag.add("FMT_NAME_COLLISION", "warning",
                             f"{name} is defined as both type {other.type} and type {t}",
                             subject=name, offset=i)
                    if other.columns != mf.columns:
                        mf.name = f"{name}@{t}"
                formats[t] = mf
                self.formats_by_name[mf.name] = mf
                unparseable.pop(t, None)
                msgs["FMT"].append(dict(zip(
                    ["Type", "Length", "Name", "Format", "Columns"],
                    [t, length, name, fmt, ",".join(cols)])))
                i += 3 + FMT_BODY_LEN
                continue

            mf = formats.get(mtype)
            if mf is None:
                if mtype in unparseable:
                    diag.add("UNPARSEABLE_TYPE", "error",
                             f"message of type {mtype} ({unparseable[mtype]}) whose FMT "
                             "could not be used", subject=unparseable[mtype], offset=i)
                    subj = unparseable[mtype]
                else:
                    diag.add("UNKNOWN_MSG_TYPE", "error",
                             f"message type {mtype} has no FMT definition; its length is "
                             "unknown so the bytes up to the next header were skipped",
                             subject=mtype, offset=i)
                    subj = mtype
                nxt = resync(i, f"unknown type {mtype}", subject=subj)
                if nxt < 0:
                    tail_reason = ("no packet header after an unknown message type", None)
                    break
                i = nxt
                continue

            end = i + 3 + mf.body_len
            if end > n:
                tail_reason = ("message", mf.name)
                break
            vals = mf.struct.unpack(data[i + 3:end])

            row = {}
            for name_, raw, kind, scale in zip(mf.columns, vals, mf.kinds, mf.scales):
                if kind == "str":
                    s = raw.split(b"\x00", 1)[0]
                    try:
                        row[name_] = s.decode("ascii").strip()
                    except UnicodeDecodeError:
                        nonascii[mf.name] += 1
                        row[name_] = s.decode("ascii", "replace").strip()
                elif kind == "arr":
                    row[name_] = np.frombuffer(raw, dtype="<i2").copy()
                elif kind == "bytes":
                    row[name_] = raw
                elif scale is not None:
                    row[name_] = raw * scale
                else:
                    row[name_] = raw
            msgs[mf.name].append(row)
            i = end

        self.bytes_parsed = i
        if tail_reason is not None:
            self._classify_tail(data, i, tail_reason)
        elif i < n:
            # fewer than 3 bytes left: cannot even hold a header
            diag.add("TRAILING_BYTES", "info", f"{n - i} stray byte(s) after the last message",
                     offset=i, bytes=n - i)

        if not seen_fmt:
            diag.add("NO_FMT", "error",
                     "no FMT record found; this is not a DataFlash binary log (a .log text "
                     "export or a .tlog needs a different reader)")
        for name, cnt in nonascii.items():
            diag.add("STRING_NON_ASCII", "info",
                     f"{cnt} {name} string field(s) contained non-ASCII bytes "
                     "(replaced with U+FFFD)", subject=name, count_fields=cnt)

        self.messages = dict(msgs)
        self.n_messages = sum(len(v) for v in msgs.values())
        self._attach_units()
        self._post_parse_checks()

    def _classify_tail(self, data, i, reason):
        """Decide what the undecodable tail of the file is, and say so."""
        n = len(data)
        rest = data[i:]
        kind, subject = reason
        if not rest:
            return
        if rest.count(b"\xff") == len(rest):
            self.diagnostics.add("TRAILING_PADDING", "info",
                                 f"{len(rest)} bytes of 0xFF after the last message (erased "
                                 "flash - normal for a log read straight off a block-flash chip)",
                                 offset=i, bytes=len(rest))
            return
        if rest.count(b"\x00") == len(rest):
            self.diagnostics.add("TRAILING_PADDING", "info",
                                 f"{len(rest)} zero bytes after the last message",
                                 offset=i, bytes=len(rest))
            return
        if kind in ("message", "FMT"):
            need = (3 + FMT_BODY_LEN) if subject == "FMT" else 3 + self.formats_by_name[subject].body_len
            self.diagnostics.add("TRUNCATED_TAIL", "warning",
                                 f"file ends {need - len(rest)} bytes into a {subject} message "
                                 f"({len(rest)} of {need} bytes present) - the log was cut "
                                 "off, typically by power loss or a full card", offset=i,
                                 bytes=len(rest), msg_type=subject)
        else:
            self.diagnostics.add("TRAILING_GARBAGE", "warning",
                                 f"{len(rest)} undecodable bytes at end of file ({kind})",
                                 offset=i, bytes=len(rest))

    def _attach_units(self):
        """Attach FMTU/UNIT/MULT metadata to formats, as labels only.

        IMPORTANT: multipliers from MULT are *not* applied to values. Like pymavlink's
        DFReader, this parser applies only format-char scaling (c/C/e/E/L). MULT/UNIT are
        display metadata; applying them as well would double-scale.
        """
        diag = self.diagnostics
        units = {u["Id"]: u["Label"] for u in self.messages.get("UNIT", [])
                 if "Id" in u and "Label" in u}
        mults = {m["Id"]: m["Mult"] for m in self.messages.get("MULT", [])
                 if "Id" in m and "Mult" in m}
        for row in self.messages.get("FMTU", []):
            t = row.get("FmtType")
            mf = self.formats.get(t)
            if mf is None:
                diag.add("FMTU_UNKNOWN_TYPE", "warning",
                         f"FMTU refers to type {t}, which has no FMT", subject=t)
                continue
            uids = row.get("UnitIds", "")
            mids = row.get("MultIds", "")
            if len(uids) != len(mf.format) or len(mids) != len(mf.format):
                diag.add("FMTU_LENGTH_MISMATCH", "warning",
                         f"{mf.name}: FMTU has {len(uids)} unit ids and {len(mids)} multiplier "
                         f"ids for {len(mf.format)} fields", subject=mf.name)
            mf.units = [units.get(ord(c), None) for c in uids]
            mf.mults = [mults.get(ord(c), None) for c in mids]
            missing_u = sorted({c for c in uids if ord(c) not in units})
            missing_m = sorted({c for c in mids if ord(c) not in mults})
            for c in missing_u:
                diag.add("UNIT_ID_UNKNOWN", "info",
                         f"unit id {c!r} is used by FMTU but defined by no UNIT record "
                         f"(first seen on {mf.name})", subject=c, messages=1)
            for c in missing_m:
                diag.add("MULT_ID_UNKNOWN", "info",
                         f"multiplier id {c!r} is used by FMTU but defined by no MULT record "
                         f"(first seen on {mf.name})", subject=c, messages=1)

    def _post_parse_checks(self):
        diag = self.diagnostics
        if not self.messages.get("PARM"):
            diag.add("NO_PARM", "warning",
                     "no PARM records: the parameter set that flew is unknown, so every "
                     "check that reads a parameter falls back to a default and says so")
        if self.n_messages == 0:
            diag.add("NO_MESSAGES", "error", "no messages decoded")
        # Timestamp order, per message. Backwards steps in the metadata messages are
        # an artefact of when the logger emits them; anywhere else they mean something.
        for name, rows in self.messages.items():
            if len(rows) < 2 or "TimeUS" not in rows[0]:
                continue
            t = np.fromiter((r["TimeUS"] for r in rows), dtype="int64", count=len(rows))
            d = np.diff(t)
            back = int((d < 0).sum())
            if back:
                sev = "info" if name in _METADATA_MSGS else "warning"
                diag.add("TIME_NON_MONOTONIC", sev,
                         f"{name}: TimeUS runs backwards {back} time(s) "
                         f"(largest step {int(d.min())} us)", subject=name,
                         backwards_steps=back, largest_step_us=int(d.min()))

    # ------------------------------------------------------------------ quality

    def quality(self):
        """Data-quality diagnostics that need the decoded values: NaN/Inf counts, logging
        gaps, and rate estimates. Computed once, on demand. Returns a Diagnostics."""
        if self._quality is not None:
            return self._quality
        q = Diagnostics()
        gap_reports = []
        for name, rows in self.messages.items():
            if not rows:
                continue
            mf = self.formats_by_name.get(name)
            if mf is None:
                continue
            d = self.df(name)
            if d.empty:
                continue
            # NaN / Inf in float fields
            for col, kind in zip(mf.columns, mf.kinds):
                if kind != "float" or col not in d.columns:
                    continue
                v = d[col].values
                if v.dtype.kind != "f":
                    continue
                bad = int((~np.isfinite(v)).sum())
                if bad:
                    allowed = (name, col) in NAN_ALLOWED
                    q.add("NAN_VALUES", "info" if allowed else "warning",
                          f"{name}.{col}: {bad} of {len(v)} values are NaN or Inf"
                          + (" (expected for this field on some boards)" if allowed else ""),
                          subject=f"{name}.{col}", count_values=bad, of=len(v))
            # Logging gaps in streaming messages
            if "t" in d.columns and len(d) >= 50 and name not in _METADATA_MSGS:
                t = d["t"].values
                dt = np.diff(t)
                med = float(np.median(dt))
                # Only a message logged at a steady rate can show a *gap*; event-driven
                # messages (LDET, MODE, EV...) are irregular by nature.
                steady = med > 0 and med < 2.0 and float(np.percentile(dt, 95)) < 3 * med
                if steady:
                    worst = float(dt.max())
                    if worst > max(10 * med, 1.0):
                        at = float(t[int(np.argmax(dt))])
                        gap_reports.append((worst, name, med, at))
        # Duplicate data chunks (LogAnalyzer TestDupeLogData): the same run of 20 ATT.Pitch
        # values appearing twice elsewhere means the storage medium replayed a block.
        att = self.df("ATT")
        if not att.empty and "Pitch" in att.columns and len(att) > 60:
            v = np.ascontiguousarray(att["Pitch"].values, dtype="float64")
            win = 20
            # Every 20-sample window, hashed: a repeat at a non-overlapping offset is a
            # replayed block. O(n) rather than LogAnalyzer's sparse sampling, which can
            # miss a duplicate that falls between its probe points.
            from numpy.lib.stride_tricks import sliding_window_view
            windows = sliding_window_view(v, win)
            flat = (np.ptp(windows, axis=1) > 0)
            seen = {}
            for j in np.flatnonzero(flat):
                key = windows[j].tobytes()
                first = seen.get(key)
                if first is None:
                    seen[key] = int(j)
                elif j - first >= win:
                    q.add("DUPLICATE_DATA", "error",
                          f"the 20 ATT.Pitch values at rows {first}-{first + win - 1} recur at rows "
                          f"{int(j)}-{int(j) + win - 1} - flash corruption or a replayed block",
                          subject="ATT.Pitch", first_row=first, second_row=int(j))
                    break
        if gap_reports:
            gap_reports.sort(reverse=True)
            worst, name, med, at = gap_reports[0]
            q.add("LOG_GAP", "warning",
                  f"largest logging gap {worst:.2f} s in {name} (normal interval {med * 1e3:.1f} ms) "
                  f"at t={at:.1f} s; {len(gap_reports)} message type(s) show a gap >10x their "
                  "interval - the logger stalled or dropped data there",
                  subject=name, gap_s=worst, at_s=at, n_types=len(gap_reports),
                  types=[g[1] for g in gap_reports[:12]])
        self._quality = q
        return q

    # ------------------------------------------------------------------- access

    def types(self, min_count=1):
        """{message name: record count}, most numerous first."""
        return dict(sorted(((k, len(v)) for k, v in self.messages.items() if len(v) >= min_count),
                           key=lambda kv: -kv[1]))

    def declared_types(self):
        """Every message name the log's FMT records define, present or not."""
        return sorted(self.formats_by_name)

    def has(self, name):
        return name in self.messages and len(self.messages[name]) > 0

    def raw(self, name):
        return self.messages.get(name, [])

    def df(self, name, copy=False):
        """Message as a pandas DataFrame, with a `t` column of seconds since boot.

        Returns an empty DataFrame if the message is absent, so callers can
        branch on `.empty` rather than KeyError.
        """
        if pd is None:
            raise RuntimeError("pandas is required for Log.df()")
        if name in self._df_cache and not copy:
            return self._df_cache[name]
        rows = self.messages.get(name)
        if not rows:
            out = pd.DataFrame()
        else:
            out = pd.DataFrame(rows)
            if "TimeUS" in out.columns:
                out["t"] = out["TimeUS"].astype("float64") / 1e6
        self._df_cache[name] = out
        return out.copy() if copy else out

    def field(self, msg, *aliases, default=None):
        """First present column among `aliases`, as a numpy array.

        ArduPilot renames log fields between versions (BarAlt -> BAlt,
        ThrOut -> ThO, CRate -> CRt, Chan1 -> C1). Hardcoding one spelling is
        the single most common way an analysis script silently breaks on an
        older or newer log, so always go through this. Pass the modern name
        first. Returns `default` (None) if none of them exist.
        """
        d = self.df(msg)
        if d.empty:
            return default
        for a in aliases:
            if a in d.columns:
                return d[a].values
        return default

    def first_field_name(self, msg, *aliases):
        d = self.df(msg)
        for a in aliases:
            if not d.empty and a in d.columns:
                return a
        return None

    def columns(self, name):
        mf = self.formats_by_name.get(name)
        return list(mf.columns) if mf else []

    def units(self, name):
        """{field: unit label} from FMTU/UNIT, or {} if the log carries none."""
        mf = self.formats_by_name.get(name)
        if mf is None or not mf.units:
            return {}
        return {c: (u or "") for c, u in zip(mf.columns, mf.units)}

    def multipliers(self, name):
        """{field: MULT value} - display metadata, NOT applied to the values."""
        mf = self.formats_by_name.get(name)
        if mf is None or not mf.mults:
            return {}
        return {c: m for c, m in zip(mf.columns, mf.mults)}

    def rate_hz(self, name, steady_only=False):
        """Median logging rate of a message in Hz, or None.

        With `steady_only`, returns None for messages that are not logged at a steady
        rate (PARM bursts at boot, MODE/EV/ERR on events), where a "rate" would mislead.
        """
        d = self.df(name)
        if d.empty or "t" not in d.columns or len(d) < 3:
            return None
        inst = self.instance_field(name)
        if inst is not None:
            # interleaved instances would halve/quarter the apparent interval
            d = max((g for _, g in d.groupby(inst)), key=len)
            if len(d) < 3:
                return None
        dt = np.diff(d["t"].values)
        med = float(np.median(dt))
        if med <= 0:
            return None
        if steady_only and (name in _METADATA_MSGS or len(d) < 20
                            or float(np.percentile(dt, 95)) > 3 * med):
            return None
        return float(1.0 / med)

    def instance_field(self, name):
        """Name of the instance/core column for a message, or None.

        The writer marks the instance column with unit id `#` in FMTU, which is
        authoritative when present. Logs older than 3.6 have no FMTU, so a label
        heuristic (Instance, Inst, IMU, C, I, ...) is the fallback.
        """
        mf = self.formats_by_name.get(name)
        if mf is None:
            return None
        d = self.df(name)
        if d.empty:
            return None
        if mf.units:
            for col, u in zip(mf.columns, mf.units):
                if u == "instance" and col in d.columns:
                    return col
        for cand in INSTANCE_FIELD_CANDIDATES:
            if cand in mf.columns:
                vals = d[cand]
                # An instance column is a small set of small non-negative ints.
                try:
                    uniq = vals.unique()
                except Exception:
                    continue
                if len(uniq) <= 16 and np.all(np.asarray(uniq) >= 0) and np.all(np.asarray(uniq) < 32):
                    return cand
        return None

    def instances(self, name, field=None):
        """{instance index: DataFrame}. Empty dict if the message is absent.

        ArduPilot names the instance column differently per message
        (`Instance` on ESC, `I` on MAG/BAT, `C` on XKF*, `IMU` on VIBE), so
        this auto-detects unless you pass `field`.
        """
        d = self.df(name)
        if d.empty:
            return {}
        field = field or self.instance_field(name)
        if field is None:
            return {0: d}
        return {int(k): g.reset_index(drop=True) for k, g in d.groupby(field)}

    def messages_text(self):
        """MSG records as (t, text) - the FC's own commentary on the flight.

        ArduPilot 4.7+ writes long texts as 64-byte chunks with an `Id` and a chunk
        sequence number; those are reassembled here so a message never appears split.
        """
        d = self.df("MSG")
        if d.empty:
            return []
        col = "Message" if "Message" in d.columns else d.columns[-1]
        seq_col = next((c for c in ("Seq", "ChunkSeq", "Chunk", "S") if c in d.columns), None)
        id_col = "Id" if "Id" in d.columns else ("ID" if "ID" in d.columns else None)
        t = d["t"].values if "t" in d.columns else d.index.values
        if seq_col is None or id_col is None:
            return list(zip(t, d[col]))
        out, open_msgs = [], {}
        for ti, mid, seq, text in zip(t, d[id_col].values, d[seq_col].values, d[col].values):
            if int(seq) == 0 or mid not in open_msgs:
                open_msgs[mid] = [ti, str(text)]
                out.append(open_msgs[mid])
            else:
                open_msgs[mid][1] += str(text)
        return [(float(a), b) for a, b in out]

    def files(self):
        """Embedded files (FILE records) reassembled by (name, offset): {name: bytes}.

        Honours each chunk's Length and Offset, so binary files (storage.bin,
        crash_dump.bin) survive and a retried duplicate chunk is written once.
        """
        rows = self.raw("FILE")
        if not rows:
            return {}
        chunks = {}
        for r in rows:
            name = r.get("FileName", "")
            data = r.get("Data", b"")
            if isinstance(data, str):
                data = data.encode("latin-1", "replace")
            length = int(r.get("Length", len(data)))
            chunks.setdefault(name, {})[int(r.get("Offset", 0))] = data[:length]
        out = {}
        for name, parts in chunks.items():
            buf = bytearray()
            for off in sorted(parts):
                if off > len(buf):
                    buf.extend(b"\x00" * (off - len(buf)))
                buf[off:off + len(parts[off])] = parts[off]
            out[name] = bytes(buf)
        return out

    def param_at(self, name, t, default=None):
        """Value of a parameter in force at time t (last PARM write at or before t)."""
        d = self.df("PARM")
        if d.empty:
            return default
        sel = d[(d["Name"] == name) & (d["t"] <= t)]
        if sel.empty:
            sel = d[d["Name"] == name]
            return float(sel["Value"].iloc[0]) if not sel.empty else default
        return float(sel["Value"].iloc[-1])

    def params(self):
        """{name: value} from PARM records (last value wins)."""
        d = self.df("PARM")
        if d.empty:
            return {}
        return dict(zip(d["Name"], d["Value"]))

    def param_defaults(self):
        """{name: default} from PARM.Default (ArduPilot 4.3+), else {}."""
        d = self.df("PARM")
        if d.empty or "Default" not in d.columns:
            return {}
        # A parameter re-emitted after an in-flight change carries Default=NaN ("unknown"),
        # so keep the first finite default rather than the last value.
        out = {}
        for name, dflt in zip(d["Name"].values, d["Default"].values):
            if name not in out and dflt == dflt:
                out[name] = float(dflt)
        return out

    def param_changes(self):
        """[(t, name, old, new)] for parameters written more than once in the log,
        i.e. changed in flight or by the GCS after boot."""
        d = self.df("PARM")
        if d.empty:
            return []
        out = []
        last = {}
        for t, name, val in zip(d["t"], d["Name"], d["Value"]):
            if name in last and last[name] != val:
                out.append((float(t), name, last[name], val))
            last[name] = val
        return out

    def param(self, name, default=None):
        return self.params().get(name, default)

    def duration(self):
        """(t_first, t_last) in seconds since boot, over all timestamped messages."""
        lo, hi = None, None
        for name in self.messages:
            d = self.df(name)
            if d.empty or "t" not in d.columns:
                continue
            a, b = float(d["t"].min()), float(d["t"].max())
            lo = a if lo is None else min(lo, a)
            hi = b if hi is None else max(hi, b)
        return (lo, hi)

    # ------------------------------------------------------------------- identity

    def firmware(self):
        """Firmware string, from VER.FWS or the MSG banner. '' if neither exists."""
        for r in self.raw("VER"):
            if r.get("FWS"):
                return str(r["FWS"])
        for _, m in self.messages_text():
            if re.match(r"^(ArduCopter|ArduPlane|ArduRover|Rover|ArduSub|AntennaTracker|Blimp|"
                        r"ArduHeli)\b", str(m)):
                return str(m)
        return ""

    def vehicle(self):
        fw = self.firmware()
        m = re.match(r"^(ArduCopter|ArduPlane|ArduRover|Rover|ArduSub|AntennaTracker|Blimp)", fw)
        if m:
            return m.group(1)
        v = self.raw("VER")
        if v and "BT" in v[0]:
            return {2: "ArduCopter", 1: "ArduPlane", 10: "ArduRover", 12: "ArduSub",
                    20: "AntennaTracker", 29: "Blimp"}.get(int(v[0]["BT"]), f"VER.BT={v[0]['BT']}")
        return ""

    def board(self):
        """Board name from the MSG banner, e.g. 'TMotorH743'. '' if not logged."""
        # The banner line is "<board> <serial words in hex>", e.g.
        # "TMotorH743 00330030 30315112 32323838".
        for _, m in self.messages_text():
            hit = re.match(r"^([A-Za-z0-9_\-]+)\s+[0-9A-Fa-f]{8}\s+[0-9A-Fa-f]{8}\b", str(m))
            if hit:
                return hit.group(1)
        return ""

    def boot_time_unix(self):
        """Unix epoch of TimeUS=0, from the first GPS record with a valid week. None if
        the log never got GPS time - then the log date is the FC's unset RTC (1980)."""
        d = self.df("GPS")
        if d.empty or not {"GWk", "GMS"} <= set(d.columns):
            return None
        ok = d[(d["GWk"] > 0)]
        if ok.empty:
            return None
        r = ok.iloc[0]
        return gps_to_unix(r["GWk"], r["GMS"]) - float(r["TimeUS"]) / 1e6

    def sha256(self):
        h = hashlib.sha256()
        with open(self.path, "rb") as fh:
            for chunk in iter(lambda: fh.read(1 << 20), b""):
                h.update(chunk)
        return h.hexdigest()

    def info(self):
        """Identity and shape of the log in one dict - the first thing to look at."""
        lo, hi = self.duration()
        boot = self.boot_time_unix()
        import datetime as _dt
        def iso(ts):
            return _dt.datetime.fromtimestamp(ts, _dt.timezone.utc).isoformat() if ts else None
        p = self.params()
        return dict(
            path=self.path, file_name=os.path.basename(self.path),
            file_size=self.file_size, sha256=self.sha256(), source_format=self.source_format,
            firmware=self.firmware(), vehicle=self.vehicle(), board=self.board(),
            n_messages=self.n_messages, n_types_present=len(self.messages),
            n_types_declared=len(self.formats_by_name),
            t_first=lo, t_last=hi, duration_s=(hi - lo) if lo is not None else None,
            boot_time_utc=iso(boot),
            log_start_utc=iso(boot + lo) if boot and lo is not None else None,
            gps_time_available=boot is not None,
            n_params=len(p),
            frame_class=p.get("FRAME_CLASS"), frame_type=p.get("FRAME_TYPE"),
            integrity=self.diagnostics.to_dict(),
        )

    def __repr__(self):
        lo, hi = self.duration()
        span = f"{hi - lo:.0f}s" if lo is not None else "?"
        return (f"<Log {os.path.basename(self.path)} "
                f"{self.n_messages} msgs, {len(self.messages)} types, {span}, "
                f"integrity={self.diagnostics.worst}>")
