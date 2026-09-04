"""
ArduPilot DataFlash (.bin) log reader — pure stdlib + numpy.

Why this exists: pymavlink cannot be installed in the Claude cloud sandbox
(PyPI is blocked, `pip install pymavlink` fails with host_not_allowed).
This module parses the self-describing DataFlash format directly.

Typical use:

    from dflog import Log
    log = Log("flight.bin")                  # parses, then caches to .dfcache
    att  = log.df("ATT")                     # pandas DataFrame
    esc  = log.instances("ESC")              # {0: df, 1: df, 2: df, 3: df}
    print(log.types())                       # what's in this log

Re-parsing a 16 MB log costs ~10-20 s. The first parse writes a pickle cache
next to the log (`<name>.dfcache`); later runs load in well under a second.
Delete the cache or pass `use_cache=False` to force a re-parse.
"""

from __future__ import annotations

import os
import pickle
import struct
import sys
from collections import defaultdict

import numpy as np

try:
    import pandas as pd
except ImportError:  # pragma: no cover - pandas is present in the sandbox
    pd = None

__all__ = ["Log", "FORMAT_CHARS", "CACHE_VERSION"]

# Bump when the parser's output structure changes, so stale caches are ignored.
CACHE_VERSION = 3

HEAD1 = 0xA3
HEAD2 = 0x95
FMT_TYPE = 0x80          # 128
FMT_BODY_LEN = 86        # NOT 89 - the 3-byte packet header is not part of the body
FMT_STRUCT = struct.Struct("<BB4s16s64s")

# DataFlash format characters.
#   size  = bytes on the wire
#   pack  = struct code (None => handled specially)
#   scale = multiplier applied on decode (format-char scaling only)
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

# Field names ArduPilot uses for the instance/core index, most specific first.
INSTANCE_FIELD_CANDIDATES = ("Instance", "Inst", "IMU", "Core", "C", "I", "Id", "N")


def _cstr(raw: bytes) -> str:
    return raw.split(b"\x00", 1)[0].decode("ascii", "replace").strip()


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
        for ch in fmt:
            spec = FORMAT_CHARS.get(ch)
            if spec is None:
                raise ValueError(f"unknown format char {ch!r} in {name} ({fmt!r})")
            pack += spec["pack"]
            scales.append(spec["scale"])
            if ch in _STRING_CHARS:
                kinds.append("str")
            elif ch in _ARRAY_CHARS:
                kinds.append("arr")
            else:
                kinds.append("num")
            size += spec["size"]

        self.struct = struct.Struct(pack)
        self.body_len = size
        self.scales = scales
        self.kinds = kinds

    # struct.Struct is not picklable, so the cache stores only the definition
    # and the derived fields are rebuilt on load.
    def __getstate__(self):
        return (self.type, self.length, self.name, self.format, self.columns,
                self.units, self.mults)

    def __setstate__(self, state):
        type_id, length, name, fmt, columns, units, mults = state
        self.__init__(type_id, length, name, fmt, columns)
        self.units, self.mults = units, mults

    def __repr__(self):
        return f"<FMT {self.name} type={self.type} fmt={self.format!r} cols={self.columns}>"


class Log:
    """A parsed ArduPilot dataflash log."""

    def __init__(self, path, use_cache=True, verbose=False, progress=False):
        self.path = str(path)
        self.verbose = verbose
        self.formats: dict[int, MessageFormat] = {}
        self.formats_by_name: dict[str, MessageFormat] = {}
        self.messages: dict[str, list] = {}
        self.resync_bytes = 0
        self.n_messages = 0
        self._df_cache: dict[str, "pd.DataFrame"] = {}

        cache = self._cache_path()
        if use_cache and os.path.exists(cache) and os.path.getmtime(cache) >= os.path.getmtime(self.path):
            try:
                self._load_cache(cache)
                if verbose:
                    print(f"[dflog] loaded cache {cache}", file=sys.stderr)
                return
            except Exception as exc:  # corrupt or stale cache: re-parse
                if verbose:
                    print(f"[dflog] cache unusable ({exc}); re-parsing", file=sys.stderr)

        self._parse(progress=progress)
        if use_cache:
            try:
                self._save_cache(cache)
            except OSError:
                pass

    # ------------------------------------------------------------------ cache

    def _cache_path(self):
        return self.path + ".dfcache"

    def _save_cache(self, cache):
        blob = dict(
            version=CACHE_VERSION,
            src_mtime=os.path.getmtime(self.path),
            formats=self.formats,
            messages=self.messages,
            resync_bytes=self.resync_bytes,
            n_messages=self.n_messages,
        )
        tmp = cache + ".tmp"
        with open(tmp, "wb") as fh:
            pickle.dump(blob, fh, protocol=pickle.HIGHEST_PROTOCOL)
        os.replace(tmp, cache)

    def _load_cache(self, cache):
        with open(cache, "rb") as fh:
            blob = pickle.load(fh)
        if blob.get("version") != CACHE_VERSION:
            raise ValueError("cache version mismatch")
        self.formats = blob["formats"]
        self.messages = blob["messages"]
        self.resync_bytes = blob["resync_bytes"]
        self.n_messages = blob["n_messages"]
        self.formats_by_name = {f.name: f for f in self.formats.values()}

    # ------------------------------------------------------------------ parse

    def _parse(self, progress=False):
        with open(self.path, "rb") as fh:
            data = fh.read()

        n = len(data)
        i = 0
        msgs = defaultdict(list)
        formats = self.formats
        resync = 0

        while i + 3 <= n:
            if data[i] != HEAD1 or data[i + 1] != HEAD2:
                # Not on a packet boundary. Scan forward for the next header.
                nxt = data.find(b"\xa3\x95", i + 1)
                if nxt < 0:
                    resync += n - i
                    break
                resync += nxt - i
                i = nxt
                continue

            mtype = data[i + 2]

            if mtype == FMT_TYPE:
                if i + 3 + FMT_BODY_LEN > n:
                    break
                body = data[i + 3: i + 3 + FMT_BODY_LEN]
                t, length, name, fmt, cols = FMT_STRUCT.unpack(body)
                name = _cstr(name)
                fmt = _cstr(fmt)
                cols = [c for c in _cstr(cols).split(",") if c]
                try:
                    mf = MessageFormat(t, length, name, fmt, cols)
                except ValueError as exc:
                    if self.verbose:
                        print(f"[dflog] skipping {name}: {exc}", file=sys.stderr)
                    i += 3 + FMT_BODY_LEN
                    continue
                if len(cols) != len(fmt):
                    if self.verbose:
                        print(f"[dflog] {name}: {len(cols)} columns vs {len(fmt)} format chars"
                              f" - decoding by format", file=sys.stderr)
                    # keep going; extra/missing names are padded below
                    while len(mf.columns) < len(fmt):
                        mf.columns.append(f"F{len(mf.columns)}")
                    mf.columns = mf.columns[:len(fmt)]
                formats[t] = mf
                self.formats_by_name[name] = mf
                msgs["FMT"].append(dict(zip(
                    ["Type", "Length", "Name", "Format", "Columns"],
                    [t, length, name, fmt, ",".join(cols)])))
                i += 3 + FMT_BODY_LEN
                continue

            mf = formats.get(mtype)
            if mf is None:
                # Unknown type: we cannot know its length, so resync.
                nxt = data.find(b"\xa3\x95", i + 1)
                if nxt < 0:
                    resync += n - i
                    break
                resync += nxt - i
                i = nxt
                continue

            end = i + 3 + mf.body_len
            if end > n:
                break
            try:
                vals = mf.struct.unpack(data[i + 3:end])
            except struct.error:
                nxt = data.find(b"\xa3\x95", i + 1)
                if nxt < 0:
                    break
                resync += nxt - i
                i = nxt
                continue

            row = {}
            for name_, raw, kind, scale in zip(mf.columns, vals, mf.kinds, mf.scales):
                if kind == "str":
                    row[name_] = _cstr(raw)
                elif kind == "arr":
                    row[name_] = np.frombuffer(raw, dtype="<i2").copy()
                elif scale is not None:
                    row[name_] = raw * scale
                else:
                    row[name_] = raw
            msgs[mf.name].append(row)
            i = end

        self.messages = dict(msgs)
        self.resync_bytes = resync
        self.n_messages = sum(len(v) for v in msgs.values())
        self._attach_units()

    def _attach_units(self):
        """Attach FMTU/UNIT/MULT metadata to formats, as labels only.

        IMPORTANT: multipliers from MULT are *not* applied to values. Like
        pymavlink's DFReader, this parser applies only format-char scaling
        (c/C/e/E/L). MULT/UNIT are display metadata; applying them as well
        would double-scale.
        """
        units = {u["Id"]: u["Label"] for u in self.messages.get("UNIT", [])
                 if "Id" in u and "Label" in u}
        mults = {m["Id"]: m["Mult"] for m in self.messages.get("MULT", [])
                 if "Id" in m and "Mult" in m}
        for row in self.messages.get("FMTU", []):
            mf = self.formats.get(row.get("FmtType"))
            if mf is None:
                continue
            uids = row.get("UnitIds", "")
            mids = row.get("MultIds", "")
            mf.units = [units.get(ord(c) if isinstance(c, str) else c, "") for c in uids]
            mf.mults = [mults.get(ord(c) if isinstance(c, str) else c, None) for c in mids]

    # ------------------------------------------------------------------- access

    def types(self, min_count=1):
        """{message name: record count}, most numerous first."""
        return dict(sorted(((k, len(v)) for k, v in self.messages.items() if len(v) >= min_count),
                           key=lambda kv: -kv[1]))

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

    def instance_field(self, name):
        """Name of the instance/core column for a message, or None."""
        mf = self.formats_by_name.get(name)
        if mf is None:
            return None
        d = self.df(name)
        if d.empty:
            return None
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
        """MSG records as (t, text) — the FC's own commentary on the flight."""
        d = self.df("MSG")
        if d.empty:
            return []
        col = "Message" if "Message" in d.columns else d.columns[-1]
        return list(zip(d.get("t", d.index), d[col]))

    def params(self):
        """{name: value} from PARM records (last value wins)."""
        d = self.df("PARM")
        if d.empty:
            return {}
        return dict(zip(d["Name"], d["Value"]))

    def param(self, name, default=None):
        return self.params().get(name, default)

    def duration(self):
        """(t_first, t_last) in seconds since boot, over all timestamped messages."""
        lo, hi = None, None
        for name in self.messages:
            d = self.df(name)
            if d.empty or "t" not in d.columns:
                continue
            a, b = float(d["t"].iloc[0]), float(d["t"].iloc[-1])
            lo = a if lo is None else min(lo, a)
            hi = b if hi is None else max(hi, b)
        return (lo, hi)

    def __repr__(self):
        lo, hi = self.duration()
        span = f"{hi - lo:.0f}s" if lo is not None else "?"
        return (f"<Log {os.path.basename(self.path)} "
                f"{self.n_messages} msgs, {len(self.messages)} types, {span}, "
                f"resync={self.resync_bytes}B>")
