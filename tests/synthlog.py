"""A tiny DataFlash *writer*, so the parser can be tested against logs whose every byte
is known - including deliberately broken ones.

Nothing here is used by the toolkit itself. It exists because the only way to prove that
a parser fails loudly on truncation, garbage, unknown types and malformed FMT records is
to feed it exactly those, and real flight logs never contain them on demand.

    from synthlog import LogWriter
    w = LogWriter()
    w.fmt(200, "ATT", "QccC", "TimeUS,Roll,Pitch,Yaw")
    w.msg("ATT", TimeUS=1000, Roll=1.5, Pitch=-2.0, Yaw=180.0)
    blob = w.bytes()

Encoding follows dflog.parser.FORMAT_CHARS exactly: c/C/e/E are multiplied by 100 and
stored as integers, L by 1e7. Strings are NUL-padded to their fixed width.
"""

from __future__ import annotations

import struct

HEAD = b"\xa3\x95"
FMT_TYPE = 0x80

_PACK = {
    "a": "64s", "b": "b", "B": "B", "h": "h", "H": "H", "i": "i", "I": "I",
    "f": "f", "d": "d", "g": "e", "n": "4s", "N": "16s", "Z": "64s",
    "c": "h", "C": "H", "e": "i", "E": "I", "L": "i", "M": "B", "q": "q", "Q": "Q",
}
_SCALE = {"c": 100, "C": 100, "e": 100, "E": 100, "L": 1e7}
_WIDTH = {"n": 4, "N": 16, "Z": 64}


def _encode(ch, v):
    if ch in _WIDTH:
        if isinstance(v, str):
            v = v.encode("ascii")
        return v[:_WIDTH[ch]].ljust(_WIDTH[ch], b"\x00")
    if ch == "a":
        return struct.pack("<32h", *list(v)[:32] + [0] * (32 - len(v)))
    if ch in _SCALE:
        return int(round(v * _SCALE[ch]))
    if ch in "fdg":
        return float(v)
    return int(v)


class LogWriter:
    def __init__(self, with_fmt_of_fmt=True):
        self.buf = bytearray()
        self.formats = {}      # name -> (type, fmt, cols)
        if with_fmt_of_fmt:
            self.fmt(FMT_TYPE, "FMT", "BBnNZ", "Type,Length,Name,Format,Columns")

    # ------------------------------------------------------------------ records

    def fmt(self, type_id, name, fmt, cols, length=None, register=True):
        """Emit an FMT record. `length` defaults to the correct packet length."""
        # Unknown format chars are allowed here on purpose, so the parser's handling of
        # a bad FMT can be tested; they contribute no bytes to the computed length.
        body_len = struct.calcsize("<" + "".join(_PACK[c] for c in fmt if c in _PACK))
        if length is None:
            length = 3 + body_len
        body = struct.pack("<BB4s16s64s", type_id, length,
                           name.encode()[:4].ljust(4, b"\x00"),
                           fmt.encode()[:16].ljust(16, b"\x00"),
                           cols.encode()[:64].ljust(64, b"\x00"))
        self.buf += HEAD + bytes([FMT_TYPE]) + body
        if register:
            self.formats[name] = (type_id, fmt, [c for c in cols.split(",") if c])
        return self

    def msg(self, name, **fields):
        type_id, fmt, cols = self.formats[name]
        vals = []
        for i, ch in enumerate(fmt):          # by format, so a short column list still packs
            col = cols[i] if i < len(cols) else None
            vals.append(_encode(ch, fields.get(col, 0)))
        body = struct.pack("<" + "".join(_PACK[c] for c in fmt), *vals)
        self.buf += HEAD + bytes([type_id]) + body
        return self

    def raw(self, blob: bytes):
        """Append arbitrary bytes - garbage, padding, a partial packet."""
        self.buf += blob
        return self

    def bytes(self):
        return bytes(self.buf)

    def write(self, path):
        with open(path, "wb") as fh:
            fh.write(self.buf)
        return path


def standard_log(n=50, dt_us=20000, with_units=True):
    """A small but complete-looking log: FMT, FMTU/UNIT/MULT, PARM, MSG, ATT, and
    a two-instance message, so the ordinary access paths are all exercised."""
    w = LogWriter()
    w.fmt(116, "FMTU", "QBNN", "TimeUS,FmtType,UnitIds,MultIds")
    w.fmt(117, "UNIT", "QbZ", "TimeUS,Id,Label")
    w.fmt(118, "MULT", "Qbd", "TimeUS,Id,Mult")
    w.fmt(96, "PARM", "QNff", "TimeUS,Name,Value,Default")
    w.fmt(97, "MSG", "QZ", "TimeUS,Message")
    w.fmt(200, "ATT", "QccC", "TimeUS,Roll,Pitch,Yaw")
    w.fmt(201, "BAT", "QBff", "TimeUS,Inst,Volt,Curr")
    if with_units:
        w.msg("UNIT", TimeUS=10, Id=ord("s"), Label="s")
        w.msg("UNIT", TimeUS=11, Id=ord("d"), Label="deg")
        w.msg("UNIT", TimeUS=12, Id=ord("-"), Label="")
        w.msg("MULT", TimeUS=13, Id=ord("F"), Mult=1e-6)
        w.msg("MULT", TimeUS=14, Id=ord("-"), Mult=0.0)
        w.msg("FMTU", TimeUS=15, FmtType=200, UnitIds="sddd", MultIds="F---")
    w.msg("MSG", TimeUS=20, Message="ArduCopter V4.7.1 (deadbeef)")
    w.msg("PARM", TimeUS=30, Name="FRAME_CLASS", Value=1.0, Default=1.0)
    w.msg("PARM", TimeUS=31, Name="FRAME_TYPE", Value=1.0, Default=1.0)
    for i in range(n):
        t = 1_000_000 + i * dt_us
        w.msg("ATT", TimeUS=t, Roll=0.5 * i, Pitch=-0.25 * i, Yaw=(i * 3) % 360)
        w.msg("BAT", TimeUS=t, Inst=0, Volt=16.0 - i * 0.01, Curr=5.0)
        w.msg("BAT", TimeUS=t, Inst=1, Volt=8.0, Curr=1.0)
    return w
