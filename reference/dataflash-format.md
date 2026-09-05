# The ArduPilot DataFlash binary format

Enough detail to write or debug a parser from scratch. `dflog/parser.py` is the
implementation; this is the specification it was written from, plus the parts that are
easy to get wrong.

## Packet framing

Every message is:

```
0xA3 0x95      two-byte header
<type>         one byte, the message type id
<body>         length determined by that type's FMT
```

Header bytes are `HEAD1 = 0xA3`, `HEAD2 = 0x95`. There is no length field in the packet
itself — the body length comes entirely from the `FMT` record that defined the type. That
means an unknown type id cannot be skipped by length; the only recovery is to scan
forward for the next `0xA3 0x95`. Count those skipped bytes and report them: a healthy
log resyncs **zero** times, and a non-zero count is the first thing to mention when
numbers look strange.

## FMT — the self-describing part

Type id **128** is `FMT`. Its body is **86 bytes**, unpacked as:

```python
struct.Struct("<BB4s16s64s").unpack(body)   # -> (type, length, name, format, columns)
```

86, not 89 — the three-byte packet header is not part of the body. This off-by-three is
the single most common reason a hand-rolled parser produces garbage.

- `type` — the id subsequent messages of this kind will carry
- `length` — total packet length including the 3-byte header
- `name` — message name, NUL-padded (`ATT`, `RATE`, `ESC`, ...)
- `format` — one character per field, see below
- `columns` — comma-separated field names, NUL-padded

`FMT` defines itself first, so a parser can start cold. Each subsequent `FMT` is stored and
used to unpack every later message of that type.

Guard against `len(columns) != len(format)` — it happens on malformed or truncated
definitions. Decode by the format string (it determines the byte layout) and pad or trim
the names.

## Format characters

| char | struct | bytes | meaning | scaling applied |
|---|---|---|---|---|
| `b` | `b` | 1 | int8 | — |
| `B` | `B` | 1 | uint8 | — |
| `h` | `h` | 2 | int16 | — |
| `H` | `H` | 2 | uint16 | — |
| `i` | `i` | 4 | int32 | — |
| `I` | `I` | 4 | uint32 | — |
| `q` | `q` | 8 | int64 | — |
| `Q` | `Q` | 8 | uint64 | — |
| `f` | `f` | 4 | float | — |
| `d` | `d` | 8 | double | — |
| `g` | `e` | 2 | float16 | — |
| `n` | `4s` | 4 | char[4] | NUL-terminated string |
| `N` | `16s` | 16 | char[16] | NUL-terminated string |
| `Z` | `64s` | 64 | char[64] | NUL-terminated string |
| `a` | `64s` | 64 | int16[32] | reinterpret as 32 little-endian int16 |
| `c` | `h` | 2 | int16 × 100 | **× 0.01** |
| `C` | `H` | 2 | uint16 × 100 | **× 0.01** |
| `e` | `i` | 4 | int32 × 100 | **× 0.01** |
| `E` | `I` | 4 | uint32 × 100 | **× 0.01** |
| `L` | `i` | 4 | lat/lng degE7 | **× 1e-7** |
| `M` | `B` | 1 | flight mode | — |

Note `a` is 64 bytes (32 shorts), not 128. Some older documentation says 128; it is wrong
for the `a` used by `ISBD`-adjacent messages in current firmware — trust `struct.calcsize`
against the `FMT.length` field if you are ever unsure.

pymavlink's `DFReader` divides by the reciprocal rather than multiplying by a small float
(`v /= 1e7` rather than `v *= 1e-7`) because it is more accurate in binary floating point.
Worth copying if you care about the last digit of a coordinate.

## FMTU / UNIT / MULT — metadata, not scaling

`FMTU` maps a format type to per-field unit ids and multiplier ids; `UNIT` and `MULT`
define those ids.

**Multipliers from `MULT` are NOT applied to values.** Only the format-char scaling above
is. pymavlink's `DFReader` (`set_mult_ids()` writes only to `self.units`, never to
`msg_mults`), `JsDataflashParser` and the Rust `ardupilot-binlog` crate all behave the same
way, so this is the de-facto ecosystem convention rather than one parser's quirk. A `MULT`
id of ×100 on a field declared `f` changes the *displayed unit label* and leaves the number
alone.

Two gotchas if you do implement `FMTU`:

- `FMTU` can arrive before the `FMT` it describes; handle both orders.
- `MULT` values are logged as doubles that were originally floats, so round to 7
  significant figures before using them as dict keys — `float("%.7g" % v)` — otherwise
  lookups on `1.0e-2` silently miss.

`MULT_TO_PREFIX = {0: "", 1: "", 1e-1: "d", 1e-2: "c", 1e-3: "m", 1e-6: "µ", 1e-9: "n"}`.

## Instance fields

Multi-sensor messages carry an instance index, but **the column name is not consistent**:

| message | instance column |
|---|---|
| `ESC`, `EDT2` | `Instance` |
| `MAG`, `GPS`, `GPA`, `FCNS`, `BARO` | `I` |
| `BAT` | `Inst` |
| `XKF1`–`XKF5`, `XKQ`, `XKFS` | `C` (core) |
| `VIBE`, `IMU` | `IMU` |
| `ISBH`, `ISBD` | `N` (batch number) + `instance` |

`dflog` auto-detects by probing `Instance, Inst, IMU, Core, C, I, Id, N` and requiring the
column to be a small set of small non-negative integers. `log.instances("ESC")` returns
`{0: df, 1: df, ...}`.

pymavlink computes the instance offset with `struct.calcsize` on the format prefix so it
can read the instance without unpacking the whole message — worth copying if you need to
index a very large log cheaply.

## Timestamps

`TimeUS` is microseconds since boot, in almost every message. It is **not** wall-clock. If
the FC booted before GPS time was available the log's filename date will be 1980 — that is
an unset RTC, not a fault. Real date and time come from `GPS.GWk` / `GPS.GMS` (GPS week
and milliseconds-of-week), or from the `MSG` banner if the GCS wrote one.

## Practical notes

- **Cache the parse.** Re-parsing a 16 MB log on every script costs real time; pickle the
  message dict once and load that instead. `dflog` does this automatically to
  `<log>.dfcache`, keyed on the source file's mtime and a cache-version constant.
- `struct.Struct` objects are not picklable — store the `FMT` definition and rebuild the
  struct on load (`__getstate__` / `__setstate__`).
- Scan for the header with `bytes.find(b"\xa3\x95", i)` rather than a Python byte loop; it
  is roughly two orders of magnitude faster on a log with any resync at all.

## Facts from the writer (`libraries/AP_Logger`, ArduPilot master, September 2026)

These come from reading the firmware rather than any reader, and they decide how a parser
should behave when the file is not perfect.

- **Field limits.** Name 4, format 16, labels 64, units 16, multipliers 16 characters —
  exactly the FMT/FMTU field widths — written with `strncpy_noterm`, so a full field has
  **no NUL terminator** (`XKF1`'s format `QBccCfffffffccce` is 16 chars; `PM`'s labels are
  64). Decode to the first NUL or the field end.
- **Record length ≤ 255 bytes** including the header; `ISBD` (207 bytes) is the largest
  static message. `FMT.Length` always equals 3 + the size of the format string in a healthy
  log; the SITL-only `validate_structure` enforces it at build time.
- **Self-description order.** The first record is always FMT-of-FMT. Every write funnels
  through `ensure_format_emitted()`, so a type's FMT (immediately followed by its FMTU) is
  written before its first data record — **FMTs appear anywhere in the file**, not only at
  the start, and dynamic `Write()` messages get theirs inline. A data record for a type
  with no preceding FMT therefore means a lost FMT or corruption.
- **Startup sequence.** FMT for every static structure, then every `PARM`, then `UNIT` ×39,
  `MULT` ×15, then `FMTU` for every static structure, then the `MSG` banner (firmware,
  `ChibiOS: <hash>`, board id, `Param space used`, `RC Protocol`, `RCOut:` banner), `VER`,
  `RTC` (4.7+), mission/rally/fence, then the vehicle's `Frame:` message and `MODE`. Data
  records from the main thread interleave with all of this.
- **Message ids.** Only 128 (FMT) is fixed. 0–31 are vehicle-specific (Copter: 0 `CTUN`),
  common ids start at 32 (`PARM`) and depend on build options, and dynamic messages count
  **down from 254**. Never hard-code an id.
- **How a log ends.** File backend: no terminator, no padding; power loss leaves a
  512-byte-aligned end that is almost never on a record boundary → `TRUNCATED_TAIL`. Block
  (raw flash) backend: partial final page zero-filled, erased pages `0xFF`; a MAVLink
  download strips the page headers, so a downloaded block-backend log ends in 0–249 zero
  bytes → `TRAILING_PADDING`.
- **Dropped records** are counted in `DSF.Dp`; gaps in a stream are normal under buffer
  pressure and are not corruption. `coverage` reports the count.
- **`PARM`** gained `Default` in 4.3; it is NaN when the default is unknown (in-flight
  re-emission). A parameter re-appears each time it is set: take the **last** value before
  the time of interest (`log.param_at`).
- **`MSG`** text is truncated to 64 characters; from 4.7 it is chunked as
  `TimeUS,Id,Seq,Message` and must be reassembled. GCS statustexts are mirrored but only the
  first 50-character piece.
- **`FILE`** (`NIBZ`: FileName, Offset, Length, Data) has **no TimeUS**; chunks can repeat
  after a write retry; `Data` is binary — reassemble by (name, offset) honouring `Length`.
- **`ISBH`/`ISBD`.** `INS_LOG_BAT_CNT` is rounded down to a multiple of 32; `ISBD.seqno`
  runs 0..cnt/32−1 within a batch; value = raw / `mul` (gyro default 938, accel 208 — read
  it, never assume); post-filter batches use `instance + IMU count`; sampling pauses while a
  batch drains so batches are internally contiguous but separated by gaps.
- **`g` (float16)** exists from 4.6; ardupilot-binlog and JsDataflashParser do not support
  it. **`M`** is `uint8_t`; pymavlink reads it as signed.
- **Timestamps.** Not every message starts with `TimeUS` (`FMT`, `FILE`, most replay
  messages); heuristics must be defensive. The RTC message (4.7+) carries the epoch.
