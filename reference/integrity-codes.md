# Integrity and data-quality codes

Every problem the parser finds in a log is recorded in `log.diagnostics` (structure) or
`log.quality()` (decoded values) as an `Issue` with a stable `code`, a `severity`
(`error` / `warning` / `info`), an optional `subject` (the message, type id or field it
concerns), the byte offset of the first occurrence, a count, and a `detail` dict.
`alog integrity` prints them all; every other report prints errors and warnings at the top
and carries the full list in `--json` under `log.integrity`.

**Severity meaning.** `error`: data was lost or could not be decoded, or the file is not
what it claims to be; `--strict` refuses the log and the integrity check is `FAIL`.
`warning`: the log is usable but incomplete or unusual; the integrity check is `WARN`.
`info`: a normal artefact worth knowing about; does not change any verdict.

Nothing listed here is repaired. The numbers in a report are computed on the data as
logged.

## Structure (from the parse)

| code | severity | meaning | detail keys |
|---|---|---|---|
| `EMPTY_FILE` | error | zero bytes | |
| `FILE_TOO_SHORT` | error | shorter than one FMT record (89 bytes) | |
| `NO_FMT` | error | no FMT record at all: not a DataFlash binary (a `.tlog` or an unrelated file) | |
| `NO_MESSAGES` | error | nothing decoded | |
| `LEADING_GARBAGE` | warning | bytes before the first `0xA3 0x95` header | `bytes` |
| `RESYNC` | error | bytes skipped mid-file to find the next header; something between two records was not a record. Counted in `log.resync_bytes` / `log.resync_events`. A healthy log has zero. | `bytes`, subject = type id or name that triggered it |
| `UNKNOWN_MSG_TYPE` | error | a header whose type id has no FMT. Its length is unknown, so the bytes up to the next header are skipped (`RESYNC` follows). ArduPilot only writes a data record after its FMT, so this means a lost FMT or corruption. | subject = type id |
| `UNPARSEABLE_TYPE` | error | a record whose FMT was seen but could not be used (`FMT_UNKNOWN_FORMAT_CHAR`, `FMT_EMPTY`) | subject = name |
| `FMT_EMPTY` | error | FMT with an empty name or format | subject = type id |
| `FMT_UNKNOWN_FORMAT_CHAR` | error | a format character outside `a b B h H i I f d g n N Z c C e E L M q Q`; every record of that type is undecodable | subject = name |
| `FMT_COLUMN_COUNT_MISMATCH` | error | number of column names differs from the number of format chars; decoded by format, missing names padded `F<n>`, extras dropped | `columns`, `format_len` |
| `FMT_LENGTH_MISMATCH` | error | `FMT.Length` differs from 3 + the size the format string occupies; decoded by format. (pymavlink trusts `Length`; ArduPilot guarantees they agree, so a mismatch is corruption of the FMT itself.) | `declared`, `computed` |
| `FMT_REDEFINED` | warning | the same type id defined again with a different name/format; earlier records keep the old definition | subject = type id |
| `FMT_DUPLICATE` | info | the same FMT re-sent identically (Replay logs, concatenated logs) | |
| `FMT_NAME_COLLISION` | warning | two type ids share a message name; if their columns differ the later one is stored as `NAME@<type>` | subject = name |
| `FMT_SELF_UNEXPECTED` | warning | FMT's own definition is not `BBnNZ` | |
| `TRUNCATED_TAIL` | warning | the file ends inside a record (power loss, full card, download cut short). The partial record is dropped. `alog brownout` says whether the aircraft was still armed. | `bytes` present, `msg_type` |
| `TRAILING_PADDING` | info | the tail is all `0xFF` (erased flash) or all `0x00` (block-backend page fill); normal for on-board flash | `bytes` |
| `TRAILING_GARBAGE` | warning | undecodable bytes at the end that are neither a record nor padding | `bytes` |
| `TRAILING_BYTES` | info | fewer than 3 stray bytes at the end | `bytes` |
| `FMTU_UNKNOWN_TYPE` | warning | an FMTU for a type with no FMT | subject = type id |
| `FMTU_LENGTH_MISMATCH` | warning | FMTU unit/multiplier strings shorter or longer than the format | subject = name |
| `UNIT_ID_UNKNOWN` / `MULT_ID_UNKNOWN` | info | a unit/multiplier id in FMTU with no UNIT/MULT record (old firmware, or the text export) | subject = name |
| `STRING_NON_ASCII` | info | string fields with bytes outside ASCII (replaced with U+FFFD) | `count_fields` |
| `TIME_NON_MONOTONIC` | warning / info | `TimeUS` runs backwards within one message type. `info` for metadata messages (FMTU, PARM, MSG, FILE, VER...) which the logger emits out of order by design; `warning` for anything else. | `backwards_steps`, `largest_step_us` |
| `NO_PARM` | warning | no PARM records; every check that reads a parameter falls back to a default and says so | |
| `CACHE_REBUILT` | info | the `.dfcache` beside the log was unusable (version, size, mtime or corruption) and the .bin was re-parsed | |
| `CACHE_WRITE_FAILED` | warning | the cache could not be written; every run will re-parse | |
| `TEXT_LOG` | warning | the input is a text (`.log`) export, not the `.bin`: values are pre-scaled by the exporter, records may be decimated, `a` array fields do not round-trip | |
| `TEXT_UNKNOWN_TYPE` | error | text lines naming a message with no FMT line | `lines` |
| `TEXT_FIELD_COUNT` | error / warning | text lines whose field count does not match the FMT (warning when the cause is an `a` array field, which the export cannot represent) | `lines` |
| `TEXT_BAD_VALUE` | warning | a non-numeric value in a numeric column (stored as NaN) | `lines` |
| `TEXT_BAD_FMT` | error | a malformed FMT line | |

## Data quality (`log.quality()`, computed from decoded values)

| code | severity | meaning | detail keys |
|---|---|---|---|
| `NAN_VALUES` | warning / info | NaN or Inf in a float field. `info` for fields that are NaN by design (`CTUN.DSAlt/TAlt`, `POS.RelOriginAlt`, `POWR.Vcc/VServo` on boards without sensing, `PARM.Default` when unknown, `FCNS.CF/HF` while the notch has no source, temperatures ESCs do not report); `warning` otherwise. LogAnalyzer's `TestNaN` fails on any NaN outside its allow-list. | subject = `MSG.field`, `count_values`, `of` |
| `LOG_GAP` | warning | a message logged at a steady rate has a gap longer than 10x its normal interval (and > 1 s): the logger stalled or dropped data | `gap_s`, `at_s`, `n_types`, `types` |
| `DUPLICATE_DATA` | error | the same 20 consecutive `ATT.Pitch` values appear twice at non-overlapping offsets: flash corruption or a replayed block (LogAnalyzer `TestDupeLogData`) | `first_row`, `second_row` |

## How other readers behave (for comparison)

- **pymavlink `DFReader`** trusts `FMT.Length`, stops its index at the first unknown type
  (the sequential reader resyncs), prints "bad header" once per byte to stderr, does not
  check `Length == 3 + struct size` and hangs on `Length < 3`, records no count of skipped
  bytes, and has no NaN or monotonic-time checks.
- **ardupilot-binlog (Rust)** drops a truncated final record silently, stops for good after
  256 consecutive errors, does not support the `g` (float16) character, and underflows on
  `Length < 3`.
- **JsDataflashParser (UAVLogViewer)** ignores `FMT.Length` and stops parsing the whole
  file at the first record of a type with an unknown format character.

This parser decodes by the format string, reports `Length` disagreement, never stops early,
supports all 21 characters, and counts everything it skipped.
