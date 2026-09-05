# The open-source landscape: audit, and what was taken from it

Surveyed September 2026 by reading source, not documentation: pymavlink (`5def49ee`,
2026-09-02), ArduPilot firmware `AP_Logger` (master `70d834ba`, plus the 4.2–4.7 release
branches for version dating), ArduPilot `Tools/LogAnalyzer` (at `bdea9be7fb~1`, its last
commit before removal), dronekit-la (`84b1b9fe`, 2022), MAVProxy (`a7e6aec`), Mission Planner
(`27d7dab`), UAVLogViewer (`01f9d51`) with its `JsDataflashParser` submodule (`d8967f6e`),
`AveryanAlex/ardupilot-binlog` (Rust, v0.2.0), ArduPilot WebTools (`23d1ade`) and the
ArduPilot wiki. This exists so nobody re-runs the survey, and so it is clear which parts
of `dflog` are borrowed and which are original. `NOTICE.md` carries the attribution that
matters legally; this file is the reasoning.

---

## Part 1 — parsers: how each one handles bad input

The DataFlash format has no length field, no CRC and no end marker; every record is
`0xA3 0x95 <type> <payload>` and the payload length comes only from the `FMT` record that
defined the type. So the interesting differences between readers are all in what they do
when something is wrong.

| condition | pymavlink `DFReader` | ardupilot-binlog (Rust) | JsDataflashParser (UAVLogViewer) | **dflog** |
|---|---|---|---|---|
| framing length | trusts `FMT.Length` | trusts `FMT.Length` | sum of the format string; `Length` ignored | sum of the format string; **`FMT_LENGTH_MISMATCH` error** when `Length` disagrees |
| `Length` < 3 | Python indexer hangs; C indexer breaks | integer underflow / abort | n/a | reported, decoded by format |
| columns ≠ format chars | unchecked; `IndexError` later on access | pads `field_N` silently | unchecked | **`FMT_COLUMN_COUNT_MISMATCH` error**, padded `F<n>` |
| unknown format char | drops the FMT silently in the sequential reader; aborts `init_arrays` | per-record `InvalidFormat`, record dropped + resync | **stops parsing the whole file** (`offset += NaN`) | **`FMT_UNKNOWN_FORMAT_CHAR` error** + every record counted as `UNPARSEABLE_TYPE` |
| `g` float16 (4.6+) | supported | **not supported** | **not supported** | supported |
| unknown type id | **index stops at the first one** (sequential reader resyncs); stderr | resync; **stops for good after 256 consecutive errors** | does not advance; scans the payload for false headers | **`UNKNOWN_MSG_TYPE` error** by type id + `RESYNC` with byte counts; never stops |
| mid-file garbage | "bad header" to stderr **per byte**, no count kept | resync, no count | resync, no count | **`RESYNC` error** with skipped-byte count and offsets |
| truncated last record | `None` (end of log); nothing recorded | dropped silently | dropped silently | **`TRUNCATED_TAIL` warning** with message type and bytes present |
| trailing `0xFF` / `0x00` | 528-byte end-of-file heuristic suppresses the spam | scanned past | scanned past | **`TRAILING_PADDING` info**, distinguished from `TRAILING_GARBAGE` |
| FMT redefinition | last wins; caches not invalidated (desync possible) | last wins, can even overwrite type 128 | last wins | **`FMT_REDEFINED` warning**; earlier records keep the old definition |
| `MULT` | label prefix only, never applied | not read at all | static tables, not applied | not applied; read from the log and shown by `alog fields` |
| instance detection | FMTU `#` marker | none | FMTU `#` only (no split on pre-3.6 logs) | FMTU `#` first, label heuristic fallback |
| `MSG` 4.7+ chunks | no reassembly | no | no | reassembled by `(Id, Seq)` |
| `FILE` records | `Z` returned raw | text | concatenates text, **ignores Offset/Length** (corrupts binary files) | reassembled by `(name, offset)` honouring `Length`; `alog files` |
| NaN / Inf | pass through | pass through | pass through | pass through, **counted per field** (`NAN_VALUES`), null in JSON |
| time going backwards | TimeMS guarded, TimeUS not | none | none | **`TIME_NON_MONOTONIC`** per message (info for metadata, warning otherwise) |
| GPS → UTC | week/ms − 18 s leap | none | week/ms, leap-second table, back-dated by `TimeUS` | week/ms − 18 s, back-dated by `TimeUS` (`alog info`) |
| text `.log` | `DFReader_text`, no rescaling | no | no | `dflog/textlog.py`, no rescaling, **flagged `TEXT_LOG`** |

**Taken from pymavlink:** the format-char table and the confirmation that MULT is display
metadata; divide by the reciprocal for accuracy; round MULT values to 7 significant figures
before using them as keys; handle FMTU before FMT; the 18 s leap-second constant; the
observation that the block-flash backend leaves up to 249 bytes of trailing space.

**Taken from the firmware source (the writer, `libraries/AP_Logger`):** the guarantee that
FMT-of-FMT is the first record and that a type's FMT (followed by its FMTU) always precedes
its first data record, so an unknown type means a lost FMT or corruption; that `n/N/Z` and
the FMT fields are `strncpy_noterm`'d and may fill the field with no NUL; that the erased
filler is `0xFF` and the partial-page filler `0x00`; that `PARM.Default` is NaN when
unknown; that `MSG` text is chunked at 64 bytes from 4.7; that `FILE` has no `TimeUS`; that
post-filter `ISBH.instance` is offset by the IMU count; that dynamic message ids count down
from 254 and only 128 is fixed; the full `LogEvent`, `LogErrorSubsystem`, `LogErrorCode` and
`ModeReason` enumerations in `dflog/flight.py` and `dflog/analysis.py`.

**Taken from ardupilot-binlog:** the discipline of a named error for every condition, and
its test list (empty input, FMT-only, garbage between records, truncated final record,
unknown type, scaled fields, 4-byte `n` with no NUL) which `tests/test_parser_integrity.py`
covers and extends. Avoided: its 256-consecutive-error stop.

**Taken from JsDataflashParser / UAVLogViewer:** the FMTU `#` instance marker, the
"last PARM before the time of interest" idea (`log.param_at`), back-dating UTC by the
`TimeUS` delta. Avoided: stopping the whole parse on one bad format character.

---

## Part 2 — analysers: every check the ecosystem computes, and our coverage

### ArduPilot `Tools/LogAnalyzer` (removed 2024-08-14; Mission Planner's "Auto Analysis" still runs a py2exe build of it)

| test | thresholds | dflog |
|---|---|---|
| Autotune (EV 30–37 session outcome) | last session wins | `flight`: outcome + gains saved |
| Brownout (armed at log end & BAlt > 3 m) | FAIL | `brownout` + parser `TRUNCATED_TAIL` |
| Compass (offsets 300/500, field change 25/35 %, band 120–550) | | `compass` (stricter dronekit-la 100/200 for WARN; p99−p01 instead of max−min) |
| DupeLogData (repeated 20-run of ATT.Pitch) | FAIL | `DUPLICATE_DATA`, hashed O(n) rather than sparse probes |
| Empty (ThrOut never > 20 %) | FAIL | `flight` ever flew |
| Events (ERR table; fence-only WARN) | | `events`, with subsystem names |
| GPSGlitch (sats <6/<5, HDOP >3/>10, ERR 11/2) | | `gps`, plus implied-speed jumps |
| IMUMatch (LPF accel magnitude diff 0.75/1.5) | | `imu` |
| MotorBalance (channel mean spread 75/150 µs) | | superseded by the trim decomposition through the SERVOn_FUNCTION map |
| NaN (any NaN outside an allow-list) | FAIL | `NAN_VALUES` with the same allow-list extended |
| Params (NaN, MAG_ENABLE, THR_MIN/MID) | | `paramcheck` (NaN, in-flight changes, hover drift, failsafe params) |
| Performance (NLon/NLoop 6/10 %) | | `cpu` |
| PitchRollCoupling (lean > ANGLE_MAX+10° above 2 m) | | `flight` lean vs ANGLE_MAX |
| Thrust (climb rate at ThrOut > 700 — never true on 0–1 ThO) | | not ported: the check is dead on modern logs |
| VCC (min 4.6 V, ripple 0.3 V) | | `power` |
| Vibration (2σ IMU accel in LOITER chunk) | | `vibe` uses VIBE p95 (wiki), with `hover_chunks()` available |
| OptFlow calibration | | not ported (no optical flow on the development aircraft) |
| DualGyroDrift | shipped disabled | `imu` gyro bias per IMU |

### dronekit-la

Adopted: the result contract (status, severity, evidence, window), the minimum-duration
idea, and thresholds for attitude control (5/10°), attitude/altitude/velocity estimate
divergence (5/10°, 4/5 m), EKF variances (0.5/1.0), compass offsets (100/200), vector
length (120/550), gyro drift, arming checks, ever armed/flew, crash detection. Its
`Good EKF` analyser requires every status bit set and therefore fails healthy flights;
`ekf` here checks the core flags only and reports the GPS-glitching bit as a percentage.
Its battery and sensor-health analysers read MAVLink only and are inert on `.bin`.

### MAVProxy / pymavlink tools

`mavfft_isb.py` is the reference batch-FFT method and `batchfft` follows it (hole
rejection, Hann, `2·Σ|X|²/(count·fs·Σw²)`, DC/Nyquist zeroed); MAVExplorer's `fft` uses
no window and no PSD scaling. Its 50 % overlap synthesises batches from halves of
neighbours and is not ported. `mavfft_int.py`'s dropout detection (`SampleC` deltas
> 1.5× mean) is the ancestor of `LOG_GAP`. The graph presets in `MAVProxy/tools/graphs/`
are the developers' curated "what to plot" list and informed which fields the checks read.
`mavextra` (WMM expected field, magfit, low-pass helpers) is worth vendoring via
`tools/bootstrap_pymavlink.py` rather than re-implementing.

### Mission Planner

The FFT tool (`Controls/fftui.cs`) uses a 4/N-normalised Hanning window, non-overlapping
blocks averaged in dB, and for `ISBH/ISBD` **does not check `seqno`** — a hole becomes a
spurious peak. Its `MagCalib.cs` least-squares sphere/ellipsoid fit is the standard
offline compass calibration. No additional pass/fail checks beyond the bundled LogAnalyzer.

### UAVLogViewer and WebTools

UAVLogViewer has no automatic checks; its EKF status-bit decoder and MagFit tool are
plotting aids. WebTools' FilterReview (50 % overlap Hann, PSD in dB/Hz, predicted
post-filter response from the configured notch) is the best interactive spectral tool in
the ecosystem; HardwareReport decodes `WDOG`, internal errors and `DSF.Dp` dropped
records, which `coverage` now reports too.

### Wiki rules of thumb encoded here

VIBE < 30 m/s² ok, > 60 nearly always a problem; clip counts ideally 0; mag field 120–550
and motor interference < 30 % ok / > 60 % bad; HDOP < 1.5 very good, > 2.0 could be bad;
Vcc ripple 0.10–0.15 V normal; EKF failsafe when two of SM/SP/SV exceed `FS_EKF_THRESH`
for 1 s; `FFT_SNR_REF` 25 dB and the 40 dB motor-noise warning; ERR subsystem/code table.

### What no surveyed tool did (and this one does)

- A trim decomposition through the frame's mix factors, with the servo-to-motor map read
  from the log. dronekit-la hard-codes an X-quad and ignores `SERVOn_FUNCTION`.
- Validating the applied notch centre (`FCNS.CF`) against ESC RPM.
- Counting `XKF4` innovation samples above 1.0 rather than tracking the maximum.
- Order (RPM) normalisation of batch spectra before averaging.
- Battery current-vs-throttle correlation as a wiring diagnostic; `Dmod` engagement; the
  5 Hz band split of rate error.
- An integrity report with stable codes, counts and offsets, and a `--strict` refusal.
- Airborne-window discipline everywhere: LogAnalyzer's GPS/HDOP/compass/vibration
  thresholds run over the whole log including the bench; dronekit-la gates only three
  analysers on `is_flying()`.

---

## Why `dflog` has its own parser

pymavlink's `DFReader` is the reference implementation and would be the obvious dependency.
Two things argued against making it the default:

1. **pymavlink's repo ships no generated MAVLink dialects** (`dialects/.gitignore` excludes
   `*.py`), and `message_definitions/` is not in that repo at all — it lives in
   `ArduPilot/mavlink`. `DFReader.py` does `from . import mavutil`, and `mavutil` calls
   `set_dialect()` at import time, which tries to auto-generate and dies. So `DFReader` is
   not importable from a fresh clone despite being logically standalone.
2. The parser here is small, has no dependency beyond numpy, parses a 16 MB log in about a
   second, is fully understood by whoever is reading it, and — the point of this project —
   reports every deviation it meets instead of printing to stderr and carrying on.

`tools/bootstrap_pymavlink.py` (and the `.sh` original) vendors pymavlink with generated
dialects for when you want `mavextra` or `mavfft_isb.py`.

---

## Wiki references worth keeping

- Diagnosing problems using logs: https://ardupilot.org/copter/docs/common-diagnosing-problems-using-logs.html
- Measuring vibration: https://ardupilot.org/copter/docs/common-measuring-vibration.html
- IMU batch sampling: https://ardupilot.org/copter/docs/common-imu-batchsampling.html
- In-flight FFT: https://ardupilot.org/copter/docs/common-imu-fft.html
- Analog current calibration: https://ardupilot.org/copter/docs/common-analog-current-calibration.html
- Raw / batch IMU logging: https://ardupilot.org/copter/docs/common-raw-imu-logging.html
- Log message reference (generated from the firmware): https://autotest.ardupilot.org/LogMessages/
- Full parameter reference: https://ardupilot.org/copter/docs/parameters.html
