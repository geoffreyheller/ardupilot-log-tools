# The open-source landscape, and what was taken from it

Surveyed September 2026. This exists so nobody re-runs the survey, and so it is clear which
parts of `dflog` are borrowed and which are original. See `NOTICE.md` for the attribution
that matters legally; this file is the reasoning.

## The constraint that shapes the choices

This was developed in a sandbox where **PyPI was blocked** but `git clone` from GitHub
worked, so upstream tooling had to be vendored rather than installed.
`tools/bootstrap_pymavlink.sh` does that, and is still useful anywhere you want pymavlink
without adding it as a hard dependency.

---

## Why `dflog` has its own parser

pymavlink's `DFReader` is the reference implementation and would be the obvious dependency.
Two things argued against making it the default:

1. **pymavlink's repo ships no generated MAVLink dialects** (`dialects/.gitignore` excludes
   `*.py`), and `message_definitions/` is not in that repo at all — it lives in
   `ArduPilot/mavlink`, with the submodule relationship running the other way. Worse,
   `DFReader.py` does `from . import mavutil`, and `mavutil` calls `set_dialect()` at import
   time, which tries to auto-generate and dies with
   `FileNotFoundError: message_definitions/v1.0/all.xml`. So `DFReader` is not importable
   from a fresh clone despite being logically standalone.
2. Our parser is ~200 lines, has no dependency beyond numpy, parses a 6 MB log in about a
   second with zero resync bytes, and is fully understood by whoever is reading it.

The bootstrap is there for when you want `mavextra` or `mavfft_isb.py`, which are worth
having and are not worth reimplementing.

---

## Projects surveyed

### `ArduPilot/pymavlink` — actively maintained, LGPLv3
https://github.com/ArduPilot/pymavlink

`DFReader.py` is the reference `.bin` parser. **Taken from it:**

- The format-char table and the confirmation that **`MULT` is never applied to values** —
  `set_mult_ids()` only decorates unit label strings, it never writes to `msg_mults`.
- Divide by the reciprocal rather than multiply by a small float (`v /= 1e7`, not
  `v *= 1e-7`) for accuracy.
- Round `MULT` values to 7 significant figures before using them as dict keys — they are
  logged as doubles that were really floats, so lookups on `1.0e-2` otherwise miss.
- `FMTU` can arrive before the `FMT` it describes; handle both orders.
- Computing the instance offset via `struct.calcsize` on the format prefix, so an instance
  can be read without unpacking the whole message.

`tools/mavfft_isb.py` is the best single thing in the ecosystem for this purpose, and its
method is ported into `check_batch_fft`:

- Require `ISBD.seqno == previous + 1`; on a gap, **discard the whole window**.
- Convert gyro to deg/s (`numpy.degrees`) before transforming.
- Hann window; `S2 = numpy.inner(window, window)`; zero both `d_fft[0]` (DC) and
  `d_fft[-1]` (Nyquist); accumulate, then `psd = 2 × mean(|rfft|²) / (fs × S2)`. That
  factor of 2 and the `S2` normalisation are what make magnitudes physically meaningful.
  (It cites https://holometer.fnal.gov/GH_FFT.pdf.)
- 50 % overlap between windows — not yet ported here; add it if the batch count is ever
  the limiting factor.
- Its `--notch-params` suggestion: `INS_HNTCH_REF` = mean `CTUN.ThO` where `CTUN.Alt > 1`,
  `INS_HNTCH_FREQ` = the peak, `INS_HNTCH_BW` = peak ÷ 2.
- Decode tables: `hntch_mode {0 No, 1 Throttle, 2 RPM, 3 ESC, 4 FFT}`,
  `hntch_option {0 Single, 1 Double, 2 Dynamic, 4 Loop-Rate, 8 AllIMUs, 16 Triple}`,
  `batch_mode {0 Pre-filter, 1 Sensor-rate, 2 Post-filter, 4 Pre+post}`.

`mavextra.py` (which lives in pymavlink, not MAVProxy) is worth vendoring rather than
copying. Highlights: `lpalpha` + `lowpassHz` (the cleanest "filter a log field at a real
cutoff" primitive in the ecosystem); `mag_field_df()` applying the full soft-iron matrix
rather than just offsets; and `expected_earth_field()` / `earth_field_error()`, which use a
built-in WMM lookup table — a much better compass check than a fixed 120–550 mGauss band.
Also `earth_accel_df`, `earth_rates`, quaternion/Euler helpers, and geo utilities.

### `ArduPilot/ardupilot` → `Tools/LogAnalyzer` — GPLv3, **removed from master**
Removed 2024-08-14 in `bdea9be7fb1722e86185fdd0ba792a9d97acec74`, "the web-based tools are
supplanting this". Recover with `git checkout bdea9be7fb~1 -- Tools/LogAnalyzer`.

18 rule-based test classes. **Taken from it:** the numeric thresholds (see
`thresholds.md`), `DataflashLogHelper.findLoiterChunks()` — reimplemented as
`dflog.flight.hover_chunks()` — and the habit of probing field-name aliases rather than
hardcoding one spelling.

### `dronekit/dronekit-la` — Apache 2.0, unmaintained since Feb 2022
https://github.com/dronekit/dronekit-la

C++, must be built from source. **Taken from it:** the result contract — every check emits
a status, a severity score, a time window and evidence fields, so results are rankable and
machine-consumable rather than being prose. That is what `dflog.checks.Result` is. Also its
threshold set, which is generally stricter and better organised than LogAnalyzer's, and its
use of a **minimum duration** before a divergence counts, so transients do not trip a
check.

### `ArduPilot/MAVProxy` — GPLv3
Master HEAD is May 2025, so quiet but not dead. MAVExplorer's `graph`, `fft`, `magfit`,
`paramchange`, `stats`, `devid` commands are useful interactively. Its preset graph library
(`MAVProxy/tools/graphs/*.xml`, 2000+ lines) is ArduPilot devs' curated "what to actually
plot" list as expression strings — worth reading if you are adding a chart.

### `ArduPilot/UAVLogViewer` — GPLv3, actively maintained
https://github.com/ArduPilot/UAVLogViewer — the viewer behind plot.ardupilot.org. Its
parser is a submodule, `Williangalvani/JsDataflashParser`, which independently confirms the
"do not apply `MULT`" convention. Analysis logic is thin — it is a viewer — but
`src/tools/dataflashDataExtractor.js` has sensible normalisation routines (enumerate which
of `ATT`/`AHR2`/`NKF1`/`XKF1` are present and let the user choose; derive wall-clock from
GPS week/ms).

### Third-party parsers
| project | language | state | verdict |
|---|---|---|---|
| [`AveryanAlex/ardupilot-binlog`](https://github.com/AveryanAlex/ardupilot-binlog) | Rust | active (Mar 2026), MIT/Apache | The cleanest modern independent reimplementation. Purely `FMT`-driven, fuzz-tested. A good cross-reference when two parsers disagree. |
| [`rmargar/pymavlog`](https://github.com/rmargar/pymavlog) | Python | Apr 2025, MIT | A numpy-array wrapper **around** `pymavlink.DFReader`. Does not solve the PyPI problem. |
| [`PyFlightCoach/ArdupilotLogReader`](https://github.com/PyFlightCoach/ArdupilotLogReader) | Python | Apr 2026 | Returns pandas DataFrames, but also depends on pymavlink. |
| [`fredowski/swlogview`](https://github.com/fredowski/swlogview) | JS | Feb 2024, GPL-3 | Zero-install offline single-file viewer. No analysis logic. |

**There is no actively-maintained, pip-installable, pure-Python `.bin` reader independent of
pymavlink.** Every Python option either wraps `DFReader` or is a toy. The only genuinely
independent modern reimplementation is the Rust crate.

---

## Wiki references worth keeping

- IMU batch sampling: https://ardupilot.org/copter/docs/common-imu-batchsampling.html
- In-flight FFT: https://ardupilot.org/copter/docs/common-imu-fft.html
- Analog current calibration: https://ardupilot.org/copter/docs/common-analog-current-calibration.html
- Raw / batch IMU logging: https://ardupilot.org/copter/docs/common-raw-imu-logging.html
- Full parameter reference: https://ardupilot.org/copter/docs/parameters.html
