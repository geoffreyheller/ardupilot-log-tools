# Attribution

All code in this repository is original and MIT-licensed. No source from another
project was copied. Several *methods* and *numeric thresholds* were learned from the
projects below, and this file records that debt explicitly.

Numeric thresholds and measurement techniques are facts and procedures rather than
expressive works, so they carry no licence obligation — but they were somebody's work, and
saying so is the point of this file.

## Methods

**[ArduPilot/pymavlink](https://github.com/ArduPilot/pymavlink)** — LGPL-3.0.
`tools/mavfft_isb.py` is the reference implementation for batch-IMU (`ISBH`/`ISBD`)
spectral analysis. `dflog/analysis.py::check_batch_fft` follows its method:
sequence-hole rejection, Hann windowing with `S2 = inner(w, w)` normalisation,
`psd = 2 · mean(|rfft|²) / (fs · S2)`, and zeroing the DC and Nyquist bins. Its
`--notch-params` heuristics are documented in `reference/thresholds.md`.
`DFReader.py` informed the format-character table in `dflog/parser.py`, the handling of
`FMTU` arriving before its `FMT`, and the confirmation that `MULT` multipliers are display
metadata rather than value scaling.

**[ArduPilot/ardupilot](https://github.com/ArduPilot/ardupilot)** — GPL-3.0.
`Tools/LogAnalyzer` (removed from master in commit `bdea9be7fb`, August 2024) supplied
many of the pass/fail thresholds in `dflog/checks.py` and the "find a stable hover window"
idea reimplemented as `dflog.flight.hover_chunks()`. The DataFlash format itself is
documented from ArduPilot's own logging code.

**[dronekit/dronekit-la](https://github.com/dronekit/dronekit-la)** — Apache-2.0.
Its analyzer design — every check emitting a status, a severity, a time window and
evidence, gated by a minimum duration — is the shape of `dflog.checks.Result`. Several of
its thresholds are recorded in `reference/thresholds.md`, generally stricter than
LogAnalyzer's.

**[ArduPilot/UAVLogViewer](https://github.com/ArduPilot/UAVLogViewer)** and its parser
submodule **[Williangalvani/JsDataflashParser](https://github.com/Williangalvani/JsDataflashParser)** — GPL-3.0.
Independent confirmation of the `MULT` convention.

**[AveryanAlex/ardupilot-binlog](https://github.com/AveryanAlex/ardupilot-binlog)** —
MIT OR Apache-2.0. A clean, fuzz-tested Rust reimplementation, useful as a cross-reference
when two parsers disagree.

## Documentation

Threshold rationale and parameter semantics draw on the ArduPilot wiki, in particular the
pages on [IMU batch sampling](https://ardupilot.org/copter/docs/common-imu-batchsampling.html),
[in-flight FFT](https://ardupilot.org/copter/docs/common-imu-fft.html) and
[analog current calibration](https://ardupilot.org/copter/docs/common-analog-current-calibration.html).

`reference/existing-tools.md` carries a fuller survey of the ecosystem, including what was
evaluated and rejected.
