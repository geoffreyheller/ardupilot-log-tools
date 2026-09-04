# ardupilot-log-tools

A dependency-light Python toolkit for analysing ArduPilot / ArduCopter dataflash (`.bin`)
logs, built for repeatable before-and-after flight comparisons rather than one-off
plotting.

It parses the DataFlash format directly — no pymavlink, no compiled extensions — and ships
a battery of checks whose thresholds all carry a cited source, so a verdict can be argued
with instead of merely believed.

```bash
./alog.py all flight.bin
```

```
Window: 77.0-239.1 s (162.1 s) via EV NOT_LANDED->LAND_COMPLETE.

## Motors and standing trim
[PASS] roll trim: 1.1 us standing trim (warn 10, fail 25)
[WARN] pitch trim: -22.4 us standing trim (warn 10, fail 25)
[FAIL] yaw trim: 25.3 us standing trim (warn 10, fail 25)
[WARN] RPM spread: 7.5% across motors, on medians (warn 3, fail 8)

| axis     | trim (us) | reads as                                         |
| -------- | --------: | ------------------------------------------------ |
| roll     |       1.1 | mean(left) - mean(right)                         |
| pitch    |     -22.4 | mean(front) - mean(rear); negative = CG aft      |
| yaw      |      25.3 | mean(CCW) - mean(CW); non-zero = standing torque |
```

---

## Why this exists

Most ArduPilot log analysis is done by eye in a plotting tool, which is fine for spotting
a problem and poor for answering "did the change I made last flight actually help?". Three
things make that question hard, and this toolkit is mostly an attempt to fix them:

**Windows.** Every statistic depends on which slice of the log you took it over, and two
analyses of the same log can disagree purely because one included the descent. Here the
window is chosen explicitly, by a named method, and that method is printed at the top of
every report.

**Thresholds.** "Vibration looks fine" is not a baseline. Every check states the number, the
threshold it was graded against, and where that threshold came from.

**Provenance.** A `.param` file says what the aircraft was *told*; a log says what it
actually *did*. The two drift — `MOT_THST_HOVER` is relearned in flight — so the tools
keep configured and measured values distinct and will diff a log's parameters against a
snapshot for you.

---

## Install

```bash
git clone https://github.com/<you>/ardupilot-log-tools.git
cd ardupilot-log-tools
```

Python 3.9+ with numpy, pandas and scipy; matplotlib for the charts. No pymavlink
required. Nothing to build, nothing to install.

---

## Use

```bash
./alog.py all      flight.bin              # the standard battery
./alog.py types    flight.bin              # what messages this log actually contains
./alog.py motors   flight.bin --window rpm # one check
./alog.py compare  before.bin after.bin    # like-for-like, identical code both sides
./alog.py dump     flight.bin RATE --fields t,RDes,R > rate.csv
./alog.py params   flight.bin --diff snapshot.param
./plot_notch.py    flight.bin -o notch.png
```

Checks: `summary`, `events`, `vibe`, `motors`, `notch`, `pid`, `gust`, `ekf`, `compass`,
`power`, `gps`, `cpu`, `batchfft`. Output is markdown; exit code is 0 / 1 / 2 for
pass / warn / fail, so it drops into a script.

As a library:

```python
from dflog import Log, airborne_window, mix_for, trim_decomposition

log  = Log("flight.bin")            # parses once, caches beside the log
w    = airborne_window(log, method="rpm")
esc  = log.instances("ESC")         # {0: df, 1: df, 2: df, 3: df}
rate = w.clip(log.df("RATE"))       # pandas DataFrame, airborne only
```

`Log.field(msg, "BAlt", "BarAlt")` probes field-name aliases, because ArduPilot renames log
fields between versions and hardcoding one spelling is the usual way a script breaks on an
older log.

---

## Three things it does that are worth stealing

### Standing-trim decomposition, instead of "RPM spread"

Raw spread tells you the motors disagree but not *how*, and a bent blade, an aft CG and a
twisted arm all show up as spread. Projecting the standing per-motor deviation onto the
frame's own mix factors separates them. On a quad each figure is literally a difference
between two halves of the airframe:

```
roll  trim = mean(left motors)  - mean(right motors)     thrust asymmetry
pitch trim = mean(front motors) - mean(rear motors)      negative => CG aft
yaw   trim = mean(CCW motors)   - mean(CW motors)        torque asymmetry
```

The case that motivated it: a bent blade tip was straightened by hand. Roll trim went
+8.3 → +1.6 µs — fixed — while yaw sat unmoved at +22.6 → +25.3 µs. A straightened blade
recovers its thrust but not its twist and airfoil, so it recovers its lift and not its drag
torque. "Replace the prop rather than straighten it" is invisible in a spread number and
obvious in the decomposition.

Works for any frame in `dflog/frames.py`; unknown frames fall back to quad-X and say so.

### Measure the filter, not the FFT

`FTN1.PkAvg` is the in-flight FFT's *opinion* of where the noise is. `FCNS.CF` is the
frequency the harmonic notch actually *applied*. They are the same series only when
`INS_HNTCH_MODE=4`; once the notch is driven by ESC telemetry they decouple, and only
`FCNS.CF` answers the question. Ground truth is always `ESC.RPM / 60`.

This is easy to get wrong in a way that produces a confident wrong answer. One analysis
asked whether the FFT would track better after switching the notch's *source* — but
changing what consumes the FFT cannot change the FFT's own accuracy. The FFT got worse
across that change (6.3 % → 17.2 % mistracking, because the second flight was more
dynamic) while the notch itself went from 6.5 % mistracking to 0.00 %.

### Order-normalised batch-IMU spectra

For the post-filter proof you need `INS_LOG_BAT_MASK=1` and `INS_LOG_BAT_OPT=4`. Two things
then go wrong in a naive implementation:

- Batch logging drops samples routinely. Concatenating across an `ISBD.seqno` gap produces
  peaks that are not real, so any window with a gap is discarded.
- The notch centre moves with RPM, so averaging raw spectra smears the notch across tens of
  hertz and the dip disappears. Normalising each batch by the notch centre the FC was
  tracking *at that instant* puts order 1.0 at the true fundamental every time.

On a verified flight the attenuation minimum landed at order 0.990 and the second at
2.010 — evidence the notch is applied where the noise actually is, not merely configured to
be.

![notch verification](docs/notch-example.png)

---

## Layout

```
alog.py                   the CLI
plot_notch.py             notch verification chart
CLAUDE.md                 instructions for coding agents working with these logs
dflog/
  parser.py               .bin DataFlash reader + on-disk cache
  flight.py               window selection, events, modes, hover chunks
  frames.py               motor-mix geometry and the trim decomposition
  analysis.py             the check battery
  checks.py               Result contract + the cited threshold registry
  stats.py                describe / correlate / band-split / PSD helpers
  report.py               markdown formatting
reference/
  dataflash-format.md     the binary format, in enough detail to write a parser
  messages.md             message and field reference with units and decodings
  thresholds.md           every threshold and where it came from
  param-decodings.md      device IDs, bitmasks, enums, derived quantities
  pitfalls.md             mistakes made, so they need not be made again
  existing-tools.md       survey of the ecosystem and what was taken from it
templates/                report template
tools/bootstrap_pymavlink.sh
tests/
```

`reference/` is the part most likely to be useful even if you never run the code.
`dataflash-format.md` is a from-scratch parser spec; `pitfalls.md` is a list of things that
have produced wrong answers.

---

## The parser

`dflog/parser.py` reads the DataFlash format directly via its self-describing `FMT`
records. It parses a 6 MB log in about a second with zero resync bytes and caches the
result beside the log as `<name>.bin.dfcache`, keyed on mtime.

pymavlink's `DFReader` is the reference implementation and would be the obvious dependency,
but it is awkward to vendor: the repo ships no generated MAVLink dialects, the message
definitions live in a *different* repo, and `DFReader` imports `mavutil`, which tries to
generate them at import time and fails. `tools/bootstrap_pymavlink.sh` handles all of that
if you want `mavextra` (WMM expected earth field, magfit) or `mavfft_isb.py` — nothing here
requires it.

Two format details that cost people time: the `FMT` body is **86 bytes**, not 89 (the
three-byte packet header is not part of it), and **`MULT` multipliers are not applied** to
values — only format-character scaling (`c C e E L`) is. pymavlink, JsDataflashParser and
ardupilot-binlog all agree on the latter; `FMTU`/`UNIT`/`MULT` are display metadata.

---

## Tests

```bash
python3 tests/test_toolkit.py                  # pure unit tests
LOG_DIR=/path/to/logs python3 tests/test_toolkit.py   # plus the regression fixtures
```

The toolkit was validated against hand-written analyses of two logs produced before it
existed, and reproduces them exactly: attitude error sd 0.413° / 0.381°, standing trims
+8.3 / +12.9 / +22.6 µs, `XKF4.SM` max 1.56 with 6 rejections, RPM spread 7.48 %, notch
attenuation at order 0.99. Those numbers are pinned as tests. If a refactor changes them,
the refactor is wrong until proven otherwise.

The reference logs are not distributed with the repo. Tests that need one skip cleanly.
pytest is optional — `tests/_shim.py` is a 40-line stand-in so the suite runs anywhere.

---

## Scope and limitations

- Written against ArduCopter 4.x logs from multirotors. The parser is vehicle-agnostic;
  several checks (trim decomposition, notch, motor balance) assume a multirotor.
- Frame geometry covers the common quad / hexa / octa / deca layouts. Unknown frames fall
  back to quad-X and label themselves as having done so.
- Thresholds are a prompt to look, not a verdict. A clean sheet is not the same as a good
  flight, and a WARN that has been stable for ten flights is less interesting than a PASS
  that moved 3× since last time.
- Telemetry logs (`.tlog`) are not supported. Prefer the `.bin` anyway: text exports
  decimate per-instance ESC telemetry, and a `.tlog` only ever shows the first-enumerated
  GPS.

---

## Contributing

New checks go in `dflog/analysis.py` as a `check_*(log, window)` returning a `Section`, and
get a CLI subcommand automatically. New thresholds go in `dflog/checks.py::T` **with a
source** — please don't hardcode a number in a script. New decodings you had to work out
belong in `reference/`, so nobody derives them twice.

Run `python3 tests/test_toolkit.py` before opening a PR.

---

## Licence

MIT — see [LICENSE](LICENSE).

All code here is original; no source was copied from another project. Several methods and
numeric thresholds were learned from pymavlink, ArduPilot's `Tools/LogAnalyzer` and
dronekit-la, and [NOTICE.md](NOTICE.md) records that in detail.
