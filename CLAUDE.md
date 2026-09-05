# ardupilot-log-tools — operating guide for coding agents

This project exists so an AI agent can analyse an ArduCopter dataflash log **efficiently
and accurately**. This file is the operating guide; `RULES.md` is the contract it rests on.
The tooling is half the value; the other half is knowing which numbers matter, which windows
they must be taken over, and which plausible-looking conclusions are wrong.

**Read `RULES.md` and this file before analysing any `.bin` log.** Then read whatever
project-specific notes exist for the aircraft — this repo is vehicle-agnostic and holds no
hardware baselines or tune state.

---

## 1. Start here

```bash
python alog.py info  flight.bin              # ALWAYS first: identity, integrity, coverage
python alog.py all   flight.bin --json       # the standard battery, one JSON document
python alog.py all   flight.bin              # the same as markdown
python alog.py motors flight.bin --window rpm
python alog.py fft   flight.bin --plot fft.png
python alog.py compare before.bin after.bin  # like-for-like, identical code both sides
python alog.py schema                        # the JSON contract, checks, thresholds
```

`python alog.py` works identically on Windows, Linux and macOS (`pip install -e .` also
gives an `alog` console script). Exit code is 0 / 1 / 2 for pass / warn / fail and 3 when
the input could not be analysed.

`info` costs one parse (about 1 s per 16 MB; later runs load a `<log>.dfcache` beside the
log) and tells you three things nothing else should surprise you about afterwards: what
the log is (firmware, board, UTC time), whether it is intact (the integrity diagnostics),
and what was logged at what rate (so you know before you start that a 25 Hz IMU stream
cannot show a 200 Hz motor). Delete `.dfcache` files when you are done.

In Python:

```python
from dflog import Log, airborne_window, mix_for, trim_decomposition, spectral

log  = Log("flight.bin")
log.diagnostics.ok                    # False if anything in the file was wrong
w    = airborne_window(log, method="rpm")
esc  = log.instances("ESC")           # {0: df, 1: df, ...}
rate = w.clip(log.df("RATE"))         # DataFrame, airborne only
```

---

## 2. Ground rules

These are not stylistic preferences. Each one exists because ignoring it has produced a
wrong or unusable answer.

1. **Read the integrity block first.** It is at the top of every report. A `RESYNC` or
   `TRUNCATED_TAIL` changes what the rest of the numbers mean. Codes are in
   `reference/integrity-codes.md`. Nothing was repaired; the numbers are computed on the
   data as logged.
2. **One change per flight.** Never batch parameter changes. A diagnostic is only
   trustworthy when one thing moved between two logs.
3. **Name the validation test.** A recommendation without "and here is the flight that
   proves it" is half a recommendation. Say what to fly and what to measure.
4. **Cite the parameter snapshot.** Values differ between dated captures.
   `alog params <log> --diff <file>` diffs the in-log parameters against one;
   `--non-default` lists what differs from the firmware defaults.
5. **Distinguish configured from measured.** A `.param` file says what the aircraft was
   *told*; the log says what it *did*. `MOT_THST_HOVER` is the standing example — it is
   relearned in flight; the `paramcheck` section compares it with the learned `CTUN.ThH`.
6. **Quote the window and the method.** Every number depends on it. A 4.7 % vs 6.31 %
   discrepancy between two analyses of the same log turned out to be nothing but
   whole-log versus airborne-only. The method string is in every report and every JSON.
7. **State the number, not the verdict.** "Vibration is fine" is useless next flight.
   "VibeX p95 8.2 m/s², 2 clip events" is a baseline.
8. **Report "not logged" as not logged.** A check that could not run is `SKIP`, never a
   pass. `alog` does this automatically and lists the skips under the verdict; do the same
   in prose.
9. **Report defaulted parameters.** When a section says "Parameters NOT in the log,
   defaults assumed", every number that depends on them is conditional.
10. **Never carry conclusions between aircraft.** Two airframes that share a transmitter
    share nothing else.

---

## 3. Picking the window — the decision that most affects your numbers

`--window METHOD` / `airborne_window(log, method=...)`:

| method | definition | when to use |
|---|---|---|
| `ev` (first choice under `auto`) | `EV` 28 NOT_LANDED → 18 LAND_COMPLETE | general analysis; the FC's own opinion of "flying" |
| `rpm` | fleet-mean ESC fundamental > 90 Hz | **motor, notch and vibration work, and any before/after comparison** — defined identically regardless of what the land detector believed |
| `throttle` | `CTUN.ThO` > 0.15 | fallback when there is no ESC telemetry |
| `arm` | `EV` 10 ARMED → 11 DISARMED | only when you actually want ground time included |
| `none` | the whole log | explicitly |
| `T0:T1` | explicit seconds since boot, e.g. `120:180` | a manoeuvre, a hover chunk, a GPS outage |

If a method cannot be applied the window falls back to the whole log **and the method
string says `FALLBACK`**. You can never mistake a fallback for the window you asked for.

`hover_chunks(log)` finds steady LOITER / ALT_HOLD segments with the sticks centred. Use it
(via `--window T0:T1`) when a statistic is only meaningful in steady hover — FFT peaks,
motor balance, vibration baselines. Attitude-error standard deviations rise ~20 % on a
livelier flight with no change to the tune at all.

`--pad N` trims N seconds off each end, the cheap way to drop takeoff and landing
transients.

---

## 4. What `alog all` runs, and why each is there

- **integrity** — the parser's diagnostics plus the NaN / logging-gap / duplicate-block
  scan. The first thing to read.
- **coverage** — which messages exist at what rate, and whether the fastest gyro source can
  resolve the motor fundamental and its second harmonic. Tells you up front which later
  sections will be `SKIP` and whether FFT work is even possible on this log.
- **events, flight** — `EV`/`ERR`/`MSG` decoded with subsystem names; prearm messages
  *after* landing are routinely the most informative lines in the whole log. Ever armed,
  ever flew, autotune outcome, lean beyond `ANGLE_MAX`, mode changes the pilot did not
  command.
- **paramcheck, brownout** — NaN parameters, parameters rewritten in flight, hover-throttle
  drift, and whether the log ended while still armed and airborne.
- **vibe, imu** — VIBE p95 and **clip counts**. Clipping is the hard failure — the
  accelerometer saturated and the EKF was fed garbage. IMU health counters and dual-IMU
  agreement.
- **motors** — per-motor RCOU and RPM through the **SERVOn_FUNCTION channel map**, then the
  trim decomposition (§5). A standing imbalance contaminates every tuning conclusion drawn
  from the same log, so clear it first.
- **notch** — `FCNS.CF` against `ESC.RPM/60`. Not `FTN1.PkAvg` — see §6.
- **pid, gust** — desired-vs-actual rate correlation **split at 5 Hz**: low-frequency error
  is a gain problem; high-frequency error is gyro noise reaching the controller, a filter
  problem. **Do the filter work before touching a rate gain.** `Dmod` staying at 1.000
  means the D-term slew limiter never engaged — no oscillation onset anywhere.
- **ekf, estimates** — `XKF4` innovation ratios, specifically the **count of samples above
  1.0**; a rising count across flights is the signal even when the mean improves. Solution
  status flags. ATT vs AHR2 / XKF1 and baro vs EKF divergence.
- **compass, power, gps, cpu** — field magnitude and variation, motor interference as the
  throttle correlation; **battery current correlated against throttle** (a flat reading is a
  wiring or pin fault, not a calibration error — a wrong `BATT_AMP_PERVLT` changes the
  magnitude, never the correlation); sats, HDOP, fix availability, position jumps; CPU
  load, slow loops, free memory, internal errors.
- **spectrum** — a local Welch PSD (scipy) of the fastest gyro source with peaks labelled
  in motor orders. It states the source, its Nyquist limit and the timing jitter, and
  refuses irregular sampling rather than transforming it.
- **batchfft** — the pre/post-filter notch proof from `ISBH`/`ISBD` (§6).

---

## 5. The trim decomposition — the motor check that actually diagnoses

Raw "RPM spread" says the motors disagree but not *how*, and a bent prop, an aft CG and a
twisted arm all produce spread. Projecting the standing per-motor deviation onto the
frame's own mix factors separates them. On a quad each figure is literally a difference
between two halves of the airframe:

```
roll  trim = mean(left motors)  - mean(right motors)
pitch trim = mean(front motors) - mean(rear motors)     negative => CG aft
yaw   trim = mean(CCW motors)   - mean(CW motors)       non-zero => standing torque
```

Roll and pitch are **thrust** asymmetry — CG, a damaged blade, mount height, wind.
Yaw is **torque** asymmetry — mount rotation, arm twist, a blade whose airfoil is wrong.

> **`RCOU.C<n>` is servo output n, not motor n.** ArduPilot motor numbers are geometry;
> which output drives which motor is wiring, declared by `SERVOn_FUNCTION` (33 = motor 1
> ... 36 = motor 4). Plenty of 4-in-1 AIO boards default to a non-sequential order so the
> ESC connector matches Betaflight — e.g. `SERVO1..4_FUNCTION = 36,33,34,35`, where C1 is
> motor 4. Feeding C1..C4 straight into the quad-X mix then reports **true yaw as roll,
> true roll as −yaw and true pitch as −pitch**, which dresses a pure thrust asymmetry up
> as a standing yaw torque and sends you looking for a twisted arm that does not exist.
> `check_motors` reads the map from the log, prints it, and warns when it had to assume
> the identity. The first write-up of the worked example below was wrong for exactly this
> reason and was corrected on 2026-09-04.

The worked example: a bent blade tip was straightened by hand. Read in motor order the
flight showed roll −22.6 → −25.3 µs (unchanged) with yaw +5.8 → +1.1 µs, and RPM spread
moved the *other* way (5.66 % → 7.48 %) — a standing thrust asymmetry on one diagonal and
essentially no torque asymmetry. A spread number would have shown neither.

`residual` is the part no control axis explains. A large residual on a quad means this is
not a trim at all: suspect a failing motor or a bad RPM channel.

> **Convention.** Mix factors are normalised to unit peak, matching ArduPilot's
> `normalise_rpy_factors()`. Some older analyses use raw `cos()` factors (±0.7071 on a
> quad X), which makes their roll and pitch figures 1.4142× larger; yaw is identical.
> `--raw-factors` reproduces those.

---

## 6. Notch verification — measure the filter, not the FFT

`FTN1.PkAvg` is the in-flight FFT's *opinion* of where the noise is. `FCNS.CF` is the
frequency the notch actually *applied*. They are the same series only when
`INS_HNTCH_MODE=4`. Once the notch is driven by ESC telemetry (`MODE=3`) they decouple, and
**only `FCNS.CF` answers the question.**

This is easy to get wrong in a way that yields a confident wrong answer. One analysis asked
whether `FTN1.PkAvg` would track RPM better after switching the notch's *source* — but
changing what consumes the FFT cannot change the FFT's own accuracy. The FFT got worse
across that change (6.31 % → 17.24 % mistracking, purely because the second flight was more
dynamic) while the notch itself went from 6.5 % mistracking to 0.00 %.

Ground truth is always `ESC.RPM / 60`. Motor noise frequency = RPM ÷ 60.

For the post-filter proof: `INS_LOG_BAT_MASK=1`, `INS_LOG_BAT_OPT=4` (pre **and** post
filter), fly 30–60 s, then **set the mask back to 0** — batch logging roughly doubles the
log rate, and on a board with onboard flash and no SD card that fills the chip fast enough
to lose the next flight entirely.

`batchfft` does two things a naive implementation gets wrong:

1. **Discards any batch window whose `ISBD.seqno` is not contiguous.** Batch logging drops
   samples routinely, and concatenating across a hole produces peaks that are not real.
2. **Order-normalises.** The notch centre moves with RPM, so averaging raw spectra smears
   the notch over tens of hertz and the dip vanishes. Normalising each batch by the notch
   centre the FC was tracking *at that instant* puts order 1.0 at the true fundamental.

With `INS_LOG_BAT_OPT=4` the post-filter batches carry `ISBH.instance` offset by the IMU
count (instance 1 on a one-IMU board); `alog fft --list-sources` labels them.

State this caveat in any report: a raw pre→post ratio is the notch **plus** the
`INS_GYRO_FILTER` low-pass sitting after it. To isolate the notch, fit a smooth baseline to
the transfer function outside the notch bands (excluding ±20 % around orders 1 and 2) and
measure the dip below it.

---

## 7. Local FFT — `alog fft` and `alog spectrum`

The transforms are scipy's (`scipy.fft`, `scipy.signal.welch`, `scipy.signal.find_peaks`).
What the tool adds is honesty about the input:

- `--list-sources` shows every transformable signal (`ISBD:gyro:0`, `GYR:0`, `IMU:0`,
  `RATE`) with its rate and Nyquist. The default is the fastest gyro source, pre-filter
  first.
- Irregular sampling (p95 interval more than 5 % above the median) is **refused**, not
  resampled, because an FFT of uneven samples is wrong in a way that looks right.
- When the Nyquist limit is below the motor fundamental the report says so and exits 1.
  The standard `LOG_BITMASK` logs IMU at 25 Hz: that stream cannot show a 200 Hz motor.
  Batch logging (`INS_LOG_BAT_MASK`) or raw logging (`INS_RAW_LOG_OPT`) is the fix.
- Peaks are reported in dB above the spectral floor and, when ESC telemetry exists, as
  multiples of the motor fundamental, so "order 1.98" reads as the second harmonic.

---

## 8. Thresholds

Every threshold lives in `dflog/checks.py::T` with a `source` field and is documented in
`reference/thresholds.md`. **Do not hardcode a number in a script.** If you need a new one,
add it to `T` with its provenance. `alog schema` prints the live table.

Sources are ArduPilot's own `Tools/LogAnalyzer` (removed from master in August 2024,
recoverable at `bdea9be7fb~1`), `dronekit-la`, the ArduPilot wiki, and values measured on
the logs this was developed against. Where two sources disagree both are recorded and the
stricter is used for WARN. A threshold is a prompt to look, not a verdict.

---

## 9. Pitfalls worth carrying in your head

Full list in `reference/pitfalls.md`.

- **Instance index ≠ physical device.** Which GPS is `GPS[0]` depends on SERIAL port order
  and can change between parameter snapshots. Identify a u-blox unit by the presence of
  `UBX2`, which only the u-blox driver emits.
- **`RCOU.C<n>` is a servo output, not a motor.** Map it through `SERVOn_FUNCTION`; see §5.
- **`MULT` multipliers are not applied.** Only format-char scaling (`c C e E L`) is.
  `alog fields MSG` shows both the unit and the (unapplied) MULT for every field.
- **Field names drift between firmware versions** (`BarAlt`→`BAlt`, `ThrOut`→`ThO`,
  `CRate`→`CRt`, `Chan1`→`C1`). Use `log.field(msg, "BAlt", "BarAlt")`.
- **Prefer `.bin` over a same-named `.log`/`.tlog`.** The text export is readable but
  flagged `TEXT_LOG`: pre-scaled values, decimated per-instance ESC telemetry, no batch
  arrays.
- **A log opened at arming has no ARMED event.** With `LOG_DISARMED=0` the file opens *at*
  arming, so `EV 10` is often missing; the `flight` check uses `ARM.ArmState` and the
  throttle as witnesses.
- **`MOTB.FailFlags = 2` is the healthy value**, not an error.
- **`PM.Load` is percent × 10.** 203 means 20.3 %. `PM.I2CI` and `PM.SPIC` are transaction
  counters, not error counts; internal errors are `PM.ErC` / `PM.InE`.
- **`POWR.Vcc` is NaN on boards without board-voltage sensing.** Not a fault.
- **Parameters disappearing between two captures is normal** — ArduPilot hides a disabled
  subtree, so all `FFT_*` vanish when `FFT_ENABLE` goes to 0. Not data loss.
- **A derived channel is not independent evidence.** `current_consumed` rising proves
  nothing about `current_battery`; it is that channel's own integral.
- **An unset RTC gives a 1980 log date.** The FC booted before GPS time was available.
  `alog info` derives the real UTC start from `GPS.GWk/GMS`.

---

## 10. Writing the report

Copy `templates/log-analysis-template.md`.

Its shape reflects what works: the integrity block and window first, a one-paragraph
verdict, then findings ordered by value rather than by subsystem, each with the number, the
threshold, the interpretation and the validation flight. New or worsening findings get a
⚠️ and are called out **even when unrelated to what you were asked about** — in one case
a degraded motor balance was the most important thing in the log and nobody had asked
about it.

Correct visibly rather than overwriting: keep the superseded number and date the amendment,
so a reader can tell a correction from a fresh measurement. A closed investigation should
record its next untried step, so reopening never repeats work.

Finish by listing the documentation drift you found. A report that silently leaves a stale
line in the project's notes has done half the job.

---

## 11. Extending

- New check → a `check_*(log, window)` in `dflog/analysis.py` returning a `Section` built
  with `sec.add(Result)`, `sec.table(...)`, `sec.note(...)`, registered in `ALL_CHECKS`. It
  gets a CLI subcommand and a JSON section automatically. Missing inputs → `SKIP` results.
- New threshold → `dflog/checks.py::T`, with a `source`.
- New integrity condition → a code in `dflog/parser.py`, a test in
  `tests/test_parser_integrity.py` built with `tests/synthlog.py`, and a row in
  `reference/integrity-codes.md`.
- New decoding you had to work out → `reference/`, so nobody derives it twice.

`tools/bootstrap_pymavlink.py` vendors pymavlink from GitHub when you need `mavextra`
(WMM expected earth field, magfit) or `mavfft_isb.py`. It is not needed for anything in
`alog`. See `reference/existing-tools.md`.

Run all three test files after changing the parser, the trim math, the checks or the CLI:

```bash
python tests/test_parser_integrity.py
python tests/test_cli.py
LOG_DIR=/path/to/logs python tests/test_toolkit.py
```

The regression tests pin the analysis against hand-verified figures; if a refactor changes
them, the refactor is wrong until proven otherwise. pytest is optional — the suite carries
a small shim and runs standalone.
