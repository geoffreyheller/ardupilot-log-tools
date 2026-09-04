# ardupilot-log-tools — instructions for coding agents

This file is for an AI agent asked to analyse an ArduPilot flight log. It is shipped with
the repo deliberately: the tooling is only half the value, and the other half is knowing
which numbers matter, which windows they must be taken over, and which plausible-looking
conclusions are wrong.

**Read this before analysing any `.bin` log.** Then read whatever project-specific notes
exist for the aircraft — this repo is vehicle-agnostic and holds no hardware baselines or
tune state.

---

## 1. Start here

```bash
./alog.py all flight.bin                 # the standard battery
./alog.py types flight.bin               # what messages this log actually contains
./alog.py motors flight.bin --window rpm # one check
./alog.py compare before.bin after.bin   # like-for-like, identical code both sides
./plot_notch.py flight.bin -o notch-verification-YYYY-MM-DD.png
```

Everything prints markdown, ready to paste into a report. Exit code is 0 / 1 / 2 for
pass / warn / fail.

First parse of a 16 MB log takes a few seconds and writes a `<log>.dfcache` pickle beside
it; later runs are instant. Delete those when you are done — flash space on the aircraft
and disk space next to the logs are both usually tighter than you expect.

In Python:

```python
from dflog import Log, airborne_window, mix_for, trim_decomposition, stats

log  = Log("flight.bin")
w    = airborne_window(log, method="rpm")
esc  = log.instances("ESC")         # {0: df, 1: df, ...}
rate = w.clip(log.df("RATE"))       # DataFrame, airborne only
```

---

## 2. Ground rules

These are not stylistic preferences. Each one exists because ignoring it has produced a
wrong or unusable answer.

1. **One change per flight.** Never batch parameter changes. A diagnostic is only
   trustworthy when one thing moved between two logs.
2. **Name the validation test.** A recommendation without "and here is the flight that
   proves it" is half a recommendation. Say what to fly and what to measure.
3. **Cite the parameter snapshot.** Values differ between dated captures.
   `alog.py params <log> --diff <file>` diffs the in-log parameters against one.
4. **Distinguish configured from measured.** A `.param` file says what the aircraft was
   *told*; the log says what it *did*. `MOT_THST_HOVER` is the standing example — it is
   relearned in flight, so the snapshot goes stale the moment you fly, and anything pinned
   to it drifts too.
5. **Quote the window and the method.** Every number depends on it. A 4.7 % vs 6.31 %
   discrepancy between two analyses of the same log turned out to be nothing but
   whole-log versus airborne-only.
6. **State the number, not the verdict.** "Vibration is fine" is useless next flight.
   "VibeX p95 8.2 m/s², 2 clip events" is a baseline.
7. **Report "not logged" as not logged.** A check that could not run is `SKIP`, never a
   pass. `alog.py` does this automatically; do the same in prose.
8. **Never carry conclusions between aircraft.** Two airframes that share a transmitter
   share nothing else.

---

## 3. Picking the window — the decision that most affects your numbers

`airborne_window(log, method=...)`:

| method | definition | when to use |
|---|---|---|
| `ev` (default via `auto`) | `EV` 28 NOT_LANDED → 18 LAND_COMPLETE | general analysis; the FC's own opinion of "flying" |
| `rpm` | fleet-mean ESC fundamental > 90 Hz | **motor, notch and vibration work, and any before/after comparison** — defined identically regardless of what the land detector believed |
| `throttle` | `CTUN.ThO` > 0.15 | fallback when there is no ESC telemetry |
| `arm` | `EV` 10 ARMED → 11 DISARMED | only when you actually want ground time included |

`hover_chunks(log)` finds steady LOITER / ALT_HOLD segments with the sticks centred. Use it
when a statistic is only meaningful in steady hover — FFT peaks, motor balance, vibration
baselines. A number from a dynamic flight is not comparable with one from a hover, and
attitude-error standard deviations in particular will rise ~20 % on a livelier flight with
no change to the tune at all.

`--pad N` trims N seconds off each end, the cheap way to drop takeoff and landing
transients.

---

## 4. What to run every time, beyond what was asked

`alog.py all` covers this. Know why each is there:

- **Vibration and clip counts.** Clipping is the hard failure — the accelerometer
  saturated and the EKF was fed garbage for those samples.
- **Per-motor RCOU and RPM, then the trim decomposition** (§5). A standing imbalance
  contaminates every tuning conclusion drawn from the same log, so clear it first.
- **Desired-vs-actual rate correlation, split at 5 Hz.** Low-frequency error is a gain
  problem; high-frequency error is gyro noise reaching the controller, i.e. a filter
  problem. **Do the filter work before touching a rate gain.**
- **`Dmod`.** If it stays 1.000 the D-term slew limiter never engaged — no oscillation
  onset anywhere. That one number replaces a lot of speculation.
- **Notch centre (`FCNS.CF`) against `ESC.RPM/60`.** Not `FTN1.PkAvg` — see §6.
- **`XKF4` innovation ratios**, specifically the *count of samples above 1.0*. A rising
  count across consecutive flights is the signal; the mean can improve while rejections
  double.
- **Battery current correlated against throttle.** A flat, throttle-independent reading is
  a wiring or pin fault, not a calibration error — a wrong `BATT_AMP_PERVLT` changes the
  magnitude, never the correlation.
- **CPU load and long loops.** Cheap, and tells you whether there is headroom for a bigger
  FFT window or batch logging.
- **`MSG`, `EV` and `ERR`.** The FC's own commentary. Prearm messages *after* landing are
  routinely the most informative lines in the whole log.

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

The worked example that motivated it: a bent blade tip was straightened by hand. Roll trim
went +8.3 → +1.6 µs (fixed) while yaw stayed +22.6 → +25.3 µs (untouched). A straightened
blade recovers its thrust but not its twist and airfoil, so it recovers its lift and not
its drag torque. The conclusion — replace the prop rather than straighten it — is invisible
in a spread number and obvious in the decomposition.

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

`check_batch_fft` does two things a naive implementation gets wrong:

1. **Discards any batch window whose `ISBD.seqno` is not contiguous.** Batch logging drops
   samples routinely, and concatenating across a hole produces peaks that are not real.
2. **Order-normalises.** The notch centre moves with RPM, so averaging raw spectra smears
   the notch over tens of hertz and the dip vanishes. Normalising each batch by the notch
   centre the FC was tracking *at that instant* puts order 1.0 at the true fundamental.
   On a verified flight the attenuation minimum landed at order 0.990 and the second at
   2.010 — that is the evidence the notch is applied where the noise actually is.

State this caveat in any report: a raw pre→post ratio is the notch **plus** the
`INS_GYRO_FILTER` low-pass sitting after it. To isolate the notch, fit a smooth baseline to
the transfer function outside the notch bands (excluding ±20 % around orders 1 and 2) and
measure the dip below it.

---

## 7. Thresholds

Every threshold lives in `dflog/checks.py::T` with a `source` field and is documented in
`reference/thresholds.md`. **Do not hardcode a number in a script.** If you need a new one,
add it to `T` with its provenance.

Sources are mostly ArduPilot's own `Tools/LogAnalyzer` (removed from master in August 2024,
recoverable at `bdea9be7fb~1`), `dronekit-la`, the ArduPilot wiki, and values measured on
the logs this was developed against. Where two sources disagree — compass offsets are
300/500 in LogAnalyzer and 100/200 in dronekit-la — both are recorded and the stricter is
used for WARN.

A threshold is a prompt to look, not a verdict.

---

## 8. Pitfalls worth carrying in your head

Full list in `reference/pitfalls.md`.

- **Instance index ≠ physical device.** Which GPS is `GPS[0]` depends on SERIAL port order
  and can change between parameter snapshots. Identify a u-blox unit by the presence of
  `UBX2`, which only the u-blox driver emits. One analysis attributed a run of dropouts to
  the wrong unit this way and every derived statistic had to be recomputed.
- **`MULT` multipliers are not applied.** Only format-char scaling (`c C e E L`) is. If a
  value looks off by a power of ten, check `FMTU` — do not assume the parser handled it.
- **Field names drift between firmware versions** (`BarAlt`→`BAlt`, `ThrOut`→`ThO`,
  `CRate`→`CRt`, `Chan1`→`C1`). Use `log.field(msg, "BAlt", "BarAlt")`.
- **Prefer `.bin` over a same-named `.log`/`.tlog`.** Text exports decimate per-instance
  ESC telemetry, and a `.tlog` only ever shows the first-enumerated GPS.
- **`MOTB.FailFlags = 2` is the healthy value**, not an error.
- **`PM.Load` is percent × 10.** 203 means 20.3 %.
- **`POWR.Vcc` is NaN on boards without board-voltage sensing.** Not a fault.
- **Parameters disappearing between two captures is normal** — ArduPilot hides a disabled
  subtree, so all `FFT_*` vanish when `FFT_ENABLE` goes to 0. Not data loss.
- **A derived channel is not independent evidence.** `current_consumed` rising proves
  nothing about `current_battery`; it is that channel's own integral.
- **An unset RTC gives a 1980 log date.** The FC booted before GPS time was available.

---

## 9. Writing the report

Copy `templates/log-analysis-template.md`.

Its shape reflects what works: a one-paragraph verdict first, then findings ordered by
value rather than by subsystem, each with the number, the threshold, the interpretation and
the validation flight. New or worsening findings get a ⚠️ and are called out **even when
unrelated to what you were asked about** — in one case a degraded motor balance was the
most important thing in the log and nobody had asked about it.

Correct visibly rather than overwriting: keep the superseded number and date the amendment,
so a reader can tell a correction from a fresh measurement. A closed investigation should
record its next untried step, so reopening never repeats work.

Finish by listing the documentation drift you found. A report that silently leaves a stale
line in the project's notes has done half the job.

---

## 10. Extending

- New check → a `check_*(log, window)` in `dflog/analysis.py` returning a `Section`,
  registered in `ALL_CHECKS`. It gets a CLI subcommand automatically.
- New threshold → `dflog/checks.py::T`, with a `source`.
- New decoding you had to work out → `reference/`, so nobody derives it twice.

`tools/bootstrap_pymavlink.sh` vendors pymavlink from GitHub when you need `mavextra`
(WMM expected earth field, magfit) or `mavfft_isb.py`. It is not needed for anything in
`alog.py`. See `reference/existing-tools.md`.

Run `python3 tests/test_toolkit.py` after changing the parser, the trim math or the batch
FFT. The regression tests pin the analysis against hand-verified figures; if a refactor
changes them, the refactor is wrong until proven otherwise. pytest is optional — the suite
carries a small shim and runs standalone.
