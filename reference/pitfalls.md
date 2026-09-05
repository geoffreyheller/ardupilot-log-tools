# Pitfalls

Mistakes actually made during development, plus traps documented upstream. Each one cost
somebody a rerun or a wrong conclusion. Read before analysing; add to it when you find a
new one.

---

## Parsing

**The `FMT` body is 86 bytes, not 89.** The three-byte packet header is not part of the
body. Getting this wrong produces plausible-looking garbage rather than an obvious crash.

**`MULT` multipliers are not applied to values.** Only format-char scaling (`c C e E L`)
is. pymavlink's `DFReader`, `JsDataflashParser` and the Rust `ardupilot-binlog` crate all
agree — `FMTU`/`UNIT`/`MULT` are display metadata. If a number looks off by a power of ten,
check `FMTU` rather than assuming the parser handled it.

**An unknown message type cannot be skipped by length.** There is no length field in the
packet; it comes from `FMT`. The only recovery is to scan forward for the next `0xA3 0x95`.
Count and report those bytes — a healthy log resyncs **zero** times, and a non-zero count
is the first thing to mention when the numbers look odd.

**Instance column names are inconsistent.** `Instance` on `ESC`, `I` on `MAG`/`GPS`/`FCNS`,
`Inst` on `BAT`, `C` on `XKF*`, `IMU` on `VIBE`. Auto-detect or use `log.instances()`.

**Field names drift between firmware versions.** `BarAlt`→`BAlt`, `ThrOut`→`ThO`,
`CRate`→`CRt`, `Chan1`/`Ch1`→`C1`, `NSat`/`numSV`→`NSats`, `HDp`/`EPH`→`HDop`. Hardcoding
one spelling is the commonest way a script silently breaks on an older log. Use
`log.field(msg, "BAlt", "BarAlt")`.

**Re-parsing on every script costs real time.** Cache the parse; `dflog` pickles to
`<log>.dfcache`. `struct.Struct` is not picklable, so store the `FMT` definition and
rebuild.

**Delete `.dfcache` files when you are done.** They are large, and disk space next to a
log directory is usually tighter than expected.

---

## Windows and comparisons

**Every number depends on the window, and the window must be stated.** Two analyses of the
same log once produced 4.7 % and 6.31 % for the same measurement — whole-log versus
airborne-only. Not a contradiction, but it cost a paragraph of explanation that one line of
method would have prevented.

**A log can hold more than one flight, and a window that spans two is not a window.** On
a two-flight log (`2026-09-04 16-39-48.bin`: 64.5-255.9 s and 285.3-360.4 s, 30 s of
ground time between them) `alog all --window rpm` reported `notch 0 tracking` **FAIL**,
p95 error 1.234 of the fundamental, and `notch 0 harmonic lock-on` **FAIL**, 6.9 % of the
window above 1.5x. Both were pure artefacts of the ground time inside the window, where
`FCNS.CF` is NaN or clamped and there is no fundamental to track: `check_notch`
interpolates the fundamental across the gap and divides by it. Windowed per flight the
same two checks are **PASS**, 0.010 and 0 % - on *both* flights. The integrity block was
clean throughout. Fixed 2026-09-04 (issue #1): `--window rpm|throttle` no longer spans the
gap, `ev|arm` no longer silently analyse only the first flight, and a multi-flight log is
a WARN. Read the `flights in log` line, and use `--flight all` when you want everything.

**Use the same window method on both sides of a comparison.** `--window rpm` is the most
reproducible for motor and notch work because it is defined identically regardless of what
the FC's land detector believed.

**Run both flights through identical code.** Recomputing an old number with the new script
is cheaper than reconciling two methods later. `alog.py compare` exists for this.

**A statistic from a dynamic flight is not comparable with one from a hover.** In one
before/after pair, attitude-error standard deviations rose ~19 % while p95 values
were essentially unchanged — the signature of a more dynamic flight, not a worse-behaved
controller. Use `hover_chunks()` when the metric needs steady conditions.

---

## Instances and identity

**A log instance index is not a physical device.** Which GPS is `GPS[0]` depends on SERIAL
port order and **can change between parameter snapshots**. One analysis recorded "6
dropouts, 0.4–3.2 s each" against the wrong unit under a stale slot assumption; every
derived statistic — including a wind-gust correlation — had to be recomputed.

**Identify a driver by a message only that driver emits.** `UBX2` is emitted only by the
u-blox driver, so the instance carrying it is the u-blox unit. Never infer from the index.

**A device ID does not identify a physical part**, only the sensor die and where it sits.
A standalone QMC5883P breakout and a GPS module with the same compass built in report an
identical `COMPASS_DEV_ID`.

**Interpret a log against the parameter snapshot contemporaneous with that flight**, not
the current one.

---

## Telemetry logs

**Prefer `.bin` over a same-named `.log` or `.tlog`.** The text dataflash export decimates
or drops per-instance ESC telemetry; any per-motor RPM work needs the `.bin`.

**A `.tlog` can never characterise GPS2.** Only the first-enumerated GPS appears in
`GPS_RAW_INT` / `GLOBAL_POSITION_INT`. Dual-GPS work is dataflash-only.

**A stream group switched off does not appear in the tlog at all.** `MAV2_EXTRA1=0` removes
`ATTITUDE`/`RPM`/`ESC_TELEMETRY`; `MAV2_RAW_SENS=0` removes `RAW_IMU`/`SCALED_*`. This does
not affect the dataflash log or on-FC consumers — the harmonic notch reads ESC RPM straight
into `AP_ESC_Telem` independent of MAVLink.

**A GCS can silently rewrite your logging configuration.** Mission Planner rewrites MAV2
rates on connect unless Config → Planner → Telemetry Rates dropdowns are all `-1`. Do RC
calibration *before* setting them, since calibration forces its own rates and does not
restore them.

**A derived channel is not independent evidence.** `current_consumed` climbing proves
nothing about `current_battery` — it is that channel's own integral.

---

## Interpreting values

**`MOTB.FailFlags = 2` is the healthy value.** It is
`_thrust_boost | (_thrust_balanced << 1)`, so 2 means balanced and not boosting.

**`PM.Load` is percent × 10.** 203 is 20.3 %.

**`POWR.Vcc` reads NaN on boards without board-voltage sensing** (T-Motor H7 Mini, for
one). Not a fault.

**`VIBE.Clip*` are cumulative counters.** Take the delta over the window.

**`RATE.*` is deg/s, not rad/s.**

**Motor saturation ceiling is `SERVO_MIN + MOT_SPIN_MAX × (MAX − MIN)`**, not 2000 µs.
Compare peak output against that, and prefer a high percentile over the raw max — a single
sample touching the ceiling in a hard manoeuvre is not saturation.

**An unset RTC gives a 1980 log date.** The FC booted before GPS time was available. Real
date is in `GPS.GWk`/`GPS.GMS` or the `MSG` banner.

**Params disappearing between two captures is normal.** ArduPilot hides a disabled subtree
— all 14 `FFT_*` and 8 `INS_HNTC3_*` vanish when their enables go to 0. That is why one
capture had 1185 params and its predecessor 1207. Not data loss.

**A parameter's presence does not prove the behaviour reached the hardware.** `GPS_SAVE_CFG`
is inert on some modules; `GPS_NAVFILTER` was never transmitted at all under
`GPS_AUTO_CONFIG=0` because `_request_next_config()` returns early. Configured ≠ applied.

**`MOT_THST_HOVER` is relearned in flight** with `MOT_HOVER_LEARN=2`, so a param capture
goes stale the moment you fly. Anything pinned to it — notably `INS_HNTC3_REF` — drifts
with it. One flight moved it 0.3208 → 0.2626, leaving a pinned reference 23 % wrong.

---

## Notch and batch logging

**`FTN1.PkAvg` is the FFT's opinion; `FCNS.CF` is what the notch applied.** They are the
same series only when `INS_HNTCH_MODE=4`. Measuring the FFT to judge a change of notch
*source* is the wrong question — switching what consumes the FFT cannot change the FFT's
own accuracy.

**Averaging raw batch spectra smears a moving notch.** The centre tracks RPM, so the dip
spreads over ~20 Hz and vanishes. Normalise each batch by the notch centre the FC was
tracking at that instant.

**Discard batch windows with an `ISBD.seqno` gap.** Batch logging drops samples routinely;
concatenating across a hole produces peaks that are not real.

**A raw pre→post ratio overstates the notch.** `INS_GYRO_FILTER` sits after it and
attenuates everything above its corner. Isolate the notch by fitting a baseline outside the
notch bands.

**Batch logging roughly doubles the log rate.** On a board with onboard flash and no SD
card, erase first and set `INS_LOG_BAT_MASK` back to 0 afterwards. A flight was lost
entirely because the previous log had filled the chip.

**`INS_HNTCH_FREQ` is a floor, not a target.** If the motors never get near it in a given
flight, that flight neither justifies nor disproves the setting — say so rather than
implying it was validated.

---

## Reporting

**A check that could not run is not a pass.** Report "not logged" as `SKIP`.

**State the number, not the verdict.** "Vibration is fine" is useless next flight;
"VibeX p95 8.2 m/s², 2 clip events, 0 before" is a baseline.

**Report findings you were not asked about.** In one case a degraded motor balance was the
most important thing in the log, and nobody had asked about it.

**Correct visibly, do not overwrite.** Keep the superseded number and mark the amendment
with a date, so a reader can tell a correction from a fresh measurement.

**A closed investigation should record its next untried step**, so reopening never repeats
work. Distinguish closed-by-decision from closed-by-resolution.

**Finish by listing documentation drift.** A report that leaves a stale line in the
project's own notes has done half the job.

## `RCOU.C<n>` is a servo output, not a motor number

`RCOU` logs servo outputs 1..14. ArduPilot's motor numbering is *geometry* — on a quad X,
motor 1 is front-right, motor 2 rear-left, motor 3 front-left, motor 4 rear-right — and
which output drives which motor is *wiring*, declared by `SERVOn_FUNCTION` (33 = motor 1,
34 = motor 2, ... 36 = motor 4; 37-40 for motors 5-8). Many 4-in-1 AIO targets ship a
non-sequential default so the ESC connector matches Betaflight's motor order, e.g.

    SERVO1_FUNCTION=36  SERVO2_FUNCTION=33  SERVO3_FUNCTION=34  SERVO4_FUNCTION=35

so `C1` is motor 4 and the motors run `M1=C2, M2=C3, M3=C4, M4=C1`.

Projecting `C1..C4` straight onto the quad-X mix factors under that map permutes the axes:

| what the naive calculation prints | what it actually is |
|---|---|
| roll  | **yaw** |
| pitch | **−pitch** |
| yaw   | **−roll** |

A pure thrust asymmetry (roll/pitch) therefore appears as a large standing *yaw* torque,
which points the investigation at motor-mount rotation and arm twist instead of at CG,
blade thrust or prop matching. This cost a real investigation several flights.

Use `frames.motor_channels(params, n_motors)`; `check_motors` does, and prints the map it
used. Verify it independently on any new airframe by correlating each channel's
collective-removed deviation against `RATE.ROut`, `RATE.POut` and `RATE.YOut`: the two
channels that rise with `ROut` are the left pair, those that rise with `POut` are the
front pair, and those that rise with `YOut` are the CCW pair. All three groupings must
agree with the `SERVOn_FUNCTION` map.

The ESC telemetry instances follow the same output ordering: `ESC[i]` is servo output
`i+1`, so it needs the same map before per-motor RPM is attributed to a corner.

---

## Added September 2026

**A log opened at arming has no ARMED event.** With `LOG_DISARMED=0` the file is created
*at* arming, so `EV 10` is frequently written before the file exists. `ARM.ArmState` and the
throttle are the witnesses; the `flight` check uses them. "Never armed" on a log that
obviously flew is this, not a fault.

**`PM.I2CI`, `PM.I2CC` and `PM.SPIC` are transaction/interrupt counters, not error
counts.** A 350,000 reading is normal. Internal errors are `PM.ErC` (count), `PM.InE`
(mask) and `PM.ErrL` (line). An early version of this toolkit graded `I2CI` as errors.

**The text `.log` export is pre-scaled and lossy.** Values already have the `c/C/e/E/L`
scaling applied; `UNIT '-'` (empty label) is dropped by the exporter; `MODE.Mode` is a
string; `ISBD` arrays do not round-trip; per-instance ESC telemetry may be decimated. The
reader flags all of this as `TEXT_LOG`. Prefer the `.bin`.

**`airborne_window` used to know nothing about second flights** — three separate ad-hoc
constructions of "the airborne part", two that spanned the ground time between flights and
two that silently dropped every flight after the first. Corrected 2026-09-04; see the
worked example under "Windows and comparisons" above. Any analysis written before that
date against a multi-flight log should be re-run: its window, and therefore every number
in it, may have covered ground time or a single flight without saying so.

**`--window rpm` is only as good as its 90 Hz floor.** On a low-KV / large-prop aircraft
that cruises below 5400 RPM the fleet-mean fundamental spends most of the flight *under*
the floor: on `2026-08-06 18-21-35.bin` only 455 of 26,418 ESC samples clear it, in bursts
separated by 32 s and 55 s of sub-floor flight. The segmenter correctly refuses to call
that one contiguous flight, so `--window rpm` returns the longest burst (15 s) rather than
the 300 s min..max span it used to. Use `--window ev` on such an aircraft, and read
`alog info`'s flight table before trusting an `rpm` window.

**A 25 Hz IMU stream cannot show a 200 Hz motor.** The standard `LOG_BITMASK` logs IMU at
25 Hz and RATE at 10 Hz. An FFT of either is honest only below 12.5 Hz. `alog fft` says so
and exits 1; `coverage` grades the fastest gyro source against the motor fundamental.

**Irregularly sampled data must not be transformed.** A Welch PSD of samples with 30 %
interval jitter produces a plausible spectrum that is wrong. `alog fft` refuses above 5 %
jitter; resample explicitly and say so if you must.

**Post-filter batch instances are offset by the IMU count.** With `INS_LOG_BAT_OPT=4` a
one-IMU board logs `ISBH.instance` 0 (pre) and 1 (post). The unit char on `ISBD` is `o`
(m/s/s) even for gyro batches; use `ISBH.type`.

**FMT fields are not NUL-terminated when full.** Names of exactly 4, formats of exactly 16
and labels of exactly 64 characters fill the field. Decode to the first NUL *or the field
end*, never `strlen`.

**Never cache a failed parse.** A `.dfcache` written for a file that parsed to zero messages
would hide the real error on the next run. The parser writes a cache only after a
successful parse, and reports `CACHE_REBUILT` when it had to discard one.

**Windows consoles are not UTF-8 by default.** A report containing `µ` or `→` raises
`UnicodeEncodeError` half-way through unless stdout is reconfigured; `alog` does this.
