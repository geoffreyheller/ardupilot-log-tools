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
