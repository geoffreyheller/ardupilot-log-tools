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
`log.field(msg, "BAlt", "BarAlt")` or `log.column(msg, name)`. Since issue #10 a missing
column raises `KeyError: no column 'HDOP' in GPS; did you mean 'HDop'? (columns: ...)` -
alias table, case-insensitive and prefix matches, difflib last - so the near-miss is named
where it fails rather than a turn later.

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
a two-flight log (`two-flights.bin`: 64.5-255.9 s and 285.3-360.4 s, 30 s of
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
the FC's land detector believed - provided its floor is right for the aircraft (see the
90 Hz entry under "Added September 2026"). `alog compare` now checks that its two windows
are the same kind of thing (same method, durations within 2x, same flight index, no
fallback) and says **NOT comparable** and exits 1 when they are not. Before that check,
and with `compare` defaulting to `rpm`, a large-prop pair compared a 15 s climb segment
against a 33 s one under a sentence saying they were comparable, and a motor-headroom PASS
flipped to FAIL on the window choice alone (issue #4).

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

**Level hover does not rule out wind.** Holding position in a steady breeze is still flying
through the air: the motors facing the breeze are loaded differently, and the trim reads
like a CG offset. `trim vs level hover` used to call equal figures "static asymmetry". On
2026-09-30 Brisket flew twice on the same pack with the battery never unstrapped. Level-hover
pitch/roll trim was 20.6/12.7 µs facing 178° (`brisket-t1`), then 35.2/0.2 µs facing 159° (`brisket-t2`)
in a light breeze, and the `cg` table moved from 3.75 % to 5.11 % forward. A 19° heading
change cannot rotate an airframe-fixed trim that far. `trim vs heading` fits airframe-fixed
plus earth-fixed parts per heading bin, and says when the hover faced too few headings to
separate them. Neither flight could.

**A narrow band cannot see the high-frequency lag.** Brisket's yaw was coherent over 0.5–3.5 Hz
only (brisket-t2). The fit pinned `tau2` at 1 ms and the delay at 0, while roll and pitch on the
same motors read 39 ms and 14 ms. Forcing those values in made the fit worse (22° vs 13.7°
phase), so the data simply does not constrain them. The virtual AutoTune then proposed yaw
P 0.18 → 0.48 at confidence 0.70 on a "36 dB" gain margin read at 21 Hz. The pooled run
refused the same axis. Margins read outside the band on pinned lag are now `SKIP ... not
measured`, and a rate set that relies on them is withheld.

**Two flights, same gains, different recommendations.** Re-identifying the same aircraft on
two flights moved the margin-ceiling P 3–4 % and the plant gain 4 %. The unchanged roll loop
moved 0.9° of PM. Recommendations inside ±5 % now read "no change", except while the current
loop misses the margins: there the −4.6 % pitch change was the correction, and it was flown
and measured (43.1° → 46.2°).

**A per-motor notch writes `FCN`, not `FCNS`.** With `INS_HNTCH_OPTS` bit 1 (one notch per
motor) ArduCopter logs `FCN` (`I NF CF1..CF6 HF1..HF6`) and no `FCNS`. The `notch` check
used to report a working notch as "enabled but not logged" and advise a logging change.
On Brisket `brisket-t1.bin`, `CFk` tracked ESC k-1's RPM/60 to 0.15 % median
(1.1 % p95). Since 2026-09-30 the check falls back to `FCN` and compares each centre with
its own ESC.

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

**`--window rpm` is only as good as its floor.** On a low-KV / large-prop aircraft
that cruises below 5400 RPM the fleet-mean fundamental spends most of the flight *under*
a fixed 90 Hz floor: on `brisket-l1.bin` only 455 of 26,418 ESC samples clear it,
in bursts separated by 32 s and 55 s of sub-floor flight, so the segmenter reported three
flights (five on a later log) where the EV land detector reported one. Fixed 2026-09-14
(issue #3): the floor is derived from the log as 60 % of the spinning median - ~50 Hz on
that aircraft, ~85 Hz on a 5-inch - and the method string names it. `--hz-floor` overrides
it. The `flight` check now warns when `ev` and `rpm` disagree on the flight count; that
disagreement is the signature of this failure, so read it before trusting either window.
The committed fixture `tests/fixtures/largeprop-quad.bin` pins the fix.

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

**An ESC's current-sense output does not see the avionics.** `BAT.Curr` reads ~0 with every
motor stopped while the flight controller, GPS, receiver and video are all powered — so any
comparison against a charger, a watt meter or a clamp is comparing two different boundaries.
This is what makes the wiki's charger-mAh calibration fail to converge: its error term scales
with total pack-connected time, which nobody records. Measure the idle draw in the same
configuration and subtract it. Full account, and the method that does work, in
`reference/current-sensor-calibration.md`.

**A current-sensor scale measured props-off is not the scale at hover.** Unloaded motor
current saturates at roughly a fifth of hover current, repeats to only ±20 %, and the shunt
amplifier is least linear down there — one aircraft measured an effective scale of 39.3 at
2.8 A and 35.9 at 4.8 A. Calibrate with props on at the motor-test percentage matching the
hover `RCOU` duty, where the thrust produced is just the aircraft's own weight.

**A subset cannot exceed the total.** If the flight controller reports more motor-path
current than a whole-aircraft meter measures, the flight controller over-reads. It needs no
model and no assumptions, and it is the fastest way to catch a bad `BATT_AMP_PERVLT`.

**Regressing pack voltage on current alone conflates sag with state of charge.** Voltage
falls as charge is consumed, not only as current rises, and over a flight that trend
dominates. Fit `V = OCV₀ − α·Q − R·I` instead: on one flight the naive slope was 40.0 mΩ and
the corrected one 33.8 mΩ, and the naive figure had been used to argue a sensor read 2× high.

---

## PID tuning (added September 2026)

**The standard `LOG_BITMASK` logs `PIDx`, `RATE` and `ATT` at 10 Hz.** 180222 — the value
on both development aircraft — is bits 1–13, 15 and 17 with **no bit 0** (ATTITUDE_FAST),
so the rate loop is logged at 10 Hz while its filters sit at 20–40 Hz. All 16 reference
logs in `LOG_DIR` / `LOG_DIR_LARGE` are refused `PID_RATE_TOO_LOW` for this reason, and
the Circuit log of 2026-09-03 was refused with `INS_LOG_BAT_MASK 1 -> 0` on the same
block: batch logging had been left on from a notch flight. 180223 adds bit 0; AutoTune
and SysID modes log at loop rate regardless of the bitmask. Fast logging roughly doubles
the log rate, so set bit 0 back on an onboard-flash board. The full table is `SKILLS.md`
Skill 7 and `tune.LOGGING_REQUIREMENTS`.

**`PIDx.Tar/Act/Err` are rad/s; `RATE` is deg/s.** `PIDx` logs the rate PID's own
values, and the rate PIDs work in rad/s. The FMTU declares no unit for them. On the
first fast-logged flight (Brisket, `brisket-t1.bin`, V4.7.0-dev 259b79c3)
`PIDR.Act × 57.2958` equals `RATE.R` sample for sample. The tool was first written
assuming deg/s, so its 20 deg/s excitation gate became 20 rad/s: that flight was refused
`NO_EXCITATION` on every axis, and the SRate ceiling cut P by 60 %. Both were artefacts.
Since 2026-09-30, `extract_axes` converts at extraction and `tests/tunesynth.py` writes
rad/s, as the firmware does.

**Firmware bookkeeping is not an in-flight tune change.** AP_Stats re-saves `STAT_*` every
~30 s, `MOT_THST_HOVER` is saved on disarm, and the ground pressure and gyro calibration are
rewritten around arming. On Brisket `brisket-t1.bin` these made 28 "in-flight
changes" and a WARN that the tune had changed mid-flight. `analysis.FIRMWARE_MAINTAINED`
lists them: `paramcheck` shows them with kind `firmware` and warns only on the rest.

**`PIDx.TimeUS` is the write time; `RATE`/`ANG` carry the loop start.** On the same
log, `RATE` and `ANG` sit at 2.500 ms ± 0.02 ms. `PIDR`/`PIDP`/`PIDY` have the same
111 597 records, but each is stamped 0.4–1.7 ms after its own `RATE` tick: 39 % jitter
by the `sample_rate` rule, with no record dropped. The jitter gate used to refuse it
`IRREGULAR_SAMPLING`. `extract_axes` now restamps PIDx to the `RATE` tick when every
record falls in a distinct, consecutive tick, and refuses as before when any tick is
skipped or doubled. `AxisSignals.clock` says `RATE` and `raw_jitter` keeps the original
figure.

**A P-only ceiling is not a ceiling on a loop with D.** Heli AutoTune sizes P at the
frequency where the plant phase is −161°, as if P were the only term. On the Brisket roll
plant (`brisket-t1.bin`) that put the P ceiling at 0.138, while the flown P 0.135
keeps PM 48.6° / GM 8.9 dB, because D's phase lead is ignored. Fusion then clipped the
virtual AutoTune's P 0.196 to 0.4 × 0.138 = 0.055 (−59 %, PM 86°) at confidence 0.70.
The clip was discontinuous (0.137 would have stood, 0.139 became 0.055), and the 0.196 it
started from gives PM 34°. Since 2026-09-30 the ceiling is the largest P or D whose full
`C(z) G` loop keeps 45° / 6 dB (`tune_ident.margin_ceilings`). A value above it is clipped
*to* it. 0.4 × is kept for oscillation ceilings only. The applied P/I/D set is re-checked
and withheld if it fails. Angle P found on an unclipped rate loop is withheld too. On
that log: roll P 0.153 / D 0.0044, pitch P 0.129 / D 0.0035, both at PM 45°.

**`ATT` is 10 Hz; `ANG` is the loop-rate attitude.** `ATT` is written by AHRS at 10 Hz on
every firmware, master included; `ANG` is written at loop rate with bit 0, and its
`DesRoll` is the *shaped* (jerk-limited) target, not the raw stick. Older firmware has
only `ATT`. A step response or a target-vs-actual comparison taken from `ATT` is a 10 Hz
picture of a 400 Hz loop. The tool prefers `ANG` and states which it read.

**`ATC_ACCEL_x_MAX` is cdeg/s²; `ATC_ACC_x_MAX` is deg/s².** The 4.x and master
spellings differ by 100×. The Circuit quad (V4.7.1 dbe79216) already lacks
`ATC_ACCEL_R_MAX`; the Brisket quad (V4.7.0-dev 259b79c3) has it at 116700 — which is
the Mission Planner calculator's 10-inch value, 1167 deg/s² in the other spelling. Read
both, convert once, and say which was found (`GainSet.param_names`); the recommendation
is written back in the log's own spelling and unit.

**`ATUN.ddt` is unscaled cdeg/s² despite its unit tag.** Divide by 100 for deg/s². It is
each twitch's own peak acceleration, overwritten per test, not a running maximum; the
`MSG` line `Max Accel:` is cdeg/s² on every branch too. This is the source of
`ATC_ACC_x_MAX` in a reconstruction.

**A hover-only log gives high coherence and a biased plant.** With no stick input the
reference `PIDx.Tar` is the angle loop reacting to gyro noise, not an exogenous input:
on a 60 s simulated hover the mean coherence over 0.5–40 Hz read ~1.0 and the plant
estimate is biased toward `−1/C`. Coherence is necessary, not sufficient — the bins just
above a chirp's stop frequency are coherent through window leakage and off by more than
2 dB. Two consequences: `identify()` gates on excitation *before* coherence, so a hover
is `NO_EXCITATION` rather than a confident wrong model; and the AnalyticTune chirp of
0.05–5 Hz, which gave a coherent band of 0.5–7.5 Hz against a 23 Hz rate-loop crossover
on the simulated 5-inch plant, is `NO_COHERENCE` by design. `SID_F_STOP_HZ 40`.

**AutoTune's overshoot and bounce criteria apply to its test gains, not the final loop.**
A twitch flies with `I = 0.01 P`, `FLTT 0`, `FF 0` and the gains *before* the backoff;
the step response deconvolved from a log is the final closed loop with I active and the
backed-off gains. The exact oracle step of AutoTune's own result on the simulated plant
reads `overshoot_ratio` 2.4 and `bounce_ratio` 1.9 — "fail" by a literal reading of the
0.5 × AGGR / AGGR criteria — and the deconvolved curve reads 6–10 and 2.5–4.5. Until the
ratios are calibrated on a real fast-logged flight the `step-rules` confidence is capped
at 0.39 (always withheld), the ratios are reported beside their thresholds but not graded,
and the step rules never set another tier's agreement. Read them as direction only.
