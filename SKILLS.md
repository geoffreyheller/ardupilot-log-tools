# SKILLS.md — analysis recipes

Each skill below is one question an operator or an agent actually asks of a log, with the
commands that answer it, the lines of output to read, the decision they support, and the
flight that validates the decision. `RULES.md` is the contract these rest on and
`CLAUDE.md` the long-form guide; this file is the short path from a question to a number.

Every command is `python alog.py …` (or `alog …` after `pip install -e .`). Add `--json`
for the machine-readable form of any of them. Exit codes: 0 pass, 1 warn, 2 fail, 3 the
input could not be analysed.

Always start with **Skill 0**. Then pick the skill that matches the question.

---

## Skill 0 — Triage a new log

**Question:** what is this file, is it intact, and what can it tell me?

```bash
python alog.py info flight.bin
```

Read, in order:

1. **Integrity block.** `RESYNC` or `DUPLICATE_DATA` change the meaning of every later
   number; `TRUNCATED_TAIL` after a landing is normal. Codes: `reference/integrity-codes.md`.
2. **Flights in log.** One line per flight. More than one means every later command
   analyses ONE of them unless you pass `--flight N` or `--flight all` (Skill 2).
3. **Coverage table.** A message that is `absent` makes its check `SKIP` — never a pass.
   The `spectral reach` line says whether any spectrum from this log can show the motor.
4. **UTC start.** A 1980 date is an unset RTC, not a corrupt log.

Then the standard battery:

```bash
python alog.py all flight.bin            # markdown report
python alog.py all flight.bin --json     # one JSON document for an agent
```

The verdict is at the end: counts, the WARN/FAIL findings ranked by severity, and the
list of skipped checks. Report skips as skips. Delete the `.dfcache` files when done.

---

## Skill 1 — Handle a log that holds more than one flight

**Question:** which flight are these numbers about?

```bash
python alog.py info flight.bin                 # lists every flight
python alog.py all  flight.bin --flight 2      # one of them
python alog.py all  flight.bin --flight all    # every one, one block each, one verdict
```

The window's method string names the flight (`…, flight 2 of 3`). Analysing one flight of
several is a `WARN` (`flights in log`) so a report can never hide that a flight was left
out; `--flight all` clears it. `dump`, `fft` and `compare` reject `--flight all`.

If the `flight` section shows **`flight detectors disagree`**, the EV land detector and
the ESC-RPM detector count different numbers of flights. On a large-prop aircraft that
means the RPM floor is wrong — see Skill 2; if the motors kept spinning through a landing,
trust EV.

---

## Skill 2 — Pick the window

**Question:** over which seconds should this statistic be taken?

| you want | use | why |
|---|---|---|
| a general report | `--window auto` (default) | EV land detector first, then ESC RPM, throttle, arm |
| motor, notch, vibration numbers | `--window rpm` | defined identically regardless of what the land detector believed; floor derived from the log (60 % of the spinning median), `--hz-floor HZ` overrides |
| a statistic only meaningful in steady hover (FFT peak, motor balance, vibration baseline) | `alog hover flight.bin` then `--window hover` (longest chunk) or `--window hover:N` | LOITER/ALT_HOLD/POSHOLD with sticks centred, ≥ 10 s, clipped to one flight |
| a manoeuvre, a GPS outage, a specific minute | `--window 120:180` | explicit seconds since boot |
| takeoff/landing transients removed | add `--pad 5` | trims 5 s off each end |
| ground time included | `--window arm` or `--window none` | only when that is what you want |

Quote the method string in every number you report. If it says `FALLBACK`, the method
you asked for could not be applied and the whole log was used.

---

## Skill 3 — Motor balance, CG, bent prop, bad bearing

**Question:** why do the motors disagree, and what do I move or replace?

```bash
python alog.py motors flight.bin --window rpm --arm-mm 151    # 151 = CG to front motor line, mm
```

Read these, in order:

1. **Channel → motor map** note. `RCOU.C<n>` is servo output n, not motor n; the check
   applies `SERVOn_FUNCTION`. If it says `ASSUMED`, the axis labels are unverified.
2. **`trim` table** — roll / pitch / yaw in µs:
   - roll or pitch trim → **thrust** asymmetry: CG, a damaged blade, mount height, wind.
   - yaw trim → **torque** asymmetry: mount rotation, arm twist, a blade whose airfoil is wrong.
   - large `residual` → not a trim at all; a failing motor or a bad RPM channel.
3. **`cg` table** — the trim as a CG offset, % of arm and mm (with `--arm-mm`). Positive
   pitch = CG forward of the motor-line centre; that is the distance to move the battery.
4. **`trim vs level hover`** — the same trim over hover chunks with |roll|, |pitch| < 3°.
   Different → a translation artefact; do not move anything on that evidence. Same figure
   → a CG/airframe asymmetry **or a steady breeze**, which level hover cannot tell apart.
5. **`trim vs heading`** — the level-hover trim per 45° heading bin, fitted as an
   airframe-fixed part (CG, blade, mount: move the battery / fix the blade) plus an
   earth-fixed part (the breeze, with the bearing of the side it loads). SKIP when the hover
   faced fewer than three bins spanning 90°: then the `cg` table is CG plus wind in unknown
   proportion. To measure CG, hover ~20 s facing each of N, E, S, W, sticks centred.
6. **`drive-normalised RPM spread`** — `RPM / (duty × V)` per motor. A CG offset leaves it
   flat while raw RPM spread reads 10 %; a dragging motor (bearing, damaged blade) drops
   it and nothing else shows it. The summary names the lowest motor.
7. **ESC temperature** and **spread** — one hot ESC in a set is a finding on its own.
8. **RPM spread** on medians, **DShot error rate**, **motor headroom** against the
   `MOT_SPIN_MAX` ceiling (p99.5, not the raw max).

**Validation flight:** one change (move the battery by the mm figure, or swap the named
prop), fly the same profile, run Skill 10 (`compare`) and expect the pitch trim and CG
offset to fall while `trim vs level hover` stays PASS. A trim in the same window that did
not move means the change was not the cause.

---

## Skill 4 — Is the harmonic notch working? Where should it be?

**Question:** is the notch tracking the motors, and by how much does it attenuate them?

```bash
python alog.py notch    flight.bin --window rpm
python alog.py batchfft flight.bin --window rpm      # needs INS_LOG_BAT_MASK=1, OPT=4
python plot_notch.py    flight.bin -o notch.png
```

Three cases the `notch` section distinguishes:

- **Enabled and `FCNS` logged** — read `notch N tracking` (p95 of `FCNS.CF / (ESC.RPM/60)`
  − 1; a correct notch reads ~0.014) and `harmonic lock-on` (% of time above 1.5× the
  fundamental, i.e. the notch sitting on the second harmonic). Ground truth is always
  `ESC.RPM / 60`; `FTN1.PkAvg` is the FFT's opinion and only matters when `MODE=4`.
- **Per-motor notch (`INS_HNTCH_OPTS` bit 1)** — the firmware writes `FCN.CF1..CFn`, one centre
  per ESC, and no `FCNS`. Each centre is checked against its own ESC's RPM/60 and
  `notch N tracking` grades the worst of them, naming it.
- **Enabled but neither `FCNS` nor `FCN` logged** — SKIP; fix the logging, not the configuration.
- **Disabled** — SKIP, plus the **measured fundamental envelope** (min / p01 / median /
  p99 / max, per-motor medians) and a **starting point** table: `MODE=3` if the ESC
  telemetry is good, `REF=1`, `FREQ` just under the airborne p01, `BW = FREQ/2`,
  `HMNCS=3`, `ATT=40`, `OPTS=2` when the motors spread more than 5 %. Do not re-derive
  these; they are in the report.

**The proof is `batchfft`.** It discards any batch with an `ISBD.seqno` hole, normalises
each batch by the notch centre the FC was tracking at that instant, and reports the
attenuation at the fundamental and where the deepest dip sits in motor orders (target
1.000 and 2.000). The post/pre ratio includes `INS_GYRO_FILTER`; the caveat is printed.

**Validation flight:** set the parameters, `INS_LOG_BAT_MASK=1`, `INS_LOG_BAT_OPT=4`, fly
30–60 s of hover, run `batchfft`, then **set the mask back to 0** — batch logging roughly
doubles the log rate and fills onboard flash.

---

## Skill 5 — Vibration and spectra

**Question:** is vibration acceptable, and what frequency is it at?

```bash
python alog.py vibe     flight.bin --window hover     # VIBE p95 and clip counts
python alog.py fft      flight.bin --list-sources     # what can be transformed, with Nyquist
python alog.py fft      flight.bin --window hover --plot fft.png
python alog.py spectrum flight.bin --window rpm       # the same, as a report section
```

- **Clip events** are the hard failure: the accelerometer saturated and the EKF was fed
  garbage. Any non-zero count is a finding.
- VibeX/Y p95 under 15 m/s² is good, over 30 a problem; Z tolerates a little more.
- The FFT states its source, sample rate and Nyquist, and labels peaks in **motor orders**
  when ESC telemetry exists (`order 1.98` = the second harmonic). It refuses irregular
  sampling rather than transforming it, and exits 1 when the Nyquist is below the motor
  fundamental — a 25 Hz IMU stream cannot show a 200 Hz motor. Batch logging
  (`INS_LOG_BAT_MASK`) or raw logging (`INS_RAW_LOG_OPT`) is the fix.
- Take vibration baselines over a hover chunk (Skill 2); whole-flight figures mix hover
  with manoeuvring and are not comparable between flights.

---

## Skill 6 — Assess the tune (before touching a gain)

**Question:** is this a gain problem or a filter problem?

```bash
python alog.py pid  flight.bin --window rpm
python alog.py gust flight.bin --window rpm
```

- The rate table splits desired-vs-actual error at **5 Hz**: low-band error is a **gain**
  problem, high-band error is gyro noise reaching the controller — a **filter** problem.
  Do the filter work (Skill 4) before touching a rate gain.
- `Dmod` at 1.000 means the D-term slew limiter never engaged: no oscillation onset
  anywhere in the flight. Below it, the tune is backing itself off.
- `gust` reports unrequested attitude excursions per second (|actual − desired| > 2.5°
  while |desired| < 1°). Check motor headroom and clipping first; if both are fine, it is
  the controller.
- Attitude-error standard deviations rise ~20 % on a livelier flight with no change to
  the tune. Compare like with like (Skill 2, Skill 10).

---

## Skill 7 — Recommend PID gains (and capture a log that can)

**Question:** what should `ATC_RAT_x_P/I/D`, `ATC_ANG_x_P`, `ATC_ACC_x_MAX` (and the yaw
`FLTE`) be, how sure is that, and what do I have to fly to find out?

Almost every log fails this skill at step 0. The standard `LOG_BITMASK` (180222 on both
development aircraft) writes `PIDR/PIDP/PIDY`, `RATE` and `ATT` at **10 Hz**, and a 10 Hz
stream cannot show a rate loop whose filters sit at 20–40 Hz. Such a log is refused with
the `ERROR:` block shown at the end of this skill, and nothing in that output is a gain.

**AutoTune is not required.** The point of this skill is to get gains from an ordinary
fast-logged flight: the virtual AutoTune (tier B) identifies the rate-loop plant from the
pilot's own stick inputs and runs the firmware's twitch search on it in software, and the
step-response tier (C) supplies ceilings. A log that happens to hold an AutoTune session is
read too (tier A), but nothing below asks you to fly one.

### Before you fly: set these parameters

The rows are `tune.LOGGING_REQUIREMENTS` verbatim (`tests/test_tune_docs.py` compares
them); every refusal prints the same table with the aircraft's own values filled in.

| parameter | required for a tuning log | why | tier |
|---|---|---|---|
| `LOG_BITMASK` | bits 0 (1) and 12 (4096) set | bit 0 ATTITUDE_FAST + bit 12 PID: loop-rate RATE/PID logging | B, C |
| `LOG_FILE_RATEMAX` | 0 | a non-zero cap decimates the fast stream back down (0 or >= SCHED_LOOP_RATE) | B, C |
| `LOG_BLK_RATEMAX` | 0 | same cap for the block backend (onboard-flash boards) | B, C |
| `INS_LOG_BAT_MASK` | 0 | batch logging doubles the write rate; separate flight | B, C |
| `LOG_FILE_BUFSIZE` | >= 64 | buffer for the doubled rate, KB; larger on boards that show LOG_GAP | B, C |
| `LOG_DISARMED` | any | irrelevant; the window is airborne only | - |
| `SCHED_LOOP_RATE` | as flown (400 default) | reported; sets the fast-log rate | - |
| `AUTOTUNE_AXES` | any | flying AutoTune is NOT required; read only when the log happens to hold a session (tier A) | A |
| `AUTOTUNE_AGGR` | as configured (0.075 default) | no AutoTune flight needed: the virtual AutoTune (tier B) applies this aggressiveness to an ordinary flight; --aggr overrides | A, B |
| `AUTOTUNE_MIN_D` | as configured (0.0005 default) | no AutoTune flight needed: the virtual AutoTune (tier B) uses this floor for D | A, B |
| `SID_AXIS` | 10/11/12 | optional plant-identification flight: SIDD output plus RATE.xOut input (mixer injection) | B |
| `SID_MAGNITUDE` | 0.15 (yaw 0.55) | optional plant-identification flight | B |
| `SID_F_START_HZ` | 0.5 | optional plant-identification flight (firmware default) | B |
| `SID_F_STOP_HZ` | 40 | the sweep must pass the rate-loop crossover (4.5-5 Hz measured on a 10-inch quad, higher on smaller props); AnalyticTune's 5 Hz stop is for the attitude loop | B |
| `SID_T_REC` | 70 | optional plant-identification flight (firmware default) | B |
| `SID_T_FADE_IN` | 15 | optional plant-identification flight (firmware default) | B |
| `SID_T_FADE_OUT` | 2 | optional plant-identification flight (firmware default) | B |

Tiers: **A** AutoTune reconstruction from `ATUN`, **B** plant identification plus a
virtual AutoTune, **C** step response plus oscillation ceiling (all three below). The
row that matters is `LOG_BITMASK`: 180222 → 180223 sets bit 0. Fast logging roughly
doubles the log rate, so on a board with onboard flash and no SD card set bit 0 back after
the tuning flight, exactly as for batch logging (Skill 4).

### The tuning flight

- **Data-acquisition profile** (tiers B and C): bit 0 on, `INS_LOG_BAT_MASK 0`, take off
  in ALT_HOLD, 30 s hover, then 60 s of sharp roll stick inputs (±15–20°, quick release),
  60 s pitch, 30 s yaw, land. 3–5 min. One gain set per flight: a parameter change in the
  air splits the log at that instant, and if no piece is 10 s long the axis is refused
  (`GAINS_CHANGED_IN_FLIGHT`).
- **Plant-identification profile** (raises tier B confidence): SysID mode, `SID_AXIS 10`
  (then 11, 12 on later flights), `SID_MAGNITUDE 0.15` (yaw 0.55), `SID_F_START_HZ 0.5`,
  `SID_F_STOP_HZ 40`, `SID_T_REC 70`, `SID_T_FADE_IN 15`, `SID_T_FADE_OUT 2` — the
  firmware defaults — one axis per flight. The sweep must pass the rate-loop crossover,
  which sat at 23 Hz on the simulated 5-inch plant: AnalyticTune's 0.05–5 Hz recipe is for
  the attitude loop and yields `NO_COHERENCE` on the rate loop by design.
- **A hover with no stick input is not a tuning log.** Coherence reads ~1.0 because the
  reference is the angle loop reacting to noise, and the plant estimate is biased toward
  −1/C. The tool refuses it on excitation (`NO_EXCITATION`), not on coherence.

### Commands

```bash
python alog.py tune flight.bin                        # one log
python alog.py tune before.bin after.bin              # several logs of one aircraft: plants pool, every gain set is reported
python alog.py tune flight.bin --axes roll,pitch      # a subset of axes
python alog.py tune flight.bin --aggr 0.075           # override the log's AUTOTUNE_AGGR for the virtual AutoTune (noted as override)
python alog.py tune flight.bin --prop-in 10 --json    # add the Mission Planner initial-parameter comparison (`calculator` table)
python alog.py all  flight.bin                        # the same analysis as the `tune` section, after `gust`
```

`--window`, `--pad`, `--flight N` and `--hz-floor` work as everywhere; `--flight all` is
rejected, as `compare` rejects it. `alog tune` exits 3 when no axis has any tier A/B/C
evidence (and on `DIFFERENT_AIRCRAFT`); in `alog all` the same refusal is one `SKIP`
result named `refused` — never a pass — and the exit code is unaffected.

### Read these, in order

1. **The `ERROR:` block.** If the output begins `ERROR: these log files cannot be used
   for PID tuning.`, read the code on each line and the parameter table under it, which
   is the table above with the log's values (`180222 -> 180223`, `1 -> 0`, `ok`, `not in
   log`). When only some axes are unusable the block is absent; the codes are in the
   **`refusals`** table and each such axis has a WARN `<axis> insufficient data`. The
   codes (`tune.REFUSAL_CODES`):
   - `NO_PID_MESSAGES` — no `PIDx` and no `RATE` in the window: set bits 0 and 12.
   - `PID_RATE_TOO_LOW` — the stream is below 100 Hz (200 Hz recommended): the 10 Hz
     standard; the fix line names the exact new `LOG_BITMASK`.
   - `IRREGULAR_SAMPLING` — timing jitter above 5 %, or `LOG_GAP`s cutting every piece
     below 10 s: the logger stalled (SD card, `LOG_FILE_BUFSIZE`, batch logging on the
     same flight). Not transformed, not resampled.
   - `WINDOW_TOO_SHORT` — the window is shorter than the 10 s minimum segment.
   - `NO_EXCITATION` — fewer than 10 deconvolution frames with |target| ≥ 20 deg/s: fly
     the profile above.
   - `OUTPUT_SATURATED` — `PIDx.Flags` bit 0 on more than 20 % of samples: the loop is
     nonlinear there; hover throttle and motor headroom first (Skill 3, Skill 8). The
     signals are still analysed; read this as a caveat on everything below.
   - `GAINS_CHANGED_IN_FLIGHT` — no parameter-constant piece of 10 s: one gain set per
     flight.
   - `DIFFERENT_AIRCRAFT` — frame class/type, board or MCU id differ between the logs.
   - `NO_COHERENCE` (tier B only) — the coherent band is empty or stops below the loop
     crossover: SysID sweep to 40 Hz, or sharper stick inputs; tier C is still reported.
   - `AUTOTUNE_INCOMPLETE` (tier A only, informational) — a session in the log ended
     before `TUNE_COMPLETE` on that axis, so tier A has nothing for it. Tiers B and C from
     an ordinary fast-logged flight do not need a session; rerun AutoTune only if you
     want tier A.
2. **`recommendations` table** — `axis | param | current | recommended | change % |
   confidence | method | why`, one row per parameter, and a result line `<axis> <param>`
   per row: PASS at confidence ≥ 0.7, WARN "validate before applying" at 0.4–0.7, WARN
   with **`withheld`** in the recommended column (and `-` for change) below 0.4 — the
   value then exists only as `evidence.withheld_value`. The **method** says where the
   number came from:
   - `autotune-log` — tier A: the gains the firmware found, re-derived from every `ATUN`
     twitch with the branch's own backoff (`AUTOTUNE_GMBK` on 4.7+, fixed on 4.3–4.6) and
     checked against the `MSG` lines and the saved `PARM`. Median over sessions, spread
     graded by `tune_session_spread`. Prior 1.0.
   - `virtual-autotune` — tier B: AutoTune's twitch search run in software on the
     rate-loop plant identified from this log (`PIDx.Tar/Act` and the plant input, joint
     I/O estimate gated by coherence), with the log's `AUTOTUNE_AGGR`/`GMBK`/`MIN_D` and
     hover throttle. Prior 0.8.
   - `ceiling` — a measured oscillation ceiling (`SRate` p95 above `QUIK_OSC_SMAX` 5, a
     limit cycle in the D term or the tracking error, or the heli −161°/−251° phase rule
     on the plant) times QuickTune's 0.4 margin; also the method of any higher-tier value
     that was clipped to a ceiling (the note says so). Prior 0.7.
   - `step-rules` — tier C: bounded ±25 % adjustments from the deconvolved step response.
     **Uncalibrated**: AutoTune's overshoot and bounce criteria are written for a twitch
     flown with *test* gains (I ≈ 0, FLTT 0, before the backoff), while the log's step is
     the final loop with I active — even the exact step of AutoTune's own result reads
     `overshoot_ratio` 2.4 and `bounce_ratio` 1.9. The confidence is capped at 0.39, so a
     `step-rules` row is always `withheld` and its note starts "uncalibrated". Its ratios
     are evidence of direction, not a value to apply.
   - `unchanged` — not a recommendation: `ATC_ACC_x_MAX` is left as configured because
     the proposing tier was not A/B at ≥ 0.7, or the tier's value is inside the **±5 %
     deadband** (`fuse_deadband_pct`: the same aircraft and gains re-identified on two flights
     moved 3–4 %). The note starts `no change:` and names the tier's value, which is also in
     `evidence.deadband_value`. Rate gains are not deadbanded while the current loop misses
     45° / 6 dB — there a small change is the correction.
   A rate set is also `withheld` when its margins are **not measured**: the crossover lies
   above the coherent band, or the gain margin is read above it on a fit whose `tau2`/delay
   sit at their bounds. The note says which and asks for a SysID sweep on that axis.
   `I` follows `P` by AutoTune's ratio (1.0 roll/pitch, 0.1 yaw) unless the log's own
   I/P was deliberately different, which is preserved and said. Yaw gets `P`, `I` and
   `FLTE` only (and `D` only when `AUTOTUNE_AXES` bit 8 was set).
3. **The confidence components** on each result line: `prior × adequacy × excitation ×
   consistency × agreement`. *Adequacy* is the sample rate against 100/200 Hz times the
   unsaturated fraction; *excitation* is frames against 30 (tier C), coherence × band
   cover (tier B), or 1.0 / 0.3 for a complete / partial AutoTune axis (tier A);
   *consistency* is the per-frame step spread, the Bendat–Piersol error of |G| at the
   crossover, or 1 − session spread; *agreement* is 1 − |Δ|/max when another tier
   proposed a value (floored at 0.5; `step-rules` never sets it). A low number is
   auditable from these five, and `--json` carries them under `components`.
4. **`ceilings`** — `axis | param | ceiling | method | clips to | evidence`. Hard
   upper bounds, whatever the tier. `margin-45deg-6dB` is the largest P or D whose full
   loop keeps the margins on the identified plant: a value above it is clipped *to* it. An
   oscillation ceiling (SRate, limit cycle) is where the loop was seen to oscillate: a value
   above it is clipped to 0.4 × ceiling. The fused P/I/D set is then re-checked and withheld
   (`the rate set as applied ... below the limits`) if it fails. The heli AutoTune -161°/-251°
   figures are printed as evidence only: they are P-only rules that ignore D's phase lead. "No ceiling found" is a note and a fact, not a pass.
5. **`step`** — one row per gain set per log: frames (or the refusal code), latency, rise,
   peak, `overshoot ratio` and `bounce ratio` with `warn 1, fail 2` beside them, settling,
   steady state, consistency, `SRate p95`, ceiling yes/no. The ratios are **not graded**
   (item 2, `step-rules`); read `SRate p95` against 5 and `ceiling` first.
6. **`plant`, `margins`** — `k`, `τ1`, `τ2`, delay, fit residual in dB and degrees, the
   coherent band and mean coherence, the Bendat–Piersol error at the crossover; then the
   open-loop crossover, phase margin and gain margin on the fit and measured in band.
   Results `<axis> gain margin` / `<axis> phase margin` are graded at 6 dB / 45°
   (`tune_gain_margin_db`, `tune_phase_margin_deg`), or `SKIP ... not measured` when read
   outside the coherent band on an unidentified high-frequency lag. Tier B is used only when the band
   contains the crossover and the fit residual is below 3 dB; the note says when it was
   not, and why.
7. **`autotune`** — per session and axis: the completed steps, the twitch outcomes
   re-derived from `Targ/Min/Max` with the log's AGGR, the backoff assumed and the backoff
   *observed* in the log (`RP` of the ANGLE rows ÷ the last RATE_P_UP row), and the
   agreement with `MSG`. A `<axis> AutoTune reconstruction vs MSG` FAIL means the
   reconstruction and the firmware disagree by more than 1 %: say which is more likely
   wrong from the version before using the number.
8. **`params`** (and **`calculator`** with `--prop-in`) — tier D, always present: `<axis>
   I/P ratio` against AutoTune's final ratio, `<axis> FLTD` / `FLTT` against
   `INS_GYRO_FILTER/2`, `D/P ratio` (reported, not graded), `AC_PID ranges`, `SMAX`,
   `ATC_RATE_FF_ENAB`, and the configured gain set per axis with its `defaulted` column.
   Tier D never produces a gain; it tells you whether the set is self-consistent.
9. **`logs`** — which tiers each log contributed (`A`, `B`, `C`, `D`) and its refusal
   codes, so a reader sees which log carried the evidence.
10. **Notes** — the "Parameters NOT in the log, defaults assumed" line (every number that
    depends on one is conditional), each tier's own description, and the validation
    flights.

**Decision rule.** Apply a value only at confidence ≥ 0.7. Between 0.4 and 0.7 fly it as
a validation, not a change: apply, fly the profile, `alog tune before.bin after.bin`,
and keep it only if the numbers below moved the right way. Never apply a withheld value —
it is in the evidence so the next analysis can compare against it, not for the aircraft.
One axis per flight, one gain set per flight, and `I` moves with `P`.

**Validation flight:** apply the recommended set for one axis, fly the tuning profile
again, then `alog tune before.bin after.bin` — both gain sets appear in `step`, the
plants pool. Expect `overshoot ratio` and `bounce ratio` to move toward 1.0 (direction,
not value — they are uncalibrated), gain and phase margin to stay above 6 dB / 45°,
`SRate p95` below 5 and `Dmod` at 1.000 (Skill 6). Run Skill 10 (`compare`) for
everything else: a tune that improved the step and worsened the gust rate is not done.

**Worked example — the block a standard log produces.** The Circuit quad's
`circuit-batch.bin` (ArduCopter V4.7.1, `LOG_BITMASK` 180222, batch logging left
on from a notch flight), `tune` section of `alog all`; `alog tune` prints the same block
first and appends `exit code 3`:

```
ERROR: these log files cannot be used for PID tuning.
  circuit-batch.bin: PID_RATE_TOO_LOW - PIDR logged at 10.0 Hz over 77.0-239.1 s; 100 Hz needed (200 Hz recommended) [roll, pitch, yaw]
Set these ArduCopter parameters and fly the tuning profile (SKILLS.md, 'Recommend PID gains'):
  LOG_BITMASK       180222 -> 180223  (bit 0 ATTITUDE_FAST + bit 12 PID: loop-rate RATE/PID logging)
  LOG_FILE_RATEMAX  0  ok
  LOG_BLK_RATEMAX   0  ok
  INS_LOG_BAT_MASK  1 -> 0  (batch logging doubles the write rate; separate flight)
  LOG_FILE_BUFSIZE  172  ok
  LOG_DISARMED      0  ok
  SCHED_LOOP_RATE   400  ok
  AUTOTUNE_AXES     4  ok
  AUTOTUNE_AGGR     0.075  ok
  AUTOTUNE_MIN_D    0.0005  ok
  SID_AXIS          0  ok
  SID_MAGNITUDE     not in log
  SID_F_START_HZ    not in log
  SID_F_STOP_HZ     not in log
  SID_T_REC         not in log
  SID_T_FADE_IN     not in log
  SID_T_FADE_OUT    not in log
```

Two parameters to change (`LOG_BITMASK`, `INS_LOG_BAT_MASK`), the rest confirmed. The
tier-D lines above the block in that report were all PASS (I/P 1.000, FLTD/FLTT 37.5 Hz =
`INS_GYRO_FILTER`/2, yaw I/P 0.100) — a self-consistent AutoTune set, which is still not a
gain recommendation.

---

## Skill 8 — GPS quality, an antenna or ground-plane change, two receivers

**Question:** is the GPS working, and better than before?

```bash
python alog.py gps flight.bin
python alog.py compare before.bin after.bin --checks gps
```

- **`HDOP` is geometry; `GPA.HAcc` is the receiver's own accuracy estimate in metres**,
  and it is the number that answers "better than before". The `gpa` table carries HAcc /
  VAcc / SAcc median and p95 per receiver, `VDop`, the fix interval (`GPA.Delta`: 200 ms
  = 5 Hz) and the 3D-fix sample count. Fail levels are the EKF's own limits (5 m / 7.5 m /
  1.0 m/s).
- A receiver with no 3D fix, or an NMEA unit whose driver reports no estimate (HAcc 0,
  VDop 655.35), is SKIP — 0 m is an absence, not an accuracy.
- **Identify a u-blox by `UBX2`**, never by instance index; the index follows SERIAL port
  order and changes between parameter snapshots.
- With two receivers the *differential* is the diagnostic: both degraded → a shared
  emitter; one much worse → that unit.
- `position jumps` is the implied ground speed between fixes; `GPS glitch (ERR)` is the
  FC's own verdict.

**Validation flight:** same site, same time of day if you can, one change, then `compare`
on `--checks gps` and read HAcc median before/after.

---

## Skill 9 — Current sensor: is it lying, and by how much?

**Question:** what does the sensor read when nothing is drawing, and what does hover cost?

```bash
python alog.py power flight.bin
```

- **`BATn current with motors stopped`** — `BAT.Curr` over every sample where each ESC
  reports RPM 0 and every motor output sits at `SERVO_MIN`. A healthy sensor reads 0.0 A;
  one aircraft read 12.3 A, which put every figure in the flight that much high. It is
  measured outside the airborne window by definition, and the summary says how many
  samples came before and after it.
- **`bat0_bands`** — current at idle (armed on the ground), hover (`ThH` ± 0.04) and full
  throttle (`ThO` ≥ 0.95), raw and offset-corrected, with **hover watts** — the number
  people compare between builds.
- **`current sensing`** — `corr(throttle, current)`. A flat reading is a wiring or pin
  fault, not a calibration error: a wrong `BATT_AMP_PERVLT` changes the magnitude, never
  the correlation.
- **Consumed** is printed raw and offset-corrected.

**Fix order:** `BATT_AMP_OFFSET` first (from the motors-stopped reading), then
`BATT_AMP_PERVLT` from a charger cross-check:
`PERVLT_new = PERVLT_old × (charger mAh / logged mAh)`. A scale correction on top of an
offset is wrong at every current but one.

---

## Skill 10 — Before/after comparison (one change per flight)

**Question:** did the change help?

```bash
python alog.py compare before.bin after.bin
python alog.py compare before.bin after.bin --checks motors,notch --window rpm
python alog.py compare before.bin after.bin --flight 1 --window ev
```

- Both logs run through identical code. `compare` then checks the two windows are the same
  kind of thing — same method, durations within 2×, same flight index, no fallback — and
  says **NOT comparable** (exit 1, `comparable: false` in JSON) when they are not. Do not
  read the table until it stops saying that; pass `--window` and `--flight` explicitly.
- One change per flight. A diagnostic is only trustworthy when one thing moved.
- Read the numbers, not the statuses: a PASS that moved 3× is more interesting than a
  stable WARN.
- Report what got worse even if nobody asked about it.

---

## Skill 11 — EKF, compass and estimator health

**Question:** does the flight controller trust its own sensors?

```bash
python alog.py ekf       flight.bin
python alog.py compass   flight.bin
python alog.py estimates flight.bin
```

- `XKF4` innovation ratios: ArduPilot rejects the measurement at 1.0. Read the **count of
  samples over 1.0**, not the mean; a rising count across flights is the signal.
- `solution status` lists any core flag not set for the whole window and the GPS-glitch
  flag time.
- Compass: field magnitude (120–550 mGauss), variation, `MAG.Health`, and **motor
  interference** as `corr(throttle, |B|)` — above 0.3, run `COMPASS_MOT`. Prearm
  "mag field" messages are quoted.
- `estimates`: ATT vs AHR2 / XKF1 and baro vs EKF altitude — two estimators of the same
  quantity disagreeing is the classic sensor-problem signature.

---

## Skill 12 — Parameters: what flew, what changed, what differs

**Question:** which parameter set does this log belong to?

```bash
python alog.py params     flight.bin --non-default          # what differs from firmware defaults
python alog.py params     flight.bin --diff snapshot.param  # in-log vs a dated capture
python alog.py params     flight.bin --grep HNTCH
python alog.py paramcheck flight.bin                        # NaN values, in-flight rewrites, hover-throttle drift
```

- A `.param` file says what the aircraft was *told*; the log says what it *did*.
  `MOT_THST_HOVER` is relearned in flight; `paramcheck` compares it with the learned
  `CTUN.ThH` and anything pinned to it (`INS_HNTCH_REF`) drifts too.
- Parameters that *disappear* between two captures are ArduPilot hiding a disabled
  subtree (all `FFT_*` when `FFT_ENABLE` goes to 0), not data loss.
- Interpret a log against the snapshot contemporaneous with that flight.

---

## Skill 13 — Events, log end, and "why did it do that"

**Question:** what happened, and did the log end in flight?

```bash
python alog.py events   flight.bin
python alog.py flight   flight.bin
python alog.py brownout flight.bin
```

- `events` decodes `EV`/`ERR`/`MSG` with subsystem names. **Prearm messages after landing
  are routinely the most informative lines in the log.**
- `flight`: ever armed, ever flew, autotune outcome, lean beyond `ANGLE_MAX`, mode changes
  the pilot did not command (`MODE.Rsn`), and the detector-disagreement WARN.
- `brownout`: still armed at log end with altitude — the log stopped in flight.
- A log opened at arming has no ARMED event (`LOG_DISARMED=0`); that is not a fault.

---

## Skill 14 — Get at the raw data

**Question:** I need the numbers themselves.

```bash
python alog.py types  flight.bin                       # every message, count, rate, fields
python alog.py fields flight.bin ESC                   # one message: units, MULT, ranges
python alog.py dump   flight.bin RATE --fields t,RDes,R --every 10 > rate.csv
python alog.py dump   flight.bin ESC --instance 2 --window rpm --json
python alog.py files  flight.bin --out embedded/       # hwdef, threads.txt and friends
```

In Python:

```python
from dflog import Log, airborne_window, flights, hover_chunks, esc_fundamental, spectral
from dflog import mix_for, motor_channels, trim_decomposition, motors_stopped

log  = Log("flight.bin")
w    = airborne_window(log, method="rpm")     # w.method says how it was chosen
rate = w.clip(log.df("RATE"))                 # airborne rows only
esc  = log.instances("ESC")                   # {0: df, 1: df, ...}
t, f0 = esc_fundamental(log)                  # fleet-mean motor fundamental, Hz
log.column("GPS", "HDOP")                     # -> "HDop": resolve a spelling
r    = spectral.analyse(log, window=w)        # Welch PSD + peaks in motor orders
```

- `MULT` multipliers are display metadata and are **not** applied; only format-char
  scaling is. `fields` shows both.
- Field names drift between firmware versions. A missing column raises
  `no column 'HDOP' in GPS; did you mean 'HDop'?`; `log.field(msg, "BAlt", "BarAlt")`
  probes alternatives.
- Prefer the `.bin` over a same-named `.log` or `.tlog`: the text export is pre-scaled and
  decimates per-instance ESC telemetry.

---

## Skill 15 — Write it up

Copy `templates/log-analysis-template.md`. Integrity block and window first, a
one-paragraph verdict, then findings ordered by value — each with the number, the
threshold and its source, the interpretation and the validation flight. New or worse
findings get a ⚠️ even when unrelated to the question. Correct visibly (keep the superseded
number, date the amendment), record the next untried step, and finish by listing the
documentation drift you found.

---

## Reading a status

| status | meaning | what to do |
|---|---|---|
| `PASS` | inside the threshold | read the number anyway; a stable WARN beats a PASS that moved 3× |
| `WARN` | above the warn level (`alog schema` lists every threshold with its source) | a prompt to look, not a verdict |
| `FAIL` | above the fail level, or an integrity error | lead with it |
| `SKIP` | the check could not run: message not logged, no ground truth | **never a pass**; report it as not logged |

Thresholds live in `dflog/checks.py::T` with provenance in `reference/thresholds.md`.
Never hardcode one in a script.
