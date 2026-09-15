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
   Same figure → static asymmetry (move the CG / fix the blade). Different → a
   translation artefact or wind; do not move anything on that evidence.
5. **`drive-normalised RPM spread`** — `RPM / (duty × V)` per motor. A CG offset leaves it
   flat while raw RPM spread reads 10 %; a dragging motor (bearing, damaged blade) drops
   it and nothing else shows it. The summary names the lowest motor.
6. **ESC temperature** and **spread** — one hot ESC in a set is a finding on its own.
7. **RPM spread** on medians, **DShot error rate**, **motor headroom** against the
   `MOT_SPIN_MAX` ceiling (p99.5, not the raw max).

**Validation flight:** one change (move the battery by the mm figure, or swap the named
prop), fly the same profile, run Skill 9 (`compare`) and expect the pitch trim and CG
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
- **Enabled but `FCNS` not logged** — SKIP; fix the logging, not the configuration.
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
  the tune. Compare like with like (Skill 2, Skill 9).

---

## Skill 7 — GPS quality, an antenna or ground-plane change, two receivers

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

## Skill 8 — Current sensor: is it lying, and by how much?

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

## Skill 9 — Before/after comparison (one change per flight)

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

## Skill 10 — EKF, compass and estimator health

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

## Skill 11 — Parameters: what flew, what changed, what differs

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

## Skill 12 — Events, log end, and "why did it do that"

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

## Skill 13 — Get at the raw data

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

## Skill 14 — Write it up

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
