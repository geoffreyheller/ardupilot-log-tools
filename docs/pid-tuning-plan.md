# Implementation plan — `alog tune`: PID gains from logs, with confidence

Written 2026-09-16 after the investigation recorded in `reference/pid-tuning-sources.md`.
Read that file first; every constant, field name and formula below is cited there. This
plan is written for subagents: each work package names the files it owns, what it consumes,
what it delivers, and the test that proves it. `RULES.md` binds all of it.

---

## 0. Decision summary

**What the tool will do.** `alog tune log1.bin [log2.bin ...]` reads one or more logs of
the same aircraft and reports, per axis (roll, pitch, yaw) and per parameter
(`ATC_RAT_x_P/I/D`, `ATC_ANG_x_P`, `ATC_ACC_x_MAX`, and the yaw `FLTE`), the current value,
a recommended value, the method that produced it, a **confidence in [0, 1] built from
named components**, the evidence, and the validation flight. When no log can support a
recommendation it **refuses with exit 3** and a JSON error naming what the logs contain and
what to change. The same analysis runs as a `tune` section of `alog all`, where
insufficiency is a `SKIP` per the contract.

**How gains are computed.** Three evidence tiers, in descending trust, fused per axis:

| tier | method | needs | gives |
|---|---|---|---|
| A | **AutoTune reconstruction** from `ATUN`/`ATDE`/`MSG` | a log containing an AutoTune session | the gains the firmware found, re-derived with the firmware's own rules, checked against what it saved; multi-session median and spread |
| B | **Virtual AutoTune**: identify the rate-loop plant from `PIDx.Tar/Act` and `RATE.xOut` (joint I/O Welch estimate gated by coherence), fit a low-order model, then run the AutoTune twitch procedure in software with its exact constants and the log's `AUTOTUNE_AGGR` / `GMBK`; plus 6 dB / 45° margins and heli-style P/D ceilings | `PIDx` (or `RATE`) at ≥ 100 Hz with pilot excitation | recommended gains that are what AutoTune would have found, with fit quality and coherence as confidence |
| C | **Closed-loop step response + oscillation ceiling**: PID-Analyzer/PIDReview Wiener deconvolution (`Tar → Act`) scored with AutoTune's criteria (overshoot vs `0.5 AGGR`, bounce vs `AGGR`); QuickTune's `SRate > QUIK_OSC_SMAX` ceiling; Ziegler–Nichols from a detected limit cycle | same as B, no coherence requirement | bounded multiplicative adjustments with direction (the fpvpidlab-style rules re-based on AutoTune's numbers), and hard ceilings |
| D | **Parameter consistency** (always available) | `PARM` | I/P and D/P ratios vs AutoTune's final ratios, `FLTD/FLTT` vs `INS_GYRO_FILTER/2`, `ACC_MAX` vs the prop-size formula (with `--prop-in`), AC_PID ranges, `RATE_FF_ENAB`, `SMAX` |

Tier D never produces a recommended gain by itself. **AutoTune is never required**: the
user's stated purpose (2026-09-17) is to obtain gains without flying AutoTune, so tier A is
opportunistic and every refusal, fix string and validation flight must be satisfiable by an
ordinary fast-logged flight. Tiers B and C require **fast attitude logging**; none of the 16 reference logs has it (§9 of the sources file), so the first
real-log behaviour of the tool is a refusal that names `LOG_BITMASK` bit 0.

**Why not a single model-based "optimal" formula.** No surveyed tool computes ArduCopter
gains from a log; the shipped tuners are a twitch search (Copter), a relay search
(QuickTune) and a frequency-response search (Heli). Reproducing the firmware's own search
on an identified plant (tier B) gives an answer whose criteria are the firmware's, is
deterministic, is testable against a simulated plant, and degrades honestly: when the
plant cannot be identified the tool says so and falls back to tier C. PX4's GMVC design
and VRFT are recorded as future options, not built.

---

## 1. Findings that shape the design

1. **Standard logging is 10 Hz.** `PIDR/RATE/ATT` at 10 Hz cannot show a rate loop whose
   filters sit at 20–40 Hz. `LOG_BITMASK` bit 0 (+ bit 12) logs them at the 400 Hz loop
   rate. AutoTune and SysID modes log at loop rate regardless. The tool's gate is
   `T["tune_pid_rate_hz"]` (fail below 100 Hz, warn below 200 Hz).
2. **AutoTune's criteria are explicit numbers**: bounce-back ≥ `AGGR × peak` sizes D,
   overshoot ≥ `0.5 AGGR` sizes P and angle P, 5 % steps, four consecutive passes, then
   `× (1 − GMBK)` and angle P `× (1 − GMBK)(1 − AGGR)`; `I = P` (yaw `0.1 P`);
   `ACC_MAX` from the measured peak acceleration. These are the scoring function for
   tiers B and C.
3. **The log carries the reference, the output and the plant input** (`Tar`, `Act`,
   `xOut`), so the joint I/O estimate `G = T_yr / T_ur` is unbiased under feedback and
   coherence gives a per-frequency confidence (Bendat–Piersol).
4. **`SRate` is QuickTune's oscillation detector**, logged in every PID record: an
   offline ceiling test for free.
5. **Field and parameter names drift**: `ATC_ACCEL_*_MAX` (cdeg/s²) → `ATC_ACC_*_MAX`
   (deg/s²); `ATT` → `ANG` at loop rate; `AUTOTUNE_GMBK` vs fixed backoffs. Read both
   spellings, convert units, and state which was found.
6. **Existing code**: checks are `check_*(log, w) -> Section` auto-wired to the CLI and
   JSON; `compare` is the only multi-log precedent (`cli.py:599-691`); `_grade` is the only
   sanctioned path from a number to a graded `Result`; `_notch_recommendation`
   (`analysis.py:721-767`) is the template for a recommendation table; `SKIP` never raises
   the exit code; `spectral.sample_rate()` is the "refuse irregular timing" gate;
   `tests/synthlog.py` can write any message once its FMT is declared; the committed
   fixture has no `RATE` or `PID*`.

---

## 2. Architecture

### 2.1 Modules

```
dflog/tune.py           NEW  pure analysis: signals → step response, plant model, virtual
                             autotune, ATUN reconstruction, ceilings, confidence, fusion
dflog/tunesim.py        NEW  AC_PID replica + plant + AutoTune twitch simulator (used by
                             tune.py tier B and by tests)
dflog/analysis.py       MOD  check_tune(log, w) → Section, registered in ALL_CHECKS as "tune"
dflog/cli.py            MOD  cmd_tune (multi-log), dispatch entry, schema keys
dflog/checks.py         MOD  new T entries (§4)
tests/tunesynth.py      NEW  synthetic fast-logged copter logs from tunesim (PIDx, RATE,
                             ANG/ATT, PARM, EV, ATUN/ATDE, SIDD)
tests/test_tune.py      NEW  unit + regression tests on synthetic logs
tests/test_cli.py       MOD  alog tune contract, refusal JSON, exit codes
tests/test_toolkit.py   MOD  real-log pins (refusals on the 10 Hz logs; fast-log pins later)
reference/*.md, SKILLS.md, CLAUDE.md, README.md   MOD  docs (§8)
```

`tune.py` depends on `parser`, `flight`, `spectral`, `stats`, `checks`, `tunesim`; it never
imports `analysis` or `cli`. `analysis.check_tune` and `cli.cmd_tune` both call
`tune.analyse(logs, ...)` so single-log and multi-log share one code path.

### 2.2 Data model (`dflog/tune.py`, dataclasses; define first, everything else codes to them)

```python
@dataclass
class GainSet:                # one axis' controller as configured, from PARM at segment start
    axis: str                 # "roll" | "pitch" | "yaw"
    rat_p: float; rat_i: float; rat_d: float; rat_ff: float
    fltd: float; fltt: float; flte: float; smax: float; imax: float
    ang_p: float; acc_max_dps2: float | None      # deg/s^2, converted from either spelling
    ff_enab: bool; gyro_filter: float; thst_hover: float | None
    loop_hz: float; aggr: float; gmbk: float | None; min_d: float
    defaulted: list[str]      # parameters that were not in the log
    param_names: dict         # which spelling was found (e.g. "ATC_ACCEL_R_MAX")

@dataclass
class AxisSignals:            # one axis over one parameter-constant segment of one log
    axis: str; log_name: str; segment: tuple[float, float]; source: str   # "PIDR" | "RATE"
    fs: float; jitter: float                                              # from spectral.sample_rate
    t: np.ndarray; tar: np.ndarray; act: np.ndarray; out: np.ndarray | None
    p: ...; i: ...; d: ...; ff: ...; dmod: ...; srate: ...; flags: ...   # None when source == "RATE"
    gains: GainSet
    limited_pct: float        # Flags & 1

@dataclass
class StepResponse:
    t: np.ndarray; mean: np.ndarray; frames: np.ndarray   # frames × samples, 0.5 s
    n_frames: int; n_dropped_low: int
    metrics: dict             # latency_s, rise_s, peak, peak_t, overshoot, bounce, settle_s, ss
    consistency: float        # 1 - IQR(peak over frames)/median(peak), clipped to [0, 1]

@dataclass
class PlantModel:
    freqs: np.ndarray; G: np.ndarray; coh: np.ndarray; n_avg: int      # non-parametric, joint I/O
    band: tuple[float, float]                                          # where coh ≥ gate
    k: float; tau1: float; tau2: float; delay: float                   # parametric fit
    fit_rms_db: float; fit_rms_deg: float; coh_mean_band: float
    eps_mag_at_crossover: float                                        # Bendat–Piersol

@dataclass
class Ceiling:                # a hard upper bound on a gain, with its origin
    param: str; value: float; method: str; evidence: dict

@dataclass
class Recommendation:
    axis: str; param: str; current: float; value: float
    change_pct: float; method: str                     # "autotune-log" | "virtual-autotune" | "step-rules" | "ceiling"
    confidence: float; components: dict                # prior, adequacy, excitation, consistency, agreement
    evidence: dict; note: str

@dataclass
class TuneAnalysis:
    logs: list[dict]          # identity per log: file, firmware, board, mcu, frame, boot_time, window
    identity_ok: bool; identity_reasons: list[str]
    per_axis: dict            # axis → {signals: [...], step: [...], plant: PlantModel|None, autotune: [...], ceilings: [...]}
    recommendations: list[Recommendation]
    refusals: list[dict]      # code, message, fix — see §2.6
    params: list[Result]      # tier D results
    constants: dict           # the constants table used (AGGR, GMBK, frame lengths, ...)
```

### 2.3 Pipeline (`tune.analyse(logs, windows, axes, prop_in=None, aggr=None) -> TuneAnalysis`)

1. **Identity** (multi-log): `FRAME_CLASS`, `FRAME_TYPE`, board, MCU id (from `log.info()`),
   firmware major.minor. Different frame or MCU → refusal `DIFFERENT_AIRCRAFT` (exit 3).
   Different firmware minor → WARN, continue.
2. **Segmentation** per log per axis: the window from `airborne_window` (default `auto`),
   split at every `param_changes()` entry touching that axis' `ATC_RAT_x_*`, `ATC_ANG_x_P`,
   `INS_GYRO_FILTER` or `ATC_RATE_FF_ENAB`; drop segments < 10 s; each segment gets its
   `GainSet` via `param_at(name, t0)`. The **latest** segment by boot time is the
   "current" gain set that recommendations are relative to.
3. **Signals**: prefer `PIDx` (`Tar, Act`, plant input `P+I+D+FF+DFF`); fall back to
   `RATE` (`xDes, x, xOut`) with a note that `Tar` is then pre-FLTT. Gate with
   `spectral.sample_rate()` (jitter ≤ 5 %, ≥ 16 samples) and `T["tune_pid_rate_hz"]`.
   `LOG_GAP` issues inside the segment split it further (PIDReview's batch rule).
4. **Tier A** on every log that has `ATUN` (independent of steps 2–3).
5. **Tier C** step response and oscillation on every segment with fs ≥ 100 Hz.
6. **Tier B** plant estimate pooled **across all segments and logs of the axis** (the
   plant does not depend on the gains — this is why multiple logs help), then the
   virtual AutoTune on the pooled model, then margins and ceilings.
7. **Tier D** on the current gain set.
8. **Fusion** (§2.5) → `recommendations`, `refusals`.

### 2.4 Algorithms

**Step response** (`tune.step_response(sig) -> StepResponse`). PID-Analyzer constants,
kept in a `CONSTANTS` table with sources (§4): frame 1.0 s, response 0.5 s, overlap 1/16,
Hann, regulariser `sn = 10 (1 − mask25Hz + 1e-9)` with the mask Gaussian-smoothed, frames
whose `max|tar| < 20 deg/s` dropped, `≥ 10` frames required (fail) / `≥ 30` (warn). Mean of
frames (PIDReview) is the reported curve; the per-frame stack gives `consistency`. Metrics on
the mean: latency (t to 50 %), rise (10–90 %), peak and its time, `overshoot = peak − 1`,
post-peak minimum and `bounce = (peak − postmin)/peak`, settling (±2 %, PIDtoolbox),
steady state (mean over 0.4–0.5 s). Score against AutoTune: `overshoot_ratio = overshoot /
(0.5 AGGR)`, `bounce_ratio = bounce / AGGR`.

**Oscillation / ceiling** (`tune.oscillation(sig) -> list[Ceiling] + evidence`).
`SRate` p95 vs `QUIK_OSC_SMAX` (5); `Dmod < 1` fraction; Welch PSD of `d` and of `act − tar`
over 3–40 Hz, peak prominence ≥ 10 dB → limit-cycle frequency `f_osc`, period `Tu`. If at
ceiling: attribute to P or D by the larger band-passed variance of the `P` and `D` terms at
`f_osc`; ceiling value = current gain; QuickTune recommendation `× 0.4`; Z–N "no overshoot"
figures reported as evidence, not as the recommendation. If `Dmod == 1` throughout and no
peak: "no ceiling found" (a fact, not a pass).

**Plant identification** (`tune.identify(signals: list[AxisSignals]) -> PlantModel`).
Per segment: `scipy.signal.csd`/`welch` with `nperseg = 2 s × fs`, 50 % overlap, Hann;
`T_yr = P_yr/P_rr`, `T_ur = P_ur/P_rr`, `G = T_yr / T_ur`; coherence `γ²_yr`, `n_avg`.
Pool segments by coherence-weighted averaging of `G` on a common frequency grid. Valid band
= frequencies where `γ² ≥ T["tune_coherence"]` (fail 0.6). Fit
`G(s) = k e^{−τd s} / ((τ1 s + 1)(τ2 s + 1))` by weighted (coherence) least squares on
log-magnitude and unwrapped phase over the band (`scipy.optimize.least_squares`, bounded,
deterministic start from the low-frequency gain and the −45° frequency). Report
`fit_rms_db`, `fit_rms_deg`, and `eps_mag_at_crossover = sqrt(1 − γ²)/(γ sqrt(2 n_avg))`
at the loop crossover. Also compute, from `G` and the configured controller `C(s)` (AC_PID
with FLTT/FLTE/FLTD first-order filters), the open-loop `L = C G`, gain margin and phase
margin (graded by `T["tune_gain_margin_db"]`, `T["tune_phase_margin_deg"]`), and the heli
ceilings `max_P` at the −161° frequency and `max_D` at −251° (formulas in sources §5).

**Virtual AutoTune** (`tunesim.autotune(plant: PlantModel, gains: GainSet, aggr, gmbk) -> dict`).
Discrete simulation at `loop_hz` of: AC_PID (`update_all` as in sources §2, SMAX 0,
sqrt controller off, test gains `I = 0.01 P`, FF 0, FLTT 0), the fitted plant (discretised
with `scipy.signal.cont2discrete`, delay as an integer sample buffer), output clipped ±1,
rate integrated to angle for the angle steps. Run the multicopter sequence exactly: targets
from `max_rate_step_bf` with the log's `thst_hover` (`CTUN.ThH` median if logged, else
`MOT_THST_HOVER`), `RATE_D_UP → RATE_D_DOWN → RATE_P_UP → ANGLE_P_DOWN → ANGLE_P_UP` with
the rules, limits, `SUCCESS_COUNT 4`, `step 0.05`, the 2× FLTD measurement LPF, the
timeout and early-stop rules, the `test_accel_max` measurement; yaw(E) moves FLTE. Apply
the backoffs; produce `rat_p, rat_i, rat_d, ang_p, acc_max_dps2` (and `flte` for yaw).
Deterministic (no randomness anywhere). Record the twitch count and the final
overshoot/bounce so the report can show *why* the number is what it is.

**Tier A** (`tune.autotune_sessions(log) -> list[dict]`). Sources §1.7 recipe. For each
axis: completed steps, gains per step, per-twitch pass/fail re-derived from `Targ/Min/Max`
with the log's AGGR, `ddt` max, final gains with GMBK (if `AUTOTUNE_GMBK` in PARM) or the
branch backoffs (verified in WP4), and cross-checks against `MSG` lines and against
`PARM` values after `EV 37`. Disagreement > 1 % between reconstruction and `MSG` is a
FAIL on that session (the reconstruction is wrong or the firmware differs — say which is
more likely from the version).

**Tier D** (`tune.param_consistency(gains: GainSet, prop_in=None) -> list[Result]`).
Graded with `_grade` where a `T` entry exists; ranges from AC_PID `var_info` reported as
PASS/WARN with source "AC_PID param ranges".

### 2.5 Confidence and fusion

Per recommended value:
```
confidence = prior[method] × adequacy × excitation × consistency × agreement
```
- `prior`: autotune-log 1.0, virtual-autotune 0.8, step-rules 0.5, ceiling 0.7. Source:
  MEAS (this plan). Documented in `reference/thresholds.md`.
- `adequacy`: 0 below `tune_pid_rate_hz.fail`, 1 at or above `.warn`, linear between; ×
  the fraction of the segment not output-limited (`1 − limited_pct/100`).
- `excitation`: step tier — `min(1, n_frames / tune_min_frames.warn)`; plant tier —
  `coh_mean_band` and the band width relative to `[0.5 Hz, FLTD]`; autotune-log tier —
  1 if the axis reached `TUNE_COMPLETE`, 0.3 if partial.
- `consistency`: step tier — the per-frame `consistency`; plant tier —
  `max(0, 1 − eps_mag_at_crossover / 0.25)`; autotune-log tier — 1 − spread across
  sessions (`tune_session_spread`).
- `agreement`: when two tiers produce a value for the same parameter,
  `1 − |Δ|/max(values)` clipped to [0.5, 1]; 1 when only one tier exists.

Every component is in `Recommendation.components` and printed, so a low number is
auditable. Bands (`T["tune_confidence"]`, higher is better): ≥ 0.7 recommend (PASS),
0.4–0.7 indicative (WARN "validate before applying"), < 0.4 withheld — the value is in the
evidence, the result is WARN "insufficient confidence", the recommendation column reads
"withheld".

Fusion order per parameter: tier A if present, else tier B if the plant band covers the
loop crossover and `fit_rms_db < 3`, else tier C. **Tier C calibration caveat (WP3,
2026-09-17):** AutoTune's overshoot/bounce criteria are applied to a twitch flown with
*test* gains (I ≈ 0, FLTT 0, before the 25 % backoff), while the log's step response is
the *final* closed loop with I active. Even the exact oracle step of AutoTune's own result
reads `overshoot_ratio` 2.4 and `bounce_ratio` 1.9, and the deconvolved estimate reads
higher still. Until the ratios are calibrated on a real fast-logged flight, tier C
contributes (a) ceilings from the oscillation detector as hard bounds and (b) the step
metrics and ratios as evidence only; its `step_rules` values are emitted with confidence
capped at 0.39 (the withheld band) and labelled "uncalibrated" in the note. The ratios are
not graded against `tune_overshoot_ratio`/`tune_bounce_ratio` as PASS/WARN/FAIL until then;
they are reported in the `step` table with the thresholds beside them. Ceilings always apply: a recommended value
above a ceiling is clipped to `0.4 × ceiling` (QuickTune margin) with the note naming the
ceiling. *Amended 2026-09-30 after the first real fast-logged flight (Brisket `brisket-t1.bin`):*
0.4 × applies only to an oscillation ceiling. Tier B's ceiling is now the margin ceiling
(`tune_ident.margin_ceilings`: the largest P/D keeping PM ≥ 45°, GM ≥ 6 dB on `C(z) G`), and a
value is clipped *to* it. The heli −161° P rule ignored D's phase lead: it put roll P's
ceiling at 0.138 against a flown P 0.135 with PM 48.6°, and the 0.4 × clip then recommended
−59 %. The fused rate set is re-checked against the margins and withheld if it fails. `I` follows `P` by the AutoTune ratio (1.0 roll/pitch, 0.1 yaw) unless the log's
own I/P ratio was deliberately different (tier D reports it; the tool preserves a
non-default ratio and says so).

Multi-log: tier B pools plants; tier C responses are reported per gain set and only frames
from the current gain set feed the rules; tier A pools sessions. Logs are listed with which
tiers each contributed to, so a reader sees which log carried the evidence.

### 2.5a Logging requirements — the parameters a usable log needs

This table is the single source of truth. It lives in code as `tune.LOGGING_REQUIREMENTS`
(a list of `dict(param, required, why, tier)`), is evaluated against a log by
`tune.logging_requirements(log) -> list[dict(param, current, required, ok, why)]`, and is
printed **verbatim in every refusal, in the markdown section, in the JSON, in `SKILLS.md`
and in `CLAUDE.md`** so the operator never has to look it up.

| parameter | required for a tuning log | why | tier |
|---|---|---|---|
| `LOG_BITMASK` | bit 0 (ATTITUDE_FAST, value 1) **and** bit 12 (PID, value 4096) set; e.g. 180222 → 180223 | `RATE`, `ANG`/`ATT` and `PIDR/PIDP/PIDY` at the loop rate instead of 10 Hz | B, C |
| `LOG_FILE_RATEMAX` | 0 (or ≥ `SCHED_LOOP_RATE`) | a non-zero cap decimates the fast stream back down | B, C |
| `LOG_BLK_RATEMAX` | 0 (onboard-flash boards) | same cap for the block backend | B, C |
| `INS_LOG_BAT_MASK` | 0 for the tuning flight | batch logging doubles the write rate and produces `LOG_GAP`; do notch work on a separate flight | B, C |
| `LOG_FILE_BUFSIZE` | ≥ 64 (KB), larger on boards that show `LOG_GAP` | buffer for the doubled rate | B, C |
| `LOG_DISARMED` | any | irrelevant; the window is airborne only | — |
| `SCHED_LOOP_RATE` | as flown (400 default) | reported; sets the fast-log rate | — |
| `AUTOTUNE_AXES`, `AUTOTUNE_AGGR`, `AUTOTUNE_MIN_D` | as used | tier A reads the session's own settings; `ATUN` is logged regardless of `LOG_BITMASK` | A |
| `SID_AXIS` 10/11/12, `SID_MAGNITUDE` 0.15 (yaw 0.55), `SID_F_START_HZ` 0.5, `SID_F_STOP_HZ` 40, `SID_T_REC` 70, `SID_T_FADE_IN` 15, `SID_T_FADE_OUT` 2 | for the optional plant-identification flight | `SIDD` output plus `RATE.xOut` input. The firmware defaults (0.5–40 Hz); AnalyticTune's 0.05–5 Hz sweep stops far below a small quad's rate-loop crossover (~23 Hz measured on the simulated 5-inch plant in WP5) and cannot identify the rate loop | B |

The `fix` string of every logging-related refusal is generated from this table with the
log's **current** values filled in (`LOG_BITMASK is 180222; set 180223`), so the message
is specific to the aircraft, not generic.

### 2.6 Failing loudly — the refusal catalogue

**The error must be obvious.** When `alog tune` refuses, the markdown output begins with
a block in this exact shape (JSON carries the same fields under `error`, `refusals`,
`logging_requirements`):

```
ERROR: these log files cannot be used for PID tuning.
  <file>: PID_RATE_TOO_LOW - PIDR/PIDP/PIDY logged at 10.0 Hz; 100 Hz needed (200 Hz recommended)
  <file>: NO_EXCITATION   - 0 frames above 20 deg/s in a 162 s window
Set these ArduCopter parameters and fly the tuning profile (SKILLS.md, "Recommend PID gains"):
  LOG_BITMASK        180222 -> 180223   (bit 0 ATTITUDE_FAST + bit 12 PID: loop-rate RATE/PID logging)
  LOG_FILE_RATEMAX   0      ok
  INS_LOG_BAT_MASK   1      -> 0        (batch logging doubles the write rate; separate flight)
  ...
exit code 3
```
The same block, minus the exit line, is the `tune` section's first note in `alog all`.

Each has a stable code, a message, and a `fix` string with the exact parameter change or
flight. In `alog tune`, when **no axis** has any tier A/B/C evidence the command exits 3
with the JSON error shape (`error`, `exit_code: 3`, plus `refusals` and `logs`). When some
axes are usable, unusable axes get a WARN result `"<axis> insufficient data"` and the
refusal list is in the section. In `alog all`, the whole-tool refusal is one SKIP result
carrying the same code and fix.

| code | condition | fix text |
|---|---|---|
| `NO_PID_MESSAGES` | no `PIDx` and no `RATE` in the window | set `LOG_BITMASK` bits 0 and 12 |
| `PID_RATE_TOO_LOW` | fs < 100 Hz (the 10 Hz standard) | `LOG_BITMASK = <current \| 1>` (e.g. 180222 → 180223), fly the tuning profile, set it back |
| `IRREGULAR_SAMPLING` | `spectral.sample_rate` jitter > 5 % or `LOG_GAP` fragments every segment below 10 s | logger stalled; check SD card / `LOG_FILE_BUFSIZE`, do not enable batch logging on the same flight |
| `NO_EXCITATION` | fewer than `tune_min_frames.fail` frames above 20 deg/s | fly the tuning profile (§7) |
| `OUTPUT_SATURATED` | `Flags & 1` on > 20 % of samples | the loop is nonlinear here; hover throttle / motor headroom first (`alog motors`) |
| `GAINS_CHANGED_IN_FLIGHT` | every segment < 10 s after splitting at parameter changes | one gain set per flight |
| `DIFFERENT_AIRCRAFT` | frame class/type or MCU id differ between logs | pass logs of one aircraft |
| `NO_COHERENCE` (tier B only) | valid band empty or does not reach crossover | SysID chirp flight (§7) or sharper stick inputs; tier C still reported |
| `AUTOTUNE_INCOMPLETE` (tier A only) | session ended before `TUNE_COMPLETE` on an axis | informational: tiers B and C from an ordinary fast-logged flight do not need a session; rerun AutoTune on that axis only if tier A is wanted |

---

## 3. CLI and JSON contract

```
alog tune LOG [LOG ...] [--window M] [--pad N] [--flight N] [--hz-floor HZ]
               [--axes roll,pitch,yaw] [--aggr 0.075] [--prop-in 10] [--json]
```
- `--flight all` rejected (as `compare`); `--flight N` applies to every log.
- `--aggr` overrides the log's `AUTOTUNE_AGGR` for the virtual AutoTune (the note says so).
- Exit: 0 / 1 / 2 by the worst result; 3 for the whole-tool refusal or `InputError`.
- JSON (`_envelope("tune", ...)` with no single `log`): `logs: [{file_name, path,
  integrity, window, firmware, board, mcu, frame, contributed: [tiers]}]`, `identity:
  {ok, reasons}`, `sections: [tune section to_dict()]` (one section, tables:
  `recommendations`, `ceilings`, `step`, `plant`, `margins`, `autotune`, `params`,
  `refusals`), `verdict`, `exit_code`. Whole-tool refusal: the error shape plus
  `refusals` and `logs`.
- `alog schema` gains `tune_constants` (the `CONSTANTS` table with sources) alongside
  `thresholds`.
- `check_tune(log, w)` (single log) adds section key `tune` to `alog all`, placed after
  `gust` in `ALL_CHECKS`; `tests/test_cli.py::test_all_json_has_the_contract_shape` picks
  it up automatically.

Markdown: the section renders the refusal (if any) first, then the recommendation table
(`axis | param | current | recommended | change % | confidence | method | why`), then
ceilings, step metrics per gain set, plant fit and margins, AutoTune sessions, parameter
consistency, and the validation flight note. Every table row's "why" is a measured number.

---

## 4. Thresholds and constants

Add to `dflog/checks.py::T` (each with `source`; rows in `reference/thresholds.md`):

| key | warn | fail | direction | source |
|---|---|---|---|---|
| `tune_pid_rate_hz` | 200 | 100 | lower worse | PID-Analyzer 0.5 s window + 25 Hz regulariser (Nyquist), ArduCopter fast logging = loop rate |
| `tune_min_frames` | 30 | 10 | lower worse | PID-Analyzer `high.sum() < 10` rule; 30 MEAS |
| `tune_coherence` | 0.8 | 0.6 | lower worse | fpvpidlab 0.5 gate, AnalyticTune "sufficient coherence", Bendat–Piersol |
| `tune_confidence` | 0.7 | 0.4 | lower worse | MEAS (this plan) |
| `tune_srate_osc` | 5 | 10 | higher worse | `QUIK_OSC_SMAX` default 5 |
| `tune_overshoot_ratio` | 1.0 | 2.0 | higher worse | AutoTune `0.5 × AGGR` overshoot allowance |
| `tune_bounce_ratio` | 1.0 | 2.0 | higher worse | AutoTune `AGGR` bounce-back criterion |
| `tune_gain_margin_db` | 6 | 3 | lower worse | AnalyticTune / heli AutoTune 6 dB |
| `tune_phase_margin_deg` | 45 | 30 | lower worse | AnalyticTune 45° |
| `tune_session_spread` | 0.15 | 0.30 | higher worse | MEAS |
| `tune_pi_ratio_dev` | 0.25 | 0.50 | higher worse | AutoTune `PI_RATIO_FINAL` 1.0 / yaw 0.1 |
| `tune_flt_ratio_dev` | 0.25 | 0.50 | higher worse | wiki `FLTD = FLTT = INS_GYRO_FILTER/2` |
| `tune_limited_pct` | 5 | 20 | higher worse | MEAS |

Algorithm constants (not gradings) live in `tune.CONSTANTS`, each entry
`dict(value=, source=)`, printed by `alog schema` as `tune_constants`: frame 1.0 s,
response 0.5 s, overlap 16, cut 25 Hz, min target 20 deg/s, 500 deg/s split, and the
full AutoTune table from sources §1.2/§1.6 (AGGR/GMBK defaults, steps, limits, targets,
success count, floors) plus QuickTune's 0.4 margin. RULES §2 forbids hard-coding numbers in
a check; this table is where they live and where their provenance is.

---

## 5. Work packages

Dependencies are stated; WP1 + WP2 first (parallel), then WP3/WP4/WP5 in parallel, then
WP6, WP7, WP8, WP9. Each package: run the six test suites before and after
(`.venv/Scripts/python.exe tests/test_*.py`, with `LOG_DIR`/`LOG_DIR_LARGE` set for
`test_toolkit`), no pinned figure may move, and delete stray `.dfcache` files in the Drive
log folders when done. Do not change the parser, the trim math or existing checks.

### WP1 — data model, signal extraction, segmentation, gates
**Owns** `dflog/tune.py` (dataclasses in §2.2, `CONSTANTS`, `extract_axes(log, w, axes)`,
`GainSet.from_log(log, t, axis)`, `segment(log, w, axis)`), `dflog/checks.py` (the `T`
rows of §4), `reference/thresholds.md` rows.
**Consumes** `parser.Log`, `flight.Window`, `spectral.sample_rate`, `param_at`,
`param_changes`, `quality()` for `LOG_GAP`.
**Delivers** `AxisSignals` per segment with the refusal codes `NO_PID_MESSAGES`,
`PID_RATE_TOO_LOW`, `IRREGULAR_SAMPLING`, `GAINS_CHANGED_IN_FLIGHT`, `OUTPUT_SATURATED`
raised as structured refusals (not exceptions). Both parameter spellings for `ACC_MAX`
with unit conversion; `ANG` preferred over `ATT` when present; `PIDx` preferred over
`RATE`. Also `LOGGING_REQUIREMENTS` and `logging_requirements(log)` of §2.5a, and
`format_logging_fix(rows) -> str` producing the parameter lines of the §2.6 block; every
logging-related refusal's `fix` is built from it with the log's current values.
**Tests** (`tests/test_tune.py`, section "extract"): a synthlog with `PIDR` at 400 Hz
yields fs ≈ 400 and source `PIDR`; at 10 Hz yields `PID_RATE_TOO_LOW` with the fix
string containing the current `LOG_BITMASK | 1`; a `PARM` change mid-window splits the
segment and the two `GainSet`s differ; missing `ATC_ACCEL_R_MAX` but present
`ATC_ACC_R_MAX` converts correctly (and vice versa); the `defaulted` list names what was
assumed.

### WP2 — simulator and synthetic fast-logged copter logs
**Owns** `dflog/tunesim.py` (AC_PID replica `ACPID.update_all` faithful to sources §2
including FLTT/FLTE/FLTD, IMAX, limit flag; `Plant` = `k e^{−τd s}/((τ1 s+1)(τ2 s+1))`
discretised; `ClosedLoop.run(target_series)`; `autotune(...)` per §2.4 — WP5 may extend it)
and `tests/tunesynth.py` (`fast_log(plant, gains, seconds, loop_hz=400, seed=..., stick=...)`
writing `PARM`, `EV` 28/18, `PIDR/PIDP/PIDY`, `RATE`, `ANG` and `ATT`, `CTUN`, `MODE`, `RCIN`
via `synthlog.LogWriter`; `autotune_log(...)` writing an `ATUN`/`ATDE`/`MSG`/`EV 30..37`
session produced by running `tunesim.autotune` on the simulated plant; optional `SIDD`
chirp). The stick generator is a deterministic pseudo-random sequence of shaped steps
(`numpy.random.default_rng(seed)`) passed through the sqrt controller + jerk limit so
`RDes` looks like a real log.
**Consumes** nothing from WP1 except the `GainSet` field names.
**Delivers** a way to make a log whose true plant and true closed-loop step response are
known.
**Tests** (`tests/test_tune.py`, section "sim"): the replica reproduces AC_PID's
first-order filter alphas; a unit rate step on a known plant gives the analytically
expected steady state; the writer's log parses with `Log(...)` with `diagnostics.ok`;
`alog info` reports `PIDR` at 400 Hz.

### WP3 — step response and oscillation ceiling (tier C)
**Owns** `tune.step_response`, `tune.step_metrics`, `tune.oscillation`,
`tune.step_rules` (the direction/step rules re-based on AutoTune: overshoot_ratio > 1 →
P −5 %/step or D +10 %, bounce_ratio > 1 → D −5 %, rise slow with no overshoot → P +5 %,
SS error > 5 % → I toward P; every step size cites `RD_STEP/RP_STEP` and the QuickTune
margin; never more than ±25 % total, citing `GMBK`).
**Consumes** WP1 `AxisSignals`, WP2 logs.
**Tests**: on a simulated log the deconvolved mean step response matches the closed loop's
true step response (from `tunesim`) within 5 % RMS over 0–0.3 s; peak/overshoot within
0.02; frames below 20 deg/s are counted in `n_dropped_low`; a hover-only log (no stick)
gives `NO_EXCITATION`; a log simulated with P raised until the loop rings shows a PSD
peak at the predicted frequency and `SRate` above `tune_srate_osc.warn`, attributed to P;
`Dmod` stays 1.0 on a calm log and "no ceiling found" is reported as a note, not PASS.

### WP4 — AutoTune reconstruction (tier A) and branch constants
**Owns** `tune.autotune_sessions`, `reference/pid-tuning-sources.md` §1.6 (replace
"inferred" with verified values per branch: fetch `AC_AutoTune_Multi.cpp` on
`Copter-4.3`, `Copter-4.4`, `Copter-4.5`, `Copter-4.6`, `Copter-4.7` and record
`RD/RP/SP_BACKOFF` or `GMBK`, and confirm the EV ids 30–37 from `AP_Logger/LogStructure.h`
`LogEvent`), `reference/messages.md` rows for `ATUN`, `ATDE`, `SIDD`, `SIDS`, `QUIK`.
**Consumes** WP2 `autotune_log`.
**Tests**: reconstruction of a synthetic session reproduces the gains `tunesim.autotune`
saved, to 0.1 %; a truncated session yields `AUTOTUNE_INCOMPLETE` for the missing axis
with the completed steps listed; a session whose `MSG` line disagrees with the
reconstruction by 5 % is FAIL naming both numbers; two sessions in two logs pool to a
median with the spread graded by `tune_session_spread`.

### WP5 — plant identification, margins, ceilings, virtual AutoTune (tier B)
**Owns** `tune.identify`, `tune.margins`, `tune.ceilings_from_plant`, `tunesim.autotune`
(finalise), `tune.virtual_autotune`.
**Consumes** WP1, WP2.
**Tests**: on a simulated log with pilot excitation the joint I/O estimate recovers the
true plant magnitude within 1 dB and phase within 10° over the coherent band, and the
parametric fit recovers `k, τ1, τd` within 15 %; with the excitation reduced 10× the
coherence drops below the gate and `NO_COHERENCE` is raised (not a wrong model); the
computed gain/phase margins match `scipy.signal`'s on the true `L = C G` within 0.5 dB /
3°; **the virtual AutoTune run on the true plant produces the same gains as the
`autotune_log` session written by WP2 for that plant** (same code, so this pins the
sequence, and a second test perturbs `AGGR` and checks the direction of change); the
heli ceilings are above the virtual-AutoTune P and D for a well-damped plant.

### WP6 — tier D, confidence, fusion, `TuneAnalysis`, Section rendering
**Owns** `tune.param_consistency`, `tune.confidence`, `tune.fuse`, `tune.analyse`,
`tune.to_section(analysis) -> Section` (using `Section.table/note`, `_grade` for graded
items, the `_Params` defaults note), `analysis.check_tune` + `ALL_CHECKS` + `__all__`.
**Consumes** WP3, WP4, WP5 outputs.
**Tests**: components multiply as specified and every component is in `components`; a
value above an oscillation ceiling is clipped to 0.4 × ceiling, above a margin ceiling to the ceiling (amended 2026-09-30), with the note; tier A wins over B over C
when all exist; a withheld recommendation (< 0.4) is WARN and its value is in evidence
only; `I` follows `P` by axis ratio unless the log's ratio was non-default (then preserved
and noted); `check_tune` on a 10 Hz synthetic log returns a SKIP carrying
`PID_RATE_TOO_LOW` and exit code is unaffected; tier D grades `FLTD ≠ INS_GYRO_FILTER/2`
and a yaw `I/P` of 1.0 as WARN with sources; `--prop-in` produces the Mission Planner
comparison row; the Section's JSON round-trips through `_jsonable` with no NaN.

### WP7 — CLI `alog tune`, schema, JSON contract
**Owns** `cli.cmd_tune`, subparser (`nargs="+"`, `add_window`, `--axes`, `--aggr`,
`--prop-in`), dispatch entry, `cmd_schema` additions (`tune_constants`, the `tune` payload
row), `reference/json-output.md` row, `tests/test_cli.py` additions.
**Consumes** WP6.
The refusal block of §2.6 is printed first, in that exact shape, on stdout for markdown
and as `error` + `refusals` + `logging_requirements` in JSON; it must be impossible to
miss (first lines of output, the word `ERROR`, every parameter with current → required).
**Tests**: the refusal output on a 10 Hz log starts with `ERROR: these log files cannot be
used for PID tuning.` and contains a `LOG_BITMASK` line with the current and required
values; two simulated logs of one aircraft → one JSON document with `logs[]`, one
section, `exit_code` matching the verdict; two logs with different `FRAME_TYPE` → exit 3,
JSON error with `refusals[0].code == "DIFFERENT_AIRCRAFT"`; a single 10 Hz log → exit 3
with `PID_RATE_TOO_LOW` and the `fix` naming the new `LOG_BITMASK`; `--flight all` → exit
3 naming the rejection; markdown and JSON carry the same numbers; output is byte-identical
across two runs; `alog schema` lists the new thresholds with sources and `tune_constants`.

### WP8 — documentation
**Owns** `SKILLS.md` (new "Skill — Recommend PID gains" after Skill 6, in the house
format: question, **the §2.5a parameter table as "before you fly: set these
parameters"**, commands, lines to read in order, decision, validation flight; the table
must be the same rows as `tune.LOGGING_REQUIREMENTS` — add a test that compares them),
`README.md` (a "Logging for PID tuning" paragraph with the same parameters),
`CLAUDE.md` (a §4 bullet for `tune`, a §9 pitfall "standard logging is 10 Hz; tuning
needs bit 0", the `--aggr`/`--prop-in` flags in §1), `README.md` command list,
`reference/existing-tools.md` (a "PID tuning tools" subsection pointing to the sources
file and stating what no surveyed tool did), `reference/pitfalls.md` (10 Hz, `ATT` vs
`ANG`, `ACCEL` vs `ACC`, `ddt` unscaled), `templates/log-analysis-template.md` (a
"tune" block).
**Consumes** WP7 final CLI.
**Tests**: `tests/test_cli.py::test_schema_lists_checks_and_thresholds_with_sources` still
passes; every `T` key added has a row in `reference/thresholds.md` (add a test that greps
for it).

### WP9 — real-log validation and fixtures
**Owns** `tests/test_toolkit.py` additions, a new fixture via `tools/make_fixture.py`,
`tests/fixtures/README.md`.
**Consumes** WP7 and **data from the user** — ordinary flights only; **no AutoTune session
is required or expected** (the user's stated purpose for this tool is to avoid AutoTune as a
requirement, 2026-09-17; tier A stays opportunistic): (a) one fast-logged flight of the
Circuit quad (its `PARM` gains were AutoTuned in the past → they are the ground truth: the
virtual AutoTune with the log's AGGR should land within ±25 % of `ATC_RAT_RLL_P` 0.1235 /
`_D` 0.00334 / `ATC_ANG_RLL_P` 16.0 — a wider band than the confidence bands because the
aircraft's AutoTune flight was a different day), (b) a fast-logged flight of the Brisket
quad on the initial parameters (a "before" log whose recommendation can be flown as the
validation flight), (c) the same Brisket aircraft flown on the recommended set (the
"after" log that closes the loop). The tier-C calibration (§2.5 caveat) uses (a).
**Tests**: every current reference log → `alog tune` exits 3 with `PID_RATE_TOO_LOW`
(pin it: this is the real-world "fail loudly"); `check_tune` raises on no reference log
(add to `test_no_check_raises`); pins on the fast-log fixture once it exists (step peak,
plant `k, τ1`, recommended P/D, confidence, to stated tolerances).

---

## 6. Test strategy in one paragraph

Ground truth comes from the simulator: a known plant and a faithful AC_PID replica give
the true closed-loop step response, the true frequency response and — because the virtual
AutoTune is the same code that writes the synthetic `ATUN` session — the gains AutoTune
would find. Each tier is tested against that truth with stated tolerances, then against
its failure mode (no excitation, low rate, saturation, changed gains). Real logs pin the
refusal path today and the numbers once a fast-logged flight exists. Determinism is
asserted by running twice and comparing bytes.

---

## 7. Validation flights (what the report will tell the operator to fly)

- **Data-acquisition / tuning profile** (for tiers B and C): `LOG_BITMASK` bit 0 on (e.g.
  180222 → 180223; bit 12 already set on both aircraft), `INS_LOG_BAT_MASK 0`, take off in
  ALT_HOLD, 30 s hover, then 60 s of sharp roll stick inputs (±15–20°, quick release), 60 s
  pitch, 30 s yaw, land. 3–5 min total. Set bit 0 back afterwards on onboard-flash boards.
- **Plant-identification profile** (raises tier B confidence): SysID mode, `SID_AXIS` 10
  (then 11, 12), `SID_MAGNITUDE 0.15` (yaw 0.55), `SID_F_START_HZ 0.5`, `SID_F_STOP_HZ 40`,
  `SID_T_REC 70`, fade in 15 s, fade out 2 s (the firmware defaults), one axis per flight.
  `SIDD` is then the output and `RATE.xOut` the input. AnalyticTune's 0.05–5 Hz recipe is
  for the attitude loop; a small quad's rate-loop crossover sits near 20–25 Hz (WP5 measured
  23 Hz on the simulated 5-inch plant), so the sweep must reach 30–40 Hz for tier B.
- **Hover-only logs are refused on excitation, not coherence** (WP5 finding): with no stick
  the reference is the angle loop's reaction to noise, coherence reads 0.98, and the plant
  estimate is biased toward −1/C. The frame gate (`tune_min_frames`) runs before the
  coherence gate for that reason.
- **Post-change verification**: apply the recommended set for one axis, fly the tuning
  profile again, `alog tune before.bin after.bin` — the tool reports both gain sets;
  expect overshoot_ratio and bounce_ratio to move toward 1.0 and the margins to stay
  above 6 dB / 45°. `alog compare` for everything else.

---

## 8. Documentation drift to fix while doing this

- `reference/existing-tools.md` says nothing about PIDReview, AnalyticTune or PID-Analyzer;
  add the pointer to `reference/pid-tuning-sources.md`.
- `analysis.check_coverage` `key_msgs` lists `PIDR` but not `PIDP`/`PIDY`/`ANG`/`ATUN`;
  add them so an absent input reports as absent.
- `reference/messages.md` lacks `ATUN`, `ATDE`, `SIDD`, `SIDS`, `ANG`, `QUIK`.
- `CLAUDE.md` §9 should carry "standard `LOG_BITMASK` logs PID at 10 Hz".

---

## 9. Open decisions (recommendation first)

1. **Refusal severity in `alog all`** — recommended: SKIP (contract), with the code and fix
   in the summary; `alog tune` alone exits 3. Alternative: WARN in `alog all` so exit code
   rises. Stay with the contract unless the user says otherwise.
2. **Plant model order** — recommended: two poles + delay (motor lag, filter lag); the
   wiki's three-pole-one-zero model can be added as an option once real fast logs show the
   two-pole fit is inadequate (`fit_rms_db > 3`).
3. **Yaw** — recommended: tier B/C for `ATC_RAT_YAW_P` and `FLTE` only (AutoTune's yaw(E)
   moves FLTE, not D); yaw-D only if `AUTOTUNE_AXES` bit 8 was set on the aircraft.
4. **`ACC_MAX`** — recommended: report the virtual-AutoTune value and the Mission Planner
   prop-size value side by side; recommend the AutoTune one only when tier B confidence
   ≥ 0.7, otherwise leave the parameter unchanged and say why.
5. **Scope of WP2's simulator** — it is a test oracle and the tier B engine, not a flight
   simulator: no mixer, no motor saturation beyond ±1 output, no coupling between axes.
   Say so in its docstring.
