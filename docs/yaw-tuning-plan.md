# Yaw rate-loop tuning without a SysID sweep — plan

Written 2026-10-01 for the agent that picks this up. Read `CLAUDE.md`, `RULES.md` and
`docs/pid-tuning-plan.md` first. This plan assumes their vocabulary: tiers A–D, margin
ceilings, the extrapolation gate, the deadband.

## 1. Situation

Aircraft: **Brisket**, 10-inch quad X, ArduCopter V4.7.0-dev (259b79c3), 3DRControlN1 board, 8S
pack. Roll and pitch were tuned with `alog tune` over five fast-logged flights on 2026-09-30
and 2026-10-01. Each gain change was predicted, flown and measured, and every prediction
landed within about 1° of phase margin:

| axis | P | I | D | PM / GM, full pack (30.8 V) | status |
|---|---:|---:|---:|---|---|
| roll | 0.140 | 0.140 | 0.0044 | 45.6° / 7.6 dB | done |
| pitch | 0.121 | 0.121 | 0.0035 | 44.6° / 7.9 dB | done |
| yaw | 0.18 | 0.018 | 0 | 98–105° on the fit / gain margin **not measured** | **open** |

Yaw is still on the firmware defaults. Yaw rate tracking correlation is 0.64–0.65 when
yaw was exercised. A yaw step reaches only 56–88 % of the target within 0.5 s, with about
100–120 ms latency. The loop is very conservative: phase margin is around 100° and the crossover
is about 1 Hz. More yaw P is almost certainly right; *how much* is not known.

### Why yaw is stuck

Tier B identifies the plant from pilot stick input. For yaw, the coherent band ends at
**3–3.5 Hz** on every flight that excited yaw. The fit then pins the high-frequency lag
at its bounds: `tau2` at the 1 ms floor and the delay at 0. Roll and pitch, on the same
motors, read `tau2` 39–42 ms and delay 13.3–14.2 ms on all five flights.

The virtual AutoTune proposes yaw P 0.18 → **0.48–0.53**, FLTE 2 → 1.3–1.5, at confidence
0.64–0.70. The fusion withholds it through the extrapolation gate
(`tune_ident.extrapolated`). That set's gain margin is read at about 21 Hz, far above the
measured band, on a model whose lag is not identified. The withhold is correct. Do not
weaken the gate to get a number out.

Last session I refitted with roll/pitch's `tau2`/delay forced in. Phase error rose from
13.7° to **22°**, so the yaw data actively disagrees with a plain "roll/pitch lag + first-order
body" model. That disagreement is the most interesting lead (§4 A).

### The obvious fix is ruled out

The standard answer is a SysID chirp (`SID_AXIS 12`, 0.5–40 Hz). **The user has ruled it
out:** the flying areas are confined, and wind makes SysID runs a crash risk. Also standing
(memory, 2026-09-17): **the user will not fly AutoTune**, and the tool exists so AutoTune
is not needed. Do not propose either. Anything else that changes gains automatically in
flight (QuickTune-style Lua) is out unless the user approves it explicitly. Ask; don't
assume.

## 2. Constraints

1. No SysID-mode sweep. No AutoTune. No automated in-flight gain changes without explicit
   user approval.
2. Test flights must be **confined-area safe**: Loiter or AltHold, little or no translation,
   short, pilot in control throughout. Yaw-in-place is the friendly axis, because rotating
   on the spot does not move the aircraft.
3. One parameter change per flight (CLAUDE.md §2.2). Before/after only at similar pack
   voltage: plant gain tracks voltage, about 14 % between 25.7 V and 30.9 V, so use a full
   pack (~31.7 V at takeoff).
4. Keep `LOG_BITMASK` 180223 and `INS_LOG_BAT_MASK` 0. Don't touch roll/pitch gains; they are
   the control axes for any yaw flight.
5. Thresholds go in `dflog/checks.py::T` with a source. Constants go in the module's
   `*_CONSTANTS` with a source. No hardcoded numbers (CLAUDE.md §9).
6. Every claim needs a validation: predict the yaw margins, fly, and measure them like
   roll/pitch.

## 3. Data you already have

Logs are in the directory `LOG_DIR_LARGE` points at, named by the labels below (`tests/logmap.local.json` maps each label to its file).
Parameter snapshots are the dated captures in the params column. Yaw
gains are identical in all of them.

| log | params | mean V | yaw \|Tar\| p95 / max (deg/s) | yaw identification |
|---|---|---:|---|---|
| brisket-t1 | 30d | 27.56 | 13 / 45 | refused `NO_COHERENCE` (band 0.5–1.5 Hz) |
| brisket-t2 | 30e | 25.65 | 49 / 106 | band 0.5–3.5 Hz, coh 0.91, k 520, τ1 0.174 s, fit 1.60 dB / 13.1° |
| brisket-t3 | 30f | 30.88 | 18 / 65 | refused `NO_COHERENCE` |
| brisket-t4 | 30g | 30.79 | 52 / 163 | band 0.5–3.0 Hz, coh 0.89, k 533, τ1 0.169 s, fit 1.68 dB / 13.1° |
| brisket-t5 | 10-01a | 30.78 | 40 / 99 | band 0.5–3.5 Hz, coh 0.83, k 593, τ1 0.197 s, fit 2.13 dB / 12.0° |

The three identified yaw plants agree: k 520–593 deg/s per unit output, τ1 0.17–0.20 s, and
the same ~12–13° phase misfit. Roll/pitch propulsion across all five flights: `tau2`
39.1–42.2 ms, delay 13.3–14.2 ms.

Relevant yaw parameters (unchanged throughout):
- **Rate loop:** `ATC_RAT_YAW_P` 0.18, `_I` 0.018, `_D` 0, `_FF` 0, `_FLTT` 21, `_FLTE` 2, `_FLTD` 0, `_IMAX` 0.5.
- **Angle loop and shaping:** `ATC_ANG_YAW_P` 4.5, `ATC_ACCEL_Y_MAX` 27000 cdeg/s² (270 deg/s²), `ATC_SLEW_YAW` 6000, `ATC_RATE_Y_MAX` 0.
- **Pilot input:** `PILOT_Y_RATE` 202.5, `PILOT_Y_EXPO` 0, `PILOT_Y_RATE_TC` 0, `ATC_INPUT_TC` 0.15.
- **Motors and filters:** `MOT_YAW_HEADROOM` 200, `INS_GYRO_FILTER` 42, `SCHED_LOOP_RATE` 400.

## 4. Workstreams

Start with **A, B and C**. They use only existing logs and need no flight. D is the
confined-area flight that replaces the sweep. E is the validation sequence.

### A. Grey-box yaw plant: borrow the lag, model the physics

**Hypothesis.** Yaw torque on a multirotor has two parts: rotor drag torque (∝ ω², through
the motor lag) and the **reaction torque from accelerating the rotors** (∝ dω/dt). The second
is a derivative term: it adds a zero, which is phase lead. A first-order-body plus motor-lag
model has no lead, so the free fit buys it back by driving `tau2` and the delay to zero.
That would explain both the pinned parameters and why forcing in the roll/pitch lag made
the phase fit worse.

**Steps.**
1. In `dflog/tune_ident.py`, add a yaw model variant: `G = k (1 + Tz s) e^{-s d} / ((τ1 s + 1)(τ2 s + 1))`.
   Fix `τ2` and `d` from that same log's roll/pitch fits (their mean), or from B. Fit
   `k, τ1, Tz` on the yaw band. Keep the existing 4-parameter fit for roll/pitch untouched.
2. Fit the three identified yaw flights (15-32, 18-50, 12-42). Accept the model only if
   **all** of these hold:
   - phase residual well under the free fit's 12–13° (target ≤ 5°);
   - magnitude residual ≤ the free fit's;
   - `Tz` consistent across the three flights (≤ 15 % spread) and physically plausible;
   - no parameter at a bound.
3. If that fails, try other physically motivated structures before giving up: e.g. a
   second-order body for frame flex, or a separate motor pole for the yaw diagonal pair.
   Report each model's residuals in a table. Never pick a model by its recommendation.
4. Cross-check `Tz` against first principles if you can: rotor inertia vs drag-torque
   coefficient. The prop/motor specs may be in the user's build notes in the Drive folder;
   ask before assuming.

**Output.** A yaw plant whose high-frequency behaviour comes from measured physics,
borrowed from the sibling axes, rather than from a fit at its bounds.

### B. Measure the motor lag directly from ESC telemetry

**Why.** A borrows `τ2`/delay from roll/pitch identifications. An independent measurement on
a different instrument makes that borrowing defensible.

**Steps.** The ESC RPM is ~51 Hz per motor (`ESC` instances 0–3; map through
`SERVOn_FUNCTION`; see `frames.motor_channels`). PID output is 400 Hz (`PIDR/PIDP/PIDY`
P+I+D+FF; mix through `mix_for`). Identify the per-motor command → RPM dynamics: a first-order
lag plus delay, with the 51 Hz sampling and its jitter respected (`spectral.sample_rate`
refuses irregular sampling; resample only with a stated justification). Compare with
`tau2` ≈ 40 ms and delay ≈ 13.5 ms. If they disagree, find out why before using either.

### C. Robust margins instead of a binary withhold

**Idea.** The extrapolation gate withholds because the gain margin rests on unidentified
parameters. Replace "unidentified, so withhold" with "evaluate the worst case over the
plausible range":
- `τ2` ∈ [B or roll/pitch min − 20 %, max + 20 %];
- delay likewise;
- `Tz` ∈ its cross-flight range;
- k at the full-pack value.

A gain set passes when **every** corner keeps PM ≥ 45° and GM ≥ 6 dB.

**Steps.**
1. Add `tune_ident.robust_margins(plant, gains, ranges)` and a robust margin ceiling. Use the
   same scan/bisection as `margin_ceilings`, but on the worst corner.
2. In `tune_fuse.fuse`, when `extrapolated()` fires *and* robust ranges are available, use the
   robust check instead of withholding. Say so in the note: the corner that binds, and the
   ranges used. Keep the plain withhold when no ranges exist.
3. Confidence: the robust result must not inherit tier B's 0.8 prior unexamined. Add a
   named component, or reuse `consistency`, that falls with the width of the ranges.
   Document it in `reference/thresholds.md` and SKILLS.md Skill 7.

### D. A confined-area excitation flight (replaces the sweep)

**Observation to verify first.** Stick yaw is shaped by `ATC_ACCEL_Y_MAX` = 270 deg/s²
before it reaches the rate loop. A target limited to acceleration *a* can only reach an
amplitude of about a/(2πf): ~43 deg/s at 1 Hz, ~12 at 3.5 Hz, ~4 at 10 Hz. That would put
the reference below gyro noise above ~3 Hz, which is exactly where the coherent band ends,
whatever the pilot does.

**Steps.**
1. From the existing logs, compute the PSD of yaw `PIDY.Tar` (and `RATE.YDes`) against
   frequency. Check whether it rolls off where the accel-limit model says. Read
   `AC_AttitudeControl` input shaping in ArduPilot master to confirm which parameters shape
   the yaw rate target in Loiter/AltHold: `ATC_ACCEL_Y_MAX`, `ATC_SLEW_YAW`,
   `PILOT_Y_RATE_TC`, `ATC_INPUT_TC`, `ATC_RATE_FF_ENAB`. Cite file and function.
2. If confirmed: design a **yaw-in-place excitation flight** in **Loiter**, where GPS holds
   position, so wind and the confined area are handled by the position controller. The
   pilot gives yaw-only stick inputs: sharp pulses and doublets, then the fastest left-right
   stick wiggles they can manage, ~60–90 s in all.
3. The one candidate parameter change is a **temporary** increase of `ATC_ACCEL_Y_MAX`.
   It shapes the *target* only, and should not change the rate-loop plant or controller;
   verify that claim in the source. Pick the value from step 1 so the reference reaches the
   frequencies the margins need. State the expected coherent band before the flight, and
   restore the parameter afterwards. This is a configuration change, so it counts as the
   flight's one change: no gain changes on this flight.
4. Write the flight card in the user's preferred format: plain numbered lines, parameters
   named, no tables (the user asked for that on 2026-09-30).

### E. Validation sequence (gain flights)

Only after A–C produce a yaw recommendation the robust check accepts:
1. Step yaw P (I follows at 0.1 × P; FLTE as recommended) **partway**, e.g. to keep the new
   crossover inside the measured band with room to spare, not straight to the full
   recommendation. Predict PM/GM (nominal and robust), crossover and step peak.
2. Fly the standard profile in Loiter on a full pack: hover, roll/pitch twitches (the
   control axes), yaw pulses and doublets.
3. Compare measured vs predicted, as for roll/pitch (pinned in
   `tests/test_toolkit.py::test_brisket_*`). The yaw crossover should move inside the band,
   so the in-band phase margin becomes a direct check of the prediction.
4. Repeat until the robust ceiling is reached, or the measured margin reaches 45° / 6 dB.
5. Then the angle loop: `ATC_ANG_YAW_P` (and roll/pitch angle P). This is a known tool gap:
   the virtual AutoTune finds angle P on its own unclipped rate loop, so it is always
   withheld. Re-running the angle step on the final rate set is a separate task; note it,
   don't solve it here unless asked.

## 5. Deliverables

1. Code: the yaw grey-box model (A), the motor-lag identification (B), robust margins (C),
   with tests in `tests/test_tune_ident.py` / `test_tune_fuse.py`. Build synthetic logs with
   `tests/tunesynth.py`: add a reaction-torque zero to the simulator's yaw plant
   (`dflog/tunesim.py`) so the true plant is known. Add real-log pins on the three yaw
   flights in `tests/test_toolkit.py`, gated on `LOG_DIR_LARGE`.
2. A short findings section appended to this file: which model won and its residuals on
   all three flights, the measured motor lag, and the robust yaw ceiling. Mark it
   "amended <date>"; don't rewrite the history above.
3. The yaw recommendation, with nominal and robust PM/GM and the predicted crossover.
4. The flight card for D and/or E (numbered lines, no tables).
5. Doc updates the change implies: CLAUDE.md §8/§10, SKILLS.md Skill 7,
   `reference/pitfalls.md`, `reference/thresholds.md`. `tests/test_tune_docs.py` checks some
   of these against the code.

## 6. Done means

- A yaw model fits all three yaw flights with no parameter at a bound and a phase residual
  ≤ 5°. Or a written, evidenced explanation of why none does.
- Either a robust-margin yaw recommendation or a clear statement of what data is still
  missing and the safe flight that gets it.
- All 13 suites pass (CLAUDE.md §12 list; `LOG_DIR`/`LOG_DIR_LARGE` set for
  `test_toolkit.py`).
- Nothing in the existing roll/pitch pins changed. If one did, the change is wrong until
  proven otherwise.

## 7. Working notes

- Python: `.venv/Scripts/python.exe` (uv venv; no system python). `LOG_DIR` holds the 5-inch
  reference logs, `LOG_DIR_LARGE` the 10-inch Brisket logs.
- Don't run Python through bash heredocs: backslashes get mangled. Write a script file
  (scratchpad) or use the Edit tool.
- Delete `*.dfcache` beside the logs when done. Use literal paths: the safety check blocks
  `rm` on `$VAR/*.dfcache`.
- `PIDx.Tar/Act/Err` are rad/s; `extract_axes` converts to deg/s and restamps PIDx to the
  RATE tick. `RATE` is deg/s.
- The user flies; the agent proposes. Confirm any new flight profile with the user before
  calling it the plan. State predictions *before* the flight.
- All work since 2026-09-17 on `alog tune` is **uncommitted** on `main`. Don't commit
  unless the user asks.

---

## 8. Findings — amended 2026-11-05

The sections above are kept as written on 2026-10-01. Read this one first.

### What changed under the plan

- **The hardware.** Master Airscrew 3MR 10x5x3 props from 2026-10-06; FETtec SFOC 4in1 ESC from
  2026-10-19. §3's yaw plants (t2, t4, t5) and the borrowed lag (39–42 ms / 13–14 ms) are the old
  ESC and props. Clean FETtec roll/pitch fits: `tau2` 18.6–33.6 ms, delay 13.6–21.8 ms, total
  `tau2 + delay` 40–49 ms (brisket-f1..f4).
- **Roll/pitch are not at 45° on the FETtec** (brisket-f1: PM ~39°, GM 4.8 dB, same gains). A roll
  set P/I 0.125, D 0.0027 waits for validation. The user chose to do yaw first, on the current
  roll/pitch gains.
- **The repo.** The operating guide moved from CLAUDE.md to `GUIDE.md` (§2.2, §8, §9, §10 there;
  there is no §12); the suites are `python tests/run_all.py` (16 files) and
  `python tools/ci_local.py`; the work above was committed and released (2.1.0). Logs are named
  by label only (`tests/logmap.local.json`): §4 A step 2's "15-32, 18-50, 12-42" are t2, t4, t5.
  `LOG_DIR_LARGE` moved off Drive.

### A false limit cycle (fixed first)

On brisket-f3 and f5 a 10–12 dB line at 34.5 Hz in the yaw tracking error read as a limit cycle and
cut yaw P 0.18 → 0.072 at confidence 0.70, the opposite direction to this plan. Its output
amplitude through `|C|` (FLTE 2 Hz: −25 dB) is 1.1e-5, ~45× below one DShot step. `oscillation()`
now requires `|C(f)|` × the line ≥ `osc_min_out_amp` (5e-4); below it the note calls it a
disturbance. Commit 7f69ce8.

### D — the confined-area flight (done: brisket-y1)

D step 1 confirmed: in Loiter/AltHold the yaw rate target is shaped by `ATC_ACCEL_Y_MAX` in
`attitude_command_model` (`input_euler_angle_roll_pitch_euler_rate_yaw_rad`, ArduPilot master
`AC_AttitudeControl.cpp`); `PILOT_Y_RATE_TC` 0 adds no jerk limit; the heading-hold correction is
`sqrt_controller(..., constrain(ACC_Y/2, 10, 120) deg/s²)` — 120 at both 270 and 600; the rate
controller (`rate_controller_run_dt`, `AC_AttitudeControl_Multi.cpp`) never reads it. The logs
agree: |dTar/dt| never exceeds ~2 × 270 deg/s², and a 200 deg/s stick step is a 0.7 s ramp.

brisket-y1: `ATC_ACCEL_Y_MAX` 27000 → 60000 for one flight, yaw-only Loiter (30 s hover, 10
pulses, 10 doublets, 6 × 5 s wiggles, 15 s hover), pack 32.6 V at take-off. Predicted coherent band
5–6 Hz; measured **0.5–7.0 Hz**, mean coherence 0.85 (every FETtec flight before it:
`NO_COHERENCE` on yaw, band 0.5–2.5 Hz). Roll/pitch on that flight were unexcited: their fits pin.

### A — the grey-box yaw plant (built, accepted)

The reaction-torque hypothesis holds. With `tau2`/delay borrowed from roll/pitch and a zero
`(1 + tz s)`, all four flights with yaw data fit to the plan's ≤ 5° target:

| flight | ESC / props | free fit | lag borrowed, no zero | grey-box | tz |
|---|---|---|---|---|---:|
| t2 | old | 1.60 dB / 13.1° | 0.89 dB / 24.7° | 0.77 dB / 2.0° | 214 ms |
| t4 | old | 1.68 / 13.1° | 0.46 / 22.9° | 0.92 / 3.0° | 236 ms |
| t5 | old | 2.13 / 12.0° | 0.75 / 27.3° | 0.84 / 3.4° | 219 ms |
| y1 | FETtec | 2.12 / 11.4° | 1.12 / 25.6° | 0.70 / 2.5° | 197 ms |

(scratch fits with fixed `tau1` free; the tool's numbers for t2 and y1 are pinned in
`tests/test_toolkit.py`.) `tz` spans 197–236 ms (≤ 15 % of its mean) across two ESCs and two prop
sets. `tau1` runs to its 5 s ceiling on the old flights: the yaw body is near an integrator and
only k/τ1 (1140–1290 deg/s² per unit output) is identified — the same convention `pinned_params`
already applies to `tau1`. With every parameter free on y1, the grey-box puts the lag at 37.5 ms
/ 10.2 ms, close to roll/pitch; without the zero the free fit drives it to 1 ms / 0.

The consequence: the free fit read the gain margin at ~21 Hz (40+ dB); the grey-box at ~8 Hz,
**18.8 dB at P 0.18**. The extrapolation gate's withhold was right, and the old P 0.48–0.53 was
far beyond what this plant supports.

Code: `tunesim.Plant(tz=)`; `PlantModel.tz/model/lag/free_fit/corners/notes`;
`tune_ident.fit_greybox`, `sibling_lag`, `_maybe_greybox` (accept: phase ≤ 5°, magnitude no
worse than the free fit, `tz` off its bounds). Siblings are fitted **per log**, never pooled —
pooling y1 with f4 pinned f4's clean fits, and pooling y1's yaw with other flights moved its
crossover below the band. `alog tune --lag-from LOG ...` lends a lag from other flights and
nothing else.

### C — robust margins (built)

`tune_ident.robust_margins` evaluates the 3 × 3 corners of `tau2` × delay (siblings' min × 0.8 to
max × 1.2), refitting k, τ1, tz at each. A grey-box rate set passes only if every corner keeps
45° / 6 dB; then the gain margin above the band counts as measured. Margin ceilings use every
corner (`margin-45deg-6dB-robust`); margins are graded on the worst corner. Confidence: tier B's
consistency × `1 − half-width / nominal` of the siblings' total lag ±20 % (0.70 on y1; the
`tau2` × delay box itself is wider because the fits trade one for the other). B (motor lag from
ESC RPM) is **not done**.

### The recommendation (not yet flown)

`alog tune brisket-y1.bin --axes yaw --lag-from brisket-f1..f4.bin`:

- yaw **P 0.308, I 0.0308, FLTE 1.33**, confidence 0.46 ("validate before applying");
- the set as applied: PM 84.4°, GM 16.7 dB nominal; worst corner PM 80.5°, GM 12.2 dB
  (tau2 40.3 ms / delay 26.1 ms);
- robust P ceiling 0.473 at the current FLTE.

Before the tool did this, a hand analysis proposed P 0.30, I 0.030 **keeping FLTE 2**: crossover
2.05 Hz, PM 84° (worst 77°), GM 14.2 dB (worst 11.7) at ~8 Hz. Either is a partway step (§4 E1)
with the crossover inside the 7 Hz band, so the flight measures its phase margin directly. Fly it
with `ATC_ACCEL_Y_MAX` 60000 again so the excitation matches y1, full pack, same card.

### Still open

1. Fly the P ~0.30 validation and compare measured in-band PM with the prediction.
2. B: motor lag from ESC RPM (51 Hz telemetry on the FETtec), the independent check on the
   borrowed lag. Telemetry latency will contaminate the delay; `tau2` is the usable part.
3. The roll set P/I 0.125, D 0.0027 on the FETtec (roll/pitch margins ~39°).
4. Whether to keep `ATC_ACCEL_Y_MAX` 60000: the virtual AutoTune puts yaw ACC at 610–810 deg/s².
5. The angle-loop gap (§4 E5) is unchanged.
