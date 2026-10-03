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
