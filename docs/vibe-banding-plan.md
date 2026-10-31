# Vibration banded by throttle (issue #18) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `check_vibration` (the `vibe` section) reports VIBE p95, max and clip counts per throttle band — idle / hover / high / touchdown — and grades the high band on its own, so punch-out vibration is visible instead of hidden under a hover-dominated whole-window p95.

**Architecture:** Lift the throttle-interpolation and hover-throttle logic out of `_current_bands` (`dflog/analysis.py:1274`) into two shared helpers, then add a `_vibe_bands` helper called from `check_vibration` (`dflog/analysis.py:322`) that masks each IMU's VIBE samples into bands, adds a `vibe_bands` table, and grades the high band. Whole-window results keep their exact names and values, so `alog compare` and every existing baseline are unaffected.

**Tech Stack:** Python 3, numpy, pandas; tests are plain functions run by `tests/_shim.py` or pytest, on synthetic logs built with `tests/synthlog.py`.

**Spec:** GitHub issue geoffreyheller/ardupilot-log-tools#18 (`gh issue view 18`).

## Global Constraints

- `RULES.md` binds: state numbers not verdicts; a band that cannot be computed is reported as such, never as a pass.
- Whole-window results (`IMU{i} VibeX p95`, …, `IMU{i} accel clipping`) and the `vibe` table are unchanged in name, columns and value.
- No new dependencies. Thresholds for grading come from the existing `T["vibe_xy"]`, `T["vibe_z"]` (`dflog/checks.py:95-96`); band edges are module constants in `dflog/analysis.py`, not registered thresholds.
- Band definitions (decided here; the issue left them approximate):
  - `idle` — outside the window, motors not stopped (`motors_stopped`), `ThO <= 0.05` (same as `check_power`).
  - `hover` — inside the window, `|ThO - ThH| <= 0.04`, ThH = median learned `CTUN.ThH` in the window, else `MOT_THST_HOVER`.
  - `high` — inside the window, `ThO >= max(VIBE_HIGH_THO, ThH + 0.10)`, `VIBE_HIGH_THO = 0.60` (the `+0.10` keeps it disjoint from hover on a heavy aircraft).
  - `touchdown` — the last `TOUCHDOWN_S = 3.0` s of the window, any throttle; it takes precedence over hover/high. Only formed when the window is longer than `2 * TOUCHDOWN_S`.
- A band needs `n >= 10` VIBE samples (1 s at the 10 Hz VIBE rate) to appear in the table or be graded.
- Clips per band = sum over the band's samples of `max(0, Clip[k] - Clip[k-1])` (the counter is cumulative; a reset must not produce a negative count).
- Commit messages end with `(#18)`, as the repo's recent history does.

## Review Focus

1. **`ThO` logged in permille (0–1000) or as `ThrOut`** — bands must still form; the shared helper keeps `_current_bands`' `> 1.5 → /1000` rule and `ThrOut` fallback. Test in Task 1.
2. **Log with VIBE but no CTUN** — whole-window results unchanged, no `vibe_bands` table, a note saying bands need CTUN. Test in Task 2.
3. **A flight with no punch-out** — no `high` row and no high-throttle result (not a SKIP: nothing was missing). Test in Task 2.
4. **`Clip` counter that resets or goes backwards** — no negative clip count in any band. Test in Task 2.
5. **Two IMUs** — bands and high-throttle results are per IMU instance, named `IMU{i} ...`. Test in Task 2.

---

### Task 1: Shared throttle helpers (refactor, behaviour-preserving)

**Files:**
- Modify: `dflog/analysis.py` (`_current_bands`, ~1274-1328; new helpers placed just above `check_vibration`)
- Test: `tests/test_checks.py`

**Interfaces:**
- Produces:
  - `_throttle_at(log, t: np.ndarray) -> np.ndarray | None` — `CTUN.ThO` (fallback `ThrOut`) interpolated onto `t`, scaled to 0–1 when the log is in permille; `None` when CTUN or both columns are absent.
  - `_hover_throttle(log, w, params: _Params) -> float` — median `CTUN.ThH` inside `w` if finite and > 0, else `params.get("MOT_THST_HOVER", np.nan)`.

- [ ] **Step 1: Write the failing test** `test_throttle_helpers_scale_permille_and_fall_back_to_param` in `tests/test_checks.py`: build a log via `_writer()` with a `CTUN` whose `ThO` is 400 (permille) and `ThH` 0 throughout, plus `PARM MOT_THST_HOVER=0.35`; assert `_throttle_at(log, np.array([50.0]))[0] == pytest.approx(0.40, abs=1e-3)` and `_hover_throttle(log, w, _Params(log)) == pytest.approx(0.35)`. Also assert `_throttle_at` on a log with no CTUN returns `None`.
- [ ] **Step 2: Run** `python tests/test_checks.py` — expect FAIL (`ImportError`/`NameError` on `_throttle_at`).
- [ ] **Step 3: Implement** both helpers and rewrite `_current_bands` to call them (its body otherwise unchanged).
- [ ] **Step 4: Run** `python tests/test_checks.py` and `python tests/test_largeprop.py` — expect all pass, including the untouched `test_current_is_banded_by_throttle` (proves the refactor preserved behaviour).
- [ ] **Step 5: Commit** — `git commit -m "analysis: share throttle and hover-throttle helpers ahead of banded vibration (#18)"`

---

### Task 2: Banded vibration in `check_vibration`

**Files:**
- Modify: `dflog/analysis.py` (`check_vibration` ~322; new `_vibe_bands`, constants `VIBE_HIGH_THO`, `TOUCHDOWN_S`, `VIBE_BAND_MIN_N = 10`)
- Modify: `tests/test_checks.py` (`_writer` gets `w.fmt(211, "VIBE", "QBfffI", "TimeUS,IMU,VibeX,VibeY,VibeZ,Clip")`; `_motor_log` gets `vibe=None`, a callable `t -> (x, y, z, clip_cumulative)` written as VIBE IMU 0 each sample, and `vibe_imus=1`)
- Test: `tests/test_checks.py`

**Interfaces:**
- Consumes: `_throttle_at`, `_hover_throttle` (Task 1); `motors_stopped(log, t, params)` (`dflog/flight.py:209`).
- Produces:
  - `_vibe_bands(log, w, i, g, params, sec) -> None` — `g` is one IMU's **unclipped** VIBE frame (idle lies outside the window).
  - Table `vibe_bands`, columns `["IMU", "band", "ThO band", "n", "VibeX p95", "VibeY p95", "VibeZ p95", "VibeX max", "VibeY max", "VibeZ max", "clips"]`, one row per IMU × band with `n >= 10`, band order idle, hover, high, touchdown.
  - Results `IMU{i} VibeX p95 (high throttle)` / `VibeY` / `VibeZ`, graded with `_grade` on `vibe_xy` / `vibe_z`, evidence `max`, `n`, `clips`, `tho_min` (the high edge used).
  - Note listing the band edges and ThH used; when CTUN is absent, the note `"Throttle bands need CTUN.ThO; not logged, so only whole-window figures are given."`

Test log for the steps below: `_motor_log(..., full=(80.0, 90.0), vibe=f)` where `f` gives 5.0 on all axes at hover, 28.0 X/Y and 10.0 Z during the burst (80–90 s), a single 40.0 X sample at 109.0 s, and `Clip` stepping +3 inside the burst and +1 at 109.0 s.

- [ ] **Step 1: Write failing tests:**
  - `test_vibration_is_banded_by_throttle` — table `vibe_bands` has rows `hover`, `high`, `touchdown` for IMU 0; hover VibeX p95 ≈ 5.0 (abs 0.1); high VibeX p95 ≈ 28.0 (abs 0.1), VibeZ p95 ≈ 10.0; whole-window result `IMU0 VibeX p95` still ≈ 5.0 (the issue's symptom is preserved, now explained); `IMU0 VibeX p95 (high throttle)` is `WARN` with `evidence["value"] ≈ 28.0`.
  - `test_touchdown_spike_is_not_blamed_on_punch_outs` — touchdown VibeX max == 40.0, high VibeX max ≈ 28.0, hover max ≈ 5.0.
  - `test_clips_are_counted_per_band` — clips: high 3, touchdown 1, hover 0; `IMU0 accel clipping` evidence value still 4.
  - `test_vibe_bands_without_ctun` — `_writer()` + VIBE only, window `method="none"`: no `vibe_bands` table, note contains `"need CTUN"`, `IMU0 VibeX p95` present.
  - `test_no_punch_out_means_no_high_band` — `_motor_log` with `full=(999.0, 999.0)`: no `high` row, no result whose name ends `(high throttle)`.
  - `test_clip_counter_reset_is_not_negative` — `Clip` drops from 7 to 0 mid-hover: every `clips` cell `>= 0`.
  - `test_vibe_bands_per_imu` — `vibe_imus=2`: rows for IMU 0 and IMU 1, and both `IMU0 … (high throttle)` and `IMU1 … (high throttle)` results.
- [ ] **Step 2: Run** `python tests/test_checks.py` — expect the seven new tests FAIL (no `vibe_bands` table).
- [ ] **Step 3: Implement** `_vibe_bands` and call it from `check_vibration` once per IMU group, passing the unclipped `log.df("VIBE")` group; build `params = _Params(log)` in `check_vibration` and call `params.note(sec)` at the end. Masks are built in band order, then `hover &= ~touchdown`, `high &= ~touchdown`.
- [ ] **Step 4: Run** `python tests/test_checks.py`, `python tests/test_cli.py`, `python tests/test_toolkit.py` — expect all pass (`test_toolkit` still 4 passed, 15 skipped on a machine without flight logs). If flight logs are available, `python tests/test_flights.py` and `python tests/test_largeprop.py` too.
- [ ] **Step 5: Smoke-test on a real punch-out log** — `python alog.py vibe "<a Brisket .bin with punch-outs>"`; expect a `vibe_bands` table whose high-band p95 is in the 20s on X/Y, matching the hand-script figures in the issue (27.2 / 27.4 / 9.9) to within the ±0.04/0.60 band-edge difference. Record the figures in the PR description; do **not** commit the log or any aircraft-specific numbers to the repo.
- [ ] **Step 6: Commit** — `git commit -m "vibe: band VIBE p95, max and clips by throttle; grade the high band (#18)"`

---

### Task 3: Documentation

**Files:**
- Modify: `SKILLS.md` (Skill 5, ~line 167-176), `README.md` (checks table row `vibe, imu`, ~line 165), `CLAUDE.md` (§ on `vibe, imu`, ~line 184)

- [ ] **Step 1:** In each, say that `vibe` now also reports idle / hover / high / touchdown bands with clips per band, that the high band is graded separately as `IMU{i} VibeX p95 (high throttle)`, and that a flight's punch-out baseline is the high-band p95 — compare that between flights, not the whole-window max. State the band edges once (in SKILLS.md) and refer to it from the others.
- [ ] **Step 2: Run** `python tests/test_docs_api.py` — expect pass (it executes the documented accessors).
- [ ] **Step 3: Commit** — `git commit -m "docs: banded vibration in SKILLS, README and CLAUDE (#18)"`, then close the issue from the final commit or PR with `Closes #18`.
