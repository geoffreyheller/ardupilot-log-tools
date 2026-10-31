# GPS consistency, time-binned accuracy, UBX2 and fix timeline (issue #14) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `check_gps` (the `gps` section) reports the seven things every GPS analysis on the development aircraft had to hand-script: GPS-vs-EKF velocity disagreement, 30 s bins from take-off, a `UBX2` front-end summary, accuracy split by current/throttle, HAcc/HDOP, take-off conditions, and a per-receiver fix timeline with distance to GPS0.

**Architecture:** Six helpers beside `_gps_accuracy` in `dflog/analysis.py`, each adding its own table/results to the `gps` Section and called from `check_gps`. Two of them return per-sample arrays (velocity error, UBX2 low flag) that the bins and power-split helpers consume, so each signal is computed once. No new messages: everything comes from `GPS`, `GPA`, `XKF1`, `XKF4`, `UBX2`, `BAT`, `CTUN`.

**Tech Stack:** Python 3, numpy, pandas; tests are plain functions run by `tests/_shim.py` or pytest, on synthetic logs built with `tests/synthlog.py`.

**Spec:** GitHub issue geoffreyheller/ardupilot-log-tools#14 (`gh issue view 14`). Calibration evidence for the thresholds below comes from the Brisket analyses of 2026-10-20 (clean, 0–15 velocity glitches per 60 s, `UBX2` mag pinned at 255), 10-21 and 10-26 (RF-degraded: 18–96 glitches per 30 s bin, mag dropping to 0) and 10-27 (fixed, 255 throughout). Those notes stay outside this public repo; only measured figures go into `reference/thresholds.md`, without the aircraft name.

## Global Constraints

- `RULES.md` binds: numbers, not verdicts; a figure that cannot be computed is `SKIP` or "not logged", never a pass.
- Every existing `gps` result name, value and table (`gps`, `gpa`) keeps its name and existing columns, so `alog compare --checks gps` across old and new logs still pairs up. The only change to an existing table is one column **appended** to `gpa`.
- New thresholds go in `dflog/checks.py::T` with a `source`, and each gets a row in `reference/thresholds.md` in the existing `| \`key\` | warn | fail | source |` form. Constants that define a window or a bin are module constants in `dflog/analysis.py`, not thresholds:
  - `GPS_VEL_GLITCH = 0.5` — m/s; a 3D-fix sample with horizontal |v_GPS − v_EKF| above this is a velocity glitch (the issue's figure).
  - `GPS_SETTLE_S = 60.0` — glitches in the first 60 s after take-off are acquisition, not graded (issue: "no clusters after the first 60 s").
  - `GPS_BIN_S = 30.0`; `GPS_BIN_MIN_S = 10.0` — a trailing partial bin is kept only if it spans at least 10 s.
  - `GPS_DEFAULT_LAG_S = 0.2` — `AP_GPS_Backend::get_lag()` default, used when neither `GPS{n}_DELAY_MS` nor `GPS_DELAY_MS` is in the log and > 0.
  - `UBX2_LOW_FRAC = 0.8` — a `UBX2` sample is "low" when min(magI, magQ) < 0.8 × the on-ground reference (255 × 0.8 ≈ 200, the cut the hand analyses used).
- New thresholds:
  - `gps_vel_glitch_pct`: `_t(5.0, 15.0, MEAS, "worst 30 s bin after the first 60 s: percent of 3D-fix samples with horizontal |vGPS - vEKF| > 0.5 m/s")`.
  - `gps_ubx2_low_pct`: `_t(5.0, 25.0, MEAS, "percent of airborne UBX2 samples with min(magI, magQ) below 0.8 x the on-ground median")`.
  Both are provisional until Task 6 step 2 checks them against real logs.
- Horizontal GPS velocity is `vN = Spd·cos(GCrs)`, `vE = Spd·sin(GCrs)`, `GCrs` in degrees from true north. The EKF velocity is `XKF1.VN/VE` of the **primary core**: the median of `XKF4.PI` over the window when `XKF4` has a `PI` column, else core 0. It is evaluated at `t − lag` (the GPS sample describes the vehicle `lag` seconds earlier).
- "Take-off" is `w.t0`. Bins run from `w.t0`. A window that has no take-off (the whole-log method or any `FALLBACK`; match the literal method strings `airborne_window` sets in `dflog/flight.py`) still gets bins, labelled from the window start, but no take-off table and no "vs take-off" figures.
- No new dependencies. Interpreter on the Windows dev machine: `python` on PATH is the Microsoft Store stub; run tests with `"%LOCALAPPDATA%\Programs\Python\Python312\python.exe"`.
- Commit messages end with `(#14)`. Do not commit logs, `.dfcache` files, or aircraft-specific numbers.

## Review Focus

1. **Receiver lag during acceleration.** GPS velocity lags the EKF by ~0.1–0.2 s; without the shift every punch-out reads as a glitch. Test in Task 1 (`test_gps_velocity_compensates_receiver_lag`).
2. **`GCrs` convention.** Degrees from north: east is 90°, west 270°. A radians or east-referenced reading turns every cruise leg into glitches. Test in Task 1 (`test_gps_velocity_course_is_degrees_from_north`).
3. **A bench log or fallback window.** There is no take-off, so take-off figures must be absent, not the log start mislabelled. Test in Task 3 (`test_gps_takeoff_not_reported_without_takeoff`).
4. **`UBX2` is logged every ~5 s.** A 30 s bin with no `UBX2` sample reads `None`, not `0 %`. Test in Task 3 (in `test_gps_bins_are_timed_from_takeoff`).
5. **ESC-telemetry current that barely moves.** A near-flat current must not produce a "high current" half from noise; the row is omitted with a note. Test in Task 4 (`test_gps_vs_power_flat_current_is_not_split`).

---

### Task 1: GPS-vs-EKF velocity consistency (issue item 1)

**Files:**
- Modify: `dflog/checks.py` (add `gps_vel_glitch_pct` after `gps_sacc`), `reference/thresholds.md` (its row beside `gps_sacc`)
- Modify: `dflog/analysis.py` (constants above; new helpers after `_gps_accuracy`; `check_gps` builds `params = _Params(log)`, calls `_gps_velocity`, ends with `params.note(sec)`)
- Modify: `tests/test_checks.py` (fixture, tests)

**Interfaces:**
- Produces:
  - `_time_bins(w, t: np.ndarray) -> list[tuple[float, float, np.ndarray]]` — `(b0, b1, mask)` per bin of `GPS_BIN_S` from `w.t0`, `b0/b1` in seconds since boot, trailing bin dropped if shorter than `GPS_BIN_MIN_S`.
  - `_gps_lag_s(params: _Params, i: int) -> tuple[float, bool]` — `(lag, assumed)`; reads `GPS{i+1}_DELAY_MS` then `GPS_DELAY_MS` via `in params` (not `params.get`, so nothing is recorded as defaulted).
  - `_gps_vel_error(log, w, i: int, g: pd.DataFrame, params) -> dict | None` — keys `t` (np.ndarray, 3D-fix samples in `w`), `err` (np.ndarray, m/s), `core` (int), `core_from` (`"XKF4.PI"` or `"core 0 (no XKF4.PI)"`), `lag_s`, `lag_assumed`. `None` when `XKF1` is missing or `GPS` has no `Spd`/`GCrs`.
  - `_gps_velocity(log, w, gps: dict, params, sec) -> dict[int, dict]` — adds per instance the result `GPS{i} velocity vs EKF`, graded with `_grade(worst_bin_pct, "gps_vel_glitch_pct", ...)` over bins with `b0 >= w.t0 + GPS_SETTLE_S`, evidence `n_over`, `n_late`, `max`, `t_max`, `worst_bin` (`[b0, b1]`), `core`, `lag_s`. Summary format: `"worst 30 s bin after the first 60 s: {v}% of samples > 0.5 m/s (warn {w}, fail {f}); N in the window, max X m/s at T s; vs XKF1 core C, GPS lag L s"` (with "assumed" when it was). With no full bin past settle it is `SKIP` saying so; with `None` from `_gps_vel_error` it is `SKIP` naming the missing `XKF1` or `GPS.Spd`; with no 3D fix, `SKIP "no 3D fix in the window"`. Returns `{i: _gps_vel_error(...)}` for instances that had one.

Fixture changes (`tests/test_checks.py`):
- `_writer`: `GPS` format becomes `"QBBBfLLfIHff"`, `"TimeUS,I,Status,NSats,HDop,Lat,Lng,Alt,GMS,GWk,Spd,GCrs"` (missing fields encode as 0, so existing callers are unaffected); add `XKF1` (`"QBff"`, `"TimeUS,C,VN,VE"`), `XKF4` (`"QBB"`, `"TimeUS,C,PI"`), `UBX2` (`"QBbBbB"`, `"TimeUS,Instance,ofsI,magI,ofsQ,magQ"`) on unused type ids.
- `_gps_log` gains keyword arguments, all defaulting to today's behaviour plus a stationary EKF. Each of `vel`, `ekf`, `ekf1` takes either a constant tuple or a callable of `t`; a callable `ubx2` (Task 2) returning `None` writes no record at that time: `vel=lambda t: (0.0, 0.0)` → GPS0 `(Spd, GCrs)`; `ekf=lambda t: (0.0, 0.0)` → `XKF1` C=0 `(VN, VE)` at 10 Hz, `ekf=False` omits `XKF1`; `ekf1=None` → a second core C=1 when given; `pi=None` → `XKF4` with that `PI` for both cores when given; `params={}` → `_params(w, **params)`.

- [ ] **Step 1: Write the failing tests** (window `airborne_window(log, method="ev")` = 10–110 s unless stated):
  - `test_gps_velocity_agrees_with_ekf` — defaults: `GPS0 velocity vs EKF` is `PASS`, `evidence["n_over"] == 0`, `evidence["core"] == 0`; `GPS1 velocity vs EKF` is `SKIP` with `"no 3D fix"` in the summary.
  - `test_gps_velocity_glitches_after_settle_are_graded` — `vel` returns `(0.8, 0.0)` for 30.0 ≤ t ≤ 30.95 and `(1.2, 0.0)` for 80.0 ≤ t ≤ 82.95: `n_over == 40`, `n_late == 30`, `max ≈ 1.2` (abs 1e-3), `t_max ≈ 80.0` (abs 0.25), `worst_bin == [70.0, 100.0]`, `value ≈ 10.0` (abs 0.5), status `WARN`. With the late span widened to 80.0–86.95: `FAIL`.
  - `test_gps_velocity_uses_the_primary_ekf_core` — `vel=(2.0, 0.0)`, `ekf=(0.0, 0.0)`, `ekf1=(2.0, 0.0)`, `pi=1`: `n_over == 0`, `core == 1`. Same log without `pi`: `core == 0` and `n_over > 0`, and the summary says `no XKF4.PI`.
  - `test_gps_velocity_course_is_degrees_from_north` — `vel=(5.0, 90.0)`, `ekf=(0.0, 5.0)`: `n_over == 0`; `vel=(5.0, 270.0)`, `ekf=(0.0, -5.0)`: `n_over == 0`.
  - `test_gps_velocity_compensates_receiver_lag` — EKF VN ramps `3·(t − 20)` for 20 ≤ t ≤ 25 (0 before, 15 after), GPS `Spd` the same ramp delayed 0.2 s, `GCrs` 0: `n_over == 0`, `max < 0.05`, `lag_s == 0.2`, summary contains `"assumed"`. Same with a 0.1 s delay and `params={"GPS1_DELAY_MS": 100}`: `n_over == 0`, `lag_s == 0.1`.
  - `test_gps_velocity_skips_without_xkf1` — `ekf=False`: `SKIP`, `"XKF1"` in the summary.
  - `test_gps_vel_threshold_is_registered_with_a_row` — `T["gps_vel_glitch_pct"]` warn 5.0, fail 15.0, non-empty source; `reference/thresholds.md` has a line starting ``| `gps_vel_glitch_pct` |``.
- [ ] **Step 2: Run** `python tests/test_checks.py` — expect the seven new tests FAIL and the 36 existing tests pass.
- [ ] **Step 3: Implement** the constants, the threshold and its row, the four helpers, and the call in `check_gps`. EKF velocity at GPS times is `np.interp(t - lag, xkf1_core["t"], VN/VE)`.
- [ ] **Step 4: Run** `python tests/test_checks.py` and `python tests/test_cli.py` — expect all pass.
- [ ] **Step 5: Commit** — `git commit -m "gps: GPS-vs-EKF velocity consistency on the primary core, graded per 30 s bin after settle (#14)"`

---

### Task 2: `UBX2` front-end summary (issue item 3)

**Files:**
- Modify: `dflog/checks.py` (`gps_ubx2_low_pct`), `reference/thresholds.md` (row)
- Modify: `dflog/analysis.py` (new `_ubx2_low`, `_gps_ubx2`; `check_gps`'s existing `if log.has("UBX2")` block moves into `_gps_ubx2` with its identity note text unchanged)
- Modify: `tests/test_checks.py` (`_gps_log(ubx2=None)`: a callable `t -> (magI, magQ)` written as `UBX2` Instance 0 every 5 s from t = 0, `ofsI = ofsQ = 0` unless the callable returns 4 values `(ofsI, magI, ofsQ, magQ)`)

**Interfaces:**
- Produces:
  - `_ubx2_low(log, w, i: int) -> dict | None` — keys `t` (airborne sample times), `low` (bool array), `ref` (on-ground reference: median of min(magI, magQ) over samples before `w.t0` and after `w.t1`), or `None` when instance `i` has no `UBX2` or no on-ground samples.
  - `_gps_ubx2(log, w, sec) -> dict[int, dict]` — table `ubx2`, columns `["gps", "n", "magI min", "magI med", "magQ min", "magQ med", "ofsI range", "ofsQ range", "min(magI,magQ)", "at (s)", "before take-off med", "airborne med", "after landing med", "airborne low %"]` (medians of min(magI, magQ); ranges as `"lo..hi"`); result `GPS{i} RF front end (UBX2)` graded on airborne low % with `gps_ubx2_low_pct`, or `SKIP "no on-ground UBX2 to reference"`; a note saying `UBX2` is u-blox MON-HW2 I/Q magnitude and offset, healthy units sit pinned near 255, and it is logged about every 5 s so it cannot be lined up glitch by glitch. Returns `{i: _ubx2_low(...)}` for instances that have one.

- [ ] **Step 1: Write the failing tests:**
  - `test_ubx2_collapse_airborne_is_flagged` — mag 255 except `(0, 0)` for 40 ≤ t ≤ 90: `ubx2` row GPS0 `"before take-off med" == 255`, `"airborne med"` < 255, `"after landing med" == 255`, `"min(magI,magQ)" == 0`, `"at (s)" ≈ 40.0`; `GPS0 RF front end (UBX2)` is `FAIL` with `value ≈ 52.4` (abs 1.0; 11 of 21 airborne samples).
  - `test_ubx2_steady_passes` — constant 255: `PASS`, value 0.
  - `test_ubx2_without_ground_reference_is_skip` — `ubx2` returns `None` for t < 10 or t > 110 (no record written): result `SKIP`, table row still present.
  - `test_ubx2_identity_note_is_kept` — any `UBX2`: notes still contain `"emitted only by the u-blox driver"`.
- [ ] **Step 2: Run** `python tests/test_checks.py` — expect the four new tests FAIL.
- [ ] **Step 3: Implement.** Resolve columns with `log.column("UBX2", "magI")` etc. so a spelling drift raises the named `KeyError`.
- [ ] **Step 4: Run** `python tests/test_checks.py` — expect all pass.
- [ ] **Step 5: Commit** — `git commit -m "gps: summarise UBX2 front-end magnitude before, during and after flight (#14)"`

---

### Task 3: Time bins, take-off conditions, HAcc/HDOP (issue items 2, 5, 6)

**Files:**
- Modify: `dflog/analysis.py` (`_gps_accuracy` appends one column; new `_has_takeoff`, `_gps_bins`, `_gps_takeoff`)
- Modify: `tests/test_checks.py`

**Interfaces:**
- Consumes: `_time_bins`, the `dict[int, dict]` from `_gps_velocity` (Task 1), the `dict[int, dict]` from `_gps_ubx2` (Task 2).
- Produces:
  - `gpa` table: new last column `"HAcc/HDOP med"` — median over reported 3D-fix GPA samples of `HAcc / HDop`, with `GPS.HDop` of the same instance interpolated onto `GPA` times and samples with `HDop <= 0` dropped; `None` where the row has no reported accuracy.
  - `_has_takeoff(w) -> bool` (see Global Constraints).
  - `_gps_bins(log, w, gps, vel: dict, ubx: dict, sec) -> None` — table `gps{i}_bins` per instance with any 3D fix in `w`, columns `["from (s)", "to (s)", "n", "sats min", "HAcc med (m)", "SAcc p95 (m/s)", "vel glitches", "UBX2 low %"]`; from/to are seconds after `w.t0`; `vel glitches` is `None` when the instance has no velocity error; `UBX2 low %` is `None` when the bin holds no `UBX2` sample or the instance has none. One note says what the bins are timed from (take-off, or the window start when `_has_takeoff` is False).
  - `_gps_takeoff(log, w, gps, sec) -> None` — when `_has_takeoff(w)`: table `gps_takeoff`, columns `["gps", "take-off (s from log start)", "status", "sats", "HAcc (m)", "HDOP"]`, values from the last sample at or before `w.t0` per instance (`HAcc` from `GPA`, `None` if not logged or 0). Log start is `log.duration()[0]`. Otherwise one note: `"No take-off in this window (<method>): take-off conditions not reported."`

- [ ] **Step 1: Write the failing tests:**
  - `test_gpa_table_has_hacc_per_hdop` — `_gps_log("…")` defaults: GPS0 `"HAcc/HDOP med" ≈ 1.0` (0.8/0.8), GPS1 `None`.
  - `test_gps_bins_are_timed_from_takeoff` — HAcc spike `(50.0, 52.0, 3.0)`, the Task 1 late velocity span 80.0–82.95, and a `ubx2` that writes 255 for t < 10 and t > 110, `(0, 0)` for 80 ≤ t ≤ 85, and nothing otherwise: `gps0_bins` from-column `[0, 30, 60, 90]` (the 100–110 s tail is 10 s, so kept); vel glitches `[0, 0, 30, 0]`; sats min 12 and HAcc med 0.8 in every row except HAcc in the 30–60 row, which holds the spike (assert ≥ 0.8); `UBX2 low %` exactly `[None, None, 100.0, None]`, so a bin with no `UBX2` sample is never `0`.
  - `test_gps_takeoff_conditions` — `gps_takeoff` GPS0 take-off ≈ 10.0 (abs 0.2), status 3, sats 12, HAcc 0.8; GPS1 status 1, HAcc `None`.
  - `test_gps_takeoff_follows_the_analysed_flight` — `Window(40.0, 110.0, "test")`: GPS0 take-off ≈ 40.0, and `gps0_bins` first row's from-column is 0 with its bin starting at 40 s since boot.
  - `test_gps_takeoff_not_reported_without_takeoff` — `airborne_window(log, method="none")`: no `gps_takeoff` table, note contains `"No take-off in this window"`; `gps0_bins` still present.
- [ ] **Step 2: Run** `python tests/test_checks.py` — expect the five new tests FAIL.
- [ ] **Step 3: Implement** and call `_gps_bins` and `_gps_takeoff` from `check_gps` after `_gps_ubx2`.
- [ ] **Step 4: Run** `python tests/test_checks.py` — expect all pass, including the untouched `test_gps_check_reports_receiver_accuracy_per_instance` (the appended column must not disturb the existing ones).
- [ ] **Step 5: Commit** — `git commit -m "gps: 30 s bins from take-off, take-off conditions and HAcc/HDOP (#14)"`

---

### Task 4: Accuracy against current or throttle (issue item 4)

**Files:**
- Modify: `dflog/analysis.py` (new `_gps_vs_power`)
- Modify: `tests/test_checks.py` (`_gps_log(curr=None)`: a callable `t -> A` written as `BAT` Inst 0 at 10 Hz with `Volt` 25.0)

**Interfaces:**
- Consumes: `vel` dict from Task 1; `_throttle_at(log, t)` (existing, `dflog/analysis.py:322`).
- Produces: `_gps_vs_power(log, w, gps, vel: dict, sec) -> None` — table `gps_vs_power`, columns `["gps", "split on", "split at", "n low", "n high", "HAcc med low (m)", "HAcc med high (m)", "vel glitches low", "vel glitches high"]`, one row per instance with reported HAcc. Split source: the lowest `BAT` instance with finite `Curr` in `w` (`"BAT{k}.Curr (A)"`), else `CTUN.ThO` via `_throttle_at` (`"CTUN.ThO"`). Split value: the median of the source interpolated onto that instance's 3D-fix GPA times in `w`; low is `<=`, high is `>`. HAcc halves use GPA times; glitch halves use `vel[i]["t"]`. A row whose high half has fewer than 10 GPA samples is omitted and the note says the source never rose meaningfully above its median. No grade: a note says this tests for interference from the power system and that an offset in the current reading does not move a median split. With neither source: no table, note `"needs BAT.Curr or CTUN.ThO"`.

- [ ] **Step 1: Write the failing tests:**
  - `test_gps_accuracy_is_split_by_current` — `curr` 10 A, 30 A for 80.0 ≤ t ≤ 82.95; HAcc spike `(80.0, 82.95, 3.0)`; late velocity span 80.0–82.95: GPS0 row `"split on" == "BAT0.Curr (A)"`, `"split at" ≈ 10.0`, `"HAcc med high (m)" ≈ 3.0`, `"HAcc med low (m)" ≈ 0.8`, `"vel glitches high" == 30`, `"vel glitches low" == 0`.
  - `test_gps_vs_power_flat_current_is_not_split` — `curr` 10 A throughout: no GPS0 row; note mentions the median.
  - `test_gps_vs_power_needs_a_source` — no `curr`, no `CTUN`: no `gps_vs_power` table, note contains `"needs BAT.Curr or CTUN.ThO"`.
- [ ] **Step 2: Run** `python tests/test_checks.py` — expect the three new tests FAIL.
- [ ] **Step 3: Implement** and call it from `check_gps` after `_gps_bins`.
- [ ] **Step 4: Run** `python tests/test_checks.py` — expect all pass.
- [ ] **Step 5: Commit** — `git commit -m "gps: HAcc and velocity glitches split by current or throttle (#14)"`

---

### Task 5: Per-receiver fix timeline and distance to GPS0 (issue item 7)

**Files:**
- Modify: `dflog/analysis.py` (new `_gps_fix_timeline`)
- Modify: `tests/test_checks.py` (`_gps_log(fix1=())`: spans `[(t0, t1)]` where GPS1 writes `Status 3`, 8 sats, and GPS0's position offset 3.0 m east; outside them it writes today's no-fix record)

**Interfaces:**
- Produces: `_gps_fix_timeline(log, w, gps, sec) -> None`, over the **whole log** (acquisition happens before take-off):
  - Table `gps_fix`, columns `["gps", "first 3D fix (s from log start)", "vs take-off (s)", "outages", "longest (s)", "no fix after first (s)", "dist to GPS0 med (m)", "dist p95 (m)"]`. An outage is a run of `Status < 3` after the first 3D fix; one still open at log end ends at the last sample. `vs take-off` is `first_fix − w.t0` (negative = before), `None` when `_has_takeoff(w)` is False. Distance is for `i > 0` only: equirectangular metres between this instance and GPS0 (GPS0 lat/lng and status interpolated onto this instance's times), over samples where both have a 3D fix.
  - Result `GPS{i} first 3D fix` per instance, source `"issue #14"`: `PASS` when first fix ≤ `w.t0` (or no take-off), `WARN` `"first 3D fix X s after take-off"`, `WARN` `"never got a 3D fix in the log"`.
  - Note listing each instance's outages as `"GPS1 outages: 60.0-65.0 s (5.0 s)"`, the first 10, then `"+N more"`.

- [ ] **Step 1: Write the failing tests:**
  - `test_fix_timeline_secondary_receiver` — `fix1=[(25.0, 60.0), (65.0, 120.0)]`: GPS1 first fix ≈ 25.0 (abs 0.2), vs take-off ≈ 15.0, outages 1, longest ≈ 5.0 (abs 0.2), dist med ≈ 3.0 (abs 0.1); `GPS1 first 3D fix` is `WARN` with `"after take-off"` in the summary; GPS0 first fix ≈ 0.0, `PASS`, dist `None`.
  - `test_fix_timeline_never_fixed` — defaults (GPS1 never fixes): `GPS1 first 3D fix` is `WARN` with `"never"`; its distance cells `None`.
  - `test_fix_timeline_outage_open_at_log_end` — `fix1=[(5.0, 100.0)]`: outages 1, longest ≈ 20.0 (abs 0.2).
- [ ] **Step 2: Run** `python tests/test_checks.py` — expect the three new tests FAIL.
- [ ] **Step 3: Implement** and call it from `check_gps` last, before `params.note(sec)`.
- [ ] **Step 4: Run** `python tests/test_checks.py`, `python tests/test_cli.py`, `python tests/test_docs_api.py`, `python tests/test_toolkit.py` — expect all pass (`test_toolkit` 4 passed, 15 skipped without `LOG_DIR`).
- [ ] **Step 5: Commit** — `git commit -m "gps: per-receiver fix timeline, outages and distance to GPS0 (#14)"`

---

### Task 6: Real-log calibration and documentation

**Files:**
- Modify (only if step 2 says so): `dflog/checks.py`, `reference/thresholds.md`
- Modify: `SKILLS.md` (Skill 8, ~line 462), `README.md` (checks table row `compass, power, gps, cpu`, ~line 171), `CLAUDE.md` (§4, the `gps` part of the `compass, power, gps, cpu` bullet, ~line 218), `reference/messages.md` (`UBX2` row ~line 153: fields `Instance ofsI magI ofsQ magQ`, MON-HW2, ~0.2 Hz; and that the `gps` check reads `XKF4.PI` for the primary core), plus any table list in `reference/json-output.md` that enumerates `gps` tables (`grep -n gpa reference/json-output.md`).

- [ ] **Step 1: Smoke run** `python alog.py gps "<log>.bin"` on the Brisket logs of 2026-10-20, 10-21, 10-26 and 10-27 (from the `Logs\` folder of the Brisket project on whichever machine holds them; use the `.bin`, not the `.log`). Expect no exception, and on 10-26 a `gps0_bins` table whose late bins (≈ 630–750 s after boot) carry the cluster: tens of glitches per bin, HAcc med rising to ~2 m, `UBX2 low %` non-zero. Counts may sit below the hand analysis because of the lag shift; say so if they do.
- [ ] **Step 2: Check the provisional grades separate the flights.** Expected `GPS0 velocity vs EKF`: 10-20 PASS or WARN, 10-21 and 10-26 FAIL, 10-27 PASS. Expected `GPS0 RF front end (UBX2)`: 10-20 and 10-27 PASS, 10-21 and 10-26 WARN or FAIL. If either threshold misclassifies, move it to the midpoint between the measured clean and degraded figures. Record the measured figures in its `reference/thresholds.md` row as "clean flights X–Y %, RF-degraded flights Z–W %", with no aircraft name, and re-run `python tests/test_checks.py` (the Task 1 test pins 5/15 and is updated with the threshold).
- [ ] **Step 3: Documentation.** In SKILLS.md Skill 8, add: velocity vs EKF is the first line to read for in-flight RF or supply noise (sats and fix can both look fine), with the per-bin grade and the 60 s settle; the `gps{i}_bins` table separates acquisition at take-off from mid-flight degradation; `UBX2` magnitudes collapsing airborne and recovering on the ground point at self-interference; `gps_vs_power` tests the power system directly; HAcc/HDOP is the figure to compare across antenna or ground-plane changes; take-off conditions say whether the receiver had converged (wait for HAcc < 1 m); `gps_fix` answers "did the secondary ever converge" where "% without fix" cannot. Add one line to each of README and CLAUDE.md pointing to Skill 8.
- [ ] **Step 4: Run** `python tests/test_docs_api.py` and `python tests/test_tune_docs.py` — expect pass.
- [ ] **Step 5: Commit** — `git commit -m "docs: GPS velocity consistency, bins, UBX2 and fix timeline in SKILLS, README, CLAUDE (#14)"`. Put the step 1–2 figures in the PR description, not the repo, and close with `Closes #14`.
