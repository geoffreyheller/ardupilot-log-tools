# PID tuning from logs — sources, constants and prior art

Surveyed 2026-09-16 by reading source, not documentation: ArduPilot `master`
(`libraries/AC_AutoTune`, `AC_PID`, `AC_AttitudeControl`, `Filter/SlewLimiter`,
`ArduCopter/{Copter,rate_thread,mode_systemid,mode_autotune}.cpp`, the `VTOL-quicktune.lua`
applet), the `ardupilot_wiki` `.rst` sources, Mission Planner `ConfigInitialParams.cs`,
PID-Analyzer (Plasmatree), ArduPilot WebTools `PIDReview` and `AnalyticTune`, PX4
`mc_autotune_attitude_control` + `flight_review`, INAV `pid_autotune.c`, and the closed-loop
identification / VRFT literature. This file exists so nobody derives any of it twice. The
implementation plan that rests on it is `docs/pid-tuning-plan.md`.

Where a value could not be read from source it is marked **inferred**. Values differ between
firmware branches; the branch is named where it matters.

---

## 1. ArduCopter AutoTune (multicopter) — how the firmware computes gains

`libraries/AC_AutoTune/AC_AutoTune_Multi.cpp` (master, 2026-09). It is a **twitch search**,
not a model fit: it steps a gain by 5 %, flies a body-rate or angle step, measures peak and
bounce-back, and moves the gain until the response meets a criterion set by
`AUTOTUNE_AGGR`. Nothing is identified; the aircraft is the model.

### 1.1 Parameters

| param | default | range | meaning |
|---|---|---|---|
| `AUTOTUNE_AXES` | 7 | bitmask 1 roll, 2 pitch, 4 yaw (tunes FLTE), 8 yaw-D | axes to tune |
| `AUTOTUNE_AGGR` | 0.075 | 0.05–0.10 (constrained 0.05–0.2 in code) | bounce-back fraction used to size D; also sets overshoot allowance |
| `AUTOTUNE_MIN_D` | 0.0005 | 0.0001–0.005 | minimum rate D |
| `AUTOTUNE_GMBK` | 0.25 | 0.0–0.5 | fraction by which tuned P and D (and angle P) are reduced when a step completes (master; see §1.6 for 4.x) |

### 1.2 Constants (`AC_AutoTune_Multi.cpp`, quoted)

```
AUTOTUNE_TESTING_STEP_TIMEOUT_MS      2000
AUTOTUNE_RD_STEP / RP_STEP / SP_STEP  0.05          gain step, multiplicative
AUTOTUNE_PI_RATIO_FOR_TESTING         0.1           I = 0.1 P while testing
AUTOTUNE_PI_RATIO_FINAL               1.0           I = P when saved (roll, pitch)
AUTOTUNE_YAW_PI_RATIO_FINAL           0.1           I = 0.1 P when saved (yaw)
AUTOTUNE_RD_MAX                       0.200
AUTOTUNE_RP_MIN / RP_MAX              0.01 / 2.0
AUTOTUNE_SP_MIN / SP_MAX              0.5 / 40.0
AUTOTUNE_RLPF_MIN / RLPF_MAX          1.0 / 5.0     yaw FLTE search range, Hz
AUTOTUNE_FLTE_MIN                     2.5           yaw FLTE seed when it was 0
AUTOTUNE_RP_ACCEL_MIN                 4000          cdeg/s^2 floor for ATC_ACC_R/P_MAX
AUTOTUNE_Y_ACCEL_MIN                  1000          cdeg/s^2 floor for ATC_ACC_Y_MAX
AUTOTUNE_Y_FILT_FREQ                  10.0          gyro LPF used while tuning yaw FLTE
AUTOTUNE_D_UP_DOWN_MARGIN             0.2           peak must reach 80 % of target to count
AUTOTUNE_ACCEL_RP_BACKOFF / Y_BACKOFF 1.0
AUTOTUNE_TARGET_RATE_RLLPIT_CDS       18000         180 deg/s rate step
AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS   4500
AUTOTUNE_TARGET_RATE_YAW_CDS          9000
AUTOTUNE_TARGET_MIN_RATE_YAW_CDS      1500
AUTOTUNE_TARGET_ANGLE_MAX_RP_SCALE    1/2 of ATC_ANGLE_MAX   (min 1/3)
AUTOTUNE_TARGET_ANGLE_MAX_Y_SCALE     1.0                    (min 1/6)
AUTOTUNE_ANGLE_ABORT_RP_SCALE         2.5/3
AUTOTUNE_ANGLE_NEG_RP_SCALE           1/5
AUTOTUNE_SUCCESS_COUNT                4             (AC_AutoTune.h) consecutive passes to finish a step
AUTOTUNE_LEVEL_ANGLE_CD 250, LEVEL_RATE_RP_CD 500, LEVEL_RATE_Y_CD 750, REQUIRED_LEVEL_TIME_MS 250
```

### 1.3 Sequence

Per axis, in order roll, pitch, yaw(E), yaw-D:
`RATE_D_UP (0) → RATE_D_DOWN (1) → RATE_P_UP (2) → ANGLE_P_DOWN (4) → ANGLE_P_UP (5) → TUNE_COMPLETE (8)`.
`TuneType` enum: 0 RATE_D_UP, 1 RATE_D_DOWN, 2 RATE_P_UP, 3 RATE_FF_UP, 4 ANGLE_P_DOWN,
5 ANGLE_P_UP, 6 MAX_GAINS, 7 TUNE_CHECK, 8 TUNE_COMPLETE. `AxisType`: 0 roll, 1 pitch,
2 yaw, 3 yaw-D.

State machine: WAITING_FOR_LEVEL (intra-test gains: original P/D/FF/filters with
`I = 0.1 P`; must be level for 250 ms, level thresholds relax up to 2× over 2 s) →
EXECUTING_TEST (test gains, `ATDE` streamed) → UPDATE_GAINS (`ATUN` written, rule applied,
`success_counter >= 4` ends the step). Direction alternates every twitch.

**Test gains** (`load_test_gains`): sqrt controller off, `P = tune_rp`, `I = 0.01 P`,
`D = tune_rd`, `FF = 0`, `D_FF = 0`, `FLTT = 0`, `SMAX = 0`, `ANG_P = tune_sp`.
Yaw(E): `D = 0`, `FLTE = tune_yaw_rLPF`. Yaw-D: `D = tune_yaw_rd`.

### 1.4 The twitch and what is measured

Targets (cdeg, cdeg/s):
```
target_max_rate = MAX(4500, step_scaler * 18000)
target_rate     = constrain(deg(max_rate_step_bf_axis()) * 100, 4500, target_max_rate)
target_angle    = constrain(deg(max_angle_step_bf_axis()) * 100, ANGLE_MAX/3, ANGLE_MAX/2)
yaw: rate 0.75 * max_rate_step, in [1500, max(1500, step_scaler*9000)]; angle in [ANGLE_MAX/6, ANGLE_MAX]
```
`max_rate_step_bf_roll()` (`AC_AttitudeControl.cpp`) — the rate step that saturates the
rate controller in 4 loops:
```
alpha = MIN(filt_E_alpha(dt), filt_D_alpha(dt));  throttle_hover = constrain(hover, 0.1, 0.5)
rate_max = 2 * throttle_hover / ( ((1-alpha)^3 * alpha * kD) / dt + kP )      [rad/s], capped by ATC_RATE_R_MAX
```
`max_angle_step_bf_*` = `max_rate_step / angle_P` (**inferred**, not read).

Command shape: rate steps feed `input_rate_step_bf_roll_pitch_yaw_rads(dir * (target_rate + start_rate))`
every loop — a held body-rate step straight into the rate PID, attitude target reset,
feed-forward zero. Angle steps rotate the attitude target once by `target_angle`, then
command zero rate, so the angle-P loop (sqrt controller off) drives the response.

Measurement (`twitching_test_rate`): gyro through an LPF at `2 × FLTD` (10 Hz for
yaw(E)); `test_rate_max` = running max; `test_rate_min` = minimum after the peak once
`max > 0.25 target` (**bounce-back**). Ends when `max > target`, or
`max - min > max * AGGR`, or timeout. Early stop while `max < 0.6321 target`:
`step_timeout = 3 × elapsed`, capped 2000 ms. Abort on lean beyond `ANGLE_MAX/2`
(`step_scaler *= 0.9`; below 0.2 → "Twitch Size Determination Failed").
`twitching_measure_acceleration`: `accel = 1000 * rate_max / (now - step_start_ms)` in
cdeg/s², maximum kept as `test_accel_max_cdss` — the source of `ATC_ACC_*_MAX`.

Angle tests use `target_angle * (1 + 0.5 AGGR)`, min tracked after 25 % of target, plus
rate max/min; abort when `lean <= -ANGLE_MAX/5` or lean > `2.5/3 ANGLE_MAX`.

### 1.5 Gain update rules (every step ×/÷ 1.05)

- **RATE_D_UP** (`D ∈ [MIN_D, 0.2]`, `P ∈ [0.01, 2.0]`): peak > target → `P -= 5 %`
  (P at 0.01 → `D -= 5 %`; D at min → step success "Min Rate D limit reached").
  Peak < 0.8 target → `P += 5 %`. Otherwise (peak in 80–100 % of target): bounce
  `max - min > max * AGGR` → success++; else success-- and `D += 10 %` (D ≥ 0.2 → forced
  success). Result: the D that first produces bounce-back ≥ AGGR × peak.
- **RATE_D_DOWN**: same P management; bounce `< max * AGGR` → success++, else `D -= 5 %`.
  Result: the largest D with bounce just below AGGR.
- **RATE_P_UP** (test target `target_rate * (1 + 0.5 AGGR)`): peak > that → success++.
  Peak in [0.8 target, target) with bounce > AGGR × peak and D > min → `D -= 5 %`
  (D at min → "Rate D Gain Determination Failed", except yaw(E)) and `P -= 5 %`.
  Else `P += 5 %` (P ≥ 2.0 → forced success). Yaw(E) moves `FLTE` (1–5 Hz) instead of D.
- **ANGLE_P_DOWN** (`SP ≥ 0.5`): angle peak < `target (1 + 0.5 AGGR)` → success++,
  else `SP -= 5 %`.
- **ANGLE_P_UP** (`SP ≤ 40`): angle peak > `target (1 + 0.5 AGGR)`, or angle peak > target
  and `rate_min < -rate_max * AGGR` → success++; else `SP += 5 %`.

So AGGR sets: bounce-back threshold = AGGR × peak (D steps); overshoot threshold =
0.5 AGGR × target (rate P and angle P).

### 1.6 Backoff, final gains, save

**Verified per branch on 2026-09-17** by fetching `libraries/AC_AutoTune/AC_AutoTune_Multi.cpp`
raw from GitHub (`https://raw.githubusercontent.com/ArduPilot/ardupilot/<branch>/...`).
The 4.6 and 4.7 release branches are named `ArduPilot-4.6` / `ArduPilot-4.7`, not
`Copter-4.x` — the `Copter-4.6` and `Copter-4.7` URLs return 404 (`git ls-remote` lists
`Copter-4.0` … `Copter-4.5`, then `ArduPilot-4.6`, `ArduPilot-4.7`). `ArduPilot-4.7`'s file
is byte-identical to master. `dflog/tune_atun.py::BACKOFF_BY_FIRMWARE` encodes this table
with the URL and line numbers as `source` per row.

| branch | `AUTOTUNE_GMBK` | `RD_BACKOFF` | `RP_BACKOFF` | `SP_BACKOFF` | applied by | `(1 − AGGR)` on SP | `TESTING_STEP_TIMEOUT_MS` | lines (#define / backoff fn) |
|---|---|---|---|---|---|---|---|---|
| Copter-4.3 | no | 1.0 | 1.0 | 0.9 | `set_gains_post_tune` | no | 1000 | 60–62 / 673–721 |
| Copter-4.4 | no | 1.0 | 1.0 | 0.9 | `set_gains_post_tune` | no | 1000 | 63–65 / 710–764 |
| Copter-4.5 | no | 1.0 | 1.0 | 0.9 | `set_gains_post_tune` | no | 1000 | 67–69 / 735–789 |
| ArduPilot-4.6 | no | 1.0 | 1.0 | 0.9 | `set_gains_post_tune` | no | 2000 | 67–69 / 748–802 |
| ArduPilot-4.7 | **yes**, default 0.25, constrained 0–0.5 | — | — | — | `set_tuning_gains_with_backoff` | **yes** | 2000 | 118–123 / 909–953 |
| master (2026-09) | identical to ArduPilot-4.7 | | | | | | | |

4.3 – 4.6, `set_gains_post_tune` (quoted from Copter-4.5 lines 743–789; the other three
differ only in the yaw-D rows 4.4 added):
```
after RATE_D_DOWN: tune_x_rd = MAX(min_d, rd * AUTOTUNE_RD_BACKOFF);  tune_x_rp = MAX(RP_MIN, rp * AUTOTUNE_RD_BACKOFF)
                   (yaw(E): tune_yaw_rLPF = MAX(RLPF_MIN, rLPF * AUTOTUNE_RD_BACKOFF))
after RATE_P_UP:   tune_x_rp = MAX(RP_MIN, rp * AUTOTUNE_RP_BACKOFF)
after ANGLE_P_UP:  tune_x_sp = MAX(SP_MIN, sp * AUTOTUNE_SP_BACKOFF)
                   tune_x_accel = MAX(RP_ACCEL_MIN, test_accel_max * AUTOTUNE_ACCEL_RP_BACKOFF)   (yaw: Y_ACCEL_MIN, ACCEL_Y_BACKOFF)
```
With the constants as defined: rate P and D are saved at **100 %** of the found values,
angle P at **90 %**, and there is no aggressiveness term anywhere in the backoff. The
comments beside the `#define`s ("reduced to 50 %", "97.5 %") are stale; the values are
what count. The 4.0 `SP_BACKOFF 0.75` figure in the earlier draft of this section was not
verified and is withdrawn.

ArduPilot-4.7 / master, `set_tuning_gains_with_backoff` (lines 909–953):
```
after RATE_P_UP:   rd *= (1 - GMBK);  rp *= (1 - GMBK)                (yaw(E): rp only; yaw-D: rd and rp)
after ANGLE_P_UP:  sp *= (1 - GMBK) * (1 - AGGR)
                   accel_rp = cd_to_rad(MAX(4000, test_accel_max_cdss * 1.0));  accel_y = cd_to_rad(MAX(1000, ...))
```
`GMBK` is constrained to 0–0.5 and saved back before use. Defaults: rate P and D saved at
75 % of the found values, angle P at 0.75 × 0.925 = 69.4 %.

Which backoff a log's firmware used decides the reconstruction. Two witnesses, in order
of strength: `AUTOTUNE_GMBK` in `PARM` (a parameter the firmware wrote), then the
firmware string's major.minor against the table. And the log carries its own check: the
rate backoff runs when RATE_P_UP *completes*, so the ANGLE_P_DOWN / ANGLE_P_UP `ATUN`
rows already hold the backed-off `RP/RD`; **ANGLE rows' RP ÷ last RATE_P_UP RP is the rate
backoff the firmware actually applied**, whatever the banner says (`tune_atun` reports it
as `backoff.rate_p_observed`). Only angle P has no in-log witness.

Other per-branch facts that matter to a reader: `aggressiveness` is constrained 0.05–0.2
on every branch (4.3 line 136 … master line 308); the twitch *measurement* thresholds
differ (4.x `twitching_test_rate` tracks the minimum once the peak exceeds 0.5 × target
and applies the early stop at 0.75 × target; master uses 0.25 and 0.6321) — they change
what `Min/Max` are, not how the rules judge them.

Saved on disarm (`save_tuning_gains`, only for completed axes):
```
ATC_RAT_x_P = tune_rp;  ATC_RAT_x_I = tune_rp × 1.0 (yaw × 0.1);  ATC_RAT_x_D = tune_rd
FF, D_FF, FLTT, SMAX restored;  yaw(E): FLTE = tune_yaw_rLPF;  yaw-D: D = tune_yaw_rd
ATC_ANG_x_P = tune_sp;  ATC_ACC_x_MAX = tune_accel
ATC_RATE_FF_ENAB was 0 → set to 1 and ACC_R/P_MAX saved as 0
```

### 1.7 What AutoTune logs

```
ATUN  TimeUS,Axis,TuneStep,Targ,Min,Max,RP,RD,SP,ddt     fmt QBBfffffff   units s--ddd---o
```
One row per twitch, written in UPDATE_GAINS *before* the rule changes the gain
(`Log_AutoTune()` is the first statement of the `UPDATE_GAINS` case, `AC_AutoTune.cpp`
master line 445, before `updating_*_all()`), so `RP/RD/SP` are the gains **tested** in
that twitch and the rule's change appears in the *next* row (`RD` is FLTE in Hz when
`Axis = 2`; `Axis = 3` is yaw-D). Aborted twitches write no `ATUN` row. `Targ/Min/Max`
are ×0.01 → degrees (angle steps) or deg/s (rate steps). `ddt` is `test_accel_max_cdss`
**unscaled** (cdeg/s²) despite the unit tag; divide by 100. It is **overwritten per
test**, not max-kept: `test_init()` zeroes `accel_measure_rate_max` (master line 191) and
`twitching_measure_acceleration` rewrites `test_accel_max_cdss` on the first sample of
every test, so each row's `ddt` is that twitch's own peak acceleration.

```
ATDE  TimeUS,Angle,Rate        deg, deg/s; direction-normalised (positive = twitch direction), every loop while testing
```
`RATE` and `PIDR/PIDP/PIDY` are written every loop during a test regardless of
`LOG_BITMASK` (`log_pids()` in `mode_autotune.cpp`). `EV` ids **confirmed from
`libraries/AP_Logger/AP_Logger.h` `enum class LogEvent`** (master; the enum is not in
`LogStructure.h`): 28 NOT_LANDED, 30 AUTOTUNE_INITIALISED, 31 AUTOTUNE_OFF,
32 AUTOTUNE_RESTART, 33 AUTOTUNE_SUCCESS, 34 AUTOTUNE_FAILED, 35 AUTOTUNE_REACHED_LIMIT,
36 AUTOTUNE_PILOT_TESTING, 37 AUTOTUNE_SAVEDGAINS, 38 SAVE_TRIM. `EV 37` is written when
*any* completed axis was saved, so it does not by itself say which axes completed.
`MSG` text (`report_axis_gains`, identical on every branch): `AutoTune: <axis> Rate:
P:%0.3f, I:%0.3f, D:%0.4f`, `AutoTune: <axis> Angle P:%0.3f, Max Accel:%0.0f` (cdeg/s²
on every branch — master converts with `rad_to_cd`), `AutoTune: <axis> complete`,
`AutoTune: Saved gains for ...`; `<axis>` is `Roll`, `Pitch`, `Yaw` or `Yaw(D)`. The
printed precision matters when cross-checking: a D of 0.00079 prints as `0.0008`, 1.5 %
away from the value while being exactly the printed number.

**Reconstruction recipe (corrected 2026-09-17; the earlier text applied GMBK to the
ANGLE_P_UP row's RP/RD as well, which backs off twice).** Group `ATUN` by session (`EV
30`), then by `(Axis, TuneStep)` in time order. Re-derive each twitch's outcome from
`Targ/Min/Max` with §1.5 and the log's `AUTOTUNE_AGGR`; the ANGLE_P_UP rule's second
clause needs the twitch's rate extremes, which are not in `ATUN` — take them from the
`ATDE` burst that ends at the row (running maximum, minimum after it). Carry
`ignore_next` across steps and axes: the base state machine never resets it (UPDATE_GAINS
zeroes only `success_counter` and `step_scaler`), so the twitch after a step's fourth
pass is ignored by the next step's rule when it lands in that rule's "else" branch. Then:

```
rate P, D    = last ANGLE_P_UP row's RP, RD             (already backed off by the firmware; equal to
                                                          last RATE_P_UP row's RP, RD × rate backoff)
angle P      = last ANGLE_P_UP row's SP × SP backoff     (4.7+: (1 − GMBK)(1 − AGGR); 4.3–4.6: 0.9)
ACC_MAX      = max(floor, ddt of the last ANGLE_P_UP row) / 100  deg/s²   (floor 4000 roll/pitch, 1000 yaw, cdeg/s²)
I            = P (roll, pitch), 0.1 P (yaw);  yaw(E): FLTE = last row's RD, D unchanged
```
Cross-check against the `MSG` lines (within their printed precision) and the `PARM`
values written after `EV 37` (or at the next boot). `dflog/tune_atun.py` implements this
and reports, per (session, axis), the re-derived outcome of every row, whether the
rule-predicted gains match the next row, the assumed and the observed backoff, and the
agreement with `MSG` and `PARM`.

---

## 2. AC_PID and the PID log messages

`AC_PID::update_all(target, measurement, dt, limit, pd_scale, i_scale)`:
```
_target  += alpha_T * (target - _target)                        FLTT (optional notch NTF before)
error     = _target - measurement;  _error += alpha_E * (error - _error)      FLTE (notch NEF before)
_derivative += alpha_D * ((_error - error_last)/dt - _derivative)            FLTD, on the filtered error
_target_derivative = (_target - target_last)/dt                               for D_FF
I: if !limit or error opposes integrator: _integrator += _error * ki * i_scale * dt, clamp ±IMAX
P_out = _error * kP;  D_out = _derivative * kD;  I_out = _integrator
Dmod  = slew_limiter.modifier((P + D) * scale, dt);  P_out *= Dmod;  D_out *= Dmod   (I not scaled)
P_out, D_out *= pd_scale;  PDMX clamp on |P + D|
return P_out + D_out + I_out        (FF + DFF added separately by attitude control)
```
`alpha = calc_lowpass_alpha_dt(dt, hz)` (first order; 0 Hz = off). Defaults
(`AC_AttitudeControl_Multi.h`): roll/pitch P 0.135, I 0.135, D 0.0036, IMAX 0.5, FLTT 20,
FLTE 0, FLTD 20, SMAX 0; yaw P 0.18, I 0.018, D 0, FLTT 20, FLTE 2.5, FLTD 20. Param ranges:
RLL/PIT P 0.01–0.5, I 0.01–2.0, D 0–0.05; YAW P 0.10–2.50, I 0.01–1.0, D 0–0.02; SMAX
0–200.

```
PIDR/PIDP/PIDY/PIDA  TimeUS,Tar,Act,Err,P,I,D,FF,DFF,Dmod,SRate,Flags    fmt QffffffffffB
```
`Tar` = FLTT-filtered target, `Act` = gyro, `Err` = FLTE-filtered error, all three in
**rad/s** (the PID's own units; FMTU declares none; corrected 2026-09-30 from "deg/s"
after `PIDR.Act × 57.2958 == RATE.R` on the Brisket log `brisket-t1.bin`), `P/D`
post-Dmod, `I` integrator, `FF = _target × kff`, `DFF`, `Dmod` slew-limiter multiplier
(1.0 = never engaged), `SRate` measured output slew rate (normalised output / s).
`Flags`: bit0 LIMIT (output saturated, anti-windup active), bit1 PD_SUM_LIMIT, bit2 RESET,
bit3 I_TERM_SET. `PIDA` is the vertical acceleration PID.

SlewLimiter (`Filter/SlewLimiter.cpp`): `WINDOW_MS 300`, `MODIFIER_GAIN 1.5`,
`DERIVATIVE_CUTOFF_FREQ 25 Hz`, `N_EVENTS 2`; `SRate` = attack-filtered mean of the held
positive/negative peak slew of `d(P+D)/dt`; modifier engages after two consecutive ±
exceedances: `mod = SMAX / (SMAX + 1.5 (slew − SMAX))`. `SMAX ≤ 0` → `Dmod = 1`, `SRate`
still computed.

```
RATE  TimeUS,RDes,R,ROut,PDes,P,POut,YDes,Y,YOut,ADes,A,AOut,AOutSlew
```
`RDes` = rate-controller input (`_ang_vel_body + _sysid_ang_vel_body`, deg/s), `R` = the
gyro fed to the rate loop, `ROut = motors.get_roll() + get_roll_ff()` (normalised ±1, PID
output plus FF). In a log: `RATE.RDes → FLTT → PIDR.Tar`; `PIDR.Act × 180/π == RATE.R`
(same gyro sample; `PIDx.TimeUS` is the write time, 0.4–1.7 ms after the RATE loop tick);
`RATE.ROut == PIDR.P+I+D+FF+DFF` (± mixer limits).

`ANG` (master, loop rate): `DesRoll = _euler_angle_target` (the *shaped* target),
`Roll = ahrs.roll`, `Dt`. `ATT` is written by AHRS at 10 Hz with the same target fields
(`DesRoll, Roll, DesPitch, Pitch, DesYaw, Yaw`). Older firmware has only `ATT`.

### 2.1 Logging rates (`Copter.cpp`, `rate_thread.cpp`)

| LOG_BITMASK bits | what | rate |
|---|---|---|
| bit 0 ATTITUDE_FAST (1) | `ANG`/`ATT`, `RATE`, and `PIDx` if bit 12 | every loop (`SCHED_LOOP_RATE`, 400 Hz default); with the rate thread (`FSTRATE_ENABLE`) capped at 1 kHz |
| bit 1 ATTITUDE_MED (2) without bit 0 | `ATT`, `RATE` | 10 Hz |
| bit 12 PID (4096) without bit 0 | `PIDx` | 10 Hz |
| AutoTune / SysID modes | `RATE`, `PIDx`, `ATUN`/`ATDE`, `SIDD` | loop rate regardless of bitmask (SysID: ÷1 with FAST+MED, ÷2 FAST, ÷4 MED, ÷8 else) |

`LOG_BITMASK = 180222` (both development aircraft) = bits 1–13, 15, 17: **no bit 0**, so
`PIDR/RATE/ATT` are at 10 Hz. `180223` adds fast attitude. Fast logging roughly doubles
the log rate; on onboard-flash boards set it only for the tuning flight.

---

## 3. What `RDes` represents — attitude controller input shaping

`ATC_ANG_*_P` default 4.5 (3–12); `ATC_ACC_R/P_MAX` default 1100 deg/s² (0 disables),
`ATC_ACC_Y_MAX` 270; `ATC_RATE_*_MAX` 0; `ATC_INPUT_TC` (0.5 very soft … 0.05 very crisp;
0.15 default **inferred**); `ATC_RATE_FF_ENAB` 1. Internal accel limits 40–720 deg/s²
(yaw 10–120); thrust error angle 30°.

With `RATE_FF_ENAB = 1`: per axis `shape_angle_vel_accel` with velocity limit
`ATC_RATE_*_MAX`, acceleration limit `ACC_MAX`, **jerk limit `ACC_MAX / INPUT_TC`**, then
`attitude_controller_run_quat()`:
`ang_vel_body = sqrt_controller(err, ANG_P, constrain(ACC_MAX/2, 40, 720), dt) + feed-forward rate`.
`sqrt_controller(error, p, a, dt)`: `linear_dist = a/p²`; `|error| ≤ linear_dist → p·error`,
else `±sqrt(2a(|error| − linear_dist/2))`, clamped to `|error|/dt`. With `RATE_FF_ENAB = 0`
the target is the raw pilot angle and `RDes` is purely the angle-P correction. So the
pilot's stick never appears unshaped in a log; `ANG.DesRoll` is already jerk-limited.

---

## 4. Lua VTOL-quicktune — a relay-style ceiling search on `SRate`

Constants: `UPDATE_RATE_HZ 40`, `STAGE_DELAY 4 s`, `PILOT_INPUT_DELAY 4 s`,
`YAW_FLTE_MAX 8`, `FLTD_MUL 0.5`, `FLTT_MUL 0.5`, `DEFAULT_SMAX 50`. Params:
`QUIK_DOUBLE_TIME 10 s`, `QUIK_GAIN_MARGIN 60 %`, `QUIK_OSC_SMAX 5`, `QUIK_YAW_P_MAX 0.5`,
`QUIK_YAW_D_MAX 0.01`, `QUIK_RP_PI_RATIO 1.0`, `QUIK_Y_PI_RATIO 10`, `QUIK_MAX_REDUCE 20 %`,
`QUIK_ANGLE_MAX 10°`.

Order RLL_D, RLL_P, PIT_D, PIT_P, YAW_D, YAW_P. Start: any zero SMAX → 50;
`FLTT = FLTD = 0.5 × INS_GYRO_FILTER`; yaw FLTE ≤ 8 Hz. Each tick
`oscillating = pid_info.slew_rate > QUIK_OSC_SMAX`. Not oscillating:
`gain *= exp(ln 2 / (40 × DOUBLE_TIME))`. Oscillating: `gain *= (1 − GAIN_MARGIN/100)`
= 0.4×; if roll/pitch D ends below its original, P is scaled by `max(new/old, 0.5)` too.
When a `_P` changes and `FF == 0`: `I = P / PI_ratio`. Abort on attitude error > 10°.
**The same `PIDx.SRate` signal is in every log with the PID bit set**, so the ceiling test
can be applied offline: `SRate > 5` means the current P/D is at or above the oscillation
ceiling.

---

## 5. Frequency-domain tuning in ArduPilot (heli AutoTune, SysID, AnalyticTune)

`AC_AutoTune_Heli.cpp` / `AC_AutoTune_FreqResp`: dwells (5° sine, 6 cycles) or a 23 s
sweep `AUTOTUNE_FRQ_MIN..MAX` (10–70, used as rad/s). Gain = measured/target peak-to-peak
ratio; phase = `freq × Δt(peaks) × 360°`. Two responses: motor-command → rate and
target-rate → rate. Rules:
```
at motor→rate phase 161°:  max_allowed_P = 10^(-(20 log10(gain) + 2.42) / 20)         (≤ 2 RP_MAX)
at phase 251°:             max_allowed_D = 10^(-(20 log10(freq × gain) + 2.42) / 20)   (≤ 2 RD_MAX)
                           ("max gain to 6 dB gain margin for a unity feedback controller")
RATE_D_UP: D += 0.05 max_allowed_D while closed-loop gain at the 161° frequency keeps falling and D < 0.6 max_allowed
RATE_P_UP: P += 0.05 max_allowed_P while gain at 161° < AUTOTUNE_GN_MAX (1.0) and P < 0.6 max_allowed; back off one step
ANGLE_P_UP: SP += 0.5 until the disturbance-rejection peak gain > GN_MAX; interpolate to GN_MAX; range 3–10
FF: dwell at 0.25 Hz, adjust until rate gain = 0.95 ± 0.025;  saved I = 0.5 FF (roll/pitch), yaw I = 0.1 P
dwell_max_accel = freq × max_meas_rate × 5730 / (2 × max_command)   [cdeg/s^2]
```
Logs `ATNH` (`Axis,TuneStep,Freq,Gain,Phase,RFF,RP,RD,SP,ACC`), `ATDH`, `ATSH`.

**SysID mode** (`mode_systemid.cpp`): `SID_AXIS` 1–3 angle input, 4–6 recovery, 7–9 rate
(added inside `RATE.RDes`), 10–12 mixer (added inside `RATE.ROut`, after the PID), 13
thrust, 14–19 position controller; `SID_MAGNITUDE 15`, `SID_F_START_HZ 0.5`,
`SID_F_STOP_HZ 40`, `SID_T_FADE_IN 15`, `SID_T_REC 70`, `SID_T_FADE_OUT 2`. Logs
`SIDS TimeUS,Ax,Mag,FSt,FSp,TFin,TC,TR,TFout` once and
`SIDD TimeUS,Time,Targ,F,Gx,Gy,Gz,Ax,Ay,Az` (Targ = chirp sample, F = instantaneous Hz,
Gx.. deg/s from raw delta-angle, Ax.. m/s²). The firmware computes nothing; the wiki
(`systemid-model-development`) uses `RATE.ROut` as input and `SIDD.Gx` as output, averaged
over flights, and fits `(b1 s + b0) / (s³ + a2 s² + a1 s + a0) e^{−τs}` per axis.

**AnalyticTune** (WebTools): needs SysID chirps (`SID_AXIS` 10–12, 0.05–5 Hz over 130 s,
magnitude 0.15 roll/pitch, 0.55 yaw), FFT window 1024 at 100 Hz, **coherence as the
data-quality gate**, targets gain margin ≥ 6 dB and phase margin ≥ 45°. Interactive, not a
recommender.

---

## 6. Wiki rules and the Mission Planner initial-parameter calculator

`setting-up-for-tuning.rst`: `MOT_BAT_VOLT_MAX = 4.2 × cells`, `MIN = 3.3 × cells`;
`MOT_THST_EXPO` 0.55 (5"), 0.65 (10"), 0.75 (≥ 20"); `INS_GYRO_FILTER` 80 / 40 / 20 Hz for
5" / 10" / ≥ 20"; `ATC_ACC_R/P_MAX` 1100 / 500 / 200 deg/s² for 10" / 20" / 30",
`ATC_ACC_Y_MAX` 200 / 100 / 90; `ATC_RAT_{RLL,PIT}_FLTD = FLTT = ATC_RAT_YAW_FLTT = INS_GYRO_FILTER / 2`;
`ATC_RAT_YAW_FLTE = 2`. `autotune.rst`: "for pitch and roll P and I should be equal and D
should be 1/10th P; for yaw I should be 1/10th P and D = 0" (note the firmware default
D/P is 0.0036/0.135 = 0.027; the 1/10 figure predates the current scaling). AGGR 0.10
aggressive, 0.075 medium, 0.05 weak; ≥ 13" props set roll/pitch FLTT/FLTD to 10 Hz before
autotune. `ac_rollpitchtuning.rst` manual method: D up in 50 % steps to oscillation, down
10 % steps until clean, then a further 25 % down; same for P; `I = P`.

Mission Planner `ConfigInitialParams.cs` (`prop` in inches):
```
atc_accel_y_max = max(8000,  round100(-900 prop + 36000))                                    [cdeg/s^2]
atc_accel_p_max = atc_accel_r_max = max(10000, round100(-2.613267 prop^3 + 343.39216 prop^2 - 15083.7121 prop + 235771))
ins_gyro_filter = max(20, round(289.22 prop^-0.838));   FLTD = FLTT = max(10, gyro_filter/2);  RLL/PIT FLTE 0;  YAW FLTE 2
mot_thst_expo   = min(round2(0.15686 ln(prop) + 0.23693), 0.80)
```
(10": gyro 42 Hz, expo 0.60, accel R/P 116 700 cdeg/s², Y 27 000 — exactly the Brisket
aircraft's values. Corrected 2026-09-17: the first draft of this line said 110 700, a
transcription slip against the cubic above, which gives 116 659.8 → 116 700; the Brisket
log carries 116700 and `tune_fuse.mission_planner_initial(10)` returns it.)

---

## 7. Open-source log-based methods — what each does and what it needs

| tool | input | algorithm | output | confidence / insufficient data |
|---|---|---|---|---|
| **PID-Analyzer** (Plasmatree) | Betaflight gyro vs setpoint, ≥ 1 kHz | 1.0 s Hann frames stepped 1/16; Wiener deconvolution `G H* / (H H* + 1/sn)` with `sn = 10 (1 − mask(25 Hz) + 1e-9)`; `cumsum` of the first 0.5 s → unit step response; frames with max input < 20 deg/s dropped; split at 500 deg/s; modal (histogram-weighted) average | step-response plot 0–2 | frames < 20 deg/s ignored; high-input plot needs ≥ 10 frames; no gains |
| **PIDtoolbox** (2018 `PTstepcalc.m`) | Betaflight | stick-release detection, 400 ms windows of PID error normalised by the first-50 ms minimum; later versions Wiener like PID-Analyzer, 20 deg/s minimum | rise 10–90 %, peak, settling ±2 %, latency (50 %) | segments outside bounds rejected; no gains |
| **ArduPilot PIDReview** (WebTools) | `PIDR/P/Y Tar,Act` or `RATE` at fast rate | sets split at each PARM change; batches on sample gaps, ≥ 64 samples; Welch `H = Y X*/(X X*)` with all windows as "shadows"; step response = PID-Analyzer's JS port, windows with `TarMax < 20 deg/s` skipped | frequency response + step response | "unless there is good consistency between your shadows it's probably not trustworthy"; no gains |
| **PX4 mc_autotune** | onboard, own square-wave excitation | RLS ARX(2,2,1) with forgetting `λ = 1 − dt/60`, `P0 = 1e4 I`; converged when all `diag(P) < 50`; GMVC pole placement (`sigma = MC_AT_RISE_TIME 0.14 s`, `lbda 0.7`, `ki /= 5` admitted fudge); `att_p = clamp(1/(60° kc), 2, 6.5)`; sanity `K < 0.5, I < 10, D < 0.1` | K, I, D, att P | covariance gate; 20 s timeout → fail |
| **PX4 flight_review** | ulog fast rates | PID-Analyzer port, same constants | plots | none |
| **fpvpidlab** | Betaflight | explicit steps (slope > 500 deg/s², ≥ 150 deg/s): rise, overshoot, settling, latency, ringing, SS error; `H = S_xy/S_xx`, bandwidth, phase margin | rule-based: overshoot > 25 % → D +5..15 %; overshoot < 10 % and rise > 80 ms → P +5 %; ringing > 2 → D +5 %; SS error > 5 % → I +5 %; D/P kept 0.45–0.85 (Betaflight units) | coherence gate 0.5 over 1–30 Hz; < 3 steps → confidence downgraded; no steps → whole-flight fallback with warning |
| **Plane AP_AutoTune** | onboard | FF = median `max_actuator/(max_rate × scaler)` per demand event; `D *= 1.3` until `Dmod < 1`; then `P *= 0.35; D *= 0.75`; `I = min(P, FF/TCONST)` | FF, P, I, D | events need `max_rate ≥ 0.01 rmax`, > 100 ms |
| **INAV fixed-wing AUTOTUNE** | onboard | `FF += (|out|/|rate| × FF_MULT − FF) × 0.1` after 250 samples with stick > 50 % | FF (2.6: `P = 0.1 FF`, I from a 600 ms time constant) | stick gate |
| LogAnalyzer, dronekit-la, MAVExplorer, Mission Planner, UAVLogViewer, Methodic Configurator | — | plot or count AutoTune events only | — | no gain logic |

### 7.1 Theory needed for closed-loop logs with no designed excitation

- Closed loop: `y = S G r + S v`, `u = S r − S C v`, `S = 1/(1 + G C)`. A **direct**
  non-parametric estimate `Ĝ = P_yu/P_uu` is biased toward `−1/C` where loop noise
  dominates the reference. The **joint I/O** estimate `Ĝ = T_yr / T_ur` with
  `T_yr = P_yr/P_rr`, `T_ur = P_ur/P_rr` is unbiased given a measured reference — and
  `PIDx.Tar` is exactly the reference, `PIDx.Act` the output, `RATE.xOut` (or
  `P+I+D+FF+DFF`) the plant input. With the controller `C` known from `PARM`, the
  indirect form `Ĝ = T_yr / (C (1 − T_yr))` is also available.
- Identifiability under feedback needs external excitation with spectral content at
  ≥ (na + nb) frequencies in the band of interest; the pilot's stick is that excitation.
- Least-squares ARX covariance: `cov(θ̂) = σ̂² (ΦᵀΦ)⁻¹`, `σ̂² = ‖y − Φθ̂‖² / (N − p)`.
- Non-parametric confidence (Bendat & Piersol): with coherence `γ²` and `n_d` Welch
  averages, random error of `|H|` is `ε = sqrt(1 − γ²) / (γ sqrt(2 n_d))` and of the phase
  `sqrt((1 − γ²) / (2 γ² n_d))` radians. This is the per-frequency confidence band.
- Ziegler–Nichols from an observed limit cycle at ultimate gain `Ku`, period `Tu`:
  PID `Kp = 0.6 Ku, Ti = 0.5 Tu, Td = 0.125 Tu`; "no overshoot" `Kp = 0.2 Ku, Ti = 0.5 Tu,
  Td = 0.333 Tu`. Applicable when `Dmod < 1` / `SRate` or a visible limit cycle gives `Tu`.
- SIMC (Skogestad) for `G = k e^{−θs}/(τ1 s + 1)`: `Kc = τ1/(k (τc + θ))`,
  `τI = min(τ1, 4 (τc + θ))`, `τD = τ2`, default `τc = θ`.
- VRFT (Campi, Lecchini, Savaresi 2002; PID form Formentin et al. 2019): choose a reference
  model `M`; virtual error `ē = (M⁻¹ − 1) y`; prefilter `L = (1 − M) M W / Φu^{1/2}`;
  regressors `x_i = C_i(z) ē_L` with Tustin P/I/D; one-shot least squares for
  `[Kp Ki Kd]`; instrumental variables when `y` is noisy. Closed-loop data allowed; no
  drone implementation exists yet.
- Iterative Feedback Tuning needs new flights per iteration — not usable on existing logs.

### 7.2 Python

`scipy.signal` (`welch, csd, coherence, dlti, dlsim, lsim, step`), `numpy.linalg.lstsq`
suffice and are already dependencies. `python-control` works on Windows without Slycot
for SISO; `sippy_unipi` needs CasADi and Python ≥ 3.10. Neither is required.

---

## 8. Version drift that a log reader must absorb

| master (2026-09) | 4.x logs | note |
|---|---|---|
| `ATC_ACC_{R,P,Y}_MAX` deg/s² | `ATC_ACCEL_{R,P,Y}_MAX` cdeg/s² | Circuit (V4.7.1 dbe79216) already lacks `ATC_ACCEL_R_MAX`; Brisket (V4.7.0-dev 259b79c3) has it at 116700 |
| `PSC_D_ACC_P/I` (×0.1) | `PSC_ACCZ_P/I` | initial-tune rules changed with it |
| `ANG` at loop rate | `ATT` only | `ATT` still written at 10 Hz on master |
| `AUTOTUNE_GMBK` | fixed `*_BACKOFF` constants | decides the reconstruction in §1.7 |

## 9. The development aircraft's logs (2026-09-16 scan)

Every reference log in `LOG_DIR` and `LOG_DIR_LARGE` (16 logs) has `PIDR/PIDP/PIDY`,
`RATE`, `ATT` at **10.0 Hz** and no `ATUN`, `ATDE`, `SIDD` or `SIDS`. `LOG_BITMASK`
180222, `SCHED_LOOP_RATE` 400, `FSTRATE_ENABLE` 0, `ATC_RATE_FF_ENAB` 1, `AUTOTUNE_AGGR`
0.075, `AUTOTUNE_MIN_D` 0.0005. The Circuit quad carries AutoTune-derived gains
(`ATC_RAT_RLL_P` 0.1235, `_D` 0.00334, `ATC_ANG_RLL_P` 16.01, `ATC_ANG_PIT_P` 17.22,
`INS_GYRO_FILTER` 75, FLTD/FLTT 37.5); the Brisket quad carries the Mission Planner
initial-parameter set (0.135 / 0.0036 / 4.5, gyro 42, FLTD/FLTT 21, accel 116700 / 27000).
No fast-logged flight exists yet; every current log must be refused for gain
recommendation, and that refusal is the first real-log test.
