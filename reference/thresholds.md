# Thresholds and where they came from

Every number `alog` grades against lives in `dflog/checks.py::T` with a `source` field.
This file is the provenance and the reasoning. **Do not hardcode a threshold in a script**
— add it to `T`.

A threshold is a prompt to look, not a verdict. A clean sheet is not the same as a good
flight, and a WARN on a metric that has been stable for ten flights is less interesting
than a PASS that moved 3× since last time.

## Not thresholds: the window-construction parameters

Five numbers in `dflog/flight.py` and `dflog/cli.py` look like thresholds and deliberately are not in `T`.
They do not judge an aircraft — nothing is graded against them and none appears in a
verdict — they *define a window*, and putting them in the graded table would imply an
aircraft can fail them.

| name | default | what it means |
|---|---|---|
| `hz_floor` | derived: 0.6 × median of the fundamental while spinning (> 20 Hz) | `--window rpm`: fleet-mean ESC fundamental above this is "airborne". Lands near 0.6 × hover — below every descent, above armed idle (20–60 Hz on every aircraft seen). A fixed 90 Hz was right for a 5-inch quad hovering at 200 Hz and split a 10-inch quad's one flight into five (issue #3). `--hz-floor` overrides it; 90 reproduces the old behaviour. Falls back to 90 Hz only when nothing spins, where the answer is "no flight" anyway. Limitation: a log that is mostly armed idle with a short flight pulls the median toward idle; the `flight detectors disagree` WARN catches that. |
| `thr_floor` | 0.15 | `--window throttle`: `CTUN.ThO` above this is "airborne" |
| `gap_seconds` | 10 s | ground time shorter than this is a bounced landing or a dip below the floor, not a new flight. Large enough that no plausible mid-flight transient splits one flight in two, far below any real disarm / walk-out / re-arm cycle. |
| `min_seconds` | 5 s | anything shorter is a bench spin-up, not a flight |
| `COMPARE_DURATION_RATIO` | 2× | `alog compare`: two windows whose durations differ by more than this are not like-for-like, whatever the method (issue #4) |

They are keyword arguments on `flights()` and `airborne_window()`. `alog schema`'s
threshold table does not list them, and it should not.

## Sources

| tag | what it is |
|---|---|
| `LA` | ArduPilot `Tools/LogAnalyzer` — the official rule-based analyzer. **Removed from master 2024-08-14** (commit `bdea9be7fb1722e86185fdd0ba792a9d97acec74`, "the web-based tools are supplanting this"). Recover with `git checkout bdea9be7fb~1 -- Tools/LogAnalyzer`. GPLv3. |
| `DLA` | `dronekit/dronekit-la` — C++ analyzer, Apache 2.0, **unmaintained since Feb 2022**, but the most systematic threshold set available and generally stricter than LA. |
| `WIKI` | ardupilot.org documentation |
| `MEAS` | measured on the development logs — roughly 2,000 s of flight across two multirotors (a sub-250 g 5-inch and a 10-inch long-range quad). Aircraft-dependent; treat as a starting point, not gospel. |

Where LA and DLA disagree, both are recorded and the stricter is used for WARN.

---

## In use here

### Vibration
| key | warn | fail | source |
|---|---|---|---|
| `vibe_xy` | 15 | 30 | WIKI — `VIBE.VibeX/Y` m/s²; below 15 good, above 30 problematic |
| `vibe_z` | 20 | 30 | WIKI |
| `vibe_2sigma_xy` | 1.5 g | 3.0 g | LA `TestVibration` — 2σ of raw `IMU.AccX/Y` over a loiter chunk |
| `vibe_2sigma_z` | 2.0 g | 5.0 g | LA `TestVibration` |
| `clip_events` | >1 | >20 | WIKI — `VIBE.Clip` delta. Any clipping means the accel saturated. |

LA's version measures over "the largest LOITER chunk ≥10 s with no RC input" — see
`hover_chunks()`. Whole-flight vibration numbers are not comparable between flights.

### Compass
| key | warn | fail | source |
|---|---|---|---|
| `compass_offsets` | 100 | 300 | DLA 100/200, LA 300/500 — `\|COMPASS_OFS\|` vector length |
| `compass_field_lo` | 120 | 100 | LA, DLA — mGauss, low side |
| `compass_field_hi` | 550 | 600 | LA, DLA — mGauss, high side |
| `compass_field_var` | 0.25 | 0.35 | LA — variation over the flight |
| `compass_mot_corr` | 0.30 | 0.50 | MEAS — `\|corr(throttle, \|B\|)\|`; above this, run `COMPASS_MOT` |

`compass_field_var` here uses `(p99−p01)/p01` rather than LA's `(max−min)/min`: one sample
near touchdown or a landing-gear magnet fails an otherwise healthy compass. The raw max/min
is still reported in the table and in the evidence dict.

The 120–550 band is a crude global proxy. The better check is the WMM expected field at the
flight's lat/lon — `mavextra.expected_earth_field()`.

### EKF
| key | warn | fail | source |
|---|---|---|---|
| `ekf_innov` | 0.5 | 1.0 | DLA — all five variances warn 0.5 / fail 1.0. ArduPilot rejects at 1.0. |
| `ekf_errRP` | 0.05 | 0.10 | MEAS |

DLA also applies a **minimum duration** (250 ms) before a divergence counts, so transients
do not trip it. Worth adding if these produce noise.

DLA's divergence thresholds, for reference if you add those checks: attitude warn 5° /
fail 10°; altitude warn 4 m / fail 5 m; gyro drift warn 0.09 / fail 0.10 over a 2 s
average.

### GPS
| key | warn | fail | source |
|---|---|---|---|
| `gps_sats` | <6 | <5 | LA `TestGPSGlitch` |
| `gps_hdop` | >3.0 | >10.0 | LA `TestGPSGlitch` |
| `gps_nofix_pct` | 2 % | 10 % | MEAS — a badly sited GPS in the development logs sat at **69.9 %** |
| `gps_hacc` | 2.0 m | 5.0 m | fail: ArduPilot `AP_NavEKF3::calcGpsGoodToAlign` (`EK3_GPS_CHECK`) rejects hAcc > 5 m; warn: MEAS — `GPA.HAcc` median over 3D-fix samples. A receiver behind a bad ground plane read 4.25 m, the same receiver fixed read 0.94 then 0.64 m (issue #5) |
| `gps_vacc` | 3.0 m | 7.5 m | fail: EKF3 rejects vAcc > 7.5 m; warn: MEAS — `GPA.VAcc` median (5.69 → 1.58 → 0.79 m across the same three flights) |
| `gps_sacc` | 0.5 m/s | 1.0 m/s | fail: EKF3 rejects sAcc > 1.0 m/s; warn: MEAS — `GPA.SAcc` median (1.34 → 0.33 → 0.20 m/s) |

`HDop` is satellite geometry; `GPA.HAcc` is the receiver's own estimate of its horizontal
error in metres, and it is the number that answers "is the GPS better than it was". Graded
on the median; the p95 is reported beside it for dropouts. A receiver with no 3D fix, or
one whose driver supplies no estimate (NMEA units report `HAcc` 0 and the saturated
`VDop` 655.35), is SKIP - 0 m is an absence, not an accuracy. `GPA.Delta` (the fix
interval) identifies each receiver's update rate with no parameter lookup.

LA also flags `ERR` Subsys 11 / ECode 2 as an outright glitch → FAIL.

### Power and CPU
| key | warn | fail | source |
|---|---|---|---|
| `vcc_min` | <4.7 V | <4.6 V | LA `TestVCC` |
| `vcc_spread` | 0.3 V | 0.5 V | LA `TestVCC` |
| `cpu_load` | 60 % | 80 % | MEAS — `PM.Load` ÷ 10 |
| `cpu_slow_pct` | 6 % | 10 % | LA `TestPerformance` — `NLon/NL`; LA also fails on >6 slow lines |
| `curr_stopped_a` | 2.0 A | 5.0 A | MEAS — `BAT.Curr` mean with every motor provably stopped (every ESC `RPM` 0 and every motor output at `SERVO_MIN`): the sensor's zero offset plus the avionics draw. A healthy sensor read 0.00 A on every development log; a faulty ESC-telemetry sum read 12.3–12.8 A and put every figure in the flight that much high (issue #6). A VTX and a GPS are ~1 A, not 5 |

The motors-stopped measurement lives outside the airborne window by definition; the
power check reads the whole log for it and says how many samples came before and after
the window. It then reports consumption raw and offset-corrected, and current at idle,
hover (`ThH` ± 0.04) and full throttle (`ThO` ≥ 0.95), raw and corrected, with hover power
in watts. Fix `BATT_AMP_OFFSET` before `BATT_AMP_PERVLT`: a scale correction on top of an
offset is wrong at every current but one.

DLA's `battery` low threshold is 15 % remaining.

### Motors
| key | warn | fail | source |
|---|---|---|---|
| `rpm_spread_pct` | 3 % | 8 % | MEAS — on **medians**. Healthy baseline 1.4 %, bent prop 5.7 %, worst observed 7.5 %. |
| `trim_us` | 10 | 25 | MEAS — `\|roll/pitch/yaw trim\|` in µs; healthy baseline yaw −5.0, bent-prop yaw +22.6 |
| `trim_hover_diff_us` | 10 | 25 | MEAS — largest-axis difference between the trim over the whole window and the same trim over level hover (hover chunks with `\|roll\|,\|pitch\|` < 3°, or level samples when there is no chunk). Different means the trim depends on translating — a forward-flight artefact (issue #8). Identical means a CG/airframe asymmetry **or a steady breeze** (a static case read 58.6 vs 57.8 µs; Brisket 2026-09-30 read identical in level hover on both flights while a breeze moved the trim 20.6/12.7 → 35.2/0.2 µs with the battery untouched); `trim vs heading` separates the two, grading its airframe-fixed part on `trim_us` |
| `esc_err_pct` | 5 % | 15 % | MEAS — a healthy bidirectional-DShot link runs 2.7–3.3 % steadily with no ill effect |
| `motor_headroom` | 0.90 | 0.97 | DLA-style — p99.5 output as a fraction of the `MOT_SPIN_MAX` ceiling |
| `drive_norm_spread_pct` | 3 % | 6 % | MEAS — (max−min)/mean of per-motor median `RPM / (duty × pack V)` over the p20–p80 band of fleet duty. Healthy 1.7–2.4 % on three flights of a 10-inch quad whose *raw* RPM spread was 10–12 % (issue #7): load asymmetry from a CG offset leaves this flat, a dragging motor drops it |
| `esc_temp_c` | 80 °C | 100 °C | ESC vendor thermal-protection limits (BLHeli_32 default 140 °C) and MEAS — max `ESC.Temp`; healthy development aircraft ran 15–50 °C |
| `esc_temp_spread_c` | 10 °C | 20 °C | MEAS — max−min of per-ESC mean `ESC.Temp`; healthy 1–5 °C. One hot ESC in a set of four is a finding on its own |

The ESC table also carries p05/p95 RPM (min and max are single samples dominated by
spin-up and brief saturation) and the motor each ESC drives through the `SERVOn_FUNCTION`
map, since `ESC[i]` is servo output `i+1`.

The trim table is followed by the **CG offset** it implies, from per-motor RPM medians
with thrust ∝ RPM²: `r = (mean front RPM / mean rear RPM)²`, offset `(r−1)/(r+1)` of the
fore-aft arm (CG to the front motor line), positive forward; likewise left/right for roll.
`--arm-mm` turns it into millimetres. Not graded — `trim_us` already is — but it is the
number that answers "is that the battery too far aft".

DLA's `motorbalance` uses a PWM delta of warn 50 / fail 100 µs measured only while pitch,
roll and yaw rates are all below 1 °/s for ≥100 ms — a stricter "stable" gate than
`hover_chunks()`. The trim thresholds here are tighter because the decomposition is a
cleaner signal than a raw delta.

### Attitude and tune
| key | warn | fail | source |
|---|---|---|---|
| `att_err_deg` | 5 | 10 | DLA `attitude_control` offset thresholds |
| `rate_corr` | <0.6 | <0.4 | MEAS — a gentle hover read 0.87 roll / 0.83 pitch / 0.27 yaw; a soft axis stands out clearly |
| `gust_event_rate` | 0.2/s | 0.5/s | MEAS — an untuned airframe read 143 events in 422 s = 0.34/s, i.e. weak gust rejection |

The gust-event definition is `|actual − desired| > 2.5°` while `|desired| < 1°`. Report the
**rate**, not the count: counts are not comparable between flights of different length. Only meaningful in a mode where desired attitude is
directly commanded.

`rate_corr` is strongly flight-dependent — a gentle hover and an aggressive flight will
differ by 2× on the same tune. Compare like with like, and prefer the 5 Hz band split
(low = gain problem, high = filter problem) over the bare correlation.

### Notch
| key | warn | fail | source |
|---|---|---|---|
| `notch_track` | 0.05 | 0.15 | MEAS — p95 of `\|FCNS.CF / fundamental − 1\|`; a correctly tracking notch reads 0.014 |
| `notch_mistrack_pct` | 1 % | 5 % | MEAS — % above 1.5× the fundamental; FFT-driven read 6.5 %, ESC-driven 0.00 % |
| `notch_atten_db` | >−10 | >−6 | WIKI — attenuation at the fundamental; a verified flight read −29 dB |

Wiki guidance worth knowing: motor noise sits near 200 Hz on small copters and 100 Hz on
larger ones; vibration above 100 Hz is the concerning band; default FREQ:BW ratio is 2:1,
use 4:1 when tracking three peaks to limit phase lag; `FFT_OPTIONS` bit 1 warns when motor
noise exceeds 40 dB.

`mavfft_isb.py --notch-params` suggests: `INS_HNTCH_REF` = mean `CTUN.ThO` where
`CTUN.Alt > 1`; `INS_HNTCH_FREQ` = the FFT peak; `INS_HNTCH_BW` = peak ÷ 2.

When the notch is **disabled** (`INS_HNTCH_ENABLE=0`) and the log carries ESC telemetry,
the notch check prints the measured fundamental envelope (min, p01, median, p99, max and
each motor's median) and a starting point derived from it (issue #9): `MODE=3` justified
by the ESC telemetry quality it measured, `REF=1`, `FREQ` = 0.95 × the airborne p01
fundamental rounded down to 5 Hz (the p01, not the instantaneous minimum, which is a
touchdown), `BW = FREQ/2`, `HMNCS=3`, `ATT=40`, and `OPTS=2` (per-motor notches) when the
inter-motor spread exceeds 5 %. It says what cannot be verified from that log and names
the batch-logging flight that would. A notch that is *enabled* but has no `FCNS` is a
different SKIP — fix the logging, not the configuration.

### PID tuning
| key | warn | fail | source |
|---|---|---|---|
| `tune_pid_rate_hz` | < 200 Hz | < 100 Hz | PID-Analyzer 0.5 s step window + 25 Hz Wiener regulariser (Nyquist); ArduCopter fast logging (`LOG_BITMASK` bit 0) = `SCHED_LOOP_RATE`. The standard bitmask logs `PIDx`/`RATE` at 10 Hz, which cannot show a rate loop whose filters sit at 20–40 Hz |
| `tune_min_frames` | < 30 | < 10 | PID-Analyzer `high.sum() < 10` rule (fail); 30 MEAS — deconvolution frames whose max \|target\| ≥ 20 deg/s |
| `tune_coherence` | < 0.8 | < 0.6 | fpvpidlab 0.5 gate, AnalyticTune "sufficient coherence", Bendat & Piersol random-error formula — mean coherence over the identification band |
| `tune_confidence` | < 0.7 | < 0.4 | MEAS (this plan, `docs/pid-tuning-plan.md` §2.5) — ≥ 0.7 recommend, 0.4–0.7 indicative ("validate before applying"), < 0.4 withheld |
| `tune_srate_osc` | > 5 | > 10 | `QUIK_OSC_SMAX` default 5 (VTOL-quicktune.lua) — `PIDx.SRate` p95; QuickTune calls the loop oscillating above it |
| `tune_overshoot_ratio` | > 1.0 | > 2.0 | AutoTune overshoot allowance `0.5 × AGGR` (rate P and angle P steps) — step overshoot ÷ (0.5 × `AUTOTUNE_AGGR`) |
| `tune_bounce_ratio` | > 1.0 | > 2.0 | AutoTune bounce-back criterion `AGGR × peak` (rate D steps) — step bounce-back ÷ `AUTOTUNE_AGGR` |
| `tune_gain_margin_db` | < 6 dB | < 3 dB | AnalyticTune / heli AutoTune 6 dB — open-loop `L = C·G` gain margin |
| `tune_phase_margin_deg` | < 45° | < 30° | AnalyticTune 45° — open-loop phase margin |
| `tune_session_spread` | > 0.15 | > 0.30 | MEAS — (max − min)/median of a gain across AutoTune sessions of one aircraft |
| `tune_pi_ratio_dev` | > 0.25 | > 0.50 | AutoTune `PI_RATIO_FINAL` 1.0 (roll, pitch) / `YAW_PI_RATIO_FINAL` 0.1 — \|I/P ÷ ratio − 1\| |
| `tune_flt_ratio_dev` | > 0.25 | > 0.50 | WIKI `FLTD = FLTT = INS_GYRO_FILTER / 2` — \|FLTD or FLTT ÷ (INS_GYRO_FILTER/2) − 1\| |
| `tune_limited_pct` | > 5 % | > 20 % | MEAS — percent of `PIDx` samples with `Flags` bit 0 (output limited, anti-windup active); above fail the loop is nonlinear and refusal `OUTPUT_SATURATED` is raised |

The "MEAS" rows here are chosen in `docs/pid-tuning-plan.md` §4, not yet measured: none of
the reference logs has fast attitude logging, so the first real-log behaviour is a
`PID_RATE_TOO_LOW` refusal. Re-measure them when a fast-logged flight exists.

Algorithm constants — not gradings — live in `dflog/tune.py::CONSTANTS`, each entry
`dict(value=, source=, note=)`: the PID-Analyzer frame (1.0 s), response (0.5 s), overlap
(1/16), regulariser cut (25 Hz), minimum target (20 deg/s) and low/high split (500
deg/s); the full multicopter AutoTune table (`AUTOTUNE_AGGR` 0.075, `GMBK` 0.25, `MIN_D`
0.0005, the 5 % steps, gain limits, rate and angle targets, `SUCCESS_COUNT` 4, the
acceleration floors, `D_UP_DOWN_MARGIN` 0.2, the PI ratios); QuickTune's `QUIK_OSC_SMAX` 5
and `QUIK_GAIN_MARGIN` 0.6; and the 10 s minimum parameter-constant segment. Firmware
defaults used when a parameter is absent from the log are in `tune.DEFAULTS` with the
header each came from, and every one that was used is named in `GainSet.defaulted`.
RULES §2 forbids hard-coding a number in a check; those tables are where the numbers live
and where their provenance is. `alog schema` prints `T`; the constants table is
`tune_constants` (`tune.all_constants()` merges every tuning module's table).

**Confidence priors.** A recommendation's confidence is `prior × adequacy × excitation ×
consistency × agreement` (`docs/pid-tuning-plan.md` §2.5) and is graded by
`tune_confidence` above. The prior per method is `dflog/tune_fuse.py::PRIORS`, source
"docs/pid-tuning-plan.md section 2.5" (chosen, MEAS-class, not yet measured on a real
fast-logged flight):

| method | prior | what it is |
|---|---|---|
| `autotune-log` | 1.0 | the gains the firmware found and saved, re-derived from `ATUN` |
| `virtual-autotune` | 0.8 | AutoTune's own search run on the identified plant |
| `ceiling` | 0.7 | a measured oscillation ceiling × QuickTune's 0.4 margin |
| `step-rules` | 0.5 | bounded adjustments from the deconvolved step response |
| `unchanged` | 0.0 | not a recommendation: the parameter is left as configured and the row says why |

Two fusion constants sit beside them in `tune_fuse.FUSE_CONSTANTS`: `step-rules` values
are capped at `fuse_tier_c_confidence_cap` 0.39 — below `tune_confidence.fail`, so always
withheld — because AutoTune's overshoot/bounce criteria apply to a twitch flown with test
gains, not to the final loop a log shows (plan §2.5, tier C caveat, WP3 2026-09-17); and
`agreement` is floored at `fuse_agreement_floor` 0.5. Re-measure the priors when a
fast-logged flight exists, in the same pass as the MEAS rows above.

---

## LogAnalyzer checks not yet ported

Worth adding if a log ever needs them. Thresholds recorded so nobody has to go and dig
them out of a removed directory again.

| test | thresholds |
|---|---|
| `TestBrownout` | still armed at log end and `CTUN.BAlt` > 3.0 m → FAIL (truncated log) |
| `TestIMUMatch` | filtered accel-magnitude difference IMU vs IMU2: warn 0.75, fail 1.5; low-pass τ 5.0 s |
| `TestThrust` | throttle >700, ignore if `\|roll\|` or `\|pitch\|` > 20°, segment >50 samples; avg climb rate FAIL <50 cm/s, WARN <100 cm/s |
| `TestPitchRollCoupling` | max lean = `ANGLE_MAX/100` + 10° buffer; ignore below 2.0 m relative alt; ACRO/SPORT/FLIP/AUTOTUNE/THROW excluded |
| `TestOptFlow` | tilt 15°, flow quality ≥124, scale-factor 1σ threshold 5.0 |
| `TestNaN` | any NaN → FAIL, allow-list `{CTUN: [DSAlt, TAlt], POS: [RelOriginAlt]}` |
| `TestDupeLogData` | 10 samples of 20 consecutive `ATT.Pitch` values, search for an exact repeat elsewhere (detects flash corruption) |
| `TestParams` | NaN in any param → FAIL; copter also `MAG_ENABLE==1`, `THR_MIN<200`, `299<THR_MID<701` |
| `TestDualGyroDrift` | present but `enable = False` — never runs |

LA's `ERR` subsystem map: 2/1 PPM, 3/1|2 COMPASS, 5/1 FS_THR, 6/1 FS_BATT, 7/1 GPS,
8/1 GCS, 9/1|2 FENCE, 10 FLT_MODE, 11/2 GPS_GLITCH, 12/1 CRASH. FENCE-only → WARN, the
rest → FAIL.

---

## Added September 2026 (v2.0)

| key | warn | fail | source |
|---|---|---|---|
| `imu_match_mss` | 0.75 | 1.5 | LA `TestIMUMatch` — low-passed accel-magnitude difference between IMUs, m/s² |
| `gyro_bias_dps` | 1.0 | 3.0 | MEAS — largest-axis mean gyro rate over the airborne window, deg/s |
| `att_div_deg` | 5 | 10 | DLA `attitude_estimate_divergence` — ATT vs AHR2 / XKF1, p99 of the difference |
| `alt_div_m` | 4 | 5 | DLA `altitude_estimate_divergence` — baro vs EKF relative altitude, p99 |
| `gps_glitch_speed` | 30 m/s | 60 m/s | MEAS — implied ground speed between consecutive 3D fixes on a multirotor |
| `free_mem_bytes` | < 20000 | < 5000 | MEAS — `PM.Mem` minimum; scripting and logging need headroom |
| `brownout_alt_m` | 1 m | 3 m | LA `TestBrownout` — still armed at log end with `BAlt` above this |
| `lean_over_max_deg` | > 0 | > 10 | LA `TestPitchRollCoupling` — lean beyond `ANGLE_MAX` |
| `motor_peak_db` | 25 dB | 40 dB | WIKI — `FFT_SNR_REF` default 25 dB; `FFT_OPTIONS` warns above 40 dB of motor noise |

The "LogAnalyzer checks not yet ported" list above is now history: Brownout, IMUMatch,
PitchRollCoupling, NaN, DupeLogData, Params, Autotune and Empty are ported (see
`reference/existing-tools.md` for the mapping). `TestThrust` was not — its throttle
threshold of 700 can never be met by the modern 0–1 `CTUN.ThO`, so the check is dead on
every log this tool will see. `TestOptFlow` was not — no optical flow on the development
aircraft; the calibration procedure is documented in the source survey.
