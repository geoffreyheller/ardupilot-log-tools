# Thresholds and where they came from

Every number `alog.py` grades against lives in `dflog/checks.py::T` with a `source` field.
This file is the provenance and the reasoning. **Do not hardcode a threshold in a script**
— add it to `T`.

A threshold is a prompt to look, not a verdict. A clean sheet is not the same as a good
flight, and a WARN on a metric that has been stable for ten flights is less interesting
than a PASS that moved 3× since last time.

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

LA also flags `ERR` Subsys 11 / ECode 2 as an outright glitch → FAIL.

### Power and CPU
| key | warn | fail | source |
|---|---|---|---|
| `vcc_min` | <4.7 V | <4.6 V | LA `TestVCC` |
| `vcc_spread` | 0.3 V | 0.5 V | LA `TestVCC` |
| `cpu_load` | 60 % | 80 % | MEAS — `PM.Load` ÷ 10 |
| `cpu_slow_pct` | 6 % | 10 % | LA `TestPerformance` — `NLon/NL`; LA also fails on >6 slow lines |

DLA's `battery` low threshold is 15 % remaining.

### Motors
| key | warn | fail | source |
|---|---|---|---|
| `rpm_spread_pct` | 3 % | 8 % | MEAS — on **medians**. Healthy baseline 1.4 %, bent prop 5.7 %, worst observed 7.5 %. |
| `trim_us` | 10 | 25 | MEAS — `\|roll/pitch/yaw trim\|` in µs; healthy baseline yaw −5.0, bent-prop yaw +22.6 |
| `esc_err_pct` | 5 % | 15 % | MEAS — a healthy bidirectional-DShot link runs 2.7–3.3 % steadily with no ill effect |
| `motor_headroom` | 0.90 | 0.97 | DLA-style — p99.5 output as a fraction of the `MOT_SPIN_MAX` ceiling |

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
