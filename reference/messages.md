# Message and field reference

The messages that matter for copter log analysis, with units and the non-obvious
interpretations. Run `python alog.py types "<log>"` (or `alog fields "<log>" MSG` for units and ranges) for the exact fields in a specific log —
they vary by firmware version and `LOG_BITMASK`.

Field names drift between versions. Use `log.field(msg, "NewName", "OldName")` rather than
hardcoding: `BarAlt`→`BAlt`, `ThrOut`→`ThO`, `CRate`→`CRt`, `Chan1`/`Ch1`→`C1`,
`NSat`/`numSV`→`NSats`, `HDp`/`EPH`→`HDop`.

---

## Attitude and control

| message | fields | notes |
|---|---|---|
| `ATT` | `DesRoll Roll DesPitch Pitch DesYaw Yaw ErrRP ErrYaw AEKF` | degrees. Attitude error = actual − desired. |
| `RATE` | `RDes R ROut PDes P POut YDes Y YOut ADes A AOut AOutSlew` | **deg/s, not rad/s.** `*Out` is the controller output, normalised to ±1.0. |
| `ANG` | `DesRoll Roll DesPitch Pitch DesYaw Yaw` | angle-controller view, higher rate than `ATT` on some builds |
| `PIDR` `PIDP` `PIDY` `PIDA` | `Tar Act Err P I D FF DFF Dmod SRate Flags` | per-axis PID term breakdown. `PIDA` is the altitude/throttle PID. |
| `CTUN` | `ThI ABst ThO ThH DAlt Alt BAlt DSAlt SAlt TAlt DCRt CRt` | `ThO` = throttle out (0–1), `ThH` = learned hover throttle, `BAlt` = barometric altitude (m), `CRt` = climb rate (cm/s) |
| `MOTB` | `LiftMax BatVolt ThLimit ThrAvMx ThrOut FailFlags` | see FailFlags below |
| `RCOU` | `C1`…`C14` | motor/servo outputs in µs |
| `RCIN` | `C1`…`C14` | RC input in µs; `C1/C2/C4` are roll/pitch/yaw sticks, 1500 = centre |

**`Dmod`** is the D-term slew limiter's gain scaling. `Dmod = 1.000` for the whole flight
means the limiter never engaged — no oscillation onset anywhere. Anything below 1.0 means
the tune was backing itself off, and that is worth a sentence in the report.

**`SRate`** is the measured slew rate the limiter watches. Compare against
`ATC_RAT_*_SMAX`; if `SMAX` is 0 the limiter is disabled and `Dmod` will read 1.000
trivially.

**`MOTB.FailFlags = _thrust_boost | (_thrust_balanced << 1)`** — so **2 is the healthy
value**, not an error. `MOTB.ThLimit = 1.000` means never throttle-limited.

**Motor saturation ceiling** is `SERVO1_MIN + MOT_SPIN_MAX × (SERVO1_MAX − SERVO1_MIN)`,
not 2000 µs. With the usual 1000/2000 and `MOT_SPIN_MAX=0.95` that is 1950 µs.

---

## Motors, ESC and the harmonic notch

| message | fields | notes |
|---|---|---|
| `ESC` | `Instance RPM RawRPM Volt Curr Temp CTot MotTemp Err` | per-motor. `Err` is the bidirectional-DShot error rate in **percent**. `Curr` is often 0 on 4-in-1 ESCs that expose only a combined analog output. |
| `EDT2` | `Instance Stress MaxStress Status` | Extended DShot Telemetry; needs `SERVO_DSHOT_ESC=4` |
| `FCNS` | `I CF HF` | **the frequency the notch actually applied.** `CF` = centre, `HF` = 2nd harmonic. Logged every 100 ms. |
| `FTN1` | `PkAvg PkMax BwAvg SnX SnY SnZ ...` | the in-flight FFT's *opinion*. Only drives the notch when `INS_HNTCH_MODE=4`. |
| `FTN2` `FTNS` | per-axis FFT detail | large; costs real log space |
| `ISBH` | `N type instance mul smp_cnt SampleUS smp_rate` | batch-IMU window header. `type` 0=accel 1=gyro; value = raw / `mul`. |
| `ISBD` | `N seqno x y z` | batch-IMU samples, int16 arrays |

**Motor noise frequency = RPM ÷ 60.** Fleet mean of the four `ESC` instances is the ground
truth every notch check is measured against.

**RPM depends on the pole count.** `SERVO_BLH_POLES` must match the motor (12 and 14 are
both common). A wrong pole count silently scales every RPM figure — cross-check the ESC
fundamental against `FTN1.PkAvg`; if they agree within an FFT bin, the pole count is right.

**`ISBH.instance` for pre/post-filter logging is not self-describing.** With
`INS_LOG_BAT_OPT=4` you get two series per sensor and the mapping to pre/post is not
documented in the message. Identify them empirically: the pre-filter series has far more
energy above `INS_GYRO_FILTER`.

**`ISBD.seqno` must be contiguous within a batch.** Batch logging drops samples routinely;
discard any window with a gap rather than concatenating across it.

---

## IMU and vibration

| message | fields | notes |
|---|---|---|
| `VIBE` | `IMU VibeX VibeY VibeZ Clip` | m/s². `Clip` is a **cumulative counter** — take the delta over the window, not the value. |
| `IMU` | `I GyrX/Y/Z AccX/Y/Z EG EA T GH AH GHz AHz` | `EG`/`EA` are gyro/accel error counts, `GH`/`AH` health flags, `T` temperature °C |

ArduPilot's rule of thumb: below 15 m/s² good, above 30 problematic. **Clipping is the
harder failure** — a non-zero clip count means the accelerometer saturated and the EKF was
fed garbage for those samples.

---

## EKF

| message | fields | notes |
|---|---|---|
| `XKF1` | `C Roll Pitch Yaw VN VE VD dPD PN PE PD GX GY GZ OH` | state estimate |
| `XKF2` | `C AX AY AZ VWN VWE MN ME MD MX MY MZ MI` | accel bias, wind, mag states |
| `XKF3` | `C IVN IVE IVD IPN IPE IPD IMX IMY IMZ IYAW IVT` | **innovations** in native units |
| `XKF4` | `C SV SP SH SM SVT errRP OFN OFE FS TS SS GPS PI` | **innovation test ratios** |
| `XKF5` | `C NI FIX FOFF AFI HAGL offset RI rng Herr eAng eVel ePos` | optical flow / range |
| `XKFS` | `C FS MS GS AS` | source-set selection |

**`XKF4` fields are test ratios and ArduPilot rejects the measurement at 1.0.**
`SV` velocity, `SP` position, `SH` height, `SM` magnetometer, `SVT` airspeed.

Report the **count of samples above 1.0**, not just the max. A rising count across
consecutive flights is the signal; the mean can improve while the rejections double.
`errRP` is the roll/pitch error estimate.

---

## Compass

| message | fields | notes |
|---|---|---|
| `MAG` | `I MagX MagY MagZ OfsX OfsY OfsZ MOX MOY MOZ Health S` | mGauss. `MO*` are motor-compensation offsets. |

Field magnitude `|B| = sqrt(X²+Y²+Z²)`; healthy Earth field is roughly 250–650 mGauss
depending on location, and the fixed 120–550 band used by LogAnalyzer is a crude proxy.
The better check is the expected field at the flight's lat/lon from the WMM table —
`mavextra.expected_earth_field()`, available after `tools/bootstrap_pymavlink.sh`.

Motor interference: correlate throttle against `|B|`. A strong correlation means a
`COMPASS_MOT` calibration is worth running; near zero means it is not.

`PreArm: Check mag field` messages after landing are worth quoting verbatim — they carry
the actual numbers (`xy diff:127>100`, or `1181, max 875, min 185`).

---

## GPS

| message | fields | notes |
|---|---|---|
| `GPS` | `I Status GMS GWk NSats HDop Lat Lng Alt Spd GCrs VZ Yaw U` | `Status` ≥3 is a 3D fix; 4 = SBAS/DGPS. `GCrs` is ground course. |
| `GPA` | `I VDop HAcc VAcc SAcc YAcc VV SMS Delta AEl RTCMFU RTCMFD` | accuracy estimates in metres |
| `UBX2` | u-blox diagnostics | **emitted only by the u-blox driver** |

**Identify which physical GPS an instance is by `UBX2`, never by the instance index.**
Which unit lands in slot 0 depends on SERIAL port order and can change between param
snapshots — one analysis attributed a run of dropouts to the wrong unit exactly this way,
and every statistic derived from it had to be recomputed.

With two GPS units, the *differential* is the diagnostic: both degraded similarly points at
a shared external emitter; one much worse points at that unit (self-jam, weak front end).

Compass orientation cross-check that a hover flight cannot provide: fly one straight 100 m
pass at 8–10 m/s in each of two roughly opposite directions, then compare `GPS.GCrs`
against `ATT.Yaw`. That is the test that catches a 180°/90° orientation error a good
calibration can otherwise hide.

---

## Power and system

| message | fields | notes |
|---|---|---|
| `BAT` | `Inst Volt VoltR Curr CurrTot EnrgTot Temp Res RemPct H SH` | `CurrTot` mAh cumulative, `Res` internal resistance estimate (Ω) |
| `POWR` | `Vcc VServo Flags AccFlags Safety` | `Vcc` is **NaN on boards without board-voltage sensing** — not a fault |
| `MCU` | `MTemp MVolt MVmin MVmax` | MCU temperature and rail |
| `PM` | `LR NLon NL MaxT Mem Load ErrL InE ErC SPIC I2CC I2CI Ex R` | **`Load` is percent × 10.** `Mem` is free bytes. |

Current should scale as thrust^1.5 (thrust ∝ ω², power ∝ ω³). A strong throttle
correlation with that power law is a real sensor. **A flat, throttle-independent reading is
a wiring or pin fault, not a calibration error** — a wrong `BATT_AMP_PERVLT` changes the
magnitude, never the correlation.

Absolute current scale is only trustworthy after a charger cross-check: fly a pack, note
logged `CurrTot`, recharge and read the mAh put back, then
`BATT_AMP_PERVLT_new = BATT_AMP_PERVLT × (logged ÷ charger)`.

Analog current decode: `A = V_adc × BATT_AMP_PERVLT + BATT_AMP_OFFSET`.

LogAnalyzer deliberately ignores `PM.MaxT` — it throws false positives around arm and
disarm.

---

## Events, modes, messages

| message | fields | notes |
|---|---|---|
| `MSG` | `Id Seq Message` (4.7+; `Message` only before) | the FC's own commentary: firmware banner, prearm failures, EKF resets. From 4.7 long texts are 64-byte chunks reassembled by `(Id, Seq)`; `log.messages_text()` does this. |
| `FILE` | `FileName Offset Length Data` | embedded files written at arming (`@SYS/threads.txt`, `@ROMFS/hwdef.dat`, `defaults.parm`...). No `TimeUS`; binary `Data`; `alog files` reassembles them. |
| `DSF` | `Dp Blk Bytes FMn FMx FAv` | logger statistics: `Dp` is the count of records **dropped** for lack of buffer |
| `ARM` | `ArmState ArmChecks Forced Method` | arming state changes; the witness for "armed" when the log opened at arming and `EV 10` is missing |
| `RTC` | `Epoch Src` (4.7+) | wall-clock epoch, when the RTC has been set |
| `EV` | `Id` | see `dflog.flight.EVENTS` for the full id table |
| `MODE` | `Mode ModeNum Rsn` | see `dflog.flight.MODES` |
| `ERR` | `Subsys ECode` | subsystem errors; LogAnalyzer's map is in `reference/thresholds.md` |
| `PARM` | `Name Value` | the full parameter set, written at boot |

Key `EV` ids: 10 ARMED, 11 DISARMED, 15 AUTO_ARMED, 17 LAND_COMPLETE_MAYBE,
18 LAND_COMPLETE, 28 NOT_LANDED, 30–37 AUTOTUNE lifecycle, 56/57 MOTORS_INTERLOCK
DISABLED/ENABLED, 60 EKF_ALT_RESET, 62 EKF_YAW_RESET.

The airborne window is 28 → 18. Arm-to-disarm (10 → 11) includes ground time.
