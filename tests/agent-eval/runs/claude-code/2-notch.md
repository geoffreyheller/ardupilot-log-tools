## Commands
cd "<ws>"; ls; S=.claude/skills/ardupilot-log-analysis; grep -n "Skill 4" -A60 $S/references/recipes.md | head -90
S=.claude/skills/ardupilot-log-analysis/scripts/alog.sh; bash $S info largeprop-quad.bin; echo "EXIT $?"
S=.claude/skills/ardupilot-log-analysis/scripts/alog.sh; bash $S notch largeprop-quad.bin --window rpm; echo "EXIT $?"; bash $S params largeprop-quad.bin 2>/dev/null | grep -E "INS_HNTCH|INS_HNTC2|INS_GYRO_FILTER|INS_FAST_SAMPLE|INS_GYRO_RATE|FFT_ENABLE|MOT_PWM_TYPE|SERVO_BLH|SERVO_DSHOT|ESC_TLM|LOG_BITMASK"
rm -f largeprop-quad.bin.dfcache; ls

## Answer
## Harmonic notch: largeprop-quad.bin

**The notch is turned off, so there's nothing to check.** `INS_HNTCH_ENABLE=0` and `INS_HNTC2_ENABLE=0`. No notch was applied in this flight, and no notch-centre messages (`FCNS`/`FCN`) were logged. `alog notch` therefore reports **SKIP**. That means "not applicable", not a pass.

**Log integrity:** no errors and no warnings. There's one info note (FMTU timestamp stepped back 46 µs) and the logger dropped 0 records. ArduCopter V4.7.0-dev, 3DRControlN1, one flight from 76.5 to 377.4 s.

**Window:** 78.1–376.2 s (298 s). It covers the time the ESC fundamental was above 50.3 Hz, which is 60% of the 83.8 Hz median while the motors were spinning.

### Measured motor fundamental (ESC.RPM / 60)

| min | p01 | median | p99 | max |
|---:|---:|---:|---:|---:|
| 51.0 | 67.2 | 83.9 | 106.2 | 120.4 Hz |

Typical motor speed per ESC: ESC2/M1 89.3, ESC0/M3 85.9, ESC1/M2 80.7, ESC3/M4 79.0 Hz. **The fastest and slowest motors differ by 12.2%**, which is large.

### Suggested starting point (from `alog`; not tested on this aircraft)

| param | value | reason |
|---|---:|---|
| `INS_HNTCH_ENABLE` | 1 | |
| `INS_HNTCH_MODE` | 3 | ESC telemetry: all 4 motors report RPM for the whole window, 0.0% error, longest gap between samples 180 ms |
| `INS_HNTCH_REF` | 1 | follows RPM directly |
| `INS_HNTCH_FREQ` | 60 | just below the airborne p01 of 67.2 Hz |
| `INS_HNTCH_BW` | 30 | half of FREQ |
| `INS_HNTCH_HMNCS` | 3 | 1st and 2nd harmonics: 67–106 Hz and 134–212 Hz |
| `INS_HNTCH_ATT` | 40 | default |
| `INS_HNTCH_OPTS` | 2 | one notch per motor, because the motors spread more than 5% |

### Other things to know

- **This log can't show whether the aircraft needs a notch.** `VIBE`, `IMU`, `GYR`, `ISBD` and `FTN1` were not logged, so there's no vibration or gyro spectrum data at all. "Notch disabled" is a config fact, not evidence of a problem.
- **`INS_GYRO_FILTER=42`** has been raised from the default of 20. That filter sits below the ~84 Hz fundamental, so it does some of the notch's work already, at the cost of control-loop lag. Once a notch is proven, leave this value alone until you've compared before and after logs.
- **The 12% motor spread is a separate issue.** M1 runs about 13% faster than M4, which points to CG offset, a motor or prop problem, or frame twist. Check motor balance before relying on the per-motor notch to cover it.
- The notch parameters other than `ENABLE` aren't in the log, so the current table shows them as "-". None of the suggested values depend on them.

### Test flight to prove the notch works

1. Change only the notch parameters above. Also set `INS_LOG_BAT_MASK=1` and `INS_LOG_BAT_OPT=4`, which log gyro data before and after filtering.
2. Fly 30–60 s of steady hover in LOITER or ALT_HOLD.
3. Run `alog notch <log> --window rpm`. Tracking should be about 0.014, with harmonic lock-on near 0%.
4. Run `alog batchfft <log> --window rpm`. The fundamental should be attenuated, and the deepest dips should sit at 1.000 and 2.000 motor orders.
5. **Set `INS_LOG_BAT_MASK` back to 0 afterwards.** Batch logging roughly doubles the log rate and fills flash.

I deleted the parse cache (`largeprop-quad.bin.dfcache`) that `alog` left next to the log.
