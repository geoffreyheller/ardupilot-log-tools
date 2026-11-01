## Commands
Get-ChildItem; & .\.claude\skills\ardupilot-log-analysis\scripts\alog.ps1 info largeprop-quad.bin
ls -la; .claude/skills/ardupilot-log-analysis/scripts/alog.sh info largeprop-quad.bin
.claude/skills/ardupilot-log-analysis/scripts/alog.sh all largeprop-quad.bin; echo EXIT=$?
ls *.dfcache 2>/dev/null && rm -f largeprop-quad.bin.dfcache; ls

## Answer
# largeprop-quad.bin: mostly OK, with some things to fix

The flight was controlled and safe: no errors, no failsafes, plenty of power left over, cool ESCs and a normal landing and disarm. But there are three real problems, one of which affects your battery readings. The log also leaves out the data needed to check vibration, the EKF, the compass or the tune, so I can't call it a full bill of health.

**Log:** ArduCopter V4.7.0-dev, 3DRControlN1, Quad/X. The file is intact: every byte decoded, no resyncs, no dropped records. It holds one flight, 301 s long (76.5–377.4 s, measured from the flight controller's "not landed" to "land complete" events). All numbers below come from that window.

## Fix these

1. **The current sensor reads about 12.3 A with every motor stopped** (standard deviation 0.14 A, 167 samples; the tool warns above 2 A and fails above 5 A). Part of that is the flight controller and other electronics drawing power, but almost all of it is a zero offset. So every current number in this flight is too high:
   - The log says you used **1845 mAh**; corrected, it's about **815 mAh**.
   - Corrected hover power is about 169 W (6.5 A at 26.1 V).
   - Any battery failsafe based on mAh used will trigger far too early.

   **Fix:** set `BATT_AMP_OFFSET` from a reading taken with the motors stopped. Then check the scale against a charger: compare the logged mAh with the mAh the charger puts back. Fix the offset before touching the scale.

2. **Your second GPS (GPS1, NMEA) is poor.** It had no 3D fix for 15.5% of the flight, including a 12.8 s gap at 84–97 s. It went as low as 0 satellites, with a position jump implying 83 m/s. Its first fix came 34 s after take-off. GPS0 (u-blox) was solid: at least 7 satellites, HDOP 2.27 or better, a fix the whole time, 0.64 m median horizontal accuracy.
   - Look at that unit's antenna, sky view and wiring.
   - Make sure the EKF isn't blending it or switching to it (`GPS_AUTO_SWITCH`). The EKF uses GPS0 (`GPS_PRIMARY=0`), but this log can't show whether it ever switched.

3. **The front motors work much harder than the rear ones.** Standing pitch trim is **+58 µs** (the tool fails above 25 µs), and the front motors spin about 12% faster than the rear (RPM spread 12.2%). That works out to the centre of gravity sitting about **9% of the arm length forward**.
   - Once you allow for how hard each motor is driven, the motors agree to within 2.2% (passes). So this looks like weight distribution, not a bad motor or prop.
   - One caveat: you hovered facing only one direction (~150°), so a steady breeze can't be ruled out. **To confirm:** move the battery back, or hover about 20 s facing each of N, E, S and W, then check pitch trim again.
   - Yaw trim of +21.6 µs (warning) means some motors are twisting against the others. Check that the motors are mounted straight.

## Minor

- **Attitude error:** max 5.9° roll and 6.4° pitch (warning; the tool warns above 5°, fails above 10°). Typical error is small (95th percentile about 2°), and only 21 unrequested wobbles in 301 s (passes). This may partly come from the nose-heavy balance above.
- **Compass:** the EKF reported a "ground mag anomaly" at take-off. After landing, a pre-arm check failed with `Check mag field (xy diff:101>100)`. Compass data (`MAG`) wasn't logged, so I can't dig further. Check what's near the compass, like power leads or the battery.
- **`MOT_THST_HOVER`:** the parameter snapshot in the log says 0.296, but in flight it learned about 0.204. Hover throttle of 0.20 also means the quad has plenty of power to spare.
- **Fine:** motor headroom (99.5th percentile 0.87 of the ceiling), ESC temperatures (37 °C max), DShot errors (0%), CPU load (29% peak).

## What this log can't show you

These checks were skipped, which is not the same as passing:

- Vibration and IMU: `VIBE` and `IMU` weren't logged.
- EKF health, compass, frequency analysis.
- PID tuning: the rate and PID messages weren't logged.

**The harmonic notch is disabled** (`INS_HNTCH_ENABLE=0`). With big props, the motor frequency sits low, around 80–89 Hz, which makes a notch worth having. The tool suggests a starting point:

`INS_HNTCH_MODE=3` (RPM-driven from ESC telemetry), `REF=1`, `FREQ=55`, `BW=27`, `HMNCS=3`, `OPTS=2` (one notch per motor, because the motors' speeds differ by 12%)

This is a suggestion, not a tested setting. To check it, fly a 30–60 s hover with `INS_LOG_BAT_MASK=1` and `INS_LOG_BAT_OPT=4`, then set the mask back to 0.

## Next flights (one change each)

1. Set `BATT_AMP_OFFSET`. Confirm the current reads about 0 A with the motors stopped.
2. Fix the CG, or do the four-direction hover. Pitch trim should drop under 10 µs.
3. Turn on the notch and record vibration data, so the vibration and notch checks can actually run.

If you want the tune checked, set `LOG_BITMASK=180223` and fly the tuning routine the tool describes.

I deleted the cache file the tool created (`largeprop-quad.bin.dfcache`).
