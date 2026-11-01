## Commands
$s=".claude\skills\ardupilot-log-analysis\scripts\alog.ps1"; & $s info largeprop-quad.bin; & $s tune largeprop-quad.bin
& ".claude\skills\ardupilot-log-analysis\scripts\alog.ps1" info largeprop-quad.bin
cd "<ws>" && ls && .claude/skills/ardupilot-log-analysis/scripts/alog.sh info largeprop-quad.bin
cd "<ws>" && .claude/skills/ardupilot-log-analysis/scripts/alog.sh tune largeprop-quad.bin; echo "exit $?"
cd "<ws>" && A=.claude/skills/ardupilot-log-analysis/scripts/alog.sh; $A params largeprop-quad.bin --grep "ATC_RAT_|ATC_ANG_|ATC_ACCEL|INS_GYRO_FILTER|MOT_THST_HOVER|INS_HNTCH_ENABLE"; $A all largeprop-quad.bin 2>&1 | grep -iE "I/P|FLT|D/P|SMAX|FF_ENAB|AC_PID|Parameters NOT|refused|verdict|FAIL|WARN" | head -40; rm -f largeprop-quad.bin.dfcache

## Answer
I can't recommend PID gains from this log. The tool refused it, and I didn't try to estimate gains some other way, because any number I gave you would be a guess.

**Why it was refused.** The log has no rate-loop data at all: no `PIDR`, `PIDP`, `PIDY` or `RATE` messages. `LOG_BITMASK` is 180222, which leaves out bit 0 (ATTITUDE_FAST). Everything that was logged is at about 10 Hz. `alog tune` exited with code 3 (`NO_PID_MESSAGES`) on roll, pitch and yaw.

**About the log.** It's ArduCopter V4.7.0-dev on a 3DRControlN1 board, quad X frame. The file is intact (0 errors, 0 resyncs, 0 dropped records) and holds one flight of 301 s (76.5–377.4 s, measured from the controller's own takeoff and landing events).

**What the log can say about the current tune:**
- The gains look like firmware defaults. Roll is P 0.135, I 0.135, D 0.0036, and the D and gyro filters are at 21 Hz, which is half of `INS_GYRO_FILTER` (42 Hz). The set is consistent with itself, but that doesn't mean it suits this airframe. Large props usually want different gains, and only the rate data can show which way.
- The tune isn't obviously bad. The aircraft drifted off its commanded attitude 0.07 times per second (21 times in 301 s; the warning level is 0.2). The largest attitude error was 5.9° on roll and 6.4° on pitch, just over the 5° warning level.

**Settings to change before the tuning flight:**

| parameter | now | set to |
|---|---|---|
| `LOG_BITMASK` | 180222 | **180223** (records the rate loop at full speed) |
| `LOG_BLK_RATEMAX` | not in log | 0 |
| everything else (`LOG_FILE_RATEMAX` 0, `INS_LOG_BAT_MASK` 0, `LOG_FILE_BUFSIZE` 172, `SCHED_LOOP_RATE` 400, `AUTOTUNE_AGGR` 0.075) | | already OK |

**How to fly it.** You don't need AutoTune. Take off in ALT_HOLD, hover for 30 s, then do 60 s of sharp roll inputs (±15–20°, released quickly), 60 s of pitch inputs and 30 s of yaw inputs, then land. Keep the gains the same for the whole flight. Then run `alog tune <new>.bin`. Only apply values it rates at confidence 0.7 or higher. Values between 0.4 and 0.7 should be flown as a test first, and anything marked "withheld" should not be applied. Once you're done tuning, set `LOG_BITMASK` back to 180222.

**Fix these before the tuning flight, because they would skew the result:**
1. **⚠️ Pitch trim is 58 µs (failure threshold 25 µs).** It's the same in level hover, so it's a centre-of-gravity offset or a steady breeze. The log can't separate the two because the hover didn't face enough directions. Hover for about 20 s facing each of N, E, S and W, then move the battery. The 12.2% raw RPM spread between motors comes from this too. Once the different throttle levels are allowed for, the spread is only 2.2%, so no motor looks faulty.
2. **⚠️ `MOT_THST_HOVER` is out of date.** It's set to 0.296 but the aircraft learned 0.204 in flight. The tuning tool uses hover throttle, so let it update or set it before the tuning flight.
3. **The harmonic notch is off.** Its parameters aren't in the log, which on this firmware means it's disabled. Sorting out the vibration filtering is normally done before touching gains. This log has no vibration or IMU data to check it with.

Also, the pre-arm check complained about the compass ("mag field xy diff 101 > 100"). It has nothing to do with tuning, but it's worth looking at.
