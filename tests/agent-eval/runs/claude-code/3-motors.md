## Commands
cd "<ws>"; ls; S=.claude/skills/ardupilot-log-analysis; grep -n "Skill 3" -A60 $S/references/recipes.md | head -90; $S/scripts/alog.sh info largeprop-quad.bin
cd "<ws>"; .claude/skills/ardupilot-log-analysis/scripts/alog.sh motors largeprop-quad.bin --window rpm; echo "EXIT $?"
cd "<ws>"; rm -f largeprop-quad.bin.dfcache; ls

## Answer
The motors themselves are fine, but the aircraft isn't balanced. The front pair is working much harder than the rear, and there's a smaller yaw torque imbalance. That points to where the weight sits or to a steady wind, not to a bad motor or prop.

**Log and window:** ArduCopter 4.7.0-dev, Quad/X. The log has no integrity errors or warnings and holds one flight. Numbers are taken over **78.1–376.2 s (298 s)**, the time the motors were spinning above 50.3 Hz, which is 60% of their typical in-flight speed. The motor-to-output mapping was read from your `SERVOn_FUNCTION` settings, so the axis labels are reliable.

## What the log shows

| check | value | limits (warn / fail) | status |
|---|---|---|---|
| Pitch trim (front minus rear) | **+58.7 µs** | 10 / 25 | FAIL |
| Yaw trim (CCW minus CW) | **+21.7 µs** | 10 / 25 | WARN |
| Roll trim (left minus right) | −3.1 µs | 10 / 25 | PASS |
| Same trims in level hover only | pitch +58.9 µs over 56 s | difference 10 / 25 | PASS (same figure) |
| RPM spread between motors | **12.2%** | 3 / 8 | FAIL |
| RPM per unit of drive (RPM ÷ duty × voltage) | 2.2% (lowest M1 345.9, highest M4 353.5) | 3 / 6 | PASS |
| ESC temperature | hottest peak 37 °C, means within 1.0 °C | 80 / 100 | PASS |
| DShot error rate | 0% | 5 / 15 | PASS |
| Motor headroom | 1825 µs (p99.5) = 0.87 of the 1950 µs ceiling | 0.9 / 0.97 | PASS |

Average output per motor: M1 1597, M3 1573 (front) and M2 1536, M4 1517 µs (rear).

## What it means

1. **No motor or prop is dragging.** RPM per unit of drive is within 2.2% across all four. A bad bearing or a damaged blade would show up as one motor turning slower for the same power, and none does. The 12% raw RPM spread is the front motors being asked for more thrust, not a hardware fault. Temperatures and DShot errors are clean.
2. **The front is carrying more load (+58.7 µs).** That works out to a centre of gravity about **9.2% of the arm length forward** of centre. It reads the same in level, sticks-centred hover, so it isn't caused by flying forward. It is either a nose-heavy CG **or a steady wind on the nose**, and this log can't tell those apart. All the hover was flown facing one direction (around 151°), so the heading comparison that separates the two couldn't run.
3. **The yaw imbalance (+21.7 µs)** means the CCW motors (M1, M2) are working harder than the CW pair to hold heading. The usual causes are a motor mount rotated slightly on its arm, or a twisted arm. This is a warning, not a failure, so look into it after the pitch issue.

Headroom is still fine (0.87 of the ceiling), so this isn't a safety problem yet. But it costs efficiency, and the front motors run out of authority first.

## What to do

1. **Fly a short heading-box test before moving anything:** hover about 20 s facing each of N, E, S and W, sticks centred, in Loiter or AltHold. Then re-run `alog motors <log> --window rpm --arm-mm <CG-to-front-motor-line distance in mm>`. If the forward bias rotates with heading, it's wind. If it stays fixed to the airframe, it's CG. Passing `--arm-mm` turns the 9.2% into the number of millimetres to move the battery back.
2. **If it's CG,** move the battery aft by that amount. That should be the only change before the next flight. Compare the two logs: pitch trim should drop below 10 µs, and the motor RPMs should come together.
3. **For the yaw trim,** check that each motor mount is square to its arm and that no arm is twisted.
