# Test fixtures

Real flight logs, cut down and scrubbed by `tools/make_fixture.py`. They exist because the
synthetic logs in `tests/synthlog.py` cannot reproduce the failure that motivated issue #3:
a segmenter threshold that was right for one aircraft and wrong for another. Every number
pinned against a fixture in `tests/test_largeprop.py` was taken on the fixture itself, after
the scrub, so the tests do not depend on the original file.

## `largeprop-quad.bin`

A 10-inch, 380 KV quad-X with a 30 s pre-flight ground period, one 301 s flight (`EV`
NOT_LANDED 76.5 s to LAND_COMPLETE 377.4 s) that includes three LOITER hover chunks, and a
landing. Its hover fundamental is ~84 Hz, **below** the 90 Hz `rpm` floor this toolkit
used to hardcode, which split the one flight into five.

| property | value |
|---|---|
| firmware / board | ArduCopter V4.7.0-dev, 3DRControlN1 (MCU serial zeroed) |
| messages kept | ESC (decimated 10:1 per instance, ~6 Hz), CTUN, RCOU, RCIN, ATT, BAT, GPS, GPA, UBX2, EV, MODE, ARM, PARM, MSG, VER, PM, DSF, MOTB, plus every FMT/FMTU/UNIT/MULT |
| messages dropped | everything else, including IMU, XKF*, POS, AHR2, ORGN, TERR, CMD, FILE |
| GPS | two receivers: instance 0 u-blox (UBX2 present, 5 Hz, `GPA.Delta` 200 ms), instance 1 NMEA (10 Hz, mostly no fix, `GPA.VDop` 655.35) |
| positions | shifted so the first 3D fix is at 30.0 N, 140.0 W, 100 m (open ocean); relative motion preserved |
| notch | `INS_HNTCH_ENABLE=0`, no FCNS - the "notch disabled" case for issue #9 |
| current sensor | `BATT_MONITOR=9` (ESC telemetry); reads ~12.3 A with every motor provably stopped - the zero-offset case for issue #6 |
| motor map | `SERVO1..4_FUNCTION = 35,34,33,36` (non-identity) |

Rebuild (the source log is not distributed):

```bash
python tools/make_fixture.py "<source>.bin" tests/fixtures/largeprop-quad.bin \
    --keep ESC,CTUN,RCOU,RCIN,ATT,BAT,GPS,GPA,UBX2,EV,MODE,ARM,PARM,MSG,VER,PM,DSF,MOTB \
    --decimate ESC=10
```

The tool prints every MSG text that survives the scrub; review that list before committing
a new fixture.
