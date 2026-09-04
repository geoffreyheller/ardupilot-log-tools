# Parameter and bitfield decodings

Things that had to be worked out once. Vehicle-specific values live in each vehicle's
`CLAUDE.md`; this file holds the *decoding rules*, which are the same everywhere.

## Device IDs (`INS_ACC_ID`, `INS_GYR_ID`, `COMPASS_DEV_ID`, `BARO1_DEVID`)

Bit layout: `bits 0-2 bus_type, 3-7 bus, 8-15 address, 16-23 devtype`.
`bus_type`: 1 = I²C, 2 = SPI, 3 = UAVCAN/DroneCAN, 4 = SITL.

Worked examples:

```
3408138  -> 0x340A0A   devtype 0x34 = ICM-42688, SPI, bus 1
1453057  -> 0x162C01   devtype 0x16 = QMC5883P, I2C bus 0, addr 0x2C
 855297  -> 0x0D0D01   devtype 0x0D = QMC5883L, I2C, addr 0x0D
 423425  -> 0x067601   devtype 6    = DPS310,   I2C, addr 0x76
```

**A device ID does not identify a physical part**, only the sensor die and where it sits.
Two different boards carrying the same chip at the same address report the same id.

## Enums

```
MOT_PWM_TYPE      : 5 = DShot300, 6 = DShot600
SERVO_DSHOT_ESC   : 2 = no EDT, 4 = BLHeli_S/Bluejay + Extended DShot Telemetry
SERIALn_PROTOCOL  : -1 disabled, 2 MAVLink2, 5 GPS, 16 ESC Telemetry, 23 RCIN, 33 DJI FPV
INS_HNTCH_MODE    : 0 Fixed, 1 Throttle, 2 RPM sensor, 3 ESC telemetry, 4 Dynamic FFT
INS_HNTCH_OPTS    : bitmask - 1 Double, 2 Dynamic, 4 Loop-Rate, 8 AllIMUs, 16 Triple
INS_LOG_BAT_OPT   : 0 Pre-filter, 1 Sensor-rate, 2 Post-filter, 4 Pre+post-filter
EK3_SRC1_*        : POSXY/VELXY/VELZ 3 = GPS, POSZ 1 = Baro, YAW 1 = Compass
BATT_MONITOR      : 4 Analog V+I, 9 ESC, 31 Analog Current Only (skips voltage entirely)
GPS_AUTO_SWITCH   : 4 = prefer primary, fall back on fix loss
FRAME_CLASS       : 1 Quad, 2 Hexa, 3 Octa, 4 OctaQuad, 5 Y6, 7 Tri, 14 Deca (full table in dflog/frames.py)
FRAME_TYPE        : 0 Plus, 1 X, 2 V, 3 H, 12 BetaFlightX, 13 DJIX, 14 ClockwiseX
```

## Derived quantities

```
motor noise Hz          = ESC.RPM / 60
eRPM -> RPM             = eRPM / (SERVO_BLH_POLES / 2)
motor saturation ceiling= SERVO1_MIN + MOT_SPIN_MAX * (SERVO1_MAX - SERVO1_MIN)
analog current          = V_adc * BATT_AMP_PERVLT + BATT_AMP_OFFSET
current recalibration   = BATT_AMP_PERVLT * (logged mAh / charger mAh)
loaded KV               = RPM / (pack_volts * mean output duty)
                          nameplate KV is typically loaded KV / 0.75-0.85
PM.Load                 = percent * 10
FFT bin width           = sample_rate / FFT_WINDOW_SIZE
notch BW rule of thumb  = FREQ / 2  (FREQ / 4 when tracking three peaks)
```

## Quad-X mix factors

ArduPilot `AP_MotorsMatrix::add_motor(motor, angle_deg, yaw_factor, order)`:
`roll_factor = cos(angle + 90)`, `pitch_factor = cos(angle)`, then
`normalise_rpy_factors()` divides each family by its peak. Quad X:

| motor | angle | position | yaw | roll | pitch |
|---|---|---|---|---|---|
| M1 | 45 | front-right | CCW +1 | −1 | +1 |
| M2 | −135 | rear-left | CCW +1 | +1 | −1 |
| M3 | −45 | front-left | CW −1 | +1 | +1 |
| M4 | 135 | rear-right | CW −1 | −1 | −1 |

Positive roll = roll right, positive pitch = nose up, positive yaw = yaw right.

## Log bitmask

`LOG_BITMASK` is a bitfield; `180222` = bits 1–13, 15, 17 (attitude medium, GPS, PM, CTUN,
NTUN, RCIN, IMU, battery, RCOUT, PID, compass, motors — no fast/raw IMU). Keep the raw-IMU
bit **off** when using batch logging.
