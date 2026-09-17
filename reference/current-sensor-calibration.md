# Calibrating an analog battery current sensor

`BATT_AMP_PERVLT` is one number, and getting it right took one afternoon of bench work
after six weeks of flights and charge cycles that produced four mutually contradictory
answers. This file records every method tried, what each actually measures, and why most
of them cannot work — so the next calibration starts with the ten-minute method instead of
ending with it.

Written from a 2S 5-inch quadcopter with a 4-in-1 BLHeli_S/Bluejay ESC feeding
`BATT_MONITOR=4` on an analog pin. The reasoning generalises to any board whose current
signal comes from an ESC shunt.

---

## The one thing to understand first: measurement boundaries

ArduPilot computes

```
current = (V_pin − BATT_AMP_OFFSET) × BATT_AMP_PERVLT
```

from a signal the **ESC** produces. On the aircraft this was measured with every motor
stopped and the avionics fully powered:

```
BAT.Curr = 0.000194 A
```

Not 0.3 A, not 0.03 A — zero. **The ESC's current-sense output is motor-path only.** The
flight controller, GPS, receiver, video transmitter and lighting are invisible to it.

Every other instrument you might calibrate against — a charger, an inline watt meter, a
clamp on the battery lead — measures the **whole aircraft**. The two numbers have different
boundaries, and every failed method below failed on that difference or on something it
hides.

**First action in any calibration: measure the idle draw in the configuration you will
calibrate in, and subtract it.** On the example aircraft this was 0.34 A at 8.1 V (2.8 W).
Measure it *in that configuration* — with a USB cable attached, part of the avionics load
can come from the USB port instead of the pack, changing the offset. (On this aircraft it
did not, but only because it was checked.)

---

## Methods, in the order they should be tried

### 1. Inline watt meter at the hover operating point ⭐ this is the method

Props on, aircraft restrained, watt meter between pack and aircraft, GCS connected.

The trick that makes it work: **ArduPilot's motor test percentage maps onto the PWM output
range, so a test at the percentage matching the hover `RCOU` duty reproduces the hover
condition on the bench.** On the example aircraft hover `RCOU` was 1487 µs, so a 50 % motor
test (~1500 µs) put it within 1 % — and with all four props fitted, the thrust produced is
the aircraft's own weight. Not a heavy or dangerous bench load, and the restraint only has
to hold the airframe down against its own hover thrust.

```
BATT_AMP_PERVLT_new = PERVLT_loaded × (meter A − idle A) ÷ reported A
```

Cost: ten minutes. Accuracy: limited by the watt meter, a few percent. It was the last
method tried and should have been the first.

**Take at least two points** — one at hover duty and one 1.5–2× higher. The second is not a
luxury; see "The sensor is not linear" below.

**Read both instruments at the same steady moment**, with all motors spinning together. A
GCS telemetry value lags by up to a second; a motor test that steps through motors one at a
time is sampling a different machine each second and the comparison is meaningless.

### 2. A calibrated reference sensor as a second monitor

An I²C sensor (INA226/INA228 class, ~2 % accuracy, a few grams) configured as `BATT2_*`
for one flight, then removed. Compare `BAT` against `BAT2` across the whole throttle range,
in the air, at every operating point including the peaks. This is the only method on the
list that reaches full-throttle current.

Worth it when the peaks matter — setting `MOT_BAT_CURR_MAX`, or checking a pack against its
C rating. Overkill for mAh bookkeeping.

### 3. Model-based estimates — useful for sanity, not for setting the parameter

Four were run on the example aircraft before any direct measurement. Scored against the
measured answer of 35.9:

| method | answer | error |
|---|---:|---:|
| Winding-drop regression (below) | ~35 | −3 % |
| Voltage/state-of-charge across a flight | 32–47 | contains it |
| Pack resistance vs the log's sag slope | ~45 | +25 % |
| eCalc drive model | ~51 | **+42 %** |
| Momentum theory with an assumed figure of merit | 25–34 | −20 % |

**Use the ensemble, never a single one.** The centre of the four was 40, which was 11 %
off — better than any individual method except the first, and good enough to rule out the
badly wrong candidates (one earlier value implied 16 % propulsive efficiency, which no
multirotor achieves). Their real job is to catch an answer that is wrong by 2×, not to
produce the value.

**The winding-drop regression is the best of them and costs nothing.** For a brushless
motor, `duty × V_pack = RPM/KV + I_phase × R`, so

```
4 × duty × (duty·V − RPM/KV)  =  I_battery × R
```

Every term on the left is in the log — `RCOU` duty, `BAT.Volt`, `ESC.RPM` — and `KV` is
usually derivable. Regress the left side against logged current across the whole throttle
range; the slope is `R × (true ÷ logged)`. With a motor resistance you trust, that gives
the scale factor. On the example aircraft: slope 0.4608, r = 0.909, n = 4406.

Its weakness is `R_phase`, which is rarely published. Scaling from a catalogued variant of
the same motor works — rewinding the same stator gives `R ∝ 1/KV²` and `Io ∝ KV` — and that
estimate turned out to be the most accurate method on the list.

**Momentum theory is the weakest** because it needs a propeller figure of merit, and the
plausible range (0.35–0.60) is wider than the answer. The example aircraft's real figure of
merit was ~0.40; assuming 0.50–0.60 produced a bracket that excluded the true value.

**eCalc ran 42 % high**, almost certainly because the propeller was a generic catalogue
entry rather than the fitted one. Its hover RPM agreed with the log within 4.5 %, which is
a *thrust* check and says nothing about power. Treat its currents as an upper bound.

### 4. Charger mAh — retired, and here is why

The standard advice:

```
new BATT_AMP_PERVLT = old × (mAh the charger returns ÷ mAh logged as consumed)
```

Three iterations on the example aircraft gave **50 → 72.3 → 30.8**, and a fourth reading
pointed at **78**. It never converged, because the method has at least four uncontrolled
variables:

1. **The avionics term** (the boundary problem above), which scales with total
   pack-connected time — not flight time. Nobody records pack-connected time.
2. **Start state of charge.** "Fully charged" has to mean the same thing before the flight
   and after the recharge. Discharging to storage and balance-charging to full before the
   flight fixes this, and is worth doing if the method is used at all.
3. **What the charger's counter is counting.** A counter that was not zeroed, or a cycle or
   balance routine whose displayed mAh includes a discharge leg, inflates the number
   silently. One reading on the example aircraft came in at 1222 mAh against a flight that
   the pack's own voltage said had consumed 600–850 mAh.
4. **Charger coulombic efficiency and termination**, smaller but real.

**The arithmetic also hides how bad it is.** Since logged mAh is strictly proportional to
`BATT_AMP_PERVLT`, one iteration *should* converge immediately: `new = old × C/(k·old)`
has no dependence on `old` at all. So successive iterations landing on different answers
is proof that the charger reading itself is changing between tests, not that the method is
converging slowly. **If two iterations disagree, stop — do not run a third.**

Normalising each flight by the scale it flew (`logged mAh ÷ BATT_AMP_PERVLT`) gives a
scale-free "charge integral" that makes this visible immediately: two flights with
near-identical integrals had charger readings differing by 2.4×.

The formula direction is also easy to get backwards. It is `charger ÷ logged`. Backwards,
it roughly doubles an over-read instead of correcting it (issue #2).

### 5. Props-off throttle sweep — does not work, do not bother

Tried because it needs no restraint and no risk. It fails on two counts:

- **The current is an order of magnitude too small.** Unloaded motor current saturates once
  windage and iron losses dominate: on the example aircraft the sweep reached 0.34 A at
  30 %, 0.65 A at 50 % and 0.75 A at 70 % — a 2.5:1 span topping out at **22 % of hover
  current**. Reaching the real operating point means extrapolating 4.5× through exactly the
  region where a shunt amplifier misbehaves.
- **It does not repeat.** A second reading at the same 50 % differed by **20 %** (0.65 vs
  0.54 A motor-only). With no aerodynamic load there is nothing holding the operating point
  still.

Fit props. Two props on opposite arms halves the thrust and still gives four times the
signal; four props at hover duty is better still and no harder to restrain.

---

## The sensor is not linear — calibrate at the operating point that matters

Three bench points on the example aircraft, all taken with `BATT_AMP_PERVLT=40` loaded:

| configuration | true motor A | reported A | implied scale |
|---|---:|---:|---:|
| 2 props, 50 % | 2.77 | 2.82 | 39.3 |
| **4 props, 50 % ≈ hover** | **4.78** | **5.32** | **35.9** |
| 2 props, 70 % | 5.66 | 6.50 | 34.8 |

The effective scale falls smoothly from 39.3 to 34.8 as current doubles. Least squares over
those points plus the measured zero: `reported = 1.145 × true − 0.12`. The intercept is
negligible against a measured zero of 0.000194 A, so this is **gain, not offset** —
`BATT_AMP_OFFSET` stays 0.

Consequences:

- **A single-point calibration is only valid near that point.** Calibrating at 2.8 A and
  flying at 4.8 A would have left a 9 % error.
- **Choose the point deliberately.** Hover is where the mAh and endurance numbers are
  spent, so the value was set to be exact there — 8 % low at 2.8 A, 0 % at hover, 3 % high
  at 5.7 A. Document the error curve rather than pretending the number is exact everywhere.
- **Do not extrapolate to the peaks.** Nothing above 5.7 A was ever measured on an aircraft
  whose full-throttle passes draw over 20 A. Since the over-read grows with current, the
  true peak is *below* the reported one — reassuring for a pack's C rating, but not a
  number to set a current limiter from.

A four-prop point also cleared the obvious suspect for the non-linearity: it is not an
artefact of running some motors unloaded, since it reproduces with uniform loading.

---

## Consistency checks that catch a wrong answer early

**A subset cannot exceed the total.** If the flight controller reports more *motor-path*
current than a whole-aircraft meter measures, the flight controller over-reads. No
modelling, no assumptions. This appeared at two of the three bench points and was the first
hard evidence of the over-read.

**Specific thrust.** All-up weight ÷ hover electrical power. A small multirotor lands
around **5.5–7.5 g/W**; the example aircraft measured 7.3. A candidate scale implying
8.6 g/W was too good, and one implying 3.5 g/W was impossibly bad. This single number
disqualified two of the four historical values in seconds.

**Separate sag from state of charge.** Regressing pack voltage on current alone conflates
the two, because voltage also falls as charge is consumed. Add a consumed-charge term:

```
V = OCV₀ − α·Q − R·I
```

On the example flight the naive slope was 40.0 mΩ and the corrected one 33.8 mΩ — and the
naive figure had been used to argue the sensor read 2× high, a conclusion that was wrong.
The regression's `OCV₀` intercept is a bonus: it estimates the pack's open-circuit voltage
at the start of the window.

**Do not calibrate from datasheet pack resistance.** The plausible spread in 18650 DC
internal resistance (13–25 mΩ per cell, depending on whether you mean AC-IR, DC-IR or what
a 10 Hz log actually sees) is wider than the answer you are trying to find.

**Check the premise before declaring a second fault.** The same `OCV₀` intercept was used
to conclude the voltage sensor read 4 % low — against an assumed 4.20 V/cell full charge.
The charger's "full" was actually ~4.07 V/cell, so the flight controller was within 1 % and
there was nothing wrong. A meter reading is worth more than an assumption about someone
else's equipment.

---

## Instrument behaviour worth knowing

**Watt meters may not sample volts and amps together.** Dividing displayed watts by
displayed amps gave implied pack voltages of 8.24, 7.94 and 7.27 V across a rising load —
0.9 V of apparent sag at 1 A, which no healthy 2S pack does. At the next point it recovered
to 7.98 V. A genuine bad joint or tired pack cannot recover as load *increases*, which is
what identified it as a sampling artefact. **Read the voltage digit; do not infer it.**

**Repeat every point.** The 20 % repeat scatter props-off is what killed that method, and
it was only visible because a point was taken twice.

**Telemetry values lag.** Read the GCS number and the meter at the same steady moment, and
give each point several seconds to settle.

---

## The procedure, condensed

1. **Idle offset.** Aircraft powered, motors stopped, in the same configuration (USB or
   not) as the calibration. Record meter amps. Confirm against the log's motors-stopped
   `BAT.Curr` — if that is ~0, the sensor is motor-path only and the offset must be
   subtracted from every meter reading.
2. **Find the hover duty** from a previous flight's `RCOU`, and convert to a motor-test
   percentage of the `MOT_PWM_MIN`–`MOT_PWM_MAX` range.
3. **All props on, aircraft restrained**, meter inline, GCS connected.
4. **Motor test at that percentage**, all motors together. Record meter amps, meter volts
   and the reported current at the same steady moment. Repeat the point.
5. **Second point** 1.5–2× higher for the gain check.
6. `PERVLT_new = PERVLT_loaded × (meter − idle) ÷ reported`, at the hover point.
7. **Check** `BATT_AMP_OFFSET` against the motors-stopped reading, and the result against
   specific thrust.
8. **Record the error curve**, not just the number.

Safety: the restraint holds the aircraft's own hover thrust at step 4, which is modest, but
the props are spinning near your hands. Two props on opposite arms halves it if the
restraint looks marginal.

---

## What it cost

Six weeks, several flights, several charge cycles and four contradictory values, versus ten
minutes of bench work with a $12 watt meter. The direct measurement at the operating point
was tried last because it seemed to need equipment and restraint that the indirect methods
avoided — but the indirect methods needed a flight each, and none of them converged.

**When a parameter can be measured directly, measure it directly, first.**
