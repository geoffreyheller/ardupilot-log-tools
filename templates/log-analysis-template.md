# Flight log analysis — <Vehicle>, YYYY-MM-DD

**Log:** `<file>.bin` (<size>) · **Params that flew:** `<Vehicle> Ardupilot Params - YYYY-MM-DDx.param`
**Comparison:** `<earlier log>.bin` · **Firmware:** ArduCopter x.y.z (<hash>)
**Armed:** <n> s · **Airborne:** <n> s · **Window:** <t0>–<t1> s via <method>
**Flights in log:** <n> (<list them>) — this report covers <which>
**Analysed:** YYYY-MM-DD with `ardupilot-log-tools` v<x> (`python alog.py all --json`)

> State any oddity about the log itself here — an unset RTC giving a 1980 date, a
> truncated file, a lost predecessor flight, a second flight in the same file that this
> report does not cover.

---

## Verdict

One paragraph. What is the headline, and is the valuable work repair or opportunity? Say
plainly if there is nothing wrong with the aircraft.

| | BEFORE | AFTER |
|---|---|---|
| <the single metric this flight was flown to move> | | |

---

## 1. <Highest-value finding first, not subsystem order>

The number, the threshold, what it means, and what would change it.

**Validation:** what to fly and what to measure to confirm. A recommendation without this
is half a recommendation.

## 2. ⚠️ NEW — <anything that got worse, whether or not it was asked about>

Mark new or degrading findings clearly. Include the before/after and say what changed
between the flights.

## 3. <Everything that checked out>

Short. Numbers, not adjectives — they are next flight's baseline.

## 4. Tune (`alog tune <logs>`, window <t0>–<t1> s via `<method>`)

> Delete this block when the section was refused. A refusal is one line here, verbatim:
> `ERROR: these log files cannot be used for PID tuning.` + the code(s), and the parameters
> to set (`LOG_BITMASK 180222 -> 180223`, …) go under "Recommended next steps" as the
> tuning flight. Nothing below the ERROR block is a gain.

| axis | param | current | recommended | change % | confidence | method | why |
|---|---|---|---|---|---|---|---|
| roll | `ATC_RAT_RLL_P` | | | | | `autotune-log` / `virtual-autotune` / `ceiling` / `step-rules` / `unchanged` | |

Copy the rows from the `recommendations` table. Keep `withheld` where the tool printed
it (confidence < 0.4; the value is in the JSON evidence only and is **not** applied).
For each row you propose to apply, quote the confidence components
(`prior × adequacy × excitation × consistency × agreement`) and the tier that produced
it; a `step-rules` row is uncalibrated and never applied. Then the evidence behind it:
the `step` row of the current gain set (frames, peak, overshoot/bounce ratio — direction
only, not graded — `SRate` p95, ceiling), the `plant` fit and `margins` (gain margin dB,
phase margin °, against 6 / 45), the `autotune` session agreement with `MSG` if tier A
ran, the tier-D `params` lines that were WARN, and the `logs` table (which log fed which
tier). State the defaults assumed.

**Validation flight:** one axis, one gain set. Fly the tuning profile again (30 s hover,
60 s sharp inputs on that axis), `alog tune before.bin after.bin`, and expect the ratios
to move toward 1.0, margins to stay above 6 dB / 45°, `SRate` p95 < 5 and `Dmod` 1.000;
`alog compare` for the rest.

---

## Recommended next steps

| # | Change | Flight test | Why now |
|---|---|---|---|
| 1 | | | |

**Do not batch these. One change per flight.**

---

## Method

Window <t0>–<t1> s via `<method>`; both flights run through identical code. Note any
deviation from the standard battery, and any metric where this flight's conditions make the
comparison invalid.

---

## Documentation drift to fix

- `<Vehicle>/CLAUDE.md` §N says X; this log shows Y.
- Param snapshot `<file>.param` is stale for `<param>` — the aircraft now holds `<value>`.
