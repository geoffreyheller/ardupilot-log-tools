# Flight log analysis — <Vehicle>, YYYY-MM-DD

**Log:** `<file>.bin` (<size>) · **Params that flew:** `<Vehicle> Ardupilot Params - YYYY-MM-DDx.param`
**Comparison:** `<earlier log>.bin` · **Firmware:** ArduCopter x.y.z (<hash>)
**Armed:** <n> s · **Airborne:** <n> s · **Window:** <t0>–<t1> s via <method>
**Analysed:** YYYY-MM-DD with `ardupilot-log-tools` v<x> (`python alog.py all --json`)

> State any oddity about the log itself here — an unset RTC giving a 1980 date, a
> truncated file, a lost predecessor flight.

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
