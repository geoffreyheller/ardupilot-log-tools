---
name: ardupilot-log-analysis
description: >-
  Analyse ArduPilot / ArduCopter dataflash flight logs (.bin) with the alog command-line
  tool: log integrity, how many flights and which airborne window, vibration and
  accelerometer clipping, harmonic notch tracking, motor and ESC balance, standing trim and
  CG offset, GPS quality and glitches, EKF innovations, compass, battery current-sensor
  calibration, CPU load, and PID gain recommendations with a confidence (or the logging a
  tuning flight needs). Use when someone shares or asks about an ArduPilot or ArduCopter
  .bin log, a multicopter flight log, "why did my drone do that", vibration, a notch, motor
  balance, a before/after comparison of two flights, or what to log for PID tuning. Not for
  PX4 .ulg logs or MAVLink .tlog telemetry; a .log text export is read but flagged as
  second-class.
license: MIT
compatibility: Requires Python 3.9+ and either uv, pipx or a pip-installed alog; on first use, network access and git to fetch it. Windows, Linux and macOS.
metadata:
  version: "2.1.0"
  source: "https://github.com/geoffreyheller/ardupilot-log-tools"
---

# ArduPilot log analysis

Analyse an ArduCopter dataflash log (`.bin`) with `alog`. It reports every number with the
window it was measured over, the threshold it was graded against, and where that threshold
came from. The tool is half the value. The other half is knowing which numbers matter,
which window they must be taken over, and which plausible-looking conclusions are wrong;
that is what this file and `references/` carry.

## Run `alog`

`scripts/alog.sh ARGS` (POSIX shells, Git Bash) or `scripts/alog.ps1 ARGS` (PowerShell) runs
the version this skill was written for: an installed `alog` when `alog --version` prints
`ardupilot-log-tools` followed by this skill's `metadata.version`, otherwise the same version
through `uvx` or `pipx`, without installing anything permanently. If neither is available it
prints the `pip install` line and exits 3. When `alog --version` already matches, call
`alog` directly. The examples below write `alog`.

Exit codes: 0 pass, 1 warn, 2 fail, 3 the input could not be analysed. Add `--json` to any
command for one machine-readable document (`references/json-output.md`). `alog docs` lists
the documentation for the installed version, including the provenance documents this skill
leaves out; `alog docs pitfalls` prints one.

## Always start here

```bash
alog info flight.bin           # identity, integrity, flights, what was logged at what rate
alog all  flight.bin --json    # the standard battery, one JSON document
alog all  flight.bin           # the same as markdown
```

`info` comes first, always. Read, in order: the **integrity block** (a `RESYNC` or
`DUPLICATE_DATA` changes the meaning of every later number; codes in
`references/integrity-codes.md`); the **flight table** (more than one flight means every
later command analyses one of them); and the **coverage table** (a message that was not
logged makes its check `SKIP`, and the spectral-reach line says whether any spectrum from
this log can show the motor at all). Then run the check, or the battery, the question
needs.

## Ground rules

Each of these exists because ignoring it produced a wrong or unusable answer.

<!-- generated:ground-rules -->
1. **Read the integrity block first.** It is at the top of every report. A `RESYNC` or
   `TRUNCATED_TAIL` changes what the rest of the numbers mean. Codes are in
   `references/integrity-codes.md`. Nothing was repaired; the numbers are computed on the
   data as logged.
2. **One change per flight.** Never batch parameter changes. A diagnostic is only
   trustworthy when one thing moved between two logs.
3. **Name the validation test.** A recommendation without "and here is the flight that
   proves it" is half a recommendation. Say what to fly and what to measure.
4. **Cite the parameter snapshot.** Values differ between dated captures.
   `alog params <log> --diff <file>` diffs the in-log parameters against one;
   `--non-default` lists what differs from the firmware defaults.
5. **Distinguish configured from measured.** A `.param` file says what the aircraft was
   *told*; the log says what it *did*. `MOT_THST_HOVER` is the standing example — it is
   relearned in flight; the `paramcheck` section compares it with the learned `CTUN.ThH`.
6. **Quote the window and the method.** Every number depends on it. A 4.7 % vs 6.31 %
   discrepancy between two analyses of the same log turned out to be nothing but
   whole-log versus airborne-only. The method string is in every report and every JSON.
7. **State the number, not the verdict.** "Vibration is fine" is useless next flight.
   "VibeX p95 8.2 m/s², 2 clip events" is a baseline.
8. **Report "not logged" as not logged.** A check that could not run is `SKIP`, never a
   pass. `alog` does this automatically and lists the skips under the verdict; do the same
   in prose.
9. **Report defaulted parameters.** When a section says "Parameters NOT in the log,
   defaults assumed", every number that depends on them is conditional.
10. **Never carry conclusions between aircraft.** Two airframes that share a transmitter
    share nothing else.
<!-- /generated -->

## Pick the window

Every number depends on the window it was taken over, and the method string that chose it
is in every report and every JSON document: quote it. `--window METHOD`:

<!-- generated:window-table -->
| method | definition | when to use |
|---|---|---|
| `ev` (first choice under `auto`) | `EV` 28 NOT_LANDED → 18 LAND_COMPLETE | general analysis; the FC's own opinion of "flying" |
| `rpm` | fleet-mean ESC fundamental above a floor **derived from the log**: 60 % of the median while spinning (> 20 Hz). `--hz-floor HZ` overrides it | **motor, notch and vibration work** — defined identically regardless of what the land detector believed. The method string names the floor and the median it came from |
| `throttle` | `CTUN.ThO` > 0.15 | fallback when there is no ESC telemetry |
| `arm` | `EV` 10 ARMED → 11 DISARMED | only when you actually want ground time included |
| `hover`, `hover:N` | the longest (or the Nth) steady-hover chunk from `hover_chunks()`: LOITER / ALT_HOLD / POSHOLD, sticks centred, ≥ 10 s | FFT peaks, motor balance, vibration baselines — anything only meaningful in steady hover. `alog hover` lists them. Not a flight |
| `none` | the whole log | explicitly |
| `T0:T1` | explicit seconds since boot, e.g. `120:180` | a manoeuvre, a GPS outage |
<!-- /generated -->

If a method cannot be applied the window falls back to the whole log and the method string
says `FALLBACK`. A log can hold more than one flight; the window is always one of them, its
method string says which, and analysing one of several is a `WARN`:

<!-- generated:flight-table -->
| flag | effect |
|---|---|
| *(default)* | the **longest** flight; ties broken to the earliest |
| `--flight N` | flight N, 1-based. Out of range is exit 3 naming what exists |
| `--flight all` | the battery once per flight: a markdown block each and one combined verdict, or `per_flight` in JSON *instead of* `sections`. Rejected on `dump`, `fft` and `compare` |
<!-- /generated -->

`references/guide.md` §3 has the rest: the derived `rpm` floor and `--hz-floor`, hover
chunks, `--pad`, and what `alog compare` checks before calling two windows comparable.

## Question → recipe

Each recipe in `references/recipes.md` gives the commands, the lines of output to read, the
decision they support and the flight that validates it.

| question | recipe |
|---|---|
| What is this log, is it intact, what can it tell me? | Skill 0 |
| The log holds more than one flight | Skill 1 |
| Which window should this number come from? | Skill 2 |
| Motor balance, CG offset, a bent prop, a bad bearing | Skill 3 |
| Is the harmonic notch working, and where should it be? | Skill 4 |
| Vibration, clipping and spectra | Skill 5 |
| Is the tune OK? (before touching a gain) | Skill 6 |
| Recommend PID gains, or what to log so that a log can | Skill 7 |
| GPS quality, an antenna change, two receivers | Skill 8 |
| Is the current sensor lying, and by how much? | Skill 9 |
| Did one change make it better? (before/after) | Skill 10 |
| EKF, compass and estimator health | Skill 11 |
| Parameters: what flew, what changed, what differs from defaults | Skill 12 |
| Events, how the log ended, "why did it do that" | Skill 13 |
| The raw data: fields, a CSV dump, embedded files | Skill 14 |
| Write it up | Skill 15 |

Two of these are easy to get confidently wrong. **Notch:** judge it by `FCNS.CF` (the
frequency the filter applied) against `ESC.RPM / 60`, never by `FTN1.PkAvg` (the in-flight
FFT's opinion). **PID gains:** the standard `LOG_BITMASK` logs the rate loop at 10 Hz, and
`alog tune` refuses such a log with an `ERROR:` block naming the parameters to set; do not
derive gains from a refused log by other means.

## Write the report

Copy `references/report-template.md`. The integrity block and the window come first, then a
one-paragraph verdict, then the findings ordered by value rather than by subsystem, each
with the number, the threshold, the interpretation and the validation flight. Call out a new
or worsening finding even when nobody asked about it. Correct visibly: keep a superseded
number and date the amendment. `references/guide.md` §11 has the reasoning.

## Housekeeping

`alog` caches each parsed log beside it as `<log>.dfcache`. Delete those when done, or keep
them out of the log's folder (an SD card, a synced or read-only folder) with
`alog --cache-dir DIR ...` or the `ALOG_CACHE_DIR` environment variable. Global flags
(`--cache-dir`, `--no-cache`, `--strict`) go before the subcommand; `--json` may go anywhere.

## References

Read these as needed; they are the tool's own documentation for this version.

| file | read it when |
|---|---|
| `references/guide.md` | the full operating guide: every check in `alog all`, trim decomposition, notch, FFT, PID tuning, pitfalls |
| `references/recipes.md` | you know the question (the index above) |
| `references/rules.md` | the contract behind every output: fail loudly, state the number, the window and the source |
| `references/pitfalls.md` | before concluding anything surprising |
| `references/integrity-codes.md` | a report shows an integrity or data-quality code |
| `references/json-output.md` | reading `--json` output |
| `references/thresholds.md` | where a threshold came from, and how sure it is |
| `references/messages.md` | what a dataflash message or field means, with units |
| `references/param-decodings.md` | decoding a bitmask, enum or device ID parameter |
| `references/current-sensor-calibration.md` | calibrating the battery current sensor |
| `references/report-template.md` | writing the report |
