# RULES.md — the contract

This project exists so that an AI agent can analyse an ArduCopter dataflash log
efficiently and, above all, accurately. These rules are the contract that makes that
possible. They bind two audiences: an agent **using** the tools on a log, and an agent
**changing** the tools. `CLAUDE.md` is the long-form guide; this file is the part you must
not break.

## 1. Fail loudly. Never repair silently.

- Every deviation from a well-formed input is recorded in `log.diagnostics` with a stable
  code, a severity, a byte offset and a count, printed at the top of every report and
  carried in every JSON document. The codes are in `reference/integrity-codes.md`.
- Nothing is repaired. A truncated message is dropped and reported; a resync is counted and
  reported; a malformed `FMT` is decoded by its format string and reported. The numbers in
  the rest of the report are computed on the data as logged.
- A check that could not run is `SKIP`, never `PASS`. "Not logged" is a finding.
- A check that crashes is `FAIL` with the exception named, never an empty section.
- A parameter that was not in the log and had to be defaulted is listed in the section that
  used it. A number that depends on a default is conditional on that default.
- A window method that could not be applied falls back to the whole log, and the method
  string says `FALLBACK`. An explicit `--window T0:T1` that is impossible is an error.
- A log holding more than one flight is reported as such. The window is always *one*
  flight, never the span across the ground time between two, and its method string names
  which (`, flight k of n`). Analysing one of several is a `WARN`, whether or not the
  flight was chosen explicitly - a report outlives the command line that produced it, and
  a reader must not have to know which flags were passed to know that a flight was left
  out. `--flight all` analyses every one.
- `--strict` turns any error-level integrity issue into exit code 3: the input is refused.
- Exit codes are the verdict: `0` pass, `1` warn, `2` fail, `3` the input could not be
  analysed. Nothing else is ever returned. Pipe closures are not failures.

## 2. State the number, the window and the source.

- Every result carries the value, the threshold, and where the threshold came from.
  "Vibration is fine" is not an output of this tool; "VibeX p95 8.2 m/s², warn 15, fail 30,
  source: ardupilot.org wiki" is.
- Every report names the window and the method that chose it. Numbers from different
  windows are not comparable and must not be compared.
- Thresholds live only in `dflog/checks.py::T`, each with a `source`. No number in a check
  is hard-coded. A new threshold without provenance is a defect.
- Configured is not measured. `.param` files say what the aircraft was told; the log says
  what it did. Keep them distinct in every sentence.

## 3. Output for agents first.

- `--json` produces one JSON document per invocation with the shape printed by
  `alog schema` (`reference/json-output.md`). Non-finite floats are `null`, never `NaN`.
  Markdown is the human rendering of the same data, not a different analysis.
- Tables are structured (`columns` + `rows`), not markdown strings inside JSON.
- Errors are JSON too when `--json` was asked for, with `exit_code: 3` and an `error`
  string that names what exists (`dump NOPE` lists the messages that do exist).
- Output is deterministic: same log, same version, same bytes. No timestamps, no
  randomness, no wall-clock in the report.
- `alog info` is the first command on any new log. It costs one parse and answers: what is
  this, is it intact, what was logged and at what rate, so nothing downstream is a surprise.

## 4. Analysis discipline (for the agent reading the log).

- One change per flight. A comparison is only trustworthy when one thing moved.
- Quote the window and the method - and the flight, on a log that holds more than one.
  Use `--window rpm` for motor, notch and vibration work and for every before/after
  comparison.
- Report what you were not asked about when it is worse than what you were asked about.
- A recommendation names its validation flight: what to fly and what to measure.
- Never carry conclusions between aircraft. Read the aircraft's own notes first.
- `RCOU.C<n>` is servo output n, not motor n. The channel map comes from
  `SERVOn_FUNCTION`; the motors check prints the map it used.
- `FCNS.CF` is where the notch was; `FTN1.PkAvg` is where the FFT thought the noise was.
  Only the first verifies a notch.
- An FFT is only as honest as its sample rate. The tool refuses irregular timing and
  states the Nyquist limit; do not read motor noise off a 25 Hz IMU stream.

## 5. Changing the tools.

- Run all four test files before and after: `tests/test_toolkit.py` (pinned regression
  figures on the reference logs, skipped when `LOG_DIR` is unset),
  `tests/test_parser_integrity.py` (synthetic malformed logs, every integrity code),
  `tests/test_cli.py` (exit codes and JSON contract) and `tests/test_flights.py`
  (flight segmentation and window selection). A change that makes a pinned figure
  move is wrong until proven otherwise.
- A new integrity condition gets a code, a severity, a test that triggers it, and a line in
  `reference/integrity-codes.md`.
- A new check is a `check_*(log, window) -> Section` in `dflog/analysis.py`, registered in
  `ALL_CHECKS`, using `Section.table()` / `Section.note()` so JSON and markdown stay
  identical, and it must return `SKIP` results for missing inputs.
- Platform-agnostic: no shebang-only invocations in docs, no shell-specific scripts as the
  only path, forward-slash-agnostic paths, UTF-8 output. CI runs Windows, Linux and macOS.
- The parser applies format-character scaling (`c C e E L`) and never `MULT`. Changing that
  convention is not a bug fix; it is a different tool.
- Decodings you had to work out go in `reference/`, so nobody derives them twice.
