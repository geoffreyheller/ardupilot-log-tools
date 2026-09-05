# JSON output contract (`alog --json`, schema `alog/2`)

Every `alog` invocation with `--json` (before or after the subcommand) prints exactly one
JSON document to stdout and nothing else. `alog schema` prints this contract with the live
check list and threshold table. Non-finite floats are `null`. Bytes are Latin-1 strings.

## Envelope (every command)

```json
{
  "schema": "alog/2",
  "tool": "ardupilot-log-tools",
  "version": "2.0.0",
  "command": "all",
  "log": {
    "path": "...", "file_name": "...", "file_size": 10475996, "n_messages": 228776,
    "integrity": { "ok": true, "worst": "warning", "n_errors": 0, "n_warnings": 1, "n_info": 1,
                   "issues": [ { "code": "TRUNCATED_TAIL", "severity": "warning", "subject": null,
                                 "message": "...", "count": 1, "first_offset": 10475991,
                                 "offsets": [10475991], "detail": { "bytes": 5, "msg_type": "RTC" } } ] }
  },
  "window": { "t0": 77.4, "t1": 277.8, "duration_s": 200.4,
              "method": "EV NOT_LANDED->LAND_COMPLETE", "note": "" },
  "exit_code": 2
}
```

`log` is absent for `schema`; `window` is present only when a window was used.

## Check reports (`all`, or any single check name)

```json
"sections": [
  { "key": "motors", "title": "Motors and standing trim", "worst": "FAIL",
    "results": [
      { "name": "roll trim", "status": "FAIL",
        "summary": "34.256 us standing trim (warn 10, fail 25)",
        "evidence": { "value": 34.256, "warn": 10.0, "fail": 25.0, "signed": 34.256,
                      "channel_map": { "M1": "C2", "M2": "C3", "M3": "C4", "M4": "C1" },
                      "map_source": "SERVOn_FUNCTION" },
        "severity": 20, "window": null,
        "source": "measured on the development logs; see reference/thresholds.md" } ],
    "tables": [ { "name": "trim", "columns": ["axis", "trim (us)", "reads as"],
                  "rows": [["roll", 34.3, "mean(left) - mean(right)"], ...] } ],
    "notes": [ "Mix: Quad/X\nChannel -> motor map from SERVOn_FUNCTION: M1=C2, ..." ] }
],
"verdict": { "counts": { "PASS": 40, "WARN": 6, "FAIL": 9, "SKIP": 3 },
             "findings": [ ...results with status WARN or FAIL, most severe first... ] }
```

- `status` is one of `PASS`, `WARN`, `FAIL`, `SKIP`. `SKIP` means the check could not run
  (message not logged, no ground truth); it is never a pass.
- `evidence.value` is the graded number; `warn`/`fail` the thresholds it was graded
  against; `source` where they came from. Extra keys are check-specific and documented in
  the check's docstring.
- `severity` is 10 for WARN and 20 for FAIL unless a check raises it.
- Section `key` is the check name usable as a subcommand (`alog motors ...`), except
  `integrity` (which has a richer standalone command) and `paramcheck` / `spectrum` (named
  to avoid clashing with the `params` and `fft` commands).

## Other commands

| command | payload keys |
|---|---|
| `info` | `info` (identity: firmware, vehicle, board, sha256, UTC start from GPS time, durations, counts), `quality` (data-quality Diagnostics), `coverage` (a section) |
| `integrity` | `structure`, `quality` (both Diagnostics dicts) |
| `types` | `present: [{name, count, rate_hz, instances, fields, units}]`, `declared_but_absent: [name]` |
| `fields MSG` | `message`, `format` (the FMT), `count`, `rate_hz`, `instance_field`, `fields: [{name, type, unit, mult, range}]` |
| `dump MSG` | `message`, `n`, `rows: [ {field: value} ]` |
| `params` | `n`, `params: {name: {value, default}}`, `changes: [{t, name, old, new}]`; with `--diff`: `diff_file`, `unparsed_lines`, `differences: [{name, in_log, in_file, note}]` |
| `compare` | `logs: [{file_name, path, integrity, window}]`, `checks: [{name, per_log: [result or null]}]` |
| `fft` | `fft: {source, fs_hz, band_hz, timing, esc_fundamental_hz, peaks: {axis: [{freq_hz, psd, db_above_floor, prominence_db, order}]}, warnings, spectrum (with --spectrum), plot, csv}`; with `--list-sources`: `sources` |
| `files` | `files: [{name, bytes}]`, `written: [path]` |
| `schema` | `exit_codes`, `result`, `section`, `integrity`, `window`, `checks`, `window_methods`, `thresholds` |

## Errors

```json
{ "schema": "alog/2", "tool": "ardupilot-log-tools", "version": "2.0.0", "command": "dump",
  "error": "NOPE: not present in this log. Present types: AHR2, ATT, BAT, ...", "exit_code": 3 }
```

Exit codes: `0` every result PASS or SKIP; `1` at least one WARN; `2` at least one FAIL
(including an integrity error); `3` the input could not be analysed (missing file, not a
dataflash log, unknown check or message, impossible window, or `--strict` refused the log).
`fft` exits `1` when it emits a warning (Nyquist below the motor fundamental, no peaks, no
ESC telemetry for orders).
