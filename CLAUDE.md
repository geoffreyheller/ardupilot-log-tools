# CLAUDE.md

The operating guide for analysing a log is `GUIDE.md`, imported below so it loads with this
file. `RULES.md` is the contract it rests on and `SKILLS.md` the recipes; all three are
agent-neutral and ship inside the package (`alog docs guide`). This file adds only what is
specific to developing this repository.

@GUIDE.md

---

## Extending

- New check → a `check_*(log, window)` in `dflog/analysis.py` returning a `Section` built
  with `sec.add(Result)`, `sec.table(...)`, `sec.note(...)`, registered in `ALL_CHECKS`. It
  gets a CLI subcommand and a JSON section automatically. Missing inputs → `SKIP` results.
- New threshold → `dflog/checks.py::T`, with a `source`.
- New integrity condition → a code in `dflog/parser.py`, a test in
  `tests/test_parser_integrity.py` built with `tests/synthlog.py`, and a row in
  `reference/integrity-codes.md`.
- New decoding you had to work out → `reference/`, so nobody derives it twice.

`tools/bootstrap_pymavlink.py` vendors pymavlink from GitHub when you need `mavextra`
(WMM expected earth field, magfit) or `mavfft_isb.py`. It is not needed for anything in
`alog`. See `reference/existing-tools.md`.

There is no cloud CI; before merging, run `python tools/ci_local.py`, which runs the suite
under Python 3.9 and 3.12 on this machine (it covers this OS only). It also builds the
sdist and wheel from a clean copy of the tree and runs the installed wheel from outside the
repo, so a document missing from `MANIFEST.in` or `dflog/docs.py` fails there.

Run every test file after changing the parser, the trim math, the checks or the CLI. This
is the same command `tools/ci_local.py` runs, and it finds each `tests/test_*.py` by glob:

```bash
python tests/run_all.py                # every file, each in its own process; exit 1 if any failed
python tests/run_all.py cli flights    # only test_cli.py and test_flights.py
LOG_DIR=/path/to/logs python tests/run_all.py toolkit   # the reference-log pins
```

What each file covers, for running one directly:

```bash
python tests/test_parser_integrity.py
python tests/test_cli.py
python tests/test_flights.py
python tests/test_checks.py
python tests/test_largeprop.py
LOG_DIR=/path/to/logs python tests/test_toolkit.py
python tests/test_tune_extract.py      # PID tuning: signals, segmentation, gates, LOGGING_REQUIREMENTS
python tests/test_tune_sim.py          # the AC_PID replica, plant and AutoTune simulator
python tests/test_tune_step.py         # tier C: step response, oscillation ceiling, step rules
python tests/test_tune_atun.py         # tier A: ATUN reconstruction and branch backoffs
python tests/test_tune_ident.py        # tier B: plant identification, margins, virtual AutoTune
python tests/test_tune_fuse.py         # tier D, confidence, fusion, the tune Section
python tests/test_tune_docs.py         # SKILLS.md/CLAUDE.md/README.md/thresholds.md agree with the tune code
python tests/test_docs_api.py          # the §1 Python snippet runs, and its accessors return what it says
```

The regression tests pin the analysis against hand-verified figures; if a refactor changes
them, the refactor is wrong until proven otherwise. pytest is optional — the suite carries
a small shim and runs standalone.
