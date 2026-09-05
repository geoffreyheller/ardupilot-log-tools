# AGENTS.md

This repository is built for you. Its purpose is to let an AI agent analyse an ArduCopter
dataflash log efficiently and accurately, and every output is shaped for that.

Read, in this order:

1. **`RULES.md`** — the contract (fail loudly, state the number, agents first). Short.
2. **`CLAUDE.md`** — the operating guide: which command to run first, how to pick the
   window, what to check every time, the pitfalls that have already produced wrong answers.
3. **`reference/`** as needed — `integrity-codes.md` when a report shows a diagnostic,
   `json-output.md` for the `--json` shape, `thresholds.md` for where a number came from.

Before you touch a log:

- **This repo is vehicle-agnostic.** It holds no hardware baselines and no tune state. Read
  whatever notes exist for the aircraft you are analysing, and never carry parameters or
  conclusions between airframes.
- **Write reports into the aircraft's own project folder**, not here. A template is in
  `templates/log-analysis-template.md`.

Quick start (any OS):

```bash
python alog.py info "<path to .bin>"          # identity, integrity, coverage
python alog.py all  "<path to .bin>" --json   # the standard battery, machine-readable
python alog.py schema                         # the JSON contract
```
