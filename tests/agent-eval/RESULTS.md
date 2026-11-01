# Cross-agent acceptance results

One row per agent run (see README.md). `pass` / `FAIL: <criterion>` from `grade.py`; a `?`
criterion is resolved by reading the answer and noted as `pass*` or `FAIL*`. Transcripts are
in `runs/<agent>/` (workspace paths replaced by `<ws>`).

| date | agent | version | skill version | 1-triage | 2-notch | 3-motors | 4-pid | 5-px4 | notes |
|---|---|---|---|---|---|---|---|---|---|
| 2026-10-31 | Claude Code (`claude -p`, model `claude-opus-5-5`) | 2.1.316 | 2.1.0.dev0 (`main` at 0326a05) | pass, pass* | pass, pass* | pass, pass* | pass | pass, pass* | see below |

## 2026-10-31, Claude Code

Windows, one fresh session and one fresh workspace per case, tools `Bash Read Glob Grep
Skill`. The skill loaded (`Skill` tool call) in cases 1–4 and **not** in case 5, which is the
triggering the plan asks for. Each case took 23–59 s. Every answer ran `alog info` first
(through the skill's `scripts/alog.sh` or `alog.ps1`), stated its window and figures, and
deleted the `.dfcache` it created.

- **1-triage**: intact, one flight of 301 s, and the FAILs with numbers (current-sensor
  zero 12.3 A, GPS1 15.5 % without a fix, pitch trim 58 µs); SKIPs reported as not logged.
- **2-notch**: disabled; followed recipe Skill 4 to `--window rpm` and quoted that
  window's starting point (`MODE 3`, `FREQ 60`, `BW 30`, `HMNCS 3`, `OPTS 2`) and the
  batch-logging flight. Its "a correct notch reads about 0.014" comes from `recipes.md`.
- **3-motors**: pitch +58.7 µs on `--window rpm`, drive-normalised spread 2.2 % ("no motor
  or prop is dragging"), and "a nose-heavy CG or a steady wind on the nose, and this log
  can't tell those apart", with the four-heading hover as the test.
- **4-pid**: `NO_PID_MESSAGES`, `LOG_BITMASK 180222 → 180223`, the tuning profile, no
  gains; flagged the pitch trim and stale `MOT_THST_HOVER` as things to fix first.
- **5-px4**: did not load the skill or run `alog`; read the 16-byte file as a ULog header
  with no data, and noted that the `.bin` in the folder is an ArduPilot log the tool reads.

The first grading failed six criteria on these five answers. All six were the harness, not
the agent, and were fixed before this row was recorded: "integrity" demanded the word
(the answer said "intact"); the window method demanded the literal `EV
NOT_LANDED->LAND_COMPLETE` (the answer named both events in words, which still meets
ground rule 6's intent); `info`-first could not see `S=.../alog.sh; bash $S info`; the
notch `FREQ` expected the default window's 55 where recipe Skill 4's `--window rpm` gives
60; and load-vs-drag missed "RPM ÷ duty". `tests/test_agent_eval.py` now pins each of those
phrasings, and that a non-answer still fails them.

Still needed for S5: two agents from other vendors.
