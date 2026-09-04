# AGENTS.md

Read **`CLAUDE.md`** in this repository first. It is the full instruction file for
analysing ArduPilot dataflash logs with these tools: how to pick a flight window, what to
check every time, where the thresholds come from, and the pitfalls that have already
produced wrong answers.

Two things before you touch anything:

1. **This repo is vehicle-agnostic.** It holds no hardware baselines and no tune state.
   Read whatever project notes exist for the aircraft you are analysing, and never carry
   parameters or conclusions between airframes.
2. **Write reports into the aircraft's own project folder**, not here. A template is in
   `templates/log-analysis-template.md`.

Quick start: `./alog.py all "<path to .bin>"`
