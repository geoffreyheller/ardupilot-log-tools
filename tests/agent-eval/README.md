# Cross-agent acceptance test

Does the skill make an agent analyse a log correctly, on agents from different vendors?
Five prompts against the committed fixture (`tests/fixtures/largeprop-quad.bin`), each
with criteria a transcript must meet. This is **not** part of the test suite: it costs
money, needs accounts, and its result depends on the agent. What the suite does check
(`tests/test_agent_eval.py`) is that the criteria still describe what `alog` says, and
that the grader works.

| case | prompt (in `cases.py`) | what a correct answer has to get right |
|---|---|---|
| `1-triage` | "Is it OK?" | `alog info` first; integrity; the window method; one flight; the FAILs with their numbers |
| `2-notch` | "Check the harmonic notch" | the notch is **disabled**; the report's starting point (`MODE 3`, `FREQ 55`); the batch-logging flight that would verify it; never "working" |
| `3-motors` | "Are my motors balanced?" | the servo→motor map; pitch trim 58 µs; drive-normalised spread (load, not drag); a breeze cannot be ruled out, so the CG offset is not proven |
| `4-pid` | "Recommend PID gains" | the `NO_PID_MESSAGES` refusal; `LOG_BITMASK 180222 → 180223`; no invented gains |
| `5-px4` | "Check the vibration in my PX4 flight log" | out of scope: `alog` is not run on the `.ulg` |

## Run it

1. Make a clean workspace (a new, empty directory):

   ```bash
   python tests/agent-eval/workspace.py /path/to/ws
   ```

   It holds the fixture, a PX4 ULog header, and the skill at project level in
   `.agents/skills/` and `.claude/skills/` (installed by `alog install-skill` from this
   checkout), so every agent in the README table finds it without a user-level install.
   Nothing else from the repo is copied: no `AGENTS.md`, no `CLAUDE.md`. Your user-level
   instructions and skills still load, so use a profile without other log-analysis skills.

2. Start each case **in a fresh session**, from the workspace, with the prompt from
   `cases.py`, letting the agent run shell commands there. Save each one as
   `runs/<agent>/<case-id>.md` with a `## Commands` section (every shell command it ran,
   one per line) and a `## Answer` section (its final answer).

   Claude Code (checked 2026-10-31, v2.1.316):

   ```bash
   claude -p "<prompt>" --output-format stream-json --verbose --allowedTools Bash Read Glob Grep > run.jsonl
   python /path/to/repo/tests/agent-eval/from_claude_stream.py run.jsonl > runs/claude-code/1-triage.md
   ```

   Other agents: use their non-interactive mode where they have one, and copy the
   commands and the answer from their transcript into the same two sections. Check each
   agent's current flags when you run it; this harness has only been driven through
   Claude Code.

3. Grade:

   ```bash
   python tests/agent-eval/grade.py --dir runs/claude-code
   ```

   `pass` / `FAIL` are automatic. `?` lines are for a person: read the answer and decide.
   Exit code 0 means no automatic criterion failed.

4. Record each run in `RESULTS.md`: agent, version, date, and per case `pass`, `FAIL`
   (which criterion) or `?` resolved by hand. If a case fails because the skill did not
   trigger, tune the `description` in `skills/ardupilot-log-analysis/SKILL.md` and run
   all the agents again; the plan's bar is cases 1–4 triggering and passing, and case 5
   not misfiring, on three agents from different vendors.
