# TODO: Code-quality rules for ardupilot-log-tools

Plan to adopt the Scientific Python Development Guide (checked by `sp-repo-review`)
plus a small set of project-specific engineering rules. Nothing here has been applied yet.

Repo: https://github.com/geoffreyheller/ardupilot-log-tools
Guide: https://learn.scientific-python.org/development/
Checker: https://github.com/scientific-python/cookie (`sp-repo-review`)

## Ground rules for this whole plan

- Every phase is its own branch and PR. Never mix tooling or formatting changes with logic changes.
- Run the full test suite (all six files) before and after every phase. A pinned figure that moves means the change is wrong until proven otherwise (RULES.md §5).
- `alog all <log> --json` output on the committed fixture must be byte-identical before and after every phase except where a phase explicitly intends a change.
- Keep Python 3.9 support. All tool configs target `py39`.
- Touch only what the phase is about. No drive-by refactors.

---

## Phase 0: Baseline (no code changes)

- [ ] Record the current state on `main`:
  - [ ] Run all six test files and save the results.
  - [ ] Save `python alog.py all tests/fixtures/largeprop-quad.bin --json` output as the reference snapshot (outside the repo, or under a gitignored path).
- [ ] Run `pipx run "sp-repo-review[cli]" .` (or `uvx "sp-repo-review[cli]" .`) and save the report.
- [ ] Run `ruff check --target-version py39 .` and `mypy dflog` with default settings and save the counts only, as a baseline. Do not fix anything yet.

**Done when:** baseline test results, JSON snapshot and three tool reports are saved.

---

## Phase 1: Fix test-run drift (highest priority)

CI runs only `test_toolkit.py`, `test_parser_integrity.py` and `test_cli.py`. `RULES.md` §5 lists six files; `CLAUDE.md` §11 says "all three". `test_flights.py`, `test_checks.py` and `test_largeprop.py` never run in CI.

- [ ] Add a single entry point that runs all six files (e.g. `python -m pytest`, which already uses `testpaths = ["tests"]`, or a small `tests/run_all.py` that works without pytest via the existing shim).
- [ ] Update `.github/workflows/ci.yml` to use that entry point on all OS/Python combinations.
- [ ] Fix any test that fails once it actually runs in CI (separate commit per fix, with an explanation).
- [ ] Update `CLAUDE.md` §11 and `RULES.md` §5 so both name the same single command.

**Done when:** CI runs all six test files on Windows, Linux and macOS, Python 3.9 and 3.12, and both docs agree.

---

## Phase 2: Repository hygiene

- [ ] Remove the committed `.coverage` file (`git rm --cached .coverage`) and add `.coverage` / `.coverage.*` / `htmlcov/` to `.gitignore`.
- [ ] Confirm `*.dfcache` is ignored.
- [ ] Decide whether `requirements.txt` stays or CI installs from `pyproject.toml` (`pip install -e ".[test]"`), and document the choice.

**Done when:** `git status` is clean after running tests with coverage.

---

## Phase 3: Triage the sp-repo-review report

- [ ] Go through every failed check from Phase 0 and mark each as **adopt**, **later**, or **ignore**.
- [ ] Record ignored checks, each with a one-line reason, in `pyproject.toml` under `[tool.repo-review]`.
- [ ] Likely adopt: pytest configuration in `pyproject.toml` (`minversion`, `addopts` with `-ra --strict-config --strict-markers`, `xfail_strict`, `filterwarnings = ["error"]`, `log_level`), ruff and mypy configuration, pre-commit.
- [ ] Likely review carefully: packaging layout (`src/` layout would change import paths used in docs and by `alog.py`; probably **ignore** or **later**), docs/ReadTheDocs checks (probably **ignore**), nox.
- [ ] `filterwarnings = ["error"]` may surface numpy/pandas/scipy deprecation warnings; fix or explicitly allow each one rather than dropping the setting.

**Done when:** `sp-repo-review` shows only adopted-and-passing checks plus documented ignores.

---

## Phase 4: Ruff (lint and format)

- [ ] Add `[tool.ruff]` to `pyproject.toml` with `target-version = "py39"`.
- [ ] Start lint with a small rule set and grow it one family per commit, following the guide's recommended list. Suggested order:
  1. `E`, `F`, `W` (basics), `I` (import sorting)
  2. `B` (bugbear), `UP` (pyupgrade, py39-safe)
  3. `NPY` (numpy), `PD` (pandas); review `PD` findings individually, some are style opinions
  4. `RET`, `SIM`, `C4`, `PIE`, `RUF`
  5. `PL` (pylint) last, with complexity limits relaxed at first
  6. `T20` (no `print`), with per-file ignores for `dflog/cli.py` and `dflog/report.py` if they print by design
- [ ] For each family: fix findings or add a justified `per-file-ignores` entry. No blanket `noqa` without a rule code.
- [ ] Formatting: run `ruff format` as **one isolated commit** with no other changes, confirm tests and the JSON snapshot are unchanged, then add that commit's hash to `.git-blame-ignore-revs`.
- [ ] Add `ruff check` and `ruff format --check` to CI.
- [ ] Optional: add pre-commit with ruff hooks (as the guide recommends).

**Done when:** CI fails on any lint or formatting violation, and the JSON snapshot is unchanged.

---

## Phase 5: Gradual typing with mypy

Current state: no return annotations on any of the 49 functions in `dflog/analysis.py`, and no `typing` imports.

- [ ] Add `[tool.mypy]` to `pyproject.toml` with `python_version = "3.9"`, `files = ["dflog"]`, `warn_unused_configs = true`, and the guide's recommended error codes. Start permissive (not `strict`).
- [ ] Use `from __future__ import annotations` in each module that gets annotations (already present in `checks.py`) so `X | None` syntax works on 3.9.
- [ ] Annotate in this order, one module per PR:
  1. `checks.py`: `Result`, status constants, `T`
  2. `report.py` / the `Section` API
  3. `parser.py` public API: `Log`, `log.df`, `log.instances`, `log.field`, `log.column`, diagnostics
  4. `flight.py`: `airborne_window`, `flights`, `hover_chunks`, the window object
  5. `frames.py`, `spectral.py`, `stats.py`
  6. `analysis.py` check signatures (`check_*(log, window) -> Section`)
  7. `cli.py`
- [ ] After each module is annotated, turn on stricter settings for it via a per-module `[[tool.mypy.overrides]]` block.
- [ ] Add mypy to CI (one Python version is enough).
- [ ] No annotation-only PR may change runtime behaviour; tests and JSON snapshot must be unchanged.

**Done when:** every public function is annotated, mypy passes in CI, and strict settings are on for at least `checks.py`, `parser.py` and `flight.py`.

---

## Phase 6: Audit inline numeric literals

A rough scan found about 33 comparisons against float literals in `dflog/analysis.py` that don't reference `T`. Some are likely legitimate (window definitions are allowed outside `T` per CLAUDE.md §3); others may break RULES.md §2 ("No number in a check is hard-coded").

- [ ] List every literal comparison with file and line.
- [ ] Classify each as **threshold** (judges the aircraft), **window/definition** (defines what is measured), or **numerical constant** (unit conversion, epsilon).
- [ ] Move every threshold into `checks.py::T` with a `source`, and add it to `reference/thresholds.md`.
- [ ] Give window/definition values a named module-level constant or keyword argument with a comment.
- [ ] Consider a test that fails if `analysis.py` gains a new unnamed float comparison (or a ruff `PLR2004` rule scoped to `analysis.py`).

**Done when:** every remaining literal is classified, thresholds live in `T` with sources, and pinned figures are unchanged.

---

## Phase 7: Function size

Longest functions in `analysis.py`: `check_motors` (~190 lines), `check_batch_fft` (~170), `check_flight` (~115), plus several over 70.

- [ ] Set a soft limit for new code (e.g. ~60 lines per function), enforced by ruff `PLR0915` / `C901` with a threshold, starting in warn-only mode or with existing functions listed as exceptions.
- [ ] Split the largest checks only when there is a reason to touch them (e.g. while fixing issues #5–#10), never as a standalone sweep.
- [ ] Each split: extract helpers, keep `check_*` signatures unchanged, confirm pinned figures and JSON snapshot are identical.

**Done when:** no new function exceeds the limit, and the exceptions list only shrinks.

---

## Phase 8: Add engineering principles to RULES.md §5

Adapted from the Karpathy-inspired guidelines (https://github.com/forrestchang/andrej-karpathy-skills, MIT); keep them short and in the project's own voice.

- [ ] Add to `RULES.md` §5:
  - [ ] Touch only what the change is about; no drive-by refactors or reformatting in a logic change.
  - [ ] State assumptions explicitly in the PR or report; if a requirement is ambiguous, ask before implementing.
  - [ ] Minimum code that solves the problem; no abstractions for a single use.
  - [ ] Formatting, typing and refactoring changes go in their own commits and must leave tests and JSON output unchanged.
  - [ ] Code must pass `ruff check`, `ruff format --check` and `mypy` before merge.
- [ ] Add an attribution line to `NOTICE.md` if any wording is taken directly from the source.
- [ ] Make sure `AGENTS.md` points at the updated rules.

**Done when:** RULES.md §5 contains the new lines, and CLAUDE.md §11 agrees with it.

---

## Phase 9: Final check

- [ ] Re-run `sp-repo-review` and compare with the Phase 0 report.
- [ ] Confirm CI runs: all six test files, ruff lint, ruff format check, mypy.
- [ ] Confirm the JSON snapshot on the committed fixture matches Phase 0, or that every difference is explained in a PR.
- [ ] Update `README.md` with a short "Development" section: install with `pip install -e ".[test]"`, the single test command, and the lint/type commands.
- [ ] Close or update this TODO.

---

# TODO: Package as an installable, agent-neutral skill

Make the analysis usable by any coding agent (Claude Code, GitHub Copilot, Codex, Cursor,
Gemini CLI, …) without cloning the repo. Revised 2026-10-30 from the 2026-10-28 draft after
reading the code; nothing here has been started. Third-party marketplaces were considered
and declined for now (Agensi, 2026-10-28); listing anywhere is optional once the skill
installs from this repo.

## Shape: three layers, one source

```
 ┌──────────────────────────────────────────────────────────────────────┐
 │ 3. Per-agent adapters (optional, thin, outside the skill folder)     │
 │    .claude-plugin/ manifest · Copilot/Codex install notes · MCP config│
 ├──────────────────────────────────────────────────────────────────────┤
 │ 2. skills/ardupilot-log-analysis/   SKILL.md + references/           │
 │    open Agent Skills format; plain shell commands; no vendor names   │
 ├──────────────────────────────────────────────────────────────────────┤
 │ 1. The engine: `ardupilot-log-tools` wheel → `alog` on PATH          │
 │    pure-Python (py3-none-any), docs shipped inside, `alog docs`      │
 └──────────────────────────────────────────────────────────────────────┘
```

- **Layer 1 is the only thing that has to work everywhere.** Every agent that can run a
  shell can run `alog`. The skill is guidance on top; the adapters are convenience.
- **An MCP server (S6) is the fallback for agents without a shell** — chat clients and IDE
  chat panes. It wraps the same CLI and returns the same JSON; it is not a second engine.

## Ground rules

- **No agent-specific content in the skill itself.** `SKILL.md` uses only the standard
  frontmatter (`name`, `description`, optionally `license`, `compatibility`, `metadata`).
  No vendor-only fields, tool names, slash commands or agent product names in the body.
  Commands are plain shell and work the same in bash and PowerShell.
- **Per-agent wiring lives outside the skill folder**, so adding or dropping an agent never
  edits the skill.
- **One source of truth.** The repo docs stay canonical. The skill's `references/` and the
  docs inside the wheel are generated from them, and CI fails when they drift.
- **The engine and the guidance ship at the same version.** A skill that describes flags
  the installed `alog` does not have is worse than no skill.
- **Free and MIT**, same as the repo.

## What the code review of 2026-10-30 found

Facts that change the draft plan:

1. **`alog --version` already exists** (`dflog/cli.py:976`, `__version__ = "2.0.0"`). No
   new flag is needed; the skill only has to check it.
2. **Output tells the reader to open repo files that an installed user does not have.**
   `analysis.py:243`, `cli.py:939`, `checks.py` sources and parser docstrings say "see
   `reference/integrity-codes.md`" etc. After `pip install` there is no `reference/`
   directory. This is the main gap between "runs" and "usable by an agent".
3. **The wheel contains only `dflog/`** (`[tool.setuptools] packages = ["dflog"]`): no
   `reference/`, `RULES.md`, `SKILLS.md` or template.
4. **No git tags.** `pyproject.toml` says 2.0.0 but nothing pins a release.
5. **`CLAUDE.md` is two documents.** §1–§11 are an agent-neutral operating guide that
   happens to carry one vendor's file name; §12 is repo development. `AGENTS.md` (the
   cross-agent convention) already exists and points at it. `tests/test_tune_docs.py`
   and `tests/test_docs_api.py` read `CLAUDE.md` directly, so any split must move those
   tests with it.
6. **Docs say `python alog.py …` 80+ times** (`SKILLS.md` 43, `README.md` 24, `CLAUDE.md`
   12). In the skill that form is wrong: there is no `alog.py` outside a clone.
7. **`.dfcache` is written beside the log.** An agent working in a read-only or synced
   folder (OneDrive, a mounted SD card) either fails to write it or litters it. `--no-cache`
   exists; there is no way to put the cache elsewhere.
8. **The code-quality plan above is a prerequisite for S3.** CI runs 3 of 14 test files;
   doc-sync tests that CI never runs prove nothing. Do code-quality Phase 1 first.

## Phase S0: Decisions (decided 2026-10-30)

- [x] **PyPI or GitHub-only → PyPI.** `uvx ardupilot-log-tools@2.1.0 …` pins exactly and
  is short enough to put in a skill; `git+https://…@v2.1.0` works but is slower and needs
  git on the machine. The `ardupilot-log-tools` name was free on 2026-10-30 (PyPI JSON API
  404). Publish with a trusted-publishing GitHub Action, not a token.
- [x] **Where the operating guide lives → a neutral `GUIDE.md`.** `CLAUDE.md` §1–§11 move
  there; `CLAUDE.md` becomes a two-line pointer plus §12. `AGENTS.md` points at
  `GUIDE.md`. Move the `test_tune_docs.py` / `test_docs_api.py` targets in the same PR.
- [x] **MCP server (S6) in the first release → no.** Ship S1–S5, then add S6 only if a
  target agent cannot run a shell.

## Phase S1: Make the engine self-contained

- [ ] Ship the docs as package data: `reference/*.md`, `RULES.md`, `SKILLS.md`,
  `GUIDE.md`, `templates/log-analysis-template.md` → `dflog/docs/` in the wheel, copied
  at build time (setuptools `package-data`), never committed twice.
- [ ] Add `alog docs [TOPIC]`: no argument lists topics; `alog docs integrity-codes` prints
  one. Same exit-code contract. This version-locks the guidance to the code for any
  agent, with or without the skill.
- [ ] Change every "see `reference/X.md`" string in output to "see `alog docs X`
  (reference/X.md)". Add a test that every topic named in output exists in `alog docs`.
- [ ] Add `ALOG_CACHE_DIR` (env var) / `--cache-dir`: when set, `.dfcache` goes there,
  keyed by the log's path + size + mtime. Default unchanged.
- [ ] Docs: the installed form `alog …` is primary everywhere; `python alog.py …` is
  mentioned once, in the README's "from a clone" section. A test greps the shipped docs
  for `python alog.py` and fails.
- [ ] Confirm, on Windows, Linux and macOS, Python 3.9 and 3.12, from a directory that
  is not the repo: `uvx ardupilot-log-tools alog info <fixture>`, `pipx run …`, and
  `pip install … && alog …`. Add this as a CI job that builds the wheel and installs it
  into a fresh venv (catches missing package data, which editable installs hide).
- [ ] Attach the wheel to each GitHub release, for sandboxes with no PyPI access but with
  numpy/pandas/scipy preinstalled (`pip install --no-deps <wheel>`).

**Done when:** a clean machine with only `uv` (or only `pipx`) runs `alog all <log> --json`
and `alog docs pitfalls` from any directory, and the installed-wheel CI job passes.

## Phase S2: Write the skill folder

```
skills/ardupilot-log-analysis/
  SKILL.md               hand-written, < 300 lines
  references/            GENERATED by tools/build_skill.py — do not edit
    rules.md  guide.md  recipes.md  pitfalls.md  integrity-codes.md
    json-output.md  thresholds.md  messages.md  report-template.md
  scripts/
    alog.sh  alog.ps1    run the pinned version: installed alog if its --version
                         matches, else uvx, else pipx; print the install line if neither
```

- [ ] `SKILL.md` frontmatter: `name: ardupilot-log-analysis`; a `description` that
  triggers on ArduPilot / ArduCopter / PX4-adjacent wording users actually type —
  "dataflash", ".bin log", "flight log", vibration, clipping, harmonic notch, motor
  balance, CG / trim, GPS glitch, EKF innovations, current sensor, PID tuning — and says
  what it does not cover (`.tlog` telemetry beyond the text-log path, PX4 `.ulg`);
  `license: MIT`; `compatibility: requires Python 3.9+ and uv or pipx`; `metadata.version`
  equal to the package version.
- [ ] `SKILL.md` body, in this order: install/run line; "run `alog info` first and read
  the integrity block"; the ten ground rules (guide §2) verbatim; the window table (§3)
  with the multi-flight rule; a question → recipe index pointing into
  `references/recipes.md`; the report shape; "delete `.dfcache` or set
  `ALOG_CACHE_DIR`". Everything else is behind a reference link.
- [ ] `tools/build_skill.py`: copies the canonical files into `references/`, rewrites
  `python alog.py` → `alog`, rewrites cross-links (`CLAUDE.md §3` → `guide.md §3`,
  `reference/X.md` → `X.md`), strips repo-development sections (guide §12, test
  commands), and stamps a "generated from <commit>, do not edit" header.
- [ ] Leave out: `TODO.md`, `docs/*-plan.md`, `reference/pid-tuning-sources.md` and
  `reference/existing-tools.md` (provenance, not operating guidance — reachable through
  `alog docs`), `tools/`, the tests.
- [ ] Validate with the spec's reference validator (`skills-ref validate`) and a test that
  greps the folder for vendor names, slash commands and tool names.

**Done when:** the folder validates, contains no agent-specific wording, and
`build_skill.py` is idempotent (running it twice changes nothing).

## Phase S3: Keep the skill, the wheel and the docs in step

- [ ] `tests/test_skill.py`:
  - [ ] `build_skill.py --check` reports no diff (generated files are current).
  - [ ] every `alog <cmd> --flag` quoted in `SKILL.md` and `references/` parses with the
    real argparse tree (extend the approach in `test_tune_docs.py`).
  - [ ] exit codes and JSON top-level keys quoted in the skill match `alog schema`.
  - [ ] `metadata.version` in `SKILL.md` == `dflog.__version__`, and the run scripts pin it.
- [ ] Run in CI (after code-quality Phase 1, so CI actually runs it).

**Done when:** changing a canonical doc, a CLI flag or the version without regenerating
the skill fails CI.

## Phase S4: Per-agent installation

Install paths and manifest formats change often; verify each one when writing it and put
the date beside it in the README table.

- [ ] README "Use as an agent skill": a table of agent → skills directory (user-level and
  project-level) → install command (copy, symlink, or the agent's own install-from-repo
  command). Start with the agents that read the open SKILL.md format: Claude Code,
  GitHub Copilot (CLI and VS Code agent mode), OpenAI Codex, Cursor, Gemini CLI.
- [ ] A one-shot installer, `alog install-skill [--dir PATH]`, that copies the skill
  folder shipped inside the wheel to a given directory. No agent knowledge in it: the user
  (or README table) supplies the path. This keeps installation to one command on every OS
  without a curl-pipe-shell script.
- [ ] Claude Code: `.claude-plugin/plugin.json` + `marketplace.json` at the repo root
  pointing at `skills/ardupilot-log-analysis/`, so `/plugin marketplace add
  geoffreyheller/ardupilot-log-tools` works.
- [ ] Other agents with an install-from-repo mechanism: one adapter file each, outside the
  skill folder, none required by the others.
- [ ] Agents without SKILL.md support: `AGENTS.md` already serves them when the repo is the
  workspace; for other workspaces, document pasting the `SKILL.md` body into that agent's
  instructions file.

**Done when:** the README table is dated and each listed path has been tried.

## Phase S5: Cross-agent acceptance test, then release

- [ ] Write `tests/agent-eval/` (not run in CI — it costs money and needs accounts): five
  prompts against `tests/fixtures/largeprop-quad.bin`, each with pass criteria checked
  against the transcript:
  1. "Is this log OK?" → runs `alog info` first; reports integrity codes; quotes the window
     and method string.
  2. "Check the harmonic notch" → reads `FCNS.CF` against ESC RPM, not `FTN1.PkAvg`.
  3. "Are my motors balanced?" → quotes the `SERVOn_FUNCTION` map and the trim
     decomposition, not raw spread alone.
  4. "Recommend PID gains" → reports the `PID_RATE_TOO_LOW` refusal and the logging
     table; does not invent gains.
  5. A question the skill should not trigger on (a PX4 `.ulg`) → skill not used.
- [ ] Run it in each agent's non-interactive mode where one exists, by hand otherwise, on
  at least three agents from different vendors. Record agent, version, date, pass/fail
  per prompt in `tests/agent-eval/RESULTS.md`.
- [ ] Tune the `description` until prompts 1–4 trigger and 5 does not, on all three.
- [ ] Release: bump to 2.1.0 (new `docs`, `install-skill`, `--cache-dir`), CHANGELOG
  entry, tag `v2.1.0`, publish to PyPI from the tag, attach the wheel and a zipped skill
  folder to the GitHub release.
- [ ] Optional: list in public skill directories. Free listings only, no exclusivity.

**Done when:** three agents from different vendors install the skill from the release and
pass prompts 1–5, and the results are recorded.

## Phase S6 (later, only if needed): MCP server for agents without a shell

- [ ] `alog mcp` (optional extra `ardupilot-log-tools[mcp]`): a stdio MCP server exposing
  `info`, `all`, one tool per check, `compare`, `tune`, `params` and `docs`, each taking
  a log path and the window/flight options and returning the CLI's JSON unchanged.
- [ ] Expose the docs as MCP resources and the ground rules as the server's instructions,
  generated by the same `build_skill.py`.
- [ ] Document the config snippet for the clients that need it (desktop chat apps, IDE
  chat panes); the server itself stays agent-neutral.

**Done when:** a client with no shell access produces the same `info` + `all` analysis of
the fixture as the CLI.

## Order and dependencies

```
code-quality Phase 1 (CI runs every test)
        │
S0 decisions ─► S1 engine ─► S2 skill ─► S3 sync tests ─► S4 install ─► S5 eval + release
                                                                              │
                                                                         S6 MCP (later)
```

Each phase is its own branch and PR, under the same ground rules as the code-quality plan:
tests green before and after, the fixture's `alog all --json` byte-identical except where a
phase intends a change (S1's "see `alog docs X`" strings are the only intended one).
