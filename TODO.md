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

Make the analysis usable by any coding agent without cloning the repo: one skill folder in
the open SKILL.md format (https://agentskills.io) that installs and runs `alog`, plus the
guidance from `RULES.md`, `CLAUDE.md` and `SKILLS.md`. Nothing here has been started.
Third-party marketplaces were considered and declined for now (Agensi, 2026-10-28); listing
anywhere is optional once the skill installs from this repo.

## Ground rules

- **No agent-specific content in the skill itself.** `SKILL.md` uses only the standard
  frontmatter (`name`, `description`, optionally `license`, `compatibility`, `metadata`).
  No vendor-only fields, tool names, slash commands or agent product names in the body.
  Commands are plain shell.
- **Per-agent wiring lives outside the skill folder** (a plugin/marketplace manifest, an
  install path in the README), so adding or dropping an agent never edits the skill.
- **One source of truth.** The repo docs stay canonical; the skill's reference files are
  generated from them or checked against them, never hand-maintained copies.
- **Free and MIT**, same as the repo.

## Phase S1: Make `alog` runnable without a clone

- [ ] Confirm `uvx --from git+https://github.com/geoffreyheller/ardupilot-log-tools alog info <log>` works on Windows, Linux and macOS, Python 3.9 and 3.12.
- [ ] Document the `pipx install git+…` and plain `pip install git+…` equivalents for machines without `uv`.
- [ ] Decide whether to publish to PyPI (shorter command, pinned versions) or stay GitHub-only. Check the `ardupilot-log-tools` name is free first.
- [ ] Make sure every path the docs give (`python alog.py …`) has an installed-command equivalent (`alog …`), and that no command depends on the working directory being the repo.

**Done when:** a clean machine with only `uv` (or only `pipx`) can run `alog all <log> --json` from any directory.

## Phase S2: Write the skill folder

- [ ] Create `skills/ardupilot-log-analysis/` with:
  - [ ] `SKILL.md`: a `description` that triggers on ArduPilot / ArduCopter dataflash `.bin` logs, vibration, harmonic notch, motor balance, CG/trim, GPS, current sensor and PID tuning. The body covers the install/run command, "run `info` first", the ground rules (`CLAUDE.md` §2), the window choice (§3) and an index of the reference files. Keep it short; agents load the rest on demand.
  - [ ] `references/`: `RULES.md`, the recipes from `SKILLS.md`, `pitfalls.md`, `integrity-codes.md`, `json-output.md`, `thresholds.md`, and the report template.
- [ ] Leave out repo-development material: `CLAUDE.md` §12 "Extending", test commands, this TODO.
- [ ] Pin the tool version the skill expects (a git tag or PyPI version) and have the skill check `alog --version`, or add that flag if it does not exist.
- [ ] Validate the folder against the spec (`skills-ref validate`, from the agentskills reference library).

**Done when:** the folder validates and contains no agent-specific wording.

## Phase S3: Keep the skill and the docs in step

- [ ] Add a script (e.g. `tools/build_skill.py`) that regenerates `references/` from the canonical files, or a test that fails when they differ.
- [ ] Extend `tests/test_tune_docs.py` (or a new `tests/test_skill_docs.py`) so command names, flags and exit codes quoted in `SKILL.md` match the CLI.
- [ ] Run it in CI.

**Done when:** changing a canonical doc without regenerating the skill fails CI.

## Phase S4: Installation per agent

- [ ] README "Use as an agent skill" section: copy or symlink `skills/ardupilot-log-analysis/` into the agent's skills directory, with a table of directories for the agents that support SKILL.md (verify each one at the time of writing; they change).
- [ ] Optional thin adapters, each in its own file outside the skill folder and none required by the others: a Claude Code plugin/marketplace manifest, and the equivalent for any other agent that has an install-from-repo mechanism.
- [ ] Test on at least three different agents, each against the committed fixture (`tests/fixtures/largeprop-quad.bin`): the skill triggers on a natural question ("check the notch on this log"), runs `info` first and quotes the window and method.

**Done when:** three agents from different vendors install the skill from this repo and produce a correct `info` + `all` analysis of the fixture.

## Phase S5: Release

- [ ] Tag a release; the skill's pinned version points at it.
- [ ] Add a CHANGELOG entry and the README section.
- [ ] Optional: list in public skill directories. Free listings only, no exclusivity.
