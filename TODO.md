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

*(2026-10-31: by now there were 14 test files and CI still ran 3.)*

*(2026-10-31, later: the GitHub Actions workflow was removed. Nothing runs in the cloud. "CI" throughout this file now means `tools/ci_local.py`, run locally before merging, which runs `tests/run_all.py` and `alog.py --help` under Python 3.9 and 3.12 on the machine it runs on. Items below that say "add X to CI" mean adding a step to its `STEPS`.)*

- [x] Add a single entry point that runs every file: `tests/run_all.py`, which globs `tests/test_*.py`, runs each in its own process through the shim (no pytest needed), and fails a file that has no `_shim.run` main block rather than counting it as passed.
- [x] Update `.github/workflows/ci.yml` to use that entry point on all OS/Python combinations. *(Superseded 2026-10-31: the workflow was removed; `tools/ci_local.py` runs the same entry point locally.)*
- [ ] Fix any test that fails once it actually runs in CI (separate commit per fix, with an explanation). All 14 pass locally on Windows, Python 3.12 and 3.9.25 (numpy 2.0.2, pandas 2.3.3, scipy 1.13.1); Linux and macOS are untested: there is no cloud CI, so they need a local run of `tools/ci_local.py` on each.
- [x] Update `CLAUDE.md` §12, `RULES.md` §5 and `README.md` (Tests, Contributing) so all name the same single command.

**Done when:** `tools/ci_local.py` runs every test file under Python 3.9 and 3.12 and passes on Windows (and on Linux and macOS when run there), and the docs agree.

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

*(Worked 2026-10-31 on branch `s1-self-contained-engine`.)*

- [x] Ship the docs as package data: `reference/*.md`, `RULES.md`, `SKILLS.md`,
  `GUIDE.md`, `templates/log-analysis-template.md` → `dflog/docs/` in the wheel, copied
  at build time, never committed twice. Done with a `build_py` hook in `setup.py` rather
  than `package-data` (which cannot reach files outside the package); the file list is
  `dflog/docs.py::sources()`, `MANIFEST.in` puts it in the sdist, and a missing file fails
  the build.
- [x] Add `alog docs [TOPIC]`: no argument lists topics; `alog docs integrity-codes` prints
  one; an unknown topic is exit 3 naming them. Reads the wheel's copy when installed, the
  repo's files in a clone.
- [x] Change every "see `reference/X.md`" string in output to "see `alog docs X`
  (reference/X.md)" (27 strings, mostly threshold and tune-constant `source`s). A test
  checks every topic named in output exists and no bare path is left.
- [x] Add `ALOG_CACHE_DIR` (env var) / `--cache-dir`: when set, `.dfcache` goes there.
  Keyed by a hash of the log's absolute path only: size and mtime are already checked
  inside the cache, and putting them in the name would leave a stale file per edit.
- [x] Docs: the installed form `alog …` is primary everywhere (83 replacements).
  Deviation: besides the README's "From a clone" section, `GUIDE.md` §1 says once that
  `python alog.py` is the same command in a clone, because `CLAUDE.md` imports `GUIDE.md`
  and agents working in this repo need the fallback. The test allows exactly that one.
- [~] Confirm from a directory that is not the repo: done for `pip install <wheel>` on
  Windows, Python 3.9 and 3.12, by `tools/ci_local.py` (cloud CI no longer exists). It
  builds the sdist and the wheel from a copy of the files a fresh clone would have (a
  stale ignored `*.egg-info/SOURCES.txt` in the working tree hid a file missing from
  `MANIFEST.in`), installs the wheel into a throwaway environment and runs `--version`,
  `docs` (must be the installed copy), `info` and `all --json` on the fixture. Also done,
  once the branch was pushed: `uvx --from git+https://github.com/...@s1-self-contained-engine
  alog` from an empty directory (`--version`, `docs` read the installed copy of 14 topics,
  `info` exit 0). Not done: Linux, macOS, `pipx` (not installed), and `uvx
  ardupilot-log-tools`, which needs a PyPI release.
- [ ] Attach the wheel to each GitHub release, for sandboxes with no PyPI access but with
  numpy/pandas/scipy preinstalled (`pip install --no-deps <wheel>`). Moves to S5, which
  makes the first release.

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

*(Worked 2026-10-31 on branch `s2-skill-folder`.)*

- [x] `SKILL.md` frontmatter: `name`, a `description` (794 characters) naming the
  ArduPilot/ArduCopter wording, every check family and what it does not cover (PX4 `.ulg`,
  MAVLink `.tlog`), `license: MIT`, `compatibility` (Python 3.9+, uv/pipx/pip, and network
  plus git on first use: the fallback installs from the git URL until there is a PyPI
  release), `metadata.version` and `metadata.source`.
- [x] `SKILL.md` body in the planned order, 185 lines. The ground rules (guide §2), the
  window table and the `--flight` table (§3) are **generated** into marked regions from
  `GUIDE.md`, not copied by hand, so they cannot drift; the rest is hand-written. Added
  the two easy-to-get-wrong cases (notch: `FCNS.CF` not `FTN1.PkAvg`; PID: a 10 Hz log is
  refused) next to the recipe index.
- [x] `tools/build_skill.py`: copies 11 files into `references/` (the planned nine plus
  `param-decodings.md` and `current-sensor-calibration.md`, which are operating guidance);
  rewrites links to the skill's names (`SKILLS.md` → `recipes.md`, `reference/X.md` →
  `X.md`, a link to a left-out document → `alog docs X`); strips `RULES.md` §5 by heading
  and fails if the heading is gone; `--check` for CI. Deviation: the header names the
  source file and repository but **no commit**, because a commit hash would make every
  build differ and `--check` could never pass. `python alog.py` needed no rewrite (S1).
- [x] Left out: `TODO.md`, `docs/`, `pid-tuning-sources.md`, `existing-tools.md`,
  `dataflash-format.md` (reachable through `alog docs`), `tools/`, the tests. Code paths
  such as `dflog/checks.py::T` stay in the text as provenance; the header says they refer
  to the repository.
- [x] Validation: the spec's reference validator passes: `uvx --from skills-ref
  agentskills validate skills/ardupilot-log-analysis` → "Valid skill" (skills-ref 0.1.1,
  2026-10-31; the PyPI package's command is `agentskills`, not `skills-ref`). Its
  `read-properties` parses the same name, 794-character description, compatibility and
  metadata as `tests/test_skill.py`, which also checks the frontmatter rules itself (so
  the suite needs no download), plus: the skill is current, no vendor names, slash
  commands or agent tool names, every linked file and `alog docs` topic exists, and the
  scripts pin the package version and parse.
- [x] Run scripts `scripts/alog.sh` / `alog.ps1`: an installed `alog` whose version matches
  the pin, else `uvx` / `uv tool run`, else `pipx run`, else a mismatched installed `alog`
  with a warning, else the install line and exit 3. The pin is the package version and
  the git ref is `v<version>` for a release and `main` for a dev build, both set by
  `build_skill.py`. Tried: the matching-install, uvx and nothing-installed routes of
  `alog.sh`, and the matching and nothing routes of `alog.ps1`, on Windows. pipx untested
  (not installed). `.gitattributes` keeps `*.sh` LF so a Windows checkout can run it.
- [x] Package version bumped 2.0.0 → 2.1.0.dev0 (`pyproject.toml` and
  `dflog/__init__.py`): main already had features 2.0.0 never had, and the skill needs a
  version that has `alog docs`. S5's bump to 2.1.0 turns the scripts' ref into the tag.

**Done when:** the folder validates, contains no agent-specific wording, and
`build_skill.py` is idempotent (running it twice changes nothing).

## Phase S3: Keep the skill, the wheel and the docs in step

*(Worked 2026-10-31 on branch `s3-skill-docs-sync`.)*

- [x] `tests/test_skill.py`:
  - [x] the generated files are current (S2's `test_skill_is_current`, the same comparison
    as `build_skill.py --check`).
  - [x] every `alog ...` quoted in `SKILL.md`, `references/`, `README.md` and `AGENTS.md`
    (137 in the skill) uses a real subcommand and real options, checked against
    `dflog.cli.build_parser()` (extracted from `main()` for this, behaviour unchanged):
    global options only before the subcommand, the subcommand's own options after it,
    and a `--window` value that is a real method. A placeholder (`…`, `X`, `<cmd>`) ends
    the check. Not a full `parse_args`: the docs use placeholder notation that cannot
    parse, and names are what drift.
  - [x] exit codes: the "Exit codes: 0 pass, ..." sentences in `SKILL.md` and
    `recipes.md` name exactly the codes in `alog schema`, with matching meanings.
    JSON keys: the top-level keys in `json-output.md`'s envelope, check-report and
    `--flight all` examples are exactly the keys `alog all` and `alog all --flight all`
    emit on synthetic logs. Compared with real output rather than `alog schema`, whose
    key list is prose.
  - [x] `metadata.version` == `dflog.__version__` and the scripts pin it (S2).
- [x] Run in CI: `tests/run_all.py` finds `test_skill.py` by glob, and `tools/ci_local.py`
  runs it on Python 3.9 and 3.12.

Checked by breaking each one: an edited `reference/pitfalls.md` fails
`test_skill_is_current`; renaming `--cache-dir` fails the command check (it names the
`SKILL.md` line); bumping the version fails the frontmatter, script-pin and currency tests.

**Done when:** changing a canonical doc, a CLI flag or the version without regenerating
the skill fails CI.

## Phase S4: Per-agent installation

Install paths and manifest formats change often; verify each one when writing it and put
the date beside it in the README table.

*(Worked 2026-10-31 on branch `s4-agent-install`.)*

- [x] README "Use as an agent skill": agent → project directory → user directory → its own
  installer, for Claude Code, GitHub Copilot, Codex, Cursor and Gemini CLI, each read
  from the vendor's page on 2026-10-31 and linked. Finding: `~/.agents/skills/` (and
  project `.agents/skills/`) is read by Copilot, Codex, Cursor and Gemini CLI alike; Codex
  no longer documents `~/.codex/skills`. Claude Code reads `~/.claude/skills/`.
- [x] Install commands. `gh skill install` (GitHub CLI 2.90+, preview) installs from a
  GitHub repo for ~50 agents, finds `skills/*/SKILL.md` by the spec's convention and pins
  tags, so it is the README's first line rather than anything built here.
- [x] `alog install-skill --dir PATH [--force]`: copies the skill shipped inside the wheel
  (`setup.py` now copies `skills/` to `dflog/skill/`; a clone uses the repo's) to
  `PATH/ardupilot-log-analysis`. Same files → `unchanged`; different → exit 3 naming them
  unless `--force`; and it never deletes a folder whose `SKILL.md` is not this skill.
  `--dir` is required: no agent knowledge in the command.
- [x] Claude Code: `.claude-plugin/marketplace.json` only, no `plugin.json`. Its entry's
  `source` is `./skills/ardupilot-log-analysis`, so the skill folder is the plugin and gets
  no Claude-specific file, and the repo-root `CLAUDE.md` is not inside the plugin (the
  validator warns about one). `build_skill.py` sets its `version`; `test_skill.py` fails if
  stale. `claude plugin validate .` passes.
- [x] Other agents: no adapter file needed. Gemini installs from the git URL with `--path`;
  Copilot, Codex and Cursor read the shared directory, which `gh skill`, `alog
  install-skill` or a copy fill. Cursor's own `.cursor-plugin/marketplace.json` route was
  not built.
- [x] Agents without skill support: README says to put `alog` on `PATH` and paste the
  `SKILL.md` body into the agent's instructions file.

Tried on Windows, 2026-10-31: `alog install-skill` (install, unchanged, conflict,
`--force`, foreign folder), `gh skill install` from GitHub `main` and `--from-local`
(each into a scratch `--dir`; the result still passes `agentskills validate`), `claude
plugin validate .`, and the wheel's `install-skill` in `tools/ci_local.py`. **Not tried:**
installing into any agent's real directory, a Claude Code plugin install, and every agent
but Claude Code (none installed on the development machine). So the "done when" below
holds for the table's dating, not for "each path tried"; S5's cross-agent run is where the
rest gets tried.

**Done when:** the README table is dated and each listed path has been tried.

## Phase S5: Cross-agent acceptance test, then release

*(Started 2026-10-31 on branch `s5-agent-eval`.)*

- [x] Write `tests/agent-eval/` (not run in the suite — it costs money and needs
  accounts): `cases.py` (five prompts, each criterion a regex over the commands the agent
  ran or its answer, or `manual` where a regex cannot judge), `workspace.py` (a clean
  directory with the fixture, a PX4 ULog header and the skill at project level, nothing
  else from the repo), `grade.py`, `from_claude_stream.py`, `README.md`, `RESULTS.md`.
  The criteria were written against what `alog` actually says on the fixture, which
  corrected two of the planned ones:
  1. "Is it OK?" → `alog info` first; integrity; the window method `EV
     NOT_LANDED->LAND_COMPLETE`; one flight; the FAILs with their numbers.
  2. "Check the harmonic notch" → the fixture's notch is **disabled** (no `FCNS` to read),
     so: says so, quotes the report's starting point (`MODE 3`, `FREQ 55`) and the
     batch-logging flight that would verify it, never calls it working.
  3. "Are my motors balanced?" → the servo→motor map; pitch trim 58 µs; the
     drive-normalised spread (2.2 %, so load, not drag); and since `trim vs heading` is a
     SKIP, a breeze is not ruled out and the CG offset is not proven.
  4. "Recommend PID gains" → the fixture's refusal is **`NO_PID_MESSAGES`** (no PIDR or
     RATE at all), not `PID_RATE_TOO_LOW`; `LOG_BITMASK 180222 → 180223`; no invented gains.
  5. A PX4 `.ulg` → `alog` is not run on it.
  `tests/test_agent_eval.py` (in the suite) checks that `alog`'s own output on the fixture
  passes every automatic criterion, so a tool change that moves a figure fails there, and
  tests the grader. Writing it found a grader bug (an answer's own `## ` headings cut it
  short) and a tool drift (the tune refusal pointed at `SKILLS.md`), both fixed.
- [ ] Run it in each agent's non-interactive mode where one exists, by hand otherwise, on
  at least three agents from different vendors. Record agent, version, date, pass/fail
  per prompt in `tests/agent-eval/RESULTS.md`.
- [ ] Tune the `description` until prompts 1–4 trigger and 5 does not, on all three.
- [ ] Release: bump to 2.1.0 (new `docs`, `install-skill`, `--cache-dir`), CHANGELOG
  entry (drafted as "2.1.0 (unreleased)"), tag `v2.1.0`, publish to PyPI from the tag,
  attach the wheel and a zipped skill folder to the GitHub release.
  S0 chose trusted publishing from a GitHub Action, but this repository no longer runs
  Actions, so publishing is `uv publish` from a local build with a PyPI API token, run by
  the account owner. `tools/ci_local.py` must pass on the release commit first.
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
