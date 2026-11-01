# Changelog

## 2.1.0 (2026-10-31)

Packaging as an installable, agent-neutral skill (issue #24).

Not on PyPI. Install from the tag, `pip install
git+https://github.com/geoffreyheller/ardupilot-log-tools@v2.1.0` (or `uvx --from ...@v2.1.0
alog`), or from the wheel attached to the GitHub release. The skill's run scripts pin this
tag.

- **Agent skill** `skills/ardupilot-log-analysis/` in the open Agent Skills format:
  `SKILL.md`, the tool's documentation in `references/` (generated from the repo docs by
  `tools/build_skill.py`), and run scripts that use the matching `alog` or fetch it with
  `uvx` / `pipx`. Passes the reference validator (`agentskills validate`).
- **`alog install-skill --dir PATH`** copies the skill for the installed version into an
  agent's skills directory; the README lists each agent's. `gh skill install
  geoffreyheller/ardupilot-log-tools ardupilot-log-analysis` also works, and the repo is a
  Claude Code plugin marketplace (`.claude-plugin/marketplace.json`).
- **`alog docs [TOPIC]`**: the wheel carries the guide, rules, recipes and references, so
  the guidance matches the installed version. Report output that cited `reference/X.md`
  (every threshold `source`, the tune refusal) now names `alog docs X` too.
- **`--cache-dir DIR` / `ALOG_CACHE_DIR`** keep `.dfcache` files out of the log's folder.
- **Docs** write `alog ...`, the installed command; `python alog.py` is the clone form.
  The operating guide moved from `CLAUDE.md` to a neutral `GUIDE.md`.
- **Tests**: `tests/run_all.py` runs every test file; `tools/ci_local.py` runs the suite on
  Python 3.9 and 3.12 and the built wheel from outside the repo (there is no cloud CI);
  `tests/test_skill.py` keeps the skill in step with the docs and the CLI;
  `tests/agent-eval/` is the cross-agent acceptance test.

JSON output: `version` changes, and the `source` strings of results, thresholds and tune
constants now read "... `alog docs X` (reference/X.md)". No key was added or removed
except under the new `docs` and `install-skill` commands.

## 2.0.0

The version before packaging; never tagged.
