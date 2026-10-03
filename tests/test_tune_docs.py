"""WP8 of docs/pid-tuning-plan.md: the PID-tuning documentation agrees with the code.

The logging-requirements table is printed verbatim in SKILLS.md and CLAUDE.md from
`tune.LOGGING_REQUIREMENTS`; the refusal codes, the thresholds and the confidence priors
are named in the reference docs. Nothing here reads a log: it reads the markdown and the
tables in `dflog.tune` / `dflog.tune_fuse` / `dflog.checks` and compares them, so a
constant that changes without its documentation fails here rather than in a report.

    python tests/test_tune_docs.py
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import pytest                       # noqa: F401
except ImportError:
    import _shim as pytest              # noqa: F401

from dflog.checks import T                                               # noqa: E402
from dflog import tune, tune_fuse                                        # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

SKILLS = os.path.join(ROOT, "SKILLS.md")
CLAUDE = os.path.join(ROOT, "CLAUDE.md")
README = os.path.join(ROOT, "README.md")
THRESHOLDS = os.path.join(ROOT, "reference", "thresholds.md")
SOURCES = os.path.join(ROOT, "reference", "pid-tuning-sources.md")
PITFALLS = os.path.join(ROOT, "reference", "pitfalls.md")
EXISTING = os.path.join(ROOT, "reference", "existing-tools.md")
TEMPLATE = os.path.join(ROOT, "templates", "log-analysis-template.md")

ERROR_LINE = "ERROR: these log files cannot be used for PID tuning."


def _read(path):
    with open(path, encoding="utf-8") as fh:
        return fh.read()


def _table_after(text, marker):
    """The first markdown table after `marker`: [[cell, ...], ...] without the header
    and separator rows, backticks stripped from every cell."""
    at = text.index(marker)
    lines = text[at:].splitlines()
    rows, started = [], False
    for line in lines[1:]:
        if line.startswith("|"):
            started = True
            cells = [c.strip().strip("`") for c in line.strip().strip("|").split("|")]
            rows.append(cells)
        elif started:
            break
    assert len(rows) >= 3, f"no table after {marker!r}"
    header, sep, body = rows[0], rows[1], rows[2:]
    assert all(set(c) <= set("-: ") for c in sep), f"second row after {marker!r} is not a separator"
    return header, body


def _requirements_rows():
    return [[r["param"], r["required"], r["why"], r["tier"]] for r in tune.LOGGING_REQUIREMENTS]


# ------------------------------------------------------------- (a) thresholds

def test_every_tune_threshold_has_a_row_in_thresholds_md():
    doc = _read(THRESHOLDS)
    keys = [k for k in T if k.startswith("tune_")]
    assert len(keys) >= 13, keys
    for k in keys:
        assert T[k]["source"], k
        assert re.search(r"^\| `" + re.escape(k) + r"` \|", doc, re.M), f"{k} has no row in reference/thresholds.md"


def test_confidence_priors_are_documented_with_their_values():
    doc = _read(THRESHOLDS)
    assert "Confidence priors" in doc and "PRIORS" in doc
    for method, c in tune_fuse.PRIORS.items():
        assert c["source"], method
        m = re.search(r"^\| `" + re.escape(method) + r"` \| ([0-9.]+) \|", doc, re.M)
        assert m and float(m.group(1)) == c["value"], \
            f"{method} prior {c['value']:g} is not in reference/thresholds.md"
    cap = tune_fuse.FUSE_CONSTANTS["fuse_tier_c_confidence_cap"]["value"]
    assert f"{cap:g}" in doc and "fuse_tier_c_confidence_cap" in doc
    assert cap < T["tune_confidence"]["fail"]


# ------------------------------------------------ (b) the logging-requirements table

def test_skills_table_is_logging_requirements_verbatim():
    header, body = _table_after(_read(SKILLS), "### Before you fly: set these parameters")
    assert [h.lower() for h in header] == ["parameter", "required for a tuning log", "why", "tier"]
    want = _requirements_rows()
    assert len(body) == len(want), (len(body), len(want))
    for got, exp in zip(body, want):
        assert got == exp, f"SKILLS.md row {got} != tune.LOGGING_REQUIREMENTS row {exp}"


def test_claude_md_table_is_logging_requirements_verbatim():
    header, body = _table_after(_read(CLAUDE), "PID tuning needs a fast-logged flight")
    assert [h.lower() for h in header] == ["parameter", "required for a tuning log", "why", "tier"]
    want = _requirements_rows()
    assert len(body) == len(want), (len(body), len(want))
    for got, exp in zip(body, want):
        assert got == exp, f"CLAUDE.md row {got} != tune.LOGGING_REQUIREMENTS row {exp}"


def test_every_required_parameter_is_named_in_skills_claude_and_readme():
    docs = {name: _read(path) for name, path in (("SKILLS.md", SKILLS), ("CLAUDE.md", CLAUDE), ("README.md", README))}
    for row in tune.LOGGING_REQUIREMENTS:
        for name, text in docs.items():
            assert re.search(r"\b" + re.escape(row["param"]) + r"\b", text), f"{row['param']} is not in {name}"


def test_the_fix_header_points_at_a_skill_that_exists():
    # tune.LOGGING_FIX_HEADER says "(SKILLS.md, 'Recommend PID gains')"; the heading must exist
    m = re.search(r"SKILLS\.md, '([^']+)'", tune.LOGGING_FIX_HEADER)
    assert m, tune.LOGGING_FIX_HEADER
    skills = _read(SKILLS)
    assert re.search(r"^## Skill \d+ — " + re.escape(m.group(1)), skills, re.M), \
        f"no SKILLS.md heading contains {m.group(1)!r}"


# ------------------------------------------------------------ (c) refusal codes

def test_every_refusal_code_is_documented():
    text = _read(SKILLS) + _read(CLAUDE)
    for code in tune.REFUSAL_CODES:
        assert f"`{code}`" in text, f"{code} is in tune.REFUSAL_CODES but not in SKILLS.md or CLAUDE.md"
    # and each is in SKILLS.md's numbered list with a meaning, not just mentioned
    skills = _read(SKILLS)
    for code in tune.REFUSAL_CODES:
        assert re.search(r"^\s+- `" + re.escape(code) + r"`( \(tier [AB] only[^)]*\))? — ", skills, re.M), \
            f"{code} has no bullet with a meaning in SKILLS.md"


# ------------------------------------------------------------- (d) the ERROR block

def test_skills_carries_the_error_block_and_the_methods():
    skills = _read(SKILLS)
    assert ERROR_LINE in skills
    assert tune.LOGGING_FIX_HEADER in skills, "the worked example must carry the real fix header"
    # the worked example is the real block: the parameter column of every requirement row
    at = skills.index(ERROR_LINE)
    block = skills[at:at + 3000]
    for row in tune.LOGGING_REQUIREMENTS:
        assert re.search(r"^  " + re.escape(row["param"]) + r"\s", block, re.M), f"{row['param']} missing from the worked example"
    for method in tune_fuse.PRIORS:
        assert f"`{method}`" in skills, f"method {method} is not named in SKILLS.md"
    for tbl in ("recommendations", "ceilings", "step", "plant", "margins", "autotune", "params", "refusals", "logs"):
        assert f"`{tbl}`" in skills, f"table {tbl} is not named in SKILLS.md"
    assert "withheld" in skills and "validate before applying" in skills
    for word in ("prior", "adequacy", "excitation", "consistency", "agreement"):
        assert word in skills


def test_claude_md_names_the_command_the_pitfalls_and_the_tests():
    claude = _read(CLAUDE)
    assert "alog.py tune" in claude and ERROR_LINE in claude
    for line in ("standard `LOG_BITMASK` logs PID at 10 Hz", "`ATT` vs `ANG`", "`ATC_ACCEL_x_MAX` is cdeg/s²",
                 "`ATUN.ddt` is unscaled cdeg/s²", "hover-only log gives high coherence", "test gains, not the final"):
        assert line in claude, line
    for name in ("test_tune_extract", "test_tune_sim", "test_tune_step", "test_tune_atun", "test_tune_ident",
                 "test_tune_fuse", "test_tune_docs"):
        assert f"tests/{name}.py" in claude, name
        assert os.path.exists(os.path.join(ROOT, "tests", name + ".py")), name


def test_reference_docs_carry_the_pid_tuning_material():
    pit = _read(PITFALLS)
    assert "## PID tuning" in pit
    for phrase in ("10 Hz", "`ANG`", "ATC_ACCEL_x_MAX", "ATUN.ddt", "hover-only", "test gains"):
        assert phrase in pit, phrase
    ex = _read(EXISTING)
    assert "### PID tuning tools" in ex and "pid-tuning-sources.md" in ex
    for tool in ("PID-Analyzer", "PIDtoolbox", "PIDReview", "AnalyticTune", "PX4", "fpvpidlab"):
        assert tool in ex, tool
    src = _read(SOURCES)
    assert "116 700" in src and "110 700 cdeg" not in src
    assert tune_fuse.mission_planner_initial(10)["accel_rp_cdss"] == 116700.0
    tpl = _read(TEMPLATE)
    assert "Tune" in tpl and "withheld" in tpl and "| axis | param | current | recommended | change % | confidence | method | why |" in tpl
    readme = _read(README)
    assert "Logging for PID tuning" in readme and "alog.py tune" in readme


if __name__ == "__main__":
    import _shim
    sys.exit(_shim.run(sys.modules[__name__]))
