"""The cross-agent acceptance cases (tests/agent-eval/) still describe what alog says.

The cases are run by hand against real agents and are not part of this suite. What runs
here: an answer that faithfully quotes `alog`'s own output on the fixture must pass every
automatic answer criterion of its case, so a change to the tool that moves a figure the
cases expect fails here instead of making the next agent run look wrong; and the grader
itself is checked on canned transcripts.

    python tests/test_agent_eval.py
"""

import io
import os
import re
import sys
from contextlib import redirect_stdout

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path[:0] = [HERE, ROOT, os.path.join(HERE, "agent-eval")]

try:
    import pytest                       # noqa: F401
except ImportError:
    import _shim as pytest              # noqa: F401

import grade                                          # noqa: E402
from cases import CASES, FIXTURE                      # noqa: E402
from dflog.cli import main                            # noqa: E402

FIX = os.path.join(HERE, "fixtures", FIXTURE)
# what a faithful answer to each case would be built from
TOOL_OUTPUT = {"1-triage": [["info"], ["all"]], "2-notch": [["notch"]], "3-motors": [["motors"]],
               "4-pid": [["tune"]]}
# The window the recipes use (Skill 3, Skill 4) moves some figures, so a faithful answer on it
# must pass too: the notch FREQ is 60 there, not 55.
TOOL_OUTPUT_RPM = {"2-notch": [["notch", "--window", "rpm"]], "3-motors": [["motors", "--window", "rpm"]]}


def _alog(*argv):
    buf = io.StringIO()
    with redirect_stdout(buf):
        main(["--no-cache"] + list(argv))
    return buf.getvalue()


def _case(case_id):
    return next(c for c in CASES if c["id"] == case_id)


def test_the_tool_output_satisfies_each_case_it_answers():
    for case_id, runs in list(TOOL_OUTPUT.items()) + list(TOOL_OUTPUT_RPM.items()):
        text = "\n".join(_alog(cmd[0], FIX, *cmd[1:]) for cmd in runs)
        transcript = f"## Commands\nalog info {FIXTURE}\n\n## Answer\n{text}"
        results = dict(grade.grade(_case(case_id), transcript))
        failed = [name for name, verdict in results.items() if verdict is False]
        assert not failed, f"{case_id}: alog's own output fails {failed}"


def test_every_case_has_automatic_criteria_and_known_parts():
    assert [c["id"] for c in CASES] == ["1-triage", "2-notch", "3-motors", "4-pid", "5-px4"]
    for case in CASES:
        assert case["prompt"] and any(k != "manual" for _, _, k, _ in case["criteria"]), case["id"]
        for name, where, kind, pattern in case["criteria"]:
            assert where in ("commands", "answer") and kind in ("present", "absent", "first", "manual"), name
            if kind in ("present", "absent"):
                re.compile(pattern)


def test_first_subcommand_skips_global_options_and_the_run_scripts():
    f = grade.first_subcommand
    assert f("ls\nalog info x.bin\nalog all x.bin") == "info"
    assert f("alog --json --cache-dir /tmp/c info x.bin") == "info"
    assert f("sh skills/ardupilot-log-analysis/scripts/alog.sh --no-cache all x.bin") == "all"
    assert f("pwsh scripts/alog.ps1 motors x.bin") == "motors"
    assert f("alog --version\nalog docs pitfalls") == "docs"
    assert f("cat notes.txt") is None


def test_grader_passes_a_good_answer_and_fails_a_bad_one():
    case = _case("4-pid")
    good = ("## Commands\nalog info largeprop-quad.bin\nalog tune largeprop-quad.bin\n\n"
            "## Answer\nRefused: NO_PID_MESSAGES. Set LOG_BITMASK 180222 -> 180223 and fly the profile.\n")
    bad = ("## Commands\nalog tune largeprop-quad.bin\n\n"
           "## Answer\nSet ATC_RAT_RLL_P to 0.12 and ATC_RAT_PIT_P = 0.11.\n")
    assert all(v for _, v in grade.grade(case, good))
    verdicts = dict(grade.grade(case, bad))
    assert verdicts["runs `alog info` first"] is False
    assert verdicts["reports the refusal code"] is False
    assert verdicts["invents no rate gain"] is False


def test_an_answer_with_its_own_headings_is_read_to_the_end():
    text = "## Commands\nalog info x.bin\n\n## Answer\n## Summary\nfine\n## Detail\nNO_PID_MESSAGES\n"
    parts = grade.sections(text)
    assert parts["commands"].strip() == "alog info x.bin"
    assert "NO_PID_MESSAGES" in parts["answer"] and "## Summary" in parts["answer"]


def test_real_phrasings_from_the_first_claude_code_run_pass():
    # 2026-10-31: five correct answers failed six criteria on wording alone. Each phrase is
    # quoted from that run (tests/agent-eval/RESULTS.md); each must now pass.
    triage = _case("1-triage")
    answer = ("The file is intact: every byte decoded, no resyncs. It holds one flight, 301 s long "
              "(76.5-377.4 s, measured from the flight controller's \"not landed\" to \"land complete\" "
              "events). Standing pitch trim is +58 µs (the tool fails above 25 µs).")
    verdicts = dict(grade.grade(triage, "## Commands\nalog.sh info x.bin\n\n## Answer\n" + answer))
    assert verdicts["reports the integrity result"] and verdicts["names the window and its method"]
    assert grade.first_subcommand('S=.claude/skills/x/scripts/alog.sh; bash $S info largeprop-quad.bin') == "info"
    assert grade.first_subcommand("S=.claude/skills/x; ls; $S/scripts/alog.sh info a.bin") == "info"
    notch = dict(grade.grade(_case("2-notch"), "## Answer\n| `INS_HNTCH_FREQ` | 60 | just below the p01 |\n"))
    assert notch["gives FREQ from the report"]
    motors = dict(grade.grade(_case("3-motors"),
                              "## Answer\n| RPM per unit of drive (RPM ÷ duty × voltage) | 2.2% |\n"))
    assert motors["separates load from drag"]


def test_the_widened_criteria_still_fail_an_answer_that_misses_them():
    verdicts = dict(grade.grade(_case("1-triage"), "## Answer\nLooks fine to me, nice flight.\n"))
    assert verdicts["reports the integrity result"] is False
    assert verdicts["names the window and its method"] is False
    assert grade.first_subcommand("S=notes.txt; cat $S; X=1 alog-like info") is None
    notch = dict(grade.grade(_case("2-notch"), "## Answer\nSet INS_HNTCH_FREQ to 80.\n"))
    assert notch["gives FREQ from the report"] is False


def test_out_of_scope_case_fails_when_alog_reads_the_ulog():
    case = _case("5-px4")
    ran = "## Commands\nalog info flight.ulg\n\n## Answer\nalog says it is not a dataflash log.\n"
    declined = "## Commands\nls\n\n## Answer\nThat is a PX4 ULog; this skill reads ArduPilot logs only.\n"
    assert dict(grade.grade(case, ran))["does not run alog on the .ulg"] is False
    assert dict(grade.grade(case, declined))["does not run alog on the .ulg"] is True


if __name__ == "__main__":
    import _shim
    sys.exit(_shim.run(sys.modules[__name__]))
