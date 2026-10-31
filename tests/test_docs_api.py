"""The Python snippet in CLAUDE.md §1 runs, and its accessors return what it says (#20).

The snippet is executed as written against a synthetic log, with only the file name
swapped, so an accessor renamed in `dflog` fails here rather than in someone's script.

    python tests/test_docs_api.py
"""

import os
import re
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import pytest                       # noqa: F401
except ImportError:
    import _shim as pytest              # noqa: F401

import synthlog                                                          # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CLAUDE = os.path.join(ROOT, "CLAUDE.md")


def _snippet():
    """The first ```python block in CLAUDE.md §1."""
    with open(CLAUDE, encoding="utf-8") as fh:
        text = fh.read()
    sec = text[text.index("## 1. Start here"):text.index("## 2. ")]
    m = re.search(r"```python\n(.*?)```", sec, re.S)
    assert m, "no ```python block in CLAUDE.md §1"
    return m.group(1)


def _log_path():
    """A standard synthetic log plus a MODE record (LOITER, by number) and an in-flight
    parameter change."""
    w = synthlog.standard_log()
    w.fmt(202, "MODE", "QMBB", "TimeUS,Mode,ModeNum,Rsn")
    w.msg("MODE", TimeUS=1_100_000, Mode=5, ModeNum=5, Rsn=1)
    w.msg("PARM", TimeUS=1_200_000, Name="FRAME_TYPE", Value=3.0, Default=1.0)
    path = os.path.join(tempfile.mkdtemp(), "flight.bin")
    w.write(path)
    return path


def _run():
    code = _snippet()
    assert '"flight.bin"' in code
    path = _log_path()
    ns = {}
    exec(compile(code.replace('"flight.bin"', repr(path)), "CLAUDE.md §1", "exec"), ns)
    return ns


def test_snippet_runs_as_written():
    ns = _run()
    for name in ("log", "w", "esc", "rate"):
        assert name in ns, name


def test_snippet_names_the_accessors_from_issue_20():
    code = _snippet()
    for needle in ("w.t0, w.t1", "log.params()", "log.param(", "mode_timeline(log)"):
        assert needle in code, needle


def test_window_bounds_are_t0_t1():
    w = _run()["w"]
    assert w.t1 >= w.t0 and abs(w.duration - (w.t1 - w.t0)) < 1e-9


def test_params_is_a_method_returning_a_dict():
    log = _run()["log"]
    assert callable(log.params)
    p = log.params()
    assert p["FRAME_CLASS"] == 1.0 and p["FRAME_TYPE"] == 3.0   # last value wins
    assert log.param("FRAME_CLASS") == 1.0
    assert log.param("NO_SUCH_PARAM", -1) == -1
    assert [(n, o, v) for _t, n, o, v in log.param_changes()] == [("FRAME_TYPE", 1.0, 3.0)]


def test_mode_is_a_number_and_mode_timeline_decodes_it():
    ns = _run()
    log = ns["log"]
    d = log.df("MODE")
    assert int(d["Mode"].iloc[0]) == 5
    assert (d["Mode"] == "LOITER").sum() == 0           # the trap the docs warn about
    tl = ns["mode_timeline"](log)
    assert [(num, name, rsn) for _t, num, name, rsn in tl] == [(5, "LOITER", 1)]


if __name__ == "__main__":
    import _shim
    sys.exit(_shim.run(sys.modules[__name__]))
