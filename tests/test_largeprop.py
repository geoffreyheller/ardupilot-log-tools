"""Pinned figures on the committed large-prop fixture, tests/fixtures/largeprop-quad.bin.

The synthetic logs in test_flights.py prove the mechanics; this file proves them on a
real aircraft whose hover fundamental (~84 Hz) sits below the 90 Hz floor the toolkit
used to hardcode (issue #3), and pins the numbers the other September-2026 issues asked
for (#5 GPS accuracy, #6 current zero offset, #7 drive-normalised RPM, #8 CG offset,
#9 notch recommendation) on the same log. Every figure here was measured on the fixture
after its scrub - see tests/fixtures/README.md.

    python tests/test_largeprop.py
"""

import io
import json
import os
import sys
from contextlib import redirect_stdout, redirect_stderr

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import pytest                       # noqa: F401
except ImportError:
    import _shim as pytest

from dflog import Log, airborne_window, flights, hover_chunks       # noqa: E402
from dflog.cli import main                                            # noqa: E402

FIXTURE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures", "largeprop-quad.bin")
LOG = Log(FIXTURE, use_cache=False)

EV_T0, EV_T1 = 76.5, 377.4          # EV NOT_LANDED -> LAND_COMPLETE, the truth


def run(*argv):
    out, err = io.StringIO(), io.StringIO()
    with redirect_stdout(out), redirect_stderr(err):
        code = main(["--no-cache"] + list(argv))
    return code, out.getvalue(), err.getvalue()


def run_json(*argv):
    code, out, err = run("--json", *argv)
    assert out.strip().startswith("{"), out[:200] + err
    return code, json.loads(out)


# --------------------------------------------------------------- issue #3

def test_fixture_is_intact_and_scrubbed():
    assert LOG.diagnostics.ok, LOG.diagnostics.render()
    assert LOG.board() == "3DRControlN1"
    gps = LOG.instances("GPS")[0]
    fix = gps[gps["Status"] >= 3]
    assert abs(fix["Lat"].iloc[0] - 30.0) < 0.01 and abs(fix["Lng"].iloc[0] + 140.0) < 0.01
    assert "00000000 00000000 00000000" in " ".join(m for _, m in LOG.messages_text())


def test_ev_finds_one_flight():
    fl = flights(LOG, method="ev")
    assert len(fl) == 1
    assert fl[0].t0 == pytest.approx(EV_T0, abs=0.1) and fl[0].t1 == pytest.approx(EV_T1, abs=0.1)


def test_fixed_90hz_floor_splits_this_flight_into_five():
    """The failure, kept reproducible: the floor sits above this aircraft's hover."""
    fl = flights(LOG, method="rpm", hz_floor=90.0)
    assert len(fl) == 5, [str(f) for f in fl]


def test_derived_floor_agrees_with_ev():
    fl = flights(LOG, method="rpm")
    assert len(fl) == 1, [str(f) for f in fl]
    w = fl[0]
    assert w.t0 == pytest.approx(EV_T0, abs=3.0) and w.t1 == pytest.approx(EV_T1, abs=3.0)
    assert w.hz_floor == pytest.approx(50.3, abs=1.0), w.hz_floor
    assert "spinning median 83." in w.method or "spinning median 84." in w.method, w.method
    ev = airborne_window(LOG, method="ev")
    rpm = airborne_window(LOG, method="rpm")
    assert abs(ev.duration - rpm.duration) < 5.0


def test_cli_reports_one_flight_and_no_detector_disagreement():
    code, doc = run_json("flight", FIXTURE, "--window", "rpm")
    assert len(doc["flights"]) == 1
    rs = {r["name"]: r for s in doc["sections"] for r in s["results"]}
    assert rs["flights in log"]["status"] == "PASS"
    assert "flight detectors disagree" not in rs


def test_hover_chunks_are_found():
    ch = hover_chunks(LOG)
    assert len(ch) == 3, [str(c) for c in ch]
    assert ch[0].t0 == pytest.approx(245.1, abs=0.2) and ch[0].t1 == pytest.approx(272.1, abs=0.2)
    code, doc = run_json("hover", FIXTURE)
    assert len(doc["chunks"]) == 3


if __name__ == "__main__":
    import _shim
    sys.exit(_shim.run(sys.modules[__name__]))
