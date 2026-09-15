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
from dflog.analysis import check_gps, check_motors, check_notch, check_power   # noqa: E402
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


def _results(sec):
    return {r.name: r for r in sec.results}


def _table(sec, name):
    t = next((t for t in sec.tables if t["name"] == name), None)
    assert t is not None, [t["name"] for t in sec.tables]
    return t


def _notes(sec):
    return "\n".join(sec.notes)


# --------------------------------------------------------------- issue #5

def test_gps_accuracy_figures_from_gpa():
    """Two receivers: GPS0 is the u-blox (UBX2 present, GPA.Delta 200 ms), GPS1 an NMEA
    unit that spends most of the flight without a fix (VDop 655.35 = saturated)."""
    sec = check_gps(LOG, airborne_window(LOG, method="ev"))
    t = _table(sec, "gpa")
    rows = {r[0]: dict(zip(t["columns"], r)) for r in t["rows"]}
    g0 = rows["GPS0"]
    assert g0["HAcc med (m)"] == pytest.approx(0.64, abs=0.02)
    assert g0["HAcc p95 (m)"] == pytest.approx(1.45, abs=0.05)
    assert g0["VAcc med (m)"] == pytest.approx(0.79, abs=0.02)
    assert g0["SAcc med (m/s)"] == pytest.approx(0.20, abs=0.02)
    assert g0["VDop med"] == pytest.approx(1.05, abs=0.02)
    assert g0["fix interval (ms)"] == 200
    rs = _results(sec)
    assert rs["GPS0 horizontal accuracy"].status == "PASS"
    assert rs["GPS0 vertical accuracy"].status == "PASS"
    assert rs["GPS0 speed accuracy"].status == "PASS"
    # GPS1 holds a 3D fix for most of the window but its NMEA driver supplies no accuracy
    # estimate: HAcc is 0 and VDop the saturated 655.35. That is "not reported", which is
    # a different SKIP from "no fix", and it must not be graded as 0 m.
    g1 = rows["GPS1"]
    assert g1["fix interval (ms)"] == 99
    assert g1["VDop med"] == "n/a", g1
    assert g1["HAcc med (m)"] is None
    assert rs["GPS1 horizontal accuracy"].status == "SKIP"
    assert "not report" in rs["GPS1 horizontal accuracy"].summary


# --------------------------------------------------------------- issue #6

def test_current_sensor_reads_12_amps_with_the_motors_stopped():
    """BATT_MONITOR=9 (ESC telemetry) on this aircraft reports ~12.3 A with every motor
    provably stopped. Every current figure in the flight is that much high: hover draw
    is ~7 A, not the ~19 A the check used to report."""
    w = airborne_window(LOG, method="ev")
    sec = check_power(LOG, w)
    rs = _results(sec)
    r = rs["BAT0 current with motors stopped"]
    assert r.status == "FAIL"
    assert r.evidence["value"] == pytest.approx(12.32, abs=0.15), r.evidence
    assert r.evidence["n"] > 100
    t = _table(sec, "bat0_bands")
    rows = {row[0]: dict(zip(t["columns"], row)) for row in t["rows"]}
    hover = rows["hover"]
    assert hover["current (A)"] == pytest.approx(19.3, abs=1.5)
    assert hover["offset-corrected (A)"] == pytest.approx(7.0, abs=1.5)
    assert 150.0 < hover["power (W)"] < 230.0
    # full throttle is a 5-sample band here (the 63 A in the issue was the single max)
    full = rows["full throttle"]
    assert full["n"] >= 5 and 45.0 < full["current (A)"] < 65.0
    assert full["offset-corrected (A)"] == pytest.approx(full["current (A)"] - 12.32, abs=0.2)
    assert rs["BAT0 hover power"].status == "PASS"
    assert rs["BAT0 hover power"].evidence["watts"] == pytest.approx(169.0, abs=5.0)


# --------------------------------------------------------------- issue #7

def test_drive_normalised_rpm_is_flat_while_raw_spread_is_not():
    """The raw RPM spread on this aircraft is ~12 %: pure load asymmetry from a CG
    offset. Per unit of drive the four motors sit within ~2.5 % - no motor is dragging."""
    sec = check_motors(LOG, airborne_window(LOG, method="ev"))
    rs = _results(sec)
    assert rs["RPM spread"].evidence["value"] > 10.0
    dn = rs["drive-normalised RPM spread"]
    assert dn.status == "PASS", dn.summary
    assert dn.evidence["value"] == pytest.approx(2.4, abs=0.8), dn.evidence
    t = _table(sec, "esc")
    rows = {r[0]: dict(zip(t["columns"], r)) for r in t["rows"]}
    for i in range(4):
        row = rows[f"ESC{i}"]
        assert row["RPM p05"] < row["RPM median"] < row["RPM p95"]
        assert 30.0 < row["Temp mean"] < 35.0 and row["Temp max"] <= 37.0
    assert rows["ESC0"]["motor"] == "M3"           # SERVO1_FUNCTION=35
    assert rs["ESC temperature"].status == "PASS"
    assert rs["ESC temperature spread"].status == "PASS"


# --------------------------------------------------------------- issue #8

def test_cg_offset_and_level_hover_cross_check():
    """Motor-order RPM medians M1 5349, M2 4843, M3 5148, M4 4742: front pair 5248,
    rear 4792, thrust ratio (5248/4792)^2 = 1.199, so the CG sits 9.1 % of the fore-aft
    arm forward - 13.7 mm on a 151 mm arm. The pitch trim over the whole window and over
    the three LOITER hover chunks agree, so it is a static asymmetry."""
    w = airborne_window(LOG, method="ev")
    sec = check_motors(LOG, w, arm_mm=151.0)
    t = _table(sec, "cg")
    rows = {r[0]: dict(zip(t["columns"], r)) for r in t["rows"]}
    assert rows["pitch"]["CG offset (% of arm)"] == pytest.approx(9.1, abs=0.5), rows
    assert rows["pitch"]["mm"] == pytest.approx(13.7, abs=0.8)
    assert rows["roll"]["CG offset (% of arm)"] == pytest.approx(-1.0, abs=0.7)
    rs = _results(sec)
    assert rs["pitch trim"].evidence["signed"] > 40.0
    r = rs["trim vs level hover"]
    assert r.status == "PASS", r.summary
    assert "hover chunk" in r.summary
    assert abs(r.evidence["window"]["pitch"] - r.evidence["hover"]["pitch"]) < 10.0


# --------------------------------------------------------------- issue #9

def test_notch_disabled_gets_an_envelope_and_a_starting_point():
    sec = check_notch(LOG, airborne_window(LOG, method="ev"))
    rs = _results(sec)
    assert rs["notch"].status == "SKIP" and "DISABLED" in rs["notch"].summary
    env = {r[0]: r[1] for r in _table(sec, "fundamental")["rows"]}
    assert env["median"] == pytest.approx(83.8, abs=0.5), env
    assert env["p01"] == pytest.approx(59.0, abs=3.0)
    assert env["max"] == pytest.approx(121.0, abs=3.0)
    rec = {r[0]: r[1] for r in _table(sec, "recommendation")["rows"]}
    assert rec["INS_HNTCH_MODE"] == 3 and rec["INS_HNTCH_FREQ"] == 55 and rec["INS_HNTCH_BW"] == 27
    assert rec["INS_HNTCH_HMNCS"] == 3 and rec["INS_HNTCH_OPTS"] == 2
    notes = _notes(sec)
    assert "4/4 motors" in notes and "batchfft" in notes


if __name__ == "__main__":
    import _shim
    sys.exit(_shim.run(sys.modules[__name__]))
