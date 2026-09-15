"""Check-level tests on synthetic logs: the September-2026 issues (#2, #5-#10).

Each test builds exactly the log its check needs with tests/synthlog.py, so it runs
anywhere with no flight data, then asserts on the Section the check returns. The real-log
counterparts are in tests/test_largeprop.py.

    python tests/test_checks.py
"""

import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import pytest                       # noqa: F401
except ImportError:
    import _shim as pytest

from dflog import Log, airborne_window                                   # noqa: E402
from dflog.analysis import check_gps, check_power                        # noqa: E402
from dflog.checks import T                                               # noqa: E402
from synthlog import LogWriter, standard_log                             # noqa: E402

TMP = tempfile.mkdtemp(prefix="dflog-checks-")
HZ = 10.0
SPAN = 120.0
FLIGHT = (10.0, 110.0)


def _load(w, name):
    return Log(w.write(os.path.join(TMP, name)), use_cache=False)


def _writer():
    """A copter log skeleton: banner, frame params, a flight by EV, and the formats the
    check tests below fill in. Column names and format chars follow ArduCopter 4.7."""
    w = LogWriter()
    w.fmt(96, "PARM", "QNff", "TimeUS,Name,Value,Default")
    w.fmt(97, "MSG", "QZ", "TimeUS,Message")
    w.fmt(64, "EV", "QB", "TimeUS,Id")
    w.fmt(200, "GPS", "QBBBfLLfIH", "TimeUS,I,Status,NSats,HDop,Lat,Lng,Alt,GMS,GWk")
    w.fmt(201, "GPA", "QBCfffH", "TimeUS,I,VDop,HAcc,VAcc,SAcc,Delta")
    w.msg("MSG", TimeUS=20, Message="ArduCopter V4.7.1 (deadbeef)")
    w.msg("PARM", TimeUS=30, Name="FRAME_CLASS", Value=1.0, Default=1.0)
    w.msg("PARM", TimeUS=31, Name="FRAME_TYPE", Value=1.0, Default=1.0)
    w.msg("EV", TimeUS=int(FLIGHT[0] * 1e6), Id=28)
    w.msg("EV", TimeUS=int(FLIGHT[1] * 1e6), Id=18)
    return w


def _gps_log(name, hacc0=0.8, vacc0=1.2, sacc0=0.3, spikes=(), second=True):
    """Two receivers: instance 0 with a 3D fix (accuracies as given, with `spikes` =
    [(t0, t1, hacc)] where HAcc jumps), instance 1 an NMEA unit that never gets a fix
    and reports HAcc 0 and the saturated VDop 655.35."""
    w = _writer()
    for i in range(int(SPAN * HZ)):
        t = i / HZ
        us = int(t * 1e6)
        h = next((h for a, b, h in spikes if a <= t <= b), hacc0)
        w.msg("GPS", TimeUS=us, I=0, Status=3, NSats=12, HDop=0.8, Lat=30.0 + t * 1e-6, Lng=-140.0,
              Alt=100.0, GMS=int(t * 1000), GWk=2400)
        w.msg("GPA", TimeUS=us, I=0, VDop=1.1, HAcc=h, VAcc=vacc0, SAcc=sacc0, Delta=200)
        if second:
            w.msg("GPS", TimeUS=us, I=1, Status=1, NSats=0, HDop=99.99, Lat=0.0, Lng=0.0, Alt=0.0,
                  GMS=0, GWk=0)
            w.msg("GPA", TimeUS=us, I=1, VDop=655.35, HAcc=0.0, VAcc=0.0, SAcc=0.0, Delta=99)
    return _load(w, name)


def _results(sec):
    return {r.name: r for r in sec.results}


def _table(sec, name):
    t = next((t for t in sec.tables if t["name"] == name), None)
    assert t is not None, f"no table {name!r}; have {[t['name'] for t in sec.tables]}"
    return t


def _notes(sec):
    return "\n".join(sec.notes)


# ------------------------------------------------------------ issue #2

def test_charger_recalibration_formula_is_charger_over_logged():
    """BATT_AMP_PERVLT scales the reading, so logged mAh > charger mAh means PERVLT must
    come DOWN: new = old * (charger / logged). The note used to print the inverse."""
    log = _load(standard_log(n=200), "power.bin")
    sec = check_power(log, airborne_window(log, method="none"))
    text = _notes(sec)
    assert "BATT_AMP_PERVLT * (charger / logged)" in text, text
    assert "(logged / charger)" not in text


# ------------------------------------------------------------ issue #5

def test_gpa_thresholds_are_registered_with_sources():
    for key in ("gps_hacc", "gps_vacc", "gps_sacc"):
        assert key in T and T[key]["source"], key
    assert T["gps_hacc"]["warn"] == 2.0 and T["gps_hacc"]["fail"] == 5.0


def test_gps_check_reports_receiver_accuracy_per_instance():
    """HAcc/VAcc/SAcc are the receiver's own accuracy estimates in metres - the number
    that answers 'is the GPS working better than it was', where HDOP is only geometry."""
    log = _gps_log("gpa.bin", spikes=[(50.0, 52.0, 3.0)])
    sec = check_gps(log, airborne_window(log, method="ev"))
    rs = _results(sec)
    t = _table(sec, "gpa")
    rows = {r[0]: r for r in t["rows"]}
    assert "GPS0" in rows and "GPS1" in rows, rows
    cols = t["columns"]
    g0 = dict(zip(cols, rows["GPS0"]))
    assert g0["HAcc med (m)"] == pytest.approx(0.8, abs=0.01)
    assert g0["HAcc p95 (m)"] == pytest.approx(0.8, abs=0.3)          # 2 s spike out of 100 s
    assert g0["VAcc med (m)"] == pytest.approx(1.2, abs=0.01)
    assert g0["SAcc med (m/s)"] == pytest.approx(0.3, abs=0.01)
    assert g0["VDop med"] == pytest.approx(1.1, abs=0.01)
    assert g0["fix interval (ms)"] == 200
    r = rs["GPS0 horizontal accuracy"]
    assert r.status == "PASS" and r.evidence["value"] == pytest.approx(0.8, abs=0.01)
    assert r.evidence["p95"] == pytest.approx(0.8, abs=0.3)
    assert "0.8" in r.summary and "m" in r.summary
    assert rs["GPS0 vertical accuracy"].status == "PASS"
    assert rs["GPS0 speed accuracy"].status == "PASS"
    # the receiver with no fix: not graded against zeros, and VDop 655.35 is "no fix"
    g1 = dict(zip(cols, rows["GPS1"]))
    assert g1["VDop med"] == "no fix", g1
    assert g1["fix interval (ms)"] == 99
    assert rs["GPS1 horizontal accuracy"].status == "SKIP"
    assert "no 3D fix" in rs["GPS1 horizontal accuracy"].summary
    notes = _notes(sec)
    assert "5.0 Hz" in notes and "10.1 Hz" in notes, notes


def test_gps_accuracy_is_graded_on_the_median():
    log = _gps_log("gpa_warn.bin", hacc0=4.25, vacc0=5.69, sacc0=1.34, second=False)
    rs = _results(check_gps(log, airborne_window(log, method="ev")))
    assert rs["GPS0 horizontal accuracy"].status == "WARN"
    assert rs["GPS0 vertical accuracy"].status == "WARN"
    assert rs["GPS0 speed accuracy"].status == "FAIL"
    log = _gps_log("gpa_fail.bin", hacc0=6.0, second=False)
    rs = _results(check_gps(log, airborne_window(log, method="ev")))
    assert rs["GPS0 horizontal accuracy"].status == "FAIL"


def test_gps_check_skips_accuracy_when_gpa_is_not_logged():
    w = _writer()
    for i in range(int(SPAN * HZ)):
        t = i / HZ
        w.msg("GPS", TimeUS=int(t * 1e6), I=0, Status=3, NSats=12, HDop=0.8, Lat=30.0, Lng=-140.0,
              Alt=100.0, GMS=int(t * 1000), GWk=2400)
    log = _load(w, "no_gpa.bin")
    sec = check_gps(log, airborne_window(log, method="ev"))
    rs = _results(sec)
    assert rs["GPS0 satellites"].status == "PASS"
    assert rs["receiver accuracy"].status == "SKIP" and "GPA" in rs["receiver accuracy"].summary
    assert not any(t["name"] == "gpa" for t in sec.tables)


if __name__ == "__main__":
    import _shim
    sys.exit(_shim.run(sys.modules[__name__]))
