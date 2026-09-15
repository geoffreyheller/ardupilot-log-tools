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
from dflog.analysis import check_gps, check_motors, check_power          # noqa: E402
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
    w.fmt(202, "ESC", "QBfffff", "TimeUS,Instance,RPM,Volt,Curr,Temp,Err")
    w.fmt(203, "RCOU", "QHHHH", "TimeUS,C1,C2,C3,C4")
    w.fmt(204, "BAT", "QBfff", "TimeUS,Inst,Volt,Curr,CurrTot")
    w.fmt(205, "CTUN", "Qfff", "TimeUS,ThO,ThH,BAlt")
    w.fmt(206, "ATT", "Qffff", "TimeUS,DesRoll,Roll,DesPitch,Pitch")
    w.fmt(207, "MODE", "QBBB", "TimeUS,ModeNum,Rsn,ThrCrs")
    w.fmt(208, "RCIN", "QHHHH", "TimeUS,C1,C2,C3,C4")
    w.fmt(209, "IMU", "QBfff", "TimeUS,I,GyrX,GyrY,GyrZ")
    w.msg("MSG", TimeUS=20, Message="ArduCopter V4.7.1 (deadbeef)")
    w.msg("PARM", TimeUS=30, Name="FRAME_CLASS", Value=1.0, Default=1.0)
    w.msg("PARM", TimeUS=31, Name="FRAME_TYPE", Value=1.0, Default=1.0)
    w.msg("EV", TimeUS=int(FLIGHT[0] * 1e6), Id=28)
    w.msg("EV", TimeUS=int(FLIGHT[1] * 1e6), Id=18)
    return w


def _params(w, **kv):
    for i, (k, v) in enumerate(kv.items()):
        w.msg("PARM", TimeUS=40 + i, Name=k, Value=float(v), Default=float("nan"))


# A quad whose motors obey RPM = K * duty * V. Servo outputs are wired in a non-identity
# order (C1 = motor 4, C2 = motor 1, C3 = motor 2, C4 = motor 3 - the common AIO default),
# so ESC instance i is servo output i+1 and motor m is on chans[m-1].
SERVO_FN = dict(SERVO1_FUNCTION=36, SERVO2_FUNCTION=33, SERVO3_FUNCTION=34, SERVO4_FUNCTION=35)
CHAN_OF_MOTOR = {1: 2, 2: 3, 3: 4, 4: 1}          # motor -> servo output (1-based)
PACK_V = 25.0


def _motor_log(name, k=(340.0, 340.0, 340.0, 340.0), pwm_trim=(0, 0, 0, 0), temps=(35.0, 36.0, 35.0, 36.0),
               stopped_current=0.0, hover_pwm=1400, idle_pwm=1100, full=(80.0, 90.0), att=None,
               modes=(), hover_thh=0.40, extra_params=None):
    """Ground (0-5 s motors stopped, 5-10 s armed idle), flight 10-110 s at hover_pwm with a
    full-throttle burst over `full`, landing, then motors stopped again 110-120 s.

    `k[m-1]` is motor m's RPM per (duty x volt); `pwm_trim[m-1]` a standing offset in us on
    motor m (a CG offset loads a pair harder); `stopped_current` what BAT.Curr reads with
    every motor stopped (a zero offset, issue #6). Current follows total duty otherwise.
    `att` is a callable t -> (roll, pitch) degrees; `modes` [(t, mode_num)].
    """
    w = _writer()
    _params(w, SERVO1_MIN=1000, SERVO1_MAX=2000, MOT_SPIN_MIN=0.15, MOT_SPIN_MAX=0.95, MOT_SPIN_ARM=0.10,
            MOT_THST_HOVER=hover_thh, **SERVO_FN, **(extra_params or {}))
    for t, num in modes:
        w.msg("MODE", TimeUS=int(t * 1e6), ModeNum=num, Rsn=1, ThrCrs=0)
    tot_mah = 0.0
    for i in range(int(SPAN * HZ)):
        t = i / HZ
        us = int(t * 1e6)
        stopped = t < 5.0 or t > FLIGHT[1]
        idle = 5.0 <= t < FLIGHT[0]
        burst = full[0] <= t <= full[1]
        base = 1000 if stopped else (idle_pwm if idle else (1950 if burst else hover_pwm))
        pwm = {}
        for m in range(1, 5):
            pwm[m] = base if stopped else base + pwm_trim[m - 1]
        v = PACK_V - 0.002 * t
        duty = {m: (pwm[m] - 1000) / 1000.0 for m in pwm}
        rpm = {m: (0.0 if stopped else k[m - 1] * duty[m] * v) for m in pwm}
        curr = stopped_current + 60.0 * sum(duty.values()) / 4.0
        tot_mah += curr / HZ / 3600.0 * 1000.0
        tho = 0.0 if (stopped or idle) else (1.0 if burst else hover_thh)
        w.msg("RCOU", TimeUS=us, **{f"C{CHAN_OF_MOTOR[m]}": int(pwm[m]) for m in pwm})
        for m in range(1, 5):
            inst = CHAN_OF_MOTOR[m] - 1
            w.msg("ESC", TimeUS=us, Instance=inst, RPM=rpm[m], Volt=v, Curr=curr / 4, Temp=temps[m - 1], Err=2.5)
        w.msg("BAT", TimeUS=us, Inst=0, Volt=v, Curr=curr, CurrTot=tot_mah)
        w.msg("CTUN", TimeUS=us, ThO=tho, ThH=hover_thh, BAlt=0.0 if (stopped or idle) else 5.0)
        if att is not None:
            r, p = att(t)
            w.msg("ATT", TimeUS=us, DesRoll=r, Roll=r, DesPitch=p, Pitch=p)
        if modes:
            w.msg("RCIN", TimeUS=us, C1=1500, C2=1500, C3=1500, C4=1500)
    return _load(w, name)


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


# ------------------------------------------------------------ issue #6

def test_power_thresholds_are_registered():
    assert "curr_stopped_a" in T and T["curr_stopped_a"]["source"]


def test_current_zero_offset_is_measured_with_the_motors_stopped():
    """Every motor provably stopped (ESC RPM 0, RCOU at SERVO_MIN) and the sensor still
    reads 12.5 A: that is offset, not draw, and every figure in the flight is 12.5 A high.
    The measurement lives outside the airborne window by definition."""
    log = _motor_log("offset.bin", stopped_current=12.5)
    w = airborne_window(log, method="ev")
    sec = check_power(log, w)
    rs = _results(sec)
    r = rs["BAT0 current with motors stopped"]
    assert r.status == "FAIL", r.summary
    assert r.evidence["value"] == pytest.approx(12.5, abs=0.05)
    assert r.evidence["n"] >= 100 and r.evidence["sd"] < 0.1
    assert "12.5" in r.summary and "motor stopped" in r.summary.lower()
    # offset-corrected consumption beside the raw figure
    text = _notes(sec)
    assert "offset-corrected" in text, text
    consumed = r.evidence["consumed_mah"]
    corrected = r.evidence["consumed_corrected_mah"]
    assert corrected == pytest.approx(consumed - 12.5 * w.duration / 3600.0 * 1000.0, rel=0.02)
    # a healthy sensor reads 0.0 and passes
    log = _motor_log("no_offset.bin", stopped_current=0.0)
    r = _results(check_power(log, airborne_window(log, method="ev")))["BAT0 current with motors stopped"]
    assert r.status == "PASS" and r.evidence["value"] == pytest.approx(0.0, abs=0.01)


def test_current_zero_offset_skips_when_the_motors_never_stopped():
    """A log opened at arming with the motors already turning has no such sample."""
    w = _writer()
    _params(w, SERVO1_MIN=1000, **SERVO_FN)
    for i in range(int(SPAN * HZ)):
        t = i / HZ
        us = int(t * 1e6)
        w.msg("RCOU", TimeUS=us, C1=1400, C2=1400, C3=1400, C4=1400)
        for inst in range(4):
            w.msg("ESC", TimeUS=us, Instance=inst, RPM=3400.0, Volt=25.0, Curr=5.0, Temp=30.0, Err=0.0)
        w.msg("BAT", TimeUS=us, Inst=0, Volt=25.0, Curr=20.0, CurrTot=t)
        w.msg("CTUN", TimeUS=us, ThO=0.4, ThH=0.4, BAlt=5.0)
    log = _load(w, "never_stopped.bin")
    rs = _results(check_power(log, airborne_window(log, method="ev")))
    r = rs["BAT0 current with motors stopped"]
    assert r.status == "SKIP" and "stopped" in r.summary


def test_current_is_banded_by_throttle():
    log = _motor_log("bands.bin", stopped_current=12.5)
    sec = check_power(log, airborne_window(log, method="ev"))
    t = _table(sec, "bat0_bands")
    rows = {r[0]: dict(zip(t["columns"], r)) for r in t["rows"]}
    assert {"idle", "hover", "full throttle"} <= set(rows), rows
    idle, hover, full = rows["idle"], rows["hover"], rows["full throttle"]
    # idle: armed, motors at spin-arm (duty 0.1 -> 6 A real) plus the 12.5 A offset
    assert idle["current (A)"] == pytest.approx(18.5, abs=0.3)
    assert idle["offset-corrected (A)"] == pytest.approx(6.0, abs=0.3)
    # hover: duty 0.4 -> 24 A real
    assert hover["current (A)"] == pytest.approx(36.5, abs=0.5)
    assert hover["offset-corrected (A)"] == pytest.approx(24.0, abs=0.5)
    assert hover["power (W)"] == pytest.approx(24.0 * PACK_V, rel=0.03)
    assert "0.36" in str(hover["ThO band"]) and "0.44" in str(hover["ThO band"])
    assert full["offset-corrected (A)"] == pytest.approx(57.0, abs=0.5)
    assert full["n"] > 50
    r = _results(sec)["BAT0 hover power"]
    assert r.status == "PASS" and r.evidence["watts"] == pytest.approx(24.0 * PACK_V, rel=0.03)
    assert "W" in r.summary


# ------------------------------------------------------------ issue #7

def test_motor_thresholds_are_registered():
    for key in ("drive_norm_spread_pct", "esc_temp_c", "esc_temp_spread_c"):
        assert key in T and T[key]["source"], key


def test_esc_table_carries_percentiles_and_temperature():
    log = _motor_log("esc_table.bin", temps=(35.0, 36.0, 35.0, 52.0))
    sec = check_motors(log, airborne_window(log, method="ev"))
    t = _table(sec, "esc")
    cols = t["columns"]
    for c in ("RPM p05", "RPM p95", "Temp mean", "Temp max", "RPM/(duty x V)", "motor"):
        assert c in cols, cols
    rows = {r[0]: dict(zip(cols, r)) for r in t["rows"]}
    assert set(rows) == {"ESC0", "ESC1", "ESC2", "ESC3"}
    # ESC0 is servo output 1 = motor 4 (SERVO1_FUNCTION=36), whose temp is 52 C
    assert rows["ESC0"]["motor"] == "M4"
    assert rows["ESC0"]["Temp max"] == pytest.approx(52.0, abs=0.1)
    assert rows["ESC1"]["Temp mean"] == pytest.approx(35.0, abs=0.1)
    # p05/p95 bracket the hover RPM (the burst pushes p95 up, spin-up pulls p05 down)
    assert rows["ESC1"]["RPM p05"] <= rows["ESC1"]["RPM median"] <= rows["ESC1"]["RPM p95"]
    rs = _results(sec)
    r = rs["ESC temperature"]
    assert r.status == "PASS" and r.evidence["value"] == pytest.approx(52.0, abs=0.1)
    r = rs["ESC temperature spread"]
    assert r.status == "WARN", r.summary                       # 17 C between ESC means
    assert r.evidence["value"] == pytest.approx(17.0, abs=0.2)
    assert "ESC0" in r.summary and "M4" in r.summary


def test_drive_normalised_rpm_separates_load_from_drag():
    """A CG offset loads the front pair harder: raw RPM spread says the motors disagree,
    RPM per unit of drive (RPM / (duty x V)) says they are all healthy. A dragging motor
    shows in the normalised figure and nowhere else."""
    # front pair (motors 1 and 3) +100 us: raw RPM spread ~ 23 %, all motors identical
    log = _motor_log("cg.bin", pwm_trim=(100, 0, 100, 0))
    sec = check_motors(log, airborne_window(log, method="ev"))
    rs = _results(sec)
    assert rs["RPM spread"].evidence["value"] > 15.0
    dn = rs["drive-normalised RPM spread"]
    assert dn.status == "PASS" and dn.evidence["value"] < 1.0, dn.summary
    per = dn.evidence["per_motor"]
    assert set(per) == {"M1", "M2", "M3", "M4"}
    assert all(v == pytest.approx(340.0, rel=0.02) for v in per.values()), per
    assert "load" in _notes(sec).lower() and "drag" in _notes(sec).lower()
    # motor 3 dragging: 7.5 % below its siblings at the same drive
    log = _motor_log("drag.bin", k=(340.0, 340.0, 315.0, 340.0))
    rs = _results(check_motors(log, airborne_window(log, method="ev")))
    dn = rs["drive-normalised RPM spread"]
    assert dn.status == "FAIL" and dn.evidence["value"] == pytest.approx(7.5, abs=0.5), dn.summary
    assert dn.evidence["per_motor"]["M3"] == pytest.approx(315.0, rel=0.02)
    assert "M3" in dn.summary


def test_drive_normalised_rpm_skips_without_a_pack_voltage():
    w = _writer()
    _params(w, **SERVO_FN)
    for i in range(int(SPAN * HZ)):
        t = i / HZ
        us = int(t * 1e6)
        up = FLIGHT[0] <= t <= FLIGHT[1]
        w.msg("RCOU", TimeUS=us, C1=1400 if up else 1000, C2=1400 if up else 1000,
              C3=1400 if up else 1000, C4=1400 if up else 1000)
        for inst in range(4):
            w.msg("ESC", TimeUS=us, Instance=inst, RPM=3400.0 if up else 0.0, Volt=0.0, Curr=0.0, Temp=30.0, Err=0.0)
    log = _load(w, "no_volt.bin")
    rs = _results(check_motors(log, airborne_window(log, method="ev")))
    assert rs["drive-normalised RPM spread"].status == "SKIP"
    assert "volt" in rs["drive-normalised RPM spread"].summary.lower()


if __name__ == "__main__":
    import _shim
    sys.exit(_shim.run(sys.modules[__name__]))
