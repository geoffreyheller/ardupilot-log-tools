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
from dflog.analysis import check_gps, check_motors, check_notch, check_power   # noqa: E402
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
               modes=(), hover_thh=0.40, extra_params=None, trim_fn=None, imu_hz=None):
    """Ground (0-5 s motors stopped, 5-10 s armed idle), flight 10-110 s at hover_pwm with a
    full-throttle burst over `full`, landing, then motors stopped again 110-120 s.

    `k[m-1]` is motor m's RPM per (duty x volt); `pwm_trim[m-1]` a standing offset in us on
    motor m (a CG offset loads a pair harder); `stopped_current` what BAT.Curr reads with
    every motor stopped (a zero offset, issue #6). Current follows total duty otherwise.
    `att` is a callable t -> (roll, pitch) degrees; `modes` [(t, mode_num)]; `trim_fn`
    t -> pwm trims overrides `pwm_trim` (a trim that only exists while translating);
    `imu_hz` adds a gyro stream at that rate so the coverage/Nyquist logic has a source.
    """
    w = _writer()
    _params(w, SERVO1_MIN=1000, SERVO1_MAX=2000, MOT_SPIN_MIN=0.15, MOT_SPIN_MAX=0.95, MOT_SPIN_ARM=0.10,
            MOT_THST_HOVER=hover_thh, **SERVO_FN, **(extra_params or {}))
    for t, num in modes:
        w.msg("MODE", TimeUS=int(t * 1e6), ModeNum=num, Rsn=1, ThrCrs=0)
    if imu_hz:
        for j in range(int(SPAN * imu_hz)):
            w.msg("IMU", TimeUS=int(j / imu_hz * 1e6), I=0, GyrX=0.01 * (j % 3), GyrY=0.0, GyrZ=0.0)
    tot_mah = 0.0
    for i in range(int(SPAN * HZ)):
        t = i / HZ
        us = int(t * 1e6)
        stopped = t < 5.0 or t > FLIGHT[1]
        idle = 5.0 <= t < FLIGHT[0]
        burst = full[0] <= t <= full[1]
        base = 1000 if stopped else (idle_pwm if idle else (1950 if burst else hover_pwm))
        trims = trim_fn(t) if trim_fn is not None else pwm_trim
        pwm = {}
        for m in range(1, 5):
            pwm[m] = base if stopped else base + trims[m - 1]
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


# ------------------------------------------------------------ issue #8

# Quad X: motor 1 front-right, 2 rear-left, 3 front-left, 4 rear-right.
FRONT_PLUS_60 = (60, 0, 60, 0)


def test_cg_offset_is_expressed_as_a_fraction_of_the_arm():
    """Thrust goes as RPM^2: with the front pair at duty 0.46 and the rear at 0.40 the
    thrust ratio is (0.46/0.40)^2 = 1.3225 and the CG sits (r-1)/(r+1) = 13.9 % of the
    fore-aft arm forward of centre. In millimetres when the arm is given."""
    log = _motor_log("cg_pct.bin", pwm_trim=FRONT_PLUS_60)
    w = airborne_window(log, method="ev")
    sec = check_motors(log, w)
    t = _table(sec, "cg")
    rows = {r[0]: dict(zip(t["columns"], r)) for r in t["rows"]}
    assert rows["pitch"]["CG offset (% of arm)"] == pytest.approx(13.9, abs=0.4), rows
    assert rows["roll"]["CG offset (% of arm)"] == pytest.approx(0.0, abs=0.3)
    assert rows["pitch"]["mm"] is None                      # no arm length given
    assert "forward" in rows["pitch"]["reads as"]
    rs = _results(sec)
    assert rs["pitch trim"].evidence["cg_offset_pct"] == pytest.approx(13.9, abs=0.4)
    sec = check_motors(log, w, arm_mm=151.0)
    t = _table(sec, "cg")
    rows = {r[0]: dict(zip(t["columns"], r)) for r in t["rows"]}
    assert rows["pitch"]["mm"] == pytest.approx(0.139 * 151.0, abs=0.8), rows
    assert "151" in _notes(sec)


def test_cg_offset_needs_esc_rpm():
    w = _writer()
    _params(w, **SERVO_FN)
    for i in range(int(SPAN * HZ)):
        j = i % 3
        w.msg("RCOU", TimeUS=int(i / HZ * 1e6), C1=1400 + j, C2=1460 + j, C3=1400 + j, C4=1460 + j)
    log = _load(w, "cg_norpm.bin")
    sec = check_motors(log, airborne_window(log, method="ev"))
    assert not any(t["name"] == "cg" for t in sec.tables)
    assert "ESC RPM" in _notes(sec)


def _tilt_att(t):
    """Level in the two LOITER chunks (40-60, 100-150 s), pitched 10 deg forward elsewhere."""
    level = 40.0 <= t <= 60.0 or 100.0 <= t <= 150.0
    return (0.0, 0.0) if level else (0.0, 10.0)


LOITER_MODES = ((5.0, 0), (40.0, 5), (60.0, 0), (100.0, 5), (110.0, 0))


def test_trim_is_cross_checked_against_level_hover():
    """A standing trim that is the same over the whole window and over level hover is a
    static asymmetry (CG, blade); one that vanishes when the aircraft stops translating
    is a translation artefact. The check reports both and flags a divergence."""
    static = _motor_log("trim_static.bin", pwm_trim=FRONT_PLUS_60, att=_tilt_att, modes=LOITER_MODES)
    sec = check_motors(static, airborne_window(static, method="ev"))
    rs = _results(sec)
    r = rs["trim vs level hover"]
    assert r.status == "PASS", r.summary
    assert r.evidence["window"]["pitch"] == pytest.approx(60.0, abs=1.0)
    assert r.evidence["hover"]["pitch"] == pytest.approx(60.0, abs=1.0)
    assert r.evidence["value"] < 2.0
    assert "static" in r.summary.lower()
    t = _table(sec, "trim_hover")
    assert [row[0] for row in t["rows"]] == ["roll", "pitch", "yaw"]
    # the same trim only while pitched forward: gone in level hover
    artefact = _motor_log("trim_artefact.bin", att=_tilt_att, modes=LOITER_MODES,
                          trim_fn=lambda t: (0, 0, 0, 0) if _tilt_att(t)[1] == 0.0 else FRONT_PLUS_60)
    rs = _results(check_motors(artefact, airborne_window(artefact, method="ev")))
    r = rs["trim vs level hover"]
    assert r.status == "FAIL", r.summary
    assert r.evidence["hover"]["pitch"] == pytest.approx(0.0, abs=1.0)
    assert r.evidence["window"]["pitch"] > 30.0
    assert "translation" in r.summary.lower()


def test_trim_cross_check_falls_back_to_level_samples_and_skips_without_them():
    # no hover chunk (no MODE), but level ATT samples exist: use them and say so
    log = _motor_log("trim_level.bin", pwm_trim=FRONT_PLUS_60, att=_tilt_att)
    rs = _results(check_motors(log, airborne_window(log, method="ev")))
    r = rs["trim vs level hover"]
    assert r.status == "PASS" and "level-attitude samples" in r.summary, r.summary
    # never level: nothing to cross-check against
    log = _motor_log("trim_never_level.bin", pwm_trim=FRONT_PLUS_60, att=lambda t: (0.0, 10.0))
    rs = _results(check_motors(log, airborne_window(log, method="ev")))
    assert rs["trim vs level hover"].status == "SKIP"
    # no ATT at all
    log = _motor_log("trim_noatt.bin", pwm_trim=FRONT_PLUS_60)
    rs = _results(check_motors(log, airborne_window(log, method="ev")))
    assert rs["trim vs level hover"].status == "SKIP" and "ATT" in rs["trim vs level hover"].summary


# ------------------------------------------------------------ issue #9

def test_notch_disabled_is_distinguished_from_not_logged():
    """Both used to be 'SKIP - FCNS not logged'. They call for opposite actions."""
    off = _motor_log("notch_off.bin", extra_params=dict(INS_HNTCH_ENABLE=0), imu_hz=25.0)
    rs = _results(check_notch(off, airborne_window(off, method="ev")))
    assert rs["notch"].status == "SKIP" and "DISABLED" in rs["notch"].summary, rs["notch"].summary
    on = _motor_log("notch_on_nolog.bin", imu_hz=25.0,
                    extra_params=dict(INS_HNTCH_ENABLE=1, INS_HNTCH_MODE=3, INS_HNTCH_FREQ=80, INS_HNTCH_BW=40))
    rs = _results(check_notch(on, airborne_window(on, method="ev")))
    assert rs["notch"].status == "SKIP"
    assert "ENABLED" in rs["notch"].summary and "not logged" in rs["notch"].summary, rs["notch"].summary
    assert "DISABLED" not in rs["notch"].summary
    unknown = _motor_log("notch_unknown.bin", imu_hz=25.0)
    rs = _results(check_notch(unknown, airborne_window(unknown, method="ev")))
    assert "INS_HNTCH_ENABLE" in rs["notch"].summary and "not in the log" in rs["notch"].summary


def test_notch_recommendation_when_disabled():
    """The check already holds the fundamental envelope; when the notch is off it should
    say what the envelope is and suggest a starting point, instead of discarding it."""
    log = _motor_log("notch_rec.bin", extra_params=dict(INS_HNTCH_ENABLE=0), imu_hz=25.0,
                     pwm_trim=FRONT_PLUS_60)
    sec = check_notch(log, airborne_window(log, method="ev"))
    env = {r[0]: r[1] for r in _table(sec, "fundamental")["rows"]}
    # hover: motors at duty 0.40/0.46 x 25 V x 340 -> 3400 / 3910 RPM, fleet mean ~ 61 Hz
    assert env["median"] == pytest.approx(60.9, abs=1.5), env
    assert env["p01"] == pytest.approx(60.9, abs=2.0)
    assert env["max"] > 120.0                                   # the full-throttle burst
    assert env["min"] <= env["p01"] <= env["median"] <= env["p99"] <= env["max"]
    rec = {r[0]: r[1] for r in _table(sec, "recommendation")["rows"]}
    assert rec["INS_HNTCH_MODE"] == 3
    assert rec["INS_HNTCH_REF"] == 1
    assert rec["INS_HNTCH_FREQ"] == 55                          # 0.95 x p01, rounded down to 5 Hz
    assert rec["INS_HNTCH_BW"] == 27                            # FREQ / 2
    assert rec["INS_HNTCH_HMNCS"] == 3
    assert rec["INS_HNTCH_ATT"] == 40
    assert rec["INS_HNTCH_OPTS"] == 2                           # per-motor spread > 5 %
    per = {r[0]: r for r in _table(sec, "per_motor")["rows"]}
    assert set(per) == {"ESC0", "ESC1", "ESC2", "ESC3"}
    notes = _notes(sec)
    assert "4/4 motors" in notes and "Nyquist 12.5 Hz" in notes and "batchfft" in notes, notes
    assert "INS_LOG_BAT_MASK=1" in notes
    # motors within 5 %: no per-motor notches suggested
    log = _motor_log("notch_rec_even.bin", extra_params=dict(INS_HNTCH_ENABLE=0), imu_hz=25.0)
    rec = {r[0]: r[1] for r in _table(check_notch(log, airborne_window(log, method="ev")), "recommendation")["rows"]}
    assert rec["INS_HNTCH_OPTS"] == 0


def test_notch_recommendation_needs_esc_telemetry():
    log = _load(standard_log(n=200), "notch_noesc.bin")
    sec = check_notch(log, airborne_window(log, method="none"))
    assert _results(sec)["notch"].status == "SKIP"
    assert not any(t["name"] == "recommendation" for t in sec.tables)


# ------------------------------------------------------------ issue #10 (item 2)

def test_missing_column_names_the_near_miss():
    """`gg["HDOP"]` -> KeyError: 'HDOP' cost a turn; the column is `HDop`."""
    log = _gps_log("alias.bin")
    d = log.df("GPS")
    for bad, want in (("HDOP", "HDop"), ("hdop", "HDop"), ("NSat", "NSats"), ("Lon", "Lng")):
        try:
            d[bad]
        except KeyError as exc:
            assert want in str(exc) and bad in str(exc) and "GPS" in str(exc), (bad, str(exc))
        else:
            raise AssertionError(f"{bad!r} must raise")
    # and through the paths a check actually uses: window clip and instance split
    w = airborne_window(log, method="ev")
    try:
        w.clip(d)["HDOP"]
    except KeyError as exc:
        assert "HDop" in str(exc), str(exc)
    try:
        log.instances("GPS")[0]["HDOP"]
    except KeyError as exc:
        assert "HDop" in str(exc), str(exc)
    # a column this board simply does not log: say so, and list what exists
    try:
        d["Clip0"]
    except KeyError as exc:
        assert "Clip0" in str(exc) and "HDop" in str(exc) and "not logged" in str(exc).lower(), str(exc)
    # the happy path is untouched
    assert d["HDop"].iloc[0] == pytest.approx(0.8, abs=0.01)
    assert log.column("GPS", "HDOP") == "HDop" and log.column("GPS", "Clip0") is None


if __name__ == "__main__":
    import _shim
    sys.exit(_shim.run(sys.modules[__name__]))
