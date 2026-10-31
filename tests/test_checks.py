"""Check-level tests on synthetic logs: the September-2026 issues (#2, #5-#10).

Each test builds exactly the log its check needs with tests/synthlog.py, so it runs
anywhere with no flight data, then asserts on the Section the check returns. The real-log
counterparts are in tests/test_largeprop.py.

    python tests/test_checks.py
"""

import math
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
from dflog.flight import Window                                          # noqa: E402
from dflog.analysis import check_gps, check_motors, check_notch, check_power   # noqa: E402
from dflog.analysis import check_vibration                                      # noqa: E402
from dflog.analysis import _Params, _hover_throttle, _throttle_at               # noqa: E402
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
    w.fmt(200, "GPS", "QBBBfLLfIHff", "TimeUS,I,Status,NSats,HDop,Lat,Lng,Alt,GMS,GWk,Spd,GCrs")
    w.fmt(201, "GPA", "QBCfffH", "TimeUS,I,VDop,HAcc,VAcc,SAcc,Delta")
    w.fmt(202, "ESC", "QBfffff", "TimeUS,Instance,RPM,Volt,Curr,Temp,Err")
    w.fmt(203, "RCOU", "QHHHH", "TimeUS,C1,C2,C3,C4")
    w.fmt(204, "BAT", "QBfff", "TimeUS,Inst,Volt,Curr,CurrTot")
    w.fmt(205, "CTUN", "Qfff", "TimeUS,ThO,ThH,BAlt")
    w.fmt(206, "ATT", "Qfffff", "TimeUS,DesRoll,Roll,DesPitch,Pitch,Yaw")
    w.fmt(207, "MODE", "QBBB", "TimeUS,ModeNum,Rsn,ThrCrs")
    w.fmt(208, "RCIN", "QHHHH", "TimeUS,C1,C2,C3,C4")
    w.fmt(209, "IMU", "QBfff", "TimeUS,I,GyrX,GyrY,GyrZ")
    w.fmt(210, "FCN", "QBB" + "f" * 12, "TimeUS,I,NF," + ",".join(f"CF{k}" for k in range(1, 7)) + ","
          + ",".join(f"HF{k}" for k in range(1, 7)))
    w.fmt(211, "VIBE", "QBfffI", "TimeUS,IMU,VibeX,VibeY,VibeZ,Clip")
    w.fmt(212, "XKF1", "QBff", "TimeUS,C,VN,VE")
    w.fmt(213, "XKF4", "QBB", "TimeUS,C,PI")
    w.fmt(214, "UBX2", "QBbBbB", "TimeUS,Instance,ofsI,magI,ofsQ,magQ")
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
               modes=(), hover_thh=0.40, extra_params=None, trim_fn=None, imu_hz=None, fcn=None,
               end_drop_v=None, vibe=None, vibe_imus=1):
    """Ground (0-5 s motors stopped, 5-10 s armed idle), flight 10-110 s at hover_pwm with a
    full-throttle burst over `full`, landing, then motors stopped again 110-120 s.

    `k[m-1]` is motor m's RPM per (duty x volt); `pwm_trim[m-1]` a standing offset in us on
    motor m (a CG offset loads a pair harder); `stopped_current` what BAT.Curr reads with
    every motor stopped (a zero offset, issue #6). Current follows total duty otherwise.
    `att` is a callable t -> (roll, pitch) degrees; `modes` [(t, mode_num)]; `trim_fn`
    t -> pwm trims overrides `pwm_trim` (a trim that only exists while translating);
    `imu_hz` adds a gyro stream at that rate so the coverage/Nyquist logic has a source.
    `fcn` = {esc_instance: centre / RPM-over-60 factor} writes a per-motor notch (FCN, no
    FCNS) as ArduCopter does with INS_HNTCH_OPTS bit 1: CFk follows ESC k-1, NaN while stopped.
    `vibe` is a callable t -> (VibeX, VibeY, VibeZ, cumulative Clip) written for `vibe_imus` IMUs.
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
        if vibe is not None:
            x, y, z, clip = vibe(t)
            for imu in range(vibe_imus):
                w.msg("VIBE", TimeUS=us, IMU=imu, VibeX=x, VibeY=y, VibeZ=z, Clip=clip)
        if att is not None:
            a = att(t)
            r, p, yw = (a[0], a[1], a[2] if len(a) > 2 else 0.0)
            w.msg("ATT", TimeUS=us, DesRoll=r, Roll=r, DesPitch=p, Pitch=p, Yaw=yw)
        if modes:
            w.msg("RCIN", TimeUS=us, C1=1500, C2=1500, C3=1500, C4=1500)
        if fcn is not None:
            cf = {k: (float("nan") if k <= 4 else 0.0) for k in range(1, 7)}    # Brisket: unused slots read 0
            for m in range(1, 5):
                inst = CHAN_OF_MOTOR[m] - 1
                if not stopped:
                    cf[inst + 1] = rpm[m] / 60.0 * fcn.get(inst, 1.0)
            w.msg("FCN", TimeUS=us, I=0, NF=8, **{f"CF{k}": cf[k] for k in cf},
                  **{f"HF{k}": (2 * cf[k] if k <= 4 else float("nan")) for k in cf})
    if end_drop_v is not None:
        w.msg("BAT", TimeUS=int(SPAN * 1e6), Inst=0, Volt=end_drop_v, Curr=0.0, CurrTot=tot_mah)
    return _load(w, name)


def _gps_log(name, hacc0=0.8, vacc0=1.2, sacc0=0.3, spikes=(), second=True, vel=(0.0, 0.0),
             ekf=(0.0, 0.0), ekf1=None, pi=None, params=None, ubx2=None, nofix=(), curr=None, tho=None, fix1=()):
    """Two receivers: instance 0 with a 3D fix (accuracies as given, with `spikes` =
    [(t0, t1, hacc)] where HAcc jumps), instance 1 an NMEA unit that never gets a fix
    and reports HAcc 0 and the saturated VDop 655.35.

    `vel` is GPS0's (Spd, GCrs), `ekf` the XKF1 core 0 (VN, VE) written at 10 Hz (False
    omits XKF1), `ekf1` a second core 1 when given, `pi` an XKF4.PI written for both
    cores when given. Each of those takes a constant tuple or a callable of t.
    `nofix=[(t0, t1)]` drops GPS0 to Status 1 in those spans.
    `fix1=[(t0, t1)]` gives GPS1 Status 3, 8 sats and a position 3.0 m east of GPS0 in those spans.
    `curr` is a callable t -> A written as BAT instance 0 at 10 Hz (Volt 25.0).
    `tho` is a callable t -> CTUN.ThO written at 10 Hz (ThH 0.4).
    `params` are written as PARM records. `ubx2` is a callable of t returning (magI, magQ)
    or (ofsI, magI, ofsQ, magQ), or None for no record; written as UBX2 instance 0 every
    5 s from t = 0.
    """
    def at(v, t):
        return v(t) if callable(v) else v

    w = _writer()
    _params(w, **(params or {}))
    for i in range(int(SPAN * HZ)):
        t = i / HZ
        us = int(t * 1e6)
        h = next((h for a, b, h in spikes if a <= t <= b), hacc0)
        spd, crs = at(vel, t)
        status = 1 if any(a <= t <= b for a, b in nofix) else 3
        w.msg("GPS", TimeUS=us, I=0, Status=status, NSats=12, HDop=0.8, Lat=30.0 + t * 1e-6, Lng=-140.0,
              Alt=100.0, GMS=int(t * 1000), GWk=2400, Spd=spd, GCrs=crs)
        w.msg("GPA", TimeUS=us, I=0, VDop=1.1, HAcc=h, VAcc=vacc0, SAcc=sacc0, Delta=200)
        if curr is not None:
            w.msg("BAT", TimeUS=us, Inst=0, Volt=25.0, Curr=curr(t), CurrTot=0.0)
        if tho is not None:
            w.msg("CTUN", TimeUS=us, ThO=tho(t), ThH=0.4, BAlt=5.0)
        if ubx2 is not None and i % int(5 * HZ) == 0:
            u = ubx2(t)
            if u is not None:
                ofs_i, mag_i, ofs_q, mag_q = (0, *u[:1], 0, *u[1:]) if len(u) == 2 else u
                w.msg("UBX2", TimeUS=us, Instance=0, ofsI=ofs_i, magI=mag_i, ofsQ=ofs_q, magQ=mag_q)
        if second and any(a <= t <= b for a, b in fix1):
            w.msg("GPS", TimeUS=us, I=1, Status=3, NSats=8, HDop=1.0, Lat=30.0 + t * 1e-6,
                  Lng=-140.0 + 3.0 / (111320.0 * math.cos(math.radians(30.0 + t * 1e-6))), Alt=100.0,
                  GMS=int(t * 1000), GWk=2400)
        elif second:
            w.msg("GPS", TimeUS=us, I=1, Status=1, NSats=0, HDop=99.99, Lat=0.0, Lng=0.0, Alt=0.0,
                  GMS=0, GWk=0)
            w.msg("GPA", TimeUS=us, I=1, VDop=655.35, HAcc=0.0, VAcc=0.0, SAcc=0.0, Delta=99)
        for core, e in ((0, ekf), (1, ekf1)):
            if e is False or e is None:
                continue
            vn, ve = at(e, t)
            w.msg("XKF1", TimeUS=us, C=core, VN=vn, VE=ve)
            if pi is not None:
                w.msg("XKF4", TimeUS=us, C=core, PI=pi)
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


# ------------------------------------------------------------ issue #14

def _vel_flight(**kw):
    log = _gps_log("vel.bin", **kw)
    return check_gps(log, airborne_window(log, method="ev"))


def test_gps_velocity_agrees_with_ekf():
    rs = _results(_vel_flight())
    r = rs["GPS0 velocity vs EKF"]
    assert r.status == "PASS" and r.evidence["n_over"] == 0 and r.evidence["core"] == 0, r
    assert rs["GPS1 velocity vs EKF"].status == "SKIP"
    assert "no 3D fix" in rs["GPS1 velocity vs EKF"].summary


def test_gps_velocity_glitches_after_settle_are_graded():
    def vel(end):
        return lambda t: (0.8, 0.0) if 30.0 <= t <= 30.95 else (
            (1.2, 0.0) if 80.0 <= t <= end else (0.0, 0.0))
    r = _results(_vel_flight(vel=vel(82.95)))["GPS0 velocity vs EKF"]
    ev = r.evidence
    assert ev["n_over"] == 40 and ev["n_late"] == 30, ev
    assert ev["max"] == pytest.approx(1.2, abs=1e-3)
    assert ev["t_max"] == pytest.approx(80.0, abs=0.25)
    assert ev["worst_bin"] == [70.0, 100.0], ev
    assert ev["value"] == pytest.approx(10.0, abs=0.5)
    assert r.status == "WARN", r
    assert _results(_vel_flight(vel=vel(86.95)))["GPS0 velocity vs EKF"].status == "FAIL"


def test_gps_velocity_uses_the_primary_ekf_core():
    kw = dict(vel=(2.0, 0.0), ekf=(0.0, 0.0), ekf1=(2.0, 0.0))
    r = _results(_vel_flight(pi=1, **kw))["GPS0 velocity vs EKF"]
    assert r.evidence["n_over"] == 0 and r.evidence["core"] == 1, r
    r = _results(_vel_flight(**kw))["GPS0 velocity vs EKF"]
    assert r.evidence["core"] == 0 and r.evidence["n_over"] > 0, r
    assert "no XKF4.PI" in r.summary


def test_gps_velocity_course_is_degrees_from_north():
    for crs, ve in ((90.0, 5.0), (270.0, -5.0)):
        r = _results(_vel_flight(vel=(5.0, crs), ekf=(0.0, ve)))["GPS0 velocity vs EKF"]
        assert r.evidence["n_over"] == 0, (crs, r)


def test_gps_velocity_compensates_receiver_lag():
    def ramp(t):
        return 3.0 * min(max(t - 20.0, 0.0), 5.0)
    for delay, params in ((0.2, None), (0.1, {"GPS1_DELAY_MS": 100})):
        r = _results(_vel_flight(vel=lambda t: (ramp(t - delay), 0.0), ekf=lambda t: (ramp(t), 0.0),
                                 params=params))["GPS0 velocity vs EKF"]
        assert r.evidence["n_over"] == 0 and r.evidence["max"] < 0.05, r
        assert r.evidence["lag_s"] == pytest.approx(delay)
        assert ("assumed" in r.summary) == (params is None), r.summary


def test_gps_velocity_skips_without_xkf1():
    r = _results(_vel_flight(ekf=False))["GPS0 velocity vs EKF"]
    assert r.status == "SKIP" and "XKF1" in r.summary, r


def test_gps_vel_threshold_is_registered_with_a_row():
    assert T["gps_vel_glitch_pct"]["warn"] == 5.0 and T["gps_vel_glitch_pct"]["fail"] == 15.0
    assert T["gps_vel_glitch_pct"]["source"]
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "reference", "thresholds.md")
    with open(path, encoding="utf-8") as f:
        assert any(l.startswith("| `gps_vel_glitch_pct` |") for l in f), "no row in thresholds.md"


def _ubx2_flight(ubx2):
    log = _gps_log("ubx2.bin", ubx2=ubx2)
    return check_gps(log, airborne_window(log, method="ev"))


def test_ubx2_collapse_airborne_is_flagged():
    sec = _ubx2_flight(lambda t: (0, 0) if 40.0 <= t <= 90.0 else (255, 255))
    row = dict(zip(_table(sec, "ubx2")["columns"], next(r for r in _table(sec, "ubx2")["rows"] if r[0] == "GPS0")))
    assert row["before take-off med"] == 255 and row["after landing med"] == 255
    assert row["airborne med"] < 255 and row["min(magI,magQ)"] == 0
    assert row["at (s)"] == pytest.approx(40.0, abs=0.1)
    r = _results(sec)["GPS0 RF front end (UBX2)"]
    assert r.status == "FAIL" and r.evidence["value"] == pytest.approx(52.4, abs=1.0), r


def test_ubx2_steady_passes():
    r = _results(_ubx2_flight(lambda t: (255, 255)))["GPS0 RF front end (UBX2)"]
    assert r.status == "PASS" and r.evidence["value"] == 0, r


def test_ubx2_without_ground_reference_is_skip():
    sec = _ubx2_flight(lambda t: (255, 255) if 10.0 <= t <= 110.0 else None)
    r = _results(sec)["GPS0 RF front end (UBX2)"]
    assert r.status == "SKIP" and "no on-ground UBX2" in r.summary, r
    assert any(row[0] == "GPS0" for row in _table(sec, "ubx2")["rows"])


def test_ubx2_identity_note_is_kept():
    sec = _ubx2_flight(lambda t: (255, 255))
    assert "emitted only by the u-blox driver" in _notes(sec)


# ------------------------------------------------------------ issue #14: bins and take-off

def _rows(sec, name):
    t = _table(sec, name)
    return [dict(zip(t["columns"], r)) for r in t["rows"]]


def test_gpa_table_has_hacc_per_hdop():
    log = _gps_log("hacc_hdop.bin")
    sec = check_gps(log, airborne_window(log, method="ev"))
    t = _table(sec, "gpa")
    assert t["columns"][-1] == "HAcc/HDOP med"
    rows = {r["gps"]: r for r in _rows(sec, "gpa")}
    assert rows["GPS0"]["HAcc/HDOP med"] == pytest.approx(1.0, abs=0.01)
    assert rows["GPS1"]["HAcc/HDOP med"] is None


def _bins_flight():
    def vel(t):
        return (1.2, 0.0) if 80.0 <= t <= 82.95 else (0.0, 0.0)

    def ubx2(t):
        if t < 10.0 or t > 110.0:
            return (255, 255)
        return (0, 0) if 80.0 <= t <= 85.0 else None
    log = _gps_log("bins.bin", spikes=[(50.0, 52.0, 3.0)], vel=vel, ubx2=ubx2)
    return check_gps(log, airborne_window(log, method="ev"))


def test_gps_bins_are_timed_from_takeoff():
    rows = _rows(_bins_flight(), "gps0_bins")
    assert [r["from (s)"] for r in rows] == [0, 30, 60, 90]
    assert [r["vel glitches"] for r in rows] == [0, 0, 30, 0]
    assert all(r["sats min"] == 12 for r in rows)
    assert rows[1]["HAcc med (m)"] >= 0.8
    assert all(r["HAcc med (m)"] == pytest.approx(0.8, abs=1e-6) for i, r in enumerate(rows) if i != 1)
    assert [r["UBX2 low %"] for r in rows] == [None, None, 100.0, None]


def test_gps_bins_exclude_samples_without_a_fix():
    """GPA has no status of its own: its samples during a no-fix span (here 2/3 of the
    30-60 s bin, HAcc 9.0) must be dropped using GPS.Status, or the bin median reads 9.0."""
    log = _gps_log("bins_nofix.bin", spikes=[(40.0, 59.95, 9.0)], nofix=[(40.0, 59.95)])
    sec = check_gps(log, airborne_window(log, method="ev"))
    row = _rows(sec, "gps0_bins")[1]
    assert row["from (s)"] == 30
    assert row["HAcc med (m)"] == pytest.approx(0.8, abs=1e-6), row


def _power_flight(curr, tho=None):
    def vel(t):
        return (1.2, 0.0) if 80.0 <= t <= 82.95 else (0.0, 0.0)
    log = _gps_log("power.bin", spikes=[(80.0, 82.95, 3.0)], vel=vel, curr=curr, tho=tho)
    return check_gps(log, airborne_window(log, method="ev"))


def test_gps_accuracy_is_split_by_current():
    sec = _power_flight(lambda t: 30.0 if 80.0 <= t <= 82.95 else 10.0)
    rows = {r["gps"]: r for r in _rows(sec, "gps_vs_power")}
    r = rows["GPS0"]
    assert r["split on"] == "BAT0.Curr (A)"
    assert r["split at"] == pytest.approx(10.0)
    assert r["HAcc med high (m)"] == pytest.approx(3.0)
    assert r["HAcc med low (m)"] == pytest.approx(0.8)
    assert r["vel glitches high"] == 30 and r["vel glitches low"] == 0
    assert "GPS1" not in rows


def test_gps_vs_power_flat_current_is_not_split():
    sec = _power_flight(lambda t: 10.0)
    assert not any(t["name"] == "gps_vs_power" for t in sec.tables)
    assert "never rose meaningfully above its median" in _notes(sec), _notes(sec)


def test_gps_vs_power_noisy_flat_current_is_not_split():
    sec = _power_flight(lambda t: 10.0 + 0.1 * math.sin(7.0 * t))
    assert not any(t["name"] == "gps_vs_power" for t in sec.tables)
    assert "never rose meaningfully above its median" in _notes(sec), _notes(sec)


def test_gps_vs_power_sparse_punches_on_noisy_hover_are_split():
    sec = _power_flight(lambda t: 40.0 if (t % 10.0) < 1.0 else 10.0 + 0.5 * math.sin(7.0 * t))
    rows = {r["gps"]: r for r in _rows(sec, "gps_vs_power")} if any(
        t["name"] == "gps_vs_power" for t in sec.tables) else {}
    assert "GPS0" in rows, _notes(sec)


def test_gps_vs_power_falls_back_to_throttle():
    sec = _power_flight(None, tho=lambda t: 0.8 if 80.0 <= t <= 82.95 else 0.4)
    r = {r["gps"]: r for r in _rows(sec, "gps_vs_power")}["GPS0"]
    assert r["split on"] == "CTUN.ThO"
    assert r["vel glitches high"] == 30 and r["vel glitches low"] == 0


def test_gps_vs_power_needs_a_source():
    sec = _power_flight(None)
    assert not any(t["name"] == "gps_vs_power" for t in sec.tables)
    assert "needs BAT.Curr or CTUN.ThO" in _notes(sec), _notes(sec)


def test_gps_takeoff_conditions():
    rows = {r["gps"]: r for r in _rows(_bins_flight(), "gps_takeoff")}
    g0, g1 = rows["GPS0"], rows["GPS1"]
    assert g0["take-off (s from log start)"] == pytest.approx(10.0, abs=0.2)
    assert g0["status"] == 3 and g0["sats"] == 12 and g0["HAcc (m)"] == pytest.approx(0.8)
    assert g1["status"] == 1 and g1["HAcc (m)"] is None


def test_gps_takeoff_follows_the_analysed_flight():
    log = _gps_log("takeoff_window.bin")
    sec = check_gps(log, Window(40.0, 110.0, "test"))
    rows = {r["gps"]: r for r in _rows(sec, "gps_takeoff")}
    assert rows["GPS0"]["take-off (s from log start)"] == pytest.approx(40.0, abs=0.2)
    assert _rows(sec, "gps0_bins")[0]["from (s)"] == 0
    assert "timed from take-off (40.0 s since boot)" in _notes(sec), _notes(sec)


def test_gps_takeoff_not_reported_without_takeoff():
    log = _gps_log("no_takeoff.bin")
    sec = check_gps(log, airborne_window(log, method="none"))
    assert not any(t["name"] == "gps_takeoff" for t in sec.tables)
    assert "No take-off in this window" in _notes(sec)
    assert any(t["name"] == "gps0_bins" for t in sec.tables)


def test_fix_timeline_secondary_receiver():
    log = _gps_log("fixtl.bin", fix1=[(25.0, 60.0), (65.0, 120.0)])
    sec = check_gps(log, airborne_window(log, method="ev"))
    rows = {r["gps"]: r for r in _rows(sec, "gps_fix")}
    g1, g0 = rows["GPS1"], rows["GPS0"]
    assert g1["first 3D fix (s from log start)"] == pytest.approx(25.0, abs=0.2)
    assert g1["vs take-off (s)"] == pytest.approx(15.0, abs=0.2)
    assert g1["outages"] == 1
    assert g1["longest (s)"] == pytest.approx(5.0, abs=0.2)
    assert g1["dist to GPS0 med (m)"] == pytest.approx(3.0, abs=0.1)
    r = _results(sec)["GPS1 first 3D fix"]
    assert r.status == "WARN" and "after take-off" in r.summary, r.summary
    assert g0["first 3D fix (s from log start)"] == pytest.approx(0.0, abs=0.2)
    assert g0["dist to GPS0 med (m)"] is None
    assert _results(sec)["GPS0 first 3D fix"].status == "PASS"


def test_fix_timeline_never_fixed():
    log = _gps_log("fixtl_never.bin")
    sec = check_gps(log, airborne_window(log, method="ev"))
    r = _results(sec)["GPS1 first 3D fix"]
    assert r.status == "WARN" and "never" in r.summary, r.summary
    row = {r["gps"]: r for r in _rows(sec, "gps_fix")}["GPS1"]
    assert row["dist to GPS0 med (m)"] is None and row["dist p95 (m)"] is None


def test_fix_timeline_outage_open_at_log_end():
    log = _gps_log("fixtl_open.bin", fix1=[(5.0, 100.0)])
    sec = check_gps(log, airborne_window(log, method="ev"))
    row = {r["gps"]: r for r in _rows(sec, "gps_fix")}["GPS1"]
    assert row["outages"] == 1
    assert row["longest (s)"] == pytest.approx(20.0, abs=0.2)


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
    # equal in level hover is a CG/airframe asymmetry OR a steady breeze, not proof of CG
    assert "cg/airframe asymmetry or a steady breeze" in r.summary.lower() and "trim vs heading" in r.summary
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


def _heading_trims(cg_pitch, cg_roll, wind_us, wind_bearing):
    """t -> per-motor trims (M1 FR, M2 RL, M3 FL, M4 RR) for a CG trim fixed to the airframe
    plus a breeze load fixed to the earth (heavier on the side facing `wind_bearing`), while
    the aircraft yaws through N, E, S, W in 25 s steps (`_yaw_steps`)."""
    def fn(t):
        psi = np.radians(_yaw_steps(t))
        wn, we = wind_us * np.cos(np.radians(wind_bearing)), wind_us * np.sin(np.radians(wind_bearing))
        dx = cg_pitch + wn * np.cos(psi) + we * np.sin(psi)          # front heavier
        dy = -cg_roll - wn * np.sin(psi) + we * np.cos(psi)          # right heavier
        pitch, roll = dx, -dy
        return tuple(pitch / 2 * f + roll / 2 * l for f, l in ((1, -1), (-1, 1), (1, 1), (-1, -1)))
    return fn


def _yaw_steps(t):
    return 0.0 if t < 35 else 90.0 if t < 60 else 180.0 if t < 85 else 270.0


def test_trim_vs_heading_separates_cg_from_a_breeze():
    """2026-09-30 Brisket, battery untouched: pitch/roll trim 20.6/12.7 us facing 178 deg,
    35.2/0.2 us facing 159 deg four hours later in a light breeze - and 'trim vs level hover'
    called both a static asymmetry. Only the heading dependence separates the two."""
    from dflog.analysis import check_motors
    log = _motor_log("trim_heading.bin", modes=((5.0, 2),), trim_fn=_heading_trims(20.0, -6.0, 15.0, 90.0),
                     att=lambda t: (0.0, 0.0, _yaw_steps(t)))
    sec = check_motors(log, airborne_window(log, method="ev"))
    r = _results(sec)["trim vs heading"]
    assert r.status == "WARN", r.summary                              # 20 us airframe trim: warn 10
    assert r.evidence["airframe"]["pitch"] == pytest.approx(20.0, abs=0.5)
    assert r.evidence["airframe"]["roll"] == pytest.approx(-6.0, abs=0.5)
    assert r.evidence["earth"]["magnitude"] == pytest.approx(15.0, abs=0.5)
    assert r.evidence["earth"]["bearing_deg"] == pytest.approx(90.0, abs=2.0)
    assert r.evidence["bins"] == 4 and r.evidence["residual_us"] < 0.5
    assert "heavier on the side facing 90 deg" in r.summary
    assert len(_table(sec, "trim_heading")["rows"]) == 4
    # 'trim vs level hover' no longer calls it static
    lv = _results(sec)["trim vs level hover"]
    assert "static asymmetry" not in lv.summary
    # one heading: cannot separate, says so, and does not pretend
    one = _motor_log("trim_heading_one.bin", modes=((5.0, 2),), pwm_trim=FRONT_PLUS_60, att=lambda t: (0.0, 0.0, 170.0))
    r = _results(check_motors(one, airborne_window(one, method="ev")))["trim vs heading"]
    assert r.status == "SKIP" and "cannot be told apart" in r.summary and "four headings" in r.summary, r.summary


def test_battery_at_log_end_ignores_the_power_off_sample():
    """Unplugging the pack drops the last BAT record: 22.02 V after a 26.0 V landing on
    brisket-t2.bin read as a WARN."""
    from dflog.analysis import check_brownout
    log = _motor_log("bat_end.bin", end_drop_v=0.7 * PACK_V)
    r = _results(check_brownout(log, airborne_window(log, method="ev")))["battery at log end"]
    assert r.status == "PASS", r.summary
    assert r.evidence["final_v"] == pytest.approx(0.7 * PACK_V, abs=0.01)
    assert r.evidence["end_v"] == pytest.approx(PACK_V - 0.002 * SPAN, abs=0.05)
    assert "median of the last 5 s" in r.summary


def test_per_motor_notch_is_read_from_fcn_against_each_esc():
    """INS_HNTCH_OPTS bit 1 logs FCN.CF1..CFn and no FCNS. The check used to SKIP a working
    notch as 'not logged' (brisket-t1.bin, Brisket: CFk matched ESC k-1 to 0.15 %)."""
    notch = dict(INS_HNTCH_ENABLE=1, INS_HNTCH_MODE=3, INS_HNTCH_FREQ=40, INS_HNTCH_BW=20, INS_HNTCH_OPTS=2)
    log = _motor_log("notch_fcn.bin", imu_hz=25.0, extra_params=notch, pwm_trim=FRONT_PLUS_60, fcn={})
    sec = check_notch(log, airborne_window(log, method="ev"))
    rs = _results(sec)
    assert "notch" not in rs, rs["notch"].summary                     # not a SKIP any more
    tr = rs["notch 0 tracking"]
    assert tr.status == "PASS" and tr.evidence["n_centres"] == 4 and tr.evidence["source_msg"] == "FCN", tr.summary
    assert "worst of 4 per-motor centres" in tr.summary and rs["notch 0 harmonic lock-on"].status == "PASS"
    rows = {r[0]: r for r in _table(sec, "tracking")["rows"]}
    assert "notch 0 CF1 CF / ESC0 RPM/60" in rows and rows["notch 0 CF1 CF / ESC0 RPM/60"][3] == pytest.approx(1.0, abs=1e-6)
    assert "FCNS is not written in this mode" in _notes(sec)
    # each centre is judged against its own ESC: one motor's notch 30 % high fails on its own
    bad = _motor_log("notch_fcn_bad.bin", imu_hz=25.0, extra_params=notch, pwm_trim=FRONT_PLUS_60, fcn={2: 1.3})
    tr = _results(check_notch(bad, airborne_window(bad, method="ev")))["notch 0 tracking"]
    assert tr.status == "FAIL" and "CF3" in tr.summary and "ESC2" in tr.summary, tr.summary


def test_firmware_rewrites_are_not_an_in_flight_tune_change():
    """AP_Stats re-saves STAT_* every ~30 s and MOT_THST_HOVER is saved on disarm: 28
    'in-flight changes' on brisket-t1.bin, none of them a tune."""
    from dflog.analysis import check_params, firmware_maintained
    w = _writer()
    _params(w, STAT_RUNTIME=100, STAT_FLTTIME=10, MOT_THST_HOVER=0.24, ATC_RAT_RLL_P=0.135, BARO1_GND_PRESS=88000)
    for t in (40.0, 71.0, 102.0):
        w.msg("PARM", TimeUS=int(t * 1e6), Name="STAT_RUNTIME", Value=100 + t, Default=float("nan"))
        w.msg("PARM", TimeUS=int(t * 1e6), Name="STAT_FLTTIME", Value=10 + t, Default=float("nan"))
    w.msg("PARM", TimeUS=int(FLIGHT[1] * 1e6) - 1000, Name="MOT_THST_HOVER", Value=0.25, Default=float("nan"))
    log = _load(w, "stat_rewrites.bin")
    r = _results(check_params(log, airborne_window(log, method="ev")))["parameter changes"]
    assert r.status == "PASS" and r.evidence["n_inflight"] == 0 and r.evidence["n_firmware"] == 7, r.summary
    assert "7 rewrite(s) the firmware makes itself" in r.summary
    kinds = {row[1]: row[4] for row in _table(check_params(log, airborne_window(log, method="ev")), "changes")["rows"]}
    assert kinds == {"STAT_RUNTIME": "firmware", "STAT_FLTTIME": "firmware", "MOT_THST_HOVER": "firmware"}
    # a real gain change in flight still warns, and is named
    w.msg("PARM", TimeUS=int(60.0 * 1e6), Name="ATC_RAT_RLL_P", Value=0.15, Default=float("nan"))
    log = _load(w, "stat_rewrites_tune.bin")
    r = _results(check_params(log, airborne_window(log, method="ev")))["parameter changes"]
    assert r.status == "WARN" and r.evidence["n_inflight"] == 1 and r.evidence["names"] == ["ATC_RAT_RLL_P"], r.summary
    for n in ("STAT_DISTFLWN", "BARO1_GND_PRESS", "INS_GYR2OFFS_X", "INS_GYROFFS_Z", "INS_GYR1_CALTEMP", "COMPASS_DEC"):
        assert firmware_maintained(n), n
    for n in ("ATC_RAT_RLL_P", "INS_HNTCH_FREQ", "MOT_SPIN_MIN", "COMPASS_OFS_X"):
        assert firmware_maintained(n) is None, n


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


# ------------------------------------------------------------ issue #18

def test_throttle_helpers_scale_permille_and_fall_back_to_param():
    """CTUN.ThO logged in permille is read as 0-1, and a hover throttle the log never
    learned (ThH 0) falls back to MOT_THST_HOVER."""
    w = _writer()
    _params(w, MOT_THST_HOVER=0.35)
    for i in range(int(SPAN * HZ)):
        w.msg("CTUN", TimeUS=int(i / HZ * 1e6), ThO=400.0, ThH=0.0, BAlt=5.0)
    log = _load(w, "permille.bin")
    win = airborne_window(log, method="none")
    assert _throttle_at(log, np.array([50.0]))[0] == pytest.approx(0.40, abs=1e-3)
    assert _hover_throttle(log, win, _Params(log)) == pytest.approx(0.35)
    w = _writer()
    w.msg("BAT", TimeUS=1000, Inst=0, Volt=25.0, Curr=1.0, CurrTot=0.0)
    assert _throttle_at(_load(w, "no_ctun.bin"), np.array([0.0])) is None


# A flight that hovers at 5 m/s^2 on every axis, punches out over 80-84 s (28 on X/Y, 10 on
# Z, 3 clips) and takes one 40 m/s^2 X spike (and 1 clip) at 109 s, inside the last 3 s.
PUNCH = (80.0, 84.0)


def _punch_vibe(t):
    clip = (3 if t > 82.0 else 0) + (1 if t >= 109.0 else 0)
    if PUNCH[0] <= t <= PUNCH[1]:
        return 28.0, 28.0, 10.0, clip
    if t == 109.0:
        return 40.0, 5.0, 5.0, clip
    return 5.0, 5.0, 5.0, clip


def _vibe_rows(sec):
    t = _table(sec, "vibe_bands")
    return {(r[0], r[1]): dict(zip(t["columns"], r)) for r in t["rows"]}


def _punch_log(name, **kw):
    return _motor_log(name, full=PUNCH, vibe=kw.pop("vibe", _punch_vibe), **kw)


def test_vibration_is_banded_by_throttle():
    log = _punch_log("vibe_bands.bin")
    sec = check_vibration(log, airborne_window(log, method="ev"))
    rows = _vibe_rows(sec)
    assert {(0, "idle"), (0, "hover"), (0, "high"), (0, "touchdown")} <= set(rows), rows
    assert rows[(0, "hover")]["VibeX p95"] == pytest.approx(5.0, abs=0.1)
    assert rows[(0, "high")]["VibeX p95"] == pytest.approx(28.0, abs=0.1)
    assert rows[(0, "high")]["VibeZ p95"] == pytest.approx(10.0, abs=0.1)
    rs = _results(sec)
    # the whole-window p95 still hides the punch-outs - that is what the bands are for
    assert rs["IMU0 VibeX p95"].evidence["value"] == pytest.approx(5.0, abs=0.1)
    r = rs["IMU0 VibeX p95 (high throttle)"]
    assert r.status == "WARN" and r.evidence["value"] == pytest.approx(28.0, abs=0.1)


def test_touchdown_spike_is_not_blamed_on_punch_outs():
    log = _punch_log("vibe_touchdown.bin")
    rows = _vibe_rows(check_vibration(log, airborne_window(log, method="ev")))
    assert rows[(0, "touchdown")]["VibeX max"] == pytest.approx(40.0)
    assert rows[(0, "high")]["VibeX max"] == pytest.approx(28.0, abs=0.1)
    assert rows[(0, "hover")]["VibeX max"] == pytest.approx(5.0, abs=0.1)


def test_clips_are_counted_per_band():
    log = _punch_log("vibe_clips.bin")
    sec = check_vibration(log, airborne_window(log, method="ev"))
    rows = _vibe_rows(sec)
    assert rows[(0, "high")]["clips"] == 3
    assert rows[(0, "touchdown")]["clips"] == 1
    assert rows[(0, "hover")]["clips"] == 0
    assert _results(sec)["IMU0 accel clipping"].evidence["value"] == 4


def test_vibe_bands_without_ctun():
    w = _writer()
    for i in range(int(SPAN * HZ)):
        w.msg("VIBE", TimeUS=int(i / HZ * 1e6), IMU=0, VibeX=5.0, VibeY=5.0, VibeZ=5.0, Clip=0)
    log = _load(w, "vibe_no_ctun.bin")
    sec = check_vibration(log, airborne_window(log, method="none"))
    assert not any(t["name"] == "vibe_bands" for t in sec.tables)
    assert "need CTUN" in _notes(sec)
    assert "IMU0 VibeX p95" in _results(sec)


def test_no_punch_out_means_no_high_band():
    log = _motor_log("vibe_no_punch.bin", full=(999.0, 999.0), vibe=_punch_vibe)
    sec = check_vibration(log, airborne_window(log, method="ev"))
    assert (0, "high") not in _vibe_rows(sec)
    assert not any(n.endswith("(high throttle)") for n in _results(sec))


def test_clip_counter_reset_is_not_negative():
    def vibe(t):
        return 5.0, 5.0, 5.0, (7 if 20.0 < t <= 50.0 else 0)
    log = _punch_log("vibe_clip_reset.bin", vibe=vibe)
    rows = _vibe_rows(check_vibration(log, airborne_window(log, method="ev")))
    assert all(r["clips"] >= 0 for r in rows.values()), rows


def test_vibe_bands_per_imu():
    log = _punch_log("vibe_two_imus.bin", vibe_imus=2)
    sec = check_vibration(log, airborne_window(log, method="ev"))
    rows = _vibe_rows(sec)
    assert (0, "high") in rows and (1, "high") in rows
    rs = _results(sec)
    assert "IMU0 VibeX p95 (high throttle)" in rs and "IMU1 VibeX p95 (high throttle)" in rs


def test_touchdown_band_needs_a_window_ending_at_a_landing():
    """A hover chunk, an explicit window or a padded flight does not end at a landing, so
    its last 3 s are not touchdown and stay in the hover band."""
    log = _punch_log("vibe_hover_window.bin")
    for win in (Window(30.0, 60.0, "hover chunk 1 of 1: test"), Window(30.0, 60.0, "explicit 30:60 s"),
                airborne_window(log, method="ev", pad=2.0)):
        sec = check_vibration(log, win)
        rows = _vibe_rows(sec)
        assert (0, "touchdown") not in rows, (win.method, rows)
        assert "does not end at a detected\nlanding" in _notes(sec), win.method
    rows = _vibe_rows(check_vibration(log, Window(30.0, 60.0, "hover chunk 1 of 1: test")))
    assert rows[(0, "hover")]["n"] == 301
    # outside a hover chunk is mostly flight, so idle is not "armed on the ground" either
    assert (0, "idle") not in rows, rows


def test_hover_band_label_names_the_hover_throttle():
    log = _punch_log("vibe_thh_label.bin")
    rows = _vibe_rows(check_vibration(log, airborne_window(log, method="ev")))
    assert "ThH 0.400" in rows[(0, "hover")]["ThO band"]


def test_no_band_with_enough_samples_says_so():
    """CTUN is logged but no band reaches 10 VIBE samples: no table, and a note saying why
    rather than one that points at a missing table."""
    w = _writer()
    for i in range(int(SPAN * HZ)):
        us = int(i / HZ * 1e6)
        w.msg("CTUN", TimeUS=us, ThO=0.4, ThH=0.4, BAlt=5.0)
        w.msg("VIBE", TimeUS=us, IMU=0, VibeX=5.0, VibeY=5.0, VibeZ=5.0, Clip=0)
    log = _load(w, "vibe_short.bin")
    sec = check_vibration(log, Window(50.0, 50.5, "explicit 50:50.5 s"))
    assert not any(t["name"] == "vibe_bands" for t in sec.tables)
    text = _notes(sec)
    assert "No throttle band reached 10 VIBE samples" in text and "vibe_bands)" not in text, text


if __name__ == "__main__":
    import _shim
    sys.exit(_shim.run(sys.modules[__name__]))
