"""WP1 of docs/pid-tuning-plan.md: signal extraction, segmentation and the gates.

Each test builds exactly the log it needs with tests/synthlog.py (a copter skeleton with
PIDR/PIDP/PIDY, RATE and CTUN at a chosen rate), so it runs anywhere with no flight data,
then asserts on the AxisSignals / Refusal lists `dflog.tune.extract_axes` returns.

    python tests/test_tune_extract.py
"""

import os
import re
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
from dflog.checks import T                                               # noqa: E402
from dflog import tune                                                   # noqa: E402
from dflog.tune import GainSet, extract_axes, segment                    # noqa: E402
from synthlog import LogWriter                                           # noqa: E402

TMP = tempfile.mkdtemp(prefix="dflog-tune-")
SPAN = 80.0
FLIGHT = (5.0, 75.0)
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

BASE_PARAMS = dict(
    LOG_BITMASK=180222, SCHED_LOOP_RATE=400, INS_GYRO_FILTER=40, ATC_RATE_FF_ENAB=1,
    MOT_THST_HOVER=0.30, AUTOTUNE_AGGR=0.08, AUTOTUNE_MIN_D=0.001, AUTOTUNE_GMBK=0.25,
    ATC_RAT_RLL_P=0.1235, ATC_RAT_RLL_I=0.1235, ATC_RAT_RLL_D=0.00334, ATC_RAT_RLL_FF=0.0,
    ATC_RAT_RLL_FLTD=20.0, ATC_RAT_RLL_FLTT=20.0, ATC_RAT_RLL_FLTE=0.0, ATC_RAT_RLL_SMAX=0.0,
    ATC_RAT_RLL_IMAX=0.5, ATC_ANG_RLL_P=16.0,
    ATC_RAT_PIT_P=0.15, ATC_RAT_PIT_I=0.15, ATC_RAT_PIT_D=0.004, ATC_RAT_PIT_FF=0.0,
    ATC_RAT_PIT_FLTD=20.0, ATC_RAT_PIT_FLTT=20.0, ATC_RAT_PIT_FLTE=0.0, ATC_RAT_PIT_SMAX=0.0,
    ATC_RAT_PIT_IMAX=0.5, ATC_ANG_PIT_P=15.0,
    ATC_RAT_YAW_P=0.5, ATC_RAT_YAW_I=0.05, ATC_RAT_YAW_D=0.0, ATC_RAT_YAW_FF=0.0,
    ATC_RAT_YAW_FLTD=0.0, ATC_RAT_YAW_FLTT=20.0, ATC_RAT_YAW_FLTE=2.5, ATC_RAT_YAW_SMAX=0.0,
    ATC_RAT_YAW_IMAX=0.5, ATC_ANG_YAW_P=6.0,
    ATC_ACCEL_R_MAX=110000, ATC_ACCEL_P_MAX=110000, ATC_ACCEL_Y_MAX=27000,
)


def _load(w, name):
    return Log(w.write(os.path.join(TMP, name)), use_cache=False)


def _writer(params):
    """A copter log skeleton: banner, a flight by EV, PARM, and the rate-loop formats
    (column names and format chars follow ArduCopter 4.7)."""
    w = LogWriter()
    w.fmt(96, "PARM", "QNff", "TimeUS,Name,Value,Default")
    w.fmt(97, "MSG", "QZ", "TimeUS,Message")
    w.fmt(64, "EV", "QB", "TimeUS,Id")
    w.fmt(200, "PIDR", "QffffffffffB", "TimeUS,Tar,Act,Err,P,I,D,FF,DFF,Dmod,SRate,Flags")
    w.fmt(201, "PIDP", "QffffffffffB", "TimeUS,Tar,Act,Err,P,I,D,FF,DFF,Dmod,SRate,Flags")
    w.fmt(202, "PIDY", "QffffffffffB", "TimeUS,Tar,Act,Err,P,I,D,FF,DFF,Dmod,SRate,Flags")
    w.fmt(203, "RATE", "Qfffffffff", "TimeUS,RDes,R,ROut,PDes,P,POut,YDes,Y,YOut")
    w.fmt(205, "CTUN", "Qfff", "TimeUS,ThO,ThH,BAlt")
    w.msg("MSG", TimeUS=20, Message="ArduCopter V4.7.1 (deadbeef)")
    w.msg("PARM", TimeUS=30, Name="FRAME_CLASS", Value=1.0, Default=1.0)
    w.msg("PARM", TimeUS=31, Name="FRAME_TYPE", Value=1.0, Default=1.0)
    for i, (k, v) in enumerate(params.items()):
        w.msg("PARM", TimeUS=40 + i, Name=k, Value=float(v), Default=float("nan"))
    w.msg("EV", TimeUS=int(FLIGHT[0] * 1e6), Id=28)
    w.msg("EV", TimeUS=int(FLIGHT[1] * 1e6), Id=18)
    return w


def _pid_log(name, hz=400.0, params=None, pid=True, rate=True, ctun=True, changes=(),
             limited_frac=0.0, drop=None, extra=None, pid_write_us=None, pid_skip_every=0):
    """PIDR/PIDP/PIDY (and RATE) at `hz` over the whole span. `changes` is
    [(t, name, value)] PARM writes inside the flight; `limited_frac` sets Flags bit 0 on
    that fraction of samples; `drop` = (t0, t1) leaves a hole in every stream.

    `pid_write_us` = (lo, hi) stamps each PIDx record a pseudo-random lo..hi us after its
    RATE tick, as ArduCopter 4.7 does (PIDx carries the time it was written, RATE the loop
    start); `pid_skip_every` = k omits every k-th PIDx record (a genuine drop).

    The signals are a deterministic sum of sines - enough to be non-constant, not a
    closed loop; WP1 measures rate and shape, not response."""
    p = dict(BASE_PARAMS) if params is None else dict(params)
    if extra:
        p.update(extra)
    w = _writer(p)
    n = int(SPAN * hz)
    rng = np.random.default_rng(7)
    for i in range(n):
        t = i / hz
        if drop is not None and drop[0] <= t < drop[1]:
            continue
        us = int(round(t * 1e6))
        tar = 30.0 * np.sin(2 * np.pi * 0.5 * t)
        act = 28.0 * np.sin(2 * np.pi * 0.5 * t - 0.2)
        limited = 1 if (limited_frac > 0 and (i % 100) < limited_frac * 100) else 0
        pid_us = us if pid_write_us is None else us + int(rng.integers(*pid_write_us))
        if pid and not (pid_skip_every and i % pid_skip_every == pid_skip_every - 1):
            for msg, k in (("PIDR", 1.0), ("PIDP", 0.9), ("PIDY", 0.5)):
                r = np.pi / 180.0          # PIDx logs rad/s; RATE logs deg/s
                w.msg(msg, TimeUS=pid_us, Tar=k * tar * r, Act=k * act * r, Err=k * (tar - act) * r,
                      P=0.02 * k * (tar - act), I=0.001 * k, D=0.0005 * k, FF=0.0, DFF=0.0,
                      Dmod=1.0, SRate=0.5, Flags=limited)
        if rate:
            w.msg("RATE", TimeUS=us, RDes=tar, R=act, ROut=0.02 * (tar - act),
                  PDes=0.9 * tar, P=0.9 * act, POut=0.018 * (tar - act),
                  YDes=0.5 * tar, Y=0.5 * act, YOut=0.01 * (tar - act))
        if ctun and i % max(int(hz / 10), 1) == 0:
            w.msg("CTUN", TimeUS=us, ThO=0.31, ThH=0.33, BAlt=5.0)
    for t, name, value in changes:
        w.msg("PARM", TimeUS=int(t * 1e6), Name=name, Value=float(value), Default=float("nan"))
    return _load(w, name)


def _by_axis(sigs):
    out = {}
    for s in sigs:
        out.setdefault(s.axis, []).append(s)
    return out


def _codes(refs):
    return [r.code for r in refs]


# ------------------------------------------------------------------ extraction

def test_pidr_at_400hz_gives_fs_400_and_source_pidr():
    log = _pid_log("fast.bin", hz=400.0)
    w = airborne_window(log, method="ev")
    sigs, refs = extract_axes(log, w)
    assert refs == [], [r.line() for r in refs]
    ax = _by_axis(sigs)
    assert set(ax) == {"roll", "pitch", "yaw"}
    r = ax["roll"][0]
    assert r.source == "PIDR" and ax["pitch"][0].source == "PIDP" and ax["yaw"][0].source == "PIDY"
    assert r.fs == pytest.approx(400.0, rel=0.01)
    assert r.jitter < 0.01
    assert r.segment == (FLIGHT[0], FLIGHT[1])
    assert r.n == pytest.approx(70 * 400, abs=2)
    # the plant input is the sum of the PID terms and FF (+ DFF)
    assert np.allclose(r.out, r.p + r.i + r.d + r.ff)
    assert r.dmod is not None and r.srate is not None and r.flags is not None
    assert r.limited_pct == 0.0
    assert r.gains.rat_p == pytest.approx(0.1235) and r.gains.axis == "roll"
    assert r.summary()["fs"] == pytest.approx(400.0, rel=0.01)


def test_pid_write_time_jitter_is_restamped_to_the_rate_loop_clock():
    # ArduCopter 4.7 stamps PIDx with the time it was written, 0.4-1.7 ms into the loop,
    # while RATE carries the loop start: raw PIDx jitter ~40 %, one record per loop tick
    # (brisket-t1.bin, Brisket). That is not irregular sampling.
    log = _pid_log("pidjit.bin", hz=400.0, pid_write_us=(400, 1700))
    w = airborne_window(log, method="ev")
    sigs, refs = extract_axes(log, w)
    assert refs == [], [r.line() for r in refs]
    r = _by_axis(sigs)["roll"][0]
    assert r.source == "PIDR" and r.clock == "RATE"
    assert r.fs == pytest.approx(400.0, rel=0.01) and r.jitter < 0.01
    assert r.raw_jitter > 0.05
    rate_t = w.clip(log.df("RATE"))["t"].values
    assert np.isin(r.t, rate_t).all()
    assert r.summary()["clock"] == "RATE"


def test_pid_jitter_with_dropped_records_is_still_irregular():
    log = _pid_log("pidjitdrop.bin", hz=400.0, pid_write_us=(400, 1700), pid_skip_every=50)
    sigs, refs = extract_axes(log, airborne_window(log, method="ev"))
    assert sigs == [] and set(_codes(refs)) == {"IRREGULAR_SAMPLING"}
    assert "loop tick" in refs[0].message


def test_pid_jitter_without_rate_has_no_loop_clock_and_is_irregular():
    log = _pid_log("pidjitnorate.bin", hz=400.0, pid_write_us=(400, 1700), rate=False)
    sigs, refs = extract_axes(log, airborne_window(log, method="ev"))
    assert sigs == [] and set(_codes(refs)) == {"IRREGULAR_SAMPLING"}


def test_10hz_is_pid_rate_too_low_with_the_bitmask_fix():
    log = _pid_log("slow.bin", hz=10.0)
    w = airborne_window(log, method="ev")
    sigs, refs = extract_axes(log, w)
    assert sigs == []
    assert _codes(refs) == ["PID_RATE_TOO_LOW"] * 3
    r = refs[0]
    assert r.axis == "roll" and r.log_name == "slow.bin"
    assert r.message.startswith("PIDR logged at 10.0 Hz")
    assert "100 Hz needed (200 Hz recommended)" in r.message
    assert "LOG_BITMASK" in r.fix and "180222 -> 180223" in r.fix and str(180222 | 1) in r.fix
    assert r.fix.startswith(tune.LOGGING_FIX_HEADER + "\n")
    assert "fly the tuning profile" in r.fix
    d = r.to_dict()
    assert d["code"] == "PID_RATE_TOO_LOW" and d["fix"] == r.fix
    assert d["requirements"][0]["param"] == "LOG_BITMASK" and d["requirements"][0]["ok"] is False
    # without LOG_BITMASK in the log the fix still names the bits
    log2 = _pid_log("slow2.bin", hz=10.0, params={k: v for k, v in BASE_PARAMS.items() if k != "LOG_BITMASK"})
    _s, refs2 = extract_axes(log2, airborne_window(log2, method="ev"))
    assert "LOG_BITMASK" in refs2[0].fix and "not in log" in refs2[0].fix and "bit 0" in refs2[0].fix


def test_param_change_mid_window_splits_the_segment_and_the_gainsets_differ():
    log = _pid_log("change.bin", hz=400.0, changes=[(40.0, "ATC_RAT_RLL_P", 0.15)])
    w = airborne_window(log, method="ev")
    assert segment(log, w, "roll") == [(FLIGHT[0], 40.0), (40.0, FLIGHT[1])]
    assert segment(log, w, "pitch") == [(FLIGHT[0], FLIGHT[1])]
    sigs, refs = extract_axes(log, w)
    assert refs == [], [r.line() for r in refs]
    ax = _by_axis(sigs)
    assert len(ax["roll"]) == 2 and len(ax["pitch"]) == 1 and len(ax["yaw"]) == 1
    a, b = ax["roll"]
    assert a.segment == (FLIGHT[0], 40.0) and b.segment == (40.0, FLIGHT[1])
    assert a.gains.rat_p == pytest.approx(0.1235) and b.gains.rat_p == pytest.approx(0.15)
    assert a.gains != b.gains
    assert a.gains.rat_d == b.gains.rat_d
    # the time bases do not overlap and together cover the window
    assert a.t[-1] <= 40.0 <= b.t[0]
    # a change on every axis' shared parameter splits every axis
    log2 = _pid_log("change2.bin", hz=400.0, changes=[(30.0, "INS_GYRO_FILTER", 30.0)])
    w2 = airborne_window(log2, method="ev")
    for axis in ("roll", "pitch", "yaw"):
        assert segment(log2, w2, axis) == [(FLIGHT[0], 30.0), (30.0, FLIGHT[1])], axis
    # a change that leaves nothing >= 10 s is GAINS_CHANGED_IN_FLIGHT
    log3 = _pid_log("change3.bin", hz=400.0,
                    changes=[(t, "ATC_RAT_PIT_D", 0.0041 + 0.0001 * k) for k, t in enumerate(np.arange(9.0, 75.0, 8.0))])
    sigs3, refs3 = extract_axes(log3, airborne_window(log3, method="ev"), axes=("pitch",))
    assert sigs3 == [] and _codes(refs3) == ["GAINS_CHANGED_IN_FLIGHT"]
    assert "one gain set per flight" in refs3[0].fix


def test_accel_max_spellings_convert_both_ways_and_record_the_name():
    base = {k: v for k, v in BASE_PARAMS.items() if not k.startswith("ATC_ACC")}
    old = _pid_log("accel_old.bin", hz=10.0, params=dict(base, ATC_ACCEL_R_MAX=110000, ATC_ACCEL_P_MAX=116700))
    g = GainSet.from_log(old, 10.0, "roll")
    assert g.acc_max_dps2 == pytest.approx(1100.0)
    assert g.param_names["acc_max_dps2"] == "ATC_ACCEL_R_MAX"
    assert "ATC_ACCEL_R_MAX" not in g.defaulted and "ATC_ACC_R_MAX" not in g.defaulted
    assert GainSet.from_log(old, 10.0, "pitch").acc_max_dps2 == pytest.approx(1167.0)

    new = _pid_log("accel_new.bin", hz=10.0, params=dict(base, ATC_ACC_R_MAX=1100, ATC_ACC_P_MAX=1167))
    g2 = GainSet.from_log(new, 10.0, "roll")
    assert g2.acc_max_dps2 == pytest.approx(1100.0)
    assert g2.param_names["acc_max_dps2"] == "ATC_ACC_R_MAX"
    assert GainSet.from_log(new, 10.0, "pitch").acc_max_dps2 == pytest.approx(1167.0)

    # neither spelling: the firmware default (110000 cdeg/s^2) and it is named as defaulted
    g3 = GainSet.from_log(old, 10.0, "yaw")
    assert g3.acc_max_dps2 == pytest.approx(270.0) and "ATC_ACCEL_Y_MAX" in g3.defaulted


def test_defaulted_lists_every_assumed_parameter():
    log = _pid_log("bare.bin", hz=10.0, params=dict(LOG_BITMASK=180222), ctun=False)
    g = GainSet.from_log(log, 10.0, "roll")
    for name in ("ATC_RAT_RLL_P", "ATC_RAT_RLL_I", "ATC_RAT_RLL_D", "ATC_RAT_RLL_FLTD", "ATC_ANG_RLL_P",
                 "ATC_ACCEL_R_MAX", "INS_GYRO_FILTER", "ATC_RATE_FF_ENAB", "SCHED_LOOP_RATE",
                 "MOT_THST_HOVER", "AUTOTUNE_AGGR", "AUTOTUNE_MIN_D"):
        assert name in g.defaulted, name
    assert g.rat_p == pytest.approx(0.135) and g.rat_d == pytest.approx(0.0036)
    assert g.ang_p == pytest.approx(4.5) and g.loop_hz == 400.0 and g.gyro_filter == 20.0
    assert g.aggr == pytest.approx(0.075) and g.min_d == pytest.approx(0.0005)
    assert g.thst_hover == pytest.approx(0.35) and g.param_names["thst_hover"] == "MOT_THST_HOVER"
    assert g.gmbk is None and "AUTOTUNE_GMBK" not in g.defaulted     # absence means 4.x backoffs
    assert g.ff_enab is True
    y = GainSet.from_log(log, 10.0, "yaw")
    assert y.rat_p == pytest.approx(0.18) and y.rat_i == pytest.approx(0.018) and y.flte == pytest.approx(2.5)

    # a fully parameterised log defaults nothing, reads the learned hover throttle, and
    # honours an AGGR override
    full = _pid_log("full.bin", hz=10.0)
    g2 = GainSet.from_log(full, 10.0, "roll")
    assert g2.defaulted == [], g2.defaulted
    assert g2.thst_hover == pytest.approx(0.33) and g2.param_names["thst_hover"] == "CTUN.ThH"
    assert g2.gmbk == pytest.approx(0.25) and g2.aggr == pytest.approx(0.08) and g2.gyro_filter == 40.0
    g3 = GainSet.from_log(full, 10.0, "roll", aggr_override=0.1)
    assert g3.aggr == pytest.approx(0.1) and g3.param_names["aggr"] == "override"
    assert set(g2.to_dict()) >= {"rat_p", "acc_max_dps2", "defaulted", "param_names"}


def test_rate_only_log_falls_back_to_rate_with_pid_arrays_none():
    log = _pid_log("rateonly.bin", hz=400.0, pid=False)
    w = airborne_window(log, method="ev")
    sigs, refs = extract_axes(log, w)
    assert refs == [], [r.line() for r in refs]
    ax = _by_axis(sigs)
    for axis in ("roll", "pitch", "yaw"):
        s = ax[axis][0]
        assert s.source == "RATE" and s.fs == pytest.approx(400.0, rel=0.01)
        assert s.p is None and s.d is None and s.flags is None and s.srate is None
        assert s.out is not None and s.limited_pct == 0.0
    r = ax["roll"][0]
    assert np.max(np.abs(r.tar)) == pytest.approx(30.0, rel=0.01)
    assert np.max(np.abs(ax["pitch"][0].tar)) == pytest.approx(27.0, rel=0.01)
    # a 10 Hz RATE-only log's fix sets bit 12 too, since PIDx was not logged at all
    cur = 180222 & ~4096
    slow = _pid_log("rateonly_slow.bin", hz=10.0, pid=False, extra=dict(LOG_BITMASK=cur))
    _s, refs2 = extract_axes(slow, airborne_window(slow, method="ev"), axes=("roll",))
    assert _codes(refs2) == ["PID_RATE_TOO_LOW"] and refs2[0].message.startswith("RATE logged at 10.0 Hz")
    assert f"{cur} -> {cur | 1 | 4096}" in refs2[0].fix and "bit 12" in refs2[0].fix


def test_no_messages_is_no_pid_messages():
    log = _pid_log("none.bin", hz=10.0, pid=False, rate=False)
    w = airborne_window(log, method="ev")
    sigs, refs = extract_axes(log, w)
    assert sigs == []
    assert _codes(refs) == ["NO_PID_MESSAGES"] * 3
    assert "absent from the log" in refs[0].message and "bit 12" in refs[0].message
    assert "LOG_BITMASK" in refs[0].fix and "180222 -> 180223" in refs[0].fix
    assert refs[0].requirements and refs[0].requirements[0]["param"] == "LOG_BITMASK"
    assert {r.axis for r in refs} == {"roll", "pitch", "yaw"}


# ------------------------------------------------------------ logging requirements

def test_logging_fix_names_the_bitmask_and_batch_mask_changes():
    log = _pid_log("req_batch.bin", hz=10.0, extra=dict(INS_LOG_BAT_MASK=1, LOG_FILE_RATEMAX=0,
                                                        LOG_BLK_RATEMAX=0, LOG_FILE_BUFSIZE=64))
    _s, refs = extract_axes(log, airborne_window(log, method="ev"), axes=("roll",))
    assert _codes(refs) == ["PID_RATE_TOO_LOW"]
    fix = refs[0].fix
    assert "LOG_BITMASK" in fix and "180222 -> 180223" in fix
    assert "INS_LOG_BAT_MASK  1 -> 0" in fix
    assert "LOG_FILE_RATEMAX  0  ok" in fix and "LOG_FILE_BUFSIZE  64  ok" in fix
    assert "batch logging doubles the write rate" in fix
    assert not any(line != line.rstrip() for line in fix.splitlines())
    # the rows behind the text
    rows = {r["param"]: r for r in refs[0].requirements}
    assert rows["LOG_BITMASK"]["current"] == 180222 and rows["LOG_BITMASK"]["required"] == "180223"
    assert rows["LOG_BITMASK"]["ok"] is False and rows["INS_LOG_BAT_MASK"]["ok"] is False
    assert rows["LOG_FILE_RATEMAX"]["ok"] is True and rows["SCHED_LOOP_RATE"]["ok"] is True
    assert rows["SCHED_LOOP_RATE"]["current"] == 400


def test_fast_log_with_no_caps_passes_every_log_row():
    log = _pid_log("req_ok.bin", hz=400.0, extra=dict(LOG_BITMASK=180223, LOG_FILE_RATEMAX=0,
                                                      LOG_BLK_RATEMAX=0, INS_LOG_BAT_MASK=0,
                                                      LOG_FILE_BUFSIZE=200, LOG_DISARMED=0))
    rows = tune.logging_requirements(log)
    assert [r["param"] for r in rows] == [r["param"] for r in tune.LOGGING_REQUIREMENTS]
    for r in rows:
        if r["param"].startswith("LOG_"):
            assert r["ok"] is True, r
    by = {r["param"]: r for r in rows}
    assert by["LOG_BITMASK"]["required"] == "180223" and by["INS_LOG_BAT_MASK"]["ok"] is True
    # a rate cap at or above the loop rate is fine; below it is not
    capped = _pid_log("req_cap.bin", hz=400.0, extra=dict(LOG_BITMASK=180223, LOG_FILE_RATEMAX=400))
    assert {r["param"]: r["ok"] for r in tune.logging_requirements(capped)}["LOG_FILE_RATEMAX"] is True
    low = _pid_log("req_cap2.bin", hz=400.0, extra=dict(LOG_BITMASK=180223, LOG_FILE_RATEMAX=50))
    assert {r["param"]: r["ok"] for r in tune.logging_requirements(low)}["LOG_FILE_RATEMAX"] is False
    assert "LOG_FILE_RATEMAX  50 -> 0" in tune.format_logging_fix(tune.logging_requirements(low))


def test_logging_requirements_reports_absent_params_as_none():
    log = _pid_log("req_absent.bin", hz=10.0, params=dict(LOG_BITMASK=180222))
    rows = {r["param"]: r for r in tune.logging_requirements(log)}
    for name in ("LOG_FILE_RATEMAX", "LOG_BLK_RATEMAX", "INS_LOG_BAT_MASK", "LOG_FILE_BUFSIZE",
                 "LOG_DISARMED", "SCHED_LOOP_RATE", "AUTOTUNE_AXES", "SID_AXIS", "SID_T_FADE_OUT"):
        assert rows[name]["current"] is None and rows[name]["ok"] is None, name
    assert rows["LOG_BITMASK"]["current"] == 180222 and rows["LOG_BITMASK"]["ok"] is False
    text = tune.format_logging_fix(list(rows.values()))
    assert "INS_LOG_BAT_MASK  not in log -> 0" in text
    assert re.search(r"^  SID_AXIS\s+not in log$", text, re.M), text
    # every row keeps the plan's fields
    for r in rows.values():
        assert set(r) >= {"param", "current", "required", "ok", "why", "tier"}
    for row in tune.LOGGING_REQUIREMENTS:
        assert set(row) >= {"param", "required", "why", "tier", "rule"} and row["rule"] in tune._RULES


def test_format_logging_fix_is_deterministic():
    log = _pid_log("req_det.bin", hz=10.0, extra=dict(INS_LOG_BAT_MASK=1))
    a = tune.format_logging_fix(tune.logging_requirements(log))
    b = tune.format_logging_fix(tune.logging_requirements(log))
    assert a == b and a
    assert tune.logging_fix(log)[0] == tune.LOGGING_FIX_HEADER + "\n" + a
    # the parameter column is aligned: every line's value starts at the same column
    col = 2 + max(len(r["param"]) for r in tune.LOGGING_REQUIREMENTS) + 2
    for line in a.splitlines():
        assert line.startswith("  ") and line[col - 2:col] == "  " and line[col] != " ", line


def test_flags_bit0_on_half_the_samples_is_output_saturated_but_still_returned():
    log = _pid_log("limited.bin", hz=400.0, limited_frac=0.5)
    w = airborne_window(log, method="ev")
    sigs, refs = extract_axes(log, w, axes=("roll",))
    assert len(sigs) == 1
    s = sigs[0]
    assert s.limited_pct == pytest.approx(50.0, abs=0.5)
    assert _codes(refs) == ["OUTPUT_SATURATED"]
    assert "50.0 %" in refs[0].message and f"{T['tune_limited_pct']['fail']:.0f} %" in refs[0].message
    assert "alog motors" in refs[0].fix
    # below the fail level nothing is refused, and limited_pct still says how much
    mild = _pid_log("mild.bin", hz=400.0, limited_frac=0.1)
    sigs2, refs2 = extract_axes(mild, airborne_window(mild, method="ev"), axes=("roll",))
    assert refs2 == [] and sigs2[0].limited_pct == pytest.approx(10.0, abs=0.5)


# ------------------------------------------------------------------ thresholds

def test_new_thresholds_have_sources_and_rows_in_thresholds_md():
    keys = ["tune_pid_rate_hz", "tune_min_frames", "tune_coherence", "tune_confidence",
            "tune_srate_osc", "tune_overshoot_ratio", "tune_bounce_ratio", "tune_gain_margin_db",
            "tune_phase_margin_deg", "tune_session_spread", "tune_pi_ratio_dev",
            "tune_flt_ratio_dev", "tune_limited_pct"]
    with open(os.path.join(ROOT, "reference", "thresholds.md"), encoding="utf-8") as fh:
        doc = fh.read()
    for k in keys:
        assert k in T, k
        assert T[k]["source"] and T[k]["note"], k
        assert re.search(r"^\| `" + re.escape(k) + r"` \|", doc, re.M), f"{k} has no row in thresholds.md"
    assert T["tune_pid_rate_hz"]["warn"] == 200 and T["tune_pid_rate_hz"]["fail"] == 100
    assert T["tune_limited_pct"]["warn"] == 5 and T["tune_limited_pct"]["fail"] == 20
    assert T["tune_srate_osc"]["warn"] == 5 and T["tune_srate_osc"]["fail"] == 10
    # every constant carries a value and a source
    for name, c in tune.CONSTANTS.items():
        assert set(c) >= {"value", "source"} and c["source"], name
    for name in ("step_frame_s", "step_response_s", "step_overlap", "step_cut_hz", "step_min_target_dps",
                 "step_split_dps", "segment_min_s", "autotune_aggr_default", "autotune_gmbk_default",
                 "autotune_min_d_default", "autotune_success_count", "autotune_d_up_down_margin",
                 "autotune_pi_ratio_final", "autotune_yaw_pi_ratio_final", "quik_osc_smax", "quik_gain_margin"):
        assert name in tune.CONSTANTS, name
    assert tune.CONSTANTS["segment_min_s"]["value"] == 10.0
    assert tune.CONSTANTS["autotune_success_count"]["value"] == 4
    for name, c in tune.DEFAULTS.items():
        assert c["source"], name


if __name__ == "__main__":
    import _shim
    sys.exit(_shim.run(sys.modules[__name__]))
