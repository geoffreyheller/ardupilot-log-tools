"""Synthetic fast-logged copter logs whose plant and controller are known exactly.

Built on `dflog.tunesim` (the AC_PID replica, the plant and the AutoTune twitch
simulator) and `tests/synthlog.py` (the DataFlash writer). Nothing here is used by the
toolkit itself; it exists so `alog tune` can be tested against a truth: the true plant
(`Plant.freq_response`), the true closed-loop step response
(`Plant.true_closed_loop_step`) and the gains AutoTune would find (`tunesim.autotune`).

    from tunesynth import fast_log, autotune_log, PLANT_5IN, GAINS_5IN
    w = fast_log(PLANT_5IN, GAINS_5IN, seconds=60)          # PIDR/PIDP/PIDY at 400 Hz
    log = Log(w.write("fast.bin"), use_cache=False)
    w, result = autotune_log(PLANT_5IN, GAINS_5IN, axes=("roll",))   # an ATUN session

Formats follow ArduCopter 4.7 (`reference/messages.md`, `docs/pid-tuning-plan.md`):
`PIDR TimeUS,Tar,Act,Err,P,I,D,FF,DFF,Dmod,SRate,Flags` / `QffffffffffB`,
`RATE TimeUS,RDes,R,ROut,PDes,P,POut,YDes,Y,YOut` / `Qfffffffff`,
`ATUN TimeUS,Axis,TuneStep,Targ,Min,Max,RP,RD,SP,ddt` / `QBBfffffff`. Parameters use the
4.x spellings (`ATC_ACCEL_R_MAX` in cdeg/s²). Everything is deterministic for a given
`seed`: the stick sequence and the measurement noise come from
`numpy.random.default_rng(seed)`, the simulator has no randomness at all.

`sidd_log` (a SysID chirp log with SIDS/SIDD) is not implemented in this version; the
`stick="chirp"` option of `fast_log` gives a chirp on the rate target through the
ordinary PIDx/RATE messages instead.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from synthlog import LogWriter                                            # noqa: E402
from dflog.tunesim import Plant, ClosedLoop, autotune, gain_dict, AXIS_ID  # noqa: E402

AXES = ("roll", "pitch", "yaw")
BANNER = "ArduCopter V4.7.1 (deadbeef)"
ALT_HOLD, AUTOTUNE_MODE = 2, 15

# --------------------------------------------------------------------- reference aircraft
#
# Two plants that make the AutoTune search land near the gains real aircraft carry:
# a 5-inch quad (the Circuit quad: AutoTune found ATC_RAT_RLL_P 0.1235, D 0.00334,
# ANG_P 16 with INS_GYRO_FILTER 75) and a 10-inch one (the Brisket quad on the Mission
# Planner initial set, gyro 42, FLTD/FLTT 21). `k` is deg/s per unit of normalised
# output at DC; `tau1` the dominant lag, `tau2` the motor/ESC lag, `delay` the transport
# delay (gyro filtering + one loop). Yaw has far less authority and a slower response.
# See the WP2 report for how `k` was chosen (the AutoTune outcome, not physics).

PLANT_5IN = dict(roll=Plant(k=10000.0, tau1=0.25, tau2=0.015, delay=0.005),
                 pitch=Plant(k=9000.0, tau1=0.27, tau2=0.015, delay=0.005),
                 yaw=Plant(k=2000.0, tau1=0.50, tau2=0.020, delay=0.005))
PLANT_10IN = dict(roll=Plant(k=6000.0, tau1=0.30, tau2=0.030, delay=0.008),
                  pitch=Plant(k=7000.0, tau1=0.33, tau2=0.030, delay=0.008),
                  yaw=Plant(k=1200.0, tau1=0.60, tau2=0.040, delay=0.012))

_RP_5 = dict(rat_p=0.135, rat_i=0.135, rat_d=0.0036, rat_ff=0.0, fltd=37.5, fltt=37.5, flte=0.0, smax=0.0,
             imax=0.5, ang_p=4.5, acc_max_dps2=1686.0, ff_enab=True, gyro_filter=75.0, thst_hover=0.25,
             loop_hz=400.0, aggr=0.075, gmbk=0.25, min_d=0.0005)
GAINS_5IN = dict(roll=dict(_RP_5), pitch=dict(_RP_5),
                 yaw=dict(_RP_5, rat_p=0.18, rat_i=0.018, rat_d=0.0, flte=2.0, acc_max_dps2=270.0))
_RP_10 = dict(rat_p=0.135, rat_i=0.135, rat_d=0.0036, rat_ff=0.0, fltd=21.0, fltt=21.0, flte=0.0, smax=0.0,
              imax=0.5, ang_p=4.5, acc_max_dps2=1167.0, ff_enab=True, gyro_filter=42.0, thst_hover=0.30,
              loop_hz=400.0, aggr=0.075, gmbk=0.25, min_d=0.0005)
GAINS_10IN = dict(roll=dict(_RP_10), pitch=dict(_RP_10),
                  yaw=dict(_RP_10, rat_p=0.18, rat_i=0.018, rat_d=0.0, flte=2.0, acc_max_dps2=270.0))

_AX = {
    "roll": dict(sfx="RLL", acc="R", pid="PIDR", rate=("RDes", "R", "ROut"), att=("DesRoll", "Roll"), rc="C1", name="Roll"),
    "pitch": dict(sfx="PIT", acc="P", pid="PIDP", rate=("PDes", "P", "POut"), att=("DesPitch", "Pitch"), rc="C2", name="Pitch"),
    "yaw": dict(sfx="YAW", acc="Y", pid="PIDY", rate=("YDes", "Y", "YOut"), att=("DesYaw", "Yaw"), rc="C4", name="Yaw"),
}


def gain_params(axis, gains):
    """{PARM name: value} for one axis' gain set, 4.x spellings (ACCEL in cdeg/s²)."""
    g = gain_dict(gains)
    s, a = _AX[axis]["sfx"], _AX[axis]["acc"]
    out = {f"ATC_RAT_{s}_P": g["rat_p"], f"ATC_RAT_{s}_I": g["rat_i"], f"ATC_RAT_{s}_D": g["rat_d"],
           f"ATC_RAT_{s}_FF": g["rat_ff"], f"ATC_RAT_{s}_FLTD": g["fltd"], f"ATC_RAT_{s}_FLTT": g["fltt"],
           f"ATC_RAT_{s}_FLTE": g["flte"], f"ATC_RAT_{s}_SMAX": g["smax"], f"ATC_RAT_{s}_IMAX": g["imax"],
           f"ATC_ANG_{s}_P": g["ang_p"], f"ATC_ACCEL_{a}_MAX": (g["acc_max_dps2"] or 0.0) * 100.0}
    return out


def common_params(gains, loop_hz, fast, gmbk):
    """The aircraft-wide parameters, taken from the roll gain set."""
    g = gain_dict(gains)
    p = dict(FRAME_CLASS=1.0, FRAME_TYPE=1.0, SCHED_LOOP_RATE=float(loop_hz),
             LOG_BITMASK=180223.0 if fast else 180222.0, INS_GYRO_FILTER=g["gyro_filter"],
             MOT_THST_HOVER=g["thst_hover"] if g["thst_hover"] is not None else 0.35,
             AUTOTUNE_AGGR=g["aggr"], AUTOTUNE_MIN_D=g["min_d"], AUTOTUNE_AXES=7.0,
             ATC_RATE_FF_ENAB=1.0 if g["ff_enab"] else 0.0, ATC_INPUT_TC=0.15, ATC_ANGLE_MAX=4500.0,
             ATC_RATE_R_MAX=0.0, ATC_RATE_P_MAX=0.0, ATC_RATE_Y_MAX=0.0, FSTRATE_ENABLE=0.0)
    if gmbk is not None:
        p["AUTOTUNE_GMBK"] = float(gmbk)
    return p


PARAM_TO_FIELD = {"P": "rat_p", "I": "rat_i", "D": "rat_d", "FF": "rat_ff", "FLTD": "fltd", "FLTT": "fltt",
                  "FLTE": "flte", "SMAX": "smax", "IMAX": "imax"}
_SFX_TO_AXIS = {v["sfx"]: k for k, v in _AX.items()}
_ACC_TO_AXIS = {v["acc"]: k for k, v in _AX.items()}


def param_to_gain(name):
    """(axis or None for all, field, scale) for a gain parameter name, or None."""
    parts = name.split("_")
    if name.startswith("ATC_RAT_") and len(parts) == 4 and parts[2] in _SFX_TO_AXIS and parts[3] in PARAM_TO_FIELD:
        return _SFX_TO_AXIS[parts[2]], PARAM_TO_FIELD[parts[3]], 1.0
    if name.startswith("ATC_ANG_") and len(parts) == 4 and parts[2] in _SFX_TO_AXIS and parts[3] == "P":
        return _SFX_TO_AXIS[parts[2]], "ang_p", 1.0
    if name.startswith("ATC_ACCEL_") and len(parts) == 4 and parts[2] in _ACC_TO_AXIS and parts[3] == "MAX":
        return _ACC_TO_AXIS[parts[2]], "acc_max_dps2", 0.01
    if name.startswith("ATC_ACC_") and len(parts) == 4 and parts[2] in _ACC_TO_AXIS and parts[3] == "MAX":
        return _ACC_TO_AXIS[parts[2]], "acc_max_dps2", 1.0
    if name == "INS_GYRO_FILTER":
        return None, "gyro_filter", 1.0
    if name == "MOT_THST_HOVER":
        return None, "thst_hover", 1.0
    return None


# ------------------------------------------------------------------------------ writer

def _writer(with_pid=True, with_ang=True, with_atun=False):
    w = LogWriter()
    w.fmt(96, "PARM", "QNff", "TimeUS,Name,Value,Default")
    w.fmt(97, "MSG", "QZ", "TimeUS,Message")
    w.fmt(64, "EV", "QB", "TimeUS,Id")
    w.fmt(207, "MODE", "QBBB", "TimeUS,ModeNum,Rsn,ThrCrs")
    w.fmt(208, "RCIN", "QHHHH", "TimeUS,C1,C2,C3,C4")
    w.fmt(205, "CTUN", "Qfff", "TimeUS,ThO,ThH,BAlt")
    w.fmt(206, "ATT", "Qffffff", "TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw")
    w.fmt(210, "RATE", "Qfffffffff", "TimeUS,RDes,R,ROut,PDes,P,POut,YDes,Y,YOut")
    if with_ang:
        w.fmt(211, "ANG", "Qfffffff", "TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,Dt")
    if with_pid:
        w.fmt(212, "PIDR", "QffffffffffB", "TimeUS,Tar,Act,Err,P,I,D,FF,DFF,Dmod,SRate,Flags")
        w.fmt(213, "PIDP", "QffffffffffB", "TimeUS,Tar,Act,Err,P,I,D,FF,DFF,Dmod,SRate,Flags")
        w.fmt(214, "PIDY", "QffffffffffB", "TimeUS,Tar,Act,Err,P,I,D,FF,DFF,Dmod,SRate,Flags")
    if with_atun:
        w.fmt(215, "ATUN", "QBBfffffff", "TimeUS,Axis,TuneStep,Targ,Min,Max,RP,RD,SP,ddt")
        w.fmt(216, "ATDE", "Qff", "TimeUS,Angle,Rate")
    return w


def _header(w, params, banner=BANNER):
    w.msg("MSG", TimeUS=20, Message=banner)
    for i, (k, v) in enumerate(params.items()):
        w.msg("PARM", TimeUS=30 + i, Name=k, Value=float(v), Default=float("nan"))


def _per_axis(x, axes, kind):
    """A single Plant / gain set for every axis, or a {axis: ...} mapping."""
    if isinstance(x, dict) and any(a in x for a in AXES):
        missing = [a for a in axes if a not in x]
        if missing:
            raise ValueError(f"{kind}: no entry for {missing}")
        return {a: x[a] for a in axes}
    return {a: x for a in axes}


def _us(t):
    return int(round(t * 1e6))


# ------------------------------------------------------------------------------- stick

def step_stick(rng, n, dt, amp_deg=(5.0, 20.0), hold_s=(0.3, 1.5), release_s=(0.5, 2.0), quiet_s=(0.0, 0.0)):
    """A deterministic sequence of pilot angle steps: ± a random amplitude, held for a
    random time, released to zero for a random time. Returns a length-n array (deg)."""
    cmd = np.zeros(n)
    j = int(round(quiet_s[0] / dt))
    while j < n:
        amp = rng.uniform(*amp_deg) * (1.0 if rng.uniform() < 0.5 else -1.0)
        hold = int(round(rng.uniform(*hold_s) / dt))
        rel = int(round(rng.uniform(*release_s) / dt))
        cmd[j:j + hold] = amp
        j += hold + rel
    return cmd


def chirp_rate(n, dt, f0=0.05, f1=5.0, amp_dps=30.0):
    """A linear chirp on the rate target, f0 → f1 over the whole span."""
    t = np.arange(n) * dt
    T = max(n * dt, dt)
    phase = 2.0 * np.pi * (f0 * t + 0.5 * (f1 - f0) / T * t * t)
    return amp_dps * np.sin(phase)


# ---------------------------------------------------------------------------- fast_log

def fast_log(plant, gains, seconds=120.0, loop_hz=400, seed=1, stick="steps", axes=AXES, pid_hz=None,
             with_ang=True, params_extra=None, lograte_hz=None, param_change_at=None, noise_dps=0.3,
             t_takeoff=10.0, banner=BANNER):
    """A synthetic copter log with the rate loops logged fast.

    plant           a `tunesim.Plant` for every axis, or {axis: Plant}
    gains           a gain dict (GainSet names) for every axis, or {axis: dict}
    seconds         airborne time (EV 28 → EV 18); the log has 5 s of disarmed ground
                    time, arming at 5 s, take-off at `t_takeoff`, landing at
                    `t_takeoff + seconds`, disarm 2 s later and 1 s more of ground
    loop_hz         SCHED_LOOP_RATE; the plants must be built at the same rate
    seed            `numpy.random.default_rng(seed)` for the stick and the noise
    stick           "steps" - shaped angle steps (±5-20°, 0.3-1.5 s holds, releases),
                    2 s of quiet after take-off and before landing;
                    "hover" - zero stick; "chirp" - a 0.05-5 Hz chirp on the rate target
    axes            which axes get excitation (the others fly zero stick; all three are
                    always simulated and logged)
    pid_hz          PIDx / RATE / ANG logging rate (default = loop rate, i.e.
                    LOG_BITMASK bit 0); `pid_hz=10` is the standard 10 Hz
    with_ang        write ANG (at the PID rate); ATT is always written at `lograte_hz`
    params_extra    extra {name: value} PARM records at boot (override the defaults)
    lograte_hz      rate of the slow messages ATT/CTUN/RCIN (default 10 Hz)
    param_change_at {t_s: {name: value}}: PARM written at that time and, for gain
                    parameters, applied to the running controller (a GCS change in flight)
    noise_dps       rms of the gaussian measurement noise added to the gyro (Act)

    Returns the LogWriter (`.write(path)` / `.bytes()`).
    """
    loop_hz = float(loop_hz)
    dt = 1.0 / loop_hz
    plants = {a: p.copy() for a, p in _per_axis(plant, AXES, "plant").items()}
    gset = {a: gain_dict(g) for a, g in _per_axis(gains, AXES, "gains").items()}
    for a in AXES:
        if abs(plants[a].loop_hz - loop_hz) > 1e-9:
            raise ValueError(f"plant for {a} is at {plants[a].loop_hz} Hz, log at {loop_hz} Hz")
    pid_hz = float(pid_hz or loop_hz)
    lograte_hz = float(lograte_hz or 10.0)
    dec_pid = max(1, int(round(loop_hz / pid_hz)))
    dec_slow = max(1, int(round(loop_hz / lograte_hz)))
    fast = dec_pid == 1

    t_arm, t_off, t_land = 5.0, float(t_takeoff), float(t_takeoff) + float(seconds)
    t_dis, t_end = t_land + 2.0, t_land + 3.0
    n = int(round(t_end * loop_hz))
    j_arm, j_off, j_land, j_dis = (int(round(x * loop_hz)) for x in (t_arm, t_off, t_land, t_dis))

    rng = np.random.default_rng(seed)
    n_fly = j_land - j_off
    cmd = {}
    for a in AXES:
        if a in axes and stick == "steps":
            c = step_stick(rng, n_fly, dt, quiet_s=(2.0, 2.0))
            c[-int(2.0 * loop_hz):] = 0.0
        elif a in axes and stick == "chirp":
            c = chirp_rate(n_fly, dt)
        elif stick in ("steps", "hover", "chirp"):
            c = np.zeros(n_fly)
        else:
            raise ValueError(f"stick {stick!r}: expected steps, hover or chirp")
        cmd[a] = c
    noise = {a: rng.normal(0.0, noise_dps, n) if noise_dps > 0 else np.zeros(n) for a in AXES}

    loops = {a: ClosedLoop(plants[a], gset[a], loop_hz) for a in AXES}
    thh = gset["roll"]["thst_hover"] if gset["roll"]["thst_hover"] is not None else 0.35

    params = common_params(gset["roll"], loop_hz, fast, gset["roll"]["gmbk"])
    for a in AXES:
        params.update(gain_params(a, gset[a]))
    if params_extra:
        params.update(params_extra)
    w = _writer(with_pid=True, with_ang=with_ang)
    _header(w, params, banner)
    w.msg("MODE", TimeUS=_us(0.001), ModeNum=ALT_HOLD, Rsn=1, ThrCrs=0)
    changes = sorted((float(t), dict(v)) for t, v in (param_change_at or {}).items())
    ci = 0

    ev = {j_arm: 10, j_off: 28, j_land: 18, j_dis: 11}
    angle_max = params.get("ATC_ANGLE_MAX", 4500.0) / 100.0
    rows = {}
    for j in range(n):
        t = j * dt
        us = _us(t)
        if j in ev:
            w.msg("EV", TimeUS=us, Id=ev[j])
        while ci < len(changes) and changes[ci][0] <= t:
            for name, val in changes[ci][1].items():
                w.msg("PARM", TimeUS=us, Name=name, Value=float(val), Default=float("nan"))
                hit = param_to_gain(name)
                if hit is None:
                    continue
                ax, field, scale = hit
                for a in ([ax] if ax else AXES):
                    gset[a][field] = float(val) * scale
                    loops[a].set_gains(gset[a])
            ci += 1
        flying = j_off <= j < j_land
        armed = j_arm <= j < j_dis
        for a in AXES:
            loop = loops[a]
            meas = loop.rate + noise[a][j]
            if flying and stick == "chirp" and a in axes:
                rt = cmd[a][j - j_off]
                loop.angle_target = loop.angle           # no angle loop on a chirp axis
                loop.rate_ff = 0.0
            else:
                rt = loop.rate_target_from_angle(cmd[a][j - j_off] if flying else 0.0)
            out = loop.step(rt, meas)
            pid = loop.pid
            rows[a] = (rt, meas, out, pid, loop.angle, loop.angle_target)
        if armed and j % dec_pid == 0:
            for a in AXES:
                rt, meas, out, pid, ang, des = rows[a]
                # the PID works in rad/s and PIDx logs Tar/Act/Err as it holds them, rad/s
                # (PIDR.Act x 57.2958 == RATE.R on brisket-t1.bin; sources §2)
                w.msg(_AX[a]["pid"], TimeUS=us, Tar=pid.target, Act=math.radians(meas), Err=pid.error,
                      P=pid.P, I=pid.I, D=pid.D, FF=pid.FF, DFF=pid.DFF, Dmod=pid.Dmod, SRate=pid.slew_rate,
                      Flags=pid.flags)
            w.msg("RATE", TimeUS=us,
                  RDes=rows["roll"][0], R=rows["roll"][1], ROut=rows["roll"][2],
                  PDes=rows["pitch"][0], P=rows["pitch"][1], POut=rows["pitch"][2],
                  YDes=rows["yaw"][0], Y=rows["yaw"][1], YOut=rows["yaw"][2])
            if with_ang:
                w.msg("ANG", TimeUS=us, DesRoll=rows["roll"][5], Roll=rows["roll"][4],
                      DesPitch=rows["pitch"][5], Pitch=rows["pitch"][4],
                      DesYaw=rows["yaw"][5], Yaw=rows["yaw"][4], Dt=dt)
        if j % dec_slow == 0:
            w.msg("ATT", TimeUS=us, DesRoll=rows["roll"][5], Roll=rows["roll"][4],
                  DesPitch=rows["pitch"][5], Pitch=rows["pitch"][4],
                  DesYaw=rows["yaw"][5], Yaw=rows["yaw"][4])
            w.msg("CTUN", TimeUS=us, ThO=thh if flying else 0.0, ThH=thh, BAlt=5.0 if flying else 0.0)
            rc = {}
            for a in AXES:
                c = cmd[a][j - j_off] if (flying and stick != "chirp") else 0.0
                rc[_AX[a]["rc"]] = int(round(1500 + 500.0 * c / angle_max))
            w.msg("RCIN", TimeUS=us, C1=rc["C1"], C2=rc["C2"], C3=1500 if flying else 1000, C4=rc["C4"])
    return w


# ------------------------------------------------------------------------ autotune_log

def autotune_log(plant, gains, aggr=0.075, gmbk=0.25, axes=("roll",), min_d=None, loop_hz=400, seed=1,
                 angle_max_deg=45.0, truncate_after_step=None, level_s=0.75, noise_dps=0.0, banner=BANNER):
    """A log of an AutoTune session produced by `tunesim.autotune` on `plant`.

    Writes, per axis in `axes` order: the twitches as `ATUN` rows (one per twitch, the
    gains *tested*, Targ/Min/Max in deg or deg/s, `ddt` unscaled cdeg/s²), `ATDE` and
    `RATE`/`PIDx` at loop rate during every test, `EV` 30 at the start, 35/34 where the
    simulator raised them, 33 on success, 37 SAVEDGAINS at disarm, the firmware's `MSG`
    lines (`AutoTune: Roll Rate: P:..., I:..., D:...`, `AutoTune: Roll Angle P:..., Max
    Accel:...`, `AutoTune: Saved gains for Roll`) and the post-save `PARM` values.
    `AUTOTUNE_GMBK` is in the PARM block unless `gmbk` is None (a 4.x log, and then no
    backoff is applied - see `tunesim.autotune`). `truncate_after_step` (a step name, e.g.
    "RATE_P_UP") ends the log right after the last axis' last twitch of that step, with
    no SUCCESS/SAVEDGAINS/MSG/PARM - an incomplete session.

    Returns (LogWriter, {axis: autotune() result}).
    """
    loop_hz = float(loop_hz)
    dt = 1.0 / loop_hz
    plants = _per_axis(plant, axes, "plant")
    gset = {a: gain_dict(g) for a, g in _per_axis(gains, AXES, "gains").items()}
    for a in AXES:
        gset[a]["aggr"] = aggr
        gset[a]["gmbk"] = gmbk
        if min_d is not None:
            gset[a]["min_d"] = min_d
    results = {a: autotune(plants[a], gset[a], aggr=aggr, gmbk=gmbk, min_d=min_d, axis=a,
                           angle_max_deg=angle_max_deg, loop_hz=loop_hz, keep_traces=True) for a in axes}
    rng = np.random.default_rng(seed)

    params = common_params(gset["roll"], loop_hz, False, gmbk)
    for a in AXES:
        params.update(gain_params(a, gset[a]))
    w = _writer(with_pid=True, with_ang=False, with_atun=True)
    _header(w, params, banner)
    thh = params["MOT_THST_HOVER"]
    t = 0.0

    def slow(t0, t1):
        """ATT/CTUN/RCIN at 10 Hz over [t0, t1)."""
        k = math.ceil(t0 * 10.0 - 1e-9)
        while k / 10.0 < t1:
            us = _us(k / 10.0)
            flying = t_off <= k / 10.0 < t_land_holder[0]
            w.msg("ATT", TimeUS=us, DesRoll=0.0, Roll=0.0, DesPitch=0.0, Pitch=0.0, DesYaw=0.0, Yaw=0.0)
            w.msg("CTUN", TimeUS=us, ThO=thh if flying else 0.0, ThH=thh, BAlt=5.0 if flying else 0.0)
            w.msg("RCIN", TimeUS=us, C1=1500, C2=1500, C3=1500 if flying else 1000, C4=1500)
            k += 1

    t_arm, t_off = 5.0, 10.0
    t_land_holder = [float("inf")]
    w.msg("MODE", TimeUS=_us(0.001), ModeNum=ALT_HOLD, Rsn=1, ThrCrs=0)
    w.msg("EV", TimeUS=_us(t_arm), Id=10)
    w.msg("EV", TimeUS=_us(t_off), Id=28)
    t_tune = 15.0
    slow(0.0, t_tune)
    w.msg("MODE", TimeUS=_us(t_tune), ModeNum=AUTOTUNE_MODE, Rsn=1, ThrCrs=0)
    w.msg("EV", TimeUS=_us(t_tune), Id=30)
    t = t_tune
    truncated = False
    done_axes = []
    for a in axes:
        r = results[a]
        ev_iter = iter(r["events"][1:])       # 30 already written; the rest in order
        cut = None
        if truncate_after_step and a == axes[-1]:
            idx = [i for i, x in enumerate(r["twitches"]) if x["step"] == truncate_after_step]
            if not idx:
                raise ValueError(f"truncate_after_step {truncate_after_step!r}: no twitch of that step on {a}")
            cut = idx[-1]
        for ti, rec in enumerate(r["twitches"]):
            t0 = t + level_s
            slow(t, t0)
            tp = rec["trace_pid"]
            for i in range(tp.shape[0]):
                us = _us(t0 + i * dt)
                rdes, tar, act, err, P, I, D, FF, DFF, Dmod, SRate, Flags, out, ang = tp[i]
                act_n = act + (rng.normal(0.0, noise_dps) if noise_dps > 0 else 0.0)
                w.msg("ATDE", TimeUS=us, Angle=rec["trace_angle"][i], Rate=rec["trace_rate"][i])
                cols = {ax: (0.0, 0.0, 0.0) for ax in AXES}
                cols[a] = (rdes, act_n, out)
                w.msg("RATE", TimeUS=us, RDes=cols["roll"][0], R=cols["roll"][1], ROut=cols["roll"][2],
                      PDes=cols["pitch"][0], P=cols["pitch"][1], POut=cols["pitch"][2],
                      YDes=cols["yaw"][0], Y=cols["yaw"][1], YOut=cols["yaw"][2])
                for ax in AXES:
                    if ax == a:
                        w.msg(_AX[ax]["pid"], TimeUS=us, Tar=math.radians(tar), Act=math.radians(act_n),
                              Err=math.radians(err), P=P, I=I, D=D, FF=FF,
                              DFF=DFF, Dmod=Dmod, SRate=SRate, Flags=int(Flags))
                    else:
                        w.msg(_AX[ax]["pid"], TimeUS=us, Tar=0.0, Act=0.0, Err=0.0, P=0.0, I=0.0, D=0.0,
                              FF=0.0, DFF=0.0, Dmod=1.0, SRate=0.0, Flags=0)
            t1 = t0 + tp.shape[0] * dt
            slow(t0, t1)
            if rec["outcome"] == "UPDATE_GAINS":
                w.msg("ATUN", TimeUS=_us(t1), Axis=rec["axis_id"], TuneStep=rec["step_id"], Targ=rec["targ"],
                      Min=rec["min"], Max=rec["max"], RP=rec["rp"], RD=rec["rd"], SP=rec["sp"], ddt=rec["ddt"])
            t = t1
            if cut is not None and ti == cut:
                truncated = True
                break
        # limit / failure events, then the axis report
        for e in ev_iter:
            if e in (34, 35):
                w.msg("EV", TimeUS=_us(t), Id=e)
        if truncated:
            break
        if r["complete"]:
            done_axes.append(a)
            for m in r["messages"]:
                w.msg("MSG", TimeUS=_us(t), Message=m)
        else:
            for m in r["messages"]:
                w.msg("MSG", TimeUS=_us(t), Message=m)
            w.msg("EV", TimeUS=_us(t), Id=34)
            break
    t += level_s
    slow(t - level_s, t)
    all_done = (not truncated) and all(results[a]["complete"] for a in axes)
    if all_done:
        w.msg("EV", TimeUS=_us(t), Id=33)
        w.msg("MSG", TimeUS=_us(t), Message="AutoTune: Success")
    t_land = t + 3.0
    t_land_holder[0] = t_land
    w.msg("MODE", TimeUS=_us(t + 0.5), ModeNum=ALT_HOLD, Rsn=1, ThrCrs=0)
    slow(t, t_land)
    w.msg("EV", TimeUS=_us(t_land), Id=18)
    t_dis = t_land + 2.0
    slow(t_land, t_dis)
    w.msg("EV", TimeUS=_us(t_dis), Id=11)
    if all_done:
        w.msg("EV", TimeUS=_us(t_dis + 0.001), Id=37)
        w.msg("MSG", TimeUS=_us(t_dis + 0.001), Message="AutoTune: Saved gains for " + " ".join(_AX[a]["name"] for a in done_axes))
        k = 0
        for a in done_axes:
            for name, val in results[a]["params"].items():
                w.msg("PARM", TimeUS=_us(t_dis + 0.002) + k, Name=name, Value=float(val), Default=float("nan"))
                k += 1
    slow(t_dis, t_dis + 1.0)
    for a in axes:
        results[a]["truncated"] = truncated
        results[a]["saved"] = all_done and a in done_axes
    return w, results
