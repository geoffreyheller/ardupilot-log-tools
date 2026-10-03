"""WP2 of docs/pid-tuning-plan.md: the simulator (dflog/tunesim.py) and the synthetic
fast-logged copter logs (tests/tunesynth.py).

The simulator is the test oracle for `alog tune`: these tests pin it against ArduPilot's
own arithmetic (filter alphas, steady states), check that its logs parse and window like
real ones, that the AutoTune replica completes and moves the way the firmware's rules say
it must, and that everything is byte-for-byte deterministic.

    python tests/test_tune_sim.py
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

from dflog import Log, airborne_window, flights, hover_chunks                  # noqa: E402
from dflog.tunesim import (ACPID, Plant, ClosedLoop, autotune, calc_lowpass_alpha_dt,   # noqa: E402
                           gain_dict, max_rate_step_bf, AUTOTUNE, GAIN_FIELDS)
from tunesynth import (fast_log, autotune_log, PLANT_5IN, PLANT_10IN, GAINS_5IN, GAINS_10IN,  # noqa: E402
                       gain_params)

TMP = tempfile.mkdtemp(prefix="dflog-tunesim-")


def _load(w, name):
    return Log(w.write(os.path.join(TMP, name)), use_cache=False)


# AC_PID param ranges (sources §2)
RANGES = dict(roll=dict(p=(0.01, 0.5), d=(0.0, 0.05)), pitch=dict(p=(0.01, 0.5), d=(0.0, 0.05)),
              yaw=dict(p=(0.10, 2.5), d=(0.0, 0.02)))


# ------------------------------------------------------------------------- AC_PID

def test_alphas_match_ardupilot():
    """calc_lowpass_alpha_dt: rc = 1/(2π hz), alpha = dt/(dt + rc); 0 Hz = off."""
    dt = 1.0 / 400.0
    rc = 1.0 / (2.0 * math.pi * 20.0)
    assert calc_lowpass_alpha_dt(dt, 20.0) == pytest.approx(dt / (dt + rc), rel=1e-12)
    assert calc_lowpass_alpha_dt(dt, 20.0) == pytest.approx(0.2390, abs=5e-4)
    assert calc_lowpass_alpha_dt(dt, 0.0) == 1.0
    assert calc_lowpass_alpha_dt(0.0, 20.0) == 0.0
    pid = ACPID(0.135, 0.135, 0.0036, fltt=20.0, flte=0.0, fltd=20.0)
    assert pid.get_filt_T_alpha(dt) == pytest.approx(calc_lowpass_alpha_dt(dt, 20.0))
    assert pid.get_filt_E_alpha(dt) == 1.0
    assert pid.get_filt_D_alpha(dt) == pid.get_filt_T_alpha(dt)


def test_acpid_filters_and_integrator():
    """FLTT filters the target, FLTE the error, the integrator clamps at IMAX and stops
    integrating into a limit; the RESET flag is on the first update only."""
    dt = 1.0 / 400.0
    pid = ACPID(1.0, 1.0, 0.0, imax=0.1, fltt=20.0, flte=0.0, fltd=20.0)
    pid.update_all(1.0, 0.0, dt)
    assert pid.flags & 4 and pid.target == 1.0          # reset seeds the filter
    pid.update_all(0.0, 0.0, dt)
    a = calc_lowpass_alpha_dt(dt, 20.0)
    assert pid.target == pytest.approx(1.0 - a) and not (pid.flags & 4)
    # integrator: ki * error * dt per step, clamped at IMAX
    pid = ACPID(0.0, 1.0, 0.0, imax=0.01, fltt=0.0, flte=0.0, fltd=0.0)
    for _ in range(400):
        pid.update_all(1.0, 0.0, dt)
    assert pid.I == pytest.approx(0.01)
    # anti-windup: with limit set and the error pushing further in, I does not move
    pid = ACPID(0.0, 1.0, 0.0, imax=1.0, fltt=0.0, flte=0.0, fltd=0.0)
    pid.update_all(1.0, 0.0, dt)
    i0 = pid.I
    pid.update_all(1.0, 0.0, dt, limit=True)
    assert pid.I == i0 and pid.flags & 1
    pid.update_all(-1.0, 0.0, dt, limit=True)            # error opposes the integrator: allowed
    assert pid.I < i0


def test_unit_rate_step_steady_state():
    """A 1 deg/s rate step: with I active the loop settles at 1.0; with I = 0 at the
    P-only DC gain L/(1+L), L = kP * k[rad/s per unit]. Dmod stays 1.0, no limit."""
    plant = Plant(k=2000.0, tau1=0.25, tau2=0.015, delay=0.005)
    gains = dict(rat_p=0.135, rat_i=0.135, rat_d=0.0036, rat_ff=0.0, fltd=20.0, fltt=20.0, flte=0.0,
                 imax=0.5, ang_p=4.5, acc_max_dps2=1100.0)
    loop = ClosedLoop(plant.copy(), gains)
    res = loop.run(10.0, rate_cmd=np.ones(4000))
    assert res["act"][-1] == pytest.approx(1.0, abs=1e-3)
    assert res["act"][1600] == pytest.approx(1.0, abs=1e-2)          # and most of the way there by 4 s
    assert res["tar"][-1] == pytest.approx(1.0, abs=1e-6)
    assert np.all(res["dmod"] == 1.0) and not np.any(res["flags"] & 1)
    L = 0.135 * math.radians(2000.0)
    loop = ClosedLoop(plant.copy(), dict(gains, rat_i=0.0))
    res = loop.run(4.0, rate_cmd=np.ones(1600))
    assert res["act"][-1] == pytest.approx(L / (1.0 + L), rel=1e-3)
    # the helper gives the same thing
    t, act = plant.true_closed_loop_step(gains, seconds=10.0)
    assert act[-1] == pytest.approx(1.0, abs=1e-3) and t[1] - t[0] == pytest.approx(1 / 400)


def test_plant_frequency_response_and_delay():
    p = Plant(k=1000.0, tau1=0.25, tau2=0.015, delay=0.005)
    assert p.delay_samples == 2 and p.delay_s_effective == pytest.approx(0.005)
    G = p.freq_response([0.001, 1.0 / (2 * math.pi * 0.25)])
    assert abs(G[0]) == pytest.approx(1000.0, rel=1e-4)
    assert abs(G[1]) == pytest.approx(1000.0 / math.sqrt(2), rel=2e-2)     # -3 dB at the tau1 pole
    # a step of 1 unit reaches k at DC and nothing moves inside the delay
    p.reset()
    y = [p.step(1.0) for _ in range(2000)]
    assert y[0] == 0.0 and y[1] == 0.0 and y[2] > 0.0
    assert y[-1] == pytest.approx(1000.0, rel=1e-2)


def test_sqrt_controller_and_slew_rate():
    from dflog.tunesim import sqrt_controller, SlewRate
    # linear inside a/p^2, sqrt outside, clamped to |e|/dt
    assert sqrt_controller(1.0, 4.5, 720.0, 0.0025) == pytest.approx(4.5)
    big = sqrt_controller(100.0, 4.5, 720.0, 0.0025)
    assert big < 450.0 and big == pytest.approx(math.sqrt(2 * 720 * (100 - 720 / 4.5 ** 2 / 2)))
    assert sqrt_controller(1.0, 4.5, 0.0, 0.0025) == 4.5
    # a ringing P+D reads a larger SRate than a quiet one, and SRate is >= 0
    s = SlewRate()
    quiet = [s.update(0.01 * math.sin(2 * math.pi * 0.5 * i / 400), 1 / 400) for i in range(800)]
    s = SlewRate()
    ring = [s.update(0.08 * math.sin(2 * math.pi * 20 * i / 400), 1 / 400) for i in range(800)]
    # true slew 0.08 * 2pi * 20 = 10 /s, read through the 25 Hz derivative filter
    assert 10.0 > max(ring) > 5.0 > max(quiet) >= 0.0, (max(ring), max(quiet))


def test_gain_dict_accepts_gainset_names():
    import dataclasses

    @dataclasses.dataclass
    class GainSet:
        axis: str = "roll"
        rat_p: float = 0.2
        rat_i: float = 0.2
        rat_d: float = 0.004
        rat_ff: float = 0.0
        fltd: float = 30.0
        fltt: float = 30.0
        flte: float = 0.0
        smax: float = 0.0
        imax: float = 0.5
        ang_p: float = 6.0
        acc_max_dps2: float = 1500.0
        ff_enab: bool = True
        gyro_filter: float = 60.0
        thst_hover: float = 0.3
        loop_hz: float = 400.0
        aggr: float = 0.1
        gmbk: float = 0.25
        min_d: float = 0.0005
        defaulted: list = dataclasses.field(default_factory=list)
        param_names: dict = dataclasses.field(default_factory=dict)

    g = gain_dict(GainSet())
    assert set(g) == set(GAIN_FIELDS) and g["rat_p"] == 0.2 and g["aggr"] == 0.1
    assert gain_dict(dataclasses.asdict(GainSet()))["ang_p"] == 6.0
    assert gain_dict({})["rat_p"] == 0.135          # defaults fill the gaps


def test_max_rate_step_matches_formula():
    dt = 1 / 400
    a = calc_lowpass_alpha_dt(dt, 37.5)
    want = 2 * 0.25 / (((1 - a) ** 3 * a * 0.0036) / dt + 0.135)
    assert max_rate_step_bf(0.135, 0.0036, 0.0, 37.5, dt, 0.25) == pytest.approx(want)
    assert max_rate_step_bf(0.135, 0.0036, 0.0, 37.5, dt, 0.05) == max_rate_step_bf(0.135, 0.0036, 0.0, 37.5, dt, 0.1)


# ------------------------------------------------------------------------ fast_log

def test_fast_log_parses_and_windows():
    w = fast_log(PLANT_5IN, GAINS_5IN, seconds=30.0, seed=3)
    log = _load(w, "fast5.bin")
    assert log.diagnostics.ok, log.diagnostics.render()
    q = log.quality()
    assert not q.has("LOG_GAP") and not q.warnings and not q.errors, q.render()   # PARM.Default NaN is an allowed info
    for m in ("PIDR", "PIDP", "PIDY", "RATE", "ANG"):
        assert log.rate_hz(m) == pytest.approx(400.0, rel=1e-3), (m, log.rate_hz(m))
    assert log.rate_hz("ATT") == pytest.approx(10.0, rel=1e-3)
    w_ = airborne_window(log)
    assert w_.method == "EV NOT_LANDED->LAND_COMPLETE" and w_.duration == pytest.approx(30.0, abs=0.01)
    assert len(flights(log)) == 1
    assert log.vehicle() == "copter" or "Copter" in log.firmware()
    p = log.params()
    for name, v in gain_params("roll", GAINS_5IN["roll"]).items():
        assert p[name] == pytest.approx(v, rel=1e-6), name
    assert p["ATC_ACCEL_R_MAX"] == pytest.approx(168600.0) and p["LOG_BITMASK"] == 180223.0
    assert p["SCHED_LOOP_RATE"] == 400.0 and p["AUTOTUNE_GMBK"] == 0.25
    # the signals are what the plan reads: Tar tracks RDes through FLTT, Act follows Tar,
    # ROut is the PID sum clipped, Flags never limited on a benign stick
    pidr, rate = w_.clip(log.df("PIDR")), w_.clip(log.df("RATE"))
    assert np.degrees(np.abs(pidr["Tar"].values)).max() > 20.0      # there was excitation
    assert np.corrcoef(pidr["Tar"].values, pidr["Act"].values)[0, 1] > 0.95
    # PIDx is rad/s, RATE deg/s, the same gyro sample (as brisket-t1.bin logs them)
    assert np.allclose(np.degrees(pidr["Act"].values), rate["R"].values, rtol=1e-4, atol=1e-3)
    s = (pidr["P"] + pidr["I"] + pidr["D"] + pidr["FF"] + pidr["DFF"]).values
    assert np.abs(rate["ROut"].values - s).max() < 1e-5
    assert (pidr["Flags"].values & 1).mean() < 0.01
    assert np.all(pidr["Dmod"].values == 1.0)
    # ANG carries the shaped target: it lags the raw stick and never jumps
    ang = w_.clip(log.df("ANG"))
    assert np.abs(np.diff(ang["DesRoll"].values)).max() < 2.0


def test_fast_log_10hz_and_hover():
    w = fast_log(PLANT_10IN, GAINS_10IN, seconds=20.0, pid_hz=10, stick="hover")
    log = _load(w, "slow10.bin")
    assert log.diagnostics.ok
    assert log.rate_hz("PIDR") == pytest.approx(10.0, rel=1e-3)
    assert log.rate_hz("RATE") == pytest.approx(10.0, rel=1e-3)
    assert log.params()["LOG_BITMASK"] == 180222.0
    pidr = airborne_window(log).clip(log.df("PIDR"))
    assert np.degrees(np.abs(pidr["Tar"].values)).max() < 1.0        # no excitation at all
    assert len(hover_chunks(log)) >= 1                              # sticks centred, ALT_HOLD


def test_fast_log_chirp_and_param_change():
    w = fast_log(PLANT_5IN, GAINS_5IN, seconds=20.0, stick="chirp", axes=("roll",),
                 param_change_at={20.0: {"ATC_RAT_RLL_P": 0.2}})
    log = _load(w, "chirp.bin")
    assert log.diagnostics.ok
    ch = log.param_changes()
    assert any(n == "ATC_RAT_RLL_P" and new == pytest.approx(0.2) for _t, n, _old, new in ch), ch
    assert log.param_at("ATC_RAT_RLL_P", 15.0) == pytest.approx(0.135)
    assert log.param_at("ATC_RAT_RLL_P", 25.0) == pytest.approx(0.2)
    pidr = log.df("PIDR")
    before = pidr[(pidr["t"] > 12) & (pidr["t"] < 19)]
    after = pidr[(pidr["t"] > 21) & (pidr["t"] < 28)]
    # P term / error ratio is the gain: it moved from 0.135 to 0.2 (Err is logged in rad/s)
    kp_b = np.median(before["P"].values / before["Err"].values)
    kp_a = np.median(after["P"].values / after["Err"].values)
    assert kp_b == pytest.approx(0.135, rel=1e-3) and kp_a == pytest.approx(0.2, rel=1e-3)
    # the chirp is on the rate target of roll only
    assert np.degrees(np.abs(pidr["Tar"].values)).max() > 25.0
    assert np.degrees(np.abs(log.df("PIDP")["Tar"].values)).max() < 1.0


# ------------------------------------------------------------------------ autotune

def test_autotune_completes_on_5in_plant():
    r = autotune(PLANT_5IN["roll"], GAINS_5IN["roll"], axis="roll")
    assert r["aborted"] is None, r["aborted"]
    assert r["steps_completed"] == ["RATE_D_UP", "RATE_D_DOWN", "RATE_P_UP", "ANGLE_P_DOWN", "ANGLE_P_UP"]
    assert r["complete"] and r["n_twitches"] < 400
    lo, hi = RANGES["roll"]["p"]
    assert lo <= r["rat_p"] <= hi, r["rat_p"]
    lo, hi = RANGES["roll"]["d"]
    assert lo <= r["rat_d"] <= hi and r["rat_d"] >= 0.0005 * 0.75, r["rat_d"]
    assert r["rat_i"] == r["rat_p"]
    assert AUTOTUNE["SP_MIN"] <= r["ang_p"] <= AUTOTUNE["SP_MAX"]
    assert r["acc_max_dps2"] >= 40.0
    # the backoff: saved = found * (1 - GMBK); angle P also * (1 - AGGR)
    found = r["gains_per_step"]["RATE_P_UP"]
    assert r["rat_p"] == pytest.approx(found["rp"] * 0.75) and r["rat_d"] == pytest.approx(found["rd"] * 0.75)
    assert r["ang_p"] == pytest.approx(r["gains_per_step"]["ANGLE_P_UP"]["sp"] * 0.75 * 0.925)
    # every twitch record has what ATUN logs
    tw = r["twitches"]
    assert all(k in tw[0] for k in ("axis", "step", "targ", "min", "max", "rp", "rd", "sp", "ddt", "passed"))
    assert sum(t["passed"] for t in tw if t["step"] == "RATE_D_UP") >= 4
    assert abs(r["final_overshoot"]) < 0.5 and 0.0 <= r["final_bounce"] < 0.5
    # the firmware's messages
    assert r["messages"][-2] == f"AutoTune: Roll Rate: P:{r['rat_p']:0.3f}, I:{r['rat_i']:0.3f}, D:{r['rat_d']:0.4f}"
    assert r["messages"][-1].startswith("AutoTune: Roll Angle P:")


def test_autotune_10in_and_yaw():
    r = autotune(PLANT_10IN["roll"], GAINS_10IN["roll"], axis="roll")
    assert r["complete"], r["aborted"]
    assert 0.01 <= r["rat_p"] <= 0.5 and 0.0 <= r["rat_d"] <= 0.05
    y = autotune(PLANT_5IN["yaw"], GAINS_5IN["yaw"], axis="yaw")
    assert y["complete"], y["aborted"]
    assert 0.1 <= y["rat_p"] <= 2.5 and y["rat_i"] == pytest.approx(0.1 * y["rat_p"])
    assert y["rat_d"] == 0.0 and AUTOTUNE["RLPF_MIN"] <= y["flte"] <= AUTOTUNE["RLPF_MAX"]
    assert y["messages"][-2].startswith("AutoTune: Yaw Rate: P:")
    assert all(t["rd"] == pytest.approx(t["rd"]) and 1.0 <= t["rd"] <= 5.0 for t in y["twitches"])   # RD = FLTE for yaw


def test_autotune_aggr_raises_d():
    """More aggressiveness asks for more bounce-back before D_UP passes, so D rises."""
    lo = autotune(PLANT_5IN["roll"], GAINS_5IN["roll"], aggr=0.05)
    hi = autotune(PLANT_5IN["roll"], GAINS_5IN["roll"], aggr=0.10)
    assert lo["complete"] and hi["complete"]
    assert hi["rat_d"] > lo["rat_d"], (lo["rat_d"], hi["rat_d"])
    assert hi["constants"]["aggr"] == 0.10
    # GMBK scales the saved rate gains linearly
    g0 = autotune(PLANT_5IN["roll"], GAINS_5IN["roll"], gmbk=0.0)
    g1 = autotune(PLANT_5IN["roll"], GAINS_5IN["roll"], gmbk=0.25)
    assert g1["rat_p"] == pytest.approx(0.75 * g0["rat_p"]) and g1["rat_d"] == pytest.approx(0.75 * g0["rat_d"])


def test_autotune_is_deterministic_and_capped():
    a = autotune(PLANT_5IN["roll"], GAINS_5IN["roll"])
    b = autotune(PLANT_5IN["roll"], GAINS_5IN["roll"])
    assert a["rat_p"] == b["rat_p"] and a["rat_d"] == b["rat_d"] and a["n_twitches"] == b["n_twitches"]
    c = autotune(PLANT_5IN["roll"], GAINS_5IN["roll"], max_twitches=5)
    assert c["aborted"] and "max_twitches" in c["aborted"] and c["n_twitches"] == 5 and not c["complete"]


# --------------------------------------------------------------------- autotune_log

def test_autotune_log_parses_and_matches_result():
    w, res = autotune_log(PLANT_5IN, GAINS_5IN, axes=("roll",))
    log = _load(w, "atun.bin")
    assert log.diagnostics.ok, log.diagnostics.render()
    r = res["roll"]
    assert r["saved"] and not r["truncated"]
    atun = log.df("ATUN")
    assert len(atun) == sum(t["outcome"] == "UPDATE_GAINS" for t in r["twitches"])
    assert set(atun["Axis"]) == {0} and set(atun["TuneStep"]) == {0, 1, 2, 4, 5}
    # ATUN rows carry the gains *tested*. The RATE_P_UP backoff is applied when that step
    # completes, so the ANGLE_P_DOWN/UP rows already carry the backed-off RP/RD: the saved
    # rate gains are the last RATE_P_UP row x (1 - GMBK) == the last ANGLE_P_UP row x 1;
    # the saved angle P is the last ANGLE_P_UP row x (1 - GMBK)(1 - AGGR). (This corrects
    # the recipe in reference/pid-tuning-sources.md §1.7, which applies GMBK to the
    # ANGLE_P_UP row's RP/RD as well - that would back off twice.)
    last_p = atun[atun["TuneStep"] == 2].iloc[-1]
    last = atun[atun["TuneStep"] == 5].iloc[-1]
    assert last_p["RP"] * 0.75 == pytest.approx(r["rat_p"], rel=1e-6)
    assert last_p["RD"] * 0.75 == pytest.approx(r["rat_d"], rel=1e-6)
    assert last["RP"] == pytest.approx(r["rat_p"], rel=1e-6) and last["RD"] == pytest.approx(r["rat_d"], rel=1e-6)
    assert last["SP"] * 0.75 * 0.925 == pytest.approx(r["ang_p"], rel=1e-6)
    assert max(4000.0, atun["ddt"].max()) / 100.0 == pytest.approx(r["acc_max_dps2"], rel=1e-6)
    # Targ/Min/Max in deg or deg/s: the rate steps are tens of deg/s, the angle steps < 45 deg
    rate_rows = atun[atun["TuneStep"] <= 2]
    assert 30.0 < rate_rows["Targ"].max() < 200.0
    assert atun[atun["TuneStep"] >= 4]["Targ"].max() <= 45.0
    # MSG text matches the returned gains to 3 decimals, and the saved PARM values match
    msgs = [m for _t, m in log.messages_text()] if hasattr(log, "messages_text") else list(log.df("MSG")["Message"])
    text = "\n".join(str(m) for m in msgs)
    assert f"AutoTune: Roll Rate: P:{r['rat_p']:0.3f}, I:{r['rat_i']:0.3f}, D:{r['rat_d']:0.4f}" in text, text
    assert f"AutoTune: Roll Angle P:{r['ang_p']:0.3f}, Max Accel:{r['acc_max_dps2'] * 100:0.0f}" in text, text
    assert "AutoTune: Saved gains for Roll" in text
    ev = list(log.df("EV")["Id"])
    assert 30 in ev and 33 in ev and 37 in ev and ev.index(37) > ev.index(11)
    p = log.params()
    assert p["ATC_RAT_RLL_P"] == pytest.approx(r["rat_p"], rel=1e-6)
    assert p["ATC_RAT_RLL_I"] == pytest.approx(r["rat_i"], rel=1e-6)
    assert p["ATC_RAT_RLL_D"] == pytest.approx(r["rat_d"], rel=1e-6)
    assert p["ATC_ANG_RLL_P"] == pytest.approx(r["ang_p"], rel=1e-6)
    assert p["ATC_ACCEL_R_MAX"] == pytest.approx(r["acc_max_dps2"] * 100.0, rel=1e-6)
    assert p["AUTOTUNE_AGGR"] == pytest.approx(0.075, rel=1e-6) and p["AUTOTUNE_GMBK"] == 0.25
    assert log.param_at("ATC_RAT_RLL_P", 1.0) == pytest.approx(0.135)          # before the save
    # RATE/PIDR/ATDE at loop rate during the tests
    assert log.rate_hz("ATDE") == pytest.approx(400.0, rel=1e-3)
    assert log.rate_hz("PIDR") == pytest.approx(400.0, rel=1e-3)
    assert airborne_window(log).mask(log.df("ATDE")["t"].values).all()       # every test inside the flight


def test_autotune_log_truncated_and_4x():
    w, res = autotune_log(PLANT_5IN, GAINS_5IN, axes=("roll",), truncate_after_step="RATE_P_UP")
    log = _load(w, "atun_trunc.bin")
    assert log.diagnostics.ok
    assert res["roll"]["truncated"] and not res["roll"]["saved"]
    atun = log.df("ATUN")
    assert set(atun["TuneStep"]) == {0, 1, 2}
    ev = list(log.df("EV")["Id"])
    assert 33 not in ev and 37 not in ev
    assert "Saved gains" not in "\n".join(str(m) for m in log.df("MSG")["Message"])
    assert log.params()["ATC_RAT_RLL_P"] == pytest.approx(0.135)
    # gmbk=None emulates a 4.x log: no AUTOTUNE_GMBK parameter
    w, res = autotune_log(PLANT_5IN, GAINS_5IN, axes=("roll",), gmbk=None)
    log = _load(w, "atun_4x.bin")
    assert "AUTOTUNE_GMBK" not in log.params() and res["roll"]["complete"]


def test_autotune_log_two_axes():
    w, res = autotune_log(PLANT_5IN, GAINS_5IN, axes=("roll", "pitch"))
    log = _load(w, "atun_rp.bin")
    assert log.diagnostics.ok
    atun = log.df("ATUN")
    assert set(atun["Axis"]) == {0, 1}
    text = "\n".join(str(m) for m in log.df("MSG")["Message"])
    assert "AutoTune: Saved gains for Roll Pitch" in text
    assert log.params()["ATC_RAT_PIT_P"] == pytest.approx(res["pitch"]["rat_p"], rel=1e-6)


# ------------------------------------------------------------------------ determinism

def test_everything_is_deterministic():
    a = fast_log(PLANT_5IN, GAINS_5IN, seconds=8.0, seed=7).bytes()
    b = fast_log(PLANT_5IN, GAINS_5IN, seconds=8.0, seed=7).bytes()
    assert a == b
    c = fast_log(PLANT_5IN, GAINS_5IN, seconds=8.0, seed=8).bytes()
    assert c != a                                                    # the seed matters
    x, _ = autotune_log(PLANT_5IN, GAINS_5IN, axes=("roll",))
    y, _ = autotune_log(PLANT_5IN, GAINS_5IN, axes=("roll",))
    assert x.bytes() == y.bytes()


if __name__ == "__main__":
    import _shim
    sys.exit(_shim.run(sys.modules[__name__]))
