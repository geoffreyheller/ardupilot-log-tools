"""WP3 of docs/pid-tuning-plan.md: the closed-loop step response, the oscillation ceiling
and the step rules (dflog/tune_step.py), tested against the WP2 simulator's truth.

The calm reference is **AutoTune's own result** on PLANT_5IN (`tunesim.autotune`), not
`GAINS_5IN`: the initial gain set's D (0.0036 at FLTD 37.5) makes that plant ring at
25 Hz (|T| = 7, measured by sinusoidal drive), which `oscillation()` correctly reports
as a D ceiling. The ringing case raises the AutoTuned P until the loop limit-cycles.

    python tests/test_tune_step.py
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
from dflog.checks import T                                               # noqa: E402
from dflog import tune, tune_step                                        # noqa: E402
from dflog.tune import Refusal, StepResponse, Ceiling                    # noqa: E402
from dflog.tunesim import autotune                                       # noqa: E402
from tunesynth import fast_log, PLANT_5IN, GAINS_5IN                     # noqa: E402

TMP = tempfile.mkdtemp(prefix="dflog-tunestep-")
SEED = 11
RING_MULT = 3.0            # AutoTuned roll P x this limit-cycles on PLANT_5IN (measured: 15.5 Hz, 56 % saturated)
RESONANT_MULT = 2.0        # x this is still stable but resonant: a 15.5 Hz line 20 dB above the floor, SRate < 5
_cache = {}


def _calm_gains():
    """The gains AutoTune finds on PLANT_5IN roll: the well-damped reference."""
    if "gains:calm" not in _cache:
        r = autotune(PLANT_5IN["roll"], GAINS_5IN["roll"], axis="roll")
        assert r["complete"], r["aborted"]
        _cache["gains:calm"] = dict(GAINS_5IN["roll"], rat_p=r["rat_p"], rat_i=r["rat_i"], rat_d=r["rat_d"],
                                    ang_p=r["ang_p"], acc_max_dps2=r["acc_max_dps2"])
    return _cache["gains:calm"]


def _signals(name, roll_gains, seconds=90.0, **kw):
    """(signals, refusals) of the roll axis of a synthetic log, built once per name."""
    if name not in _cache:
        w = fast_log(PLANT_5IN, dict(GAINS_5IN, roll=roll_gains), seconds=seconds, seed=SEED, axes=("roll",), **kw)
        log = Log(w.write(os.path.join(TMP, name + ".bin")), use_cache=False)
        assert log.diagnostics.ok, log.diagnostics.render()
        _cache[name] = tune.extract_axes(log, airborne_window(log), axes=("roll",))
    return _cache[name]


def _calm():
    sigs, refs = _signals("calm", _calm_gains())
    assert [r.code for r in refs] == [], [r.line() for r in refs]
    return sigs[0]


def _predicted_f180(plant, g, dt=1.0 / 400.0):
    """The -180 deg crossing of L = C G e^{-s dt} from Plant.freq_response and the
    configured PID (rad/s units, D through its FLTD first-order filter, one loop of
    measurement delay): where a P-driven loop limit-cycles."""
    f = np.linspace(1.0, 100.0, 4000)
    s = 2j * np.pi * f
    G = plant.freq_response(f) * math.pi / 180.0
    C = g["rat_p"] + g["rat_i"] / s + g["rat_d"] * s / (1.0 + s / (2.0 * math.pi * g["fltd"]))
    L = C * G * np.exp(-s * dt)
    ph = np.degrees(np.unwrap(np.angle(L)))
    idx = np.flatnonzero(ph <= -180.0)
    return float(f[idx[0]])


# ------------------------------------------------------------------ step response

def test_calm_step_response_matches_the_oracle_within_the_measured_tolerance():
    """The deconvolved mean vs `Plant.true_closed_loop_step` for the same gains.

    Measured tolerance, not the plan's 5 % RMS: the PID-Analyzer estimator's 25 Hz Wiener
    cut (a constant chosen for kHz Betaflight loops) sits within a factor of two of this
    loop's 12-16 Hz mode, so the response is smoothed and its edge spread 2-3 samples
    early (the Gaussian-edged regulariser is non-causal), and the Hann-windowed per-frame
    ratio biases the peak up. On this log (seed 11, 90 s, 1224 frames) that is RMS 0.137
    over 0-0.3 s, peak 1.30 vs 1.09, steady state 1.031 vs 0.979; an ideal first-order
    20 Hz system driven by the same Tar deconvolves to RMS 0.14 as well, so the error is
    the method's, not the loop's. The pooled Welch ratio (PIDReview's H) halves it to
    0.078 and so do 2 s frames - recorded in the WP3 report as the next step."""
    sig = _calm()
    st = tune_step.step_response(sig)
    assert isinstance(st, StepResponse), st
    assert st.n_frames >= T["tune_min_frames"]["warn"] and st.n_dropped_low > 0
    assert st.frames.shape == (st.n_frames, len(st.t)) and len(st.t) == int(round(0.5 * sig.fs))
    assert st.t[1] - st.t[0] == pytest.approx(1.0 / sig.fs)
    assert np.allclose(st.mean, st.frames.mean(axis=0))
    tt, true = PLANT_5IN["roll"].true_closed_loop_step(_calm_gains(), seconds=0.5)
    # Tar is post-FLTT; at FLTT 37.5 Hz the oracle with and without FLTT is the same curve
    _t2, true_fltt0 = PLANT_5IN["roll"].true_closed_loop_step(dict(_calm_gains(), fltt=0.0), seconds=0.5)
    assert np.abs(true - true_fltt0).max() < 1e-9
    m = tt <= 0.3
    rms = math.sqrt(np.mean((st.mean[m] - true[m]) ** 2))
    assert rms < 0.15, rms
    assert abs(st.mean.max() - true.max()) < 0.25, (st.mean.max(), true.max())
    assert st.mean.max() > true.max()                                   # the bias is upward
    assert abs(st.metrics["ss"] - true[160:].mean()) < 0.06
    assert abs(st.metrics["latency_s"] - tune_step.step_metrics(tt, true, 0.075)["latency_s"]) < 0.012
    assert 0.5 < st.consistency <= 1.0
    assert st.metrics["aggr"] == pytest.approx(0.075) and st.metrics["fs"] == pytest.approx(400.0)
    assert st.metrics["n_frames_total"] == st.n_frames + st.n_dropped_low + st.metrics["n_dropped_nonfinite"]
    # the per-frame stack: the interquartile band of the peaks is a unit-step-like curve
    # (measured p25 1.10, p50 1.22, p75 1.39; a few weakly excited frames reach 5 - the
    # per-frame ratio is unregularised below 25 Hz, which is what `consistency` scores)
    pk = st.frames.max(axis=1)
    assert 0.9 < np.percentile(pk, 25) < np.percentile(pk, 75) < 1.6 and np.percentile(pk, 5) > 0.5


def test_n_dropped_low_counts_the_frames_without_excitation():
    sig = _calm()
    st = tune_step.step_response(sig)
    flen = int(round(tune.CONSTANTS["step_frame_s"]["value"] * sig.fs))
    shift = flen // int(tune.CONSTANTS["step_overlap"]["value"])
    n = len(sig.tar)
    wins = min(n // shift - 16, (n - flen) // shift + 1)
    mx = np.array([np.abs(sig.tar[i * shift:i * shift + flen]).max() for i in range(wins)])
    assert st.n_dropped_low == int((mx < 20.0).sum())
    assert st.n_frames == int((mx >= 20.0).sum())
    assert st.metrics["n_frames_total"] == wins


def test_hover_only_log_is_no_excitation():
    sigs, refs = _signals("hover", _calm_gains(), seconds=40.0, stick="hover")
    assert refs == [] and len(sigs) == 1
    r = tune_step.step_response(sigs[0])
    assert isinstance(r, Refusal) and r.code == "NO_EXCITATION"
    assert r.axis == "roll" and r.log_name == "hover.bin"
    assert r.message.startswith("0 frame(s) above 20 deg/s in a 40 s window")
    assert f"{T['tune_min_frames']['fail']} needed ({T['tune_min_frames']['warn']} recommended)" in r.message
    assert "below 20 deg/s" in r.message and "tuning profile" in r.fix
    assert tune_step.describe(r)[0].startswith("step response: NO_EXCITATION")


def test_10hz_log_never_reaches_step_response():
    """The standard LOG_BITMASK logs PIDx at 10 Hz: WP1 refuses, so there is no signal
    to deconvolve - the tier C entry point is never called."""
    sigs, refs = _signals("slow10", _calm_gains(), seconds=30.0, pid_hz=10)
    assert sigs == []
    assert [r.code for r in refs] == ["PID_RATE_TOO_LOW"]
    assert "LOG_BITMASK" in refs[0].fix


def test_step_response_is_deterministic():
    sig = _calm()
    a, b = tune_step.step_response(sig), tune_step.step_response(sig)
    assert np.array_equal(a.mean, b.mean) and np.array_equal(a.frames, b.frames)
    assert set(a.metrics) == set(b.metrics) and a.consistency == b.consistency
    for k in a.metrics:                                   # NaN-aware: settle_s is NaN here
        x, y = a.metrics[k], b.metrics[k]
        assert x == y or (isinstance(x, float) and math.isnan(x) and math.isnan(y)), k
    # a fresh log with the same seed gives the same bytes and the same curve
    w = fast_log(PLANT_5IN, dict(GAINS_5IN, roll=_calm_gains()), seconds=90.0, seed=SEED, axes=("roll",))
    log = Log(w.write(os.path.join(TMP, "calm_again.bin")), use_cache=False)
    sigs, _r = tune.extract_axes(log, airborne_window(log), axes=("roll",))
    assert np.array_equal(tune_step.step_response(sigs[0]).mean, a.mean)


# ------------------------------------------------------------------ step metrics

def test_step_metrics_first_order_analytic():
    tau = 0.02
    t = np.arange(200) / 400.0
    r = 1.0 - np.exp(-t / tau)
    m = tune_step.step_metrics(t, r, 0.075)
    assert m["latency_s"] == pytest.approx(tau * math.log(2.0), abs=1.5e-3)
    assert m["rise_s"] == pytest.approx(tau * math.log(9.0), abs=2.5e-3)
    assert m["settle_s"] == pytest.approx(tau * math.log(50.0), abs=3e-3)
    assert m["peak"] == pytest.approx(r[-1]) and m["peak_t"] == pytest.approx(t[-1])
    assert m["overshoot"] < 0 and m["bounce"] == 0.0 and m["postmin"] == m["peak"]
    assert m["ss"] == pytest.approx(1.0, abs=1e-6)
    assert m["overshoot_ratio"] == pytest.approx(m["overshoot"] / (0.5 * 0.075)) and m["bounce_ratio"] == 0.0
    # a damped ring: overshoot and bounce-back are positive and scaled by AGGR
    r2 = 1.0 - np.exp(-t / 0.05) * np.cos(2 * math.pi * 12.0 * t)
    m2 = tune_step.step_metrics(t, r2, 0.1)
    assert m2["overshoot"] > 0.3 and 0.0 < m2["bounce"] < 1.0 and m2["peak_t"] < 0.1
    assert m2["overshoot_ratio"] == pytest.approx(m2["overshoot"] / 0.05)
    assert m2["bounce_ratio"] == pytest.approx(m2["bounce"] / 0.1)
    assert m2["bounce"] == pytest.approx((m2["peak"] - m2["postmin"]) / m2["peak"])
    # NaN-safe
    m3 = tune_step.step_metrics(t, np.full(200, np.nan), 0.075)
    assert all(math.isnan(m3[k]) for k in ("latency_s", "rise_s", "peak", "ss", "overshoot_ratio"))
    m4 = tune_step.step_metrics(t, r, float("nan"))
    assert math.isnan(m4["overshoot_ratio"]) and m4["rise_s"] == pytest.approx(m["rise_s"])
    assert tune_step.step_metrics(t, r, 0.075) == m                         # deterministic


# ------------------------------------------------------------------- oscillation

def test_ringing_loop_is_at_the_p_ceiling():
    g = dict(_calm_gains())
    g["rat_p"] *= RING_MULT
    sigs, refs = _signals("ring", g, seconds=60.0)
    # a limit cycle saturates the output: WP1's advisory refusal, the signals still come
    assert [r.code for r in refs] == ["OUTPUT_SATURATED"], [r.line() for r in refs]
    sig = sigs[0]
    assert sig.limited_pct > T["tune_limited_pct"]["fail"]
    osc = tune_step.oscillation(sig)
    assert osc["at_ceiling"] is True
    assert osc["peak_source"] == "error" and osc["prominence_db"] >= 30.0
    assert osc["f_osc"] == pytest.approx(15.5, abs=1.0)                 # pinned (measured)
    assert abs(osc["f_osc"] - _predicted_f180(PLANT_5IN["roll"], g)) < 3.0   # predicted 16.9 Hz
    assert osc["period_s"] == pytest.approx(1.0 / osc["f_osc"])
    assert osc["d_peak_agrees"] is True
    # SRate over the quiet samples: the ring is in Tar too (angle loop), and must not
    # read as stick input - the mask low-passes Tar below the search band
    assert osc["srate_quiet_s"] >= 1.0, osc["srate_quiet_s"]
    assert osc["srate_p95"] > T["tune_srate_osc"]["fail"] > T["tune_srate_osc"]["warn"]
    assert osc["attribution"] == "P" and osc["var_p_band"] > osc["var_d_band"]
    assert osc["dmod_min"] == 1.0                                       # SMAX 0: Dmod never engages
    c = osc["ceilings"]
    assert len(c) == 1 and isinstance(c[0], Ceiling)
    assert c[0].param == "ATC_RAT_RLL_P" and c[0].value == pytest.approx(g["rat_p"], rel=1e-6)
    assert c[0].method == "limit-cycle"
    zn = c[0].evidence["ziegler_nichols"]
    assert zn["ku"] == pytest.approx(g["rat_p"], rel=1e-6) and zn["tu_s"] == pytest.approx(osc["period_s"])
    assert zn["no_overshoot"]["kp"] == pytest.approx(0.2 * g["rat_p"], rel=1e-6)
    assert zn["no_overshoot"]["ti_s"] == pytest.approx(0.5 * osc["period_s"])
    assert zn["classic_pid"]["kp"] == pytest.approx(0.6 * g["rat_p"], rel=1e-6)
    assert "evidence only" in zn["note"]
    assert c[0].evidence["quik_recommendation"] == pytest.approx(0.4 * g["rat_p"], rel=1e-6)
    rules = tune_step.step_rules({}, osc, sig.gains)
    assert len(rules) == 1
    r = rules[0]
    assert r["param"] == "ATC_RAT_RLL_P" and r["current"] == pytest.approx(g["rat_p"], rel=1e-6)
    assert r["value"] == pytest.approx(0.4 * g["rat_p"], rel=1e-6) and r["change_pct"] == pytest.approx(-60.0)
    assert "QUIK_GAIN_MARGIN" in r["why"] and "15.5 Hz" in r["why"] and "SRate p95" in r["why"]
    assert "source" in r["why"] and "attributed to P" in r["why"]
    # the same log's step response shows the ring: overshoot far beyond the allowance
    st = tune_step.step_response(sig)
    assert isinstance(st, StepResponse) and st.metrics["overshoot_ratio"] > T["tune_overshoot_ratio"]["fail"]
    # the ceiling overrides the step rules even with those metrics
    assert tune_step.step_rules(st.metrics, osc, sig.gains) == rules
    lines = tune_step.describe(st, st.metrics, osc)
    assert any(l.startswith("oscillation: at the oscillation ceiling") for l in lines)
    # in hover the same loop limit-cycles on its own: QuickTune's own measuring condition
    sigs_h, refs_h = _signals("ring_hover", g, seconds=30.0, stick="hover")
    osc_h = tune_step.oscillation(sigs_h[0])
    # ~26 s quiet: the low-pass's edge at take-off marks 40 ms busy, then the 4 s delay
    assert osc_h["srate_quiet_s"] > 20.0 and osc_h["srate_p95"] > T["tune_srate_osc"]["fail"]
    assert osc_h["f_osc"] == pytest.approx(osc["f_osc"], abs=1.0) and osc_h["attribution"] == "P"


def test_resonant_but_stable_loop_trips_the_psd_detector_only():
    """P x2: still stable (no saturation, SRate 0.7 in the quiet part) but |S| peaks 20 dB
    at 15.5 Hz - the PSD criterion fires before QuickTune's SRate would, and the ceiling
    is reported as `limit-cycle` with the SRate figure alongside so a reader sees which
    detector spoke."""
    g = dict(_calm_gains())
    g["rat_p"] *= RESONANT_MULT
    sigs, refs = _signals("resonant", g, seconds=60.0)
    assert refs == [] and sigs[0].limited_pct == 0.0
    osc = tune_step.oscillation(sigs[0])
    assert osc["at_ceiling"] is True and osc["peak_source"] == "error"
    assert osc["f_osc"] == pytest.approx(15.5, abs=1.0) and osc["prominence_db"] >= 15.0
    assert osc["srate_quiet_s"] >= 1.0 and osc["srate_p95"] < T["tune_srate_osc"]["warn"]
    assert osc["srate_p95_all"] > T["tune_srate_osc"]["warn"]          # the stick alone inflates it
    assert osc["attribution"] == "P" and osc["ceilings"][0].method == "limit-cycle"
    assert "SRate p95" in osc["note"] and "limit cycle at 15.5 Hz" in osc["note"]
    r = tune_step.step_rules({}, osc, sigs[0].gains)
    assert [x["param"] for x in r] == ["ATC_RAT_RLL_P"] and r[0]["value"] == pytest.approx(0.4 * g["rat_p"], rel=1e-6)


def test_calm_log_has_no_ceiling_and_says_so_as_a_note():
    sig = _calm()
    osc = tune_step.oscillation(sig)
    assert osc["at_ceiling"] is False and osc["ceilings"] == [] and osc["attribution"] is None
    assert osc["note"].startswith("no ceiling found")
    assert "not a pass" in osc["note"]
    assert osc["f_osc"] is None and osc["peaks"]["error"] is None
    assert osc["dmod_min"] == 1.0 and osc["dmod_engaged_pct"] == 0.0
    # SRate is judged on quiet samples: the whole-segment p95 is inflated by the stick
    assert osc["srate_p95"] < T["tune_srate_osc"]["warn"] < osc["srate_p95_all"]
    assert osc["srate_quiet_s"] >= 1.0 and osc["srate_quiet_delay_s"] == 4.0
    lines = tune_step.describe(tune_step.step_response(sig), None, osc)
    assert any(l.startswith("oscillation: no ceiling found") for l in lines)
    assert any("SMAX never limited the loop" in l for l in lines)
    # a RATE-source segment (no P/D/SRate arrays) still runs: no attribution, SRate NaN
    class _R:
        pass
    r = _R()
    for k in ("axis", "log_name", "fs", "t", "tar", "act", "gains", "source"):
        setattr(r, k, getattr(sig, k))
    r.p = r.d = r.dmod = r.srate = None
    o2 = tune_step.oscillation(r)
    assert math.isnan(o2["srate_p95"]) and o2["attribution"] is None and o2["at_ceiling"] is False
    assert "SRate not logged" in o2["note"]


# -------------------------------------------------------------------- step rules

def _gains(**kw):
    g = dict(axis="roll", rat_p=0.10, rat_i=0.10, rat_d=0.004, gyro_filter=75.0, aggr=0.075)
    g.update(kw)
    return g


def _metrics(**kw):
    m = dict(latency_s=0.02, rise_s=0.02, peak=1.02, peak_t=0.05, overshoot=0.02, postmin=1.0, bounce=0.01,
             settle_s=0.1, ss=1.0, overshoot_ratio=0.5, bounce_ratio=0.2, aggr=0.075)
    m.update(kw)
    return m


CALM_OSC = dict(at_ceiling=False, attribution=None, note="no ceiling found")


def test_step_rules_are_bounded_directional_and_sourced():
    assert tune_step.step_rules(_metrics(), CALM_OSC, _gains()) == []
    # overshoot with little bounce-back: D up 10 % (RATE_D_UP), not P down
    r = tune_step.step_rules(_metrics(overshoot=0.06, overshoot_ratio=1.5, bounce_ratio=0.5), CALM_OSC, _gains())
    assert [x["param"] for x in r] == ["ATC_RAT_RLL_D"]
    assert r[0]["value"] == pytest.approx(0.0044) and r[0]["change_pct"] == pytest.approx(10.0)
    assert "RATE_D_UP" in r[0]["why"] and "0.5 x AGGR" in r[0]["why"] and "1.50" in r[0]["why"]
    # overshoot with bounce-back already high: P down 5 % and D down 5 %
    r = tune_step.step_rules(_metrics(overshoot=0.06, overshoot_ratio=1.5, bounce=0.1, bounce_ratio=1.3), CALM_OSC, _gains())
    d = {x["param"]: x for x in r}
    assert set(d) == {"ATC_RAT_RLL_P", "ATC_RAT_RLL_D"}
    assert d["ATC_RAT_RLL_P"]["value"] == pytest.approx(0.095) and "RP_STEP" in d["ATC_RAT_RLL_P"]["why"]
    assert d["ATC_RAT_RLL_D"]["value"] == pytest.approx(0.0038) and "RATE_D_DOWN" in d["ATC_RAT_RLL_D"]["why"]
    # one RP_STEP per unit of excess, capped at -25 % (AUTOTUNE_GMBK)
    r = tune_step.step_rules(_metrics(overshoot=0.3, overshoot_ratio=3.2, bounce_ratio=1.3), CALM_OSC, _gains())
    p = {x["param"]: x for x in r}["ATC_RAT_RLL_P"]
    assert p["value"] == pytest.approx(0.10 * 0.95 ** 3) and p["change_pct"] == pytest.approx((0.95 ** 3 - 1) * 100)
    r = tune_step.step_rules(_metrics(overshoot=0.9, overshoot_ratio=9.0, bounce_ratio=1.3), CALM_OSC, _gains())
    p = {x["param"]: x for x in r}["ATC_RAT_RLL_P"]
    assert p["value"] == pytest.approx(0.075) and p["change_pct"] == pytest.approx(-25.0)
    assert "capped" in p["why"] and "AUTOTUNE_GMBK" in p["why"]
    # D would exceed RD_MAX: P takes the step instead, and says why
    r = tune_step.step_rules(_metrics(overshoot=0.06, overshoot_ratio=1.5, bounce_ratio=0.5), CALM_OSC, _gains(rat_d=0.19))
    assert [x["param"] for x in r] == ["ATC_RAT_RLL_P"] and "RD_MAX" in r[0]["why"]
    # slow rise with no overshoot: P up 5 %; expected rise = 1.6 / INS_GYRO_FILTER
    r = tune_step.step_rules(_metrics(rise_s=0.06, overshoot_ratio=0.2), CALM_OSC, _gains())
    assert [x["param"] for x in r] == ["ATC_RAT_RLL_P"] and r[0]["value"] == pytest.approx(0.105)
    assert "60 ms" in r[0]["why"] and "21 ms" in r[0]["why"] and "INS_GYRO_FILTER 75" in r[0]["why"]
    assert tune_step.step_rules(_metrics(rise_s=0.03, overshoot_ratio=0.2), CALM_OSC, _gains()) == []
    assert tune_step.step_rules(_metrics(rise_s=0.06, overshoot_ratio=0.8), CALM_OSC, _gains()) == []
    # a response that never reaches 80 %: AutoTune's D_UP_DOWN_MARGIN rule, rise unmeasurable
    r = tune_step.step_rules(_metrics(rise_s=float("nan"), peak=0.7, overshoot=-0.3, overshoot_ratio=-8.0), CALM_OSC, _gains())
    assert [x["param"] for x in r] == ["ATC_RAT_RLL_P"] and "D_UP_DOWN_MARGIN" in r[0]["why"]
    # steady-state error: I toward P by the AutoTune ratio, capped at +25 %
    r = tune_step.step_rules(_metrics(ss=0.9), CALM_OSC, _gains(rat_i=0.05))
    assert [x["param"] for x in r] == ["ATC_RAT_RLL_I"]
    assert r[0]["value"] == pytest.approx(0.0625) and r[0]["change_pct"] == pytest.approx(25.0)
    assert "capped" in r[0]["why"] and "PI_RATIO_FINAL" in r[0]["why"].upper()
    assert tune_step.step_rules(_metrics(ss=0.9), CALM_OSC, _gains(rat_i=0.10)) == []      # already I = P
    r = tune_step.step_rules(_metrics(ss=0.9), CALM_OSC, _gains(axis="yaw", rat_p=0.2, rat_i=0.021, rat_d=0.0))
    assert [x["param"] for x in r] == ["ATC_RAT_YAW_I"] and r[0]["value"] == pytest.approx(0.02)
    # yaw with D = 0: a bounce rule has nothing to lower, an overshoot rule falls on P
    r = tune_step.step_rules(_metrics(overshoot_ratio=1.5, bounce_ratio=1.5, overshoot=0.06, bounce=0.1),
                             CALM_OSC, _gains(axis="yaw", rat_p=0.2, rat_i=0.02, rat_d=0.0))
    assert [x["param"] for x in r] == ["ATC_RAT_YAW_P"] and "bounce ratio already" in r[0]["why"]
    r = tune_step.step_rules(_metrics(overshoot_ratio=1.5, bounce_ratio=0.5, overshoot=0.06),
                             CALM_OSC, _gains(axis="yaw", rat_p=0.2, rat_i=0.02, rat_d=0.0))
    assert [x["param"] for x in r] == ["ATC_RAT_YAW_P"] and "cannot be raised" in r[0]["why"]
    # every why carries a number, a threshold and a source
    for m in (_metrics(overshoot=0.06, overshoot_ratio=1.5, bounce_ratio=0.5), _metrics(rise_s=0.06, overshoot_ratio=0.2),
              _metrics(ss=0.9)):
        for x in tune_step.step_rules(m, CALM_OSC, _gains(rat_i=0.05)):
            assert "source" in x["why"] and any(ch.isdigit() for ch in x["why"]), x["why"]
            assert set(x) == {"param", "current", "value", "change_pct", "why"}
    # a ceiling overrides everything and is exempt from the cap; attribution None -> P assumed
    osc = dict(at_ceiling=True, attribution="D", note="at the oscillation ceiling: SRate p95 9.10 > QUIK_OSC_SMAX 5",
               srate_warn=5.0)
    r = tune_step.step_rules(_metrics(overshoot_ratio=5.0, ss=0.8), osc, _gains(rat_i=0.05))
    assert [x["param"] for x in r] == ["ATC_RAT_RLL_D"] and r[0]["value"] == pytest.approx(0.0016)
    assert r[0]["change_pct"] == pytest.approx(-60.0) and "cap does not apply" in r[0]["why"]
    r = tune_step.step_rules(_metrics(), dict(osc, attribution=None), _gains())
    assert [x["param"] for x in r] == ["ATC_RAT_RLL_P"]
    # deterministic
    a = tune_step.step_rules(_metrics(overshoot=0.06, overshoot_ratio=1.5, bounce_ratio=0.5), CALM_OSC, _gains())
    b = tune_step.step_rules(_metrics(overshoot=0.06, overshoot_ratio=1.5, bounce_ratio=0.5), CALM_OSC, _gains())
    assert a == b


def test_step_rules_on_the_autotuned_reference_are_recorded():
    """Tier C scores the final closed loop's unit step against the twitch criteria
    AutoTune applied to its test gains (I ~ 0, FLTT 0, before the 25 % backoff), so even
    the loop AutoTune itself produced reads overshoot_ratio ~2.4 / bounce_ratio ~1.9 on
    the oracle and higher on the deconvolved curve. Pinned so WP6 knows the calibration
    it is fusing (the step-rules prior is 0.5 for this reason)."""
    tt, true = PLANT_5IN["roll"].true_closed_loop_step(_calm_gains(), seconds=0.5)
    m = tune_step.step_metrics(tt, true, 0.075)
    assert m["overshoot_ratio"] == pytest.approx(2.4, abs=0.2) and m["bounce_ratio"] == pytest.approx(1.86, abs=0.15)
    sig = _calm()
    st = tune_step.step_response(sig)
    assert 6.0 < st.metrics["overshoot_ratio"] < 10.0 and 2.5 < st.metrics["bounce_ratio"] < 4.5
    rules = tune_step.step_rules(st.metrics, tune_step.oscillation(sig), sig.gains)
    d = {r["param"]: r for r in rules}
    assert set(d) == {"ATC_RAT_RLL_P", "ATC_RAT_RLL_D"}
    assert d["ATC_RAT_RLL_P"]["change_pct"] == pytest.approx(-25.0)          # capped
    assert d["ATC_RAT_RLL_D"]["change_pct"] == pytest.approx(-5.0)
    assert all(-25.0 <= r["change_pct"] <= 25.0 for r in rules)


# ---------------------------------------------------------------------- describe

def test_describe_is_short_and_deterministic():
    sig = _calm()
    st = tune_step.step_response(sig)
    osc = tune_step.oscillation(sig)
    a = tune_step.describe(st, st.metrics, osc)
    b = tune_step.describe(st, st.metrics, osc)
    assert a == b and 3 <= len(a) <= 5
    assert a[0].startswith(f"step response: {st.n_frames} frames ({st.n_dropped_low} dropped below 20 deg/s)")
    assert "consistency" in a[0] and "25 Hz regulariser" in a[0]
    assert a[1].startswith("latency ") and "rise 10-90 %" in a[1] and "vs 0.5 x AGGR 3.75 %" in a[1]
    assert "vs AGGR 7.5 %" in a[1] and "steady state" in a[1] and "settle +-2 %" in a[1]
    assert all(len(l) < 400 for l in a)
    # constants carry sources and are in the plan's shape
    for name, c in tune_step.CONSTANTS.items():
        assert set(c) >= {"value", "source", "note"} and c["source"], name
    assert not set(tune_step.CONSTANTS) & set(tune.CONSTANTS)


if __name__ == "__main__":
    import _shim
    sys.exit(_shim.run(sys.modules[__name__]))
