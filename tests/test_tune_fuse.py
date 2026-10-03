"""WP6 of docs/pid-tuning-plan.md: tier D parameter consistency, the confidence model,
fusion, `analyse`, the `tune` Section and `check_tune` (dflog/tune_fuse.py).

Ground truth is the simulator again: a fast log written with the gains
`tunesim.autotune` found on PLANT_5IN must come back from tier B within +-25 % of them
at confidence >= 0.7, and an AutoTune session log of the same plant must make tier A win.
The fusion rules themselves are exercised on hand-built candidates.

    python tests/test_tune_fuse.py
"""

import dataclasses
import io
import json
import math
import os
import sys
import tempfile
from contextlib import redirect_stdout, redirect_stderr

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import pytest                       # noqa: F401
except ImportError:
    import _shim as pytest

from dflog import Log, airborne_window                                            # noqa: E402
from dflog.checks import T, PASS, WARN, FAIL, SKIP                                # noqa: E402
from dflog import tune, tune_fuse, tunesim                                        # noqa: E402
from dflog.tune import GainSet, Ceiling, Recommendation, TuneAnalysis            # noqa: E402
from dflog.analysis import _jsonable, check_tune, ALL_CHECKS                      # noqa: E402
from tunesynth import fast_log, autotune_log, PLANT_5IN, GAINS_5IN                # noqa: E402

TMP = tempfile.mkdtemp(prefix="dflog-tunefuse-")
SEED = 11
_CACHE = {}


# ------------------------------------------------------------------ fixtures

def _calm_gains():
    """The gains AutoTune finds on PLANT_5IN roll (the truth tier B must recover)."""
    if "truth" not in _CACHE:
        r = tunesim.autotune(PLANT_5IN["roll"], GAINS_5IN["roll"], axis="roll")
        assert r["complete"], r["aborted"]
        _CACHE["truth"] = r
    return _CACHE["truth"]


def _calm_set():
    r = _calm_gains()
    return dict(GAINS_5IN["roll"], rat_p=r["rat_p"], rat_i=r["rat_i"], rat_d=r["rat_d"], ang_p=r["ang_p"],
                acc_max_dps2=r["acc_max_dps2"])


def _log(name, **kw):
    if name not in _CACHE:
        kw.setdefault("seconds", 120.0)
        kw.setdefault("seed", SEED)
        kw.setdefault("axes", ("roll",))
        w = fast_log(PLANT_5IN, dict(GAINS_5IN, roll=_calm_set()), **kw)
        path = w.write(os.path.join(TMP, name + ".bin"))
        log = Log(path, use_cache=False)
        assert log.diagnostics.ok, log.diagnostics.render()
        _CACHE[name] = (path, log)
    return _CACHE[name]


def _atun_log():
    if "atun" not in _CACHE:
        w, res = autotune_log(PLANT_5IN, GAINS_5IN, axes=("roll",))
        path = w.write(os.path.join(TMP, "atun.bin"))
        _CACHE["atun"] = (path, Log(path, use_cache=False), res)
    return _CACHE["atun"]


def _analysis(name="e2e"):
    key = "analysis:" + name
    if key not in _CACHE:
        _path, log = _log(name)
        _CACHE[key] = tune_fuse.analyse([log], [airborne_window(log)])
    return _CACHE[key]


def _gs(axis="roll", **kw):
    g = dict(GAINS_5IN[axis])
    g.update(kw)
    return GainSet(axis=axis, **g)


def _cand(field, value, method, adequacy=1.0, excitation=1.0, consistency=1.0, note=""):
    return dict(field=field, value=value, method=method, adequacy=adequacy, excitation=excitation,
                consistency=consistency, evidence={}, note=note)


def _manual(candidates, gains=None, ceilings=(), axis="roll", tiers=None, plant=None):
    """A TuneAnalysis around hand-built candidates for one axis; `plant` turns on the
    margin gate on the applied rate set."""
    g = gains or _gs(axis)
    entry = dict(current=g, candidates=list(candidates), ceilings=list(ceilings),
                 tiers=tiers or dict(A=False, B=True, C=False), signals=[], step=[], autotune=[], notes=[], errors=[])
    if plant is not None:
        from dflog import tune_ident
        entry["plant"] = plant
        m = tune_ident.margins(plant, g)
        entry["margins"] = {k: v for k, v in m.items() if k not in ("L", "np_L", "freqs_hz", "np_freqs_hz")}
    recs = tune_fuse.fuse({axis: entry}, {axis: g})
    return TuneAnalysis(logs=[], identity_ok=True, identity_reasons=[], per_axis={axis: entry}, recommendations=recs,
                        refusals=[], params=[], constants={}), recs


def _by(recs):
    return {r.param: r for r in recs}


# ---------------------------------------------------------------- confidence

def test_confidence_components_multiply_and_are_all_reported():
    v, c = tune_fuse.confidence("virtual-autotune", 0.9, 0.8, 0.7, 0.95)
    assert v == pytest.approx(0.8 * 0.9 * 0.8 * 0.7 * 0.95)
    assert {"prior", "adequacy", "excitation", "consistency", "agreement", "product", "cap", "value"} <= set(c)
    assert c["prior"] == 0.8 and c["cap"] is None and c["value"] == v and c["product"] == v
    assert v == pytest.approx(c["prior"] * c["adequacy"] * c["excitation"] * c["consistency"] * c["agreement"])
    for m, p in (("autotune-log", 1.0), ("virtual-autotune", 0.8), ("step-rules", 0.5), ("ceiling", 0.7)):
        assert tune_fuse.PRIORS[m]["value"] == p and tune_fuse.PRIORS[m]["source"]
        assert tune_fuse.confidence(m, 1, 1, 1, 1)[1]["prior"] == p
    # the agreement floor
    assert tune_fuse.confidence("autotune-log", 1, 1, 1, 0.1)[1]["agreement"] == 0.5
    # tier C is capped below the withheld band (the calibration caveat)
    v, c = tune_fuse.confidence("step-rules", 1, 1, 1, 1)
    assert v == 0.39 and c["cap"] == 0.39 and c["product"] == 0.5 and v < T["tune_confidence"]["fail"]
    # adequacy: 0 below fail, 1 at warn, linear between, x the unsaturated fraction
    assert tune_fuse.adequacy(50.0) == 0.0 and tune_fuse.adequacy(400.0) == 1.0
    assert tune_fuse.adequacy(150.0) == pytest.approx(0.5) and tune_fuse.adequacy(400.0, 10.0) == pytest.approx(0.9)
    with _raises(ValueError):
        tune_fuse.confidence("magic", 1, 1, 1, 1)


# -------------------------------------------------------------------- fusion

BRISKET_ROLL = (6.557e4, 5.0, 0.0393, 0.01343)     # the roll fit of brisket-t1.bin


def _brisket():
    from dflog import tune_ident
    g = _gs(rat_p=0.135, rat_i=0.135, rat_d=0.0036, fltd=21.0, fltt=21.0, gyro_filter=42.0)
    return g, tune_ident.model_from_params(*BRISKET_ROLL, loop_hz=400.0)


def test_margin_ceiling_clips_to_itself_and_keeps_the_tier():
    g = _gs()
    c = Ceiling(param="ATC_RAT_RLL_P", value=0.10, method="margin-45deg-6dB", evidence=dict(pm_min_deg=45.0, gm_min_db=6.0),
                includes_margin=True)
    _a, recs = _manual([_cand("rat_p", 0.12, "virtual-autotune")], g, ceilings=[c])
    p = _by(recs)["ATC_RAT_RLL_P"]
    assert p.value == pytest.approx(0.10) and p.method == "virtual-autotune" and p.components["prior"] == 0.8
    assert "clipped to the ceiling" in p.note and "PM >= 45 deg" in p.note and "0.4 x" not in p.note
    assert p.evidence["margin_clipped"] is True and p.evidence["unclipped_value"] == 0.12


def test_the_tightest_ceiling_wins():
    g = _gs()
    osc = Ceiling(param="ATC_RAT_RLL_P", value=0.10, method="limit-cycle", evidence=dict(f_osc=15.5))
    loose = Ceiling(param="ATC_RAT_RLL_P", value=0.08, method="margin-45deg-6dB", evidence={}, includes_margin=True)
    tight = Ceiling(param="ATC_RAT_RLL_P", value=0.03, method="margin-45deg-6dB", evidence={}, includes_margin=True)
    _a, recs = _manual([_cand("rat_p", 0.12, "virtual-autotune")], g, ceilings=[loose, osc])
    assert _by(recs)["ATC_RAT_RLL_P"].value == pytest.approx(0.04) and _by(recs)["ATC_RAT_RLL_P"].method == "ceiling"
    _a, recs = _manual([_cand("rat_p", 0.12, "virtual-autotune")], g, ceilings=[osc, tight])
    assert _by(recs)["ATC_RAT_RLL_P"].value == pytest.approx(0.03)


def test_applied_rate_set_below_the_margins_is_withheld():
    # the Brisket roll virtual AutoTune (P 0.1955, D 0.004448) gives PM ~34 deg on its own plant
    g, plant = _brisket()
    cands = [_cand("rat_p", 0.1955, "virtual-autotune"), _cand("rat_d", 0.004448, "virtual-autotune")]
    _a, recs = _manual(cands, g, plant=plant)
    r = _by(recs)
    for name in ("ATC_RAT_RLL_P", "ATC_RAT_RLL_I", "ATC_RAT_RLL_D"):
        assert math.isnan(r[name].value), name
        assert r[name].note.startswith("withheld: the rate set as applied") and "below the limits" in r[name].note
        assert r[name].evidence["margin_gate"]["ok"] is False and r[name].evidence["margin_gate"]["pm_deg"] < 40
    assert r["ATC_RAT_RLL_P"].evidence["withheld_value"] == pytest.approx(0.1955)
    # with the margin ceilings the same candidates pass and say so
    from dflog import tune_ident
    cs, _n = tune_ident.margin_ceilings(plant, g, d_for_p=0.004448, p_for_d=0.1955)
    _a, recs = _manual(cands, g, ceilings=cs, plant=plant)
    r = _by(recs)
    assert r["ATC_RAT_RLL_P"].value == pytest.approx(0.1528, rel=0.01)
    assert r["ATC_RAT_RLL_I"].value == pytest.approx(r["ATC_RAT_RLL_P"].value)
    assert r["ATC_RAT_RLL_P"].evidence["margin_gate"]["ok"] is True and "gives PM 45" in r["ATC_RAT_RLL_P"].note


def test_angle_p_found_on_an_unclipped_rate_loop_is_withheld():
    g = _gs()
    c = Ceiling(param="ATC_RAT_RLL_P", value=0.10, method="margin-45deg-6dB", evidence={}, includes_margin=True)
    _a, recs = _manual([_cand("rat_p", 0.12, "virtual-autotune"), _cand("ang_p", 9.0, "virtual-autotune")], g, ceilings=[c])
    a = _by(recs)["ATC_ANG_RLL_P"]
    assert math.isnan(a.value) and "unclipped" not in a.note and "rate P is clipped to 0.1" in a.note
    assert a.evidence["withheld_value"] == 9.0
    # not clipped: angle P stands
    _a, recs = _manual([_cand("rat_p", 0.09, "virtual-autotune"), _cand("ang_p", 9.0, "virtual-autotune")], g, ceilings=[c])
    assert _by(recs)["ATC_ANG_RLL_P"].value == 9.0


def test_changes_inside_the_deadband_are_no_change():
    # 2026-09-30 Brisket: the same gains re-identified on two flights moved the pitch
    # recommendation 0.129 -> 0.1343 -> 0.132; a +2-4 % change is flight-to-flight noise
    g = _gs(rat_p=0.129, rat_i=0.135, rat_d=0.0035)
    dead = tune_fuse.FUSE_CONSTANTS["fuse_deadband_pct"]["value"]
    assert dead == 5.0 and tune_fuse.FUSE_CONSTANTS["fuse_deadband_pct"]["source"]
    _a, recs = _manual([_cand("rat_p", 0.1343, "virtual-autotune"), _cand("rat_d", 0.003817, "virtual-autotune")], g)
    r = _by(recs)
    p = r["ATC_RAT_RLL_P"]
    assert p.method == "unchanged" and p.value == pytest.approx(0.129) and p.change_pct == pytest.approx(0.0)
    assert p.note.startswith("no change: virtual-autotune gives 0.1343 (+4.1 %)") and "deadband" in p.note
    assert p.evidence["deadband_value"] == pytest.approx(0.1343) and p.confidence == pytest.approx(0.8)
    assert r["ATC_RAT_RLL_I"].method == "unchanged" and r["ATC_RAT_RLL_I"].value == pytest.approx(0.135)
    d = r["ATC_RAT_RLL_D"]                                            # +9.1 %: outside, stands
    assert d.method == "virtual-autotune" and d.value == pytest.approx(0.003817)
    # a change just outside the band is kept
    _a, recs = _manual([_cand("rat_p", 0.129 * 1.06, "virtual-autotune")], g)
    assert _by(recs)["ATC_RAT_RLL_P"].method == "virtual-autotune"
    # ...and one inside it is kept when the current loop misses the margins: the Brisket pitch
    # plant of brisket-t1 at P 0.135 read PM ~43-45 deg; -4.6 % to the margin ceiling took the
    # flown loop to 46.2 deg on brisket-t2 - a correction, not noise
    from dflog import tune_ident
    pitch = tune_ident.model_from_params(7.479e4, 5.0, 0.03933, 0.01404, loop_hz=400.0)
    g0 = _gs(rat_p=0.135, rat_i=0.135, rat_d=0.0036, fltd=21.0, fltt=21.0, gyro_filter=42.0)
    assert not tune_ident.meets_margins(tune_ident.margins(pitch, g0))
    cs, _n = tune_ident.margin_ceilings(pitch, g0, d_for_p=0.003543, p_for_d=0.1613)
    _a, recs = _manual([_cand("rat_p", 0.1613, "virtual-autotune"), _cand("rat_d", 0.003543, "virtual-autotune")],
                       g0, ceilings=cs, plant=pitch)
    p = _by(recs)["ATC_RAT_RLL_P"]
    assert p.method == "virtual-autotune" and -5.0 < p.change_pct < 0.0, (p.method, p.change_pct)
    assert "not deadbanded: the current loop misses the margins" in p.note
    assert _by(recs)["ATC_RAT_RLL_D"].method == "virtual-autotune"      # -1.6 %, kept for the same reason


def test_margins_read_outside_the_coherent_band_withhold_the_rate_set():
    # brisket-t2.bin yaw: coherent 0.5-3.5 Hz, tau2 and delay pinned at their lower
    # bounds, virtual AutoTune P 0.18 -> 0.48 at confidence 0.70 with a "36 dB" gain margin
    # read at ~20 Hz: nothing measured stood behind it
    from dflog import tune_ident
    g = _gs("yaw", rat_p=0.18, rat_i=0.018, rat_d=0.0, flte=2.0)
    pinned = dataclasses.replace(tune_ident.model_from_params(519.8, 0.1735, 0.001, 0.0, loop_hz=400.0), band=(0.5, 3.5))
    assert tune_ident.pinned_params(pinned) == ["tau2 1 ms", "delay 0 ms"]
    cands = [_cand("rat_p", 0.48, "virtual-autotune"), _cand("flte", 1.47, "virtual-autotune")]
    _a, recs = _manual(cands, g, axis="yaw", plant=pinned)
    r = _by(recs)
    for name in ("ATC_RAT_YAW_P", "ATC_RAT_YAW_I", "ATC_RAT_YAW_FLTE"):
        assert math.isnan(r[name].value), name
        assert "but not measured" in r[name].note and "tau2 1 ms and delay 0 ms sit at their bounds" in r[name].note, r[name].note
        assert "SysID" in r[name].note
    gate = r["ATC_RAT_YAW_P"].evidence["margin_gate"]
    assert gate["meets"] is True and gate["ok"] is False and gate["extrapolated"]
    # the same band with the lag identified: the -180 deg point may sit above the band, the
    # fit carries it, nothing is withheld for extrapolation
    lag = dataclasses.replace(tune_ident.model_from_params(519.8, 0.1735, 0.039, 0.014, loop_hz=400.0), band=(0.5, 3.5))
    assert tune_ident.pinned_params(lag) == []
    _a, recs = _manual(cands, g, axis="yaw", plant=lag)
    assert "not measured" not in _by(recs)["ATC_RAT_YAW_P"].note
    # a crossover above the band is extrapolated whatever the fit
    m = tune_ident.margins(lag, dataclasses.replace(g, rat_p=5.0, rat_i=0.5))
    assert m["fc_hz"] > 3.5 and any(x.startswith("crossover") for x in tune_ident.extrapolated(lag, m))


def test_ceiling_clips_to_0_4x_with_a_note():
    g = _gs()
    c = Ceiling(param="ATC_RAT_RLL_P", value=0.10, method="limit-cycle", evidence=dict(f_osc=15.5))
    _a, recs = _manual([_cand("rat_p", 0.12, "virtual-autotune")], g, ceilings=[c])
    p = _by(recs)["ATC_RAT_RLL_P"]
    assert p.value == pytest.approx(0.04) and p.method == "ceiling"
    assert "clipped to 0.4 x ceiling" in p.note and "limit-cycle ceiling 0.1" in p.note and "QUIK_GAIN_MARGIN" in p.note
    assert p.evidence["unclipped_value"] == 0.12 and p.evidence["unclipped_method"] == "virtual-autotune"
    assert p.components["prior"] == 0.7 and p.change_pct == pytest.approx((0.04 / g.rat_p - 1) * 100)
    # a value below the ceiling is not touched
    _a, recs = _manual([_cand("rat_p", 0.09, "virtual-autotune")], g, ceilings=[c])
    assert _by(recs)["ATC_RAT_RLL_P"].value == 0.09 and _by(recs)["ATC_RAT_RLL_P"].method == "virtual-autotune"


def test_tier_a_wins_over_b_over_c_and_agreement_is_the_runner_up():
    g = _gs()
    cands = [_cand("rat_p", 0.10, "step-rules"), _cand("rat_p", 0.12, "virtual-autotune"), _cand("rat_p", 0.11, "autotune-log")]
    _a, recs = _manual(cands, g)
    p = _by(recs)["ATC_RAT_RLL_P"]
    assert p.value == 0.11 and p.method == "autotune-log"
    assert p.components["agreement"] == pytest.approx(1 - 0.01 / 0.12)
    assert "virtual-autotune gives 0.12" in p.note
    _a, recs = _manual(cands[:2], g)
    assert _by(recs)["ATC_RAT_RLL_P"].method == "virtual-autotune"
    # the uncalibrated step rules never set another tier's agreement, and say so
    assert _by(recs)["ATC_RAT_RLL_P"].components["agreement"] == 1.0
    assert "not used for agreement" in _by(recs)["ATC_RAT_RLL_P"].note
    _a, recs = _manual(cands[:1], g)
    assert _by(recs)["ATC_RAT_RLL_P"].method == "step-rules"
    # a ceiling candidate outranks the step rules and counts for agreement
    _a, recs = _manual([_cand("rat_p", 0.05, "ceiling"), _cand("rat_p", 0.06, "virtual-autotune")], g)
    p = _by(recs)["ATC_RAT_RLL_P"]
    assert p.method == "virtual-autotune" and p.components["agreement"] == pytest.approx(1 - 0.01 / 0.06)


def test_withheld_is_warn_with_the_value_in_evidence_only():
    g = _gs()
    a, recs = _manual([_cand("rat_p", 0.09, "step-rules")], g, tiers=dict(A=False, B=False, C=True))
    p = _by(recs)["ATC_RAT_RLL_P"]
    assert p.confidence <= 0.39 < T["tune_confidence"]["fail"]
    assert math.isnan(p.value) and math.isnan(p.change_pct)
    assert p.evidence["withheld_value"] == 0.09 and p.note.startswith("withheld: confidence")
    sec = tune_fuse.to_section(a, whole_tool_refusal=False)
    r = {x.name: x for x in sec.results}["roll ATC_RAT_RLL_P"]
    assert r.status == WARN and "withheld" in r.summary and r.evidence["withheld_value"] == 0.09
    assert r.evidence["recommended"] is None
    tbl = {t["name"]: t for t in sec.tables}["recommendations"]
    row = [x for x in tbl["rows"] if x[1] == "ATC_RAT_RLL_P"][0]
    assert row[3] == "withheld" and row[4] == "-"
    # a low-excitation tier B value (0.8 x 0.3) is withheld the same way
    _a, recs = _manual([_cand("rat_p", 0.09, "virtual-autotune", excitation=0.3)], g)
    assert math.isnan(_by(recs)["ATC_RAT_RLL_P"].value)
    # 0.4-0.7 is indicative: WARN with the value shown
    a, recs = _manual([_cand("rat_p", 0.09, "virtual-autotune", excitation=0.7)], g)
    p = _by(recs)["ATC_RAT_RLL_P"]
    assert p.value == 0.09 and 0.4 <= p.confidence < 0.7
    sec = tune_fuse.to_section(a, whole_tool_refusal=False)
    r = {x.name: x for x in sec.results}["roll ATC_RAT_RLL_P"]
    assert r.status == WARN and "validate before applying" in r.summary


def test_i_follows_p_by_the_axis_ratio_unless_the_logs_ratio_is_non_default():
    # default ratio: I = P x 1.0, same method and confidence as P; a tier's own I is ignored
    g = _gs()
    _a, recs = _manual([_cand("rat_p", 0.10, "virtual-autotune"), _cand("rat_i", 0.05, "virtual-autotune")], g)
    d = _by(recs)
    assert d["ATC_RAT_RLL_I"].value == pytest.approx(0.10) and d["ATC_RAT_RLL_I"].method == "virtual-autotune"
    assert d["ATC_RAT_RLL_I"].confidence == d["ATC_RAT_RLL_P"].confidence
    assert "follows P by AutoTune's ratio 1" in d["ATC_RAT_RLL_I"].note and d["ATC_RAT_RLL_I"].evidence["ratio_preserved"] is False
    # non-default ratio: preserved and noted
    g2 = _gs(rat_p=0.10, rat_i=0.05)                 # I/P 0.5: deviation 0.5 > warn 0.25
    _a, recs = _manual([_cand("rat_p", 0.12, "virtual-autotune")], g2)
    i = _by(recs)["ATC_RAT_RLL_I"]
    assert i.value == pytest.approx(0.06) and i.evidence["ratio_preserved"] is True
    assert "non-default" in i.note and "preserved" in i.note and "0.500" in i.note
    # yaw: 0.1
    gy = _gs("yaw")
    _a, recs = _manual([_cand("rat_p", 0.20, "virtual-autotune")], gy, axis="yaw")
    d = _by(recs)
    assert d["ATC_RAT_YAW_I"].value == pytest.approx(0.02) and "0.1" in d["ATC_RAT_YAW_I"].note
    # yaw D only with AUTOTUNE_AXES bit 8; ANG_P not from tier B on yaw (plan section 9 item 3)
    entry = dict(current=gy, candidates=[_cand("rat_d", 0.002, "virtual-autotune"), _cand("ang_p", 6.0, "virtual-autotune"),
                                         _cand("flte", 3.0, "virtual-autotune")])
    recs = tune_fuse.fuse({"yaw": entry}, {"yaw": gy})
    assert set(_by(recs)) == {"ATC_RAT_YAW_D", "ATC_ANG_YAW_P", "ATC_RAT_YAW_FLTE"}   # candidates given by hand pass through
    # ... the gating lives in tier_candidates:
    e = dict(virtual=dict(rat_p=0.2, rat_i=0.02, rat_d=0.002, ang_p=6.0, acc_max_dps2=300.0, flte=3.0, complete=True, why="x", constants={}),
             plant=_true_model(), margins=dict(np_in_band=True), adequacy=dict(pooled=1.0))
    fields = {c["field"] for c in tune_fuse.tier_candidates("yaw", e, gy, autotune_axes=7)}
    assert fields == {"rat_p", "rat_i", "flte"}
    fields = {c["field"] for c in tune_fuse.tier_candidates("yaw", e, gy, autotune_axes=15)}
    assert fields == {"rat_p", "rat_i", "flte", "rat_d"}


def test_acc_max_is_recommended_only_at_high_confidence_and_in_the_logs_spelling():
    g = _gs(acc_max_dps2=1100.0)
    g.param_names["acc_max_dps2"] = "ATC_ACCEL_R_MAX"
    _a, recs = _manual([_cand("acc_max_dps2", 1500.0, "virtual-autotune")], g)
    r = _by(recs)["ATC_ACCEL_R_MAX"]
    assert r.current == 110000.0 and r.value == 150000.0 and r.evidence["unit"] == "cdeg/s^2"
    _a, recs = _manual([_cand("acc_max_dps2", 1500.0, "virtual-autotune", excitation=0.8)], g)
    r = _by(recs)["ATC_ACCEL_R_MAX"]
    assert r.method == "unchanged" and r.value == 110000.0 and r.change_pct == 0.0
    assert "left unchanged" in r.note and "1500" in r.note and r.evidence["candidate_value"] == 1500.0
    g.param_names["acc_max_dps2"] = "ATC_ACC_R_MAX"
    _a, recs = _manual([_cand("acc_max_dps2", 1500.0, "virtual-autotune")], g)
    assert _by(recs)["ATC_ACC_R_MAX"].value == 1500.0 and _by(recs)["ATC_ACC_R_MAX"].evidence["unit"] == "deg/s^2"
    entry = dict(current=g, candidates=[_cand("acc_max_dps2", 1500.0, "virtual-autotune")])
    recs = tune_fuse.fuse({"roll": entry}, {"roll": g}, prop_in=5)
    assert "Mission Planner calculator (5 in)" in _by(recs)["ATC_ACC_R_MAX"].note


def _true_model():
    from dflog import tune_ident
    p = PLANT_5IN["roll"]
    return tune_ident.model_from_params(p.k, p.tau1, p.tau2, p.delay, loop_hz=400.0)


# -------------------------------------------------------------------- tier D

def test_tier_d_grades_filters_and_ratios_with_sources():
    res = {r.name: r for r in tune_fuse.param_consistency(_gs(fltd=20.0))}
    assert res["roll FLTD"].status == WARN and res["roll FLTD"].evidence["value"] == pytest.approx(abs(20.0 / 37.5 - 1))
    assert res["roll FLTD"].source == T["tune_flt_ratio_dev"]["source"] and "INS_GYRO_FILTER/2 = 37.5" in res["roll FLTD"].summary
    assert res["roll FLTT"].status == PASS and res["roll I/P ratio"].status == PASS
    # yaw with I = P: deviation |1.0 - 0.1| / 0.1 = 9 is beyond the fail band of T, not merely WARN
    res = {r.name: r for r in tune_fuse.param_consistency(_gs("yaw", rat_i=0.18), aircraft_wide=False)}
    r = res["yaw I/P ratio"]
    assert r.status == FAIL and r.evidence["value"] == pytest.approx(9.0) and r.evidence["expected"] == 0.1
    assert "AUTOTUNE_YAW_PI_RATIO_FINAL" in r.summary and r.source == T["tune_pi_ratio_dev"]["source"]
    res = {r.name: r for r in tune_fuse.param_consistency(_gs("yaw", rat_i=0.024), aircraft_wide=False)}
    assert res["yaw I/P ratio"].status == WARN                     # deviation 0.33
    assert "ATC_RATE_FF_ENAB" not in res
    # yaw FLTD is outside the wiki rule (yaw D is 0): reported, never graded; yaw FLTT is graded
    res = {r.name: r for r in tune_fuse.param_consistency(_gs("yaw", fltd=0.0, fltt=20.0), aircraft_wide=False)}
    assert res["yaw FLTD"].status == PASS and res["yaw FLTD"].evidence["graded"] is False and "filters nothing" in res["yaw FLTD"].summary
    assert res["yaw FLTT"].status == WARN and res["yaw FLTT"].evidence["value"] == pytest.approx(abs(20.0 / 37.5 - 1))
    # D/P reported not graded; ranges; FF_ENAB; SMAX
    res = {r.name: r for r in tune_fuse.param_consistency(_gs(rat_d=0.06, ff_enab=False, smax=50.0))}
    assert res["roll D/P ratio"].status == PASS and res["roll D/P ratio"].evidence["graded"] is False
    assert "0.027" in res["roll D/P ratio"].summary
    assert res["roll AC_PID ranges"].status == WARN and res["roll AC_PID ranges"].evidence["outside"] == ["ATC_RAT_RLL_D"]
    assert "var_info" in res["roll AC_PID ranges"].source
    assert res["ATC_RATE_FF_ENAB"].status == WARN and "wiki" in res["ATC_RATE_FF_ENAB"].source
    assert res["roll SMAX"].status == PASS and "armed" in res["roll SMAX"].summary
    assert {r.name for r in tune_fuse.param_consistency(_gs())} == {"roll I/P ratio", "roll FLTD", "roll FLTT", "roll D/P ratio",
                                                                    "roll AC_PID ranges", "roll SMAX", "ATC_RATE_FF_ENAB"}
    for r in tune_fuse.param_consistency(_gs()):
        assert r.source


def test_prop_in_gives_the_mission_planner_comparison():
    mp = tune_fuse.mission_planner_initial(10)
    assert (mp["ins_gyro_filter"], mp["fltd"], mp["mot_thst_expo"]) == (42.0, 21.0, 0.60)
    # the Brisket aircraft's values (ATC_ACCEL_R_MAX 116700 in its log, per tune.py's note); the sources
    # file's "110 700" for 10 in is a transcription slip against its own cubic
    assert mp["accel_rp_cdss"] == 116700.0 and mp["accel_y_cdss"] == 27000.0
    mp5 = tune_fuse.mission_planner_initial(5)
    assert mp5["ins_gyro_filter"] == 75.0 and mp5["mot_thst_expo"] == 0.49
    res = {r.name: r for r in tune_fuse.param_consistency(_gs(), prop_in=5, params=dict(MOT_THST_EXPO=0.55))}
    r = res["Mission Planner calculator"]
    assert r.status == PASS and "INS_GYRO_FILTER 75 Hz (configured 75)" in r.summary and "configured 0.55" in r.summary
    h, rows = tune_fuse.calculator_rows(dict(roll=_gs(), yaw=_gs("yaw")), 5, dict(MOT_THST_EXPO=0.55))
    names = [row[0] for row in rows]
    assert names[0] == "INS_GYRO_FILTER" and "ATC_ACCEL_R_MAX" in names and "ATC_ACCEL_Y_MAX" in names and names[-1] == "MOT_THST_EXPO"
    byname = {row[0]: row for row in rows}
    assert byname["INS_GYRO_FILTER"][1:4] == ["75", "75", "+0 %"]
    assert byname["ATC_RAT_YAW_FLTE"][2] == "2" and byname["MOT_THST_EXPO"][1] == "0.55"
    assert h[2] == "calculator (5 in)"
    # through analyse: the calculator table lands in the section
    _path, log = _log("e2e")
    a = tune_fuse.analyse([log], [airborne_window(log)], prop_in=5)
    sec = tune_fuse.to_section(a, whole_tool_refusal=False)
    assert "calculator" in {t["name"] for t in sec.tables}
    assert "Mission Planner calculator" in {r.name for r in sec.results}


# ------------------------------------------------------------------- refusal

def test_check_tune_on_a_10hz_log_is_the_error_block_as_skip():
    _path, log = _log("slow10", seconds=30.0, pid_hz=10)
    sec = check_tune(log, airborne_window(log))
    assert sec.key == "tune"
    note = sec.notes[0]
    assert note.startswith("ERROR: these log files cannot be used for PID tuning.\n  slow10.bin: PID_RATE_TOO_LOW - ")
    assert "LOG_BITMASK" in note and "180222 -> 180223" in note and "[roll, pitch, yaw]" in note
    assert tune.LOGGING_FIX_HEADER in note and "exit code" not in note
    skips = [r for r in sec.results if r.status == SKIP]
    assert len(skips) == 1 and skips[0].name == "refused" and skips[0].summary.startswith("PID_RATE_TOO_LOW:")
    assert skips[0].evidence["code"] == "PID_RATE_TOO_LOW" and "180223" in skips[0].evidence["fix"]
    assert sec.worst != FAIL and all(r.status in (SKIP, PASS, WARN) for r in sec.results)
    assert {r.name for r in sec.results} >= {"roll I/P ratio", "yaw FLTT", "ATC_RATE_FF_ENAB"}
    assert {t["name"] for t in sec.tables} == {"logs"}
    json.dumps(_jsonable(sec.to_dict()), allow_nan=False)
    a = tune_fuse.analyse([log], [airborne_window(log)])
    assert tune_fuse.is_whole_tool_refusal(a) and a.logs[0]["contributed"] == ["D"]
    assert [r["code"] for r in a.refusals] == ["PID_RATE_TOO_LOW"] * 3
    assert tune_fuse.refusal_block(a, with_exit_line=True).endswith("exit code 3")


# ---------------------------------------------------------------- end to end

def test_end_to_end_tier_b_recovers_the_autotune_gains():
    a = _analysis("e2e")
    truth = _calm_gains()
    assert a.identity_ok and a.per_axis["roll"]["tiers"] == dict(A=False, B=True, C=True)
    d = {r.param: r for r in a.recommendations if r.axis == "roll"}
    assert {"ATC_RAT_RLL_P", "ATC_RAT_RLL_I", "ATC_RAT_RLL_D", "ATC_ANG_RLL_P", "ATC_ACCEL_R_MAX"} <= set(d)
    figs = []
    for param, key, scale in (("ATC_RAT_RLL_P", "rat_p", 1.0), ("ATC_RAT_RLL_D", "rat_d", 1.0), ("ATC_ANG_RLL_P", "ang_p", 1.0),
                              ("ATC_ACCEL_R_MAX", "acc_max_dps2", 100.0)):
        r = d[param]
        # the log flies the truth, so a recovered value within the deadband reads "no change";
        # the tier's own value is then in the evidence
        val = r.evidence.get("deadband_value", r.value)
        method = r.evidence.get("deadband_method", r.method)
        rel = val / (truth[key] * scale) - 1.0
        figs.append(f"{param} {val / scale:.5g} vs truth {truth[key]:.5g} ({rel:+.1%}), confidence {r.confidence:.3f}, "
                    f"shown as {r.method}")
        assert method == "virtual-autotune", (param, method)
        assert abs(rel) < 0.25, (param, rel)
        assert r.confidence >= T["tune_confidence"]["warn"], (param, r.confidence, r.components)
        assert set(r.components) >= {"prior", "adequacy", "excitation", "consistency", "agreement"}
    print("  [tier B] " + "; ".join(figs))
    assert d["ATC_RAT_RLL_I"].value == pytest.approx(d["ATC_RAT_RLL_P"].value)
    e = a.per_axis["roll"]
    assert e["plant"] is not None and e["margins"]["gm_db"] > T["tune_gain_margin_db"]["warn"]
    assert e["step_current"] is not None and e["rules"]
    assert e["adequacy"]["pooled"] == 1.0 and e["adequacy"]["current"] == 1.0
    assert a.logs[0]["contributed"] == ["B", "C", "D"]
    # pitch and yaw flew no stick: refused on excitation, not on coherence
    assert {(r["axis"], r["code"]) for r in a.refusals} == {("pitch", "NO_EXCITATION"), ("yaw", "NO_EXCITATION")}
    assert "yaw" in {r.axis for r in a.recommendations} or any(r.name == "yaw insufficient data"
                                                              for r in tune_fuse.to_section(a).results)
    assert set(a.constants) >= set(tune.CONSTANTS) | set(tune_fuse.FUSE_CONSTANTS)


def test_section_of_the_end_to_end_analysis_is_complete_and_json_safe():
    a = _analysis("e2e")
    sec = tune_fuse.to_section(a, whole_tool_refusal=tune_fuse.is_whole_tool_refusal(a))
    names = {t["name"] for t in sec.tables}
    assert {"recommendations", "ceilings", "step", "plant", "margins", "params", "refusals", "logs"} <= names
    tbl = {t["name"]: t for t in sec.tables}
    assert tbl["recommendations"]["columns"] == ["axis", "param", "current", "recommended", "change %", "confidence", "method", "why"]
    assert any("warn 1, fail 2" in c for row in tbl["step"]["rows"] for c in row if isinstance(c, str))
    res = {r.name: r for r in sec.results}
    assert res["roll ATC_RAT_RLL_P"].status == PASS and res["roll gain margin"].status == PASS
    assert res["roll gain margin"].source == T["tune_gain_margin_db"]["source"]
    assert res["yaw insufficient data"].status == WARN and "NO_EXCITATION" in res["yaw insufficient data"].summary
    assert sec.notes and any(n.startswith("roll: step response:") for n in sec.notes)
    assert any(n.startswith("roll: plant fit:") for n in sec.notes) and any("SID_F_STOP_HZ 40" in n for n in sec.notes)
    assert not any(n.startswith("ERROR") for n in sec.notes)
    d = _jsonable(sec.to_dict())
    text = json.dumps(d, allow_nan=False)
    assert "NaN" not in text and "Infinity" not in text
    assert d["worst"] in (PASS, WARN)
    json.dumps(tune_fuse.analysis_to_dict(a), allow_nan=False)
    md = sec.render()
    assert "| roll  | ATC_RAT_RLL_P" in md or "ATC_RAT_RLL_P" in md


def test_tier_a_wins_when_an_autotune_log_is_added():
    _p1, atun, res = _atun_log()
    _p2, fast = _log("e2e")
    a = tune_fuse.analyse([atun, fast], [airborne_window(atun), airborne_window(fast)])
    assert a.identity_ok and a.identity_reasons == []
    d = {r.param: r for r in a.recommendations if r.axis == "roll"}
    p = d["ATC_RAT_RLL_P"]

    def tier(r):                                   # the tier behind a value, through the deadband
        return r.evidence.get("deadband_method", r.method)
    assert tier(p) == "autotune-log" and p.confidence >= 0.9
    assert p.evidence.get("deadband_value", p.value) == pytest.approx(res["roll"]["rat_p"], rel=1e-3)
    assert p.components["agreement"] > 0.9 and "virtual-autotune gives" in p.note
    assert tier(d["ATC_RAT_RLL_D"]) == "autotune-log" and tier(d["ATC_ANG_RLL_P"]) == "autotune-log"
    assert tier(d["ATC_ACCEL_R_MAX"]) == "autotune-log"
    by = {x["file_name"]: x for x in a.logs}
    assert "A" in by["atun.bin"]["contributed"] and "A" not in by["e2e.bin"]["contributed"] and "D" in by["e2e.bin"]["contributed"]
    assert a.per_axis["roll"]["pooled"]["rat_p"]["n"] == 1 and a.per_axis["roll"]["tiers"]["A"]
    sec = tune_fuse.to_section(a)
    assert "autotune" in {t["name"] for t in sec.tables}
    assert not any(r.status == FAIL for r in sec.results), [r.line() for r in sec.results if r.status == FAIL]
    assert any(n.startswith("roll: atun.bin roll: session 1 of 1") for n in sec.notes)
    json.dumps(_jsonable(sec.to_dict()), allow_nan=False)


def test_different_aircraft_is_refused():
    _p1, a_log = _log("e2e")
    _p2, b_log = _log("other_frame", seconds=30.0, params_extra=dict(FRAME_TYPE=3.0))
    a = tune_fuse.analyse([a_log, b_log], [airborne_window(a_log), airborne_window(b_log)])
    assert a.identity_ok is False and a.refusals[0]["code"] == "DIFFERENT_AIRCRAFT"
    assert "FRAME_TYPE 1 vs 3" in a.identity_reasons[0] and a.recommendations == [] and a.per_axis == {}
    assert tune_fuse.is_whole_tool_refusal(a)
    sec = tune_fuse.to_section(a)
    assert sec.notes[0].startswith("ERROR: these log files cannot be used for PID tuning.\n  all logs: DIFFERENT_AIRCRAFT - ")
    assert "-- e2e.bin" in sec.notes[0] and "-- other_frame.bin" in sec.notes[0]
    assert {r.name: r.status for r in sec.results}["aircraft identity"] == FAIL
    json.dumps(_jsonable(sec.to_dict()), allow_nan=False)


def test_analysis_is_deterministic():
    _path, log = _log("e2e")
    a1 = tune_fuse.analyse([log], [airborne_window(log)])
    a2 = tune_fuse.analyse([log], [airborne_window(log)])
    d1 = json.dumps(_jsonable(tune_fuse.to_section(a1).to_dict()), sort_keys=True)
    d2 = json.dumps(_jsonable(tune_fuse.to_section(a2).to_dict()), sort_keys=True)
    assert d1 == d2
    assert json.dumps(tune_fuse.analysis_to_dict(a1), sort_keys=True) == json.dumps(tune_fuse.analysis_to_dict(a2), sort_keys=True)


def test_alog_all_json_includes_the_tune_section():
    from dflog.cli import main
    path, _log_ = _log("e2e")
    out, err = io.StringIO(), io.StringIO()
    with redirect_stdout(out), redirect_stderr(err):
        code = main(["--no-cache", "--json", "all", path])
    doc = json.loads(out.getvalue())
    keys = [s["key"] for s in doc["sections"]]
    assert "tune" in keys and keys.index("tune") == keys.index("gust") + 1
    sec = [s for s in doc["sections"] if s["key"] == "tune"][0]
    assert sec["title"] == "PID gains from the log"
    assert any(r["name"] == "roll ATC_RAT_RLL_P" for r in sec["results"])
    assert "recommendations" in {t["name"] for t in sec["tables"]}
    assert doc["exit_code"] == code
    assert [n for n, _ in ALL_CHECKS].index("tune") == [n for n, _ in ALL_CHECKS].index("gust") + 1
    assert check_tune.__doc__.splitlines()[0] == "Recommended PID gains from PIDx/RATE/ATUN with confidence"


def test_all_constants_merges_every_table_without_duplicates():
    c = tune.all_constants()
    from dflog import tune_step, tune_ident, tune_atun
    for tbl in (tune.CONSTANTS, tune_step.CONSTANTS, tune_ident.IDENT_CONSTANTS, tune_atun.ATUN_CONSTANTS, tune_fuse.FUSE_CONSTANTS):
        assert set(tbl) <= set(c)
        for k, v in tbl.items():
            assert c[k] is v and v["source"], k
    assert len(c) == sum(len(t) for t in (tune.CONSTANTS, tune_step.CONSTANTS, tune_ident.IDENT_CONSTANTS,
                                           tune_atun.ATUN_CONSTANTS, tune_fuse.FUSE_CONSTANTS))
    json.dumps(_jsonable(c), allow_nan=False)


class _raises:
    def __init__(self, exc):
        self.exc = exc

    def __enter__(self):
        return self

    def __exit__(self, et, ev, tb):
        assert et is not None and issubclass(et, self.exc), f"expected {self.exc.__name__}"
        return True


if __name__ == "__main__":
    import _shim
    sys.exit(_shim.run(sys.modules[__name__]))
