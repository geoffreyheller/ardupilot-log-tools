"""PID tuning from logs, work package 6: tier D parameter consistency, the confidence
model, fusion of the three evidence tiers, the `analyse` pipeline and the `tune` Section.

`docs/pid-tuning-plan.md` sections 2.3 (pipeline), 2.5 (confidence and fusion), 2.6 (the
refusal block) and 9 (yaw, ACC_MAX). Everything numeric an algorithm here uses is in
`FUSE_CONSTANTS` / `PRIORS` with a source; every graded number goes through
`dflog.analysis._grade` against `checks.T`.

What this module adds on top of WP1-WP5:

* `param_consistency(gains, prop_in)` - tier D, always available from `PARM`: I/P vs
  AutoTune's final ratio, FLTD/FLTT vs `INS_GYRO_FILTER/2`, D/P reported, the AC_PID
  parameter ranges, `ATC_RATE_FF_ENAB`, SMAX; with `prop_in` the Mission Planner
  initial-parameter formulas as a configured-vs-calculator table (`calculator_rows`).
* `confidence(method, adequacy, excitation, consistency, agreement)` - the product of
  named components with a prior per method; tier C values are capped below the withheld
  band until the step criteria are calibrated on a real fast-logged flight (the WP3
  calibration caveat in plan section 2.5).
* `fuse(per_axis, current_gains)` - tier A > B > C per parameter, a margin ceiling clips
  to itself and an oscillation ceiling to `0.4 x ceiling`, the applied rate set is
  re-checked against the margins, `I` follows `P` by the axis ratio unless the log's own ratio is
  non-default, agreement between tiers, the withheld band, `ACC_MAX` only at high
  confidence, yaw P/FLTE only (D with `AUTOTUNE_AXES` bit 8).
* `analyse(logs, windows, ...)` - identity gate, extraction, the tiers, fusion, the
  deduplicated refusal list, per-log "which tiers this log fed". No exception escapes:
  a tier that crashes is recorded in `per_axis[axis]["errors"]` and rendered FAIL.
* `to_section(analysis, whole_tool_refusal)` - the `tune` Section for `alog all` and
  `alog tune`, refusal block first when nothing could be used.

Rules this module follows (RULES.md 1-3): a refusal is a SKIP, never a pass; every
result carries the number, the threshold and its source; a defaulted parameter is named;
output is deterministic and JSON-safe (no NaN, no complex, no arrays in evidence).
Imports `dflog.analysis` lazily inside functions only (it imports this module for
`check_tune`).
"""

from __future__ import annotations

import dataclasses
import math
import os
import re
import traceback

import numpy as np

from .checks import T, Result, PASS, WARN, FAIL, SKIP
from . import tune, tune_step, tune_atun, tune_ident
from .tune import (AXES, RAT_STEM, ANG_P_PARAM, ACC_MAX_PARAMS, CONSTANTS, GainSet, StepResponse,
                   PlantModel, Ceiling, Recommendation, TuneAnalysis, Refusal, extract_axes,
                   logging_requirements, format_logging_fix, LOGGING_FIX_HEADER)

__all__ = ["PRIORS", "FUSE_CONSTANTS", "ACPID_RANGES", "TIER_ORDER", "REC_FIELDS",
           "mission_planner_initial", "param_consistency", "calculator_rows", "gain_rows",
           "adequacy", "confidence", "tier_candidates", "fuse", "analyse", "is_whole_tool_refusal",
           "refusal_block", "to_section", "analysis_to_dict"]

# ------------------------------------------------------------------------- constants

_SRC_PLAN25 = "docs/pid-tuning-plan.md section 2.5 (confidence and fusion)"
_SRC_PLAN23 = "docs/pid-tuning-plan.md section 2.3 (pipeline)"
_SRC_PLAN9 = "docs/pid-tuning-plan.md section 9 (open decisions 3 and 4: yaw, ACC_MAX)"
_SRC_CAVEAT = "docs/pid-tuning-plan.md section 2.5, tier C calibration caveat (WP3, 2026-09-17)"
_SRC_ACPID_RANGES = ("ArduCopter AC_AttitudeControl_Multi.cpp var_info @Range for ATC_RAT_*; "
                     "reference/pid-tuning-sources.md section 2")
_SRC_WIKI6 = "ardupilot.org wiki setting-up-for-tuning.rst / autotune.rst; reference/pid-tuning-sources.md section 6"
_SRC_MP = "Mission Planner ConfigInitialParams.cs; reference/pid-tuning-sources.md section 6"
_SRC_AXES = "ArduCopter AC_AutoTune AUTOTUNE_AXES bitmask (1 roll, 2 pitch, 4 yaw, 8 yaw D)"


def _c(value, source, note=""):
    return dict(value=value, source=source, note=note)


#: Confidence prior per method (plan section 2.5). Keys are `Recommendation.method`.
PRIORS = {
    "autotune-log":     _c(1.0, _SRC_PLAN25, "the gains the firmware found and saved, re-derived from ATUN"),
    "virtual-autotune": _c(0.8, _SRC_PLAN25, "AutoTune's own search run on the identified plant"),
    "step-rules":       _c(0.5, _SRC_PLAN25, "bounded adjustments from the deconvolved step response"),
    "ceiling":          _c(0.7, _SRC_PLAN25, "a measured oscillation ceiling x QuickTune's 0.4 margin"),
    "unchanged":        _c(0.0, _SRC_PLAN9, "not a recommendation: the parameter is left as configured and the row says why"),
}

#: Fusion constants (not gradings). Merged into `alog schema`'s `tune_constants` by
#: `tune.all_constants()`.
FUSE_CONSTANTS = {
    "fuse_prior_autotune_log":     _c(PRIORS["autotune-log"]["value"], _SRC_PLAN25, "confidence prior, tier A"),
    "fuse_prior_virtual_autotune": _c(PRIORS["virtual-autotune"]["value"], _SRC_PLAN25, "confidence prior, tier B"),
    "fuse_prior_step_rules":       _c(PRIORS["step-rules"]["value"], _SRC_PLAN25, "confidence prior, tier C"),
    "fuse_prior_ceiling":          _c(PRIORS["ceiling"]["value"], _SRC_PLAN25, "confidence prior, a ceiling-derived value"),
    "fuse_tier_c_confidence_cap":  _c(0.39, _SRC_CAVEAT, "step-rules values are capped below T['tune_confidence']['fail'] "
                                                         "(the withheld band) until the step criteria are calibrated on a "
                                                         "real fast-logged flight"),
    "fuse_partial_session_excitation": _c(0.3, _SRC_PLAN25, "tier A excitation when the axis did not reach TUNE_COMPLETE "
                                                             "(1.0 when it did)"),
    "fuse_eps_scale":              _c(0.25, _SRC_PLAN25, "tier B consistency = max(0, 1 - eps_mag_at_crossover / this)"),
    "fuse_agreement_floor":        _c(0.5, _SRC_PLAN25, "agreement = 1 - |delta| / max(values), clipped to [this, 1]"),
    "fuse_agreement_excludes":     _c(("step-rules",), _SRC_CAVEAT, "methods whose value never sets another tier's agreement: "
                                                                   "the step rules read ~2.4x overshoot even on AutoTune's own "
                                                                   "oracle step, so their disagreement is the caveat, not evidence"),
    "fuse_band_ref_lo_hz":         _c(0.5, _SRC_PLAN25, "tier B excitation = coherence x band width relative to [this, FLTD]"),
    "fuse_tier_b_fit_rms_db_max":  _c(3.0, _SRC_PLAN25, "tier B is used only when the plant fit residual is below this"),
    "fuse_deadband_pct":           _c(5.0, "2026-09-30 Brisket, the same gains re-identified on brisket-t1 and brisket-t2: margin-ceiling "
                                           "P moved 3-4 %, plant k 4 %, the unchanged roll loop's PM 0.9 deg",
                                      "flight-to-flight variation of the same aircraft and gains; not applied to rate "
                                      "gains while the current loop misses the margins"),
    "fuse_dp_ratio_default":       _c(0.0036 / 0.135, _SRC_WIKI6, "firmware default D/P = 0.0036/0.135 = 0.027; the wiki's "
                                                                  "'D = 1/10 P' predates the current scaling; reported, not graded"),
    "fuse_autotune_axes_yaw_d_bit": _c(8, _SRC_AXES, "yaw D is recommended only when this bit is set in AUTOTUNE_AXES"),
    "fuse_gain_equal_rel":         _c(1e-9, _SRC_PLAN23, "two gain sets are 'the current one' when every gain agrees within this"),
    "fuse_mp_gyro_coef":           _c((289.22, -0.838, 20.0), _SRC_MP, "INS_GYRO_FILTER = max(20, round(289.22 prop^-0.838))"),
    "fuse_mp_flt_floor_hz":        _c(10.0, _SRC_MP, "FLTD = FLTT = max(10, INS_GYRO_FILTER / 2)"),
    "fuse_mp_accel_rp_coef":       _c((-2.613267, 343.39216, -15083.7121, 235771.0, 10000.0), _SRC_MP,
                                      "ATC_ACCEL_R/P_MAX cdeg/s^2 = max(10000, round100(cubic in prop inches))"),
    "fuse_mp_accel_y_coef":        _c((-900.0, 36000.0, 8000.0), _SRC_MP, "ATC_ACCEL_Y_MAX cdeg/s^2 = max(8000, round100(-900 prop + 36000))"),
    "fuse_mp_expo_coef":           _c((0.15686, 0.23693, 0.80), _SRC_MP, "MOT_THST_EXPO = min(round2(0.15686 ln prop + 0.23693), 0.80)"),
    "fuse_mp_flte_yaw_hz":         _c(2.0, _SRC_MP, "ATC_RAT_YAW_FLTE 2 Hz; roll/pitch FLTE 0"),
}

#: AC_PID parameter ranges (sources section 2), per axis: (low, high).
ACPID_RANGES = {
    "roll":  dict(P=(0.01, 0.5), I=(0.01, 2.0), D=(0.0, 0.05), SMAX=(0.0, 200.0)),
    "pitch": dict(P=(0.01, 0.5), I=(0.01, 2.0), D=(0.0, 0.05), SMAX=(0.0, 200.0)),
    "yaw":   dict(P=(0.10, 2.50), I=(0.01, 1.0), D=(0.0, 0.02), SMAX=(0.0, 200.0)),
}

#: Fusion priority (plan section 2.5): tier A, then B, then a measured ceiling, then C.
TIER_ORDER = ("autotune-log", "virtual-autotune", "ceiling", "step-rules")

#: The `GainSet` fields a recommendation may address, in report order.
REC_FIELDS = ("rat_p", "rat_i", "rat_d", "ang_p", "acc_max_dps2", "flte")
_FIELD_SUFFIX = {"P": "rat_p", "I": "rat_i", "D": "rat_d", "FLTE": "flte"}
_GAIN_FIELDS_EQUAL = ("rat_p", "rat_i", "rat_d", "rat_ff", "fltd", "fltt", "flte", "ang_p", "gyro_filter")

_INF = float("inf")


def _fc(name):
    return FUSE_CONSTANTS[name]["value"]


def _fin(x):
    try:
        return x is not None and math.isfinite(float(x))
    except (TypeError, ValueError):
        return False


def _fmt(x, nd=4):
    if not _fin(x):
        return "-"
    x = float(x)
    if x == int(x) and abs(x) < 1e12:
        return str(int(x))
    return f"{x:.{nd}g}"


def _safe(v):
    """JSON-safe copy of anything a tier produced: dataclasses and Refusals to dicts,
    numpy to Python, non-finite floats to None, complex dropped (None), tuples to lists."""
    if isinstance(v, Refusal):
        return v.to_dict()
    if isinstance(v, Ceiling):
        return dict(param=v.param, value=_safe(v.value), method=v.method, evidence=_safe(v.evidence))
    if isinstance(v, GainSet):
        return _safe(dataclasses.asdict(v))
    if isinstance(v, (complex, np.complexfloating)):
        return None
    if isinstance(v, np.generic):
        v = v.item()
    if isinstance(v, bool):
        return v
    if isinstance(v, float):
        return v if math.isfinite(v) else None
    if isinstance(v, np.ndarray):
        return _safe(v.tolist())
    if isinstance(v, dict):
        return {str(k): _safe(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_safe(x) for x in v]
    if dataclasses.is_dataclass(v) and not isinstance(v, type):
        return _safe(dataclasses.asdict(v))
    return v


# ------------------------------------------------------------------ tier D: parameters

def mission_planner_initial(prop_in):
    """The Mission Planner initial-parameter calculator (sources section 6) for a prop
    diameter in inches: `ins_gyro_filter`, `fltd`, `fltt`, `flte_rp`, `flte_yaw`,
    `accel_rp_cdss`, `accel_y_cdss`, `mot_thst_expo`."""
    p = float(prop_in)
    if not p > 0:
        raise ValueError(f"prop_in must be positive inches, not {prop_in!r}")
    a, b, floor = _fc("fuse_mp_gyro_coef")
    gyro = max(floor, float(round(a * p ** b)))
    flt = max(float(_fc("fuse_mp_flt_floor_hz")), gyro / 2.0)
    c3, c2, c1, c0, acc_floor = _fc("fuse_mp_accel_rp_coef")
    acc_rp = max(acc_floor, 100.0 * round((c3 * p ** 3 + c2 * p ** 2 + c1 * p + c0) / 100.0))
    y1, y0, y_floor = _fc("fuse_mp_accel_y_coef")
    acc_y = max(y_floor, 100.0 * round((y1 * p + y0) / 100.0))
    e1, e0, e_cap = _fc("fuse_mp_expo_coef")
    expo = min(round(e1 * math.log(p) + e0, 2), e_cap)
    return dict(prop_in=p, ins_gyro_filter=gyro, fltd=flt, fltt=flt, flte_rp=0.0,
                flte_yaw=float(_fc("fuse_mp_flte_yaw_hz")), accel_rp_cdss=acc_rp, accel_y_cdss=acc_y,
                mot_thst_expo=expo, source=_SRC_MP)


def _pi_ratio_key(axis):
    return "autotune_yaw_pi_ratio_final" if axis == "yaw" else "autotune_pi_ratio_final"


def param_consistency(gains, prop_in=None, aircraft_wide=True, params=None):
    """Tier D: what the configured gain set says about itself (plan section 2.4).

    gains          a `GainSet`
    prop_in        prop diameter in inches: adds a PASS-informational result naming the
                   Mission Planner calculator values (the table is `calculator_rows`)
    aircraft_wide  include the aircraft-wide results (`ATC_RATE_FF_ENAB`, the calculator
                   note) - pass False for every axis but the first
    params         the log's parameter dict, for `MOT_THST_EXPO` in the calculator note

    Returns a list of `Result`: `<axis> I/P ratio` graded by `tune_pi_ratio_dev`,
    `<axis> FLTD` / `<axis> FLTT` graded by `tune_flt_ratio_dev`, `<axis> D/P ratio`
    (reported, not graded), `<axis> AC_PID ranges` (PASS/WARN, source var_info),
    `ATC_RATE_FF_ENAB` (WARN unless 1, source wiki), `<axis> SMAX` (reported). Never
    produces a recommended gain.
    """
    from .analysis import _grade
    g = gains
    axis = g.axis
    stem = RAT_STEM[axis]
    out = []
    defaulted = list(g.defaulted)

    # I/P vs AutoTune's final ratio
    key = _pi_ratio_key(axis)
    expected = float(CONSTANTS[key]["value"])
    if g.rat_p > 0:
        ratio = g.rat_i / g.rat_p
        dev = abs(ratio - expected) / expected
        r = _grade(dev, "tune_pi_ratio_dev", name=f"{axis} I/P ratio",
                   rat_p=g.rat_p, rat_i=g.rat_i, ratio=ratio, expected=expected, expected_source=CONSTANTS[key]["source"],
                   param_i=stem + "I", param_p=stem + "P", defaulted=[d for d in defaulted if d in (stem + "I", stem + "P")])
        r.summary = (f"I/P = {g.rat_i:g}/{g.rat_p:g} = {ratio:.3f}, AutoTune saves {expected:g} ({key.upper()}); "
                     f"deviation {dev:.2f} (warn {T['tune_pi_ratio_dev']['warn']}, fail {T['tune_pi_ratio_dev']['fail']})")
    else:
        r = Result(f"{axis} I/P ratio", SKIP, f"{stem}P is {g.rat_p:g}; no ratio", evidence=dict(rat_p=g.rat_p, rat_i=g.rat_i),
                   source=T["tune_pi_ratio_dev"]["source"])
    out.append(r)

    # FLTD, FLTT vs INS_GYRO_FILTER / 2. The wiki rule is written for roll/pitch FLTD and
    # FLTT and for yaw FLTT only (sources section 6): yaw D is 0 by default, so yaw FLTD
    # filters nothing and is reported, not graded.
    half = g.gyro_filter / 2.0 if g.gyro_filter else 0.0
    for fld, name in (("fltd", "FLTD"), ("fltt", "FLTT")):
        v = float(getattr(g, fld))
        if axis == "yaw" and name == "FLTD":
            r = Result(f"{axis} {name}", PASS,
                       f"{stem}{name} {v:g} Hz (INS_GYRO_FILTER/2 = {half:g} Hz); not in the wiki rule, which covers roll/pitch "
                       f"FLTD/FLTT and yaw FLTT only - yaw D is {g.rat_d:g}, so FLTD "
                       + ("filters nothing; reported, not graded" if g.rat_d == 0 else "acts only on that D term; reported, not graded"),
                       evidence=dict(value_hz=v, gyro_filter_hz=g.gyro_filter, expected_hz=half, rat_d=g.rat_d, graded=False,
                                     param=stem + name), source=_SRC_WIKI6)
        elif half > 0:
            dev = abs(v / half - 1.0)
            r = _grade(dev, "tune_flt_ratio_dev", name=f"{axis} {name}", value_hz=v, gyro_filter_hz=g.gyro_filter,
                       expected_hz=half, param=stem + name, defaulted=[d for d in defaulted if d in (stem + name, "INS_GYRO_FILTER")])
            r.summary = (f"{stem}{name} {v:g} Hz vs INS_GYRO_FILTER/2 = {half:g} Hz; deviation {dev:.2f} "
                         f"(warn {T['tune_flt_ratio_dev']['warn']}, fail {T['tune_flt_ratio_dev']['fail']})")
        else:
            r = Result(f"{axis} {name}", SKIP, f"INS_GYRO_FILTER is {g.gyro_filter:g}; no reference for {stem}{name} {v:g} Hz",
                       evidence=dict(value_hz=v, gyro_filter_hz=g.gyro_filter), source=T["tune_flt_ratio_dev"]["source"])
        out.append(r)

    # D/P reported
    dp_default = float(_fc("fuse_dp_ratio_default"))
    if g.rat_p > 0:
        dp = g.rat_d / g.rat_p
        out.append(Result(f"{axis} D/P ratio", PASS,
                          f"D/P = {g.rat_d:g}/{g.rat_p:g} = {dp:.4f} (firmware default {dp_default:.3f}; reported, not graded - "
                          "the wiki's 'D = 1/10 P' predates the current scaling)",
                          evidence=dict(rat_d=g.rat_d, rat_p=g.rat_p, ratio=dp, firmware_default=dp_default, graded=False),
                          source=FUSE_CONSTANTS["fuse_dp_ratio_default"]["source"]))

    # AC_PID ranges
    ranges = ACPID_RANGES[axis]
    vals = dict(P=g.rat_p, I=g.rat_i, D=g.rat_d, SMAX=g.smax)
    outside, parts = [], []
    for k in ("P", "I", "D", "SMAX"):
        lo, hi = ranges[k]
        ok = lo <= vals[k] <= hi
        parts.append(f"{k} {vals[k]:g} {'in' if ok else 'OUTSIDE'} {lo:g}-{hi:g}")
        if not ok:
            outside.append(stem + k)
    out.append(Result(f"{axis} AC_PID ranges", WARN if outside else PASS,
                      ("; ".join(parts) + (f"; outside the parameter range: {', '.join(outside)}" if outside else "")),
                      evidence=dict(values={stem + k: vals[k] for k in vals}, ranges={stem + k: list(ranges[k]) for k in ranges},
                                    outside=outside), source=_SRC_ACPID_RANGES))

    # SMAX
    out.append(Result(f"{axis} SMAX", PASS,
                      f"{stem}SMAX {g.smax:g}" + (" (slew limiter off: Dmod stays 1.000)" if g.smax <= 0 else
                                                  " (slew limiter armed; PIDx.Dmod < 1 marks where it engaged)"),
                      evidence=dict(smax=g.smax, param=stem + "SMAX"), source=_SRC_ACPID_RANGES))

    if aircraft_wide:
        ff = "ATC_RATE_FF_ENAB"
        out.append(Result(ff, PASS if g.ff_enab else WARN,
                          f"{ff} = {1 if g.ff_enab else 0}" + ("" if g.ff_enab else
                                                               " - the wiki's tuning setup expects 1 (rate feed-forward on)")
                          + (" (defaulted: not in the log)" if ff in defaulted else ""),
                          evidence=dict(value=1 if g.ff_enab else 0, expected=1, defaulted=ff in defaulted), source=_SRC_WIKI6))
        if prop_in is not None:
            mp = mission_planner_initial(prop_in)
            expo = (params or {}).get("MOT_THST_EXPO")
            out.append(Result("Mission Planner calculator", PASS,
                              f"prop {mp['prop_in']:g} in: INS_GYRO_FILTER {mp['ins_gyro_filter']:g} Hz (configured "
                              f"{g.gyro_filter:g}), FLTD/FLTT {mp['fltd']:g} Hz, ATC_ACCEL_R/P_MAX {mp['accel_rp_cdss']:.0f} and "
                              f"Y {mp['accel_y_cdss']:.0f} cdeg/s^2, MOT_THST_EXPO {mp['mot_thst_expo']:.2f}"
                              + (f" (configured {float(expo):g})" if expo is not None else " (MOT_THST_EXPO not in log)")
                              + "; comparison only, see the calculator table",
                              evidence=dict(calculator=mp, configured_gyro_filter=g.gyro_filter,
                                            configured_mot_thst_expo=None if expo is None else float(expo)),
                              source=_SRC_MP))
    return out


def calculator_rows(gains_by_axis, prop_in, params=None):
    """(headers, rows) comparing the configured values with the Mission Planner
    initial-parameter calculator for `prop_in` inches: `INS_GYRO_FILTER`, per-axis
    FLTD/FLTT/FLTE, `ATC_ACC(EL)_x_MAX` in the log's own spelling and unit,
    `MOT_THST_EXPO`. Notes only; nothing here is graded."""
    mp = mission_planner_initial(prop_in)
    rows = []
    first = next(iter(gains_by_axis.values()))

    def row(param, configured, calc, unit, note=""):
        if _fin(configured) and _fin(calc) and float(calc) != 0:
            delta = f"{(float(configured) / float(calc) - 1.0) * 100:+.0f} %"
        else:
            delta = "-"
        rows.append([param, _fmt(configured), _fmt(calc), delta, unit, note])

    row("INS_GYRO_FILTER", first.gyro_filter, mp["ins_gyro_filter"], "Hz",
        "max(20, round(289.22 prop^-0.838))" + (" [defaulted]" if "INS_GYRO_FILTER" in first.defaulted else ""))
    for axis, g in gains_by_axis.items():
        stem = RAT_STEM[axis]
        row(stem + "FLTD", g.fltd, mp["fltd"], "Hz", "max(10, gyro/2)")
        row(stem + "FLTT", g.fltt, mp["fltt"], "Hz", "max(10, gyro/2)")
        row(stem + "FLTE", g.flte, mp["flte_yaw"] if axis == "yaw" else mp["flte_rp"], "Hz", "yaw 2, roll/pitch 0")
        name = g.param_names.get("acc_max_dps2") or ACC_MAX_PARAMS[axis][1]
        calc_cdss = mp["accel_y_cdss"] if axis == "yaw" else mp["accel_rp_cdss"]
        if "ACCEL" in name:
            row(name, (g.acc_max_dps2 or 0.0) * 100.0, calc_cdss, "cdeg/s^2",
                "cubic in prop (roll/pitch) / linear (yaw)" + (" [defaulted]" if name in g.defaulted else ""))
        else:
            row(name, g.acc_max_dps2, calc_cdss / 100.0, "deg/s^2",
                "cubic in prop (roll/pitch) / linear (yaw)" + (" [defaulted]" if name in g.defaulted else ""))
    expo = (params or {}).get("MOT_THST_EXPO")
    row("MOT_THST_EXPO", None if expo is None else float(expo), mp["mot_thst_expo"], "-",
        "min(round2(0.15686 ln prop + 0.23693), 0.80)" + ("" if expo is not None else " [not in log]"))
    return ["param", "configured", f"calculator ({mp['prop_in']:g} in)", "delta", "unit", "formula"], rows


def gain_rows(gains_by_axis):
    """(headers, rows) of the configured gain sets, one row per axis."""
    rows = []
    for axis, g in gains_by_axis.items():
        acc_name = g.param_names.get("acc_max_dps2") or ACC_MAX_PARAMS[axis][1]
        acc = (g.acc_max_dps2 or 0.0) * (100.0 if "ACCEL" in acc_name else 1.0)
        rows.append([axis, _fmt(g.rat_p), _fmt(g.rat_i), _fmt(g.rat_d), _fmt(g.rat_ff), _fmt(g.fltt), _fmt(g.flte),
                     _fmt(g.fltd), _fmt(g.smax), _fmt(g.imax), _fmt(g.ang_p), f"{acc_name} {_fmt(acc)}",
                     _fmt(g.gyro_filter), ", ".join(g.defaulted) or "-"])
    return ["axis", "P", "I", "D", "FF", "FLTT", "FLTE", "FLTD", "SMAX", "IMAX", "ANG_P", "ACC_MAX", "INS_GYRO_FILTER",
            "defaulted"], rows


# ------------------------------------------------------------------------ confidence

def adequacy(fs, limited_pct=0.0):
    """Plan section 2.5: 0 below `tune_pid_rate_hz.fail`, 1 at or above `.warn`, linear
    between, times the fraction of the segment not output-limited."""
    lo, hi = float(T["tune_pid_rate_hz"]["fail"]), float(T["tune_pid_rate_hz"]["warn"])
    fs = float(fs)
    a = 0.0 if fs < lo else 1.0 if fs >= hi else (fs - lo) / (hi - lo)
    lim = float(limited_pct) if _fin(limited_pct) else 0.0
    return float(a * max(0.0, 1.0 - lim / 100.0))


def _clip01(x):
    return float(min(1.0, max(0.0, float(x)))) if _fin(x) else 0.0


def confidence(method, adequacy, excitation, consistency, agreement=1.0):
    """`prior[method] x adequacy x excitation x consistency x agreement`, every factor in
    [0, 1] (agreement floored at `fuse_agreement_floor`). Tier C (`step-rules`) is
    additionally capped at `fuse_tier_c_confidence_cap` (the calibration caveat).
    Returns `(value, components)`; `components` carries every factor, the uncapped
    `product`, the `cap` applied (or None) and the `value`."""
    if method not in PRIORS:
        raise ValueError(f"unknown method {method!r}; expected one of {sorted(PRIORS)}")
    prior = float(PRIORS[method]["value"])
    floor = float(_fc("fuse_agreement_floor"))
    comps = dict(prior=prior, adequacy=_clip01(adequacy), excitation=_clip01(excitation),
                 consistency=_clip01(consistency), agreement=float(min(1.0, max(floor, float(agreement)))) if _fin(agreement) else 1.0)
    product = comps["prior"] * comps["adequacy"] * comps["excitation"] * comps["consistency"] * comps["agreement"]
    cap = float(_fc("fuse_tier_c_confidence_cap")) if method == "step-rules" else None
    value = min(product, cap) if cap is not None else product
    comps.update(product=float(product), cap=cap, value=float(value), prior_source=PRIORS[method]["source"])
    return float(value), comps


# --------------------------------------------------------------------------- fusion

def _param_name(axis, field, gains):
    if field == "ang_p":
        return ANG_P_PARAM[axis]
    if field == "acc_max_dps2":
        return gains.param_names.get("acc_max_dps2") or ACC_MAX_PARAMS[axis][1]
    return RAT_STEM[axis] + {"rat_p": "P", "rat_i": "I", "rat_d": "D", "flte": "FLTE"}[field]


def _field_of(param):
    """GainSet field for a rate/angle parameter name, or None."""
    m = re.match(r"^ATC_RAT_(RLL|PIT|YAW)_(P|I|D|FLTE)$", param)
    if m:
        return _FIELD_SUFFIX[m.group(2)]
    if re.match(r"^ATC_ANG_(RLL|PIT|YAW)_P$", param):
        return "ang_p"
    if re.match(r"^ATC_ACC(EL)?_[RPY]_MAX$", param):
        return "acc_max_dps2"
    return None


def _unit_scale(axis, field, gains):
    """Multiplier from the GainSet unit to the log's parameter spelling (cdeg/s^2 for
    `ATC_ACCEL_x_MAX`), and the unit text."""
    if field == "acc_max_dps2":
        name = _param_name(axis, field, gains)
        return (100.0, "cdeg/s^2") if "ACCEL" in name else (1.0, "deg/s^2")
    if field == "flte":
        return 1.0, "Hz"
    return 1.0, ""


def _pct(value, current):
    if _fin(value) and _fin(current) and float(current) != 0:
        return float((float(value) / float(current) - 1.0) * 100.0)
    return float("nan")


def _cand(field, value, method, adequacy_, excitation, consistency, evidence, note=""):
    return dict(field=field, value=float(value), method=method, adequacy=float(adequacy_), excitation=float(excitation),
                consistency=float(consistency), evidence=evidence, note=note)


def tier_candidates(axis, entry, current, autotune_axes=None):
    """The per-parameter values each tier proposes for `axis`, before fusion.

    entry     the `per_axis[axis]` dict of `analyse` (see its docstring); only the keys
              a tier needs must be present: `pooled` / `autotune` (A), `virtual`,
              `plant`, `margins`, `adequacy` (B), `rules`, `step_current`, `osc_current`
              (C)
    current   the current `GainSet`

    Returns a list of dicts `field, value, method, adequacy, excitation, consistency,
    evidence, note` in tier order. Yaw restricts tiers B and C to `rat_p`, `rat_i`,
    `flte` (plan section 9 item 3) and every tier's `rat_d` to `AUTOTUNE_AXES` bit 8.
    """
    cands = []
    yaw = axis == "yaw"
    bit = int(_fc("fuse_autotune_axes_yaw_d_bit"))
    yaw_d_ok = (autotune_axes is not None) and (int(autotune_axes) & bit) == bit

    def allowed(field, method):
        if yaw and field == "rat_d" and not yaw_d_ok:
            return False
        if yaw and method != "autotune-log" and field not in ("rat_p", "rat_i", "flte", "rat_d"):
            return False
        if not yaw and field == "flte":
            return False
        return True

    # ---- tier A: pooled complete sessions, else a partial session's found rate gains
    pooled = entry.get("pooled") or {}
    for field in REC_FIELDS:
        if field in pooled and allowed(field, "autotune-log"):
            e = pooled[field]
            spread = e.get("spread")
            cons = 1.0 if spread is None else max(0.0, 1.0 - float(spread))
            cands.append(_cand(field, e["median"], "autotune-log", 1.0, 1.0, cons,
                               dict(pooled=e, n_sessions=e.get("n"), spread=spread, spread_warn=T["tune_session_spread"]["warn"],
                                    spread_source=T["tune_session_spread"]["source"],
                                    adequacy_note="ATUN is logged at loop rate regardless of LOG_BITMASK"),
                               f"median of {e.get('n', 1)} AutoTune session(s)" + (f", spread {spread:.3f}" if spread is not None else "")))
    if not pooled:
        partial = [s for s in entry.get("autotune") or [] if not s.get("complete") and s.get("found")]
        if partial:
            s = partial[-1]
            b = s["backoff"]
            f = s["found"]
            exc = float(_fc("fuse_partial_session_excitation"))
            note = (f"partial AutoTune session ({s['log_name']}, ended during {s['steps'][-1]['step']}): the firmware saved "
                    f"nothing; RATE_P_UP completed, so P/D x the {b['branch']} backoff are what it would have saved")
            for field, val in (("rat_p", f["rp"] * b["rate_p"]), ("rat_d", f["rd"] * (1.0 if s["axis_id"] == 2 else b["rate_d"]))):
                if s["axis_id"] == 2 and field == "rat_d":
                    continue
                if allowed(field, "autotune-log"):
                    cands.append(_cand(field, val, "autotune-log", 1.0, exc, 1.0,
                                       dict(session=s["session"], log_name=s["log_name"], found=f, backoff=dict(rate_p=b["rate_p"], rate_d=b["rate_d"], branch=b["branch"], exact=b["exact"]),
                                            excitation_note=f"axis did not reach TUNE_COMPLETE: excitation {exc}"), note))

    # ---- tier B: the virtual AutoTune on the identified plant
    va, plant = entry.get("virtual"), entry.get("plant")
    if va and isinstance(plant, PlantModel):
        rms_max = float(_fc("fuse_tier_b_fit_rms_db_max"))
        m = entry.get("margins") or {}
        covers = m.get("np_in_band", True) is not False
        if not va.get("complete"):
            entry.setdefault("notes", []).append(f"tier B not used: the virtual AutoTune did not complete ({va.get('aborted') or 'unknown'})")
        elif plant.fit_rms_db >= rms_max:
            entry.setdefault("notes", []).append(f"tier B not used: plant fit residual {plant.fit_rms_db:.2f} dB >= {rms_max:g} dB")
        elif not covers:
            entry.setdefault("notes", []).append("tier B not used: the coherent band does not contain the loop crossover")
        else:
            adeq = float((entry.get("adequacy") or {}).get("pooled", 1.0))
            lo = float(_fc("fuse_band_ref_lo_hz"))
            hi = float(current.fltd) if current.fltd and current.fltd > lo else float(plant.band[1])
            cover = min(1.0, max(0.0, (plant.band[1] - plant.band[0]) / (hi - lo))) if hi > lo else 1.0
            exc = _clip01(plant.coh_mean_band) * cover
            cons = max(0.0, 1.0 - float(plant.eps_mag_at_crossover) / float(_fc("fuse_eps_scale")))
            ev = dict(virtual=dict((k, va.get(k)) for k in ("rat_p", "rat_i", "rat_d", "ang_p", "acc_max_dps2", "flte", "n_twitches",
                                                           "steps_completed", "final_overshoot", "final_bounce", "aborted", "why")),
                      plant=dict(k=plant.k, tau1=plant.tau1, tau2=plant.tau2, delay=plant.delay, band=list(plant.band),
                                 fit_rms_db=plant.fit_rms_db, fit_rms_deg=plant.fit_rms_deg, coh_mean_band=plant.coh_mean_band,
                                 eps_mag_at_crossover=plant.eps_mag_at_crossover, n_avg=plant.n_avg),
                      margins=dict((k, m.get(k)) for k in ("gm_db", "pm_deg", "fc_hz", "np_gm_db", "np_pm_deg", "np_fc_hz")),
                      band_cover=cover, band_ref_hz=[lo, hi], aggr=(va.get("constants") or {}).get("aggr"),
                      gmbk=(va.get("constants") or {}).get("gmbk"))
            note = f"virtual AutoTune: {va.get('why', '')}"
            for field in REC_FIELDS:
                v = va.get(field)
                if v is None or not allowed(field, "virtual-autotune"):
                    continue
                cands.append(_cand(field, v, "virtual-autotune", adeq, exc, cons, ev, note))

    # ---- tier C: the step rules on the current gain set, ceilings as their own method
    rules = entry.get("rules") or []
    st = entry.get("step_current")
    osc = entry.get("osc_current") or {}
    adeq_c = float((entry.get("adequacy") or {}).get("current", 1.0))
    ceiling_params = {c.param for c in (osc.get("ceilings") or [])}
    warn_frames = float(T["tune_min_frames"]["warn"])
    for r in rules:
        field = _field_of(r["param"])
        if field is None:
            continue
        if r["param"] in ceiling_params and osc.get("at_ceiling"):
            if allowed(field, "ceiling"):
                cands.append(_cand(field, r["value"], "ceiling", adeq_c, 1.0, 1.0,
                                   dict(rule=r, osc=dict((k, osc.get(k)) for k in ("srate_p95", "f_osc", "prominence_db", "attribution", "note")),
                                        ceiling=r["current"]),
                                   f"ceiling: {osc.get('note', '')}"))
            continue
        if not allowed(field, "step-rules"):
            continue
        exc = min(1.0, st.n_frames / warn_frames) if isinstance(st, StepResponse) else 0.0
        cons = float(st.consistency) if isinstance(st, StepResponse) else 0.0
        cands.append(_cand(field, r["value"], "step-rules", adeq_c, exc, cons,
                           dict(rule=r, n_frames=getattr(st, "n_frames", None), consistency=cons,
                                metrics=_safe(getattr(st, "metrics", {}))),
                           "uncalibrated step rules (plan section 2.5, tier C caveat): " + r["why"]))
    return cands


def _agreement(a, b):
    if not (_fin(a) and _fin(b)):
        return 1.0
    mx = max(abs(float(a)), abs(float(b)))
    if mx == 0:
        return 1.0
    return float(min(1.0, max(float(_fc("fuse_agreement_floor")), 1.0 - abs(float(a) - float(b)) / mx)))


def fuse(per_axis, current_gains, autotune_axes=None, prop_in=None):
    """Plan section 2.5: one `Recommendation` per (axis, parameter).

    per_axis        {axis: entry} as `analyse` builds it; an entry may instead carry a
                    precomputed `candidates` list (the `tier_candidates` shape) and
                    `ceilings` ([Ceiling])
    current_gains   {axis: GainSet}
    autotune_axes   the log's AUTOTUNE_AXES (yaw D only with bit 8)
    prop_in         prop diameter in inches, for the ACC_MAX side-by-side note

    Per parameter the first tier in `TIER_ORDER` with a value wins; the runner-up sets
    `agreement`. A value above an oscillation ceiling is clipped to `0.4 x ceiling` and
    the method becomes `ceiling`; above a margin ceiling (`Ceiling.includes_margin`, from
    `tune_ident.margin_ceilings`) it is clipped to the ceiling, keeping its tier's method
    and confidence. When several bind, the tightest clip wins. With a plant in the entry,
    the rate set as applied (P, I, D) is re-checked with `tune_ident.margins` and every
    rate field is withheld, reason in the note, if it misses PM/GM limits. `rat_i` is derived from the fused `rat_p` by AutoTune's ratio
    (1.0 roll/pitch, 0.1 yaw) unless the log's own I/P deviates by more than
    `T["tune_pi_ratio_dev"]["warn"]`, in which case that ratio is preserved and noted.
    `acc_max_dps2` is recommended only from tier A/B at confidence >= `tune_confidence.warn`,
    otherwise a row with the current value, method `unchanged`, says why. Below
    `tune_confidence.fail` the value moves to `evidence["withheld_value"]` and `value`
    is NaN (rendered "withheld").
    """
    warn_c, fail_c = float(T["tune_confidence"]["warn"]), float(T["tune_confidence"]["fail"])
    margin = 1.0 - float(CONSTANTS["quik_gain_margin"]["value"])
    recs = []
    for axis in AXES:
        if axis not in per_axis:
            continue
        entry = per_axis[axis]
        cur = current_gains[axis]
        cands = entry["candidates"] if "candidates" in entry else tier_candidates(axis, entry, cur, autotune_axes)
        ceilings = [c for c in (entry.get("ceilings") or []) if isinstance(c, Ceiling)]
        by_field = {}
        for c in cands:
            by_field.setdefault(c["field"], []).append(c)
        for lst in by_field.values():
            lst.sort(key=lambda c: TIER_ORDER.index(c["method"]))
        fused = {}

        for field in REC_FIELDS:
            if field == "rat_i":
                continue                                   # derived from P below
            lst = by_field.get(field)
            if not lst:
                continue
            chosen = lst[0]
            excluded = tuple(_fc("fuse_agreement_excludes"))
            runner = next((c for c in lst[1:] if c["method"] not in excluded), None)
            agree = _agreement(chosen["value"], runner["value"]) if runner else 1.0
            conf, comps = confidence(chosen["method"], chosen["adequacy"], chosen["excitation"], chosen["consistency"], agree)
            value, method = float(chosen["value"]), chosen["method"]
            notes = [chosen["note"]] if chosen["note"] else []
            if runner:
                notes.append(f"{runner['method']} gives {runner['value']:.4g} (agreement {agree:.2f})")
            for c in lst[1:]:
                if c["method"] in excluded and c is not runner:
                    notes.append(f"{c['method']} gives {c['value']:.4g} (not used for agreement: uncalibrated, plan section 2.5)")
            ev = dict(chosen=_safe(chosen), other_tiers=[_safe(dict(method=c["method"], value=c["value"], note=c["note"])) for c in lst[1:]],
                      ceilings=[_safe(c) for c in ceilings if c.param == _param_name(axis, field, cur)])
            pname = _param_name(axis, field, cur)
            over = [c for c in ceilings if c.param == pname and value > c.value]
            # the tightest effective clip wins when several ceilings bind
            over.sort(key=lambda c: float(c.value) * (1.0 if c.includes_margin else margin))
            if over:
                c = over[0]
                ev.update(unclipped_value=value, unclipped_method=method, ceiling=_safe(c))
                if c.includes_margin:
                    # the ceiling already keeps the margins and comes from the same plant as
                    # tier B: clip to it, keep the tier's method and confidence
                    ce = c.evidence or {}
                    notes.append(f"{method} value {value:.4g} is above the {c.method} ceiling {c.value:.4g} ({pname}): "
                                 f"clipped to the ceiling, the largest value keeping PM >= {ce.get('pm_min_deg', 45):g} deg "
                                 f"and GM >= {ce.get('gm_min_db', 6):g} dB on the identified plant")
                    ev.update(margin_clipped=True)
                    value = float(c.value)
                else:
                    clipped = float(c.value) * margin
                    notes.append(f"{method} value {value:.4g} is above the {c.method} ceiling {c.value:.4g} ({pname}): clipped to "
                                 f"{margin:g} x ceiling = {clipped:.4g} (QUIK_GAIN_MARGIN, source {CONSTANTS['quik_gain_margin']['source']})")
                    value, method = clipped, "ceiling"
                    conf, comps = confidence("ceiling", chosen["adequacy"], 1.0, 1.0, agree)
            fused[field] = dict(value=value, method=method, conf=conf, comps=comps, notes=notes, ev=ev)

        # I follows P
        if "rat_p" in fused:
            p = fused["rat_p"]
            expected = float(CONSTANTS[_pi_ratio_key(axis)]["value"])
            own = cur.rat_i / cur.rat_p if cur.rat_p > 0 else expected
            dev = abs(own - expected) / expected
            if dev > float(T["tune_pi_ratio_dev"]["warn"]):
                ratio, why = own, (f"the log's own I/P {own:.3f} is non-default (AutoTune's is {expected:g}, deviation {dev:.2f} > "
                                   f"{T['tune_pi_ratio_dev']['warn']}): preserved, I = P x {own:.3f}")
            else:
                ratio, why = expected, f"I follows P by AutoTune's ratio {expected:g} ({_pi_ratio_key(axis).upper()})"
            ev = dict(p_value=p["value"], ratio=ratio, own_ratio=own, expected_ratio=expected, ratio_preserved=ratio != expected,
                      tier_values=[_safe(dict(method=c["method"], value=c["value"])) for c in by_field.get("rat_i", [])])
            fused["rat_i"] = dict(value=p["value"] * ratio, method=p["method"], conf=p["conf"], comps=dict(p["comps"]),
                                  notes=[why], ev=ev)
        elif by_field.get("rat_i"):
            c = by_field["rat_i"][0]
            conf, comps = confidence(c["method"], c["adequacy"], c["excitation"], c["consistency"], 1.0)
            fused["rat_i"] = dict(value=c["value"], method=c["method"], conf=conf, comps=comps, notes=[c["note"]], ev=dict(chosen=_safe(c)))

        # ACC_MAX: only tier A/B at high confidence
        if "acc_max_dps2" in fused:
            a = fused["acc_max_dps2"]
            mp = None
            if prop_in is not None:
                mpv = mission_planner_initial(prop_in)
                mp = (mpv["accel_y_cdss"] if axis == "yaw" else mpv["accel_rp_cdss"]) / 100.0
                a["notes"].append(f"Mission Planner calculator ({prop_in:g} in): {mp:.0f} deg/s^2")
                a["ev"]["mission_planner_dps2"] = mp
            if a["method"] not in ("autotune-log", "virtual-autotune") or a["conf"] < warn_c:
                a["ev"].update(candidate_value=a["value"], candidate_method=a["method"], candidate_confidence=a["conf"])
                a["notes"].insert(0, f"left unchanged: {a['method']} proposes {a['value']:.0f} deg/s^2 at confidence {a['conf']:.2f} "
                                     f"< {warn_c:g} (plan section 9 item 4)")
                a["value"], a["method"] = float(cur.acc_max_dps2 or 0.0), "unchanged"

        # deadband: a change smaller than the same aircraft's flight-to-flight variation is
        # noise - unless the current loop misses the margins, when even a small change is the
        # correction (pitch 0.135 -> 0.129, -4.6 %, took PM 43.1 -> 46.2 deg on Brisket)
        dead = float(_fc("fuse_deadband_pct"))
        cur_m = entry.get("margins") or {}
        cur_fails = (isinstance(entry.get("plant"), PlantModel) and cur_m.get("pm_deg") is not None
                     and not tune_ident.meets_margins(cur_m))
        for field, f in fused.items():
            if f["method"] == "unchanged" or f["conf"] < fail_c or f.get("withhold"):
                continue
            if cur_fails and field in ("rat_p", "rat_i", "rat_d", "flte"):
                f["notes"].append(f"not deadbanded: the current loop misses the margins (PM {_fmt(cur_m.get('pm_deg'), 3)} deg, "
                                  f"GM {_fmt(cur_m.get('gm_db'), 3)} dB), so a small change is a correction")
                continue
            cur_f = float(getattr(cur, field) or 0.0)
            ch = _pct(f["value"], cur_f)
            if _fin(ch) and abs(ch) < dead:
                f["ev"].update(deadband_value=float(f["value"]), deadband_change_pct=ch, deadband_pct=dead,
                               deadband_method=f["method"])
                f["notes"].insert(0, f"no change: {f['method']} gives {f['value']:.4g} ({ch:+.1f} %), inside the "
                                     f"+-{dead:g} % deadband ({FUSE_CONSTANTS['fuse_deadband_pct']['note']})")
                f["value"], f["method"] = cur_f, "unchanged"

        # the rate set as it would be applied must keep the margins on the identified plant,
        # and those margins must be measured, not extrapolated beyond the coherent band
        gated = ("rat_p", "rat_i", "rat_d", "flte")
        rate_fields = [f for f in gated if f in fused and fused[f]["conf"] >= fail_c and fused[f]["method"] != "unchanged"]
        plant = entry.get("plant")
        if rate_fields and isinstance(plant, PlantModel):
            applied = {f: float(fused[f]["value"]) if f in fused and fused[f]["conf"] >= fail_c
                       else float(getattr(cur, f) or 0.0) for f in gated}
            m = tune_ident.margins(plant, dataclasses.replace(cur, **applied))
            pm_min, gm_min = tune_ident.margin_limits()
            ok = tune_ident.meets_margins(m)
            extra = tune_ident.extrapolated(plant, m)
            txt = (f"the rate set as applied (P {applied['rat_p']:.4g} I {applied['rat_i']:.4g} D {applied['rat_d']:.4g}"
                   + (f" FLTE {applied['flte']:.3g}" if axis == "yaw" else "") + ") "
                   f"gives PM {_fmt(m['pm_deg'], 3)} deg, GM {_fmt(m['gm_db'], 3)} dB on the identified plant "
                   f"(limits {pm_min:g} deg / {gm_min:g} dB)")
            gate = dict(applied=applied, pm_deg=m["pm_deg"], gm_db=m["gm_db"], fc_hz=m["fc_hz"], f180_hz=m["f180_hz"],
                        ok=ok and not extra, meets=ok, extrapolated=extra, band=list(plant.band),
                        pm_min_deg=pm_min, gm_min_db=gm_min)
            for f in rate_fields:
                fused[f]["ev"]["margin_gate"] = gate
                if not ok:
                    fused[f]["withhold"] = f"{txt}: below the limits"
                elif extra:
                    fused[f]["withhold"] = (f"{txt}, but not measured: " + "; ".join(extra)
                                            + " - fly a SysID sweep on this axis (SID_F_STOP_HZ 40) or sharper stick inputs")
                else:
                    fused[f]["notes"].append(txt)

        # the virtual AutoTune found ANG_P and ACC_MAX on its own, unclipped rate loop; once
        # the rate P is margin-clipped they belong to a loop that will not fly
        rp = fused.get("rat_p") or {}
        if (rp.get("ev") or {}).get("margin_clipped"):
            why = (f"found by the virtual AutoTune on its rate loop with P {rp['ev'].get('unclipped_value', float('nan')):.4g}, "
                   f"but the rate P is clipped to {rp['value']:.4g}: fly the clipped rate set, then re-run on that log")
            for fld in ("ang_p", "acc_max_dps2"):
                f = fused.get(fld)
                if f and f["method"] == "virtual-autotune":
                    f["withhold"] = why
                elif f and f["method"] == "unchanged":
                    f["notes"].append(why)

        for field in REC_FIELDS:
            if field not in fused:
                continue
            f = fused[field]
            scale, unit = _unit_scale(axis, field, cur)
            cur_v = float(getattr(cur, field) or 0.0) * scale
            val = float(f["value"]) * scale
            ev = dict(f["ev"], unit=unit, components=f["comps"])
            if "deadband_value" in ev:
                ev["deadband_value"] = float(ev["deadband_value"]) * scale      # parameter units, like `value`
            note = "; ".join(n for n in f["notes"] if n)
            if f.get("withhold"):
                ev.update(withheld_value=val, withheld_change_pct=_pct(val, cur_v))
                note = f"withheld: {f['withhold']}; the value {val:.4g} is in the evidence only" + (f"; {note}" if note else "")
                val = float("nan")
            elif f["conf"] < fail_c and f["method"] != "unchanged":
                ev.update(withheld_value=val, withheld_change_pct=_pct(val, cur_v))
                note = (f"withheld: confidence {f['conf']:.2f} < {fail_c:g} ({T['tune_confidence']['source']}); "
                        f"the value {val:.4g} is in the evidence only" + (f"; {note}" if note else ""))
                val = float("nan")
            recs.append(Recommendation(axis=axis, param=_param_name(axis, field, cur), current=cur_v, value=val,
                                       change_pct=_pct(val, cur_v), method=f["method"], confidence=float(f["conf"]),
                                       components=dict(f["comps"]), evidence=_safe(ev), note=note))
    return recs


# ------------------------------------------------------------------------- analyse

def _log_name(log, given=None):
    if given:
        return given
    path = getattr(log, "path", None)
    return os.path.basename(str(path)) if path else "log"


def _mcu_id(log):
    """The MCU serial words of the board banner ('<board> 00000000 00000000 00000000'),
    or '' when no such line exists."""
    try:
        for _, m in log.messages_text():
            hit = re.match(r"^[A-Za-z0-9_\-]+\s+((?:[0-9A-Fa-f]{8}\s*){2,})$", str(m).strip())
            if hit:
                return " ".join(hit.group(1).split())
    except Exception:
        pass
    return ""


def _identity(log, w, name):
    info = log.info()
    version = tune_atun.firmware_version(info.get("firmware"))
    try:
        boot = log.boot_time_unix()
    except Exception:
        boot = None
    p = log.params()
    return dict(file_name=name, path=info.get("path"), firmware=info.get("firmware") or "", version=list(version) if version else None,
                board=info.get("board") or "", mcu=_mcu_id(log), frame_class=info.get("frame_class"),
                frame_type=info.get("frame_type"), boot_time_unix=boot, window=w.to_dict(),
                integrity_ok=bool(log.diagnostics.ok), autotune_axes=p.get("AUTOTUNE_AXES"),
                requirements=logging_requirements(log), contributed=[], refusals=[], errors=[])


def _identity_gate(ids):
    """(ok, reasons): frame class/type, board and MCU must agree (when both known);
    firmware major.minor differing is a reason but not a refusal."""
    ok, reasons = True, []
    if len(ids) < 2:
        return ok, reasons
    ref = ids[0]
    for other in ids[1:]:
        pair = f"{ref['file_name']} vs {other['file_name']}"
        for key, label in (("frame_class", "FRAME_CLASS"), ("frame_type", "FRAME_TYPE")):
            a, b = ref.get(key), other.get(key)
            if a is not None and b is not None and float(a) != float(b):
                ok = False
                reasons.append(f"{label} {a:g} vs {b:g} ({pair})")
        for key, label in (("board", "board"), ("mcu", "MCU id")):
            a, b = ref.get(key), other.get(key)
            if a and b and a != b:
                ok = False
                reasons.append(f"{label} {a} vs {b} ({pair})")
        va, vb = ref.get("version"), other.get("version")
        if va and vb and va != vb:
            reasons.append(f"firmware {va[0]}.{va[1]} vs {vb[0]}.{vb[1]} ({pair}); continuing")
    return ok, reasons


def _gains_equal(a, b):
    rel = float(_fc("fuse_gain_equal_rel"))
    for f in _GAIN_FIELDS_EQUAL:
        x, y = float(getattr(a, f) or 0.0), float(getattr(b, f) or 0.0)
        if abs(x - y) > rel * max(abs(x), abs(y), 1e-12):
            return False
    return True


def _pool_steps(responses, aggr):
    """One `StepResponse` from several of the same gain set: the frames stacked, the
    mean over all of them, metrics and consistency recomputed as `step_response` does."""
    if len(responses) == 1:
        return responses[0]
    frames = np.vstack([r.frames for r in responses])
    t = responses[0].t
    mean = frames.mean(axis=0)
    peaks = frames.max(axis=1)
    med = float(np.median(peaks))
    iqr = float(np.percentile(peaks, 75) - np.percentile(peaks, 25))
    consistency = float(np.clip(1.0 - iqr / med, 0.0, 1.0)) if med > 0 else 0.0
    metrics = tune_step.step_metrics(t, mean, aggr)
    metrics.update(n_frames_total=int(sum(r.metrics.get("n_frames_total", r.n_frames) for r in responses)),
                   n_frames_high=int(sum(r.metrics.get("n_frames_high", 0) for r in responses)),
                   n_dropped_nonfinite=int(sum(r.metrics.get("n_dropped_nonfinite", 0) for r in responses)),
                   fs=responses[0].metrics.get("fs"), pooled_segments=len(responses))
    return StepResponse(t=t, mean=mean, frames=frames, n_frames=int(frames.shape[0]),
                        n_dropped_low=int(sum(r.n_dropped_low for r in responses)), metrics=metrics, consistency=consistency)


def _record_error(entry, tier, exc):
    entry.setdefault("errors", []).append(dict(tier=tier, error=f"{type(exc).__name__}: {exc}",
                                               traceback=traceback.format_exc()))


def _dedupe_refusals(refusals):
    seen, out = set(), []
    for r in refusals:
        key = (r.code, r.log_name, r.axis)
        if key in seen:
            continue
        seen.add(key)
        out.append(r)
    return out


def analyse(logs, windows, axes=AXES, prop_in=None, aggr=None, log_names=None):
    """The pipeline of plan section 2.3 over one or more logs of one aircraft.

    logs, windows   parallel lists (`Log`, `flight.Window`)
    axes            which axes to analyse
    prop_in         prop diameter in inches for the Mission Planner comparison
    aggr            override the logs' AUTOTUNE_AGGR (noted as "override")
    log_names       display names (default: the file basenames)

    Returns a `TuneAnalysis`. `per_axis[axis]` holds: `signals` (AxisSignals in time
    order), `current` (GainSet), `adequacy` (`pooled`, `current`), `step` (one dict per
    segment: log_name, segment, gains, fs, limited_pct, is_current, response |
    refusal, osc, ceilings), `step_current` (the pooled StepResponse of the current gain
    set or None), `osc_current`, `rules`, `plant` (PlantModel | None), `plant_refusal`,
    `margins` (scalars only), `virtual`, `ceilings`, `autotune` (sessions), `pooled`,
    `tiers` (`A`, `B`, `C` booleans), `notes`, `errors`. Nothing here raises for a
    tier's failure: it lands in `errors` and renders FAIL. `extra` carries `prop_in`,
    `aggr`, `autotune_axes`, `calculator` (headers, rows) and `params_by_log`.
    """
    logs, windows = list(logs), list(windows)
    if len(logs) != len(windows):
        raise ValueError("analyse() needs one window per log")
    if not logs:
        raise ValueError("analyse() needs at least one log")
    names = [_log_name(lg, (log_names or [None] * len(logs))[i]) for i, lg in enumerate(logs)]
    axes = tuple(axes)
    constants = tune.all_constants()

    ids = [_identity(lg, w, n) for lg, w, n in zip(logs, windows, names)]
    ok, reasons = _identity_gate(ids)
    extra = dict(prop_in=prop_in, aggr=aggr, autotune_axes=None, calculator=None, params_by_log={})
    if not ok:
        ref = Refusal("DIFFERENT_AIRCRAFT", "the logs are not from one aircraft: " + "; ".join(reasons),
                      "pass logs of one aircraft (same FRAME_CLASS/FRAME_TYPE, board and MCU id)")
        for d in ids:
            d["refusals"] = ["DIFFERENT_AIRCRAFT"]
        return TuneAnalysis(logs=ids, identity_ok=False, identity_reasons=reasons, per_axis={}, recommendations=[],
                            refusals=[ref.to_dict()], params=[], constants=constants, extra=extra)

    # chronological order: boot time when known, else the order given
    order = sorted(range(len(logs)), key=lambda i: (ids[i]["boot_time_unix"] if ids[i]["boot_time_unix"] is not None else -_INF, i))
    rank = {i: k for k, i in enumerate(order)}
    latest = order[-1]
    extra["autotune_axes"] = ids[latest]["autotune_axes"]

    # ---- extraction and tier A per log
    all_sigs, refusals, sessions_by_log = [], [], {}
    for i, (lg, w, n) in enumerate(zip(logs, windows, names)):
        try:
            sigs, refs = extract_axes(lg, w, axes, log_name=n)
        except Exception as exc:
            sigs, refs = [], []
            ids[i]["errors"].append(dict(tier="extract", error=f"{type(exc).__name__}: {exc}", traceback=traceback.format_exc()))
        if aggr is not None:
            sigs = [dataclasses.replace(s, gains=dataclasses.replace(s.gains, aggr=float(aggr),
                                                                    param_names=dict(s.gains.param_names, aggr="override")))
                    for s in sigs]
        for s in sigs:
            s._rank = rank[i]                                    # noqa: SLF001 - ordering key only
        all_sigs.extend(sigs)
        refusals.extend(refs)
        try:
            sessions_by_log[n] = tune_atun.autotune_sessions(lg, aggr_override=aggr, log_name=n)
        except Exception as exc:
            sessions_by_log[n] = []
            ids[i]["errors"].append(dict(tier="A", error=f"{type(exc).__name__}: {exc}", traceback=traceback.format_exc()))
        for s in sessions_by_log[n]:
            refusals.extend(s.get("refusals") or [])
        extra["params_by_log"][n] = dict(MOT_THST_EXPO=lg.param("MOT_THST_EXPO"))
    pooled_all = tune_atun.pool_sessions(sessions_by_log)

    per_axis, params, gains_by_axis = {}, [], {}
    for axis in axes:
        sigs = sorted([s for s in all_sigs if s.axis == axis], key=lambda s: (s._rank, s.segment[0]))
        entry = dict(signals=sigs, adequacy={}, step=[], step_current=None, osc_current=None, rules=[], plant=None,
                     plant_refusal=None, margins=None, virtual=None, ceilings=[], autotune=[], pooled={},
                     tiers=dict(A=False, B=False, C=False), notes=[], errors=[])
        per_axis[axis] = entry
        if sigs:
            current = sigs[-1].gains
        else:
            try:
                current = GainSet.from_log(logs[latest], windows[latest].t0, axis, aggr_override=aggr)
            except Exception as exc:
                _record_error(entry, "D", exc)
                current = None
        entry["current"] = current
        if current is None:
            continue
        gains_by_axis[axis] = current
        ids[latest]["contributed"].append("D")

        # ---- tier C per segment; rules from the current gain set only
        cur_resps, cur_osc, cur_ceilings = [], None, []
        for s in sigs:
            rec = dict(log_name=s.log_name, segment=list(s.segment), gains=s.gains, fs=s.fs, limited_pct=s.limited_pct,
                       is_current=_gains_equal(s.gains, current), response=None, refusal=None, osc=None, ceilings=[])
            try:
                st = tune_step.step_response(s)
                if isinstance(st, Refusal):
                    rec["refusal"] = st
                    refusals.append(st)
                else:
                    rec["response"] = st
                    entry["tiers"]["C"] = True
                    if s.log_name in names:
                        ids[names.index(s.log_name)]["contributed"].append("C")
                osc = tune_step.oscillation(s)
                rec["osc"] = osc
                rec["ceilings"] = list(osc.get("ceilings") or [])
                if osc.get("at_ceiling"):
                    entry["tiers"]["C"] = True                  # a measured limit cycle is evidence
                    if s.log_name in names:
                        ids[names.index(s.log_name)]["contributed"].append("C")
                if rec["is_current"]:
                    if isinstance(st, StepResponse):
                        cur_resps.append(st)
                    cur_osc = osc
                    cur_ceilings.extend(rec["ceilings"])
            except Exception as exc:
                _record_error(entry, "C", exc)
            entry["step"].append(rec)
        entry["ceilings"].extend(cur_ceilings)
        if cur_resps or cur_osc:
            try:
                st = _pool_steps(cur_resps, current.aggr) if cur_resps else None
                entry["step_current"], entry["osc_current"] = st, cur_osc
                metrics = st.metrics if st is not None else {}
                if st is not None or (cur_osc and cur_osc.get("at_ceiling")):
                    entry["rules"] = tune_step.step_rules(metrics, cur_osc, current, aggr=aggr)
            except Exception as exc:
                _record_error(entry, "C", exc)
        cur_sigs = [s for s in sigs if _gains_equal(s.gains, current)]
        for key, group in (("pooled", sigs), ("current", cur_sigs)):
            if group:
                wts = np.array([max(s.duration, 1e-9) for s in group])
                entry["adequacy"][key] = float(np.average([adequacy(s.fs, s.limited_pct) for s in group], weights=wts))

        # ---- tier B pooled over every segment of the axis
        if sigs:
            try:
                pm = tune_ident.identify(sigs)
                if isinstance(pm, Refusal):
                    entry["plant_refusal"] = pm
                    refusals.append(pm)
                else:
                    entry["plant"] = pm
                    entry["tiers"]["B"] = True
                    for s in sigs:
                        if s.log_name in names:
                            ids[names.index(s.log_name)]["contributed"].append("B")
                    m = tune_ident.margins(pm, current)
                    entry["margins"] = {k: v for k, v in m.items() if k not in ("L", "np_L", "freqs_hz", "np_freqs_hz")}
                    bo = tune_atun.backoff_for(tune_atun.firmware_version(ids[latest]["firmware"]), current.gmbk, current.aggr)
                    gmbk_eff = 1.0 - float(bo["rate_p"])
                    va = tune_ident.virtual_autotune(pm, current, aggr=aggr, gmbk=gmbk_eff, axis=axis)
                    va["backoff"] = dict((k, bo.get(k)) for k in ("rate_p", "rate_d", "sp", "gmbk", "branch", "exact", "note"))
                    if not bo["exact"]:
                        entry["notes"].append(f"virtual AutoTune backoff assumed: {bo['note']}")
                    entry["virtual"] = va
                    # margin ceilings: P at the D it would be applied with (the virtual AutoTune's)
                    done = bool(va.get("complete"))
                    d_for_p = va.get("rat_d") if done and _fin(va.get("rat_d")) else None
                    p_for_d = va.get("rat_p") if done and _fin(va.get("rat_p")) else None
                    mcs, mnotes = tune_ident.margin_ceilings(pm, current, d_for_p=d_for_p, p_for_d=p_for_d)
                    entry["ceilings"].extend(mcs)
                    entry["notes"].extend(mnotes)
                    heli = tune_ident.ceilings_from_plant(pm, current)
                    if heli:
                        entry["notes"].append(
                            "heli AutoTune ceilings (evidence only, not applied: P-only rules that ignore D's phase lead): "
                            + ", ".join(f"{c.param} {c.value:.4g} ({c.method})" for c in heli))
            except Exception as exc:
                _record_error(entry, "B", exc)

        # ---- tier A sessions of this axis (yaw_d folds into yaw's D)
        sess = []
        for n in names:
            for s in sessions_by_log.get(n, []):
                if s["axis"] == axis or (axis == "yaw" and s["axis"] == "yaw_d"):
                    sess.append(s)
                    if s.get("complete"):
                        entry["tiers"]["A"] = True
                        ids[names.index(n)]["contributed"].append("A")
        entry["autotune"] = sess
        pooled = dict(pooled_all.get(axis, {}))
        if axis == "yaw" and "yaw_d" in pooled_all and "rat_d" in pooled_all["yaw_d"]:
            pooled["rat_d"] = pooled_all["yaw_d"]["rat_d"]
        entry["pooled"] = pooled

        # ---- tier D
        try:
            params.extend(param_consistency(current, prop_in, aircraft_wide=(axis == axes[0]),
                                            params=extra["params_by_log"].get(names[latest])))
        except Exception as exc:
            _record_error(entry, "D", exc)

    if prop_in is not None and gains_by_axis:
        try:
            extra["calculator"] = calculator_rows(gains_by_axis, prop_in, extra["params_by_log"].get(names[latest]))
        except Exception as exc:
            per_axis[axes[0]].setdefault("errors", []).append(dict(tier="D", error=f"{type(exc).__name__}: {exc}",
                                                                   traceback=traceback.format_exc()))

    # ---- fusion
    try:
        recs = fuse(per_axis, {a: e["current"] for a, e in per_axis.items() if e.get("current") is not None},
                    autotune_axes=extra["autotune_axes"], prop_in=prop_in)
    except Exception as exc:
        recs = []
        per_axis[axes[0]].setdefault("errors", []).append(dict(tier="fusion", error=f"{type(exc).__name__}: {exc}",
                                                               traceback=traceback.format_exc()))

    refusals = _dedupe_refusals(refusals)
    for d in ids:
        d["contributed"] = sorted(set(d["contributed"]))
        d["refusals"] = sorted({r.code for r in refusals if r.log_name == d["file_name"]})
    if reasons:
        # firmware differs: a WARN, not a refusal - carried as an identity reason
        pass
    return TuneAnalysis(logs=ids, identity_ok=True, identity_reasons=reasons, per_axis=per_axis, recommendations=recs,
                        refusals=[r.to_dict() for r in refusals], params=params, constants=constants, extra=extra)


def is_whole_tool_refusal(analysis):
    """True when no axis has any tier A/B/C evidence (plan section 2.6): a deconvolved
    step, an identified plant or a complete AutoTune session. Tier D alone is not
    evidence for a recommendation."""
    if not analysis.identity_ok:
        return True
    for e in analysis.per_axis.values():
        if e.get("tiers", {}).get("A") or e.get("tiers", {}).get("B") or e.get("tiers", {}).get("C"):
            return False
    return True


# ------------------------------------------------------------------------- section

def refusal_block(analysis, with_exit_line=False):
    """The plan section 2.6 block, exact shape: the ERROR line, one line per (log, code),
    the parameter header, the evaluated logging requirements of every log (prefixed by
    the file name when there is more than one log), optionally `exit code 3`."""
    lines = ["ERROR: these log files cannot be used for PID tuning."]
    seen, items = set(), []
    for r in analysis.refusals:
        key = (r["code"], r.get("log_name"))
        if key in seen:
            for it in items:
                if (it["code"], it["log_name"]) == key and r.get("axis") and r["axis"] not in it["axes"]:
                    it["axes"].append(r["axis"])
            continue
        seen.add(key)
        items.append(dict(code=r["code"], log_name=r.get("log_name"), message=r["message"],
                          axes=[r["axis"]] if r.get("axis") else []))
    width = max((len(it["code"]) for it in items), default=0)
    for it in items:
        who = it["log_name"] or "all logs"
        axes = f" [{', '.join(it['axes'])}]" if it["axes"] else ""
        lines.append(f"  {who}: {it['code']:<{width}} - {it['message']}{axes}")
    lines.append(LOGGING_FIX_HEADER)
    for d in analysis.logs:
        if len(analysis.logs) > 1:
            lines.append(f"  -- {d['file_name']}")
        lines.append(format_logging_fix(d.get("requirements") or []))
    if with_exit_line:
        lines.append("exit code 3")
    return "\n".join(lines)


VALIDATION_NOTE = (
    "Validation flights (docs/pid-tuning-plan.md section 7). Data-acquisition / tuning profile (tiers B and C): "
    "LOG_BITMASK bit 0 on (e.g. 180222 -> 180223), INS_LOG_BAT_MASK 0, take off in ALT_HOLD, 30 s hover, then 60 s of "
    "sharp roll stick inputs (+-15-20 deg, quick release), 60 s pitch, 30 s yaw, land; set bit 0 back afterwards on "
    "onboard-flash boards. Plant-identification profile (raises tier B confidence): SysID mode, SID_AXIS 10 (then 11, "
    "12), SID_MAGNITUDE 0.15 (yaw 0.55), SID_F_START_HZ 0.5, SID_F_STOP_HZ 40, SID_T_REC 70, SID_T_FADE_IN 15, "
    "SID_T_FADE_OUT 2 - the sweep must pass the rate-loop crossover (4.5-5 Hz measured on a 10-inch quad, higher on smaller props); "
    "the 0.05-5 Hz AnalyticTune sweep is for the attitude loop. Post-change verification: apply the recommended set "
    "for one axis, fly the tuning profile again, `alog tune before.bin after.bin`; expect overshoot_ratio and "
    "bounce_ratio to move toward 1.0 and the margins to stay above 6 dB / 45 deg.")


def _ms(x):
    return f"{float(x) * 1e3:.0f}" if _fin(x) else "-"


def _n(x, nd=3):
    return f"{float(x):.{nd}f}" if _fin(x) else "-"


def to_section(analysis, whole_tool_refusal=None):
    """The `tune` Section (key `tune`, title "PID gains from the log").

    Whole-tool refusal (no axis with tier A/B/C evidence): the plan section 2.6 block as
    the first note, one SKIP Result per refusal code, then the tier-D results. Otherwise
    the tables `recommendations`, `ceilings`, `step`, `plant`, `margins`, `autotune`,
    `params` (+ `calculator` with `--prop-in`), `refusals`; Results per recommendation
    graded by `tune_confidence` (withheld ones WARN), `<axis> insufficient data` WARN,
    the tier-D results, margins graded by `tune_gain_margin_db` / `tune_phase_margin_deg`,
    a FAIL per AutoTune `mismatch`, a FAIL per tier that crashed; notes from each tier's
    `describe`, the defaults assumed and the validation flights.
    """
    from .analysis import Section, _grade
    if whole_tool_refusal is None:
        whole_tool_refusal = is_whole_tool_refusal(analysis)
    sec = Section("PID gains from the log", key="tune")
    fail_c = float(T["tune_confidence"]["fail"])

    if analysis.identity_reasons:
        sec.add(Result("aircraft identity", WARN if analysis.identity_ok else FAIL,
                       ("logs differ: " if analysis.identity_ok else "not one aircraft: ") + "; ".join(analysis.identity_reasons),
                       evidence=dict(reasons=list(analysis.identity_reasons), logs=[d["file_name"] for d in analysis.logs]),
                       source=_SRC_PLAN23))

    defaulted = set()
    for e in analysis.per_axis.values():
        if e.get("current") is not None:
            defaulted.update(e["current"].defaulted)
        for s in e.get("autotune") or []:
            defaulted.update(s.get("defaulted") or [])

    if whole_tool_refusal:
        sec.note(refusal_block(analysis))
        seen = set()
        for r in analysis.refusals:
            if r["code"] in seen:
                continue
            seen.add(r["code"])
            where = sorted({x.get("log_name") or "" for x in analysis.refusals if x["code"] == r["code"]})
            axes = sorted({x.get("axis") for x in analysis.refusals if x["code"] == r["code"] and x.get("axis")}, key=AXES.index)
            sec.add(Result("refused", SKIP, f"{r['code']}: {r['message']}. Fix: {r['fix'].splitlines()[0]}",
                           evidence=dict(code=r["code"], fix=r["fix"], logs=where, axes=axes,
                                         requirements=[d.get("requirements") for d in analysis.logs]),
                           source=_SRC_PLAN23))
        for r in analysis.params:
            sec.add(r)
        _crash_results(sec, analysis)
        if analysis.logs:
            sec.table("logs", ["log", "firmware", "board", "frame", "window", "contributed", "refusals"],
                      _log_rows(analysis))
        if defaulted:
            sec.note("Parameters NOT in the log, defaults assumed: " + ", ".join(sorted(defaulted))
                     + ". Every tier-D number above that depends on one of these is conditional on it.")
        sec.note(VALIDATION_NOTE)
        return sec

    # ---- recommendations
    rec_rows = []
    for r in analysis.recommendations:
        withheld = not _fin(r.value)
        g = _grade(r.confidence, "tune_confidence", higher_is_worse=False, name=f"{r.axis} {r.param}")
        status = WARN if (g.status == FAIL or withheld) else g.status
        if r.method == "unchanged":
            shown = f"{r.param} left at {_fmt(r.current)}{(' ' + r.evidence['unit']) if r.evidence.get('unit') else ''}"
        elif withheld:
            shown = (f"{r.param} {_fmt(r.current)} -> withheld (value {_fmt(r.evidence.get('withheld_value'))}, "
                     f"{r.evidence.get('withheld_change_pct'):+.1f} % in evidence)" if _fin(r.evidence.get("withheld_change_pct"))
                     else f"{r.param} {_fmt(r.current)} -> withheld")
        else:
            shown = f"{r.param} {_fmt(r.current)} -> {_fmt(r.value)} ({r.change_pct:+.1f} %)" if _fin(r.change_pct) \
                else f"{r.param} {_fmt(r.current)} -> {_fmt(r.value)}"
        comps = r.components
        summary = (f"{shown} via {r.method}, confidence {r.confidence:.2f} (warn {T['tune_confidence']['warn']}, fail "
                   f"{T['tune_confidence']['fail']}; prior {comps.get('prior', 0):.2f} x adequacy {comps.get('adequacy', 0):.2f} x "
                   f"excitation {comps.get('excitation', 0):.2f} x consistency {comps.get('consistency', 0):.2f} x agreement "
                   f"{comps.get('agreement', 0):.2f}" + (f", capped at {comps['cap']}" if comps.get("cap") else "") + ")"
                   + ("; validate before applying" if status == WARN and not withheld else "")
                   + (f"; {r.note}" if r.note else ""))
        sec.add(Result(f"{r.axis} {r.param}", status, summary,
                       evidence=_safe(dict(g.evidence, current=r.current, recommended=r.value, change_pct=r.change_pct,
                                           method=r.method, **r.evidence)), source=g.source))
        rec_rows.append([r.axis, r.param, _fmt(r.current), "withheld" if withheld else _fmt(r.value),
                         f"{r.change_pct:+.1f}" if _fin(r.change_pct) else "-", f"{r.confidence:.2f}", r.method, r.note])
    if rec_rows:
        sec.table("recommendations", ["axis", "param", "current", "recommended", "change %", "confidence", "method", "why"],
                  rec_rows, align=["l", "l", "r", "r", "r", "r", "l", "l"])

    for axis, e in analysis.per_axis.items():
        if not any(e.get("tiers", {}).values()) and not any(r.axis == axis for r in analysis.recommendations):
            codes = sorted({r["code"] for r in analysis.refusals if r.get("axis") == axis})
            sec.add(Result(f"{axis} insufficient data", WARN,
                           f"no tier A/B/C evidence for {axis}" + (f": {', '.join(codes)}" if codes else "")
                           + "; see the refusals table",
                           evidence=dict(refusals=codes), source=_SRC_PLAN23))

    for r in analysis.params:
        sec.add(r)

    # ---- margins, mismatches, crashes
    for axis, e in analysis.per_axis.items():
        m = e.get("margins")
        if m:
            ex = tune_ident.extrapolated(e["plant"], m) if isinstance(e.get("plant"), PlantModel) else []
            for key, tkey, label in (("gm_db", "tune_gain_margin_db", "gain margin"), ("pm_deg", "tune_phase_margin_deg", "phase margin")):
                v = m.get(key)
                why_x = [x for x in ex if x.startswith("gain margin" if key == "gm_db" else "crossover")]
                if why_x:
                    sec.add(Result(f"{axis} {label}", SKIP,
                                   f"{_fmt(v, 3)} {'dB' if key == 'gm_db' else 'deg'} on the fitted open loop, not measured: "
                                   + "; ".join(why_x), evidence=_safe(dict(value=v, extrapolated=why_x)),
                                   source=T[tkey]["source"]))
                    continue
                if v is None:
                    sec.add(Result(f"{axis} {label}", SKIP, f"the open loop has no {'-180 deg' if key == 'gm_db' else 'unity-gain'} "
                                                             "crossing on the fit grid", evidence=_safe(dict(margins=m)),
                                   source=T[tkey]["source"]))
                    continue
                g = _grade(v, tkey, higher_is_worse=False, name=f"{axis} {label}",
                           fc_hz=m.get("fc_hz"), f180_hz=m.get("f180_hz"), np_value=m.get("np_" + key), np_in_band=m.get("np_in_band"))
                unit = "dB" if key == "gm_db" else "deg"
                g.summary = (f"{v:.2f} {unit} on the fitted open loop (warn {T[tkey]['warn']:g}, fail {T[tkey]['fail']:g})"
                             + (f"; measured in band {m.get('np_' + key):.2f} {unit}" if _fin(m.get("np_" + key)) else ""))
                sec.add(g)
        for s in e.get("autotune") or []:
            if s.get("mismatch"):
                sec.add(Result(f"{axis} AutoTune reconstruction vs MSG", FAIL, s["mismatch_note"],
                               evidence=_safe(dict(agreement=s["agreement"], backoff=s["backoff"], log_name=s["log_name"], session=s["session"])),
                               source=CONSTANTS["autotune_msg_agree_pct"]["source"]))
    _crash_results(sec, analysis)

    # ---- tables
    ceil_rows = []
    for axis, e in analysis.per_axis.items():
        for c in e.get("ceilings") or []:
            ev = c.evidence or {}
            held = ev.get("held") or {}
            detail = (f"{ev.get('f_osc'):.1f} Hz limit cycle" if _fin(ev.get("f_osc")) else
                      f"SRate p95 {ev.get('srate_p95'):.2f}" if _fin(ev.get("srate_p95")) else
                      (f"PM {_fmt(ev.get('pm_deg'), 3)} deg, GM {_fmt(ev.get('gm_db'), 3)} dB at the ceiling"
                       + (f" (D held at {_fmt(held['rat_d'])})" if "rat_d" in held else "")) if c.includes_margin else
                      f"arg G = {ev.get('phase_deg'):.0f} deg at {ev.get('f_hz'):.1f} Hz" if _fin(ev.get("f_hz")) else "")
            clip = c.value * (1.0 if c.includes_margin else 1.0 - float(CONSTANTS["quik_gain_margin"]["value"]))
            ceil_rows.append([axis, c.param, _fmt(c.value), c.method, _fmt(clip), detail])
    if ceil_rows:
        sec.table("ceilings", ["axis", "param", "ceiling", "method", "clips to", "evidence"], ceil_rows,
                  align=["l", "l", "r", "l", "r", "l"])

    th_os, th_b = T["tune_overshoot_ratio"], T["tune_bounce_ratio"]
    step_rows = []
    for axis, e in analysis.per_axis.items():
        for rec in e.get("step") or []:
            g = rec["gains"]
            st, osc = rec.get("response"), rec.get("osc") or {}
            m = st.metrics if isinstance(st, StepResponse) else {}
            step_rows.append([axis, rec["log_name"], f"{rec['segment'][0]:.1f}-{rec['segment'][1]:.1f}",
                              f"P {_fmt(g.rat_p)} I {_fmt(g.rat_i)} D {_fmt(g.rat_d)}" + (" (current)" if rec["is_current"] else ""),
                              st.n_frames if isinstance(st, StepResponse) else (rec["refusal"].code if rec.get("refusal") else "-"),
                              _ms(m.get("latency_s")), _ms(m.get("rise_s")), _n(m.get("peak")),
                              f"{_n(m.get('overshoot_ratio'), 2)} (warn {th_os['warn']:g}, fail {th_os['fail']:g})",
                              f"{_n(m.get('bounce_ratio'), 2)} (warn {th_b['warn']:g}, fail {th_b['fail']:g})",
                              _ms(m.get("settle_s")), _n(m.get("ss")), _n(getattr(st, "consistency", None), 2),
                              _n(osc.get("srate_p95"), 2), "yes" if osc.get("at_ceiling") else "no"])
    if step_rows:
        sec.table("step", ["axis", "log", "segment s", "gains", "frames", "latency ms", "rise ms", "peak", "overshoot ratio",
                           "bounce ratio", "settle ms", "steady state", "consistency", "SRate p95", "ceiling"], step_rows,
                  align=["l", "l", "l", "l", "r", "r", "r", "r", "l", "l", "r", "r", "r", "r", "l"])

    plant_rows, margin_rows = [], []
    for axis, e in analysis.per_axis.items():
        pm = e.get("plant")
        if isinstance(pm, PlantModel):
            h, rows = tune_ident.plant_rows(pm)
            plant_rows.extend([[axis] + r for r in rows])
            plant_headers = ["axis"] + h
        if e.get("margins"):
            h2, rows2 = tune_ident.margin_rows(e["margins"])
            margin_rows.extend([[axis] + r for r in rows2])
            margin_headers = ["axis"] + h2
    if plant_rows:
        sec.table("plant", plant_headers, plant_rows, align=["l", "l", "r", "l", "l"])
    if margin_rows:
        sec.table("margins", margin_headers, margin_rows, align=["l", "l", "r", "r", "r", "r"])

    sessions = [s for e in analysis.per_axis.values() for s in (e.get("autotune") or [])]
    if sessions:
        h, rows = tune_atun.to_rows(sessions)
        sec.table("autotune", h, rows)

    gains_by_axis = {a: e["current"] for a, e in analysis.per_axis.items() if e.get("current") is not None}
    if gains_by_axis:
        h, rows = gain_rows(gains_by_axis)
        sec.table("params", h, rows, align=["l"] + ["r"] * (len(h) - 2) + ["l"])
    calc = (analysis.extra or {}).get("calculator") if hasattr(analysis, "extra") else None
    if calc:
        sec.table("calculator", calc[0], calc[1], align=["l", "r", "r", "r", "l", "l"])

    if analysis.refusals:
        sec.table("refusals", ["log", "axis", "code", "message", "fix"],
                  [[r.get("log_name") or "-", r.get("axis") or "-", r["code"], r["message"], r["fix"].splitlines()[0]]
                   for r in analysis.refusals], align=["l", "l", "l", "l", "l"])
    if analysis.logs:
        sec.table("logs", ["log", "firmware", "board", "frame", "window", "contributed", "refusals"], _log_rows(analysis))

    # ---- notes
    for axis, e in analysis.per_axis.items():
        st, osc = e.get("step_current"), e.get("osc_current")
        if st is not None or osc:
            for line in tune_step.describe(st, st.metrics if st is not None else None, osc):
                sec.note(f"{axis}: {line}")
            if e.get("rules") == [] and st is not None and not (osc or {}).get("at_ceiling"):
                sec.note(f"{axis}: step rules: no change indicated (metrics within band; uncalibrated, see plan section 2.5)")
        pm = e.get("plant")
        if isinstance(pm, PlantModel):
            for line in tune_ident.describe(pm, e.get("margins")):
                sec.note(f"{axis}: {line}")
            va = e.get("virtual")
            if va:
                sec.note(f"{axis}: virtual AutoTune -> P {_fmt(va.get('rat_p'))} I {_fmt(va.get('rat_i'))} D {_fmt(va.get('rat_d'))} "
                         f"ANG_P {_fmt(va.get('ang_p'))} ACC {_fmt(va.get('acc_max_dps2'))} deg/s^2"
                         + (f" FLTE {_fmt(va.get('flte'))} Hz" if axis == "yaw" else "") + f"; {va.get('why', '')}"
                         + (f"; backoff {va['backoff']['branch']} rate x{va['backoff']['rate_p']:.3f}" if va.get("backoff") else ""))
        elif e.get("plant_refusal") is not None:
            sec.note(f"{axis}: plant identification: {e['plant_refusal'].code} - {e['plant_refusal'].message}")
        for s in e.get("autotune") or []:
            for line in tune_atun.describe(s):
                sec.note(f"{axis}: {line}")
        for n in e.get("notes") or []:
            sec.note(f"{axis}: {n}")
    if defaulted:
        sec.note("Parameters NOT in the log, defaults assumed: " + ", ".join(sorted(defaulted))
                 + ". Every number above that depends on one of these is conditional on it.")
    sec.note("Confidence = prior x adequacy x excitation x consistency x agreement (docs/pid-tuning-plan.md section 2.5); "
             f">= {T['tune_confidence']['warn']} recommend, {T['tune_confidence']['fail']}-{T['tune_confidence']['warn']} "
             f"indicative (validate before applying), < {T['tune_confidence']['fail']} withheld. Tier C (step rules) is capped "
             f"at {_fc('fuse_tier_c_confidence_cap')} until its overshoot/bounce criteria are calibrated on a real fast-logged flight.")
    sec.note(VALIDATION_NOTE)
    return sec


def _crash_results(sec, analysis):
    for d in analysis.logs:
        for err in d.get("errors") or []:
            sec.add(Result(f"{d['file_name']} tier {err['tier']}", FAIL, f"tier {err['tier']} crashed: {err['error']}",
                           evidence=dict(traceback=err["traceback"]), source="dflog"))
    for axis, e in analysis.per_axis.items():
        for err in e.get("errors") or []:
            sec.add(Result(f"{axis} tier {err['tier']}", FAIL, f"tier {err['tier']} crashed: {err['error']}",
                           evidence=dict(traceback=err["traceback"]), source="dflog"))


def _log_rows(analysis):
    rows = []
    for d in analysis.logs:
        w = d.get("window") or {}
        rows.append([d["file_name"], d.get("firmware") or "-", d.get("board") or "-",
                     f"{_fmt(d.get('frame_class'))}/{_fmt(d.get('frame_type'))}",
                     f"{w.get('t0', 0):.1f}-{w.get('t1', 0):.1f} s ({w.get('method', '')})",
                     ", ".join(d.get("contributed") or []) or "-", ", ".join(d.get("refusals") or []) or "-"])
    return rows


def analysis_to_dict(analysis):
    """A JSON-safe dict of the `TuneAnalysis` scalars: logs, identity, recommendations,
    refusals, tier-D results, per-axis summaries (no arrays), constants."""
    per = {}
    for axis, e in analysis.per_axis.items():
        pm = e.get("plant")
        per[axis] = dict(
            current=_safe(e.get("current")), tiers=e.get("tiers"), adequacy=_safe(e.get("adequacy")),
            signals=[s.summary() for s in e.get("signals") or []],
            step_current=_safe(dict(n_frames=e["step_current"].n_frames, n_dropped_low=e["step_current"].n_dropped_low,
                                    consistency=e["step_current"].consistency, metrics=e["step_current"].metrics))
            if e.get("step_current") is not None else None,
            osc_current=_safe({k: v for k, v in (e.get("osc_current") or {}).items() if k != "peaks"}) if e.get("osc_current") else None,
            rules=_safe(e.get("rules")),
            plant=_safe(dict(k=pm.k, tau1=pm.tau1, tau2=pm.tau2, delay=pm.delay, band=list(pm.band), fit_rms_db=pm.fit_rms_db,
                             fit_rms_deg=pm.fit_rms_deg, coh_mean_band=pm.coh_mean_band, eps_mag_at_crossover=pm.eps_mag_at_crossover,
                             n_avg=pm.n_avg)) if isinstance(pm, PlantModel) else None,
            plant_refusal=_safe(e.get("plant_refusal")), margins=_safe(e.get("margins")),
            virtual=_safe({k: v for k, v in (e.get("virtual") or {}).items() if k not in ("twitches",)}) if e.get("virtual") else None,
            ceilings=_safe(e.get("ceilings")), pooled=_safe(e.get("pooled")),
            autotune=[_safe(tune_atun.session_to_dict(s)) for s in e.get("autotune") or []],
            notes=list(e.get("notes") or []), errors=_safe(e.get("errors")))
    return _safe(dict(logs=analysis.logs, identity=dict(ok=analysis.identity_ok, reasons=analysis.identity_reasons),
                      recommendations=[dataclasses.asdict(r) for r in analysis.recommendations],
                      refusals=analysis.refusals, params=[r.to_dict() for r in analysis.params], per_axis=per,
                      constants=analysis.constants, extra=getattr(analysis, "extra", {})))
