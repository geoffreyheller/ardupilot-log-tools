"""AutoTune session reconstruction from `ATUN` / `ATDE` / `MSG` / `EV` / `PARM` (tier A).

Work package 4 of `docs/pid-tuning-plan.md`. A log that holds an AutoTune session carries
the gains the firmware found, twitch by twitch, in `ATUN`; the text it announced in
`MSG`; and, after `EV 37`, the values it saved in `PARM`. This module re-derives the
outcome of every twitch with the firmware's own rules (sources file section 1.5), applies
the backoff the *flown firmware branch* used (section 1.6, verified per branch on
2026-09-17 - `BACKOFF_BY_FIRMWARE`), and cross-checks the reconstruction against the
`MSG` lines and the saved parameters. A disagreement is reported, with a statement of
which side is the more likely wrong; nothing is repaired.

What the log says, and the three facts the reconstruction rests on (all read from
`AC_AutoTune.cpp` / `AC_AutoTune_Multi.cpp`, master and the release branches):

1. `ATUN` is written in `UPDATE_GAINS` *before* the rule runs (`Log_AutoTune()` is the
   first statement of the case), so `RP/RD/SP` are the gains **tested** in that twitch
   and the rule's change shows up in the *next* row. `RD` is FLTE (Hz) on the yaw(E)
   axis (`Axis = 2`). `Targ/Min/Max` are x0.01 in the log (deg, deg/s); `ddt` is
   `test_accel_max_cdss` **unscaled** (cdeg/s^2) despite the unit tag.
2. The backoff runs when a step **completes** (`set_tuning_gains_with_backoff` /
   `set_gains_post_tune`, called right after `success_counter >= 4`). The RATE_P_UP
   backoff is therefore already in the ANGLE_P_DOWN / ANGLE_P_UP rows: the saved rate
   gains are the last RATE_P_UP row x the rate backoff **and equal to** the ANGLE rows'
   `RP/RD` x 1. Only `SP` takes its backoff from the last ANGLE_P_UP row. That ratio -
   ANGLE rows' RP / last RATE_P_UP RP - is an in-log **witness** of the rate backoff the
   firmware actually applied, independent of any version table; it is reported as
   `backoff.rate_p_observed`.
3. `test_accel_max_cdss` is overwritten per test (`accel_measure_rate_max = 0` in
   `test_init`), so `ATC_ACC_*_MAX = max(floor, ddt of the last angle twitch)`, not the
   maximum over the axis.

Session boundaries: every `EV 30` (AUTOTUNE_INITIALISED) starts a session; a gap longer
than `SESSION_GAP_S` between consecutive `ATUN` rows, or a `TuneStep` that goes backwards
on an axis, also starts one. `EV 31` (OFF) alone does not - leaving and re-entering the
mode resumes the same tuning state (`EV 32` RESTART).

Rules this module follows (RULES.md 1-3): nothing numeric is hard-coded in a function
body - constants come from `tunesim.AUTOTUNE`, `tune.CONSTANTS`, `BACKOFF_BY_FIRMWARE`
and `ATUN_CONSTANTS`, each with its source; every fallback is named in the session's
`backoff.note`; a parameter that was not in the log is listed under `defaulted`; output
is deterministic and free of wall-clock time. Imports only numpy, the stdlib and
`dflog.{tune,tunesim}`; never `dflog.analysis` or `dflog.cli`.
"""

from __future__ import annotations

import bisect
import os
import re

import numpy as np

from .tune import CONSTANTS, DEFAULTS, ACC_MAX_PARAMS, Refusal
from .tunesim import AUTOTUNE, TUNE_STEPS, AXIS_ID, SEQUENCE

__all__ = ["BACKOFF_BY_FIRMWARE", "ATUN_CONSTANTS", "SESSION_GAP_S", "STEP_NAMES", "AXIS_NAMES",
           "MSG_AXIS_NAMES", "FINAL_PARAMS", "TABLE_HEADERS",
           "firmware_version", "backoff_for", "twitch_rule", "autotune_sessions",
           "pool_sessions", "describe", "to_rows", "session_to_dict"]

# --------------------------------------------------------------------- constants

STEP_NAMES = {v: k for k, v in TUNE_STEPS.items()}                 # 0 -> "RATE_D_UP" ...
AXIS_NAMES = {v: k for k, v in AXIS_ID.items()}                    # 0 -> "roll" ... 3 -> "yaw_d"
#: The axis word in the firmware's `report_axis_gains` text (AC_AutoTune.cpp
#: `get_axis_name`): "Roll", "Pitch", "Yaw", "Yaw(D)".
MSG_AXIS_NAMES = {"Roll": 0, "Pitch": 1, "Yaw": 2, "Yaw(D)": 3}
#: The parameters a completed axis saves (`save_tuning_gains`), in report order.
FINAL_PARAMS = ("rat_p", "rat_i", "rat_d", "ang_p", "acc_max_dps2", "flte")
TABLE_HEADERS = ["log", "axis", "steps", "twitches", "complete", "P", "I", "D", "ANG_P", "ACC", "agreement"]

_SRC_STATE = ("ArduPilot libraries/AC_AutoTune/AC_AutoTune.cpp master control_attitude() UPDATE_GAINS "
              "(fetched 2026-09-17); reference/pid-tuning-sources.md section 1.7")
_SRC_PLAN = "docs/pid-tuning-plan.md section 2.4 (tier A)"

#: Session-definition constants (windows, not thresholds - like `flights()`'s gap_seconds).
ATUN_CONSTANTS = {
    "session_gap_s": dict(value=120.0, source=_SRC_PLAN,
                          note="consecutive ATUN rows further apart than this start a new session"),
    "atde_burst_gap_s": dict(value=0.1, source=_SRC_STATE,
                             note="ATDE samples further apart than this belong to different twitches "
                                  "(WAITING_FOR_LEVEL lasts >= REQUIRED_LEVEL_TIME_MS = 250 ms)"),
    "gain_agree_rel": dict(value=1e-3, source=_SRC_PLAN,
                           note="predicted next-row gain vs logged (float32) agree within this relative error"),
    "witness_agree_rel": dict(value=5e-3, source=_SRC_PLAN,
                              note="observed vs assumed rate backoff agree within this relative error"),
    "msg_decimals": dict(value=dict(rat_p=3, rat_i=3, rat_d=4, ang_p=3, acc_max_cdss=0), source=(
        "AC_AutoTune_Multi.cpp report_axis_gains: 'Rate: P:%0.3f, I:%0.3f, D:%0.4f' / "
        "'Angle P:%0.3f, Max Accel:%0.0f' (identical on Copter-4.3..ArduPilot-4.7 and master)"),
        note="the MSG text is rounded; a difference inside half a printed unit is not a disagreement"),
}
SESSION_GAP_S = ATUN_CONSTANTS["session_gap_s"]["value"]

_RAW = "https://raw.githubusercontent.com/ArduPilot/ardupilot/{}/libraries/AC_AutoTune/AC_AutoTune_Multi.cpp"


def _fixed(branch, lines_defs, lines_fn):
    return dict(gmbk=False, rd_backoff=1.0, rp_backoff=1.0, sp_backoff=0.9, sp_aggr=False,
                branch=branch, applied_by="set_gains_post_tune",
                how=("after RATE_D_DOWN: rd = MAX(min_d, rd x RD_BACKOFF), rp = MAX(RP_MIN, rp x RD_BACKOFF) "
                     "(yaw(E): rLPF x RD_BACKOFF); after RATE_P_UP: rp = MAX(RP_MIN, rp x RP_BACKOFF); "
                     "after ANGLE_P_UP: sp = MAX(SP_MIN, sp x SP_BACKOFF), accel = MAX(floor, "
                     "test_accel_max x ACCEL_*_BACKOFF 1.0); no (1 - AGGR) anywhere"),
                source=f"{_RAW.format(branch)} lines {lines_defs} (#define) and {lines_fn} "
                       f"(set_gains_post_tune), fetched 2026-09-17")


#: Verified per branch on 2026-09-17 (reference/pid-tuning-sources.md section 1.6). Keyed
#: by (major, minor) of the firmware string. `gmbk` True means the `AUTOTUNE_GMBK`
#: parameter exists and the backoff is (1 - GMBK) on rate P and D at RATE_P_UP completion
#: and (1 - GMBK)(1 - AGGR) on angle P at ANGLE_P_UP completion; False means the fixed
#: `AUTOTUNE_RD/RP/SP_BACKOFF` constants. ArduPilot-4.7's file is byte-identical to master.
BACKOFF_BY_FIRMWARE = {
    (4, 3): _fixed("Copter-4.3", "60-62", "673-721"),
    (4, 4): _fixed("Copter-4.4", "63-65", "710-764"),
    (4, 5): _fixed("Copter-4.5", "67-69", "735-789"),
    (4, 6): _fixed("ArduPilot-4.6", "67-69", "748-802"),
    (4, 7): dict(gmbk=True, gmbk_default=CONSTANTS["autotune_gmbk_default"]["value"], sp_aggr=True,
                 branch="ArduPilot-4.7", applied_by="set_tuning_gains_with_backoff",
                 how=("after RATE_P_UP: rd, rp x (1 - GMBK) (yaw(E): rp only); after ANGLE_P_UP: "
                      "sp x (1 - GMBK) x (1 - AGGR), accel = cd_to_rad(MAX(floor, test_accel_max_cdss x 1.0)); "
                      "GMBK constrained 0.0-0.5 and saved back"),
                 source=f"{_RAW.format('ArduPilot-4.7')} lines 118-123 (GMBK var_info) and 909-953 "
                        "(set_tuning_gains_with_backoff), fetched 2026-09-17; identical to master"),
}
_KNOWN_VERSIONS = sorted(BACKOFF_BY_FIRMWARE)
_AGGR_LO, _AGGR_HI = CONSTANTS["autotune_aggr_min"]["value"], CONSTANTS["autotune_aggr_max"]["value"]

_STEM = {0: "ATC_RAT_RLL_", 1: "ATC_RAT_PIT_", 2: "ATC_RAT_YAW_", 3: "ATC_RAT_YAW_"}
_ANG = {0: "ATC_ANG_RLL_P", 1: "ATC_ANG_PIT_P", 2: "ATC_ANG_YAW_P", 3: "ATC_ANG_YAW_P"}
_ACC = {0: ACC_MAX_PARAMS["roll"], 1: ACC_MAX_PARAMS["pitch"], 2: ACC_MAX_PARAMS["yaw"], 3: ACC_MAX_PARAMS["yaw"]}
_MSG_RATE = re.compile(r"AutoTune: (Roll|Pitch|Yaw\(D\)|Yaw) Rate: P:([0-9.]+), I:([0-9.]+), D:([0-9.]+)")
_MSG_ANGLE = re.compile(r"AutoTune: (Roll|Pitch|Yaw\(D\)|Yaw) Angle P:([0-9.]+), Max Accel:([0-9.]+)")
_MSG_SAVED = re.compile(r"AutoTune: Saved gains for")


# ------------------------------------------------------------- firmware / backoff

def firmware_version(firmware):
    """(major, minor) from a firmware string such as 'ArduCopter V4.7.1 (dbe79216)', or
    None when no 'V<major>.<minor>' is present."""
    m = re.search(r"\bV(\d+)\.(\d+)", str(firmware or ""))
    return (int(m.group(1)), int(m.group(2))) if m else None


def backoff_for(version, gmbk_param, aggr):
    """The multipliers the flown firmware applied to the found gains.

    version     (major, minor) from `firmware_version`, or None
    gmbk_param  the log's `AUTOTUNE_GMBK`, or None when the parameter is not in the log
    aggr        the (clamped) AUTOTUNE_AGGR

    Returns dict(rate_p, rate_d, sp, gmbk, branch, applied_by, how, source, exact, note):
    `rate_p/rate_d/sp` multiply the last RATE_P_UP row's RP/RD and the last ANGLE_P_UP
    row's SP. `exact` is False whenever anything was assumed, and `note` says what:
    an unknown version falls back to GMBK when `AUTOTUNE_GMBK` is in PARM, else to the
    nearest known branch; a GMBK branch without the parameter in the log assumes the
    default. The presence of `AUTOTUNE_GMBK` in PARM overrides the version table (a
    parameter the firmware wrote is stronger evidence than a banner).
    """
    aggr = float(aggr)
    notes = []
    exact = True
    if gmbk_param is not None:
        row = BACKOFF_BY_FIRMWARE[(4, 7)]
        gm = min(max(float(gmbk_param), 0.0), 0.5)
        if version is None:
            exact = False
            notes.append("firmware version unknown; AUTOTUNE_GMBK is in PARM so the GMBK backoff is assumed")
        elif version not in BACKOFF_BY_FIRMWARE:
            near = _nearest(version)
            if not BACKOFF_BY_FIRMWARE[near]["gmbk"]:
                exact = False
                notes.append(f"firmware {version[0]}.{version[1]} is not in the branch table; AUTOTUNE_GMBK is in "
                             f"PARM so the GMBK backoff (ArduPilot-4.7 / master) is assumed")
            else:
                notes.append(f"firmware {version[0]}.{version[1]} is newer than the branch table; "
                             f"AUTOTUNE_GMBK in PARM confirms the GMBK backoff")
        elif not BACKOFF_BY_FIRMWARE[version]["gmbk"]:
            exact = False
            notes.append(f"the branch table says {BACKOFF_BY_FIRMWARE[version]['branch']} has fixed backoffs, "
                         f"but AUTOTUNE_GMBK is in PARM; the parameter wins")
        if gm != float(gmbk_param):
            notes.append(f"AUTOTUNE_GMBK {gmbk_param} constrained to {gm} as the firmware does")
        return dict(rate_p=1.0 - gm, rate_d=1.0 - gm, sp=(1.0 - gm) * (1.0 - aggr), gmbk=gm,
                    gmbk_source="AUTOTUNE_GMBK (PARM)", branch=row["branch"], applied_by=row["applied_by"],
                    how=row["how"], source=row["source"], exact=exact, note="; ".join(notes))

    # no AUTOTUNE_GMBK in the log
    if version is None:
        key = _KNOWN_VERSIONS[-2]                          # the newest fixed-backoff branch
        exact = False
        notes.append("firmware version unknown and AUTOTUNE_GMBK not in PARM; the fixed backoffs of "
                     f"{BACKOFF_BY_FIRMWARE[key]['branch']} (the newest branch without GMBK) are assumed")
    elif version in BACKOFF_BY_FIRMWARE:
        key = version
    else:
        key = _nearest(version)
        exact = False
        notes.append(f"firmware {version[0]}.{version[1]} is not in the branch table; the nearest known branch "
                     f"{BACKOFF_BY_FIRMWARE[key]['branch']} is assumed")
    row = BACKOFF_BY_FIRMWARE[key]
    if row["gmbk"]:
        gm = float(row["gmbk_default"])
        exact = False
        notes.append(f"{row['branch']} uses AUTOTUNE_GMBK but the parameter is not in the log; the default "
                     f"{gm} is assumed - read backoff.rate_p_observed, the log's own witness")
        return dict(rate_p=1.0 - gm, rate_d=1.0 - gm, sp=(1.0 - gm) * (1.0 - aggr), gmbk=gm,
                    gmbk_source="AUTOTUNE_GMBK default (parameter not in log)", branch=row["branch"],
                    applied_by=row["applied_by"], how=row["how"], source=row["source"], exact=exact,
                    note="; ".join(notes))
    return dict(rate_p=row["rp_backoff"], rate_d=row["rd_backoff"], sp=row["sp_backoff"], gmbk=None,
                gmbk_source="none (fixed AUTOTUNE_*_BACKOFF constants)", branch=row["branch"],
                applied_by=row["applied_by"], how=row["how"], source=row["source"], exact=exact,
                note="; ".join(notes))


def _nearest(version):
    """The known (major, minor) closest to `version`, the newer one on a tie."""
    want = version[0] * 100 + version[1]
    return min(_KNOWN_VERSIONS, key=lambda k: (abs(k[0] * 100 + k[1] - want), -(k[0] * 100 + k[1])))


# ------------------------------------------------------------------- the rules

def twitch_rule(step, axis_id, targ, mn, mx, rp, rd, sp, success, ignore_next, aggr, min_d,
                rate_min=None, rate_max=None):
    """What the firmware's UPDATE_GAINS rule does with one ATUN row
    (`updating_rate_d_up/_down`, `updating_rate_p_up_d_down`, `updating_angle_p_down/_up`
    in AC_AutoTune_Multi.cpp master; sources file section 1.5).

    step        "RATE_D_UP" ... "ANGLE_P_UP"
    axis_id     ATUN.Axis: 0 roll, 1 pitch, 2 yaw(E) (RD is FLTE, limits RLPF_MIN/MAX), 3 yaw-D
    targ/mn/mx  the row's Targ/Min/Max (any consistent unit; the rules are ratios)
    rp/rd/sp    the gains tested (the row's RP/RD/SP)
    success, ignore_next   the state carried from the previous twitch
    rate_min/rate_max      the twitch's rate extremes (angle steps only, from ATDE); None
                           when not available - the ANGLE_P_UP second clause is then
                           `uncertain` unless the first clause decides

    Returns dict(code, text, rp, rd, sp, success, ignore_next, passed, limit, failed,
    uncertain): the gains and state *after* the rule, `passed` True when success_counter
    was incremented, `limit` the REACHED_LIMIT text if any, `failed` the FAILED text.
    """
    A = AUTOTUNE
    yaw_e = axis_id == 2
    d_min, d_max = (A["RLPF_MIN"], A["RLPF_MAX"]) if yaw_e else (float(min_d), A["RD_MAX"])
    rp_min, rp_max = A["RP_MIN"], A["RP_MAX"]
    rd_step, rp_step, sp_step = A["RD_STEP"], A["RP_STEP"], A["SP_STEP"]
    margin, over = A["D_UP_DOWN_MARGIN"], CONSTANTS["autotune_overshoot_aggr_scale"]["value"]
    n_ok = A["SUCCESS_COUNT"]
    passed, limit, failed, uncertain = False, None, None, False
    code, text = "", ""

    if step in ("RATE_D_UP", "RATE_D_DOWN"):
        if mx > targ:
            rp -= rp * rp_step
            code, text = "fail-high", "peak above target: P down 5 %"
            if rp < rp_min:
                rp = rp_min
                rd -= rd * rd_step
                code, text = "fail-high", "peak above target with P at its minimum: D down 5 %"
                if rd <= d_min:
                    rd = d_min
                    success = n_ok
                    limit = "Min Rate D limit reached"
                    code, text = "limit", "P and D at their minimum: step forced complete"
        elif mx < targ * (1.0 - margin) and rp <= rp_max:
            rp += rp * rp_step
            code, text = "fail-low", "peak below 80 % of target: P up 5 %"
            if rp >= rp_max:
                rp = rp_max
                limit = "Rate P max reached"
                code, text = "limit", "P at its maximum"
        elif step == "RATE_D_UP":
            if mx - mn > mx * aggr:
                ignore_next, success, passed = True, success + 1, True
                code, text = "success", "bounce-back >= AGGR x peak"
            elif not ignore_next:
                success = max(0, success - 1)
                rd += rd * rd_step * 2.0
                code, text = "bounce-too-small", "bounce-back below AGGR x peak: D up 10 %"
                if rd >= d_max:
                    rd = d_max
                    success = n_ok
                    limit = "Rate D max reached"
                    code, text = "limit", "D at its maximum: step forced complete"
            else:
                ignore_next = False
                code, text = "ignored", "twitch after a pass is ignored (ignore_next)"
        else:                                                   # RATE_D_DOWN
            if mx - mn < mx * aggr:
                if not ignore_next:
                    success, passed = success + 1, True
                    code, text = "success", "bounce-back < AGGR x peak"
                else:
                    ignore_next = False
                    code, text = "ignored", "twitch after a pass is ignored (ignore_next)"
            else:
                ignore_next = True
                success = max(0, success - 1)
                rd -= rd * rd_step
                code, text = "bounce-too-large", "bounce-back >= AGGR x peak: D down 5 %"
                if rd <= d_min:
                    rd = d_min
                    success = n_ok
                    limit = "Min Rate D limit reached"
                    code, text = "limit", "D at its minimum: step forced complete"
    elif step == "RATE_P_UP":
        if mx > targ * (1.0 + over * aggr):
            ignore_next, success, passed = True, success + 1, True
            code, text = "success", "overshoot >= 0.5 AGGR x target"
        elif mx < targ and mx > targ * (1.0 - margin) and mx - mn > mx * aggr and rd > d_min:
            success = max(0, success - 1)
            rd -= rd * rd_step
            code, text = "bounce-too-large", "peak in 80-100 % of target with bounce-back: D down 5 %, P down 5 %"
            if rd <= d_min:
                rd = d_min
                limit = "Rate D min reached"
                if not yaw_e:
                    failed = "Rate D Gain Determination Failed"
            rp -= rp * rp_step
            if rp <= rp_min:
                rp = rp_min
                failed = "Rate P Gain Determination Failed"
        elif not ignore_next:
            success = max(0, success - 1)
            rp += rp * rp_step
            code, text = "fail-low", "no overshoot: P up 5 %"
            if rp >= rp_max:
                rp = rp_max
                success = n_ok
                limit = "Rate P max reached"
                code, text = "limit", "P at its maximum: step forced complete"
        else:
            ignore_next = False
            code, text = "ignored", "twitch after a pass is ignored (ignore_next)"
    elif step == "ANGLE_P_DOWN":
        if mx < targ * (1.0 + over * aggr):
            if not ignore_next:
                success, passed = success + 1, True
                code, text = "success", "angle overshoot < 0.5 AGGR x target"
            else:
                ignore_next = False
                code, text = "ignored", "twitch after a pass is ignored (ignore_next)"
        else:
            ignore_next = True
            success = max(0, success - 1)
            sp -= sp * sp_step
            code, text = "fail-high", "angle overshoot >= 0.5 AGGR x target: angle P down 5 %"
            if sp <= A["SP_MIN"]:
                sp = A["SP_MIN"]
                limit = "Angle P min reached"
                failed = "Angle P Gain Determination Failed"
    elif step == "ANGLE_P_UP":
        first = mx > targ * (1.0 + over * aggr)
        second = None
        if rate_min is not None and rate_max is not None:
            second = mx > targ and rate_min < -rate_max * aggr
        if first or second:
            ignore_next, success, passed = True, success + 1, True
            code, text = "success", ("angle overshoot >= 0.5 AGGR x target" if first
                                     else "angle over target with rate bounce-back >= AGGR (ATDE)")
        else:
            if second is None and mx > targ:
                uncertain = True
            if not ignore_next:
                success = max(0, success - 1)
                sp += sp * sp_step
                code, text = "fail-low", "no angle overshoot: angle P up 5 %"
                if sp >= A["SP_MAX"]:
                    sp = A["SP_MAX"]
                    success = n_ok
                    limit = "Angle P max reached"
                    code, text = "limit", "angle P at its maximum: step forced complete"
            else:
                ignore_next = False
                code, text = "ignored", "twitch after a pass is ignored (ignore_next)"
            if uncertain:
                text += " (rate bounce-back clause not evaluable: no ATDE)"
    else:
        raise ValueError(f"no reconstruction rule for step {step!r}")
    return dict(code=code, text=text, rp=float(rp), rd=float(rd), sp=float(sp), success=int(success),
                ignore_next=bool(ignore_next), passed=bool(passed), limit=limit, failed=failed,
                uncertain=bool(uncertain))


# ---------------------------------------------------------------- log readers

def _log_name(log):
    path = getattr(log, "path", None)
    return os.path.basename(str(path)) if path else "log"


def _events(log):
    d = log.df("EV")
    if d is None or d.empty or "Id" not in d.columns:
        return []
    return [(float(t), int(i)) for t, i in zip(d["t"].values, d["Id"].values)]


def _atde_bursts(log):
    """[(t_start, t_end, rates)] - ATDE.Rate split into twitches at gaps."""
    d = log.df("ATDE")
    if d is None or d.empty or "Rate" not in d.columns:
        return []
    t = np.asarray(d["t"].values, dtype=float)
    r = np.asarray(d["Rate"].values, dtype=float)
    gap = ATUN_CONSTANTS["atde_burst_gap_s"]["value"]
    cuts = np.flatnonzero(np.diff(t) > gap) + 1
    out = []
    for a, b in zip(np.r_[0, cuts], np.r_[cuts, len(t)]):
        if b > a:
            out.append((float(t[a]), float(t[b - 1]), r[a:b]))
    return out


def _rate_extremes(bursts, ends, t_row):
    """(rate_min, rate_max) of the last ATDE burst ending at or before the ATUN row, as
    `twitching_test_angle` tracks them: the running maximum, and the minimum after it."""
    i = bisect.bisect_right(ends, t_row + 1e-6) - 1
    if i < 0:
        return None, None
    r = bursts[i][2]
    if len(r) == 0:
        return None, None
    k = int(np.argmax(r))
    return float(np.min(r[k:])), float(r[k])


def _param(log, name, t, defaulted):
    v = log.param_at(name, t)
    if v is None:
        if name in DEFAULTS:
            defaulted.append(name)
            return float(DEFAULTS[name]["value"])
        return None
    return float(v)


def _messages(log):
    try:
        return [(float(t), str(m)) for t, m in log.messages_text()]
    except Exception:                                          # pragma: no cover - MSG absent
        return []


def _finite(x):
    return None if x is None or not np.isfinite(x) else float(x)


# ------------------------------------------------------------- reconstruction

def autotune_sessions(log, aggr_override=None, ignore_next_carry=True, log_name=None):
    """One dict per (session, axis) reconstructed from the log's AutoTune records.

    aggr_override      use this AGGR instead of the log's `AUTOTUNE_AGGR` (noted)
    ignore_next_carry  the firmware never resets `ignore_next` when a step or an axis
                       completes (AC_AutoTune.cpp UPDATE_GAINS only zeroes success_counter
                       and step_scaler), so a twitch that follows a step's fourth pass
                       is ignored by the next step's rule. True reproduces that; False
                       resets it at every step boundary (what `tunesim.autotune` does)

    Returns [] when the log has no `ATUN`. Each entry carries: identity (`log_name`,
    `session`, `n_sessions`, `t0`, `t1`, `axis`, `axis_id`, `firmware`, `version`),
    the inputs (`aggr`, `aggr_source`, `min_d`, `gmbk_param`, `initial`, `defaulted`),
    `steps` (ordered dicts: step, step_id, n_twitches, passes, completed, first/last
    gains, limits), `twitches` (every ATUN row with the re-derived `code`, `passed`,
    `success`, the predicted next gains and `agrees_next`), `rederivation` (tallies),
    `complete` with its `witnesses`, `found` (the pre-backoff gains), `backoff`
    (assumed multipliers with source, and the in-log observed rate backoff), `final`
    (rat_p, rat_i, rat_d, ang_p, acc_max_dps2, flte; None when incomplete),
    `msg_gains`, `saved_params`, `agreement`, `mismatch`, `mismatch_note`, `events`,
    `refusals` (`tune.Refusal` instances).
    """
    name = log_name or _log_name(log)
    d = log.df("ATUN")
    need = {"Axis", "TuneStep", "Targ", "Min", "Max", "RP", "RD", "SP", "ddt", "t"}
    if d is None or d.empty or not need <= set(d.columns):
        return []
    d = d.sort_values("t", kind="stable")
    rows = [dict(t=float(r.t), axis=int(r.Axis), step_id=int(r.TuneStep), targ=float(r.Targ), min=float(r.Min),
                 max=float(r.Max), rp=float(r.RP), rd=float(r.RD), sp=float(r.SP), ddt=float(r.ddt))
            for r in d.itertuples(index=False)]
    rows = [r for r in rows if r["step_id"] in STEP_NAMES and STEP_NAMES[r["step_id"]] in SEQUENCE]
    if not rows:
        return []

    events = _events(log)
    starts = sorted(t for t, i in events if i == 30)
    msgs = _messages(log)
    bursts = _atde_bursts(log)
    burst_ends = [b[1] for b in bursts]
    firmware = log.firmware()
    version = firmware_version(firmware)
    parm = log.df("PARM")

    # ---- session boundaries: EV 30, long gaps, a TuneStep going backwards on an axis
    sessions, cur = [], []
    last_step = {}
    for r in rows:
        boundary = False
        if cur:
            prev_t = cur[-1]["t"]
            if any(prev_t < s <= r["t"] for s in starts):
                boundary = True
            elif r["t"] - prev_t > SESSION_GAP_S:
                boundary = True
            elif r["axis"] in last_step and r["step_id"] < last_step[r["axis"]]:
                boundary = True
        if boundary:
            sessions.append(cur)
            cur, last_step = [], {}
        cur.append(r)
        last_step[r["axis"]] = r["step_id"]
    if cur:
        sessions.append(cur)
    n_sessions = len(sessions)

    out = []
    for si, srows in enumerate(sessions):
        t0, t1 = srows[0]["t"], srows[-1]["t"]
        t_next = sessions[si + 1][0]["t"] if si + 1 < n_sessions else float("inf")
        ev30 = [s for s in starts if s <= t0]
        t_start = max(ev30) if ev30 else t0
        sess_events = [(t, i, ) for t, i in events if t_start <= t < t_next and 30 <= i <= 37]
        defaulted = []
        aggr_raw = _param(log, "AUTOTUNE_AGGR", t0, defaulted)
        aggr_source = "AUTOTUNE_AGGR (PARM)" if "AUTOTUNE_AGGR" not in defaulted else "AUTOTUNE_AGGR default"
        if aggr_override is not None:
            aggr_raw, aggr_source = float(aggr_override), "override"
        aggr = min(max(float(aggr_raw), _AGGR_LO), _AGGR_HI)
        min_d = _param(log, "AUTOTUNE_MIN_D", t0, defaulted)
        gmbk_param = log.param_at("AUTOTUNE_GMBK", t0)
        gmbk_param = float(gmbk_param) if gmbk_param is not None else None
        backoff = backoff_for(version, gmbk_param, aggr)

        # one ignore_next / success state for the session, as the firmware has one member
        state = dict(success=0, ignore_next=False)
        axes_in_order = []
        for r in srows:
            if r["axis"] not in axes_in_order:
                axes_in_order.append(r["axis"])
        per_axis = {a: [] for a in axes_in_order}
        cur_step = {}
        # walk in time order so the state carries exactly as it did in the firmware
        for idx, r in enumerate(srows):
            a = r["axis"]
            step = STEP_NAMES[r["step_id"]]
            if cur_step.get(a) != step:
                state["success"] = 0
                if not ignore_next_carry:
                    state["ignore_next"] = False
                cur_step[a] = step
            rate_min = rate_max = None
            if step in ("ANGLE_P_DOWN", "ANGLE_P_UP"):
                rate_min, rate_max = _rate_extremes(bursts, burst_ends, r["t"])
            res = twitch_rule(step, a, r["targ"], r["min"], r["max"], r["rp"], r["rd"], r["sp"],
                              state["success"], state["ignore_next"], aggr, min_d, rate_min, rate_max)
            state["success"], state["ignore_next"] = res["success"], res["ignore_next"]
            tw = dict(t=r["t"], step=step, step_id=r["step_id"], targ=r["targ"], min=r["min"], max=r["max"],
                      rp=r["rp"], rd=r["rd"], sp=r["sp"], ddt=r["ddt"], code=res["code"], text=res["text"],
                      passed=res["passed"], success=res["success"], limit=res["limit"], failed=res["failed"],
                      uncertain=res["uncertain"], rate_min=rate_min, rate_max=rate_max,
                      predicted=dict(rp=res["rp"], rd=res["rd"], sp=res["sp"]), agrees_next=None)
            if res["success"] >= AUTOTUNE["SUCCESS_COUNT"]:
                state["success"] = 0                              # the firmware zeroes it on completion
                tw["completed_step"] = True
            else:
                tw["completed_step"] = False
            per_axis[a].append(tw)

        for a in axes_in_order:
            tws = per_axis[a]
            out.append(_axis_session(log, name, si, n_sessions, t_start, t0, t1, t_next, a, tws, aggr,
                                     aggr_source, min_d, gmbk_param, backoff, firmware, version, msgs,
                                     parm, events, sess_events, defaulted))
    return out


def _axis_session(log, name, si, n_sessions, t_start, t0, t1, t_next, axis_id, tws, aggr, aggr_source,
                  min_d, gmbk_param, backoff, firmware, version, msgs, parm, events, sess_events, defaulted):
    axis = AXIS_NAMES.get(axis_id, f"axis{axis_id}")
    yaw_e, yaw = axis_id == 2, axis_id in (2, 3)
    stem = _STEM[axis_id]
    rel_tol = ATUN_CONSTANTS["gain_agree_rel"]["value"]
    defaulted = list(defaulted)

    # ---- configured gains at the session start (what the tune started from)
    initial = dict(rat_p=_param(log, stem + "P", t0, defaulted), rat_i=_param(log, stem + "I", t0, defaulted),
                   rat_d=_param(log, stem + "D", t0, defaulted), ang_p=_param(log, _ANG[axis_id], t0, defaulted),
                   flte=_param(log, stem + "FLTE", t0, defaulted))
    acc_new, acc_old = _ACC[axis_id]
    v = log.param_at(acc_new, t0)
    if v is not None:
        initial["acc_max_dps2"], acc_name = float(v), acc_new
    else:
        v = log.param_at(acc_old, t0)
        if v is not None:
            initial["acc_max_dps2"], acc_name = float(v) / 100.0, acc_old
        else:
            initial["acc_max_dps2"], acc_name = DEFAULTS[acc_old]["value"] / 100.0, acc_old
            defaulted.append(acc_old)

    # ---- steps, in order of appearance; agreement of each predicted gain with the next row
    steps = []
    order = []
    for tw in tws:
        if not order or order[-1] != tw["step"]:
            order.append(tw["step"])
    by_step = {s: [tw for tw in tws if tw["step"] == s] for s in order}
    for i, tw in enumerate(tws[:-1]):
        nxt = tws[i + 1]
        p = tw["predicted"]
        mult_p = mult_d = 1.0
        if nxt["step"] != tw["step"] and tw["step"] == "RATE_P_UP":
            mult_p, mult_d = backoff["rate_p"], backoff["rate_d"]
            if yaw_e:
                mult_d = 1.0
        ok = (_close(p["rp"] * mult_p, nxt["rp"], rel_tol) and _close(p["rd"] * mult_d, nxt["rd"], rel_tol)
              and _close(p["sp"], nxt["sp"], rel_tol))
        tw["agrees_next"] = bool(ok)
        # an uncertain ANGLE_P_UP verdict (no ATDE) is settled by what the firmware did next
        if tw["uncertain"] and nxt["step"] == tw["step"]:
            if _close(nxt["sp"], tw["sp"], rel_tol):
                tw["passed"], tw["code"] = True, "success"
                tw["text"] = "angle over target; the next row's unchanged SP shows the firmware counted a pass"
                tw["agrees_next"] = True
    for s in order:
        lst = by_step[s]
        limits = sorted({tw["limit"] for tw in lst if tw["limit"]})
        completed = lst[-1]["completed_step"] or order.index(s) < len(order) - 1
        steps.append(dict(step=s, step_id=TUNE_STEPS[s], n_twitches=len(lst), passes=sum(tw["passed"] for tw in lst),
                          completed=bool(completed), reached_success_count=bool(lst[-1]["completed_step"]),
                          first=dict(rp=lst[0]["rp"], rd=lst[0]["rd"], sp=lst[0]["sp"]),
                          last=dict(rp=lst[-1]["rp"], rd=lst[-1]["rd"], sp=lst[-1]["sp"]),
                          after=dict(lst[-1]["predicted"]), limits=limits,
                          failed=next((tw["failed"] for tw in lst if tw["failed"]), None),
                          t0=lst[0]["t"], t1=lst[-1]["t"]))
    judged = [tw for tw in tws if tw["agrees_next"] is not None]
    n_agree = sum(tw["agrees_next"] for tw in judged)
    rederivation = dict(n_twitches=len(tws), n_judged=len(judged), n_gain_agree=int(n_agree),
                        gain_agree_pct=(100.0 * n_agree / len(judged)) if judged else None,
                        n_passed=int(sum(tw["passed"] for tw in tws)),
                        n_uncertain=int(sum(tw["uncertain"] for tw in tws)),
                        disagreements=[tw["t"] for tw in judged if not tw["agrees_next"]])

    # ---- MSG lines for this axis after the session's last row and before the next session
    msg_gains, msg_texts = {}, []
    want = {v: k for k, v in MSG_AXIS_NAMES.items()}[axis_id]
    for t, m in msgs:
        if not (t >= t0 and t < t_next):
            continue
        h = _MSG_RATE.search(m)
        if h and h.group(1) == want and "rat_p" not in msg_gains:
            msg_gains.update(rat_p=float(h.group(2)), rat_i=float(h.group(3)), rat_d=float(h.group(4)))
            msg_texts.append(m)
        h = _MSG_ANGLE.search(m)
        if h and h.group(1) == want and "ang_p" not in msg_gains:
            msg_gains.update(ang_p=float(h.group(2)), acc_max_cdss=float(h.group(3)))
            msg_texts.append(m)
        if _MSG_SAVED.search(m) and want.split("(")[0] in m:
            msg_texts.append(m)

    # ---- completion witnesses
    witnesses = []
    if "ANGLE_P_UP" in by_step and by_step["ANGLE_P_UP"][-1]["completed_step"]:
        witnesses.append("ANGLE_P_UP reached 4 passes by re-derivation")
    if any(i == 33 for _t, i in sess_events):
        witnesses.append("EV 33 AUTOTUNE_SUCCESS")
    if "rat_p" in msg_gains:
        witnesses.append("MSG 'AutoTune: %s Rate:' line" % want)
    complete = bool(witnesses)
    # EV 37 is written when *any* axis was saved, so it corroborates but never decides
    if complete and any(i == 37 for _t, i in sess_events):
        witnesses.append("EV 37 AUTOTUNE_SAVEDGAINS")
    failed = next((s["failed"] for s in steps if s["failed"]), None)
    if any(i == 34 for _t, i in sess_events) and not complete:
        failed = failed or "EV 34 AUTOTUNE_FAILED"

    # ---- found (pre-backoff) and final gains
    found, final, witness = None, None, dict(rate_p_observed=None, rate_d_observed=None, agrees=None)
    if "RATE_P_UP" in by_step and steps[order.index("RATE_P_UP")]["completed"]:
        after = by_step["RATE_P_UP"][-1]["predicted"]
        found = dict(rp=after["rp"], rd=after["rd"])
        angle_rows = by_step.get("ANGLE_P_DOWN", []) + by_step.get("ANGLE_P_UP", [])
        if angle_rows and found["rp"] > 0:
            obs_p = float(np.median([tw["rp"] / found["rp"] for tw in angle_rows]))
            obs_d = float(np.median([tw["rd"] / found["rd"] for tw in angle_rows])) if found["rd"] > 0 else None
            witness["rate_p_observed"] = obs_p
            witness["rate_d_observed"] = obs_d
            witness["agrees"] = bool(_close(obs_p, backoff["rate_p"], ATUN_CONSTANTS["witness_agree_rel"]["value"]))
    if complete and "ANGLE_P_UP" in by_step and found is not None:
        last_up = by_step["ANGLE_P_UP"][-1]
        found["sp"] = last_up["predicted"]["sp"]
        found["ddt_last_angle_cdss"] = last_up["ddt"]
        found["ddt_max_cdss"] = max(tw["ddt"] for tw in tws)
        floor = AUTOTUNE["Y_ACCEL_MIN"] if yaw else AUTOTUNE["RP_ACCEL_MIN"]
        acc_backoff = AUTOTUNE["ACCEL_Y_BACKOFF"] if yaw else AUTOTUNE["ACCEL_RP_BACKOFF"]
        pi = AUTOTUNE["YAW_PI_RATIO_FINAL"] if yaw else AUTOTUNE["PI_RATIO_FINAL"]
        # The rate backoff ran when RATE_P_UP completed, so the ANGLE_P_UP rows carry the
        # firmware's own post-backoff RP/RD: the saved rate gains, with no table assumed.
        # Only angle P needs the branch's SP backoff (nothing is logged after ANGLE_P_UP).
        rat_p = last_up["rp"]
        if yaw_e:
            rat_d, flte = initial["rat_d"], last_up["rd"]
        else:
            rat_d, flte = last_up["rd"], initial["flte"]
        acc_cdss = max(floor, found["ddt_last_angle_cdss"] * acc_backoff)
        final = dict(rat_p=rat_p, rat_i=rat_p * pi, rat_d=rat_d, ang_p=found["sp"] * backoff["sp"],
                     acc_max_dps2=acc_cdss / 100.0, flte=flte, acc_floor_applied=bool(acc_cdss == floor),
                     rate_source="ANGLE_P_UP rows' RP/RD (the firmware's own post-backoff gains; equal to the "
                                 "last RATE_P_UP row x the rate backoff, which backoff.rate_p_observed measures)",
                     sp_source=f"last ANGLE_P_UP row's SP x {backoff['sp']:.4f} ({backoff['branch']}, "
                               f"{backoff['applied_by']})",
                     acc_source="max(floor, ddt of the last ANGLE_P_UP row) - test_accel_max is overwritten per test")

    # ---- saved parameters after EV 37
    saved_params, saved_note = {}, ""
    t37 = [t for t, i in events if i == 37 and t >= t1 and t < t_next]
    if not t37:
        saved_note = "no EV 37 (SAVEDGAINS) after the session; the PARM values at the next boot are the witness"
    elif parm is None or parm.empty:
        saved_note = "EV 37 but no PARM records in the log"
    else:
        after = parm[parm["t"] >= t37[0]]
        names = {stem + "P": "rat_p", stem + "I": "rat_i", stem + "D": "rat_d", _ANG[axis_id]: "ang_p",
                 stem + "FLTE": "flte", acc_new: "acc_max_dps2", acc_old: "acc_max_cdss"}
        for pname, val in zip(after["Name"].values, after["Value"].values):
            if pname in names:
                saved_params[names[pname]] = float(val)
        if "acc_max_cdss" in saved_params and "acc_max_dps2" not in saved_params:
            saved_params["acc_max_dps2"] = saved_params.pop("acc_max_cdss") / 100.0
        elif "acc_max_cdss" in saved_params:
            saved_params.pop("acc_max_cdss")
        if not saved_params:
            saved_note = "EV 37 but no PARM rows for this axis after it; the values at the next boot are the witness"
        else:
            saved_note = f"PARM written after EV 37 at {t37[0]:.1f} s"

    # ---- agreement and the mismatch verdict
    agreement = dict(msg={}, parm={}, msg_max_pct=None, parm_max_pct=None, msg_within_rounding=None)
    mismatch, notes = False, []
    if final is not None and msg_gains:
        dec = ATUN_CONSTANTS["msg_decimals"]["value"]
        tol_pct = CONSTANTS["autotune_msg_agree_pct"]["value"]
        worst = 0.0
        within = True
        for key in ("rat_p", "rat_i", "rat_d", "ang_p", "acc_max_cdss"):
            if key not in msg_gains:
                continue
            mine = final["acc_max_dps2"] * 100.0 if key == "acc_max_cdss" else final[key]
            ref = msg_gains[key]
            diff = mine - ref
            pct = 100.0 * diff / ref if ref else (0.0 if diff == 0 else float("inf"))
            half_unit = 0.5 * 10.0 ** (-dec[key]) * (1.0 + 1e-9)
            rounding_ok = abs(diff) <= half_unit
            within = within and rounding_ok
            agreement["msg"][key] = dict(reconstruction=mine, msg=ref, pct=pct, within_rounding=rounding_ok)
            worst = max(worst, abs(pct))
            if abs(pct) > tol_pct and not rounding_ok:
                mismatch = True
                notes.append(f"{key}: reconstruction {mine:.6g} vs MSG {ref:.6g} ({pct:+.2f} %)")
        agreement["msg_max_pct"], agreement["msg_within_rounding"] = worst, within
    if final is not None and saved_params:
        worst = 0.0
        for key, ref in saved_params.items():
            if key not in final or final[key] is None:
                continue
            mine = final[key]
            pct = 100.0 * (mine - ref) / ref if ref else (0.0 if mine == ref else float("inf"))
            agreement["parm"][key] = dict(reconstruction=mine, parm=ref, pct=pct)
            worst = max(worst, abs(pct))
        agreement["parm_max_pct"] = worst
    mismatch_note = ""
    if mismatch:
        if not backoff["exact"]:
            blame = ("the reconstruction is the more likely wrong: its backoff was assumed (" + backoff["note"] + ")")
        elif witness["agrees"] is False:
            blame = (f"the reconstruction's branch table is the more likely wrong for this firmware: the log's "
                     f"own ANGLE_P rows show a rate backoff of x{witness['rate_p_observed']:.3f} where "
                     f"x{backoff['rate_p']:.3f} ({backoff['branch']}) was assumed; the rate gains were taken "
                     f"from those rows and are unaffected, the angle P backoff is the suspect number")
        else:
            blame = (f"the firmware branch is known ({backoff['branch']}) and the in-log rate-backoff witness "
                     f"agrees (x{witness['rate_p_observed']:.3f}), so the MSG text (or a firmware that differs "
                     f"from the branch table) is the more likely wrong; check the version")
        mismatch_note = "reconstruction and MSG differ by more than %.0f %%: " % CONSTANTS["autotune_msg_agree_pct"]["value"] \
            + "; ".join(notes) + ". " + blame

    backoff_warning = ""
    if witness["agrees"] is False:
        backoff_warning = (f"the branch table's rate backoff x{backoff['rate_p']:.3f} ({backoff['branch']}) disagrees "
                           f"with the log's own witness x{witness['rate_p_observed']:.3f}; the angle P backoff "
                           f"x{backoff['sp']:.3f} is therefore unverified")

    # ---- refusals
    refusals = []
    if not complete:
        done = [s["step"] for s in steps if s["completed"]]
        last = steps[-1]["step"] if steps else "none"
        msg = (f"{axis}: the session ended during {last} after {len(tws)} twitch(es)"
               + (f"; completed steps: {', '.join(done)}" if done else "; no step completed")
               + (f"; firmware reported '{failed}'" if failed else ""))
        refusals.append(Refusal("AUTOTUNE_INCOMPLETE", msg,
                                "informational: tiers B and C from an ordinary fast-logged flight do not need "
                                f"a session; rerun AutoTune on {axis} (AUTOTUNE_AXES bit "
                                f"{1 << (axis_id if axis_id < 3 else 3)}) only if tier A is wanted; "
                                "gains of an incomplete axis are never saved by the firmware",
                                axis=axis, log_name=name))

    return dict(log_name=name, session=si + 1, n_sessions=n_sessions, t_start=t_start, t0=t0, t1=t1,
                axis=axis, axis_id=axis_id, firmware=firmware, version=list(version) if version else None,
                aggr=aggr, aggr_source=aggr_source, min_d=min_d, gmbk_param=gmbk_param,
                initial=initial, acc_param_name=acc_name, defaulted=sorted(set(defaulted)),
                steps=steps, twitches=tws, n_twitches=len(tws), rederivation=rederivation,
                complete=complete, witnesses=witnesses, failed=failed, found=found,
                backoff=dict(backoff, **witness), backoff_warning=backoff_warning, final=final,
                msg_gains=msg_gains, msg_texts=msg_texts,
                saved_params=saved_params, saved_note=saved_note, agreement=agreement,
                mismatch=mismatch, mismatch_note=mismatch_note,
                events=[(t, i) for t, i in sess_events], refusals=refusals)


def _close(a, b, rel):
    if a is None or b is None:
        return False
    return abs(a - b) <= rel * max(abs(a), abs(b), 1e-12)


def session_to_dict(session):
    """A JSON-safe copy: Refusal instances become dicts, numpy scalars floats."""
    out = dict(session)
    out["refusals"] = [r.to_dict() if hasattr(r, "to_dict") else r for r in session["refusals"]]
    return out


# ------------------------------------------------------------------ pooling

def pool_sessions(sessions_by_log):
    """Pool the final gains of every *complete* (session, axis) across logs.

    sessions_by_log  {log_name: [session dict, ...]} as `autotune_sessions` returns

    Returns {axis: {param: dict(median, spread, n, n_logs, values, values_by_log)}} with
    `spread = (max - min) / median` (None when the median is 0). Empty when no session
    is complete. Grading against `T["tune_session_spread"]` is the caller's.
    """
    pooled = {}
    for log_name in sorted(sessions_by_log):
        for s in sessions_by_log[log_name]:
            if not s.get("complete") or not s.get("final"):
                continue
            ax = pooled.setdefault(s["axis"], {})
            for p in FINAL_PARAMS:
                v = s["final"].get(p)
                if v is None or not np.isfinite(v):
                    continue
                e = ax.setdefault(p, dict(values=[], values_by_log={}))
                e["values"].append(float(v))
                e["values_by_log"].setdefault(log_name, []).append(float(v))
    out = {}
    for axis in sorted(pooled):
        out[axis] = {}
        for p in FINAL_PARAMS:
            if p not in pooled[axis]:
                continue
            vals = pooled[axis][p]["values"]
            med = float(np.median(vals))
            if max(vals) == min(vals):
                spread = 0.0
            else:
                spread = (max(vals) - min(vals)) / med if med else None
            out[axis][p] = dict(median=med, spread=spread, n=len(vals),
                                n_logs=len(pooled[axis][p]["values_by_log"]), values=list(vals),
                                values_by_log={k: list(v) for k, v in sorted(pooled[axis][p]["values_by_log"].items())})
    return out


# ---------------------------------------------------------------- rendering

def _fmt(v, nd=4):
    if v is None:
        return "-"
    if abs(v) >= 1e4:
        return f"{v:.0f}"
    return f"{v:.{nd}g}" if abs(v) < 1e-2 else f"{v:.{nd}f}".rstrip("0").rstrip(".")


def describe(session):
    """Deterministic note lines for one (session, axis) reconstruction."""
    s = session
    b = s["backoff"]
    lines = []
    head = (f"{s['log_name']} {s['axis']}: session {s['session']} of {s['n_sessions']} "
            f"({s['t0']:.1f}-{s['t1']:.1f} s), {len(s['steps'])} step(s), {s['n_twitches']} twitch(es), "
            + ("complete" if s["complete"] else "INCOMPLETE"))
    if s["witnesses"]:
        head += " (" + "; ".join(s["witnesses"]) + ")"
    if s["failed"]:
        head += f"; failed: {s['failed']}"
    lines.append(head)
    fw = s["firmware"] or "no firmware banner"
    lines.append(f"firmware: {fw}; AGGR {s['aggr']:.3f} ({s['aggr_source']}); MIN_D {_fmt(s['min_d'])}; "
                 f"AUTOTUNE_GMBK {'not in log' if s['gmbk_param'] is None else _fmt(s['gmbk_param'])}"
                 + (f"; defaults assumed for {', '.join(s['defaulted'])}" if s["defaulted"] else ""))
    for st in s["steps"]:
        lines.append(f"  {st['step']}: {st['n_twitches']} twitch(es), {st['passes']} pass(es), "
                     + ("completed" if st["completed"] else "not completed")
                     + f"; RP {_fmt(st['first']['rp'])} -> {_fmt(st['after']['rp'])}, "
                     f"RD {_fmt(st['first']['rd'])} -> {_fmt(st['after']['rd'])}, "
                     f"SP {_fmt(st['first']['sp'])} -> {_fmt(st['after']['sp'])}"
                     + (f"; limits: {', '.join(st['limits'])}" if st["limits"] else ""))
    r = s["rederivation"]
    if r["n_judged"]:
        lines.append(f"re-derivation: {r['n_gain_agree']}/{r['n_judged']} rows' rule-predicted gains match the "
                     f"next row ({r['gain_agree_pct']:.1f} %); {r['n_passed']} passes"
                     + (f"; {r['n_uncertain']} ANGLE_P_UP verdict(s) needed the next row (no ATDE)" if r["n_uncertain"] else "")
                     + (f"; disagreements at {', '.join(f'{t:.1f}' for t in r['disagreements'][:8])} s" if r["disagreements"] else ""))
    wit = "" if b["rate_p_observed"] is None else (
        f"; in-log witness (ANGLE rows RP / RATE_P_UP RP) x{b['rate_p_observed']:.3f} "
        + ("agrees" if b["agrees"] else "DISAGREES"))
    lines.append(f"backoff: {b['branch']} ({b['applied_by']}), rate x{b['rate_p']:.3f}, angle x{b['sp']:.3f}"
                 + (f", GMBK {b['gmbk']:.3f} from {b['gmbk_source']}" if b["gmbk"] is not None else
                    f", {b['gmbk_source']}")
                 + (" [assumed: " + b["note"] + "]" if not b["exact"] else "") + wit)
    f = s["final"]
    if f:
        lines.append(f"final: P {_fmt(f['rat_p'])} I {_fmt(f['rat_i'])} D {_fmt(f['rat_d'])} ANG_P {_fmt(f['ang_p'])} "
                     f"ACC {_fmt(f['acc_max_dps2'])} deg/s^2"
                     + (f" FLTE {_fmt(f['flte'])} Hz" if s["axis_id"] == 2 else "")
                     + f" (ddt of the last angle twitch {_fmt(s['found']['ddt_last_angle_cdss'])} cdeg/s^2"
                     + (", floor applied" if f["acc_floor_applied"] else "") + ")")
    if s["msg_gains"]:
        m = s["msg_gains"]
        a = s["agreement"]
        lines.append("MSG: " + ", ".join(f"{k} {_fmt(v)}" for k, v in m.items())
                     + (f" -> max diff {a['msg_max_pct']:.2f} %" + (" (within print rounding)" if a["msg_within_rounding"] else "")
                        if a["msg_max_pct"] is not None else ""))
    else:
        lines.append("MSG: no 'AutoTune: <axis> Rate:' line for this axis in the session")
    if s["saved_params"]:
        a = s["agreement"]
        lines.append(f"PARM: {s['saved_note']}: " + ", ".join(f"{k} {_fmt(v)}" for k, v in sorted(s["saved_params"].items()))
                     + (f" -> max diff {a['parm_max_pct']:.2f} %" if a["parm_max_pct"] is not None else ""))
    else:
        lines.append(f"PARM: {s['saved_note']}")
    if s["mismatch"]:
        lines.append("MISMATCH: " + s["mismatch_note"])
    elif s["backoff_warning"]:
        lines.append("WARNING: " + s["backoff_warning"])
    for ref in s["refusals"]:
        lines.append(ref.line() if hasattr(ref, "line") else str(ref))
    return lines


def to_rows(sessions):
    """(headers, rows) for the `autotune` Section table."""
    rows = []
    for s in sessions:
        f = s["final"] or {}
        a = s["agreement"]
        parts = []
        if a["msg_max_pct"] is not None:
            parts.append(f"MSG {a['msg_max_pct']:.2f} %")
        if a["parm_max_pct"] is not None:
            parts.append(f"PARM {a['parm_max_pct']:.2f} %")
        if s["mismatch"]:
            parts.append("MISMATCH")
        rows.append([s["log_name"], s["axis"], "/".join(st["step"] for st in s["steps"]), s["n_twitches"],
                     bool(s["complete"]), _finite(f.get("rat_p")), _finite(f.get("rat_i")), _finite(f.get("rat_d")),
                     _finite(f.get("ang_p")), _finite(f.get("acc_max_dps2")), ", ".join(parts) or "-"])
    return list(TABLE_HEADERS), rows
