"""WP4 of docs/pid-tuning-plan.md: AutoTune session reconstruction (dflog/tune_atun.py).

The truth is `tunesim.autotune`: `tests/tunesynth.autotune_log` writes the ATUN/ATDE/MSG/
EV/PARM records of a session the simulator ran, and returns the gains it saved. The
reconstruction must recover them from the log alone, name the firmware backoff it used,
notice when the MSG line disagrees, refuse an incomplete axis, and pool sessions.

    python tests/test_tune_atun.py
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

from dflog import Log                                                          # noqa: E402
from dflog.tune import Refusal                                                 # noqa: E402
from dflog.tunesim import Plant, AUTOTUNE                                      # noqa: E402
from dflog import tune_atun                                                    # noqa: E402
from dflog.tune_atun import (autotune_sessions, pool_sessions, describe, to_rows,   # noqa: E402
                             backoff_for, firmware_version, twitch_rule, BACKOFF_BY_FIRMWARE,
                             TABLE_HEADERS, session_to_dict)
from tunesynth import autotune_log, PLANT_5IN, GAINS_5IN                       # noqa: E402

TMP = tempfile.mkdtemp(prefix="dflog-tune-atun-")
_CACHE = {}


def _load(w, name):
    return Log(w.write(os.path.join(TMP, name)), use_cache=False)


def _roll_session():
    """The reference session, built once: a GMBK 0.25 roll AutoTune on the 5-inch plant."""
    if "roll" not in _CACHE:
        w, res = autotune_log(PLANT_5IN, GAINS_5IN, gmbk=0.25, axes=("roll",))
        _CACHE["roll"] = (w, res, _load(w, "atun_roll.bin"))
    return _CACHE["roll"]


def _rel(a, b):
    return abs(a - b) / abs(b)


# ------------------------------------------------------------- reconstruction

def test_reconstruction_reproduces_simulator_gains():
    w, res, log = _roll_session()
    r = res["roll"]
    sessions = autotune_sessions(log)
    assert len(sessions) == 1
    s = sessions[0]
    assert s["axis"] == "roll" and s["axis_id"] == 0 and s["session"] == 1 and s["n_sessions"] == 1
    assert s["complete"] is True and s["refusals"] == [] and s["failed"] is None
    assert [st["step"] for st in s["steps"]] == ["RATE_D_UP", "RATE_D_DOWN", "RATE_P_UP", "ANGLE_P_DOWN", "ANGLE_P_UP"]
    assert all(st["completed"] for st in s["steps"])
    assert s["n_twitches"] == sum(t["outcome"] == "UPDATE_GAINS" for t in r["twitches"])
    # the gains the simulator saved, to 0.1 %
    f = s["final"]
    for k in ("rat_p", "rat_i", "rat_d", "ang_p", "acc_max_dps2"):
        assert _rel(f[k], r[k]) < 1e-3, (k, f[k], r[k])
    assert f["flte"] == pytest.approx(GAINS_5IN["roll"]["flte"])
    # the backoff came from the log's own AUTOTUNE_GMBK, on the 4.7 branch, exactly
    b = s["backoff"]
    assert b["exact"] and b["gmbk"] == 0.25 and b["branch"] == "ArduPilot-4.7"
    assert b["rate_p"] == pytest.approx(0.75) and b["sp"] == pytest.approx(0.75 * (1 - 0.075))
    assert b["rate_p_observed"] == pytest.approx(0.75, rel=1e-5) and b["agrees"] is True
    assert "set_tuning_gains_with_backoff" in b["applied_by"] and "fetched" in b["source"]
    # the pre-backoff gains are the last RATE_P_UP row's RP/RD and the last ANGLE_P_UP row's SP
    found = r["gains_per_step"]
    assert s["found"]["rp"] == pytest.approx(found["RATE_P_UP"]["rp"], rel=1e-6)
    assert s["found"]["rd"] == pytest.approx(found["RATE_P_UP"]["rd"], rel=1e-6)
    assert s["found"]["sp"] == pytest.approx(found["ANGLE_P_UP"]["sp"], rel=1e-6)
    assert s["found"]["ddt_last_angle_cdss"] == pytest.approx(r["test_accel_max_cdss"], rel=1e-6)
    assert s["aggr"] == pytest.approx(0.075) and s["aggr_source"] == "AUTOTUNE_AGGR (PARM)" and s["version"] == [4, 7]
    assert s["initial"]["rat_p"] == pytest.approx(0.135) and s["defaulted"] == []


def test_msg_lines_agree_within_print_rounding():
    """The MSG text is %0.3f / %0.4f / %0.0f: agreement is asserted at half a printed unit,
    which is the honest tolerance (0.1 % of a D of 0.0033 is smaller than its last digit)."""
    w, res, log = _roll_session()
    r = res["roll"]
    s = autotune_sessions(log)[0]
    m = s["msg_gains"]
    assert set(m) == {"rat_p", "rat_i", "rat_d", "ang_p", "acc_max_cdss"}
    assert abs(s["final"]["rat_p"] - m["rat_p"]) <= 0.5e-3
    assert abs(s["final"]["rat_i"] - m["rat_i"]) <= 0.5e-3
    assert abs(s["final"]["rat_d"] - m["rat_d"]) <= 0.5e-4
    assert abs(s["final"]["ang_p"] - m["ang_p"]) <= 0.5e-3
    assert abs(s["final"]["acc_max_dps2"] * 100 - m["acc_max_cdss"]) <= 0.5
    a = s["agreement"]
    # D is printed %0.4f: a D near 0.0008 differs from its text by up to 6 % while being
    # exactly the printed number, so the honest criterion is 'within rounding', not a percent
    assert a["msg_within_rounding"] is True and s["mismatch"] is False
    assert all(v["within_rounding"] for v in a["msg"].values())
    assert a["msg"]["rat_p"]["pct"] == pytest.approx(0.0, abs=0.5) and a["msg"]["ang_p"]["pct"] == pytest.approx(0.0, abs=0.01)
    # the saved PARM values after EV 37 match the reconstruction to float32
    assert s["saved_params"]["rat_p"] == pytest.approx(r["rat_p"], rel=1e-6)
    assert s["saved_params"]["acc_max_dps2"] == pytest.approx(r["acc_max_dps2"], rel=1e-6)
    assert a["parm_max_pct"] < 1e-3 and "EV 37" in s["saved_note"]
    assert "EV 37 AUTOTUNE_SAVEDGAINS" in s["witnesses"] and "EV 33 AUTOTUNE_SUCCESS" in s["witnesses"]
    print("MSG max diff %.3f %% ; PARM max diff %.5f %%" % (a["msg_max_pct"], a["parm_max_pct"]))


def test_per_twitch_rederivation_matches_simulator():
    """Every ATUN row's re-derived verdict against the simulator's recorded `passed`, and
    the rule-predicted next gains against the next row. Reported exactly; >= 95 % required."""
    w, res, log = _roll_session()
    sim = [t for t in res["roll"]["twitches"] if t["outcome"] == "UPDATE_GAINS"]
    for carry in (True, False):
        s = autotune_sessions(log, ignore_next_carry=carry)[0]
        tws = s["twitches"]
        assert len(tws) == len(sim)
        agree = sum(a["passed"] == b["passed"] for a, b in zip(tws, sim))
        pct = 100.0 * agree / len(sim)
        r = s["rederivation"]
        print(f"ignore_next_carry={carry}: passed agrees on {agree}/{len(sim)} twitches ({pct:.1f} %); "
              f"gain prediction agrees on {r['n_gain_agree']}/{r['n_judged']} ({r['gain_agree_pct']:.1f} %)")
        assert pct >= 95.0, pct
        assert r["gain_agree_pct"] >= 95.0, r
        # the outcome vocabulary is the firmware's
        codes = {t["code"] for t in tws}
        assert codes <= {"success", "ignored", "fail-high", "fail-low", "bounce-too-small", "bounce-too-large", "limit"}
        assert "success" in codes and ("fail-low" in codes or "bounce-too-small" in codes)
        # ANGLE_P_UP verdicts used the ATDE rate extremes
        ups = [t for t in tws if t["step"] == "ANGLE_P_UP"]
        assert ups and all(t["rate_max"] is not None for t in ups) and r["n_uncertain"] == 0
    # the simulator resets ignore_next at step boundaries, the firmware does not: with
    # the firmware behaviour the disagreements can only sit on the first row of a step
    s = autotune_sessions(log, ignore_next_carry=True)[0]
    firsts = {st["t0"] for st in s["steps"]}
    assert set(s["rederivation"]["disagreements"]) <= firsts, s["rederivation"]["disagreements"]


def test_truncated_session_is_incomplete():
    w, res = autotune_log(PLANT_5IN, GAINS_5IN, axes=("roll",), truncate_after_step="RATE_P_UP")
    log = _load(w, "atun_trunc.bin")
    s = autotune_sessions(log)[0]
    assert s["complete"] is False and s["final"] is None and s["msg_gains"] == {}
    assert [st["step"] for st in s["steps"]] == ["RATE_D_UP", "RATE_D_DOWN", "RATE_P_UP"]
    assert s["steps"][0]["completed"] and s["steps"][1]["completed"]
    assert len(s["refusals"]) == 1
    ref = s["refusals"][0]
    assert isinstance(ref, Refusal) and ref.code == "AUTOTUNE_INCOMPLETE" and ref.axis == "roll"
    assert "RATE_D_UP, RATE_D_DOWN" in ref.message and "AUTOTUNE_AXES" in ref.fix
    assert "no EV 37" in s["saved_note"]
    assert to_rows([s])[1][0][4] is False
    lines = describe(s)
    assert lines[0].endswith("INCOMPLETE") and any("AUTOTUNE_INCOMPLETE" in ln for ln in lines)
    assert pool_sessions({"trunc.bin": [s]}) == {}


# --------------------------------------------------------------- branch table

def test_backoff_table_and_lookup():
    for v in ((4, 3), (4, 4), (4, 5), (4, 6)):
        row = BACKOFF_BY_FIRMWARE[v]
        assert row["gmbk"] is False and (row["rd_backoff"], row["rp_backoff"], row["sp_backoff"]) == (1.0, 1.0, 0.9)
        assert row["sp_aggr"] is False and "set_gains_post_tune" in row["applied_by"] and "raw.githubusercontent" in row["source"]
    assert BACKOFF_BY_FIRMWARE[(4, 7)]["gmbk"] is True and BACKOFF_BY_FIRMWARE[(4, 7)]["sp_aggr"] is True
    assert firmware_version("ArduCopter V4.7.1 (dbe79216)") == (4, 7)
    assert firmware_version("ArduCopter V4.5.7 (abcd1234)") == (4, 5)
    assert firmware_version("") is None
    # 4.x: fixed backoffs, no (1 - AGGR)
    b = backoff_for((4, 5), None, 0.075)
    assert (b["rate_p"], b["rate_d"], b["sp"]) == (1.0, 1.0, 0.9) and b["exact"] and b["gmbk"] is None
    assert b["branch"] == "Copter-4.5"
    # 4.7 with the parameter: (1 - GMBK), (1 - GMBK)(1 - AGGR)
    b = backoff_for((4, 7), 0.25, 0.075)
    assert b["rate_p"] == pytest.approx(0.75) and b["sp"] == pytest.approx(0.75 * 0.925) and b["exact"]
    # 4.7 without the parameter: the default is assumed and said so
    b = backoff_for((4, 7), None, 0.075)
    assert b["rate_p"] == pytest.approx(0.75) and not b["exact"] and "default" in b["note"]
    # unknown versions: GMBK in PARM wins, else the nearest known branch
    b = backoff_for(None, 0.3, 0.1)
    assert b["rate_p"] == pytest.approx(0.7) and not b["exact"] and "unknown" in b["note"]
    b = backoff_for(None, None, 0.1)
    assert (b["rate_p"], b["sp"]) == (1.0, 0.9) and not b["exact"] and "ArduPilot-4.6" in b["note"]
    b = backoff_for((4, 2), None, 0.1)
    assert b["branch"] == "Copter-4.3" and not b["exact"] and "nearest" in b["note"]
    b = backoff_for((5, 0), None, 0.1)
    assert b["branch"] == "ArduPilot-4.7" and b["rate_p"] == pytest.approx(0.75) and not b["exact"]
    b = backoff_for((5, 0), 0.25, 0.1)
    assert b["rate_p"] == pytest.approx(0.75) and b["exact"]
    # the parameter overrides a fixed-backoff branch, and says so
    b = backoff_for((4, 5), 0.25, 0.1)
    assert b["rate_p"] == pytest.approx(0.75) and not b["exact"] and "parameter wins" in b["note"]
    # GMBK is constrained 0-0.5 as the firmware does
    assert backoff_for((4, 7), 0.9, 0.1)["gmbk"] == 0.5


def test_4x_banner_log_uses_fixed_backoffs_and_the_witness_catches_the_table():
    """A log with a 4.5 banner and no AUTOTUNE_GMBK: the table says rate x1.0, angle P x0.9,
    no (1 - AGGR). `autotune_log(gmbk=None)` only omits the parameter - `gain_dict` refills
    None with the default, so the simulator still applied x0.75 (a WP2 quirk). The log's
    own ANGLE_P rows therefore witness x0.750 against the assumed x1.000: the rate gains
    (read from those rows) must still be right, angle P must be the table's x0.9 and be
    flagged as a mismatch that blames the branch table, not the MSG."""
    w, res = autotune_log(PLANT_5IN, GAINS_5IN, axes=("roll",), gmbk=None, banner="ArduCopter V4.5.7 (abcd1234)")
    log = _load(w, "atun_45.bin")
    assert "AUTOTUNE_GMBK" not in log.params()
    s = autotune_sessions(log)[0]
    r = res["roll"]
    b = s["backoff"]
    assert s["version"] == [4, 5] and b["branch"] == "Copter-4.5" and b["exact"] and b["gmbk"] is None
    assert (b["rate_p"], b["rate_d"], b["sp"]) == (1.0, 1.0, 0.9)
    assert b["rate_p_observed"] == pytest.approx(0.75, rel=1e-5) and b["agrees"] is False
    assert _rel(s["final"]["rat_p"], r["rat_p"]) < 1e-3 and _rel(s["final"]["rat_d"], r["rat_d"]) < 1e-3
    assert "ANGLE_P_UP rows" in s["final"]["rate_source"]
    assert s["final"]["ang_p"] == pytest.approx(s["found"]["sp"] * 0.9, rel=1e-6)
    assert s["final"]["ang_p"] == pytest.approx(r["ang_p"] * 0.9 / (0.75 * 0.925), rel=1e-6)
    assert s["mismatch"] is True
    note = s["mismatch_note"]
    assert "ang_p" in note and "rat_p" not in note and "rat_d" not in note
    assert "branch table is the more likely wrong" in note and "x0.750" in note and "x1.000" in note
    assert s["backoff_warning"] and any("DISAGREES" in ln for ln in describe(s))


def test_47_log_without_gmbk_parameter_names_the_assumption():
    """The 4.x-emulation log keeps the 4.7 banner: the reconstruction must assume the GMBK
    default, say so, and the in-log witness (x0.750, see the previous test) confirms it;
    the MSG then agrees and nothing is a mismatch, but `exact` stays False."""
    w, res = autotune_log(PLANT_5IN, GAINS_5IN, axes=("roll",), gmbk=None)
    log = _load(w, "atun_47_nogmbk.bin")
    s = autotune_sessions(log)[0]
    b = s["backoff"]
    assert b["branch"] == "ArduPilot-4.7" and not b["exact"] and b["gmbk"] == 0.25 and "default" in b["gmbk_source"]
    assert b["rate_p_observed"] == pytest.approx(0.75, rel=1e-5) and b["agrees"] is True
    assert s["mismatch"] is False and s["agreement"]["msg_within_rounding"] is True
    assert _rel(s["final"]["ang_p"], res["roll"]["ang_p"]) < 1e-3
    assert any("[assumed:" in ln and "agrees" in ln for ln in describe(s))
    # without a banner and without the parameter the fixed backoffs are assumed and said so
    w, res = autotune_log(PLANT_5IN, GAINS_5IN, axes=("roll",), gmbk=None, banner="no banner here")
    s = autotune_sessions(_load(w, "atun_nobanner.bin"))[0]
    assert s["version"] is None and s["backoff"]["branch"] == "ArduPilot-4.6" and not s["backoff"]["exact"]
    assert "unknown" in s["backoff"]["note"] and s["mismatch"] is True and "assumed" in s["mismatch_note"]


# --------------------------------------------------------------- disagreement

def test_edited_msg_line_is_a_mismatch_naming_both_numbers():
    """The same session with its MSG 'Rate: P:' value edited by +5 % in the file bytes."""
    w, res, _log = _roll_session()
    r = res["roll"]
    raw = w.bytes()
    old = f"Rate: P:{r['rat_p']:0.3f},".encode()
    new = f"Rate: P:{r['rat_p'] * 1.05:0.3f},".encode()
    assert len(old) == len(new) and raw.count(old) == 1
    path = os.path.join(TMP, "atun_msg_edited.bin")
    with open(path, "wb") as fh:
        fh.write(raw.replace(old, new))
    log = Log(path, use_cache=False)
    s = autotune_sessions(log)[0]
    assert s["complete"] and s["mismatch"] is True
    assert s["msg_gains"]["rat_p"] == pytest.approx(round(r["rat_p"] * 1.05, 3))
    note = s["mismatch_note"]
    assert f"{s['final']['rat_p']:.6g}" in note and f"{s['msg_gains']['rat_p']:.6g}" in note
    assert "rat_p" in note and "rat_i" not in note
    assert s["agreement"]["msg"]["rat_p"]["pct"] == pytest.approx(-100 * (1 - 1 / 1.05), abs=0.5)
    # the branch is known, GMBK is in PARM and the witness agrees: the MSG side is blamed
    assert "the MSG text" in note
    assert "MISMATCH" in to_rows([s])[1][0][-1]
    assert any(ln.startswith("MISMATCH") for ln in describe(s))


# ------------------------------------------------------------------ pooling

def test_two_logs_pool_to_median_and_spread():
    w1, res1, log1 = _roll_session()
    plant2 = dict(PLANT_5IN, roll=Plant(k=9000.0, tau1=0.22, tau2=0.015, delay=0.005))
    w2, res2 = autotune_log(plant2, GAINS_5IN, gmbk=0.25, axes=("roll",))
    log2 = _load(w2, "atun_roll_2.bin")
    s1, s2 = autotune_sessions(log1), autotune_sessions(log2)
    assert s1[0]["complete"] and s2[0]["complete"]
    pooled = pool_sessions({"a.bin": s1, "b.bin": s2})
    assert set(pooled) == {"roll"}
    assert set(pooled["roll"]) == {"rat_p", "rat_i", "rat_d", "ang_p", "acc_max_dps2", "flte"}
    p = pooled["roll"]["rat_p"]
    a, b = res1["roll"]["rat_p"], res2["roll"]["rat_p"]
    assert a != b
    assert p["median"] == pytest.approx((a + b) / 2, rel=1e-3) and p["n"] == 2 and p["n_logs"] == 2
    assert p["spread"] == pytest.approx(abs(a - b) / ((a + b) / 2), rel=1e-3)
    assert set(p["values_by_log"]) == {"a.bin", "b.bin"} and len(p["values_by_log"]["a.bin"]) == 1
    assert pooled["roll"]["flte"]["spread"] == 0.0 and pooled["roll"]["flte"]["median"] == 0.0
    # an incomplete session is left out of the pool
    w3, _ = autotune_log(PLANT_5IN, GAINS_5IN, axes=("roll",), truncate_after_step="RATE_D_DOWN")
    s3 = autotune_sessions(_load(w3, "atun_roll_3.bin"))
    assert pool_sessions({"a.bin": s1, "c.bin": s3})["roll"]["rat_p"]["n"] == 1
    assert pool_sessions({}) == {}


def test_two_axes_in_one_session():
    w, res = autotune_log(PLANT_5IN, GAINS_5IN, axes=("roll", "pitch"))
    log = _load(w, "atun_rp.bin")
    ss = autotune_sessions(log)
    assert [s["axis"] for s in ss] == ["roll", "pitch"] and all(s["session"] == 1 for s in ss)
    for s in ss:
        r = res[s["axis"]]
        assert s["complete"] and not s["mismatch"], describe(s)
        assert _rel(s["final"]["rat_p"], r["rat_p"]) < 1e-3 and _rel(s["final"]["ang_p"], r["ang_p"]) < 1e-3
        assert s["msg_gains"]["rat_p"] == pytest.approx(r["rat_p"], abs=0.5e-3)
    headers, rows = to_rows(ss)
    assert headers == TABLE_HEADERS and len(rows) == 2 and rows[1][1] == "pitch" and rows[1][4] is True
    assert rows[0][5] == pytest.approx(res["roll"]["rat_p"], rel=1e-3)


# ------------------------------------------------------------------ the rule

def test_twitch_rule_matches_sources_section_1_5():
    A = AUTOTUNE
    # RATE_D_UP: peak over target -> P down 5 %; under 80 % -> P up; bounce >= AGGR -> pass
    r = twitch_rule("RATE_D_UP", 0, 100.0, 90.0, 110.0, 0.2, 0.004, 4.5, 0, False, 0.075, 0.0005)
    assert r["code"] == "fail-high" and r["rp"] == pytest.approx(0.19) and r["rd"] == 0.004 and not r["passed"]
    r = twitch_rule("RATE_D_UP", 0, 100.0, 70.0, 75.0, 0.2, 0.004, 4.5, 0, False, 0.075, 0.0005)
    assert r["code"] == "fail-low" and r["rp"] == pytest.approx(0.21)
    r = twitch_rule("RATE_D_UP", 0, 100.0, 80.0, 95.0, 0.2, 0.004, 4.5, 2, False, 0.075, 0.0005)
    assert r["code"] == "success" and r["passed"] and r["success"] == 3 and r["ignore_next"] and r["rp"] == 0.2
    r = twitch_rule("RATE_D_UP", 0, 100.0, 92.0, 95.0, 0.2, 0.004, 4.5, 2, False, 0.075, 0.0005)
    assert r["code"] == "bounce-too-small" and r["rd"] == pytest.approx(0.0044) and r["success"] == 1
    r = twitch_rule("RATE_D_UP", 0, 100.0, 92.0, 95.0, 0.2, 0.004, 4.5, 2, True, 0.075, 0.0005)
    assert r["code"] == "ignored" and r["rd"] == 0.004 and not r["ignore_next"] and r["success"] == 2
    # RATE_D_DOWN: no bounce -> pass; bounce -> D down 5 %; D floor forces completion
    r = twitch_rule("RATE_D_DOWN", 0, 100.0, 92.0, 95.0, 0.2, 0.004, 4.5, 0, False, 0.075, 0.0005)
    assert r["code"] == "success" and r["passed"]
    r = twitch_rule("RATE_D_DOWN", 0, 100.0, 80.0, 95.0, 0.2, 0.004, 4.5, 0, False, 0.075, 0.0005)
    assert r["code"] == "bounce-too-large" and r["rd"] == pytest.approx(0.0038)
    r = twitch_rule("RATE_D_DOWN", 0, 100.0, 80.0, 95.0, 0.2, 0.0005, 4.5, 0, False, 0.075, 0.0005)
    assert r["code"] == "limit" and r["success"] == A["SUCCESS_COUNT"] and r["limit"]
    # RATE_P_UP: overshoot >= 0.5 AGGR -> pass; undershoot with bounce -> D and P down; else P up
    r = twitch_rule("RATE_P_UP", 0, 100.0, 90.0, 104.0, 0.2, 0.004, 4.5, 0, False, 0.075, 0.0005)
    assert r["code"] == "success"
    r = twitch_rule("RATE_P_UP", 0, 100.0, 80.0, 95.0, 0.2, 0.004, 4.5, 0, False, 0.075, 0.0005)
    assert r["code"] == "bounce-too-large" and r["rd"] == pytest.approx(0.0038) and r["rp"] == pytest.approx(0.19)
    r = twitch_rule("RATE_P_UP", 0, 100.0, 95.0, 101.0, 0.2, 0.004, 4.5, 0, False, 0.075, 0.0005)
    assert r["code"] == "fail-low" and r["rp"] == pytest.approx(0.21)
    # yaw(E): RD is FLTE with the 1-5 Hz limits, and the D floor does not fail the tune
    r = twitch_rule("RATE_P_UP", 2, 100.0, 80.0, 95.0, 0.2, 1.0, 4.5, 0, False, 0.075, 0.0005)
    assert r["code"] == "fail-low" and r["rd"] == 1.0          # rd > d_min is false at the floor
    # ANGLE_P_DOWN / UP with the 0.5 AGGR overshoot allowance; the ATDE clause on UP
    r = twitch_rule("ANGLE_P_DOWN", 0, 20.0, 19.0, 20.5, 0.2, 0.004, 4.5, 0, False, 0.075, 0.0005)
    assert r["code"] == "success"
    r = twitch_rule("ANGLE_P_DOWN", 0, 20.0, 19.0, 21.0, 0.2, 0.004, 4.5, 0, False, 0.075, 0.0005)
    assert r["code"] == "fail-high" and r["sp"] == pytest.approx(4.275)
    r = twitch_rule("ANGLE_P_UP", 0, 20.0, 19.0, 20.5, 0.2, 0.004, 4.5, 0, False, 0.075, 0.0005)
    assert r["code"] == "fail-low" and r["sp"] == pytest.approx(4.725) and r["uncertain"]
    r = twitch_rule("ANGLE_P_UP", 0, 20.0, 19.0, 20.5, 0.2, 0.004, 4.5, 0, False, 0.075, 0.0005,
                    rate_min=-20.0, rate_max=100.0)
    assert r["code"] == "success" and "ATDE" in r["text"] and not r["uncertain"]
    r = twitch_rule("ANGLE_P_UP", 0, 20.0, 19.0, 20.5, 0.2, 0.004, 4.5, 0, False, 0.075, 0.0005,
                    rate_min=-2.0, rate_max=100.0)
    assert r["code"] == "fail-low" and not r["uncertain"]
    r = twitch_rule("ANGLE_P_UP", 0, 20.0, 19.0, 22.0, 0.2, 0.004, 4.5, 3, False, 0.075, 0.0005)
    assert r["code"] == "success" and r["success"] == 4


# ------------------------------------------------------------- determinism

def test_deterministic_and_json_safe():
    w, res, log = _roll_session()
    a = autotune_sessions(log)
    b = autotune_sessions(Log(os.path.join(TMP, "atun_roll.bin"), use_cache=False))
    assert [session_to_dict(s) for s in a] == [session_to_dict(s) for s in b]
    assert describe(a[0]) == describe(b[0]) and to_rows(a) == to_rows(b)
    d = session_to_dict(a[0])
    assert isinstance(d["refusals"], list)

    def walk(x):
        if isinstance(x, dict):
            for v in x.values():
                walk(v)
        elif isinstance(x, (list, tuple)):
            for v in x:
                walk(v)
        elif isinstance(x, float):
            assert np.isfinite(x)
        else:
            assert x is None or isinstance(x, (str, bool, int)), type(x)
    walk(d)
    text = "\n".join(describe(a[0]))
    for word in ("session 1 of 1", "RATE_D_UP", "backoff: ArduPilot-4.7", "final: P", "MSG:", "PARM:", "re-derivation"):
        assert word in text, text
    assert "MISMATCH" not in text


def test_log_without_atun_gives_nothing():
    from tunesynth import fast_log
    log = _load(fast_log(PLANT_5IN, GAINS_5IN, seconds=5.0), "fast_noatun.bin")
    assert autotune_sessions(log) == []
    assert pool_sessions({"x": []}) == {}
    assert to_rows([]) == (TABLE_HEADERS, [])
    assert "session_gap_s" in tune_atun.ATUN_CONSTANTS and tune_atun.SESSION_GAP_S == 120.0


if __name__ == "__main__":
    import _shim
    sys.exit(_shim.run(sys.modules[__name__]))
