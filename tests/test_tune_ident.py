"""WP5 of docs/pid-tuning-plan.md: plant identification, margins, heli ceilings and the
virtual AutoTune (dflog/tune_ident.py).

Ground truth is the simulator: `tunesynth.fast_log` writes a log whose plant
(`PLANT_5IN["roll"]`) and controller are known exactly, WP1's `extract_axes` turns it
into `AxisSignals`, and every number tier B produces is checked against
`Plant.freq_response`, an independent scipy evaluation of the open loop, the closed loop
measured by `tunesim.ClosedLoop`, and `tunesim.autotune` on the true plant.

    python tests/test_tune_ident.py
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

from scipy.signal import TransferFunction, dlti                                   # noqa: E402

from dflog import Log, airborne_window                                            # noqa: E402
from dflog.checks import T                                                        # noqa: E402
from dflog.tune import GainSet, PlantModel, Refusal, Ceiling, extract_axes        # noqa: E402
from dflog import tune_ident as ti                                                # noqa: E402
from dflog import tunesim                                                         # noqa: E402
from tunesynth import fast_log, PLANT_5IN, GAINS_5IN                              # noqa: E402

TMP = tempfile.mkdtemp(prefix="dflog-tuneident-")
TRUTH = PLANT_5IN["roll"]
LOOP_HZ = 400.0
_CACHE = {}


def _signals(name, **kw):
    """Roll `AxisSignals` of a synthetic 5-inch log, built once per configuration."""
    if name not in _CACHE:
        w = fast_log(PLANT_5IN, GAINS_5IN, seconds=kw.pop("seconds", 120.0), seed=kw.pop("seed", 1), **kw)
        log = Log(w.write(os.path.join(TMP, name + ".bin")), use_cache=False)
        assert log.diagnostics.ok, log.diagnostics.render()
        sigs, refusals = extract_axes(log, airborne_window(log), axes=("roll",))
        assert sigs and not refusals, [r.line() for r in refusals]
        _CACHE[name] = sigs
    return _CACHE[name]


def _gains():
    return GainSet(axis="roll", **GAINS_5IN["roll"])


def _true_model():
    return ti.model_from_params(TRUTH.k, TRUTH.tau1, TRUTH.tau2, TRUTH.delay, loop_hz=LOOP_HZ)


def _true_G(f):
    """What the loop-rate controller sees of the true plant: G(s) x ZOH."""
    return TRUTH.freq_response(f) * ti.zoh_response(f, LOOP_HZ)


def _compare_band(name, est):
    """Max |dB| and |deg| error of a non-parametric G against the truth over `band`."""
    f, G, band = est["freqs"], est["G"], est["band"]
    m = (f >= band[0]) & (f <= band[1]) & np.isfinite(G)
    dmag = 20.0 * np.log10(np.abs(G[m]) / np.abs(_true_G(f[m])))
    dph = np.degrees(np.angle(G[m] / _true_G(f[m])))
    print(f"  [{name}] coherent band {band[0]:.1f}-{band[1]:.1f} Hz ({m.sum()} bins, {est['n_avg']} averages): "
          f"|G| error max {np.abs(dmag).max():.2f} dB, phase error max {np.abs(dph).max():.1f} deg")
    return float(np.abs(dmag).max()), float(np.abs(dph).max()), band


# ---------------------------------------------------------------- non-parametric estimate

def test_joint_io_recovers_true_plant_on_steps():
    """Shaped stick steps: the joint I/O G matches Plant.freq_response x ZOH within
    1 dB / 10 deg over the coherent band, which reaches past the loop crossover."""
    est, skipped = ti.segment_spectra(_signals("steps", stick="steps"))
    assert len(est) == 1 and not skipped
    dmag, dph, band = _compare_band("steps", est[0])
    assert dmag < 1.0 and dph < 10.0, (dmag, dph)
    assert band[0] <= 0.5 and band[1] >= 30.0, band                  # 0.5-33.5 Hz measured
    assert est[0]["n_avg"] == 119                                    # 120 s / 1 s hop - 1


def test_joint_io_on_chirp_is_exact_inside_the_sweep_and_biased_at_its_edge():
    """The 0.05-5 Hz chirp: inside the swept range G is within 1 dB / 10 deg; the bins
    just above the stop frequency are still *coherent* (window leakage of the sweep's
    end) but biased by several dB - coherence is necessary, not sufficient. The band
    (0.5-7.5 Hz) stops far below the 23 Hz crossover, so identify() refuses."""
    est, _ = ti.segment_spectra(_signals("chirp", stick="chirp", axes=("roll",)))
    e = est[0]
    f, G, coh = e["freqs"], e["G"], e["coh"]
    band = e["band"]
    print(f"  [chirp] coherent band {band[0]:.1f}-{band[1]:.1f} Hz")
    assert band[0] <= 0.5 and 5.0 <= band[1] <= 10.0, band
    inside = (f >= 0.5) & (f <= 5.0)
    dmag = np.abs(20.0 * np.log10(np.abs(G[inside]) / np.abs(_true_G(f[inside]))))
    dph = np.abs(np.degrees(np.angle(G[inside] / _true_G(f[inside]))))
    print(f"  [chirp] inside 0.5-5 Hz: |G| error max {dmag.max():.2f} dB, phase error max {dph.max():.1f} deg")
    assert dmag.max() < 1.0 and dph.max() < 10.0
    edge = (f > 6.0) & (f <= band[1])
    dmag_edge = np.abs(20.0 * np.log10(np.abs(G[edge]) / np.abs(_true_G(f[edge]))))
    print(f"  [chirp] leakage bins {f[edge].min():.1f}-{f[edge].max():.1f} Hz: coherence >= {coh[edge].min():.2f} "
          f"yet |G| error up to {dmag_edge.max():.1f} dB")
    assert coh[edge].min() >= T["tune_coherence"]["fail"] and dmag_edge.max() > 2.0
    r = ti.identify(_signals("chirp", stick="chirp", axes=("roll",)))
    assert isinstance(r, Refusal) and r.code == "NO_COHERENCE", r
    assert "does not reach the loop crossover" in r.message and f"{band[1]:.1f} Hz" in r.message, r.message
    assert "SID_AXIS" in r.fix and r.axis == "roll"


# ------------------------------------------------------------------------ parametric fit

def test_parametric_fit_recovers_k_tau1_delay():
    """The fit recovers k, tau1 and the delay within 15 % (band starts at 0.5 Hz, below
    the 0.64 Hz tau1 pole, so tau1 is observable; k/tau1 is pinned tighter still)."""
    pm = ti.identify(_signals("steps", stick="steps"))
    assert isinstance(pm, PlantModel), pm
    errs = dict(k=pm.k / TRUTH.k - 1, tau1=pm.tau1 / TRUTH.tau1 - 1, tau2=pm.tau2 / TRUTH.tau2 - 1,
                delay=pm.delay / TRUTH.delay - 1, k_over_tau1=(pm.k / pm.tau1) / (TRUTH.k / TRUTH.tau1) - 1)
    print("  [fit] " + ", ".join(f"{k} {v:+.1%}" for k, v in errs.items())
          + f"; rms {pm.fit_rms_db:.2f} dB / {pm.fit_rms_deg:.1f} deg; coh mean {pm.coh_mean_band:.3f}; "
          f"eps {pm.eps_mag_at_crossover:.3f}")
    for k in ("k", "tau1", "delay"):
        assert abs(errs[k]) < 0.15, (k, errs[k])
    assert abs(errs["k_over_tau1"]) < 0.05
    assert abs(errs["tau2"]) < 0.25                                 # the fast pole is the least observable
    assert pm.fit_rms_db < 0.5 and pm.fit_rms_deg < 3.0
    assert pm.band[0] == 0.5 and pm.band[1] >= 30.0
    assert 0.9 < pm.coh_mean_band <= 1.0 and pm.n_avg == 119
    assert 0.0 < pm.eps_mag_at_crossover < 0.05
    assert len(pm.freqs) == len(pm.G) == len(pm.coh) and pm.freqs[-1] == 200.0


def test_no_excitation_and_no_coherence_refuse_rather_than_fit():
    """Noise x10: the band stops short of the crossover -> NO_COHERENCE. Hover: the
    coherence is ~1.0 (the reference is the angle loop's reaction to the noise, not an
    input) so the excitation gate, not coherence, refuses -> NO_EXCITATION."""
    r = ti.identify(_signals("noise10", stick="steps", noise_dps=3.0))
    assert isinstance(r, Refusal) and r.code == "NO_COHERENCE", r
    assert "does not reach the loop crossover" in r.message, r.message
    print(f"  [noise x10] {r.message}")
    hover = _signals("hover", stick="hover", seconds=60.0)
    est, _ = ti.segment_spectra(hover)
    f, coh = est[0]["freqs"], est[0]["coh"]
    print(f"  [hover] mean coherence 0.5-40 Hz = {coh[(f > 0) & (f <= 40)].mean():.3f} with no stick at all")
    assert coh[(f > 0) & (f <= 40)].mean() > 0.9                    # the deceptive case
    r = ti.identify(hover)
    assert isinstance(r, Refusal) and r.code == "NO_EXCITATION", r
    assert "biased toward -1/C" in r.message and "tuning profile" in r.fix
    assert ti.excitation_frames(hover[0]) == 0
    assert ti.excitation_frames(_signals("steps", stick="steps")[0]) > T["tune_min_frames"]["warn"]


def test_band_rule_tolerates_a_single_dip_only():
    f = np.arange(0, 5.0, 0.5)
    coh = np.array([1.0, 1.0, 1.0, 0.5, 1.0, 1.0, 0.5, 0.5, 1.0, 1.0])
    assert ti._band(f, coh, 0.6) == (0.5, 2.5)
    assert ti._band(f, np.zeros(10), 0.6) == (None, None)
    assert ti._band(f, np.ones(10), 0.6) == (0.5, 4.5)              # the 0 Hz bin never counts


def test_sign_mismatch_is_refused_not_flipped():
    sigs = _signals("steps", stick="steps")
    s = sigs[0]
    import dataclasses
    flipped = dataclasses.replace(s, out=-s.out)
    r = ti.identify([flipped])
    assert isinstance(r, Refusal) and r.code == "SIGN_MISMATCH", r
    assert "nothing was flipped" in r.message


# ------------------------------------------------------------------- controller, margins

def _scipy_open_loop(g, f):
    """L = C G on the true plant by an independent route: scipy TransferFunction for the
    rational plant, dlti for the discrete controller pieces, explicit delay and ZOH."""
    w = 2.0 * math.pi * f
    Tn = 1.0 / LOOP_HZ
    _, Gr = TransferFunction([TRUTH.k], [TRUTH.tau1 * TRUTH.tau2, TRUTH.tau1 + TRUTH.tau2, 1.0]).freqresp(w=w)
    Gr = Gr * np.exp(-1j * w * TRUTH.delay) * (1.0 - np.exp(-1j * w * Tn)) / (1j * w * Tn)
    a_e = tunesim.calc_lowpass_alpha_dt(Tn, g["flte"])
    a_d = tunesim.calc_lowpass_alpha_dt(Tn, g["fltd"])
    # y[n] = (1-a) y[n-1] + a x[n]  ->  a z / (z - (1-a)); the numerator needs the z
    # (`[a]` alone would be a/(z - (1-a)), one loop late)
    _, HE = dlti([a_e, 0.0], [1.0, -(1.0 - a_e)], dt=Tn).freqresp(w=w * Tn)
    _, HD = dlti([a_d, 0.0], [1.0, -(1.0 - a_d)], dt=Tn).freqresp(w=w * Tn)
    _, I = dlti([g["rat_i"] * Tn, 0.0], [1.0, -1.0], dt=Tn).freqresp(w=w * Tn)
    _, D = dlti([g["rat_d"] / Tn, -g["rat_d"] / Tn], [1.0, 0.0], dt=Tn).freqresp(w=w * Tn)
    C = (math.pi / 180.0) * HE * (g["rat_p"] + I + D * HD)
    return C * Gr


def test_margins_match_an_independent_scipy_evaluation():
    g = GAINS_5IN["roll"]
    m = ti.margins(_true_model(), _gains())
    f = np.linspace(0.1, LOOP_HZ / 2.0, 40000)
    L = _scipy_open_loop(g, f)
    mag, ph = np.abs(L), np.degrees(np.unwrap(np.angle(L)))
    i = np.flatnonzero((mag[:-1] >= 1.0) & (mag[1:] < 1.0))[0]
    fc = f[i] + (f[i + 1] - f[i]) * (mag[i] - 1.0) / (mag[i] - mag[i + 1])
    pm = 180.0 + np.interp(fc, f, ph)
    j = np.flatnonzero((ph[:-1] >= -180.0) & (ph[1:] < -180.0))[0]
    f180 = f[j] + (f[j + 1] - f[j]) * (ph[j] + 180.0) / (ph[j] - ph[j + 1])
    gm = -20.0 * math.log10(np.interp(f180, f, mag))
    print(f"  [margins] tune_ident: fc {m['fc_hz']:.2f} Hz, PM {m['pm_deg']:.1f} deg, GM {m['gm_db']:.2f} dB at "
          f"{m['f180_hz']:.2f} Hz; scipy route: fc {fc:.2f}, PM {pm:.1f}, GM {gm:.2f} at {f180:.2f}")
    assert m["fc_hz"] == pytest.approx(fc, rel=0.01)
    assert m["pm_deg"] == pytest.approx(pm, abs=3.0)
    assert m["gm_db"] == pytest.approx(gm, abs=0.5)
    assert m["f180_hz"] == pytest.approx(f180, rel=0.01)
    # controller_response is the same C the scipy route built
    fx = np.array([1.0, 5.0, 20.0, 60.0])
    C_mine = ti.controller_response(_gains(), fx)
    C_scipy = _scipy_open_loop(g, fx) / (TRUTH.freq_response(fx) * ti.zoh_response(fx, LOOP_HZ))
    assert np.allclose(C_mine, C_scipy, rtol=1e-9)
    assert not np.isfinite(ti.controller_response(_gains(), [0.0])[0])          # the integrator
    # the identified plant's margins agree with the truth's, on both the fit and the
    # non-parametric G (which needs no ZOH correction: the samples are what the loop saw)
    pm_id = ti.identify(_signals("steps", stick="steps"))
    mi = ti.margins(pm_id, _gains())
    print(f"  [margins] identified fit: PM {mi['pm_deg']:.1f} deg, GM {mi['gm_db']:.2f} dB; non-parametric in band: "
          f"PM {mi['np_pm_deg']:.1f} deg, GM {mi['np_gm_db']:.2f} dB (truth PM {m['pm_deg']:.1f}, GM {m['gm_db']:.2f})")
    assert mi["np_in_band"] and mi["np_pm_deg"] == pytest.approx(m["pm_deg"], abs=1.0)
    assert mi["np_gm_db"] == pytest.approx(m["gm_db"], abs=0.3)
    assert mi["pm_deg"] == pytest.approx(m["pm_deg"], abs=3.0) and mi["gm_db"] == pytest.approx(m["gm_db"], abs=0.5)
    assert mi["band"] == pm_id.band and mi["loop_hz"] == LOOP_HZ


def _measured_closed_loop_gain(f_hz, seconds=4.0, amp=20.0):
    """|Act| / |Tar| at f_hz from a ClosedLoop run driven by a sine on the rate target,
    least-squares sine fit over the second half."""
    loop = tunesim.ClosedLoop(TRUTH.copy(), GAINS_5IN["roll"], LOOP_HZ)
    n = int(seconds * LOOP_HZ)
    t = np.arange(n) / LOOP_HZ
    res = loop.run(n, rate_cmd=amp * np.sin(2.0 * math.pi * f_hz * t))
    half = n // 2
    A = np.column_stack([np.sin(2 * math.pi * f_hz * t[half:]), np.cos(2 * math.pi * f_hz * t[half:])])
    amp_of = lambda x: float(np.hypot(*np.linalg.lstsq(A, x[half:], rcond=None)[0]))
    return amp_of(res["act"]) / amp_of(res["tar"])


def test_controller_times_plant_predicts_the_measured_closed_loop():
    """|T| = |L / (1 + L)| from controller_response x the true plant (with ZOH) against
    the closed loop tunesim actually runs, at three frequencies spanning the crossover."""
    g = _gains()
    for f_hz in (2.0, 8.0, 20.0):
        L = ti.controller_response(g, np.array([f_hz]))[0] * _true_G(np.array([f_hz]))[0]
        pred = abs(L / (1.0 + L))
        meas = _measured_closed_loop_gain(f_hz)
        print(f"  [closed loop] {f_hz:4.1f} Hz: predicted |T| {pred:.4f}, measured {meas:.4f} ({meas / pred - 1:+.2%})")
        assert meas == pytest.approx(pred, rel=0.03), (f_hz, pred, meas)


def test_model_step_matches_the_simulator_step():
    st = ti.closed_loop_step_from_model(_true_model(), _gains(), seconds=0.5)
    assert len(st["t"]) == 200 and st["t"][1] - st["t"][0] == pytest.approx(1.0 / LOOP_HZ)
    assert np.all(st["tar"] == 1.0) and "FLTT bypassed" in st["method"]
    t, act = TRUTH.true_closed_loop_step(dict(GAINS_5IN["roll"], fltt=0.0), seconds=0.5)
    assert np.allclose(st["act"], act)
    assert 0.9 < act.max() < 1.6 and abs(act[-1] - 1.0) < 0.15


# ------------------------------------------------------------------- ceilings, autotune

def test_virtual_autotune_matches_the_engine_and_the_identified_plant():
    g = _gains()
    direct = tunesim.autotune(TRUTH, g, axis="roll")
    va = ti.virtual_autotune(_true_model(), g)
    for k in ("rat_p", "rat_i", "rat_d", "ang_p", "acc_max_dps2", "n_twitches", "steps_completed", "final_overshoot",
              "final_bounce"):
        assert va[k] == direct[k], k
    assert va["method"] == "virtual-autotune" and va["complete"] and va["plant"]["delay_samples"] == 2
    assert "twitches" in va["why"] and "final overshoot" in va["why"] and "final bounce" in va["why"]
    assert va["constants"]["aggr"] == 0.075 and va["constants"]["thst_hover"] == 0.25
    pm = ti.identify(_signals("steps", stick="steps"))
    vi = ti.virtual_autotune(pm, g)
    rel = {k: vi[k] / direct[k] - 1 for k in ("rat_p", "rat_d", "ang_p", "acc_max_dps2")}
    print(f"  [virtual autotune] true plant: P {direct['rat_p']:.4f} D {direct['rat_d']:.5f} ANG_P {direct['ang_p']:.2f} "
          f"ACC {direct['acc_max_dps2']:.0f}; identified plant: P {vi['rat_p']:.4f} D {vi['rat_d']:.5f} "
          f"ANG_P {vi['ang_p']:.2f} ACC {vi['acc_max_dps2']:.0f}; " + ", ".join(f"{k} {v:+.1%}" for k, v in rel.items()))
    assert vi["complete"], vi["aborted"]
    for k in ("rat_p", "rat_d", "ang_p"):
        assert abs(rel[k]) < 0.25, (k, rel[k])
    assert vi["rat_i"] == vi["rat_p"]


def test_aggr_raises_d_and_overrides_are_reported():
    g = _gains()
    lo = ti.virtual_autotune(_true_model(), g, aggr=0.05)
    hi = ti.virtual_autotune(_true_model(), g, aggr=0.10)
    assert hi["rat_d"] > lo["rat_d"], (lo["rat_d"], hi["rat_d"])
    assert lo["constants"]["aggr"] == 0.05 and "vs AGGR 0.050" in lo["why"]
    nb = ti.virtual_autotune(_true_model(), g, gmbk=0.0)
    assert nb["rat_p"] == pytest.approx(ti.virtual_autotune(_true_model(), g)["rat_p"] / 0.75)


def test_heli_ceilings_exceed_the_virtual_autotune_gains():
    g = _gains()
    pm = ti.identify(_signals("steps", stick="steps"))
    cs = ti.ceilings_from_plant(pm, g)
    assert [c.param for c in cs] == ["ATC_RAT_RLL_P", "ATC_RAT_RLL_D"]
    va = ti.virtual_autotune(pm, g)
    byp = {c.param: c for c in cs}
    p, d = byp["ATC_RAT_RLL_P"], byp["ATC_RAT_RLL_D"]
    print(f"  [ceilings] max P {p.value:.4f} at {p.evidence['f_hz']:.1f} Hz (-161 deg), max D {d.value:.5f} at "
          f"{d.evidence['f_hz']:.1f} Hz (-251 deg); virtual AutoTune P {va['rat_p']:.4f}, D {va['rat_d']:.5f}; "
          f"non-parametric max P {p.evidence['np_value']:.4f}")
    assert p.value > va["rat_p"] and d.value > va["rat_d"]
    assert isinstance(p, Ceiling) and p.method == "heli-autotune-phase-161" and d.method == "heli-autotune-phase-251"
    assert p.evidence["phase_deg"] == -161.0 and d.evidence["phase_deg"] == -251.0
    assert not p.evidence["capped"] and p.evidence["cap"] == 4.0 and d.evidence["cap"] == 0.4
    # the formula, re-done by hand from the evidence
    assert p.value == pytest.approx(10 ** (-(20 * math.log10(p.evidence["gain_used"]) + 2.42) / 20))
    assert d.evidence["gain_used"] == pytest.approx(d.evidence["gain_rad_s_per_unit"] * d.evidence["w_rad_s"])
    assert p.evidence["np_value"] == pytest.approx(p.value, rel=0.05)
    assert d.evidence["np_value"] is None                            # -251 deg lies above the coherent band
    # a very slow plant never reaches -251 deg below Nyquist: no D ceiling, no crash
    slow = ti.model_from_params(100.0, 5.0, 0.001, 0.0, loop_hz=LOOP_HZ)
    assert [c.param for c in ti.ceilings_from_plant(slow, g)] == ["ATC_RAT_RLL_P"]


BRISKET_ROLL = (6.557e4, 5.0, 0.0393, 0.01343)     # the roll fit of brisket-t1.bin


def _brisket_gains():
    return GainSet(axis="roll", **dict(GAINS_5IN["roll"], rat_p=0.135, rat_i=0.135, rat_d=0.0036,
                                       fltd=21.0, fltt=21.0, gyro_filter=42.0))


def test_margin_ceiling_is_the_largest_gain_keeping_the_margins():
    # The heli -161 deg P rule is P-only. On the Brisket roll plant it put the ceiling at
    # ~0.138 while the flown P 0.135 keeps PM ~48.6 deg: D's phase lead is ignored. The
    # margin ceiling evaluates the whole C(z) G loop instead.
    plant = ti.model_from_params(*BRISKET_ROLL, loop_hz=LOOP_HZ)
    g = _brisket_gains()
    pm_min, gm_min = ti.margin_limits()
    assert (pm_min, gm_min) == (T["tune_phase_margin_deg"]["warn"], T["tune_gain_margin_db"]["warn"])
    cur = ti.margins(plant, g)
    assert cur["pm_deg"] == pytest.approx(48.6, abs=0.5) and cur["gm_db"] == pytest.approx(8.9, abs=0.3)
    heli_p = {c.param: c for c in ti.ceilings_from_plant(plant, g)}["ATC_RAT_RLL_P"].value
    assert heli_p < 0.135 * 1.05 and ti.meets_margins(cur)          # the heli rule sits on a loop with margin
    cs, notes = ti.margin_ceilings(plant, g, d_for_p=0.004448, p_for_d=0.1955)
    assert notes == []
    byp = {c.param: c for c in cs}
    p, d = byp["ATC_RAT_RLL_P"], byp["ATC_RAT_RLL_D"]
    print(f"  [margin ceilings] P {p.value:.4f} (PM {p.evidence['pm_deg']:.2f}, GM {p.evidence['gm_db']:.2f}) "
          f"D {d.value:.5f} (P held {d.evidence['held']['rat_p']:.4f}); heli P {heli_p:.4f}")
    assert p.includes_margin and d.includes_margin and p.method == "margin-45deg-6dB"
    assert p.value == pytest.approx(0.1528, rel=0.01)
    assert p.evidence["pm_deg"] == pytest.approx(pm_min, abs=0.1) and p.evidence["gm_db"] >= gm_min
    assert p.evidence["held"]["rat_d"] == 0.004448
    # just above the ceiling fails, with I following P at the log's ratio
    import dataclasses
    above = dataclasses.replace(g, rat_p=p.value * 1.01, rat_i=p.value * 1.01, rat_d=0.004448)
    assert not ti.meets_margins(ti.margins(plant, above))
    # D is scanned at the P it would fly with: the virtual AutoTune's, lowered to the P ceiling
    assert d.evidence["held"]["rat_p"] == pytest.approx(p.value)
    assert d.evidence["pm_deg"] >= pm_min - 0.1 and d.evidence["gm_db"] >= gm_min - 0.05
    # a plant no gain in the span can hold: no ceiling, a note says why
    bad = ti.model_from_params(6.557e5, 5.0, 0.0393, 0.045, loop_hz=LOOP_HZ)
    cs2, notes2 = ti.margin_ceilings(bad, g)
    assert not any(c.param == "ATC_RAT_RLL_P" for c in cs2) and any("no value" in n for n in notes2)


# ------------------------------------------------------------------- report, determinism

def test_describe_and_rows_are_deterministic_strings():
    pm = ti.identify(_signals("steps", stick="steps"))
    m = ti.margins(pm, _gains())
    lines = ti.describe(pm, m)
    assert len(lines) == 5 and lines[0].startswith("plant fit: k = ") and "phase margin" in lines[3]
    assert "measured, in band" in lines[4]
    h, rows = ti.plant_rows(pm)
    assert h == ["quantity", "value", "unit", "note"] and [r[0] for r in rows][:4] == ["k", "tau1", "tau2", "delay"]
    h2, rows2 = ti.margin_rows(m)
    assert len(h2) == 5 and len(rows2) == 2 and rows2[0][0].startswith("fit")
    for line in lines:
        assert "\n" not in line and "nan" not in line.lower()


def test_identification_is_deterministic():
    a = ti.identify(_signals("steps", stick="steps"))
    b = ti.identify(_signals("steps", stick="steps"))
    assert np.array_equal(a.G, b.G, equal_nan=True) and np.array_equal(a.coh, b.coh)
    assert (a.k, a.tau1, a.tau2, a.delay, a.fit_rms_db, a.fit_rms_deg) == (b.k, b.tau1, b.tau2, b.delay, b.fit_rms_db, b.fit_rms_deg)
    g = _gains()
    x, y = ti.virtual_autotune(a, g), ti.virtual_autotune(b, g)
    assert x["rat_p"] == y["rat_p"] and x["rat_d"] == y["rat_d"] and x["ang_p"] == y["ang_p"]
    assert ti.describe(a, ti.margins(a, g)) == ti.describe(b, ti.margins(b, g))


def test_pooling_two_logs_of_one_plant():
    """Segments from different logs (different seeds) pool into one estimate at least as
    good as either alone on k, and n_avg adds."""
    s1 = _signals("steps", stick="steps")
    s2 = _signals("steps2", stick="steps", seed=2)
    pm = ti.identify(s1 + s2)
    assert isinstance(pm, PlantModel) and pm.n_avg == 238
    err = abs(pm.k / TRUTH.k - 1)
    print(f"  [pooled 2 logs] k {pm.k:.0f} ({pm.k / TRUTH.k - 1:+.1%}), tau1 {pm.tau1:.4f}, delay {pm.delay * 1e3:.2f} ms, "
          f"band {pm.band[0]:.1f}-{pm.band[1]:.1f} Hz, rms {pm.fit_rms_db:.2f} dB")
    assert err < 0.10 and abs(pm.tau1 / TRUTH.tau1 - 1) < 0.15 and abs(pm.delay / TRUTH.delay - 1) < 0.15
    with pytest.raises(ValueError) if hasattr(pytest, "raises") else _raises(ValueError):
        ti.identify([])


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
