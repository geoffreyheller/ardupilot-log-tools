"""PID tuning from logs, tier B: plant identification, loop margins, heli-style gain
ceilings and the virtual AutoTune.

Work package 5 of `docs/pid-tuning-plan.md`. Everything here consumes WP1's `AxisSignals`
/ `GainSet` and WP2's `tunesim` engine; nothing here recommends a gain by itself - WP6
fuses the numbers and grades them against `checks.T`.

What is computed, and from where:

* `identify(signals)` - the joint input/output estimate of the rate-loop plant
  (sources file section 7.1, Bendat & Piersol): per segment a Welch cross-spectral
  estimate of `T_yr = Y/R` and `T_ur = U/R` from the logged reference `PIDx.Tar` (r),
  the gyro `PIDx.Act` (y) and the plant input `P+I+D+FF+DFF` / `RATE.xOut` (u), then
  `G = T_yr / T_ur = Y/U`, unbiased under feedback because r is exogenous. Segments are
  pooled on a common 0.5 Hz grid by coherence-weighted averaging (the plant does not
  depend on the gains, so segments from different logs and gain sets pool). The valid
  band is gated by `T["tune_coherence"]["fail"]`, and a low-order model
  `k e^{-tau_d s} / ((tau1 s + 1)(tau2 s + 1))` is fitted by weighted least squares.
* `controller_response(gains, f)` - AC_PID as the discrete-time transfer function the
  firmware actually runs (sources section 2), in the log's units.
* `margins(plant, gains)` - the open loop `L = C G`, gain and phase margins and the
  crossover, on the parametric fit and on the non-parametric estimate inside its band.
* `margin_ceilings(plant, gains)` - the largest P and D whose full loop keeps PM >= 45 deg
  and GM >= 6 dB; what fusion clips to.
* `ceilings_from_plant(plant, gains)` - the heli AutoTune rules (sources section 5):
  the P that gives 2.42 dB below unity at the -161 deg frequency, the D likewise at
  -251 deg, each capped at twice the AutoTune maximum. P-only, so reported as evidence
  and not applied on a multicopter (it ignores D's phase lead).
* `virtual_autotune(plant, gains)` - `tunesim.autotune` on the identified plant with
  the log's own AGGR / GMBK / MIN_D / hover throttle.

Unit conventions (stated once, used everywhere):

* Rates are deg/s, as `AxisSignals.tar/act` carry them (`PIDx` logs rad/s;
  `tune.extract_axes` converts). The plant `G` maps normalised controller
  output (+-1) to deg/s, so `k` is deg/s per unit output at DC - the same `k` that
  `tunesim.Plant` takes.
* The firmware's rate PIDs multiply **rad/s**, so a configured `ATC_RAT_x_P` of 0.135
  is 0.135 per rad/s. `controller_response` returns `C` in **log units** (normalised
  output per deg/s of error) by folding the pi/180 in, so that `L = C G` is dimensionless
  with `G` in deg/s per unit. The heli ceilings convert `G` back to rad/s per unit before
  applying the firmware's formulas, because their result is a gain in firmware units.
* The controller sees the plant through a sampler and a zero-order hold at the loop
  rate. `G` measured from the log carries that (the samples are what the loop saw); the
  parametric model is therefore fitted **with** the ZOH factor
  `(1 - e^{-s T}) / (s T)` (T = 1/loop_hz), so the fitted `delay` is the plant's own
  transport delay excluding the hold - exactly what `tunesim.Plant(delay=...)` expects,
  since it re-applies the ZOH when it discretises. `margins()` includes the same factor
  on the parametric model; the non-parametric `G` needs no correction.
* Frequencies are Hz at every interface; rad/s appear only inside formulas that the
  firmware states in rad/s (the heli D ceiling) and are named `w` there.

Two limits of coherence as a gate, both measured on synthetic logs and stated here so
nobody rediscovers them: (1) it is necessary, not sufficient - a bin fed by the Hann
window's leakage of a strong neighbouring tone (the bins just above a chirp's stop
frequency) is coherent and biased, and a hover with no stick is coherent everywhere
because the reference is feedback-generated; `identify` gates excitation first for the
second case, and the first is why a SysID chirp must sweep past the loop crossover.
(2) the AnalyticTune chirp of 0.05-5 Hz stops at or below the rate-loop crossover (4.5-5 Hz
measured on the 10-inch Brisket, 2026-09-30; higher on smaller props: the 5-inch simulator
plant crosses near 23 Hz), so it yields `NO_COHERENCE` where it matters;
`SID_F_STOP_HZ` must exceed the crossover for tier B to use it.

Every constant with a provenance is in `IDENT_CONSTANTS` (same shape as
`tune.CONSTANTS`, for `alog schema`); graded thresholds come from `checks.T`.
Deterministic: no randomness anywhere, fixed optimiser budget.
"""

from __future__ import annotations

import math

import numpy as np
from scipy import signal as _signal
from scipy.optimize import least_squares

from .checks import T
from .tune import (AXES, RAT_STEM, CONSTANTS, PlantModel, Ceiling, Refusal, GainSet)
from . import tunesim

__all__ = ["IDENT_CONSTANTS", "identify", "segment_spectra", "pool_spectra", "fit_model", "excitation_frames",
           "model_response", "zoh_response", "controller_response", "plant_response",
           "margins", "closed_loop_step_from_model", "ceilings_from_plant", "margin_ceilings",
           "margin_limits", "meets_margins", "pinned_params", "extrapolated",
           "virtual_autotune", "describe", "plant_rows", "margin_rows", "model_from_params",
           "bendat_piersol_eps"]

# ------------------------------------------------------------------------- constants

_SRC_PLAN = "docs/pid-tuning-plan.md section 2.4 (plant identification)"
_SRC_BP = ("Bendat & Piersol, Random Data, random error of a frequency-response estimate; "
           "reference/pid-tuning-sources.md section 7.1")
_SRC_HELI = ("ArduCopter libraries/AC_AutoTune/AC_AutoTune_Heli.cpp max_allowed_P / max_allowed_D; "
             "reference/pid-tuning-sources.md section 5")
_SRC_WP5 = "docs/pid-tuning-plan.md section 5 WP5 (this module)"
_SRC_MARGIN = ("2026-09-30 Brisket validation (reference/pitfalls.md, PID tuning): the largest gain whose full "
               "C(z) G loop keeps tune_phase_margin_deg / tune_gain_margin_db (warn levels)")


def _c(value, source, note=""):
    return dict(value=value, source=source, note=note)


#: Algorithm constants of tier B (not gradings). Same shape as `tune.CONSTANTS` so WP7 can
#: print them under `tune_constants`.
IDENT_CONSTANTS = {
    "ident_nperseg_s":       _c(2.0, _SRC_PLAN, "Welch segment length, s (0.5 Hz bins); Hann, 50 % overlap, constant detrend"),
    "ident_grid_hz":         _c(0.5, _SRC_PLAN, "common frequency grid for pooling segments, Hz"),
    "ident_min_avg":         _c(3, _SRC_WP5, "a segment with fewer Welch averages than this is not used"),
    "ident_min_band_bins":   _c(6, _SRC_WP5, "a coherent band with fewer bins than this cannot support a 4-parameter fit"),
    "ident_band_dip_bins":   _c(1, _SRC_WP5, "isolated coherence dips of at most this many bins do not end the band"),
    "ident_eps_floor":       _c(1e-3, _SRC_WP5, "floor on the Bendat-Piersol error used as 1/weight in the fit"),
    "ident_tau1_bounds_s":   _c((0.01, 5.0), _SRC_PLAN, "fit bounds for the slow (aero) pole"),
    "ident_tau2_bounds_s":   _c((0.001, 0.2), _SRC_PLAN, "fit bounds for the fast (motor/ESC) pole"),
    "ident_delay_bounds_s":  _c((0.0, 0.05), _SRC_PLAN, "fit bounds for the transport delay (excluding the ZOH)"),
    "ident_k_bounds":        _c((1e-3, 1e7), _SRC_WP5, "fit bounds for k, deg/s per unit output"),
    "ident_max_nfev":        _c(300, _SRC_WP5, "least_squares function-evaluation budget (fixed: deterministic)"),
    "ident_init_tau2_s":     _c(0.02, _SRC_WP5, "initial guess for tau2"),
    "ident_init_delay_s":    _c(0.005, _SRC_WP5, "initial guess for the delay"),
    "margin_grid_points":    _c(4000, _SRC_WP5, "log-spaced frequencies from margin_fmin_hz to Nyquist for the parametric open loop"),
    "margin_fmin_hz":        _c(0.1, _SRC_WP5, "lowest frequency of the parametric open-loop grid"),
    "heli_p_phase_deg":      _c(-161.0, _SRC_HELI, "plant phase at which max_allowed_P is evaluated"),
    "heli_d_phase_deg":      _c(-251.0, _SRC_HELI, "plant phase at which max_allowed_D is evaluated"),
    "heli_margin_db":        _c(2.42, _SRC_HELI, "max gain = 10^(-(20 log10 gain + 2.42)/20): the loop gain at that phase"),
    "heli_cap_mul":          _c(2.0, _SRC_HELI, "ceilings capped at 2 x AUTOTUNE_RP_MAX / RD_MAX"),
    "step_model_seconds":    _c(0.5, _SRC_PLAN, "length of the model-predicted closed-loop step, s (matches step_response_s)"),
    "margin_ceiling_span":   _c((0.05, 4.0), _SRC_MARGIN, "margin ceilings scan this multiple of the current gain; none above it is reported"),
    "margin_ceiling_points": _c(48, _SRC_MARGIN, "log-spaced scan points, then bisection between the last pass and first fail"),
    "margin_ceiling_bisect": _c(20, _SRC_MARGIN, "bisection steps (relative resolution ~1e-6 of the bracket)"),
}

_GRID = float(IDENT_CONSTANTS["ident_grid_hz"]["value"])
_DEG = math.pi / 180.0


# ------------------------------------------------------------------- transfer functions

def zoh_response(freqs_hz, loop_hz):
    """The sample-and-hold the controller sees the plant through:
    `(1 - e^{-jwT}) / (jwT)`, T = 1/loop_hz; 1 at 0 Hz. Phase -wT/2, magnitude sinc."""
    f = np.asarray(freqs_hz, dtype=float)
    T = 1.0 / float(loop_hz)
    w = 2.0 * np.pi * f
    out = np.ones(f.shape, dtype=complex)
    nz = w != 0
    out[nz] = (1.0 - np.exp(-1j * w[nz] * T)) / (1j * w[nz] * T)
    return out


def model_response(k, tau1, tau2, delay, freqs_hz, loop_hz=None):
    """`G(j2 pi f) = k e^{-j w delay} / ((tau1 jw + 1)(tau2 jw + 1))`, deg/s per unit
    output; with `loop_hz` the ZOH factor is included (what a loop-rate controller sees)."""
    f = np.asarray(freqs_hz, dtype=float)
    s = 1j * 2.0 * np.pi * f
    G = k * np.exp(-s * delay) / ((tau1 * s + 1.0) * (tau2 * s + 1.0))
    if loop_hz:
        G = G * zoh_response(f, loop_hz)
    return G


def plant_response(plant, freqs_hz, loop_hz=None):
    """`model_response` of a `PlantModel`'s fitted parameters."""
    return model_response(plant.k, plant.tau1, plant.tau2, plant.delay, freqs_hz, loop_hz)


def _gd(gains):
    return tunesim.gain_dict(gains)


def controller_response(gains, freqs_hz):
    """AC_PID's rate controller as a transfer function from rate error (deg/s) to
    normalised output, `C(e^{jwT})`, T = 1/loop_hz - the discrete-time filters the
    firmware runs (sources section 2), evaluated on the unit circle.

    Structure (z^-1 = one loop):
        H_E(z) = a_E / (1 - (1 - a_E) z^-1)          FLTE on the error   (0 Hz: H_E = 1)
        H_D(z) = a_D / (1 - (1 - a_D) z^-1)          FLTD on the derivative of the filtered error
        C(z)   = (pi/180) * H_E(z) * [ P  +  I T / (1 - z^-1)  +  D H_D(z) (1 - z^-1) / T ]
    with `a = calc_lowpass_alpha_dt(T, hz)`. The integrator is the firmware's forward
    accumulator `I += err * ki * T`; the derivative its backward difference. FLTT sits on
    the target path (before `PIDx.Tar`) and FF / DFF are fed from the target, so none of
    them is inside the loop and none is here. SMAX (`Dmod`) is taken as 1.0. The pi/180
    converts the deg/s of the log to the rad/s the gains multiply.

    `gains` is anything `tunesim.gain_dict` accepts. At exactly 0 Hz the integrator is
    infinite and the value is `inf`.
    """
    g = _gd(gains)
    T_ = 1.0 / float(g["loop_hz"])
    f = np.asarray(freqs_hz, dtype=float)
    z1 = np.exp(-1j * 2.0 * np.pi * f * T_)                      # z^-1
    a_e = tunesim.calc_lowpass_alpha_dt(T_, float(g["flte"]))
    a_d = tunesim.calc_lowpass_alpha_dt(T_, float(g["fltd"]))
    H_E = a_e / (1.0 - (1.0 - a_e) * z1)
    H_D = a_d / (1.0 - (1.0 - a_d) * z1)
    one_minus = 1.0 - z1
    with np.errstate(divide="ignore", invalid="ignore"):
        integ = np.where(one_minus == 0, np.inf + 0j, float(g["rat_i"]) * T_ / one_minus)
        deriv = float(g["rat_d"]) * H_D * one_minus / T_
        C = _DEG * H_E * (float(g["rat_p"]) + integ + deriv)
    return C


# --------------------------------------------------------------------- non-parametric

def bendat_piersol_eps(coh, n_avg):
    """Random error of |G| (relative) and of its phase (rad) for coherence `coh` and
    `n_avg` averages: `sqrt(1 - coh) / (|coh|^0.5 sqrt(2 n_avg))` (sources 7.1)."""
    c = np.clip(np.asarray(coh, dtype=float), 1e-12, 1.0)
    return np.sqrt(1.0 - c) / (np.sqrt(c) * math.sqrt(2.0 * max(int(n_avg), 1)))


def _welch_n_avg(n, nperseg, noverlap):
    return max(0, (n - nperseg) // (nperseg - noverlap) + 1)


def segment_spectra(signals):
    """Per-segment joint I/O estimates, one dict per usable `AxisSignals`:
    `freqs, G, coh, n_avg, T_yr, T_ur, log_name, segment, fs, nperseg, gains, band`
    (the segment's own coherent band), plus `skipped` entries `(log_name, segment,
    reason)` in the returned `(estimates, skipped)` tuple.

    Welch / csd with a Hann window, `nperseg = round(ident_nperseg_s x fs)`, 50 %
    overlap, constant detrend. `G = (P_ry / P_rr) / (P_ru / P_rr) = P_ry / P_ru` with
    r = tar, y = act, u = out; `coh = |P_ry|^2 / (P_rr P_yy)`.
    """
    seg_s = float(IDENT_CONSTANTS["ident_nperseg_s"]["value"])
    min_avg = int(IDENT_CONSTANTS["ident_min_avg"]["value"])
    out, skipped = [], []
    for sig in signals:
        where = (sig.log_name, tuple(sig.segment))
        if sig.out is None:
            skipped.append(where + ("no plant input (P+I+D+FF columns missing)",))
            continue
        r, y, u = (np.asarray(x, dtype=float) for x in (sig.tar, sig.act, sig.out))
        m = np.isfinite(r) & np.isfinite(y) & np.isfinite(u)
        r, y, u = r[m], y[m], u[m]
        fs = float(sig.fs)
        nperseg = int(round(seg_s * fs))
        noverlap = nperseg // 2
        n_avg = _welch_n_avg(len(r), nperseg, noverlap)
        if n_avg < min_avg:
            skipped.append(where + (f"{len(r)} samples give {n_avg} Welch averages; need {min_avg}",))
            continue
        kw = dict(fs=fs, window="hann", nperseg=nperseg, noverlap=noverlap, detrend="constant")
        f, P_rr = _signal.csd(r, r, **kw)
        _, P_yy = _signal.csd(y, y, **kw)
        _, P_ry = _signal.csd(r, y, **kw)          # conj(R) Y  ->  P_ry / P_rr = Y / R
        _, P_ru = _signal.csd(r, u, **kw)
        with np.errstate(divide="ignore", invalid="ignore"):
            T_yr = P_ry / P_rr
            T_ur = P_ru / P_rr
            G = T_yr / T_ur
            coh = np.abs(P_ry) ** 2 / (np.real(P_rr) * np.real(P_yy))
        coh = np.where(np.isfinite(coh), np.clip(coh, 0.0, 1.0), 0.0)
        G = np.where(np.isfinite(G), G, np.nan + 0j)
        out.append(dict(freqs=f, G=G, coh=coh, n_avg=int(n_avg), T_yr=T_yr, T_ur=T_ur,
                        log_name=sig.log_name, segment=tuple(sig.segment), fs=fs, nperseg=nperseg,
                        gains=sig.gains, band=_band(f, coh)))
    return out, skipped


def excitation_frames(sig):
    """Number of non-overlapping `step_frame_s` frames of the segment whose max |Tar|
    reaches `step_min_target_dps` (WP1's constants; PID-Analyzer's 20 deg/s rule)."""
    frame = int(round(float(CONSTANTS["step_frame_s"]["value"]) * float(sig.fs)))
    lvl = float(CONSTANTS["step_min_target_dps"]["value"])
    r = np.abs(np.asarray(sig.tar, dtype=float))
    if frame <= 0 or r.size < frame:
        return 0
    n = r.size // frame
    return int(np.sum(r[:n * frame].reshape(n, frame).max(axis=1) >= lvl))


def _band(freqs, coh, gate=None):
    """(f_lo, f_hi) of the contiguous run of bins with coh >= gate starting at the lowest
    such bin (the 0 Hz bin is never counted); isolated dips of at most
    `ident_band_dip_bins` bins inside the run are tolerated. (None, None) if empty."""
    if gate is None:
        gate = float(T["tune_coherence"]["fail"])
    dip = int(IDENT_CONSTANTS["ident_band_dip_bins"]["value"])
    ok = np.asarray(coh) >= gate
    ok[freqs <= 0] = False
    idx = np.flatnonzero(ok)
    if idx.size == 0:
        return (None, None)
    lo = int(idx[0])
    hi = lo
    j = lo
    while j + 1 < len(ok):
        if ok[j + 1]:
            j += 1
            hi = j
            continue
        run = 0
        while j + 1 + run < len(ok) and not ok[j + 1 + run]:
            run += 1
        if run <= dip and j + 1 + run < len(ok):
            j += run + 1
            hi = j
            continue
        break
    # a segment's own grid is k x fs / nperseg with fs measured from microsecond
    # timestamps, so 0.5 Hz may arrive as 0.5000000000004; report the bin cleanly
    return (round(float(freqs[lo]), 6), round(float(freqs[hi]), 6))


def pool_spectra(estimates):
    """Coherence-weighted pooling of per-segment estimates on the common grid
    `0, 0.5, 1, ... <= min Nyquist`: weights `coh x n_avg` per bin, `G` and `coh`
    averaged with them, `n_avg` summed. Returns `(freqs, G, coh, n_avg)`."""
    if not estimates:
        raise ValueError("no estimates to pool")
    f_max = min(float(e["freqs"][-1]) for e in estimates)
    f = np.arange(0.0, f_max + 1e-9, _GRID)
    num = np.zeros(len(f), dtype=complex)
    cnum = np.zeros(len(f))
    den = np.zeros(len(f))
    for e in estimates:
        ef, eG, ec = e["freqs"], e["G"], e["coh"]
        good = np.isfinite(eG)
        if good.sum() < 2:
            continue
        Gr = np.interp(f, ef[good], np.real(eG[good]), left=np.nan, right=np.nan)
        Gi = np.interp(f, ef[good], np.imag(eG[good]), left=np.nan, right=np.nan)
        c = np.interp(f, ef, ec)
        w = c * e["n_avg"]
        ok = np.isfinite(Gr) & np.isfinite(Gi) & (w > 0)
        num[ok] += w[ok] * (Gr[ok] + 1j * Gi[ok])
        cnum[ok] += w[ok] * c[ok]
        den[ok] += w[ok]
    with np.errstate(divide="ignore", invalid="ignore"):
        G = np.where(den > 0, num / den, np.nan + 0j)
        coh = np.where(den > 0, cnum / den, 0.0)
    return f, G, coh, int(sum(e["n_avg"] for e in estimates))


# -------------------------------------------------------------------------- the fit

def _bounds():
    K = IDENT_CONSTANTS
    lo = [math.log(K["ident_k_bounds"]["value"][0]), math.log(K["ident_tau1_bounds_s"]["value"][0]),
          math.log(K["ident_tau2_bounds_s"]["value"][0]), K["ident_delay_bounds_s"]["value"][0]]
    hi = [math.log(K["ident_k_bounds"]["value"][1]), math.log(K["ident_tau1_bounds_s"]["value"][1]),
          math.log(K["ident_tau2_bounds_s"]["value"][1]), K["ident_delay_bounds_s"]["value"][1]]
    return np.array(lo), np.array(hi)


def _initial_guess(f, G):
    """Deterministic start: tau1 from the frequency where the (sign-corrected, unwrapped)
    phase first passes -45 deg, k from the lowest bin's magnitude corrected by that pole."""
    K = IDENT_CONSTANTS
    ph = np.unwrap(np.angle(G))
    if ph[0] > math.pi / 2:
        ph = ph - 2.0 * math.pi
    below = np.flatnonzero(ph <= -math.pi / 4)
    f45 = float(f[below[0]]) if below.size else float(f[-1])
    tau1 = 1.0 / (2.0 * math.pi * max(f45, 1e-3))
    k = float(np.abs(G[0])) * abs(1.0 + 1j * 2.0 * math.pi * f[0] * tau1)
    lo, hi = _bounds()
    x0 = np.array([math.log(max(k, 1e-6)), math.log(tau1),
                   math.log(K["ident_init_tau2_s"]["value"]), K["ident_init_delay_s"]["value"]])
    return np.clip(x0, lo + 1e-9, hi - 1e-9)


def fit_model(freqs, G, coh, n_avg, loop_hz, sign=1.0):
    """Weighted least squares of `k, tau1, tau2, delay` to `G` (which includes the
    loop-rate ZOH). Residuals are the complex log ratio `ln|Gm/G|` (nepers) and the
    wrapped phase difference (rad) - commensurate quantities - each divided by the
    Bendat-Piersol error at that bin (floored), so a bin with coherence 0.99 counts
    about eight times a bin at the 0.6 gate. `sign=-1` fits `-G_model` (used to detect
    an inverted plant input). Returns `(k, tau1, tau2, delay, rms_db, rms_deg, cost)`.
    """
    f = np.asarray(freqs, dtype=float)
    G = np.asarray(G, dtype=complex)
    eps = bendat_piersol_eps(coh, n_avg)
    w = 1.0 / np.maximum(eps, float(IDENT_CONSTANTS["ident_eps_floor"]["value"]))
    lnG = np.log(np.abs(G))
    phG = np.angle(G)
    zoh = zoh_response(f, loop_hz)

    def resid(x):
        k, tau1, tau2, delay = math.exp(x[0]), math.exp(x[1]), math.exp(x[2]), x[3]
        Gm = sign * model_response(k, tau1, tau2, delay, f) * zoh
        dmag = np.log(np.abs(Gm)) - lnG
        dph = np.angle(np.exp(1j * (np.angle(Gm) - phG)))
        return np.concatenate([w * dmag, w * dph])

    lo, hi = _bounds()
    x0 = _initial_guess(f, sign * G)
    res = least_squares(resid, x0, bounds=(lo, hi), method="trf",
                        max_nfev=int(IDENT_CONSTANTS["ident_max_nfev"]["value"]), x_scale="jac")
    k, tau1, tau2, delay = math.exp(res.x[0]), math.exp(res.x[1]), math.exp(res.x[2]), float(res.x[3])
    Gm = sign * model_response(k, tau1, tau2, delay, f) * zoh
    rms_db = float(np.sqrt(np.mean((20.0 * np.log10(np.abs(Gm) / np.abs(G))) ** 2)))
    rms_deg = float(np.sqrt(np.mean(np.degrees(np.angle(np.exp(1j * (np.angle(Gm) - phG)))) ** 2)))
    return k, tau1, tau2, delay, rms_db, rms_deg, float(res.cost)


def model_from_params(k, tau1, tau2, delay, freqs=None, loop_hz=400.0, n_avg=0, coh=None):
    """A `PlantModel` from known parameters (a true plant in a test, or a model carried
    over from another analysis): the non-parametric arrays are the model's own response
    with coherence 1.0 unless given, fit residuals 0, band the whole grid."""
    f = np.arange(0.0, loop_hz / 2.0 + 1e-9, _GRID) if freqs is None else np.asarray(freqs, dtype=float)
    G = model_response(k, tau1, tau2, delay, f, loop_hz)
    c = np.ones(len(f)) if coh is None else np.asarray(coh, dtype=float)
    return PlantModel(freqs=f, G=G, coh=c, n_avg=int(n_avg), band=(float(f[1]) if len(f) > 1 else 0.0, float(f[-1])),
                      k=float(k), tau1=float(tau1), tau2=float(tau2), delay=float(delay),
                      fit_rms_db=0.0, fit_rms_deg=0.0, coh_mean_band=float(np.mean(c[1:])) if len(c) > 1 else 1.0,
                      eps_mag_at_crossover=float(bendat_piersol_eps(1.0, max(n_avg, 1))) if n_avg else 0.0)


# --------------------------------------------------------------------------- identify

def _np_crossover(freqs, G, coh, band, gains):
    """(f_c, |L| at the band's top bin, |L| array) for the non-parametric open loop
    inside `band`; f_c None when |L| does not cross 1 downward inside the band."""
    m = (freqs >= band[0]) & (freqs <= band[1]) & np.isfinite(G)
    f = freqs[m]
    L = np.abs(controller_response(gains, f) * G[m])
    fc = _cross_down(f, L, 1.0)
    return fc, float(L[-1]) if len(L) else float("nan"), f, L


def _cross_down(f, mag, level):
    """First frequency (linear interpolation in log magnitude) where `mag` falls through
    `level` from above; None if it never does."""
    lm = np.log(np.asarray(mag, dtype=float)) - math.log(level)
    for i in range(len(lm) - 1):
        if lm[i] >= 0 > lm[i + 1]:
            t = lm[i] / (lm[i] - lm[i + 1])
            return float(f[i] + t * (f[i + 1] - f[i]))
    return None


def _cross_phase(f, ph_deg, level_deg):
    """First frequency where the (unwrapped) phase falls through `level_deg`; None if never."""
    p = np.asarray(ph_deg, dtype=float) - level_deg
    for i in range(len(p) - 1):
        if p[i] >= 0 > p[i + 1]:
            t = p[i] / (p[i] - p[i + 1])
            return float(f[i] + t * (f[i + 1] - f[i]))
    return None


def identify(signals):
    """Tier B plant identification: `PlantModel` from one or more `AxisSignals` of one
    axis (any number of logs and gain sets), or a `Refusal`.

    Steps: `segment_spectra` per segment, `pool_spectra` across them, the coherent band
    (`T["tune_coherence"]["fail"]`, contiguous from the lowest passing bin), a check that
    the band reaches the loop crossover of the *last* segment's gain set (the current
    one), then `fit_model` for the plant and for its negative. Refusals:

      NO_EXCITATION   fewer than `T["tune_min_frames"]["fail"]` one-second frames with
                      |Tar| >= 20 deg/s across the segments. Checked *before* coherence
                      because coherence cannot detect this case: with no pilot input the
                      reference is the angle loop's reaction to the noise-driven attitude,
                      r, y and u are one process, coherence reads ~1.0 and the estimate
                      is biased toward -1/C (sources 7.1). Fix: the tuning profile
      NO_COHERENCE    no usable segment, an empty band, fewer than `ident_min_band_bins`
                      bins, or a band whose top is still above unity loop gain (the
                      crossover is not inside it) - the message carries the mean
                      coherence and the band, the fix is the SysID chirp flight or
                      sharper stick inputs
      SIGN_MISMATCH   the inverted model fits better: positive `out` does not produce
                      positive `act` on this axis (a reversed channel, or the wrong
                      column pairing) - nothing is flipped silently

    `eps_mag_at_crossover` is the Bendat-Piersol random error of |G| at the bin nearest
    the non-parametric crossover, with the pooled coherence and the summed `n_avg`.
    """
    signals = list(signals)
    if not signals:
        raise ValueError("identify() needs at least one AxisSignals")
    axes = {s.axis for s in signals}
    if len(axes) != 1:
        raise ValueError(f"identify() pools one axis at a time, got {sorted(axes)}")
    axis = signals[-1].axis
    gains = signals[-1].gains
    log_name = signals[-1].log_name
    gate = float(T["tune_coherence"]["fail"])
    fix = ("fly the plant-identification profile (SysID chirp, one axis per flight: SID_AXIS 10/11/12, "
           "SID_MAGNITUDE 0.15 (yaw 0.55), SID_F_START_HZ 0.5, SID_F_STOP_HZ 40, SID_T_REC 70, "
           "SID_T_FADE_IN 15, SID_T_FADE_OUT 2; docs/pid-tuning-plan.md section 7) or more, sharper stick "
           "inputs on this axis; tier C (step response) is still reported")

    # Excitation gate. Coherence is necessary, not sufficient: in hover the reference is
    # the angle loop's own reaction to the noise-driven attitude, so r, y and u are all
    # one noise process - the coherence reads ~1.0 and the estimate is biased to -1/C.
    # The reference must carry the pilot: WP1's frame rule (tier C uses the same one).
    n_frames = sum(excitation_frames(s) for s in signals)
    min_frames = int(T["tune_min_frames"]["fail"])
    if n_frames < min_frames:
        span = sum(s.duration for s in signals)
        return Refusal("NO_EXCITATION",
                       f"{n_frames} one-second frame(s) with |Tar| >= {CONSTANTS['step_min_target_dps']['value']:.0f} deg/s "
                       f"in {span:.0f} s of {axis} signals ({min_frames} needed): without pilot input the reference is "
                       "generated by the feedback itself, its coherence with the gyro is not evidence, and the "
                       "joint I/O estimate would be biased toward -1/C",
                       "fly the tuning profile (docs/pid-tuning-plan.md section 7): 60 s of sharp stick inputs "
                       "on the axis, +-15-20 deg with quick release", axis=axis, log_name=log_name)

    estimates, skipped = segment_spectra(signals)
    if not estimates:
        why = "; ".join(f"{ln} {s[0]:.1f}-{s[1]:.1f} s: {r}" for ln, s, r in skipped)
        return Refusal("NO_COHERENCE", f"no segment could be transformed ({why})", fix, axis=axis, log_name=log_name)
    freqs, G, coh, n_avg = pool_spectra(estimates)
    band = _band(freqs, coh, gate)
    coh_mean_all = float(np.mean(coh[1:])) if len(coh) > 1 else 0.0
    n_seg = len(estimates)
    if band[0] is None:
        return Refusal("NO_COHERENCE",
                       f"reference-to-gyro coherence never reaches the {gate:.2f} gate (mean {coh_mean_all:.2f} over "
                       f"0.5-{freqs[-1]:.0f} Hz, {n_seg} segment(s), {n_avg} averages): the stick did not excite the "
                       "loop above the noise", fix, axis=axis, log_name=log_name)
    in_band = (freqs >= band[0]) & (freqs <= band[1])
    n_bins = int(in_band.sum())
    coh_mean_band = float(np.mean(coh[in_band]))
    min_bins = int(IDENT_CONSTANTS["ident_min_band_bins"]["value"])
    if n_bins < min_bins:
        return Refusal("NO_COHERENCE",
                       f"coherent band {band[0]:.1f}-{band[1]:.1f} Hz is only {n_bins} bin(s) (mean coherence "
                       f"{coh_mean_band:.2f} there, {coh_mean_all:.2f} overall); {min_bins} needed for a "
                       "4-parameter fit", fix, axis=axis, log_name=log_name)
    fc, L_top, _fb, _Lb = _np_crossover(freqs, G, coh, band, gains)
    if fc is None:
        L_bot = float(_Lb[0]) if len(_Lb) else float("nan")
        if L_bot < 1.0:
            where = (f"the loop crossover lies below it: |C G| is already {L_bot:.2f} at {band[0]:.1f} Hz "
                     f"and {L_top:.2f} at {band[1]:.1f} Hz")
        else:
            where = f"it does not reach the loop crossover: |C G| is still {L_top:.2f} at {band[1]:.1f} Hz"
        return Refusal("NO_COHERENCE",
                       f"coherent band {band[0]:.1f}-{band[1]:.1f} Hz (mean coherence {coh_mean_band:.2f}) does not "
                       f"contain the loop crossover - {where}, with the {axis} gains of {log_name}",
                       fix, axis=axis, log_name=log_name)

    fb, Gb, cb = freqs[in_band], G[in_band], coh[in_band]
    good = np.isfinite(Gb)
    fb, Gb, cb = fb[good], Gb[good], cb[good]
    loop_hz = float(gains.loop_hz)
    fit_pos = fit_model(fb, Gb, cb, n_avg, loop_hz, sign=1.0)
    fit_neg = fit_model(fb, Gb, cb, n_avg, loop_hz, sign=-1.0)
    if fit_neg[6] < fit_pos[6]:
        return Refusal("SIGN_MISMATCH",
                       f"the plant fits better with its sign inverted (cost {fit_neg[6]:.3g} vs {fit_pos[6]:.3g}): "
                       f"positive controller output does not produce positive {axis} rate in this log - a reversed "
                       "output or the wrong column pairing; nothing was flipped",
                       "check the axis' motor/servo direction and that PIDx.Tar/Act and RATE.xOut belong to the "
                       "same axis; do not tune from this data", axis=axis, log_name=log_name)
    k, tau1, tau2, delay, rms_db, rms_deg, _cost = fit_pos
    i_c = int(np.argmin(np.abs(freqs - fc)))
    eps = float(bendat_piersol_eps(coh[i_c], n_avg))
    return PlantModel(freqs=freqs, G=G, coh=coh, n_avg=int(n_avg), band=(float(band[0]), float(band[1])),
                      k=float(k), tau1=float(tau1), tau2=float(tau2), delay=float(delay),
                      fit_rms_db=rms_db, fit_rms_deg=rms_deg, coh_mean_band=coh_mean_band,
                      eps_mag_at_crossover=eps)


# ---------------------------------------------------------------------------- margins

def _margins_on(f, L):
    """(gm_db, pm_deg, fc_hz, f180_hz) from an open-loop response on an increasing grid.
    Phase is unwrapped from the low end; None where a crossing does not exist."""
    mag = np.abs(L)
    ph = np.degrees(np.unwrap(np.angle(L)))
    fc = _cross_down(f, mag, 1.0)
    pm = None
    if fc is not None:
        pm = float(180.0 + np.interp(fc, f, ph))
    f180 = _cross_phase(f, ph, -180.0)
    gm = None
    if f180 is not None:
        gm = float(-20.0 * np.log10(np.interp(f180, f, mag)))
    return gm, pm, fc, f180


def margins(plant, gains):
    """Open-loop `L = C G` and its margins, numbers only (WP6 grades them):

      gm_db, pm_deg, fc_hz, f180_hz     on the parametric fit x ZOH, `margin_fmin_hz`
                                        to Nyquist (extends beyond the measured band)
      np_gm_db, np_pm_deg, np_fc_hz,
      np_f180_hz, np_in_band            the same on the non-parametric `G` inside `band`
                                        (None where the crossing lies outside it)
      freqs_hz, L, np_freqs_hz, np_L    the responses themselves (complex)
      band, loop_hz, method

    Gain margin is `-20 log10 |L|` at the first -180 deg crossing, phase margin
    `180 + arg L` at the first unity-gain crossing (linear interpolation on a log
    frequency grid; phase unwrapped from the low end where the integrator puts it near
    -90 deg).
    """
    g = _gd(gains)
    loop_hz = float(g["loop_hz"])
    K = IDENT_CONSTANTS
    f = np.logspace(math.log10(float(K["margin_fmin_hz"]["value"])), math.log10(loop_hz / 2.0),
                    int(K["margin_grid_points"]["value"]))
    L = controller_response(g, f) * plant_response(plant, f, loop_hz)
    gm, pm, fc, f180 = _margins_on(f, L)
    out = dict(gm_db=gm, pm_deg=pm, fc_hz=fc, f180_hz=f180, freqs_hz=f, L=L, loop_hz=loop_hz,
               band=tuple(plant.band) if plant.band and plant.band[0] is not None else None,
               method="L = C(z) G_fit(s) ZOH(s), C = AC_PID discrete filters in log units, "
                      f"grid {f[0]:.2f}-{f[-1]:.0f} Hz")
    np_gm = np_pm = np_fc = np_f180 = None
    fb = Lb = None
    if out["band"] is not None and plant.freqs is not None and len(plant.freqs):
        fr, G = np.asarray(plant.freqs, dtype=float), np.asarray(plant.G, dtype=complex)
        m = (fr >= out["band"][0]) & (fr <= out["band"][1]) & np.isfinite(G) & (fr > 0)
        if m.sum() >= 2:
            fb = fr[m]
            Lb = controller_response(g, fb) * G[m]
            np_gm, np_pm, np_fc, np_f180 = _margins_on(fb, Lb)
    out.update(np_gm_db=np_gm, np_pm_deg=np_pm, np_fc_hz=np_fc, np_f180_hz=np_f180,
               np_in_band=np_fc is not None, np_freqs_hz=fb, np_L=Lb)
    return out


def closed_loop_step_from_model(plant, gains, seconds=None):
    """The fitted model's closed-loop response to a unit step of `PIDx.Tar` (deg/s), by
    discrete simulation with `tunesim` - the like-for-like partner of WP3's deconvolved
    Tar -> Act step. FLTT is bypassed (it sits before `Tar`, so the deconvolution never
    sees it); everything else is the log's gain set. Returns `dict(t, act, tar,
    method)`; `act` is the response (1.0 = the target)."""
    g = _gd(gains)
    if seconds is None:
        seconds = float(IDENT_CONSTANTS["step_model_seconds"]["value"])
    p = tunesim.Plant(plant.k, plant.tau1, plant.tau2, plant.delay, loop_hz=float(g["loop_hz"]))
    t, act = p.true_closed_loop_step(dict(g, fltt=0.0), seconds=seconds)
    return dict(t=t, act=act, tar=np.ones_like(act),
                method=f"tunesim.ClosedLoop on the fitted plant (delay {p.delay_samples} loops), unit Tar step, FLTT bypassed")


# --------------------------------------------------------------------------- ceilings

def ceilings_from_plant(plant, gains):
    """Heli AutoTune's gain ceilings (sources section 5) applied to the identified plant:

        at the frequency where arg G = -161 deg:  max_P = 10^(-(20 log10 |G| + 2.42)/20)
        at the frequency where arg G = -251 deg:  max_D = 10^(-(20 log10 (w |G|) + 2.42)/20),  w in rad/s

    with `|G|` in the units the gains multiply (rad/s per unit output, i.e. the deg/s
    model x pi/180) and the loop-rate ZOH included, each capped at `heli_cap_mul` x
    `AUTOTUNE_RP_MAX` / `RD_MAX`. A phase the plant never reaches below Nyquist gives no
    ceiling for that gain. Evidence carries the frequency, the phase, the gain used and
    the cap; `np_value` is the same rule on the non-parametric `G` when the crossing lies
    inside the coherent band.
    """
    g = _gd(gains)
    axis = getattr(gains, "axis", None) or "roll"
    stem = RAT_STEM.get(axis, RAT_STEM["roll"])
    loop_hz = float(g["loop_hz"])
    K = IDENT_CONSTANTS
    margin_db = float(K["heli_margin_db"]["value"])
    cap_mul = float(K["heli_cap_mul"]["value"])
    f = np.logspace(math.log10(float(K["margin_fmin_hz"]["value"])), math.log10(loop_hz / 2.0),
                    int(K["margin_grid_points"]["value"]))
    G = plant_response(plant, f, loop_hz) * _DEG            # rad/s per unit output
    ph = np.degrees(np.unwrap(np.angle(G)))
    out = []
    band = plant.band if plant.band and plant.band[0] is not None else None
    fr = np.asarray(plant.freqs, dtype=float) if plant.freqs is not None else None
    Gn = np.asarray(plant.G, dtype=complex) * _DEG if plant.G is not None else None

    def np_rule(target_deg, use_w):
        if band is None or fr is None or Gn is None:
            return None
        m = (fr >= band[0]) & (fr <= band[1]) & np.isfinite(Gn) & (fr > 0)
        if m.sum() < 2:
            return None
        fb, Gb = fr[m], Gn[m]
        phb = np.degrees(np.unwrap(np.angle(Gb)))
        fx = _cross_phase(fb, phb, target_deg)
        if fx is None:
            return None
        gain = float(np.interp(fx, fb, np.abs(Gb)))
        if use_w:
            gain *= 2.0 * math.pi * fx
        return dict(f_hz=fx, value=10.0 ** (-(20.0 * math.log10(gain) + margin_db) / 20.0))

    for name, target, use_w, cap_key in (("P", float(K["heli_p_phase_deg"]["value"]), False, "autotune_rp_max"),
                                          ("D", float(K["heli_d_phase_deg"]["value"]), True, "autotune_rd_max")):
        fx = _cross_phase(f, ph, target)
        cap = cap_mul * float(CONSTANTS[cap_key]["value"])
        if fx is None:
            continue
        gain = float(np.interp(fx, f, np.abs(G)))
        w = 2.0 * math.pi * fx
        g_used = gain * w if use_w else gain
        raw = 10.0 ** (-(20.0 * math.log10(g_used) + margin_db) / 20.0)
        value = min(raw, cap)
        npv = np_rule(target, use_w)
        ev = dict(f_hz=fx, phase_deg=target, gain_rad_s_per_unit=gain, w_rad_s=w,
                  gain_used=g_used, gain_used_db=20.0 * math.log10(g_used), margin_db=margin_db,
                  raw=raw, cap=cap, capped=raw > cap,
                  formula=(f"10^(-(20 log10({'w ' if use_w else ''}|G|) + {margin_db})/20) at arg G = {target:.0f} deg, "
                           "|G| in rad/s per unit output, ZOH included"),
                  np_value=None if npv is None else npv["value"], np_f_hz=None if npv is None else npv["f_hz"],
                  source=IDENT_CONSTANTS["heli_p_phase_deg"]["source"])
        out.append(Ceiling(param=stem + name, value=float(value), method=f"heli-autotune-phase-{abs(int(target))}",
                           evidence=ev))
    return out


def pinned_params(plant, rel=0.02):
    """The high-frequency fit parameters (`tau2`, `delay`) sitting at a fit bound: the band
    did not constrain them, so the model's phase above the band is not measured.

    On brisket-t2.bin (Brisket) yaw was coherent over 0.5-3.5 Hz only; the fit put
    tau2 at its 1 ms floor and the delay at 0, while roll and pitch on the same motors read
    39 ms and 14 ms. `tau1` is left out on purpose: at its 5 s ceiling only k/tau1 is
    identifiable (roll k/tau1 1.311e4 vs 1.309e4 on two flights with k 6.6e4 vs 1.5e4),
    which is all the loop sees near crossover."""
    K = IDENT_CONSTANTS
    out = []
    lo, hi = K["ident_tau2_bounds_s"]["value"]
    if plant.tau2 <= lo * (1 + rel) or plant.tau2 >= hi * (1 - rel):
        out.append(f"tau2 {plant.tau2 * 1e3:.3g} ms")
    lo, hi = K["ident_delay_bounds_s"]["value"]
    if plant.delay <= lo + 1e-4 or plant.delay >= hi - 1e-4:
        out.append(f"delay {plant.delay * 1e3:.3g} ms")
    return out


def extrapolated(plant, m):
    """[reason]: why the margins in `m` (a `margins()` result) are not measured but
    extrapolated beyond the coherent band. A crossover above the band's top puts the phase
    margin outside the data. A -180 deg point above it puts the gain margin there too;
    that is normal (the fit carries it) unless the high-frequency lag is itself at a fit
    bound (`pinned_params`), in which case nothing measured stands behind it."""
    band = plant.band if plant.band and plant.band[0] is not None else None
    if band is None:
        return ["no coherent band"]
    out = []
    fc, f180 = m.get("fc_hz"), m.get("f180_hz")
    if fc is not None and fc > band[1]:
        out.append(f"crossover {fc:.2f} Hz lies above the coherent band {band[0]:g}-{band[1]:g} Hz")
    pins = pinned_params(plant)
    if pins and (f180 is None or f180 > band[1]):
        where = f"at {f180:.1f} Hz" if f180 is not None else "nowhere below Nyquist"
        out.append(f"gain margin read {where}, above the coherent band {band[0]:g}-{band[1]:g} Hz, on a fit whose "
                   f"{' and '.join(pins)} sit at their bounds (high-frequency lag not identified)")
    return out


def margin_limits():
    """(pm_min_deg, gm_min_db): the warn levels of `tune_phase_margin_deg` / `tune_gain_margin_db`."""
    return float(T["tune_phase_margin_deg"]["warn"]), float(T["tune_gain_margin_db"]["warn"])


def meets_margins(m):
    """True when a `margins()` result keeps both limits on the parametric fit. A loop with
    no unity crossing (no PM) or no -180 deg crossing (no GM) passes that term."""
    pm_min, gm_min = margin_limits()
    return (m["pm_deg"] is None or m["pm_deg"] >= pm_min) and (m["gm_db"] is None or m["gm_db"] >= gm_min)


def _with(gains, **kw):
    import dataclasses
    return dataclasses.replace(gains, **kw)


def margin_ceilings(plant, gains, d_for_p=None, p_for_d=None):
    """([Ceiling], [note]): the largest rate P and D whose full loop `C(z) G` still keeps
    `margin_limits()` on the identified plant.

    Replaces `ceilings_from_plant` in fusion. Heli AutoTune's -161 deg rule sizes P as if
    it were the only term, so on a multicopter with D it ignores D's phase lead:
    on brisket-t1.bin (Brisket) it put the roll P ceiling at 0.138 while P 0.135
    flew with PM 48.6 deg / GM 8.9 dB.

    P is scanned with I following it at the log's own I/P ratio and D at `d_for_p` (the D
    that would be applied with it, default the log's). D is scanned second, with P (and I
    at the same ratio) held at `p_for_d` (default the log's P), lowered to the P ceiling
    when that binds - the P it would be applied with.
    The ceiling is the upper edge of the first passing interval of a log-spaced scan over
    `margin_ceiling_span` x the current gain, refined by bisection. No failure inside the
    span gives no ceiling; no passing value gives none either, and a note says so.
    Each Ceiling has `includes_margin=True`: fusion clips to it, not to 0.4 x it.
    """
    g0 = gains
    axis = getattr(gains, "axis", None) or "roll"
    stem = RAT_STEM.get(axis, RAT_STEM["roll"])
    K = IDENT_CONSTANTS
    lo_mul, hi_mul = K["margin_ceiling_span"]["value"]
    npts, nbis = int(K["margin_ceiling_points"]["value"]), int(K["margin_ceiling_bisect"]["value"])
    pm_min, gm_min = margin_limits()
    ratio_i = (g0.rat_i / g0.rat_p) if g0.rat_p > 0 else 0.0
    d_p = float(g0.rat_d if d_for_p is None else d_for_p)
    out, notes = [], []

    p_d = float(g0.rat_p if p_for_d is None else p_for_d)

    def gains_for(name, x):
        if name == "P":
            return _with(g0, rat_p=float(x), rat_i=float(x) * ratio_i, rat_d=d_p)
        return _with(g0, rat_p=p_d, rat_i=p_d * ratio_i, rat_d=float(x))

    for name, x0 in (("P", g0.rat_p), ("D", g0.rat_d)):
        if not x0 or x0 <= 0:
            continue
        xs = np.geomspace(lo_mul * x0, hi_mul * x0, npts)
        ok = [meets_margins(margins(plant, gains_for(name, x))) for x in xs]
        if not any(ok):
            notes.append(f"margin ceiling {stem}{name}: no value in {lo_mul:g}-{hi_mul:g} x {x0:.4g} keeps "
                         f"PM >= {pm_min:g} deg and GM >= {gm_min:g} dB"
                         + (f" with D {d_p:.4g}" if name == "P" else f" with P {p_d:.4g}"))
            continue
        i0 = ok.index(True)
        i1 = next((i for i in range(i0 + 1, npts) if not ok[i]), None)
        if i1 is None:
            continue                                        # no ceiling below hi_mul x current
        a, b = float(xs[i1 - 1]), float(xs[i1])
        for _ in range(nbis):
            mid = math.sqrt(a * b)
            if meets_margins(margins(plant, gains_for(name, mid))):
                a = mid
            else:
                b = mid
        m = margins(plant, gains_for(name, a))
        held = (dict(rat_i_over_p=ratio_i, rat_d=d_p) if name == "P" else dict(rat_p=p_d, rat_i=p_d * ratio_i))
        ev = dict(pm_deg=m["pm_deg"], gm_db=m["gm_db"], fc_hz=m["fc_hz"], pm_min_deg=pm_min, gm_min_db=gm_min,
                  held=held, scan=[lo_mul * x0, hi_mul * x0, npts], source=K["margin_ceiling_span"]["source"])
        out.append(Ceiling(param=stem + name, value=a, method=f"margin-{pm_min:g}deg-{gm_min:g}dB", evidence=ev,
                           includes_margin=True))
        if name == "P":
            p_d = min(p_d, a)
    return out, notes


# --------------------------------------------------------------------- virtual autotune

def virtual_autotune(plant, gains, aggr=None, gmbk=None, **kw):
    """`tunesim.autotune` on the identified plant: `tunesim.Plant(k, tau1, tau2, delay,
    loop_hz)` from the model, the log's gain set as the starting point with its own
    AGGR / GMBK / MIN_D / hover throttle (`aggr`, `gmbk` override; other keyword
    arguments pass through, e.g. `angle_max_deg`, `max_twitches`). Returns the engine's
    dict plus `method`, `plant` (the parameters used, with the quantised delay) and a
    compact `why`."""
    g = _gd(gains)
    axis = getattr(gains, "axis", None) or kw.pop("axis", "roll")
    kw.pop("axis", None)
    p = tunesim.Plant(plant.k, plant.tau1, plant.tau2, plant.delay, loop_hz=float(g["loop_hz"]))
    r = tunesim.autotune(p, g, aggr=aggr, gmbk=gmbk, axis=axis, **kw)
    a = r["constants"]["aggr"]
    ov = r["final_overshoot"]
    bo = r["final_bounce"]
    parts = [f"{r['n_twitches']} twitches", f"{len(r['steps_completed'])}/{len(tunesim.SEQUENCE)} steps"]
    parts.append(f"final overshoot {ov:+.3f} vs allowance {0.5 * a:.4f} (0.5 x AGGR)" if ov is not None else "no RATE_P_UP twitch")
    parts.append(f"final bounce {bo:.3f} vs AGGR {a:.3f}" if bo is not None else "no RATE_D_DOWN twitch")
    if r["aborted"]:
        parts.append(f"aborted: {r['aborted']}")
    if r["constants"].get("thst_hover_defaulted"):
        parts.append("hover throttle defaulted to 0.35")
    r["method"] = "virtual-autotune"
    r["plant"] = p.params()
    r["why"] = "; ".join(parts)
    return r


# ---------------------------------------------------------------------------- report

def _fmt(v, nd=3, unit=""):
    if v is None or (isinstance(v, float) and not np.isfinite(v)):
        return "n/a"
    return f"{v:.{nd}f}{unit}"


def describe(plant, m=None):
    """Deterministic note lines for the Section: the fit, its quality, the margins."""
    lines = []
    band = plant.band if plant.band and plant.band[0] is not None else None
    lines.append(f"plant fit: k = {plant.k:.4g} deg/s per unit output, tau1 = {plant.tau1:.4g} s, "
                 f"tau2 = {plant.tau2:.4g} s, delay = {plant.delay * 1e3:.2f} ms (transport delay excluding the loop-rate hold)")
    if band is not None:
        lines.append(f"identification band {band[0]:.1f}-{band[1]:.1f} Hz (coherence >= "
                     f"{T['tune_coherence']['fail']:.2f}), mean coherence {plant.coh_mean_band:.3f}, "
                     f"{plant.n_avg} Welch averages pooled; fit residual rms {plant.fit_rms_db:.2f} dB / "
                     f"{plant.fit_rms_deg:.1f} deg")
    lines.append(f"Bendat-Piersol random error of |G| at the crossover: {100.0 * plant.eps_mag_at_crossover:.1f} %")
    if m is not None:
        lines.append(f"open loop (fit): crossover {_fmt(m['fc_hz'], 2, ' Hz')}, phase margin {_fmt(m['pm_deg'], 1, ' deg')}, "
                     f"gain margin {_fmt(m['gm_db'], 2, ' dB')} at {_fmt(m['f180_hz'], 2, ' Hz')}")
        if m.get("np_in_band"):
            lines.append(f"open loop (measured, in band): crossover {_fmt(m['np_fc_hz'], 2, ' Hz')}, phase margin "
                         f"{_fmt(m['np_pm_deg'], 1, ' deg')}, gain margin {_fmt(m['np_gm_db'], 2, ' dB')} at "
                         f"{_fmt(m['np_f180_hz'], 2, ' Hz')}")
        elif band is not None:
            lines.append("open loop (measured): the unity-gain crossing lies outside the coherent band; "
                         "only the fit's margins are available")
    return lines


def plant_rows(plant):
    """`(headers, rows)` for `Section.table`: one row per fitted quantity."""
    band = plant.band if plant.band and plant.band[0] is not None else (None, None)
    rows = [
        ["k", f"{plant.k:.4g}", "deg/s per unit output", "DC gain of the fit"],
        ["tau1", f"{plant.tau1:.4g}", "s", "slow (aero) pole"],
        ["tau2", f"{plant.tau2:.4g}", "s", "fast (motor/ESC) pole"],
        ["delay", f"{plant.delay * 1e3:.2f}", "ms", "transport delay excluding the loop-rate hold"],
        ["band", f"{_fmt(band[0], 1)}-{_fmt(band[1], 1)}", "Hz", "coherence >= gate, contiguous from the lowest bin"],
        ["coherence (band mean)", f"{plant.coh_mean_band:.3f}", "-", "pooled, coherence x n_avg weighted"],
        ["n_avg", str(int(plant.n_avg)), "-", "Welch averages summed over segments"],
        ["fit rms", f"{plant.fit_rms_db:.2f} dB / {plant.fit_rms_deg:.1f} deg", "-", "model vs measured G over the band"],
        ["eps |G| at crossover", f"{100.0 * plant.eps_mag_at_crossover:.1f}", "%", "Bendat-Piersol random error"],
    ]
    return ["quantity", "value", "unit", "note"], rows


def margin_rows(m):
    """`(headers, rows)` for `Section.table`: the parametric and the in-band
    non-parametric open loop."""
    rows = [["fit x C(z)", _fmt(m["fc_hz"], 2), _fmt(m["pm_deg"], 1), _fmt(m["gm_db"], 2), _fmt(m["f180_hz"], 2)]]
    rows.append(["measured G x C(z), in band", _fmt(m.get("np_fc_hz"), 2), _fmt(m.get("np_pm_deg"), 1),
                 _fmt(m.get("np_gm_db"), 2), _fmt(m.get("np_f180_hz"), 2)])
    return ["open loop", "crossover Hz", "phase margin deg", "gain margin dB", "-180 deg at Hz"], rows
