"""PID tuning tier C: closed-loop step response, oscillation ceiling, and the bounded
step rules (docs/pid-tuning-plan.md section 2.4 "Step response" / "Oscillation", WP3).

Three measurements on one `AxisSignals` segment and one rule set on top of them:

* `step_response(sig)` - the PID-Analyzer / PIDReview Wiener deconvolution of
  `PIDx.Tar -> PIDx.Act` (the FLTT-filtered rate target to the gyro), frame by frame,
  cumulatively summed into the closed loop's unit step response. The constants are
  PID-Analyzer's (`tune.CONSTANTS["step_*"]`); the reported curve is the mean over
  frames (PIDReview) and the per-frame stack gives `consistency`.
* `step_metrics(t, resp, aggr)` - latency, rise, peak, overshoot, bounce-back, settling
  and steady state on that curve, scored against AutoTune's own criteria: the overshoot
  allowance is `0.5 x AGGR`, the bounce-back criterion `AGGR` (sources 1.5).
* `oscillation(sig)` - QuickTune's `SRate > QUIK_OSC_SMAX` ceiling test, the `Dmod`
  engagement fraction, and a limit-cycle search in the Welch PSD of the D term and of the
  tracking error; attribution to P or D by band-passed variance; Ziegler-Nichols figures
  from `Ku = current gain, Tu = period` as evidence only.
* `step_rules(metrics, osc, gains, aggr)` - bounded multiplicative adjustments with the
  AutoTune step sizes (`RP_STEP`, `RD_STEP`, the RATE_D_UP 10 %), QuickTune's x0.4 at a
  ceiling, and a +-25 % (`AUTOTUNE_GMBK`) cap on everything else. Every `why` names the
  measured number, the threshold and its source. An empty list means "no change indicated".

Rules this module follows (RULES.md 1-2): a `Refusal` is returned, never raised; every
number an algorithm uses is in `tune.CONSTANTS` or the local `CONSTANTS` below with a
source; every graded number is in `checks.T`; nothing here reads a log directly, so the
window and its method travel with the `AxisSignals`. Deterministic: no randomness.

Imports only numpy, scipy and `dflog.{checks, spectral, tune}`; never `analysis` or `cli`.
"""

from __future__ import annotations

import math

import numpy as np
from scipy import signal as _signal
from scipy.ndimage import gaussian_filter1d

from .checks import T
from .spectral import psd_welch, find_peaks_db
from .tune import CONSTANTS as TUNE_CONSTANTS, RAT_STEM, Ceiling, Refusal, StepResponse

__all__ = ["CONSTANTS", "step_response", "step_metrics", "oscillation", "step_rules", "describe"]

# -------------------------------------------------------------------- constants

_SRC_PLAN = "docs/pid-tuning-plan.md section 2.4 (oscillation / ceiling)"
_SRC_PLAN_WP3 = "docs/pid-tuning-plan.md section 5 WP3 (step rules)"
_SRC_PIDA = ("PID-Analyzer (Plasmatree) Trace.wiener_deconvolution / winstacker; "
             "reference/pid-tuning-sources.md section 7")
_SRC_PTB = "PIDtoolbox PTstepcalc.m; reference/pid-tuning-sources.md section 7"
_SRC_ZN = "Ziegler-Nichols limit-cycle rules; reference/pid-tuning-sources.md section 7.1"
_SRC_FPV = "fpvpidlab rule set; reference/pid-tuning-sources.md section 7"
_SRC_MEAS = ("measured on tests/tunesynth.py PLANT_5IN / PLANT_10IN with their AutoTune gains "
             "(WP3 report, 2026-09-17); INS_GYRO_FILTER is the prop-size proxy of the wiki and the "
             "Mission Planner initial-parameter calculator (sources section 6)")


def _c(value, source, note=""):
    return dict(value=value, source=source, note=note)


#: Constants this tier needs that `tune.CONSTANTS` does not carry. Same shape; WP6 merges
#: them into `alog schema`'s `tune_constants`.
CONSTANTS = {
    "step_pad_block":        _c(1024, _SRC_PIDA, "frames are zero-padded to a multiple of this before the FFT"),
    "step_latency_frac":     _c(0.5, _SRC_PTB, "latency = time to this fraction of the unit step"),
    "step_rise_lo_frac":     _c(0.1, _SRC_PTB, "rise time is measured from this fraction ..."),
    "step_rise_hi_frac":     _c(0.9, _SRC_PTB, "... to this fraction of the unit step"),
    "step_settle_frac":      _c(0.02, _SRC_PTB, "settling = last excursion beyond +-this of the unit step"),
    "step_ss_window_s":      _c((0.4, 0.5), _SRC_PLAN, "steady state = mean of the response over this span, s"),
    "step_ss_err_max":       _c(0.05, _SRC_FPV, "|steady state - 1| above this moves I toward P (fpvpidlab SS error > 5 %)"),
    "step_overshoot_low_ratio": _c(0.5, _SRC_PLAN_WP3, "a slow rise counts only when overshoot_ratio is below this"),
    "step_rise_slow_mul":    _c(2.0, _SRC_PLAN_WP3, "rise slower than this x the expected rise raises P"),
    "step_rise_expected_mul": _c(1.6, _SRC_MEAS, "expected 10-90 % rise, s = this / INS_GYRO_FILTER (Hz): "
                                                  "21 ms at 75 Hz (5 in), 38 ms at 42 Hz (10 in), 80 ms at 20 Hz"),
    "quik_pilot_input_delay_s": _c(4.0, "ArduPilot libraries/AP_Scripting/applets/VTOL-quicktune.lua PILOT_INPUT_DELAY; "
                                        "reference/pid-tuning-sources.md section 4",
                                   "SRate is judged only on samples this long after the last |Tar| >= step_min_target_dps: "
                                   "commanded transients slew the output too, and the SlewLimiter decays with a 1 s attack filter"),
    "osc_srate_min_quiet_s": _c(1.0, _SRC_PLAN, "fewer quiet seconds than this and the SRate test is not evaluated (NaN)"),
    "osc_band_hz":           _c((3.0, 40.0), _SRC_PLAN, "limit-cycle search band for the PSD of D and of Act - Tar"),
    "osc_nperseg_s":         _c(2.0, _SRC_PLAN, "Welch segment length, s"),
    "osc_prominence_db":     _c(10.0, _SRC_PLAN, "a PSD peak needs this prominence above the median floor to count"),
    "osc_attrib_band_frac":  _c(0.2, _SRC_PLAN, "P vs D attribution: band-passed variance within +-this x f_osc"),
    "osc_attrib_order":      _c(4, _SRC_PLAN, "Butterworth order of the attribution band-pass (sosfiltfilt)"),
    "zn_no_overshoot":       _c(dict(kp=0.2, ti=0.5, td=1.0 / 3.0), _SRC_ZN, "Kp = 0.2 Ku, Ti = 0.5 Tu, Td = Tu/3"),
    "zn_classic_pid":        _c(dict(kp=0.6, ti=0.5, td=0.125), _SRC_ZN, "Kp = 0.6 Ku, Ti = 0.5 Tu, Td = 0.125 Tu"),
}


def _tc(name):
    return TUNE_CONSTANTS[name]["value"]


def _lc(name):
    return CONSTANTS[name]["value"]


def _fin(x):
    try:
        return x is not None and math.isfinite(float(x))
    except (TypeError, ValueError):
        return False


def _fmt(x, digits=3, unit=""):
    if not _fin(x):
        return "n/a"
    return f"{float(x):.{digits}f}{unit}"


def _ms(x):
    return "n/a" if not _fin(x) else f"{float(x) * 1e3:.0f} ms"


# ---------------------------------------------------------------- step response

def _to_mask(x):
    """PID-Analyzer `to_mask`: rescale to [0, 1]."""
    x = np.asarray(x, dtype=float)
    lo, hi = float(x.min()), float(x.max())
    if hi <= lo:
        return np.zeros_like(x)
    return (x - lo) / (hi - lo)


def _regulariser(npad, fs, cut_hz):
    """PID-Analyzer's `sn`: 10 below `cut_hz`, ~1e-8 above, the edge Gaussian-smoothed with
    sigma = (bins below the cut) / 6. `1/sn` is added to |H|^2 in the Wiener filter."""
    freq = np.abs(np.fft.fftfreq(npad, 1.0 / fs))
    sn = _to_mask(np.clip(freq, cut_hz - 1e-9, cut_hz))
    len_lpf = float(np.sum(1.0 - sn))
    if len_lpf > 0:
        sn = _to_mask(gaussian_filter1d(sn, len_lpf / 6.0))
    return 10.0 * (1.0 - sn + 1e-9)


def _frame_geometry(fs):
    flen = int(round(_tc("step_frame_s") * fs))
    rlen = int(round(_tc("step_response_s") * fs))
    shift = max(1, flen // int(_tc("step_overlap")))
    block = int(_lc("step_pad_block"))
    npad = flen + (block - flen % block)              # PID-Analyzer: pad = 1024 - (len % 1024)
    return flen, rlen, shift, npad


def _stack(x, flen, shift, wins):
    idx = np.arange(wins)[:, None] * shift + np.arange(flen)[None, :]
    return x[idx]


def deconvolve_frames(tar_frames, act_frames, fs):
    """Wiener-deconvolve every (input, output) frame pair into the first
    `step_response_s` of its impulse response and cumulatively sum it: the unit step
    response per frame (frames x rlen). Each frame is detrended (mean removed) and Hann
    windowed first. PID-Analyzer `wiener_deconvolution` + `stack_response`."""
    tar_frames = np.asarray(tar_frames, dtype=float)
    act_frames = np.asarray(act_frames, dtype=float)
    flen = tar_frames.shape[1]
    _f, rlen, _s, npad = _frame_geometry(fs)
    rlen = min(rlen, flen)
    win = np.hanning(flen)
    inp = (tar_frames - tar_frames.mean(axis=1, keepdims=True)) * win
    outp = (act_frames - act_frames.mean(axis=1, keepdims=True)) * win
    H = np.fft.fft(inp, n=npad, axis=-1)
    G = np.fft.fft(outp, n=npad, axis=-1)
    sn = _regulariser(npad, fs, _tc("step_cut_hz"))
    Hc = np.conj(H)
    imp = np.real(np.fft.ifft(G * Hc / (H * Hc + 1.0 / sn), axis=-1))[:, :rlen]
    return np.cumsum(imp, axis=1)


def step_response(sig):
    """The closed loop's unit step response from `sig.tar` (the FLTT-filtered target,
    deg/s) to `sig.act` (the gyro, deg/s), PID-Analyzer / PIDReview method.

    Frames of `step_frame_s` (1.0 s) stepped by 1/`step_overlap` (1/16) of a frame,
    detrended, Hann-windowed, Wiener-deconvolved with the 25 Hz regulariser
    (`step_cut_hz`), cumulatively summed over the first `step_response_s` (0.5 s).
    Frames whose max |target| is below `step_min_target_dps` (20 deg/s) are dropped and
    counted in `n_dropped_low`; frames with a non-finite sample are dropped and counted in
    `metrics["n_dropped_nonfinite"]`. Fewer than `T["tune_min_frames"]["fail"]` frames
    left is a `Refusal("NO_EXCITATION")`, returned not raised.

    Returns a `StepResponse`: `t` (s from the step), `mean` (mean over frames, PIDReview),
    `frames` (the stack), `n_frames`, `n_dropped_low`, `metrics` (`step_metrics` of the
    mean with the gain set's AGGR, plus `n_frames_total`, `n_frames_high` above
    `step_split_dps`, `n_dropped_nonfinite`, `fs`), `consistency` = clip(1 - IQR(peak over
    frames) / median(peak), 0, 1). Sample counts are derived from `sig.fs`, so any rate
    the WP1 gate admitted works.
    """
    fs = float(sig.fs)
    tar = np.asarray(sig.tar, dtype=float)
    act = np.asarray(sig.act, dtype=float)
    n = min(len(tar), len(act))
    tar, act = tar[:n], act[:n]
    flen, rlen, shift, _npad = _frame_geometry(fs)
    min_frames = int(T["tune_min_frames"]["fail"])
    warn_frames = int(T["tune_min_frames"]["warn"])
    min_tar = float(_tc("step_min_target_dps"))
    duration = float(sig.t[-1] - sig.t[0]) if len(sig.t) > 1 else 0.0

    def refuse(n_ok, n_total, n_low, n_nan):
        return Refusal(
            "NO_EXCITATION",
            f"{n_ok} frame(s) above {min_tar:.0f} deg/s in a {duration:.0f} s window "
            f"({sig.source} at {fs:.0f} Hz: {n_total} frames of {_tc('step_frame_s'):.1f} s, "
            f"{n_low} below {min_tar:.0f} deg/s, {n_nan} non-finite); {min_frames} needed "
            f"({warn_frames} recommended)",
            "fly the tuning profile (docs/pid-tuning-plan.md section 7): 30 s hover then 60 s of "
            "sharp stick inputs (+-15-20 deg, quick release) on each axis",
            axis=sig.axis, log_name=sig.log_name)

    wins = n // shift - int(_tc("step_overlap"))       # PID-Analyzer winstacker
    if n < flen or wins < 1:
        return refuse(0, 0, 0, 0)
    wins = min(wins, (n - flen) // shift + 1)
    tf, af = _stack(tar, flen, shift, wins), _stack(act, flen, shift, wins)
    finite = np.isfinite(tf).all(axis=1) & np.isfinite(af).all(axis=1)
    max_in = np.where(finite, np.abs(np.where(finite[:, None], tf, 0.0)).max(axis=1), 0.0)
    low = finite & (max_in < min_tar)
    keep = finite & ~low
    n_low, n_nan = int(low.sum()), int((~finite).sum())
    n_ok = int(keep.sum())
    if n_ok < min_frames:
        return refuse(n_ok, wins, n_low, n_nan)

    frames = deconvolve_frames(tf[keep], af[keep], fs)
    t = np.arange(frames.shape[1]) / fs
    mean = frames.mean(axis=0)
    peaks = frames.max(axis=1)
    med = float(np.median(peaks))
    iqr = float(np.percentile(peaks, 75) - np.percentile(peaks, 25))
    consistency = float(np.clip(1.0 - iqr / med, 0.0, 1.0)) if med > 0 else 0.0

    metrics = step_metrics(t, mean, sig.gains.aggr)
    metrics.update(n_frames_total=int(wins), n_frames_high=int((max_in[keep] >= _tc("step_split_dps")).sum()),
                   n_dropped_nonfinite=n_nan, fs=fs, max_target_dps=float(max_in.max()))
    return StepResponse(t=t, mean=mean, frames=frames, n_frames=n_ok, n_dropped_low=n_low,
                        metrics=metrics, consistency=consistency)


# ---------------------------------------------------------------- step metrics

def _cross(t, r, level):
    """Time of the first upward crossing of `level`, linearly interpolated; NaN if never."""
    above = np.flatnonzero(r >= level)
    if above.size == 0:
        return float("nan")
    i = int(above[0])
    if i == 0:
        return float(t[0])
    r0, r1 = r[i - 1], r[i]
    if r1 == r0:
        return float(t[i])
    return float(t[i - 1] + (level - r0) / (r1 - r0) * (t[i] - t[i - 1]))


def step_metrics(t, resp, aggr):
    """Metrics of a unit step response (levels relative to 1.0, the target), scored
    against AutoTune's criteria.

    latency_s (to 50 %), rise_s (10-90 %), peak, peak_t, overshoot = peak - 1, postmin
    (minimum after the peak), bounce = (peak - postmin) / peak, settle_s (last excursion
    beyond +-2 %; NaN when not settled by the end), ss (mean over 0.4-0.5 s),
    overshoot_ratio = overshoot / (0.5 x aggr), bounce_ratio = bounce / aggr. Every value
    is a float and NaN when it cannot be measured; deterministic.
    """
    keys = ("latency_s", "rise_s", "peak", "peak_t", "overshoot", "postmin", "bounce", "settle_s", "ss",
            "overshoot_ratio", "bounce_ratio")
    out = {k: float("nan") for k in keys}
    out["aggr"] = float(aggr) if _fin(aggr) else float("nan")
    t = np.asarray(t, dtype=float)
    r = np.asarray(resp, dtype=float)
    m = np.isfinite(t) & np.isfinite(r)
    if m.sum() < 3:
        return out
    t, r = t[m], r[m]
    ip = int(np.argmax(r))
    peak = float(r[ip])
    postmin = float(r[ip:].min())
    out.update(peak=peak, peak_t=float(t[ip]), overshoot=peak - 1.0, postmin=postmin,
               bounce=(peak - postmin) / peak if peak > 0 else float("nan"))
    out["latency_s"] = _cross(t, r, _lc("step_latency_frac"))
    lo, hi = _cross(t, r, _lc("step_rise_lo_frac")), _cross(t, r, _lc("step_rise_hi_frac"))
    out["rise_s"] = hi - lo if _fin(lo) and _fin(hi) else float("nan")
    tol = float(_lc("step_settle_frac"))
    outside = np.flatnonzero(np.abs(r - 1.0) > tol)
    if outside.size == 0:
        out["settle_s"] = float(t[0])
    elif outside[-1] + 1 < len(t):
        out["settle_s"] = float(t[outside[-1] + 1])
    s0, s1 = _lc("step_ss_window_s")
    win = (t >= s0) & (t <= s1)
    out["ss"] = float(r[win].mean()) if win.any() else float(r[-max(1, len(r) // 10):].mean())
    a = out["aggr"]
    if _fin(a) and a > 0:
        out["overshoot_ratio"] = out["overshoot"] / (float(_tc("autotune_overshoot_aggr_scale")) * a)
        out["bounce_ratio"] = out["bounce"] / a
    return out


# ----------------------------------------------------------------- oscillation

def _best_peak(x, fs, lo, hi, nperseg, prom_db):
    f, p = psd_welch(x, fs, nperseg=nperseg)
    pk = find_peaks_db(f, p, n=3, fmin=lo, fmax=hi, min_prominence_db=prom_db)
    pk = [q for q in pk if q["prominence_db"] >= prom_db]
    return (max(pk, key=lambda q: q["prominence_db"]) if pk else None), (f, p)


def _bandpass_var(x, fs, lo, hi, order):
    nyq = fs / 2.0
    hi = min(hi, 0.95 * nyq)
    if not (0 < lo < hi):
        return float("nan")
    sos = _signal.butter(int(order), [lo, hi], btype="bandpass", fs=fs, output="sos")
    x = np.asarray(x, dtype=float)
    x = np.where(np.isfinite(x), x, 0.0)
    return float(np.var(_signal.sosfiltfilt(sos, x)))


def quiet_mask(t, tar, fs, delay_s, min_target_dps, lp_hz):
    """True where the sample is at least `delay_s` after the last commanded rate of
    `min_target_dps` or more (and no such sample precedes it): sticks centred long enough
    for the SlewLimiter's 1 s attack filter to have decayed, QuickTune's
    PILOT_INPUT_DELAY. "Commanded" is `tar` low-passed at `lp_hz` (the bottom of the
    oscillation search band): in a limit cycle the angle loop feeds the ring back into
    `Tar` itself, and that must not read as stick input."""
    t = np.asarray(t, dtype=float)
    x = np.asarray(tar, dtype=float)
    x = np.where(np.isfinite(x), x, 0.0)
    if 0 < lp_hz < fs / 2.0 and len(x) > 24:
        sos = _signal.butter(2, lp_hz, btype="lowpass", fs=fs, output="sos")
        x = _signal.sosfiltfilt(sos, x)
    busy = np.abs(x) >= min_target_dps
    idx = np.flatnonzero(busy)
    if idx.size == 0:
        return np.ones(len(t), dtype=bool)
    # time of the last busy sample at or before each sample
    last_busy_t = np.full(len(t), -np.inf)
    last_busy_t[idx] = t[idx]
    last_busy_t = np.maximum.accumulate(last_busy_t)
    return (t - last_busy_t) >= delay_s


def oscillation(sig):
    """QuickTune's ceiling test and a limit-cycle search on one segment.

    Returns a dict: `srate_p95`, `srate_max` (PIDx.SRate, normalised output/s, over the
    **quiet** samples - at least `quik_pilot_input_delay_s` after the last commanded
    rate (Tar low-passed below the search band) of `step_min_target_dps` or more, as
    QuickTune only judges SRate with the sticks centred (`quiet_mask`);
    `srate_quiet_s` says how much of the segment that was and `srate_p95_all` is the
    whole-segment figure; NaN when SRate is not logged or the quiet part is shorter than
    `osc_srate_min_quiet_s`), `dmod_min`, `dmod_engaged_pct` (percent of samples with
    Dmod < 1), `peaks` per source (`error` = Act - Tar, `d` = the D term) from a Welch PSD
    (`osc_nperseg_s` segments) over `osc_band_hz`, `f_osc` / `period_s` /
    `prominence_db` of the `error` peak at or above `osc_prominence_db` (None when there
    is none; the D term's spectrum is kd x s x FLTD applied to gyro noise, humped by
    construction, so it only corroborates: `d_peak_agrees`), `at_ceiling` (quiet SRate
    p95 above `T["tune_srate_osc"]["warn"]` or a limit-cycle peak), `attribution`
    "P" | "D" | None by the larger band-passed variance of the P and D terms within
    `osc_attrib_band_frac` of `f_osc` (the whole band when only SRate fired; None when
    the terms are not logged), `ceilings` (a `Ceiling` per attributed gain with the
    numbers and, when a period exists and the attribution is P, Ziegler-Nichols figures
    as evidence only), `note`.
    """
    fs = float(sig.fs)
    warn = float(T["tune_srate_osc"]["warn"])
    lo, hi = _lc("osc_band_hz")
    hi = min(float(hi), 0.95 * fs / 2.0)
    prom_db = float(_lc("osc_prominence_db"))
    nperseg = int(round(_lc("osc_nperseg_s") * fs))
    delay = float(_lc("quik_pilot_input_delay_s"))
    min_quiet = float(_lc("osc_srate_min_quiet_s"))
    out = dict(srate_p95=float("nan"), srate_max=float("nan"), srate_p95_all=float("nan"),
               srate_quiet_s=0.0, srate_quiet_delay_s=delay, dmod_min=float("nan"),
               dmod_engaged_pct=float("nan"), band_hz=(float(lo), float(hi)), peaks={},
               f_osc=None, period_s=None, prominence_db=None, peak_source=None, d_peak_agrees=None,
               srate_warn=warn, srate_source=T["tune_srate_osc"]["source"],
               at_ceiling=False, attribution=None, attribution_band_hz=None,
               var_p_band=float("nan"), var_d_band=float("nan"), ceilings=[], note="")
    quiet = quiet_mask(sig.t, sig.tar, fs, delay, float(_tc("step_min_target_dps")), float(lo))
    out["srate_quiet_s"] = float(quiet.sum() / fs)
    if sig.srate is not None:
        s = np.asarray(sig.srate, dtype=float)
        ok = np.isfinite(s)
        if ok.any():
            out["srate_p95_all"] = float(np.percentile(s[ok], 95))
        sq = s[ok & quiet]
        if sq.size and out["srate_quiet_s"] >= min_quiet:
            out["srate_p95"], out["srate_max"] = float(np.percentile(sq, 95)), float(sq.max())
    if sig.dmod is not None:
        d = np.asarray(sig.dmod, dtype=float)
        d = d[np.isfinite(d)]
        if d.size:
            out["dmod_min"], out["dmod_engaged_pct"] = float(d.min()), float(100.0 * np.mean(d < 1.0))

    err = np.asarray(sig.act, dtype=float) - np.asarray(sig.tar, dtype=float)
    sources = [("error", err)] + ([("d", sig.d)] if sig.d is not None else [])
    for name, x in sources:
        x = np.asarray(x, dtype=float)
        if np.isfinite(x).sum() < 2 * nperseg or hi <= lo:
            out["peaks"][name] = None
            continue
        pk, _spec = _best_peak(x, fs, lo, hi, nperseg, prom_db)
        out["peaks"][name] = pk
    best = out["peaks"].get("error")
    if best is not None:
        out.update(f_osc=float(best["freq_hz"]), period_s=1.0 / float(best["freq_hz"]),
                   prominence_db=float(best["prominence_db"]), peak_source="error")
        dpk = out["peaks"].get("d")
        out["d_peak_agrees"] = (bool(abs(dpk["freq_hz"] / best["freq_hz"] - 1.0) <= _lc("osc_attrib_band_frac"))
                                if dpk is not None else None)

    srate_osc = _fin(out["srate_p95"]) and out["srate_p95"] > warn
    out["at_ceiling"] = bool(srate_osc or best is not None)

    if sig.p is not None and sig.d is not None:
        frac = float(_lc("osc_attrib_band_frac"))
        if out["f_osc"] is not None:
            band = (out["f_osc"] * (1.0 - frac), out["f_osc"] * (1.0 + frac))
        else:
            band = (float(lo), float(hi))
        order = _lc("osc_attrib_order")
        vp, vd = _bandpass_var(sig.p, fs, band[0], band[1], order), _bandpass_var(sig.d, fs, band[0], band[1], order)
        out.update(var_p_band=vp, var_d_band=vd, attribution_band_hz=(float(band[0]), float(band[1])))
        if _fin(vp) and _fin(vd) and out["at_ceiling"]:
            out["attribution"] = "P" if vp >= vd else "D"

    srate_txt = (f"SRate p95 {_fmt(out['srate_p95'], 2)} over {out['srate_quiet_s']:.1f} s of quiet samples "
                 f"(QUIK_OSC_SMAX {warn:g})" if _fin(out["srate_p95"]) else
                 f"SRate not evaluated ({out['srate_quiet_s']:.1f} s of samples {delay:g} s clear of stick input"
                 + ("" if sig.srate is not None else "; SRate not logged") + ")")
    if not out["at_ceiling"]:
        out["note"] = (f"no ceiling found: {srate_txt}, Dmod min {_fmt(out['dmod_min'], 3)}, no Act - Tar PSD peak "
                       f">= {prom_db:g} dB in {lo:g}-{hi:g} Hz (a fact about this flight, not a pass)")
        return out

    attr = out["attribution"] or "P"
    g = sig.gains
    current = float(g.rat_p if attr == "P" else g.rat_d)
    param = RAT_STEM[sig.axis] + attr
    method = "limit-cycle" if best is not None else "srate-ceiling"
    evidence = dict(srate_p95=out["srate_p95"], srate_max=out["srate_max"], srate_p95_all=out["srate_p95_all"],
                    srate_quiet_s=out["srate_quiet_s"], srate_warn=warn,
                    srate_source=out["srate_source"], dmod_min=out["dmod_min"],
                    dmod_engaged_pct=out["dmod_engaged_pct"], f_osc=out["f_osc"], period_s=out["period_s"],
                    prominence_db=out["prominence_db"], peak_source=out["peak_source"],
                    d_peak=out["peaks"].get("d"), d_peak_agrees=out["d_peak_agrees"],
                    prominence_required_db=prom_db, band_hz=out["band_hz"],
                    attribution=out["attribution"], attribution_assumed=out["attribution"] is None,
                    attribution_band_hz=out["attribution_band_hz"],
                    var_p_band=out["var_p_band"], var_d_band=out["var_d_band"],
                    quik_gain_margin=_tc("quik_gain_margin"),
                    quik_recommendation=current * (1.0 - _tc("quik_gain_margin")))
    if out["period_s"] is not None and attr == "P":
        ku, tu = current, out["period_s"]
        zn = {}
        for key in ("zn_no_overshoot", "zn_classic_pid"):
            c = _lc(key)
            kp = c["kp"] * ku
            zn[key[3:]] = dict(kp=kp, ti_s=c["ti"] * tu, td_s=c["td"] * tu, ki=kp / (c["ti"] * tu), kd=kp * c["td"] * tu)
        evidence["ziegler_nichols"] = dict(ku=ku, tu_s=tu, source=CONSTANTS["zn_no_overshoot"]["source"],
                                           note="evidence only, not the recommendation", **zn)
    out["ceilings"] = [Ceiling(param=param, value=current, method=method, evidence=evidence)]
    parts = []
    if srate_osc:
        parts.append(f"SRate p95 {out['srate_p95']:.2f} > QUIK_OSC_SMAX {warn:g} over {out['srate_quiet_s']:.1f} s of quiet samples")
    else:
        parts.append(srate_txt)
    if best is not None:
        parts.append(f"limit cycle at {out['f_osc']:.1f} Hz ({out['prominence_db']:.1f} dB in Act - Tar"
                     + (f", D term agrees" if out["d_peak_agrees"] else "") + ")")
    who = (f"attributed to {attr} (band-passed variance P {_fmt(out['var_p_band'], 6)} vs D {_fmt(out['var_d_band'], 6)})"
           if out["attribution"] else "attribution unavailable (P/D terms not logged), P assumed")
    out["note"] = f"at the oscillation ceiling: {'; '.join(parts)}; {who}; {param} = {current:g} is a ceiling"
    return out


# ------------------------------------------------------------------ step rules

def _gain(gains, name, default=None):
    if isinstance(gains, dict):
        return gains.get(name, default)
    return getattr(gains, name, default)


def step_rules(metrics, osc, gains, aggr=None):
    """Bounded multiplicative adjustments from the step metrics and the oscillation
    result, re-based on AutoTune's numbers (plan WP3).

    * at the ceiling: the attributed gain x (1 - QUIK_GAIN_MARGIN) = x0.4, overriding every
      other rule (a ringing loop's step metrics are not trustworthy); the +-25 % cap does
      not apply to a measured ceiling.
    * overshoot_ratio > `T["tune_overshoot_ratio"]["warn"]`: D x 1.10 (RATE_D_UP) when
      bounce_ratio is still below its warn level and D stays below RD_MAX, else
      P x 0.95 (RP_STEP) per unit of excess.
    * bounce_ratio > `T["tune_bounce_ratio"]["warn"]`: D x 0.95 (RD_STEP, RATE_D_DOWN).
    * slow: rise > `step_rise_slow_mul` x the expected rise (`step_rise_expected_mul` /
      INS_GYRO_FILTER) or peak < 1 - D_UP_DOWN_MARGIN (AutoTune's "P += 5 %" test), with
      overshoot_ratio < `step_overshoot_low_ratio`: P x 1.05 (RP_STEP).
    * |ss - 1| > `step_ss_err_max`: I toward P x the AutoTune final ratio (1.0; yaw 0.1).
    * every non-ceiling change is capped at +-AUTOTUNE_GMBK (25 %) per parameter.

    `gains` is a `GainSet` (or a dict with the same names and `axis`); `aggr` overrides
    its AGGR. Returns a list of dict(param, current, value, change_pct, why), empty when
    everything is within band (the caller reports "no change indicated"). Deterministic.
    """
    axis = _gain(gains, "axis")
    stem = RAT_STEM[axis]
    cur = {"P": float(_gain(gains, "rat_p")), "I": float(_gain(gains, "rat_i")), "D": float(_gain(gains, "rat_d"))}
    gyro = float(_gain(gains, "gyro_filter") or 0.0)
    aggr = float(aggr if aggr is not None else (_gain(gains, "aggr") or _tc("autotune_aggr_default")))
    cap = float(_tc("autotune_gmbk_default"))
    cap_src = TUNE_CONSTANTS["autotune_gmbk_default"]["source"]
    rules = []

    if osc and osc.get("at_ceiling"):
        attr = osc.get("attribution") or "P"
        factor = 1.0 - float(_tc("quik_gain_margin"))
        why = (f"{osc.get('note', 'at the oscillation ceiling')}; QuickTune margin: gain x {factor:g} "
               f"(QUIK_GAIN_MARGIN {_tc('quik_gain_margin') * 100:.0f} %, source {TUNE_CONSTANTS['quik_gain_margin']['source']}); "
               f"SRate threshold {osc.get('srate_warn', T['tune_srate_osc']['warn'])} (source {T['tune_srate_osc']['source']}). "
               f"Overrides the step rules; the +-{cap * 100:.0f} % cap does not apply to a measured ceiling")
        if cur[attr] > 0:
            rules.append(dict(param=stem + attr, current=cur[attr], value=cur[attr] * factor,
                              change_pct=(factor - 1.0) * 100.0, why=why))
        return rules

    m = metrics or {}
    osr, br = m.get("overshoot_ratio"), m.get("bounce_ratio")
    rise, peak, ss = m.get("rise_s"), m.get("peak"), m.get("ss")
    thr_os, thr_b = float(T["tune_overshoot_ratio"]["warn"]), float(T["tune_bounce_ratio"]["warn"])
    rp_step, rd_step, rd_up = _tc("autotune_rp_step"), _tc("autotune_rd_step"), _tc("autotune_rd_up_step")
    rd_max = _tc("autotune_rd_max")
    factors = {"P": 1.0, "D": 1.0}
    whys = {"P": [], "D": []}

    if _fin(osr) and osr > thr_os:
        excess = osr - thr_os
        base = (f"overshoot {m['overshoot'] * 100:.1f} % = {osr:.2f} x the AutoTune allowance 0.5 x AGGR "
                f"({0.5 * aggr * 100:.2f} %; warn above {thr_os:g}, source {T['tune_overshoot_ratio']['source']})")
        d_up = cur["D"] * (1.0 + rd_up)
        if _fin(br) and br < thr_b and cur["D"] > 0 and d_up <= rd_max:
            factors["D"] *= 1.0 + rd_up
            whys["D"].append(f"{base}; bounce ratio {br:.2f} is below {thr_b:g}, so damping is short: D x "
                             f"{1 + rd_up:.2f} (RATE_D_UP step, source {TUNE_CONSTANTS['autotune_rd_up_step']['source']}) "
                             f"chosen over P x {1 - rp_step:.2f}")
        else:
            n = max(1, int(math.ceil(excess - 1e-9)))
            factors["P"] *= (1.0 - rp_step) ** n
            reason = ("bounce ratio already at or above its warn level" if _fin(br) and br >= thr_b else
                      "D cannot be raised" if cur["D"] <= 0 else
                      f"D x {1 + rd_up:.2f} would exceed RD_MAX {rd_max:g}" if d_up > rd_max else
                      "bounce ratio unmeasured")
            whys["P"].append(f"{base}; {reason}, so P x {1 - rp_step:.2f}^{n} = {factors['P']:.3f} "
                             f"({n} RP_STEP per unit of excess, source {TUNE_CONSTANTS['autotune_rp_step']['source']})")

    if _fin(br) and br > thr_b and cur["D"] > 0:
        factors["D"] *= 1.0 - rd_step
        whys["D"].append(f"bounce-back {m['bounce'] * 100:.1f} % = {br:.2f} x AGGR ({aggr * 100:.1f} %; warn above "
                         f"{thr_b:g}, source {T['tune_bounce_ratio']['source']}): D x {1 - rd_step:.2f} "
                         f"(RATE_D_DOWN step RD_STEP, source {TUNE_CONSTANTS['autotune_rd_step']['source']})")

    expected = float(_lc("step_rise_expected_mul")) / gyro if gyro > 0 else float("nan")
    slow_mul = float(_lc("step_rise_slow_mul"))
    margin = float(_tc("autotune_d_up_down_margin"))
    low_os = not _fin(osr) or osr < float(_lc("step_overshoot_low_ratio"))
    slow_rise = _fin(rise) and _fin(expected) and rise > slow_mul * expected
    low_peak = _fin(peak) and peak < 1.0 - margin
    if (slow_rise or low_peak) and low_os:
        factors["P"] *= 1.0 + rp_step
        if slow_rise:
            reason = (f"rise 10-90 % {rise * 1e3:.0f} ms > {slow_mul:g} x the expected {expected * 1e3:.0f} ms "
                      f"({_lc('step_rise_expected_mul'):g} / INS_GYRO_FILTER {gyro:g} Hz, source "
                      f"{CONSTANTS['step_rise_expected_mul']['source']})")
        else:
            reason = (f"peak {peak:.2f} < {1 - margin:.2f} of the target (AutoTune D_UP_DOWN_MARGIN "
                      f"{margin:g}, source {TUNE_CONSTANTS['autotune_d_up_down_margin']['source']})")
        whys["P"].append(f"{reason} with overshoot ratio {_fmt(osr, 2)} < {_lc('step_overshoot_low_ratio'):g}: "
                         f"P x {1 + rp_step:.2f} (RP_STEP, source {TUNE_CONSTANTS['autotune_rp_step']['source']})")

    for k in ("P", "D"):
        f = factors[k]
        if f == 1.0 or cur[k] <= 0:
            continue
        capped = float(np.clip(f, 1.0 - cap, 1.0 + cap))
        why = "; ".join(whys[k])
        if capped != f:
            why += f"; total change capped at +-{cap * 100:.0f} % (AUTOTUNE_GMBK, source {cap_src})"
        rules.append(dict(param=stem + k, current=cur[k], value=cur[k] * capped,
                          change_pct=(capped - 1.0) * 100.0, why=why))

    ss_max = float(_lc("step_ss_err_max"))
    if _fin(ss) and abs(ss - 1.0) > ss_max:
        ratio_key = "autotune_yaw_pi_ratio_final" if axis == "yaw" else "autotune_pi_ratio_final"
        ratio = float(_tc(ratio_key))
        target = cur["P"] * ratio
        if cur["I"] > 0 and abs(target / cur["I"] - 1.0) > 1e-3:
            f = float(np.clip(target / cur["I"], 1.0 - cap, 1.0 + cap))
            why = (f"steady state {ss:.3f} of the unit step (0.4-0.5 s) is off by {abs(ss - 1) * 100:.1f} % "
                   f"(above {ss_max * 100:.0f} %, source {CONSTANTS['step_ss_err_max']['source']}): I toward "
                   f"P x {ratio:g} = {target:.4g} ({ratio_key.upper()}, source {TUNE_CONSTANTS[ratio_key]['source']})")
            if abs(f - target / cur["I"]) > 1e-12:
                why += f"; total change capped at +-{cap * 100:.0f} % (AUTOTUNE_GMBK, source {cap_src})"
            rules.append(dict(param=stem + "I", current=cur["I"], value=cur["I"] * f,
                              change_pct=(f - 1.0) * 100.0, why=why))
    return rules


# -------------------------------------------------------------------- describe

def describe(step, metrics=None, osc=None):
    """Short deterministic note lines for the report: the frame tally and consistency,
    the metrics with AutoTune's ratios, and the oscillation verdict."""
    lines = []
    if isinstance(step, Refusal):
        lines.append(f"step response: {step.code} - {step.message}")
    elif step is not None:
        lines.append(f"step response: {step.n_frames} frames ({step.n_dropped_low} dropped below "
                     f"{_tc('step_min_target_dps'):.0f} deg/s), consistency {step.consistency:.2f}, "
                     f"{_tc('step_frame_s'):.1f} s frames / {_tc('step_response_s'):.1f} s response, "
                     f"{_tc('step_cut_hz'):.0f} Hz regulariser (PID-Analyzer)")
        if metrics is None:
            metrics = step.metrics
    m = metrics or {}
    if m:
        aggr = m.get("aggr")
        lines.append(f"latency {_ms(m.get('latency_s'))}, rise 10-90 % {_ms(m.get('rise_s'))}, peak "
                     f"{_fmt(m.get('peak'), 3)} at {_ms(m.get('peak_t'))}, overshoot "
                     f"{_fmt(m.get('overshoot', float('nan')) * 100 if _fin(m.get('overshoot')) else None, 1, ' %')} "
                     f"(ratio {_fmt(m.get('overshoot_ratio'), 2)} vs 0.5 x AGGR {_fmt(0.5 * aggr * 100 if _fin(aggr) else None, 2, ' %')}), "
                     f"bounce {_fmt(m.get('bounce') * 100 if _fin(m.get('bounce')) else None, 1, ' %')} "
                     f"(ratio {_fmt(m.get('bounce_ratio'), 2)} vs AGGR {_fmt(aggr * 100 if _fin(aggr) else None, 1, ' %')}), "
                     f"settle +-{_lc('step_settle_frac') * 100:.0f} % {_ms(m.get('settle_s'))}, "
                     f"steady state {_fmt(m.get('ss'), 3)} over {_lc('step_ss_window_s')[0]:g}-{_lc('step_ss_window_s')[1]:g} s")
    if osc:
        lines.append("oscillation: " + (osc.get("note") or ""))
        if _fin(osc.get("dmod_engaged_pct")):
            lines.append(f"Dmod min {_fmt(osc['dmod_min'], 3)}, engaged on {osc['dmod_engaged_pct']:.1f} % of samples"
                         + ("; SMAX never limited the loop" if osc["dmod_min"] >= 1.0 else ""))
    return lines
