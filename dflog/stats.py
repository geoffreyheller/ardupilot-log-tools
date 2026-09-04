"""Small statistics helpers, so every script reports the same way."""

from __future__ import annotations

import numpy as np

__all__ = ["describe", "pct_above", "corr", "band_split", "psd", "peak_hz", "safe"]


def safe(x):
    """Finite values only, as a float array."""
    a = np.asarray(x, dtype=float).ravel()
    return a[np.isfinite(a)]


def describe(x, pcts=(5, 50, 95)):
    """mean/sd/min/max plus requested percentiles. NaN-safe, empty-safe."""
    a = safe(x)
    if a.size == 0:
        return {k: float("nan") for k in ("n", "mean", "sd", "min", "max")}
    out = {"n": int(a.size), "mean": float(a.mean()), "sd": float(a.std(ddof=1)) if a.size > 1 else 0.0,
           "min": float(a.min()), "max": float(a.max())}
    for p in pcts:
        out[f"p{p:02d}"] = float(np.percentile(a, p))
    return out


def pct_above(x, threshold):
    """Percent of finite samples strictly above a threshold."""
    a = safe(x)
    return float((a > threshold).mean() * 100.0) if a.size else float("nan")


def corr(x, y):
    """Pearson correlation of two series, NaN-safe and length-safe."""
    x = np.asarray(x, dtype=float).ravel()
    y = np.asarray(y, dtype=float).ravel()
    n = min(x.size, y.size)
    x, y = x[:n], y[:n]
    m = np.isfinite(x) & np.isfinite(y)
    if m.sum() < 3:
        return float("nan")
    x, y = x[m], y[m]
    if x.std() == 0 or y.std() == 0:
        return float("nan")
    return float(np.corrcoef(x, y)[0, 1])


def band_split(t, x, cut_hz=5.0):
    """Split a signal into low- and high-frequency parts about `cut_hz`.

    Returns (low, high). Used to separate genuine tracking error (low) from
    gyro noise reaching the controller (high) — the distinction that decides
    whether a rate-gain change or filter work is the right fix.
    """
    t = np.asarray(t, dtype=float)
    x = np.asarray(x, dtype=float)
    m = np.isfinite(t) & np.isfinite(x)
    t, x = t[m], x[m]
    if t.size < 8:
        return x, np.zeros_like(x)
    dt = float(np.median(np.diff(t)))
    if not np.isfinite(dt) or dt <= 0:
        return x, np.zeros_like(x)
    fs = 1.0 / dt
    try:
        from scipy.signal import butter, filtfilt
        wn = min(cut_hz / (fs / 2.0), 0.99)
        b, a = butter(2, wn, btype="low")
        low = filtfilt(b, a, x)
    except Exception:
        win = max(3, int(round(fs / cut_hz)) | 1)
        kern = np.ones(win) / win
        low = np.convolve(x, kern, mode="same")
    return low, x - low


def psd(t, x, nperseg=None):
    """One-sided power spectral density. Returns (freqs, power)."""
    from scipy.signal import welch
    t = np.asarray(t, dtype=float)
    x = np.asarray(x, dtype=float)
    m = np.isfinite(t) & np.isfinite(x)
    t, x = t[m], x[m]
    if t.size < 32:
        return np.array([]), np.array([])
    fs = 1.0 / float(np.median(np.diff(t)))
    nperseg = nperseg or min(1024, len(x))
    return welch(x, fs=fs, nperseg=nperseg)


def peak_hz(freqs, power, lo=None, hi=None):
    """Frequency of the strongest bin in [lo, hi]."""
    if len(freqs) == 0:
        return float("nan")
    m = np.ones(len(freqs), dtype=bool)
    if lo is not None:
        m &= freqs >= lo
    if hi is not None:
        m &= freqs <= hi
    if not m.any():
        return float("nan")
    return float(freqs[m][np.argmax(power[m])])
