"""Local FFT / spectral analysis of logged signals.

The maths is scipy's: `scipy.fft` (pocketfft) for the transforms, `scipy.signal.welch`
for power spectral density and `scipy.signal.find_peaks` for peak picking. Nothing here
re-implements a transform.

What this module adds is the part that goes wrong in practice:

* **Sample-rate honesty.** A spectrum is only meaningful if the samples are evenly
  spaced. `sample_rate()` measures the interval jitter and the caller refuses to
  transform a signal whose timing is irregular, rather than quietly resampling it.
* **Nyquist honesty.** The standard `LOG_BITMASK` logs IMU at 25 Hz and RATE at 10 Hz,
  which cannot see a 200 Hz motor. `sources()` lists every candidate signal in the log
  with its rate and Nyquist limit, and the check says outright when no logged signal can
  resolve the frequency of interest.
* **Motor-order labelling.** Peaks are expressed as multiples of the ESC fundamental
  when the log has ESC telemetry, so "a peak at 1.98x" reads as "the second harmonic".

Signals, in order of usefulness for motor-noise work:

    ISBD   batch IMU samples (INS_LOG_BAT_MASK), 1-4 kHz, pre and/or post filter
    GYR    raw gyro (INS_RAW_LOG_OPT), sensor rate
    ACC    raw accel, sensor rate
    IMU    the filtered IMU as the EKF sees it; loop rate only with the fast-IMU log bit
    RATE   rate controller inputs; 10 Hz unless LOG_BITMASK fast attitude is on
"""

from __future__ import annotations

import numpy as np

try:
    from scipy import fft as _fft
    from scipy import signal as _signal
    HAVE_SCIPY = True
except ImportError:  # pragma: no cover
    _fft = np.fft
    _signal = None
    HAVE_SCIPY = False

__all__ = ["sample_rate", "psd_welch", "amplitude_spectrum", "find_peaks_db",
           "sources", "analyse", "SpectralError", "HAVE_SCIPY"]


class SpectralError(ValueError):
    """Raised when a signal cannot honestly be transformed (irregular sampling, too
    short, or a Nyquist limit below the requested band)."""


def sample_rate(t, max_jitter=0.05):
    """(fs_hz, stats) for a time base in seconds.

    `stats` carries median/p95/max interval and the jitter fraction
    (p95 interval / median - 1). Raises SpectralError if fewer than 16 samples or if
    the jitter exceeds `max_jitter`, because an FFT of unevenly spaced samples is wrong
    in a way that looks right.
    """
    t = np.asarray(t, dtype=float)
    if t.size < 16:
        raise SpectralError(f"only {t.size} samples; need at least 16")
    dt = np.diff(t)
    dt = dt[np.isfinite(dt)]
    if dt.size == 0 or (dt <= 0).any():
        raise SpectralError("time base is not strictly increasing")
    med = float(np.median(dt))
    p95 = float(np.percentile(dt, 95))
    mx = float(dt.max())
    jitter = p95 / med - 1.0
    stats = dict(median_dt_s=med, p95_dt_s=p95, max_dt_s=mx, jitter=jitter,
                 n=int(t.size), span_s=float(t[-1] - t[0]))
    if jitter > max_jitter:
        raise SpectralError(
            f"sampling is irregular: p95 interval {p95 * 1e3:.2f} ms vs median "
            f"{med * 1e3:.2f} ms ({jitter * 100:.1f}% jitter, limit {max_jitter * 100:.0f}%); "
            "a spectrum of these samples would be wrong")
    return 1.0 / med, stats


def psd_welch(x, fs, nperseg=None, window="hann", detrend="constant"):
    """One-sided PSD via Welch's method. Returns (freqs_hz, psd) in units^2/Hz."""
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if nperseg is None:
        nperseg = int(min(1024, max(64, 2 ** int(np.log2(max(len(x) // 4, 64))))))
    nperseg = int(min(nperseg, len(x)))
    if _signal is None:  # pragma: no cover - numpy fallback, single segment
        w = np.hanning(len(x))
        f = _fft.rfftfreq(len(x), 1.0 / fs)
        p = np.abs(_fft.rfft((x - x.mean()) * w)) ** 2 * 2.0 / (fs * np.inner(w, w))
        return f, p
    f, p = _signal.welch(x, fs=fs, nperseg=nperseg, window=window, detrend=detrend,
                         scaling="density")
    return f, p


def amplitude_spectrum(x, fs, window="hann"):
    """Single-frame amplitude spectrum (same units as x), window-gain corrected."""
    x = np.asarray(x, dtype=float)
    x = x - np.nanmean(x)
    n = len(x)
    w = _signal.get_window(window, n) if _signal is not None else np.hanning(n)
    spec = _fft.rfft(x * w)
    amp = 2.0 * np.abs(spec) / w.sum()
    amp[0] /= 2.0
    return _fft.rfftfreq(n, 1.0 / fs), amp


def find_peaks_db(freqs, psd, n=5, fmin=None, fmax=None, min_prominence_db=6.0,
                  floor_percentile=50):
    """Strongest spectral peaks, with height above the local noise floor in dB.

    Returns a list of dicts sorted by dB above floor: freq_hz, psd, db_above_floor,
    prominence_db. The floor is the `floor_percentile` of the PSD in [fmin, fmax].
    """
    freqs = np.asarray(freqs, dtype=float)
    psd = np.asarray(psd, dtype=float)
    m = np.isfinite(psd) & (psd > 0)
    if fmin is not None:
        m &= freqs >= fmin
    if fmax is not None:
        m &= freqs <= fmax
    if m.sum() < 8:
        return []
    f, p = freqs[m], psd[m]
    db = 10.0 * np.log10(p)
    floor = float(np.percentile(db, floor_percentile))
    if _signal is not None:
        idx, props = _signal.find_peaks(db, prominence=min_prominence_db)
        prom = props["prominences"]
    else:  # pragma: no cover
        idx = np.flatnonzero((db[1:-1] > db[:-2]) & (db[1:-1] >= db[2:])) + 1
        prom = db[idx] - floor
    out = [dict(freq_hz=float(f[i]), psd=float(p[i]), db_above_floor=float(db[i] - floor),
                prominence_db=float(pr)) for i, pr in zip(idx, prom)]
    out.sort(key=lambda d: -d["db_above_floor"])
    return out[:n]


# ------------------------------------------------------------------ log sources

def _signal_from_df(d, cols, tcol="t"):
    t = d[tcol].values.astype(float)
    return t, {c: d[c].values.astype(float) for c in cols if c in d.columns}


def sources(log):
    """Every logged signal an FFT could be run on, with its rate and Nyquist.

    Returns a list of dicts: name, message, instance, kind (gyro/accel/rate),
    axes, rate_hz, nyquist_hz, n, note. Sorted by rate, highest first, so the first
    entry is the best available source for motor-noise work.
    """
    out = []

    # ISBD batches: rate from ISBH.smp_rate, one entry per (type, instance)
    isbh = log.df("ISBH")
    if not isbh.empty and log.has("ISBD"):
        n_imu = max(len(log.instances("IMU")), 1)
        for (typ, inst), g in isbh.groupby(["type", "instance"]):
            fs = float(np.median(g["smp_rate"]))
            post = int(inst) >= n_imu and int(log.param("INS_LOG_BAT_OPT", 0) or 0) & 4
            out.append(dict(name=f"ISBD:{'gyro' if int(typ) == 1 else 'accel'}:{int(inst)}",
                            message="ISBD", instance=int(inst),
                            kind="gyro" if int(typ) == 1 else "accel", axes=["x", "y", "z"],
                            rate_hz=fs, nyquist_hz=fs / 2.0, n=int(g["smp_cnt"].sum()),
                            note="batch IMU samples, "
                                 + ("POST-filter (instance offset by IMU count, INS_LOG_BAT_OPT bit 2)"
                                    if post else "pre-filter (or the only series)")))

    for msg, kind, axes, note in (
            ("GYR", "gyro", ("GyrX", "GyrY", "GyrZ"), "raw gyro at sensor rate (INS_RAW_LOG_OPT)"),
            ("ACC", "accel", ("AccX", "AccY", "AccZ"), "raw accel at sensor rate"),
            ("IMU", "gyro", ("GyrX", "GyrY", "GyrZ"), "filtered gyro as the EKF sees it"),
            ("RATE", "rate", ("R", "P", "Y"), "rate controller actual rates, deg/s")):
        d = log.df(msg)
        if d.empty or "t" not in d.columns:
            continue
        for inst, g in (log.instances(msg).items() if msg != "RATE" else [(0, d)]):
            if len(g) < 16:
                continue
            fs = float(1.0 / np.median(np.diff(g["t"].values)))
            present = [a for a in axes if a in g.columns]
            if not present:
                continue
            out.append(dict(name=f"{msg}:{inst}", message=msg, instance=int(inst), kind=kind,
                            axes=present, rate_hz=fs, nyquist_hz=fs / 2.0, n=int(len(g)),
                            note=note))
    # highest rate first; at equal rate the lower instance (pre-filter) first
    out.sort(key=lambda s: (-round(s["rate_hz"]), s["instance"]))
    return out


def _batch_signal(log, inst, kind, window, max_batches=None):
    """Concatenate contiguous ISBD batches for one (type, instance) into a list of
    (t0, fs, ndarray[3, n]) blocks. Batches with a seqno hole are discarded."""
    isbh, isbd = log.df("ISBH"), log.df("ISBD")
    by_n = {int(k): g for k, g in isbd.groupby("N")}
    typ = 1 if kind == "gyro" else 0
    blocks, holes = [], 0
    for _, h in isbh.iterrows():
        if int(h["type"]) != typ or int(h["instance"]) != inst:
            continue
        g = by_n.get(int(h["N"]))
        if g is None:
            continue
        seq = g["seqno"].values
        if len(seq) < 2 or not np.all(np.diff(seq) == 1):
            holes += 1
            continue
        t0 = float(h["TimeUS"]) / 1e6
        if window is not None and not (window.t0 <= t0 <= window.t1):
            continue
        mul = float(h["mul"]) or 1.0
        xyz = np.vstack([np.concatenate([np.asarray(v, dtype=float) for v in g[c].values])
                         for c in ("x", "y", "z")]) / mul
        if typ == 1:
            xyz = np.degrees(xyz)
        blocks.append((t0, float(h["smp_rate"]), xyz))
        if max_batches and len(blocks) >= max_batches:
            break
    return blocks, holes


def analyse(log, source=None, window=None, axes=None, fmin=5.0, fmax=None, nperseg=None,
            n_peaks=6, max_jitter=0.05):
    """Run a PSD on one logged source and identify its peaks.

    `source` is a name from `sources()` (e.g. "ISBD:gyro:0", "IMU:0", "RATE:0") or None
    for the highest-rate gyro source available. Returns a dict with the spectrum per axis,
    the peaks, the sample-rate statistics, and the motor-order labelling when ESC
    telemetry exists. Raises SpectralError rather than returning a misleading spectrum.
    """
    srcs = sources(log)
    if not srcs:
        raise SpectralError("no IMU, GYR, ACC, RATE or ISBD messages in this log")
    if source is None:
        gyro = [s for s in srcs if s["kind"] == "gyro"]
        src = (gyro or srcs)[0]
    else:
        match = [s for s in srcs if s["name"] == source or s["name"].startswith(source + ":")
                 or s["message"] == source]
        if not match:
            raise SpectralError(f"no source {source!r}; available: "
                                + ", ".join(s["name"] for s in srcs))
        src = match[0]

    axes = list(axes) if axes else src["axes"]
    result = dict(source=src, axes={}, peaks={}, fs_hz=None, timing=None, holes=0,
                  window=(window.t0, window.t1, window.method) if window is not None else None)

    if src["message"] == "ISBD":
        blocks, holes = _batch_signal(log, src["instance"], src["kind"], window)
        result["holes"] = holes
        if not blocks:
            raise SpectralError(f"no contiguous ISBD batches for {src['name']} in the window "
                                f"({holes} discarded for seqno holes)")
        fs = blocks[0][1]
        n = blocks[0][2].shape[1]
        seg = int(min(nperseg or n, n))
        win = _signal.get_window("hann", seg) if _signal is not None else np.hanning(seg)
        s2 = float(np.inner(win, win))
        freqs = _fft.rfftfreq(seg, 1.0 / fs)
        acc = {a: np.zeros(len(freqs)) for a in ("x", "y", "z")}
        cnt = 0
        for _t0, _fs, xyz in blocks:
            if xyz.shape[1] < seg:
                continue
            for ai, a in enumerate(("x", "y", "z")):
                row = xyz[ai, :seg]
                p = np.abs(_fft.rfft((row - row.mean()) * win)) ** 2
                p[0] = 0.0
                acc[a] += 2.0 * p / (fs * s2)
            cnt += 1
        if cnt == 0:
            raise SpectralError("no batch long enough for the requested segment")
        result["fs_hz"] = fs
        result["timing"] = dict(batches=cnt, samples_per_batch=n, holes=holes,
                                median_dt_s=1.0 / fs, jitter=0.0)
        for a in axes:
            result["axes"][a] = (freqs, acc[a] / cnt)
    else:
        d = log.df(src["message"])
        if src["message"] != "RATE":
            d = log.instances(src["message"]).get(src["instance"], d)
        if window is not None:
            d = window.clip(d)
        t, cols = _signal_from_df(d, axes)
        if not cols:
            raise SpectralError(f"axes {axes} not in {src['message']}")
        fs, timing = sample_rate(t, max_jitter=max_jitter)
        result["fs_hz"] = fs
        result["timing"] = timing
        for a, x in cols.items():
            result["axes"][a] = psd_welch(x, fs, nperseg=nperseg)

    nyq = result["fs_hz"] / 2.0
    hi = min(fmax, nyq) if fmax else nyq
    if fmin >= hi:
        raise SpectralError(f"band {fmin}-{hi} Hz is empty at Nyquist {nyq:.1f} Hz")
    result["band_hz"] = (fmin, hi)

    # motor-order labelling
    from .flight import esc_fundamental
    tt, fund = esc_fundamental(log)
    f0 = None
    if fund is not None:
        m = window.mask(tt) if window is not None else np.ones(len(tt), dtype=bool)
        if m.any():
            f0 = float(np.nanmedian(fund[m]))
    result["esc_fundamental_hz"] = f0

    warnings = []
    for a, (f, p) in result["axes"].items():
        pk = find_peaks_db(f, p, n=n_peaks, fmin=fmin, fmax=hi)
        for q in pk:
            q["order"] = (q["freq_hz"] / f0) if f0 else None
        result["peaks"][a] = pk
    if f0 and nyq < f0:
        warnings.append(f"Nyquist {nyq:.1f} Hz of {src['name']} is below the motor fundamental "
                        f"{f0:.1f} Hz: motor noise is invisible in this source (and aliases into it)")
    elif f0 and nyq < 2.2 * f0:
        warnings.append(f"Nyquist {nyq:.1f} Hz of {src['name']} cannot show the 2nd harmonic "
                        f"({2 * f0:.0f} Hz)")
    if not any(result["peaks"].values()):
        warnings.append("no peaks with >= 6 dB prominence in the band; the spectrum is flat or the "
                        "band is too narrow")
    if f0 is None:
        warnings.append("no ESC telemetry: peaks cannot be labelled in motor orders")
    result["warnings"] = warnings
    return result
