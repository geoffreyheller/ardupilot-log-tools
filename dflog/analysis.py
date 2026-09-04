"""The standard battery of checks.

Each `check_*` function takes (log, window) and returns a Section: a title,
a list of Result, and a markdown body. They are deliberately independent so a
session can run just the one it needs, and deliberately uniform so `alog all`
can run every one and produce a report that looks the same every time.

Rules every check here follows, and any new one must too:
  * State the number, not just the verdict.
  * Cite the window and the method that chose it.
  * Never invent a threshold - take it from dflog.checks.T and cite the source.
  * Report "not logged" as SKIP, never as a pass.
"""

from __future__ import annotations

import numpy as np

from .checks import T, Result, PASS, WARN, FAIL, SKIP
from .flight import EVENTS, MODES, airborne_window, esc_fundamental, events, mode_timeline
from .frames import mix_for, trim_decomposition
from .report import table, fmt, heading
from .stats import band_split, corr, describe, pct_above, peak_hz, psd, safe

__all__ = ["Section", "ALL_CHECKS", "run", "check_summary", "check_vibration",
           "check_motors", "check_notch", "check_pids", "check_ekf",
           "check_compass", "check_power", "check_gps", "check_cpu",
           "check_events", "check_gust_response", "check_batch_fft"]


class Section:
    def __init__(self, title, results=None, md="", data=None):
        self.title = title
        self.results = list(results or [])
        self.md = md
        self.data = data or {}

    @property
    def worst(self):
        return max(self.results, key=lambda r: r.rank).status if self.results else SKIP

    def render(self, level=2):
        out = [heading(self.title, level)]
        for r in self.results:
            out.append(r.line())
        if self.md:
            out.append("")
            out.append(self.md)
        return "\n".join(out)


def _grade(value, key, higher_is_worse=True, name="", summary_fmt=None, **ev):
    """Grade a value against a registered threshold and build a Result."""
    th = T[key]
    warn, fail = th["warn"], th["fail"]
    if value is None or (isinstance(value, float) and not np.isfinite(value)):
        status = SKIP
    elif higher_is_worse:
        status = FAIL if value > fail else (WARN if value > warn else PASS)
    else:
        status = FAIL if value < fail else (WARN if value < warn else PASS)
    summary = (summary_fmt or "{v} (warn {w}, fail {f})").format(
        v=fmt(value), w=fmt(warn), f=fmt(fail))
    return Result(name or key, status, summary, evidence=dict(value=value, warn=warn, fail=fail, **ev),
                  source=th["source"])


# --------------------------------------------------------------------- summary

def check_summary(log, w):
    lo, hi = log.duration()
    p = log.params()
    seen, fw = set(), ""
    for t, m in log.messages_text():
        m = str(m)
        if m in seen:
            continue
        if any(k in m for k in ("ArduCopter", "ArduPlane", "Rover", "ArduSub", "ChibiOS",
                                "fast sampling", "RCOut:", "RC Protocol", "Frame:")):
            seen.add(m)
            fw += f"  {m}\n"
    modes = mode_timeline(log)
    mode_names = []
    for i, (t, num, name, _) in enumerate(modes):
        dur = (modes[i + 1][0] if i + 1 < len(modes) else w.t1) - t
        if dur > 1 and (not mode_names or mode_names[-1][0] != name):
            mode_names.append((name, dur))
    rows = [
        ["log", log.path.rsplit("/", 1)[-1]],
        ["messages", f"{log.n_messages} in {len(log.messages)} types"],
        ["resync bytes", f"{log.resync_bytes} (should be 0)"],
        ["log span", f"{lo:.1f} - {hi:.1f} s ({hi - lo:.1f} s)"],
        ["airborne window", f"{w.t0:.1f} - {w.t1:.1f} s ({w.duration:.1f} s) via {w.method}"],
        ["frame", f"FRAME_CLASS={fmt(p.get('FRAME_CLASS'))} FRAME_TYPE={fmt(p.get('FRAME_TYPE'))}"],
        ["params in log", str(len(p))],
        ["modes flown", ", ".join(f"{n} {d:.0f}s" for n, d in mode_names) or "-"],
    ]
    md = table(["item", "value"], rows, align=["l", "l"])
    if fw:
        md += "\n\nFirmware banner:\n```\n" + fw + "```"
    res = [Result("parser integrity",
                  PASS if log.resync_bytes == 0 else WARN,
                  f"{log.resync_bytes} resync bytes across {log.n_messages} messages",
                  evidence=dict(resync=log.resync_bytes), source="dflog")]
    return Section("Summary", res, md)


# ------------------------------------------------------------------- vibration

def check_vibration(log, w):
    d = w.clip(log.df("VIBE"))
    if d is None or d.empty:
        return Section("Vibration", [Result("vibration", SKIP, "no VIBE messages in log")])
    inst = log.instance_field("VIBE")
    rows, res = [], []
    groups = d.groupby(inst) if inst else [(0, d)]
    for i, g in groups:
        for ax, key in (("VibeX", "vibe_xy"), ("VibeY", "vibe_xy"), ("VibeZ", "vibe_z")):
            if ax not in g.columns:
                continue
            s = describe(g[ax])
            rows.append([f"IMU{i} {ax}", s["mean"], s.get("p95"), s["max"]])
            res.append(_grade(s.get("p95"), key, name=f"IMU{i} {ax} p95",
                              summary_fmt="p95 {v} m/s^2 (warn {w}, fail {f})",
                              mean=s["mean"], max=s["max"]))
        if "Clip" in g.columns and len(g):
            clip = float(g["Clip"].iloc[-1] - g["Clip"].iloc[0])
            res.append(_grade(clip, "clip_events", name=f"IMU{i} accel clipping",
                              summary_fmt="{v} clip events in the window (warn >{w}, fail >{f})"))
    md = table(["axis", "mean", "p95", "max"], rows)
    md += "\n\nUnits are m/s^2. ArduPilot's rule of thumb: below 15 is good, above 30 is a problem.\n"
    md += "Clipping is the harder failure - any non-zero clip count means the accelerometer\n"
    md += "saturated and the EKF was fed garbage for those samples."
    return Section("Vibration", res, md)


# ---------------------------------------------------------------------- motors

def check_motors(log, w, normalise=True):
    rcou = w.clip(log.df("RCOU"))
    p = log.params()
    esc = log.instances("ESC")
    if (rcou is None or rcou.empty) and not esc:
        return Section("Motors", [Result("motors", SKIP, "no RCOU or ESC messages")])

    n = 0
    if rcou is not None and not rcou.empty:
        for i in range(1, 13):
            c = f"C{i}"
            if c in rcou.columns and rcou[c].std() > 0.5 and rcou[c].mean() > 900:
                n = i
    n = max(n, len(esc))
    mix = mix_for(p.get("FRAME_CLASS", 1), p.get("FRAME_TYPE", 1), n_motors=n, normalise=normalise)

    res, md_parts = [], []

    # --- per-motor RCOU and the trim decomposition
    if rcou is not None and not rcou.empty and n >= 3:
        chans = [f"C{i + 1}" for i in range(min(n, mix.n))]
        means = [float(rcou[c].mean()) for c in chans if c in rcou.columns]
        if len(means) == mix.n:
            tr = trim_decomposition(means, mix)
            md_parts.append(table(["motor"] + chans, [["RCOU mean (us)"] + [round(m, 1) for m in means]]))
            md_parts.append("")
            md_parts.append(table(
                ["axis", "trim (us)", "reads as"],
                [["roll", round(tr["roll"], 1), "mean(left) - mean(right)"],
                 ["pitch", round(tr["pitch"], 1), "mean(front) - mean(rear); negative = CG aft"],
                 ["yaw", round(tr["yaw"], 1), "mean(CCW) - mean(CW); non-zero = standing torque"],
                 ["residual", round(tr["residual"], 2), "unexplained by any control axis"]],
                align=["l", "r", "l"]))
            for ax in ("roll", "pitch", "yaw"):
                res.append(_grade(abs(tr[ax]), "trim_us", name=f"{ax} trim",
                                  summary_fmt="%s us standing trim (warn {w}, fail {f})" % fmt(tr[ax]),
                                  signed=tr[ax]))
            md_parts.append(f"\nMix: {mix.label}"
                            f"{'' if normalise else ' [un-normalised cos factors, legacy mode]'}")

    # --- ESC RPM spread and error rate
    if esc:
        rows = []
        rpm_means = []
        for i, g in sorted(esc.items()):
            gg = w.clip(g)
            if gg is None or gg.empty:
                continue
            r = describe(gg["RPM"]) if "RPM" in gg.columns else {}
            # Grade on the median: means are pulled around by spin-up and by
            # brief saturation.
            rpm_means.append(r.get("p50", np.nan))
            err = describe(gg["Err"]) if "Err" in gg.columns else {}
            rows.append([f"ESC{i}", r.get("mean"), r.get("p50"), r.get("p50", np.nan) / 60.0,
                         r.get("min"), r.get("max"), err.get("mean"), err.get("p95")])
        if rows:
            md_parts.append("")
            md_parts.append(table(["esc", "RPM mean", "RPM median", "Hz", "RPM min", "RPM max",
                                   "Err% mean", "Err% p95"], rows))
            rpm_means = np.array(rpm_means, dtype=float)
            if np.isfinite(rpm_means).all() and rpm_means.mean() > 0:
                spread = (rpm_means.max() - rpm_means.min()) / rpm_means.mean() * 100
                res.append(_grade(spread, "rpm_spread_pct", name="RPM spread",
                                  summary_fmt="{v}% across motors, on medians (warn {w}, fail {f})",
                                  per_motor_median=list(np.round(rpm_means, 1))))
            # index 6 is "Err% mean" - keep this in step with the header row above
            errs = [r[6] for r in rows if r[6] is not None and np.isfinite(r[6])]
            if errs:
                res.append(_grade(max(errs), "esc_err_pct", name="bidir DShot error rate",
                                  summary_fmt="worst motor {v}% (warn {w}, fail {f})"))
            md_parts.append("\nRPM spread alone is a poor statistic - it says the motors disagree but not how.\n"
                            "The trim decomposition above is what separates a CG offset (pitch/roll) from a\n"
                            "torque asymmetry (yaw). A bent blade straightened by hand typically fixes roll\n"
                            "and leaves yaw untouched, because the blade recovers its thrust but not its drag.")

    # --- headroom against the MOT_SPIN_MAX ceiling
    if rcou is not None and not rcou.empty and n >= 3:
        spin_max = p.get("MOT_SPIN_MAX", 0.95)
        lo_pwm = p.get("SERVO1_MIN", 1000.0)
        hi_pwm = p.get("SERVO1_MAX", 2000.0)
        ceiling = lo_pwm + spin_max * (hi_pwm - lo_pwm)
        chans_present = [f"C{i+1}" for i in range(min(n, mix.n)) if f"C{i+1}" in rcou.columns]
        allout = np.concatenate([rcou[c].values for c in chans_present])
        # p99.5, not the raw max: a single sample touching the ceiling during a
        # hard manoeuvre is not "saturated", sustained time there is.
        peak = float(np.percentile(allout, 99.5))
        hard_max = float(allout.max())
        sat_pct = float((allout >= ceiling - 1).mean() * 100.0)
        frac = (peak - lo_pwm) / (ceiling - lo_pwm) if ceiling > lo_pwm else np.nan
        res.append(_grade(frac, "motor_headroom", name="motor headroom",
                          summary_fmt="p99.5 output %.0f us = {v} of the MOT_SPIN_MAX ceiling %.0f us "
                                      "(warn {w}, fail {f})" % (peak, ceiling),
                          peak_us=peak, hard_max_us=hard_max, ceiling_us=ceiling,
                          saturated_pct=sat_pct))
        md_parts.append(f"\nSaturation ceiling is SERVO_MIN + MOT_SPIN_MAX * (MAX-MIN) = {ceiling:.0f} us, "
                        f"not {hi_pwm:.0f}. p99.5 output {peak:.0f} us, hard max {hard_max:.0f} us, "
                        f"at or above the ceiling for {sat_pct:.2f}% of samples.")

    return Section("Motors and standing trim", res, "\n".join(md_parts),
                   data=dict(mix=mix.label))


# ----------------------------------------------------------------------- notch

def check_notch(log, w):
    p = log.params()
    mode = p.get("INS_HNTCH_MODE")
    fcns = log.instances("FCNS")
    t, fund = esc_fundamental(log)
    res, md = [], []
    mode_names = {0: "Fixed", 1: "Throttle", 2: "RPM sensor", 3: "ESC telemetry", 4: "In-flight FFT"}
    md.append(f"INS_HNTCH_ENABLE={fmt(p.get('INS_HNTCH_ENABLE'))} "
              f"MODE={fmt(mode)} ({mode_names.get(int(mode) if mode is not None else -1, '?')}) "
              f"FREQ={fmt(p.get('INS_HNTCH_FREQ'))} BW={fmt(p.get('INS_HNTCH_BW'))} "
              f"REF={fmt(p.get('INS_HNTCH_REF'))} HMNCS={fmt(p.get('INS_HNTCH_HMNCS'))}")
    if fund is None:
        return Section("Harmonic notch", [Result("notch", SKIP,
                       "no ESC telemetry, so there is no ground truth to check the notch against")],
                       "\n".join(md))
    if not fcns:
        md.append("\nNo FCNS messages: the applied notch centre frequency was not logged, so the "
                  "notch cannot be verified. FTN1.PkAvg is the FFT's opinion, not the filter's setting.")
        return Section("Harmonic notch", [Result("notch", SKIP, "FCNS not logged")], "\n".join(md))

    for i, g in sorted(fcns.items()):
        gg = w.clip(g)
        if gg is None or gg.empty or "CF" not in gg.columns:
            continue
        f_at = np.interp(gg["t"].values, t, fund)
        ok = f_at > 1.0
        ratio = gg["CF"].values[ok] / f_at[ok]
        s = describe(ratio)
        mis = pct_above(ratio, 1.5)
        rows = [["notch %d CF / fundamental" % i, s["mean"], s.get("p05"), s.get("p50"), s.get("p95")]]
        if "HF" in gg.columns and np.nanmedian(gg["HF"].values) > 0:
            hr = gg["HF"].values[ok] / f_at[ok]
            hs = describe(hr)
            rows.append(["notch %d HF / fundamental" % i, hs["mean"], hs.get("p05"), hs.get("p50"), hs.get("p95")])
        md.append("")
        md.append(table(["series", "mean", "p05", "p50", "p95"], rows))
        res.append(_grade(abs(s.get("p95", np.nan) - 1.0), "notch_track",
                          name=f"notch {i} tracking",
                          summary_fmt="p95 error {v} of the fundamental (warn {w}, fail {f})",
                          median_ratio=s.get("p50")))
        res.append(_grade(mis, "notch_mistrack_pct", name=f"notch {i} harmonic lock-on",
                          summary_fmt="{v}% of the window above 1.5x the fundamental (warn {w}, fail {f})"))

    # Floor clamping: is INS_HNTCH_FREQ actually being hit in flight?
    freq_floor = p.get("INS_HNTCH_FREQ")
    if freq_floor:
        below = pct_above(-fund[w.mask(t)], -freq_floor)
        md.append(f"\nMotor fundamental below the INS_HNTCH_FREQ floor of {fmt(freq_floor)} Hz for "
                  f"{below:.2f}% of the airborne window (airborne min "
                  f"{np.nanmin(fund[w.mask(t)]):.1f} Hz). If that is 0.00%, the floor is untested by "
                  f"this flight - neither justified nor disproved.")

    # FFT observer, if still running
    ftn1 = w.clip(log.df("FTN1"))
    if ftn1 is not None and not ftn1.empty and "PkAvg" in ftn1.columns:
        f_at = np.interp(ftn1["t"].values, t, fund)
        ok = f_at > 1.0
        r = ftn1["PkAvg"].values[ok] / f_at[ok]
        md.append(f"\nFFT observer (FTN1.PkAvg): median {np.nanmedian(r):.3f} x fundamental, "
                  f"{pct_above(r, 1.5):.2f}% above 1.5x. "
                  f"{'This only matters if MODE=4 - otherwise it is inert.' if mode != 4 else ''}")
    return Section("Harmonic notch", res, "\n".join(md))


# ------------------------------------------------------------------------ PIDs

def check_pids(log, w, cut_hz=5.0):
    rate = w.clip(log.df("RATE"))
    att = w.clip(log.df("ATT"))
    res, md = [], []
    if rate is not None and not rate.empty:
        rows = []
        for ax, des, act, out in (("roll", "RDes", "R", "ROut"),
                                  ("pitch", "PDes", "P", "POut"),
                                  ("yaw", "YDes", "Y", "YOut")):
            if des not in rate.columns:
                continue
            c = corr(rate[des], rate[act])
            err = rate[act].values - rate[des].values
            low, high = band_split(rate["t"].values, err, cut_hz)
            rows.append([ax, describe(rate[des])["sd"], describe(rate[act])["sd"], c,
                         float(np.std(low)), float(np.std(high)),
                         describe(rate[out])["sd"] if out in rate.columns else None,
                         float(np.nanmax(np.abs(rate[out]))) if out in rate.columns else None])
            res.append(_grade(c, "rate_corr", higher_is_worse=False, name=f"{ax} rate tracking",
                              summary_fmt="desired-vs-actual correlation {v} (warn <{w}, fail <{f})"))
        md.append(table(["axis", "des sd", "act sd", "corr",
                         f"err sd <{cut_hz:g}Hz", f"err sd >{cut_hz:g}Hz", "out sd", "out peak"], rows))
        md.append(f"\nRates are deg/s. Splitting the rate error at {cut_hz:g} Hz separates genuine tracking\n"
                  "error (low band - a gain problem) from gyro noise reaching the controller (high band -\n"
                  "a filter problem). Do the filter work before touching a rate gain.")
    for msg, ax in (("PIDR", "roll"), ("PIDP", "pitch"), ("PIDY", "yaw")):
        d = w.clip(log.df(msg))
        if d is None or d.empty:
            continue
        parts = []
        if "Dmod" in d.columns:
            dm = describe(d["Dmod"])
            engaged = pct_above(-d["Dmod"].values, -0.999)
            parts.append(f"Dmod min {dm['min']:.3f}, engaged {engaged:.2f}% of samples")
            res.append(Result(f"{ax} D-term slew limiter",
                              PASS if dm["min"] > 0.999 else WARN,
                              f"Dmod min {dm['min']:.3f}"
                              + (" - never engaged, so no oscillation onset" if dm["min"] > 0.999
                                 else f" - engaged {engaged:.1f}% of samples, the tune is backing itself off"),
                              evidence=dict(dmod_min=dm["min"], engaged_pct=engaged),
                              source="ArduPilot ATC_RAT_*_SMAX slew limiter"))
        if "SRate" in d.columns:
            parts.append(f"SRate peak {describe(d['SRate'])['max']:.2f}")
        if all(c in d.columns for c in ("P", "I", "D")):
            tot = d[["P", "I", "D"]].var().sum()
            if tot > 0:
                parts.append("output variance share P/I/D = " +
                             "/".join(f"{d[c].var() / tot * 100:.0f}%" for c in ("P", "I", "D")))
        if parts:
            md.append(f"\n**{ax}**: " + "; ".join(parts))
    if att is not None and not att.empty and {"Roll", "DesRoll"} <= set(att.columns):
        rows = []
        for ax, a, dd in (("roll", "Roll", "DesRoll"), ("pitch", "Pitch", "DesPitch")):
            e = att[a].values - att[dd].values
            s = describe(np.abs(e))
            rows.append([ax, float(np.std(e)), s.get("p95"), s["max"]])
            res.append(_grade(s["max"], "att_err_deg", name=f"{ax} attitude error",
                              summary_fmt="max {v} deg (warn {w}, fail {f})", sd=float(np.std(e))))
        md.append("")
        md.append(table(["axis", "err sd (deg)", "|err| p95", "|err| max"], rows))
    if not res:
        return Section("Attitude and rate control", [Result("pids", SKIP, "no RATE/ATT messages")])
    return Section("Attitude and rate control", res, "\n".join(md))


# ------------------------------------------------------------------------- EKF

def check_ekf(log, w):
    res, md = [], []
    xkf4 = log.instances("XKF4")
    if not xkf4:
        return Section("EKF", [Result("ekf", SKIP, "no XKF4 messages")])
    names = {"SV": "velocity", "SP": "position", "SH": "height", "SM": "magnetometer", "SVT": "airspeed"}
    rows = []
    for c, g in sorted(xkf4.items()):
        gg = w.clip(g)
        if gg is None or gg.empty:
            continue
        for f, label in names.items():
            if f not in gg.columns or not np.isfinite(gg[f]).any():
                continue
            s = describe(gg[f])
            over = int((gg[f].values > 1.0).sum())
            rows.append([f"core{c} {f} ({label})", s["mean"], s.get("p95"), s["max"], over])
            res.append(_grade(s["max"], "ekf_innov", name=f"core{c} {f} innovation",
                              summary_fmt="max {v} (reject at 1.0; warn {w}, fail {f})"
                                          + (f", {over} samples over 1.0" if over else ""),
                              samples_over_1=over, mean=s["mean"]))
        if "errRP" in gg.columns:
            s = describe(gg["errRP"])
            res.append(_grade(s["max"], "ekf_errRP", name=f"core{c} errRP",
                              summary_fmt="max {v} (warn {w}, fail {f})"))
    md.append(table(["series", "mean", "p95", "max", "n>1.0"], rows))
    md.append("\nXKF4 fields are innovation test ratios: ArduPilot rejects the measurement at 1.0.\n"
              "Samples over 1.0 are the number of rejected updates, and are worth naming explicitly -\n"
              "a rising count across consecutive flights is the signal, not the mean.")
    ev = [(t, n) for t, i, n in events(log) if i in (60, 62)]
    if ev:
        md.append("\nEKF reset events: " + ", ".join(f"{n} at t={t:.1f}s" for t, n in ev))
    return Section("EKF", res, "\n".join(md))


# --------------------------------------------------------------------- compass

def check_compass(log, w):
    mag = log.instances("MAG")
    if not mag:
        return Section("Compass", [Result("compass", SKIP, "no MAG messages")])
    ctun = w.clip(log.df("CTUN"))
    res, rows, md = [], [], []
    for i, g in sorted(mag.items()):
        gg = w.clip(g)
        if gg is None or gg.empty:
            continue
        b = np.sqrt(gg["MagX"].values ** 2 + gg["MagY"].values ** 2 + gg["MagZ"].values ** 2)
        s = describe(b)
        health = float(gg["Health"].mean() * 100) if "Health" in gg.columns else np.nan
        # LogAnalyzer uses (max-min)/min; percentiles instead, because a single
        # sample near touchdown or a landing-gear magnet will fail an otherwise
        # healthy compass. The raw max/min are still in the table.
        p01, p99 = np.percentile(b, 1), np.percentile(b, 99)
        var = (p99 - p01) / p01 if p01 > 0 else np.nan
        rows.append([f"MAG{i}", s["mean"], s["sd"], s["min"], s["max"], health])
        res.append(_grade(s["mean"], "compass_field_hi", name=f"MAG{i} field magnitude",
                          summary_fmt="mean {v} mGauss (warn >{w}, fail >{f}); expected band 120-550"))
        if np.isfinite(var):
            res.append(_grade(var, "compass_field_var", name=f"MAG{i} field variation",
                              summary_fmt="(p99-p01)/p01 = {v} (warn {w}, fail {f})",
                              raw_maxmin_over_min=(s["max"] - s["min"]) / s["min"] if s["min"] > 0 else None))
        if np.isfinite(health):
            res.append(Result(f"MAG{i} health", PASS if health > 99.5 else WARN,
                              f"MAG.Health true for {health:.2f}% of samples",
                              evidence=dict(health_pct=health), source="ArduPilot"))
        if ctun is not None and not ctun.empty and "ThO" in ctun.columns:
            thr = np.interp(gg["t"].values, ctun["t"].values, ctun["ThO"].values)
            c = corr(thr, b)
            res.append(_grade(abs(c), "compass_mot_corr", name=f"MAG{i} motor interference",
                              summary_fmt="corr(throttle, |B|) = %s (warn {w}, fail {f})" % fmt(c),
                              signed=c))
    md.append(table(["mag", "|B| mean", "sd", "min", "max", "health %"], rows))
    ofs = {k: v for k, v in log.params().items() if k.startswith("COMPASS_OFS")}
    if ofs:
        md.append("\nConfigured offsets: " + ", ".join(f"{k}={fmt(v)}" for k, v in sorted(ofs.items())))
    prearm = [f"t={t:.1f} {m}" for t, m in log.messages_text() if "mag field" in str(m).lower()]
    if prearm:
        md.append("\nMag-field prearm messages in this log:\n  " + "\n  ".join(prearm))
    md.append("\nField magnitude is in mGauss. A better check than the fixed 120-550 band is the\n"
              "expected field at this lat/lon from the WMM table - see pymavlink's mavextra\n"
              "`expected_earth_field()`, available after running tools/bootstrap_pymavlink.sh.")
    return Section("Compass", res, "\n".join(md))


# ----------------------------------------------------------------------- power

def check_power(log, w):
    res, md = [], []
    bat = log.instances("BAT")
    ctun = w.clip(log.df("CTUN"))
    for i, g in sorted(bat.items()):
        gg = w.clip(g)
        if gg is None or gg.empty:
            continue
        v = describe(gg["Volt"]) if "Volt" in gg.columns else {}
        c = describe(gg["Curr"]) if "Curr" in gg.columns else {}
        rows = [["voltage (V)", v.get("min"), v.get("mean"), v.get("max")],
                ["current (A)", c.get("min"), c.get("mean"), c.get("max")]]
        md.append(f"**BAT{i}**")
        md.append(table(["series", "min", "mean", "max"], rows))
        if "CurrTot" in gg.columns and len(gg):
            md.append(f"\nConsumed: {gg['CurrTot'].iloc[-1] - gg['CurrTot'].iloc[0]:.0f} mAh")
        if "Res" in gg.columns and np.isfinite(gg["Res"]).any():
            md.append(f"ArduPilot internal resistance estimate BAT.Res: {np.nanmean(gg['Res']) * 1000:.1f} mOhm")
        if ctun is not None and not ctun.empty and "ThO" in ctun.columns and "Curr" in gg.columns:
            thr = np.interp(gg["t"].values, ctun["t"].values, ctun["ThO"].values)
            cc = corr(thr, gg["Curr"].values)
            # A flat, throttle-independent reading is a wiring/pin fault, not a calibration
            # error: a wrong AMP_PERVLT changes the magnitude, never the correlation.
            status = PASS if cc > 0.8 else (WARN if cc > 0.3 else FAIL)
            res.append(Result(f"BAT{i} current sensing", status,
                              f"corr(throttle, current) = {cc:.3f}" +
                              ("" if cc > 0.8 else
                               " - a flat, throttle-independent current reading is a wiring or pin "
                               "fault, not a calibration error; a wrong BATT_AMP_PERVLT would change "
                               "the magnitude, not erase the correlation"),
                              evidence=dict(corr=cc), source="throttle-correlation method"))
            md.append(f"corr(throttle, current) = {cc:.3f}")
        md.append("")
    powr = w.clip(log.df("POWR"))
    if powr is not None and not powr.empty and "Vcc" in powr.columns and np.isfinite(powr["Vcc"]).any():
        s = describe(powr["Vcc"])
        res.append(_grade(s["min"], "vcc_min", higher_is_worse=False, name="board Vcc",
                          summary_fmt="min {v} V (warn <{w}, fail <{f})"))
        res.append(_grade(s["max"] - s["min"], "vcc_spread", name="board Vcc spread",
                          summary_fmt="{v} V spread (warn {w}, fail {f})"))
    elif powr is not None and not powr.empty:
        md.append("POWR.Vcc is NaN - this board has no board-voltage sensing. Not a fault.")
    if not res and not md:
        return Section("Power", [Result("power", SKIP, "no BAT or POWR messages")])
    md.append("\nAbsolute current scale is only trustworthy after a charger cross-check:\n"
              "fly a pack, note logged CurrTot mAh, recharge and read the mAh put back, then\n"
              "BATT_AMP_PERVLT_new = BATT_AMP_PERVLT * (logged / charger).")
    return Section("Power", res, "\n".join(md))


# ------------------------------------------------------------------------- GPS

def check_gps(log, w):
    gps = log.instances("GPS")
    if not gps:
        return Section("GPS", [Result("gps", SKIP, "no GPS messages")])
    res, rows, md = [], [], []
    for i, g in sorted(gps.items()):
        gg = w.clip(g)
        if gg is None or gg.empty:
            continue
        sats = describe(gg["NSats"]) if "NSats" in gg.columns else {}
        hdop = describe(gg["HDop"]) if "HDop" in gg.columns else {}
        status = gg["Status"].values if "Status" in gg.columns else np.array([])
        nofix = float((status < 3).mean() * 100) if status.size else np.nan
        rows.append([f"GPS{i}", sats.get("mean"), sats.get("min"), hdop.get("mean"),
                     hdop.get("max"), nofix])
        if np.isfinite(sats.get("min", np.nan)):
            res.append(_grade(sats["min"], "gps_sats", higher_is_worse=False, name=f"GPS{i} satellites",
                              summary_fmt="min {v} (warn <{w}, fail <{f})", mean=sats.get("mean")))
        if np.isfinite(hdop.get("max", np.nan)):
            res.append(_grade(hdop["max"], "gps_hdop", name=f"GPS{i} HDOP",
                              summary_fmt="max {v} (warn {w}, fail {f})", mean=hdop.get("mean")))
        if np.isfinite(nofix):
            res.append(_grade(nofix, "gps_nofix_pct", name=f"GPS{i} fix availability",
                              summary_fmt="{v}% of the airborne window without a 3D fix (warn {w}, fail {f})"))
    md.append(table(["gps", "sats mean", "sats min", "HDOP mean", "HDOP max", "% no 3D fix"], rows))
    if log.has("UBX2"):
        ub = log.instances("UBX2")
        md.append("\nUBX2 present for instance(s) " + ", ".join(str(k) for k in sorted(ub)) +
                  " - UBX2 is emitted only by the u-blox driver, so this identifies which physical\n"
                  "unit each GPS instance is. Never infer that from the instance index: the mapping\n"
                  "depends on SERIAL port order and can change between param snapshots.")
    if len(gps) > 1:
        md.append("\nWith two GPS units, the differential is the diagnostic. Both degraded similarly\n"
                  "implicates a shared external emitter; one much worse than the other implicates that\n"
                  "unit (self-jam, weak front end) rather than a uniform RF problem.")
    return Section("GPS", res, "\n".join(md))


# ------------------------------------------------------------------------- CPU

def check_cpu(log, w):
    pm = w.clip(log.df("PM"))
    if pm is None or pm.empty:
        return Section("CPU and memory", [Result("cpu", SKIP, "no PM messages")])
    res, md = [], []
    if "Load" in pm.columns:
        load = pm["Load"].values / 10.0          # PM.Load is percent x10
        s = describe(load)
        res.append(_grade(s["max"], "cpu_load", name="CPU load",
                          summary_fmt="peak {v}%% (warn {w}, fail {f}); mean %.1f%%" % s["mean"],
                          mean=s["mean"]))
    if {"NLon", "NL"} <= set(pm.columns):
        with np.errstate(divide="ignore", invalid="ignore"):
            slow = np.where(pm["NL"].values > 0, pm["NLon"].values / pm["NL"].values * 100, 0.0)
        res.append(_grade(float(np.nanmax(slow)), "cpu_slow_pct", name="slow loops",
                          summary_fmt="worst window {v}% of loops ran long (warn {w}, fail {f})",
                          total_long=int(pm["NLon"].sum())))
    rows = [[c, describe(pm[c])["mean"], describe(pm[c])["max"]]
            for c in ("Load", "NLon", "MaxT", "Mem", "I2CI", "ErrL") if c in pm.columns]
    md.append(table(["field", "mean", "max"], rows))
    md.append("\nPM.Load is percent x10 in the raw log (203 = 20.3%); the check above divides it.\n"
              "Free memory is PM.Mem in bytes. LogAnalyzer deliberately ignores PM.MaxT - it throws\n"
              "false positives around arm and disarm.")
    return Section("CPU and memory", res, "\n".join(md))


# ---------------------------------------------------------------------- events

def check_events(log, w):
    md, res = [], []
    ev = events(log)
    if ev:
        md.append("**EV**")
        md.append(table(["t (s)", "id", "event"], [[round(t, 1), i, n] for t, i, n in ev], align=["r", "r", "l"]))
    err = log.df("ERR")
    if not err.empty:
        md.append("\n**ERR** (subsystem errors)")
        md.append(table(["t (s)", "Subsys", "ECode"],
                        [[round(r["t"], 1), int(r["Subsys"]), int(r["ECode"])] for _, r in err.iterrows()]))
        res.append(Result("subsystem errors", FAIL if len(err) else PASS,
                          f"{len(err)} ERR records", evidence=dict(n=len(err)),
                          source="ardupilot LogAnalyzer TestEvents"))
    msgs = log.messages_text()
    if msgs:
        md.append("\n**MSG**")
        md.append("\n".join(f"  t={t:7.1f}  {m}" for t, m in msgs))
    if not md:
        return Section("Events and messages", [Result("events", SKIP, "no EV/ERR/MSG records")])
    return Section("Events and messages", res, "\n".join(md))


# ---------------------------------------------------- gust / disturbance response

def check_gust_response(log, w, dev_deg=2.5, cmd_deg=1.0):
    """Unrequested attitude excursions - the disturbance-rejection metric.

    An event is |actual - desired| > dev_deg while |desired| < cmd_deg, i.e. the
    aircraft moved when it was not asked to. Only meaningful in a mode where
    desired attitude is directly commanded (Stabilize, AltHold, Loiter).
    Report the event RATE, not the count: counts are not comparable between
    flights of different length.
    """
    att = w.clip(log.df("ATT"))
    if att is None or att.empty or not {"Roll", "DesRoll"} <= set(att.columns):
        return Section("Disturbance rejection", [Result("gust", SKIP, "no ATT desired/actual pair")])
    n_ev = 0
    detail = []
    for ax, a, d in (("roll", "Roll", "DesRoll"), ("pitch", "Pitch", "DesPitch")):
        err = np.abs(att[a].values - att[d].values)
        quiet = np.abs(att[d].values) < cmd_deg
        hit = (err > dev_deg) & quiet
        # count contiguous runs, not samples
        runs = int(np.sum(np.diff(hit.astype(int)) == 1)) + int(hit[0] if len(hit) else 0)
        n_ev += runs
        detail.append([ax, runs, float(runs / w.duration) if w.duration else np.nan,
                       float(err[quiet].max()) if quiet.any() else np.nan])
    rate = n_ev / w.duration if w.duration else np.nan
    res = [_grade(rate, "gust_event_rate", name="unrequested attitude excursions",
                  summary_fmt="{v} events/s (%d in %.0f s; warn {w}, fail {f})" % (n_ev, w.duration))]
    md = table(["axis", "events", "events/s", "max |err| while quiet (deg)"], detail)
    md += (f"\n\nEvent = |actual - desired| > {dev_deg} deg while |desired| < {cmd_deg} deg.\n"
           "Cross-check before blaming the tune: if motor headroom and clipping are both fine,\n"
           "this is a controller-gain problem rather than a power or noise problem. If GPS\n"
           "dropouts are suspected, correlate the event times against the outage windows before\n"
           "blaming either - in the development logs only 4 of 143 events overlapped, which\n"
           "ruled the GPS out.")
    return Section("Disturbance rejection", res, md)


# -------------------------------------------------------- batch IMU / notch proof

def check_batch_fft(log, w, window="hann"):
    """Pre/post-filter gyro spectra from INS_LOG_BAT batch samples.

    Method follows pymavlink's tools/mavfft_isb.py:
      * discard any batch window whose ISBD seqno is not contiguous (batch
        logging drops samples routinely; concatenating across a hole produces
        a spectrum that looks like a real peak and is not)
      * Hann window, PSD = 2 * mean(|rfft|^2) / (fs * sum(w^2)), DC and
        Nyquist zeroed
      * gyro converted to deg/s before transforming

    Pre- and post-filter batches are distinguished empirically by which has
    less energy above INS_GYRO_FILTER, because the ISBH `instance` encoding
    for pre+post logging is not self-describing.
    """
    isbh, isbd = log.df("ISBH"), log.df("ISBD")
    if isbh.empty or isbd.empty:
        return Section("Batch IMU spectra", [Result("batch", SKIP,
                       "no ISBH/ISBD - set INS_LOG_BAT_MASK=1 and INS_LOG_BAT_OPT=4, fly, "
                       "then set the mask back to 0")])
    by_n = {int(k): g for k, g in isbd.groupby("N")}
    groups = {}
    holes = 0
    for _, h in isbh.iterrows():
        n = int(h["N"])
        g = by_n.get(n)
        if g is None:
            continue
        seq = g["seqno"].values
        if len(seq) < 2 or not np.all(np.diff(seq) == 1):
            holes += 1
            continue
        x = np.concatenate([np.asarray(v, dtype=float) for v in g["x"].values])
        y = np.concatenate([np.asarray(v, dtype=float) for v in g["y"].values])
        z = np.concatenate([np.asarray(v, dtype=float) for v in g["z"].values])
        mul = float(h["mul"]) or 1.0
        stype, inst = int(h["type"]), int(h["instance"])
        fs = float(h["smp_rate"])
        t0 = float(h["TimeUS"]) / 1e6
        if not (w.t0 <= t0 <= w.t1):
            continue
        data = np.vstack([x, y, z]) / mul
        if stype == 1:                       # gyro: rad/s -> deg/s
            data = np.degrees(data)
        groups.setdefault((stype, inst), []).append((t0, fs, data))

    if not groups:
        return Section("Batch IMU spectra", [Result("batch", SKIP,
                       f"no usable batches inside the window ({holes} discarded for sequence holes)")])

    # Order-normalised accumulation: because the notch centre moves with RPM,
    # averaging raw spectra smears the notch across ~20 Hz and the dip vanishes.
    # Normalising each batch's frequency axis by the notch centre the FC was
    # tracking at that instant puts order 1.0 at the true fundamental every
    # time, which is what makes the attenuation measurable.
    fcns = log.instances("FCNS").get(0)
    order_axis = np.linspace(0.2, 4.0, 400)
    orders = {}

    spectra = {}
    for key, batches in groups.items():
        fs = batches[0][1]
        n = batches[0][2].shape[1]
        win = np.hanning(n) if window == "hann" else np.ones(n)
        s2 = float(np.inner(win, win))
        acc, cnt = None, 0
        for _t0, _fs, data in batches:
            if data.shape[1] != n:
                continue
            for row in data:
                f = np.fft.rfft(row * win)
                p = np.square(np.abs(f))
                p[0] = 0.0
                p[-1] = 0.0
                acc = p if acc is None else acc + p
                cnt += 1
        if cnt:
            freqs = np.fft.rfftfreq(n, 1.0 / fs)
            spectra[key] = (freqs, 2.0 * (acc / cnt) / (fs * s2), len(batches))
            if fcns is not None and not fcns.empty:
                oacc, ocnt = np.zeros_like(order_axis), 0
                for t0, _fs, data in batches:
                    if data.shape[1] != n:
                        continue
                    cf = float(np.interp(t0, fcns["t"].values, fcns["CF"].values))
                    if not np.isfinite(cf) or cf <= 1.0:
                        continue
                    for row in data:
                        pp = np.square(np.abs(np.fft.rfft(row * win)))
                        pp[0] = 0.0
                        pp[-1] = 0.0
                        pp = 2.0 * pp / (fs * s2)
                        oacc += np.interp(order_axis, freqs / cf, pp, left=0.0, right=0.0)
                        ocnt += 1
                if ocnt:
                    orders[key] = (order_axis, oacc / ocnt, ocnt)

    res, md = [], [f"Batches used: {sum(v[2] for v in spectra.values())}; "
                   f"{holes} discarded for ISBD sequence holes."]
    gyro = {k: v for k, v in spectra.items() if k[0] == 1}
    p = log.params()
    gyro_filt = p.get("INS_GYRO_FILTER", 42.0)
    if len(gyro) >= 2:
        # more energy above the low-pass corner => pre-filter
        def hf_energy(v):
            f, ps, _ = v
            m = f > gyro_filt * 1.5
            return float(ps[m].sum())
        order = sorted(gyro.items(), key=lambda kv: -hf_energy(kv[1]))
        (pre_k, pre), (post_k, post) = order[0], order[-1]
        md.append(f"\nPre-filter identified as ISBH instance {pre_k[1]}, post-filter as {post_k[1]} "
                  f"(by energy above {gyro_filt * 1.5:.0f} Hz).")
        t, fund = esc_fundamental(log)
        f0 = float(np.nanmedian(fund[w.mask(t)])) if fund is not None else None
        rows = []
        for label, lo, hi in ([("fundamental", f0 * 0.92, f0 * 1.08),
                               ("2nd harmonic", f0 * 1.92, f0 * 2.08)] if f0 else []) + \
                             [("90-1000 Hz total", 90.0, 1000.0)]:
            fpre, ppre, _ = pre
            fpost, ppost, _ = post
            mp = (fpre >= lo) & (fpre <= hi)
            mq = (fpost >= lo) & (fpost <= hi)
            a, b = float(ppre[mp].sum()), float(ppost[mq].sum())
            db = 10 * np.log10(b / a) if a > 0 and b > 0 else np.nan
            rows.append([f"{label} ({lo:.0f}-{hi:.0f} Hz)", a, b, db])
            if label == "fundamental":
                res.append(_grade(db, "notch_atten_db", higher_is_worse=True,
                                  name="notch attenuation at the fundamental",
                                  summary_fmt="{v} dB post/pre (warn >{w}, fail >{f})"))
        md.append("")
        md.append(table(["band", "pre", "post", "post/pre dB"], rows))
        if pre_k in orders and post_k in orders:
            oa, opre, _ = orders[pre_k]
            _, opost, _ = orders[post_k]
            with np.errstate(divide="ignore", invalid="ignore"):
                tf = np.where(opre > 0, opost / opre, np.nan)
            def _dip(centre, half=0.12):
                m = np.abs(oa - centre) <= half
                if not m.any() or not np.isfinite(tf[m]).any():
                    return np.nan, np.nan
                j = np.nanargmin(tf[m])
                return float(oa[m][j]), float(10 * np.log10(tf[m][j]))
            o1, d1 = _dip(1.0)
            o2, d2 = _dip(2.0)
            md.append("")
            md.append(table(["order-normalised", "deepest at order", "dB there"],
                            [["fundamental", o1, d1], ["2nd harmonic", o2, d2]]))
            md.append(f"\nEach batch was normalised by the notch centre the FC was tracking at that\n"
                      f"instant, so order 1.0 is always the true fundamental. The attenuation minimum\n"
                      f"landing within a percent of order 1.0 (and 2.0) is the evidence that the notch\n"
                      f"is applied where the noise actually is, not merely configured to be.")
            for o, tgt, label in ((o1, 1.0, "fundamental"), (o2, 2.0, "2nd harmonic")):
                if np.isfinite(o):
                    res.append(Result(f"notch placement at the {label}",
                                      PASS if abs(o - tgt) / tgt < 0.02 else WARN,
                                      f"deepest attenuation at order {o:.3f} (target {tgt:.1f}, "
                                      f"error {abs(o - tgt) / tgt * 100:.2f}%)",
                                      evidence=dict(order=o, target=tgt),
                                      source="order-normalisation method"))
        md.append("\nCaveat: this post/pre ratio is the notch PLUS the INS_GYRO_FILTER low-pass, which\n"
                  "sits after it and attenuates everything above its corner. To isolate the notch alone,\n"
                  "fit a smooth baseline to the transfer function outside the notch bands (excluding\n"
                  "+/-20% around orders 1 and 2) and measure the dip below it.")
    else:
        md.append("\nOnly one gyro batch series present - set INS_LOG_BAT_OPT=4 to log pre AND post filter.")
    for (stype, inst), (f, ps, nb) in sorted(spectra.items()):
        kind = "gyro" if stype == 1 else "accel"
        md.append(f"\n{kind} instance {inst}: {nb} batches, peak at {peak_hz(f, ps, 40, 500):.1f} Hz")
    return Section("Batch IMU spectra (notch proof)", res, "\n".join(md),
                   data=dict(spectra=spectra, orders=orders))


ALL_CHECKS = [
    ("summary", check_summary),
    ("events", check_events),
    ("vibe", check_vibration),
    ("motors", check_motors),
    ("notch", check_notch),
    ("pid", check_pids),
    ("gust", check_gust_response),
    ("ekf", check_ekf),
    ("compass", check_compass),
    ("power", check_power),
    ("gps", check_gps),
    ("cpu", check_cpu),
    ("batchfft", check_batch_fft),
]


def run(log, names=None, window=None, method="auto", **kw):
    """Run checks by name (default: all). Returns [Section]."""
    w = window or airborne_window(log, method=method)
    out = []
    for name, fn in ALL_CHECKS:
        if names and name not in names:
            continue
        try:
            out.append(fn(log, w, **({} if fn is not check_motors else
                                     {k: v for k, v in kw.items() if k == "normalise"})))
        except Exception as exc:  # a broken check must never kill the report
            out.append(Section(name, [Result(name, SKIP, f"check raised {type(exc).__name__}: {exc}")]))
    return out
