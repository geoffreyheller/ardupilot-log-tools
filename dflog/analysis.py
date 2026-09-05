"""The standard battery of checks.

Each `check_*` function takes (log, window) and returns a Section: a title, a list of
Result, and structured parts (tables and notes) that render as markdown for a human and
as JSON for an agent. They are deliberately independent so a session can run just the
one it needs, and deliberately uniform so `alog all` can run every one and produce a
report that looks the same every time.

Rules every check here follows, and any new one must too:
  * State the number, not just the verdict.
  * Cite the window and the method that chose it.
  * Never invent a threshold - take it from dflog.checks.T and cite the source.
  * Report "not logged" as SKIP, never as a pass.
  * Report a defaulted parameter as defaulted. A check that silently assumes
    MOT_SPIN_MAX=0.95 because PARM was not logged is lying about its inputs.
"""

from __future__ import annotations

import numpy as np

from .checks import T, Result, PASS, WARN, FAIL, SKIP
from .flight import EVENTS, airborne_window, esc_fundamental, events, flights, mode_timeline
from .frames import mix_for, trim_decomposition, motor_channels
from .report import table, fmt, heading
from .stats import band_split, corr, describe, pct_above, peak_hz

__all__ = ["Section", "ALL_CHECKS", "run", "check_summary", "check_integrity",
           "check_coverage", "check_vibration", "check_motors", "check_notch",
           "check_pids", "check_ekf", "check_compass", "check_power", "check_gps",
           "check_cpu", "check_events", "check_gust_response", "check_batch_fft",
           "check_fft", "check_imu", "check_brownout", "check_params", "check_flight",
           "check_estimates"]


class Section:
    """One check's output: results plus an ordered list of tables and notes."""

    def __init__(self, title, results=None, md="", data=None, key=None):
        self.title = title
        self.key = key
        self.results = list(results or [])
        self.parts = []            # ("table", dict) | ("note", str)
        self.data = data or {}
        if md:
            self.note(md)

    # -- building
    def table(self, name, headers, rows, align=None):
        self.parts.append(("table", dict(name=name, columns=list(headers),
                                         rows=[list(r) for r in rows], align=align)))
        return self

    def note(self, text):
        if text:
            self.parts.append(("note", str(text)))
        return self

    def add(self, result):
        self.results.append(result)
        return result

    # -- reading
    @property
    def worst(self):
        return max(self.results, key=lambda r: r.rank).status if self.results else SKIP

    @property
    def tables(self):
        return [p for k, p in self.parts if k == "table"]

    @property
    def notes(self):
        return [p for k, p in self.parts if k == "note"]

    def render(self, level=2):
        out = [heading(self.title, level)]
        for r in self.results:
            out.append(r.line())
        for kind, p in self.parts:
            out.append("")
            if kind == "table":
                out.append(table(p["columns"], p["rows"], align=p["align"]))
            else:
                out.append(p)
        return "\n".join(out)

    def to_dict(self):
        return dict(key=self.key, title=self.title, worst=self.worst,
                    results=[r.to_dict() for r in self.results],
                    tables=[dict(name=t["name"], columns=t["columns"],
                                 rows=[[_jsonable(c) for c in r] for r in t["rows"]])
                            for t in self.tables],
                    notes=self.notes)


def _jsonable(v):
    if isinstance(v, np.generic):
        v = v.item()
    if isinstance(v, float) and not np.isfinite(v):
        return None
    if isinstance(v, np.ndarray):
        return [_jsonable(x) for x in v.tolist()]
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]
    if isinstance(v, dict):
        return {str(k): _jsonable(x) for k, x in v.items()}
    return v


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


class _Params:
    """Parameter lookup that records every default it had to fall back on, so the
    section can say which of its inputs were assumed rather than read."""

    def __init__(self, log):
        self.p = log.params()
        self.defaulted = {}

    def get(self, name, default=None):
        if name in self.p:
            return self.p[name]
        self.defaulted[name] = default
        return default

    def __contains__(self, name):
        return name in self.p

    def note(self, sec):
        if self.defaulted:
            sec.note("Parameters NOT in the log, defaults assumed: "
                     + ", ".join(f"{k}={fmt(v)}" for k, v in sorted(self.defaulted.items()))
                     + ". Every number above that depends on one of these is conditional on it.")


# --------------------------------------------------------------------- summary

def check_summary(log, w):
    lo, hi = log.duration()
    p = log.params()
    seen, fw = set(), []
    for t, m in log.messages_text():
        m = str(m)
        if m in seen:
            continue
        if any(k in m for k in ("ArduCopter", "ArduPlane", "Rover", "ArduSub", "ChibiOS",
                                "fast sampling", "RCOut:", "RC Protocol", "Frame:", "IMU")):
            seen.add(m)
            fw.append(m)
    modes = mode_timeline(log)
    mode_names = []
    for i, (t, num, name, _) in enumerate(modes):
        dur = (modes[i + 1][0] if i + 1 < len(modes) else (hi or t)) - t
        if dur > 1 and (not mode_names or mode_names[-1][0] != name):
            mode_names.append((name, dur))
    info = log.info()
    fl = flights(log, method="auto")
    fl_spans = [(f.t0, f.t1) for f in fl]
    rows = [
        ["log", info["file_name"]],
        ["size", f"{info['file_size']} bytes, sha256 {info['sha256'][:16]}..."],
        ["firmware", info["firmware"] or "(not logged)"],
        ["board", info["board"] or "(not logged)"],
        ["messages", f"{log.n_messages} in {len(log.messages)} types present "
                     f"({len(log.formats_by_name)} declared)"],
        ["integrity", log.diagnostics.summary()],
        ["log span", f"{lo:.1f} - {hi:.1f} s ({hi - lo:.1f} s)" if lo is not None else "no timestamps"],
        ["UTC start", info["log_start_utc"] or "unknown (no GPS time in log; a 1980 file date is the unset RTC)"],
        ["airborne window", f"{w.t0:.1f} - {w.t1:.1f} s ({w.duration:.1f} s) via {w.method}"],
        ["flights in log", ("; ".join(f"{i}: {a:.1f}-{b:.1f} s" for i, (a, b) in enumerate(fl_spans, 1))
                            or "none found by any detector") if fl_spans else "none found by any detector"],
        ["frame", f"FRAME_CLASS={fmt(p.get('FRAME_CLASS'))} FRAME_TYPE={fmt(p.get('FRAME_TYPE'))}"],
        ["params in log", str(len(p))],
        ["modes flown", ", ".join(f"{n} {d:.0f}s" for n, d in mode_names) or "-"],
    ]
    sec = Section("Summary", key="summary")
    sec.table("summary", ["item", "value"], rows, align=["l", "l"])
    if fw:
        sec.note("Firmware banner:\n```\n" + "\n".join("  " + m for m in fw) + "\n```")
    sec.data["info"] = info
    sec.data["flights"] = [dict(index=f.index, t0=f.t0, t1=f.t1, duration_s=f.duration) for f in fl]
    return sec


# ------------------------------------------------------------------- integrity

def check_integrity(log, w):
    """Log integrity: every structural and data-quality problem the parser found."""
    sec = Section("Log integrity", key="integrity")
    d = log.diagnostics
    q = log.quality()
    n_err, n_warn = len(d.errors), len(d.warnings)
    status = FAIL if n_err else (WARN if n_warn or q.warnings else PASS)
    sec.add(Result("structure", status,
                   f"{n_err} error(s), {n_warn} warning(s), {len(d.infos)} info; "
                   f"{log.bytes_parsed} of {log.file_size} bytes decoded, "
                   f"{log.resync_bytes} bytes skipped in {log.resync_events} resync(s)",
                   evidence=dict(errors=n_err, warnings=n_warn, resync_bytes=log.resync_bytes,
                                 resync_events=log.resync_events, bytes_parsed=log.bytes_parsed,
                                 file_size=log.file_size, codes=[i.code for i in d.issues]),
                   source="dflog parser"))
    if d.has("TRUNCATED_TAIL"):
        arm = [t for t, i, _ in events(log) if i == 10]
        dis = [t for t, i, _ in events(log) if i == 11]
        still_armed = bool(arm) and (not dis or max(dis) < max(arm))
        sec.add(Result("truncation", WARN if still_armed else PASS,
                       "log ends mid-message"
                       + (" while still ARMED - power loss or a full card in flight"
                          if still_armed else " but after DISARM - only the tail of the log is lost"),
                       evidence=dict(still_armed=still_armed), source="dflog parser"))
    sec.add(Result("data quality", WARN if q.warnings else PASS,
                   f"{len(q.warnings)} warning(s), {len(q.infos)} info (NaN fields, logging gaps, "
                   "duplicate data)",
                   evidence=dict(codes=[i.code + ":" + str(i.subject) for i in q.issues]),
                   source="dflog parser; LogAnalyzer TestNaN allow-list, TestDupeLogData"))
    rows = [[i.severity, i.code, i.subject if i.subject is not None else "", i.count,
             i.offsets[0] if i.offsets else "", i.message]
            for i in list(d.issues) + list(q.issues)]
    if rows:
        sec.table("issues", ["severity", "code", "subject", "count", "first byte", "message"],
                  rows, align=["l", "l", "l", "r", "r", "l"])
    else:
        sec.note("No structural or data-quality issues. Every byte of the file was decoded.")
    sec.note("Codes are documented in reference/integrity-codes.md. Nothing listed here was "
             "repaired: the numbers in the rest of the report are computed on the data as "
             "logged, gaps and all.")
    return sec


# -------------------------------------------------------------------- coverage

def check_coverage(log, w):
    """What was logged, at what rate - and what that rules out."""
    sec = Section("Logging coverage", key="coverage")
    key_msgs = [
        ("ATT", "attitude"), ("RATE", "rate loop"), ("IMU", "IMU (filtered)"),
        ("GYR", "raw gyro"), ("ACC", "raw accel"), ("ISBD", "batch IMU"), ("VIBE", "vibration"),
        ("RCOU", "motor outputs"), ("ESC", "ESC telemetry"), ("FCNS", "notch centre"),
        ("FTN1", "in-flight FFT"), ("PIDR", "PID roll"), ("CTUN", "throttle/alt"),
        ("XKF4", "EKF innovations"), ("AHR2", "DCM attitude"), ("MAG", "compass"), ("GPS", "GPS"),
        ("BAT", "battery"), ("POWR", "board power"), ("PM", "scheduler"), ("RCIN", "RC input"),
        ("MODE", "modes"), ("EV", "events"), ("ERR", "errors"), ("MSG", "messages"),
        ("PARM", "parameters"),
    ]
    rows = []
    rates = {}
    for m, what in key_msgs:
        if log.has(m):
            r = log.rate_hz(m, steady_only=True)
            n_inst = len(log.instances(m)) if m not in ("PARM", "MSG", "EV", "ERR", "MODE") else 1
            rates[m] = r
            rows.append([m, what, len(log.raw(m)), n_inst, f"{r:.1f}" if r else "event-driven / burst"])
        else:
            rows.append([m, what, 0, 0, "absent"])
    sec.table("coverage", ["message", "what", "records", "instances", "rate Hz per instance"],
              rows, align=["l", "l", "r", "r", "r"])
    p = log.params()
    bitmask = p.get("LOG_BITMASK")
    fast_gyro = max((r for m, r in rates.items() if m in ("GYR", "IMU") and r), default=None)
    if log.has("ISBD"):
        isbh = log.df("ISBH")
        if not isbh.empty:
            fast_gyro = max(fast_gyro or 0.0, float(np.median(isbh["smp_rate"])))
    t, fund = esc_fundamental(log)
    f0 = float(np.nanmedian(fund[w.mask(t)])) if fund is not None and w.mask(t).any() else None
    if fast_gyro and f0:
        ok = fast_gyro / 2 > f0 * 2.2
        sec.add(Result("spectral reach", PASS if ok else WARN,
                       f"fastest gyro source {fast_gyro:.0f} Hz (Nyquist {fast_gyro / 2:.0f} Hz) vs "
                       f"motor fundamental {f0:.0f} Hz"
                       + ("" if ok else " - cannot resolve the 2nd harmonic; set INS_LOG_BAT_MASK=1 "
                                        "or INS_RAW_LOG_OPT for FFT work"),
                       evidence=dict(gyro_rate_hz=fast_gyro, fundamental_hz=f0),
                       source="Nyquist"))
    elif fast_gyro:
        sec.add(Result("spectral reach", PASS,
                       f"fastest gyro source {fast_gyro:.0f} Hz (Nyquist {fast_gyro / 2:.0f} Hz); no ESC "
                       "telemetry to compare against", evidence=dict(gyro_rate_hz=fast_gyro),
                       source="Nyquist"))
    dsf = log.df("DSF")
    if not dsf.empty and "Dp" in dsf.columns:
        dropped = int(dsf["Dp"].max())
        sec.add(Result("dropped records", PASS if dropped == 0 else WARN,
                       f"logger reports {dropped} dropped record(s) (DSF.Dp) - buffer starvation on the "
                       "logging backend", evidence=dict(dropped=dropped), source="AP_Logger DSF"))
    sec.note(f"LOG_BITMASK={fmt(bitmask)}. INS_LOG_BAT_MASK={fmt(p.get('INS_LOG_BAT_MASK'))} "
             f"INS_LOG_BAT_OPT={fmt(p.get('INS_LOG_BAT_OPT'))} INS_RAW_LOG_OPT={fmt(p.get('INS_RAW_LOG_OPT'))}. "
             "A message absent from this table is SKIP everywhere below, never a pass.")
    return sec


# ------------------------------------------------------------------- vibration

def check_vibration(log, w):
    d = w.clip(log.df("VIBE"))
    if d is None or d.empty:
        return Section("Vibration", [Result("vibration", SKIP, "no VIBE messages in log")], key="vibe")
    inst = log.instance_field("VIBE")
    sec = Section("Vibration", key="vibe")
    rows = []
    groups = d.groupby(inst) if inst else [(0, d)]
    for i, g in groups:
        for ax, key in (("VibeX", "vibe_xy"), ("VibeY", "vibe_xy"), ("VibeZ", "vibe_z")):
            if ax not in g.columns:
                continue
            s = describe(g[ax])
            rows.append([f"IMU{i} {ax}", s["mean"], s.get("p95"), s["max"]])
            sec.add(_grade(s.get("p95"), key, name=f"IMU{i} {ax} p95",
                           summary_fmt="p95 {v} m/s^2 (warn {w}, fail {f})",
                           mean=s["mean"], max=s["max"]))
        if "Clip" in g.columns and len(g):
            clip = float(g["Clip"].iloc[-1] - g["Clip"].iloc[0])
            sec.add(_grade(clip, "clip_events", name=f"IMU{i} accel clipping",
                           summary_fmt="{v} clip events in the window (warn >{w}, fail >{f})"))
    sec.table("vibe", ["axis", "mean", "p95", "max"], rows)
    sec.note("Units are m/s^2. ArduPilot's rule of thumb: below 15 is good, above 30 is a problem.\n"
             "Clipping is the harder failure - any non-zero clip count means the accelerometer\n"
             "saturated and the EKF was fed garbage for those samples.")
    return sec


# ---------------------------------------------------------------------- motors

def check_motors(log, w, normalise=True):
    rcou = w.clip(log.df("RCOU"))
    p = _Params(log)
    esc = log.instances("ESC")
    if (rcou is None or rcou.empty) and not esc:
        return Section("Motors", [Result("motors", SKIP, "no RCOU or ESC messages")], key="motors")

    n = 0
    if rcou is not None and not rcou.empty:
        for i in range(1, 13):
            c = f"C{i}"
            if c in rcou.columns and rcou[c].std() > 0.5 and rcou[c].mean() > 900:
                n = i
    n = max(n, len(esc))
    mix = mix_for(p.get("FRAME_CLASS", 1), p.get("FRAME_TYPE", 1), n_motors=n, normalise=normalise)
    sec = Section("Motors and standing trim", key="motors", data=dict(mix=mix.label))

    # --- per-motor RCOU and the trim decomposition
    if rcou is not None and not rcou.empty and n >= 3:
        chans = motor_channels(p.p, mix.n)
        mapped = chans is not None
        if not mapped:
            chans = [f"C{i + 1}" for i in range(min(n, mix.n))]
        means = [float(rcou[c].mean()) for c in chans if c in rcou.columns]
        if len(means) == mix.n:
            tr = trim_decomposition(means, mix)
            sec.table("rcou_means", ["motor"] + [f"M{i + 1} ({c})" for i, c in enumerate(chans)],
                      [["RCOU mean (us)"] + [round(m, 1) for m in means]])
            sec.table("trim", ["axis", "trim (us)", "reads as"],
                      [["roll", round(tr["roll"], 1), "mean(left) - mean(right)"],
                       ["pitch", round(tr["pitch"], 1), "mean(front) - mean(rear); negative = CG aft"],
                       ["yaw", round(tr["yaw"], 1), "mean(CCW) - mean(CW); non-zero = standing torque"],
                       ["residual", round(tr["residual"], 2), "unexplained by any control axis"]],
                      align=["l", "r", "l"])
            for ax in ("roll", "pitch", "yaw"):
                sec.add(_grade(abs(tr[ax]), "trim_us", name=f"{ax} trim",
                               summary_fmt="%s us standing trim (warn {w}, fail {f})" % fmt(tr[ax]),
                               signed=tr[ax],
                               channel_map=dict(zip([f"M{i + 1}" for i in range(len(chans))], chans)),
                               map_source="SERVOn_FUNCTION" if mapped else "ASSUMED identity"))
            sec.note(f"Mix: {mix.label}"
                     f"{'' if normalise else ' [un-normalised cos factors, legacy mode]'}\n"
                     "Channel -> motor map from SERVOn_FUNCTION: "
                     + ", ".join(f"M{i + 1}={c}" for i, c in enumerate(chans))
                     + ("." if mapped else
                        " (ASSUMED - no SERVOn_FUNCTION motor assignments in this log; the axis "
                        "labels above are unverified)."))
            if not mapped:
                sec.add(Result("channel-to-motor map", WARN,
                               "no SERVOn_FUNCTION in log: C1..Cn assumed to be motors 1..n; a "
                               "non-identity map permutes the trim axes",
                               evidence=dict(assumed=True), source="dflog frames.motor_channels"))

    # --- ESC RPM spread and error rate
    if esc:
        rows, rpm_meds, err_means = [], [], []
        for i, g in sorted(esc.items()):
            gg = w.clip(g)
            if gg is None or gg.empty:
                continue
            r = describe(gg["RPM"]) if "RPM" in gg.columns else {}
            # Grade on the median: means are pulled around by spin-up and by brief saturation.
            rpm_meds.append(r.get("p50", np.nan))
            err = describe(gg["Err"]) if "Err" in gg.columns else {}
            if err.get("mean") is not None and np.isfinite(err.get("mean", np.nan)):
                err_means.append(err["mean"])
            rows.append([f"ESC{i}", r.get("mean"), r.get("p50"), r.get("p50", np.nan) / 60.0,
                         r.get("min"), r.get("max"), err.get("mean"), err.get("p95")])
        if rows:
            sec.table("esc", ["esc", "RPM mean", "RPM median", "Hz", "RPM min", "RPM max",
                              "Err% mean", "Err% p95"], rows)
            rpm_meds = np.array(rpm_meds, dtype=float)
            if np.isfinite(rpm_meds).all() and rpm_meds.mean() > 0:
                spread = (rpm_meds.max() - rpm_meds.min()) / rpm_meds.mean() * 100
                sec.add(_grade(spread, "rpm_spread_pct", name="RPM spread",
                               summary_fmt="{v}% across motors, on medians (warn {w}, fail {f})",
                               per_motor_median=list(np.round(rpm_meds, 1))))
            if err_means:
                sec.add(_grade(max(err_means), "esc_err_pct", name="bidir DShot error rate",
                               summary_fmt="worst motor {v}% (warn {w}, fail {f})"))
            sec.note("RPM spread alone is a poor statistic - it says the motors disagree but not how.\n"
                     "The trim decomposition above is what separates a CG offset (pitch/roll) from a\n"
                     "torque asymmetry (yaw). ESC instance i is servo output i+1, so it needs the same\n"
                     "channel map before a motor's RPM is attributed to a corner.")

    # --- headroom against the MOT_SPIN_MAX ceiling
    if rcou is not None and not rcou.empty and n >= 3:
        spin_max = p.get("MOT_SPIN_MAX", 0.95)
        lo_pwm = p.get("SERVO1_MIN", 1000.0)
        hi_pwm = p.get("SERVO1_MAX", 2000.0)
        ceiling = lo_pwm + spin_max * (hi_pwm - lo_pwm)
        chans_present = [f"C{i+1}" for i in range(min(n, mix.n)) if f"C{i+1}" in rcou.columns]
        allout = np.concatenate([rcou[c].values for c in chans_present])
        # p99.5, not the raw max: a single sample touching the ceiling during a hard
        # manoeuvre is not "saturated", sustained time there is.
        peak = float(np.percentile(allout, 99.5))
        hard_max = float(allout.max())
        sat_pct = float((allout >= ceiling - 1).mean() * 100.0)
        frac = (peak - lo_pwm) / (ceiling - lo_pwm) if ceiling > lo_pwm else np.nan
        sec.add(_grade(frac, "motor_headroom", name="motor headroom",
                       summary_fmt="p99.5 output %.0f us = {v} of the MOT_SPIN_MAX ceiling %.0f us "
                                   "(warn {w}, fail {f})" % (peak, ceiling),
                       peak_us=peak, hard_max_us=hard_max, ceiling_us=ceiling, saturated_pct=sat_pct))
        sec.note(f"Saturation ceiling is SERVO_MIN + MOT_SPIN_MAX * (MAX-MIN) = {ceiling:.0f} us, "
                 f"not {hi_pwm:.0f}. p99.5 output {peak:.0f} us, hard max {hard_max:.0f} us, "
                 f"at or above the ceiling for {sat_pct:.2f}% of samples.")
    motb = w.clip(log.df("MOTB"))
    if motb is not None and not motb.empty and "ThLimit" in motb.columns:
        lim = float(motb["ThLimit"].min())
        sec.add(Result("throttle limiting", PASS if lim >= 0.999 else WARN,
                       f"MOTB.ThLimit min {lim:.3f}" + ("" if lim >= 0.999 else
                                                        " - the mixer ran out of authority"),
                       evidence=dict(thlimit_min=lim), source="MOTB.ThLimit"))
    p.note(sec)
    return sec


# ----------------------------------------------------------------------- notch

def check_notch(log, w):
    p = _Params(log)
    mode = p.get("INS_HNTCH_MODE")
    fcns = log.instances("FCNS")
    t, fund = esc_fundamental(log)
    sec = Section("Harmonic notch", key="notch")
    mode_names = {0: "Fixed", 1: "Throttle", 2: "RPM sensor", 3: "ESC telemetry", 4: "In-flight FFT"}
    sec.note(f"INS_HNTCH_ENABLE={fmt(p.get('INS_HNTCH_ENABLE'))} "
             f"MODE={fmt(mode)} ({mode_names.get(int(mode) if mode is not None else -1, '?')}) "
             f"FREQ={fmt(p.get('INS_HNTCH_FREQ'))} BW={fmt(p.get('INS_HNTCH_BW'))} "
             f"REF={fmt(p.get('INS_HNTCH_REF'))} HMNCS={fmt(p.get('INS_HNTCH_HMNCS'))}")
    if fund is None:
        sec.add(Result("notch", SKIP,
                       "no ESC telemetry, so there is no ground truth to check the notch against"))
        return sec
    if not fcns:
        sec.note("No FCNS messages: the applied notch centre frequency was not logged, so the "
                 "notch cannot be verified. FTN1.PkAvg is the FFT's opinion, not the filter's setting.")
        sec.add(Result("notch", SKIP, "FCNS not logged"))
        return sec

    rows = []
    for i, g in sorted(fcns.items()):
        gg = w.clip(g)
        if gg is None or gg.empty or "CF" not in gg.columns:
            continue
        f_at = np.interp(gg["t"].values, t, fund)
        ok = f_at > 1.0
        ratio = gg["CF"].values[ok] / f_at[ok]
        s = describe(ratio)
        mis = pct_above(ratio, 1.5)
        rows.append([f"notch {i} CF / fundamental", s["mean"], s.get("p05"), s.get("p50"), s.get("p95")])
        if "HF" in gg.columns and np.nanmedian(gg["HF"].values) > 0:
            hr = gg["HF"].values[ok] / f_at[ok]
            hs = describe(hr)
            rows.append([f"notch {i} HF / fundamental", hs["mean"], hs.get("p05"), hs.get("p50"), hs.get("p95")])
        sec.add(_grade(abs(s.get("p95", np.nan) - 1.0), "notch_track", name=f"notch {i} tracking",
                       summary_fmt="p95 error {v} of the fundamental (warn {w}, fail {f})",
                       median_ratio=s.get("p50")))
        sec.add(_grade(mis, "notch_mistrack_pct", name=f"notch {i} harmonic lock-on",
                       summary_fmt="{v}% of the window above 1.5x the fundamental (warn {w}, fail {f})"))
    if rows:
        sec.table("tracking", ["series", "mean", "p05", "p50", "p95"], rows)

    freq_floor = p.get("INS_HNTCH_FREQ")
    if freq_floor and w.mask(t).any():
        below = pct_above(-fund[w.mask(t)], -freq_floor)
        sec.note(f"Motor fundamental below the INS_HNTCH_FREQ floor of {fmt(freq_floor)} Hz for "
                 f"{below:.2f}% of the airborne window (airborne min "
                 f"{np.nanmin(fund[w.mask(t)]):.1f} Hz). If that is 0.00%, the floor is untested by "
                 f"this flight - neither justified nor disproved.")

    ftn1 = w.clip(log.df("FTN1"))
    if ftn1 is not None and not ftn1.empty and "PkAvg" in ftn1.columns:
        f_at = np.interp(ftn1["t"].values, t, fund)
        ok = f_at > 1.0
        r = ftn1["PkAvg"].values[ok] / f_at[ok]
        sec.note(f"FFT observer (FTN1.PkAvg): median {np.nanmedian(r):.3f} x fundamental, "
                 f"{pct_above(r, 1.5):.2f}% above 1.5x. "
                 f"{'This only matters if MODE=4 - otherwise it is inert.' if mode != 4 else ''}")
    p.note(sec)
    return sec


# ------------------------------------------------------------------------ PIDs

def check_pids(log, w, cut_hz=5.0):
    rate = w.clip(log.df("RATE"))
    att = w.clip(log.df("ATT"))
    sec = Section("Attitude and rate control", key="pid")
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
            sec.add(_grade(c, "rate_corr", higher_is_worse=False, name=f"{ax} rate tracking",
                           summary_fmt="desired-vs-actual correlation {v} (warn <{w}, fail <{f})",
                           err_sd_low=float(np.std(low)), err_sd_high=float(np.std(high))))
        sec.table("rate", ["axis", "des sd", "act sd", "corr",
                           f"err sd <{cut_hz:g}Hz", f"err sd >{cut_hz:g}Hz", "out sd", "out peak"], rows)
        r_hz = log.rate_hz("RATE")
        sec.note(f"Rates are deg/s, logged at {r_hz:.0f} Hz. Splitting the rate error at {cut_hz:g} Hz "
                 "separates genuine tracking\nerror (low band - a gain problem) from gyro noise "
                 "reaching the controller (high band -\na filter problem). Do the filter work before "
                 "touching a rate gain."
                 + (f" NOTE: at {r_hz:.0f} Hz logging the high band is only {cut_hz:g}-{r_hz / 2:.0f} Hz."
                    if r_hz and r_hz < 4 * cut_hz else ""))
    for msg, ax in (("PIDR", "roll"), ("PIDP", "pitch"), ("PIDY", "yaw")):
        d = w.clip(log.df(msg))
        if d is None or d.empty:
            continue
        parts = []
        if "Dmod" in d.columns:
            dm = describe(d["Dmod"])
            engaged = pct_above(-d["Dmod"].values, -0.999)
            parts.append(f"Dmod min {dm['min']:.3f}, engaged {engaged:.2f}% of samples")
            sec.add(Result(f"{ax} D-term slew limiter",
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
        if "Flags" in d.columns:
            lim_pct = float((d["Flags"].values.astype(int) & 1 > 0).mean() * 100)
            parts.append(f"output limited {lim_pct:.2f}% of samples")
        if parts:
            sec.note(f"**{ax}**: " + "; ".join(parts))
    if att is not None and not att.empty and {"Roll", "DesRoll"} <= set(att.columns):
        rows = []
        for ax, a, dd in (("roll", "Roll", "DesRoll"), ("pitch", "Pitch", "DesPitch")):
            e = att[a].values - att[dd].values
            s = describe(np.abs(e))
            rows.append([ax, float(np.std(e)), s.get("p95"), s["max"]])
            sec.add(_grade(s["max"], "att_err_deg", name=f"{ax} attitude error",
                           summary_fmt="max {v} deg (warn {w}, fail {f})", sd=float(np.std(e))))
        sec.table("attitude", ["axis", "err sd (deg)", "|err| p95", "|err| max"], rows)
    if not sec.results:
        return Section("Attitude and rate control", [Result("pids", SKIP, "no RATE/ATT messages")], key="pid")
    return sec


# ------------------------------------------------------------------------- EKF

def check_ekf(log, w):
    sec = Section("EKF", key="ekf")
    xkf4 = log.instances("XKF4")
    if not xkf4:
        sec.add(Result("ekf", SKIP, "no XKF4 messages"))
        return sec
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
            sec.add(_grade(s["max"], "ekf_innov", name=f"core{c} {f} innovation",
                           summary_fmt="max {v} (reject at 1.0; warn {w}, fail {f})"
                                       + (f", {over} samples over 1.0" if over else ""),
                           samples_over_1=over, mean=s["mean"]))
        if "errRP" in gg.columns:
            s = describe(gg["errRP"])
            sec.add(_grade(s["max"], "ekf_errRP", name=f"core{c} errRP",
                           summary_fmt="max {v} (warn {w}, fail {f})"))
        if "SS" in gg.columns:
            ss = gg["SS"].values.astype(int)
            bits = {0: "attitude", 1: "horiz_vel", 2: "vert_vel", 3: "horiz_pos_rel", 4: "horiz_pos_abs",
                    5: "vert_pos", 7: "const_pos_mode", 14: "gps_glitching"}
            never = [n for b, n in bits.items() if b in (0, 1, 2, 3, 5) and not ((ss >> b) & 1).all()]
            glitch = float(((ss >> 14) & 1).mean() * 100)
            sec.add(Result(f"core{c} solution status", PASS if not never and glitch == 0 else WARN,
                           ("all core flags set throughout" if not never else
                            "flag(s) not set for the whole window: " + ", ".join(never))
                           + (f"; GPS glitching flag set {glitch:.1f}% of the time" if glitch else ""),
                           evidence=dict(flags_not_always_set=never, gps_glitch_pct=glitch),
                           source="XKF4.SS (EKF_STATUS_FLAGS)"))
    sec.table("innovations", ["series", "mean", "p95", "max", "n>1.0"], rows)
    sec.note("XKF4 fields are innovation test ratios: ArduPilot rejects the measurement at 1.0.\n"
             "Samples over 1.0 are the number of rejected updates, and are worth naming explicitly -\n"
             "a rising count across consecutive flights is the signal, not the mean. The EKF failsafe\n"
             "fires when two of SM/SP/SV exceed FS_EKF_THRESH (default 0.8) for 1 s.")
    ev = [(t, n) for t, i, n in events(log) if i in (60, 62)]
    if ev:
        sec.note("EKF reset events: " + ", ".join(f"{n} at t={t:.1f}s" for t, n in ev))
        sec.add(Result("EKF resets", WARN, f"{len(ev)} EKF reset event(s) in the log",
                       evidence=dict(events=[(float(t), n) for t, n in ev]), source="EV 60/62"))
    return sec


# ------------------------------------------------------------------- estimates

def check_estimates(log, w):
    """Estimate divergence: ATT vs AHR2/XKF1 attitude, baro vs EKF altitude (dronekit-la)."""
    sec = Section("Estimate divergence", key="estimates")
    att = w.clip(log.df("ATT"))
    any_result = False
    if att is not None and not att.empty and {"Roll", "Pitch", "t"} <= set(att.columns):
        for name, src in (("AHR2", w.clip(log.df("AHR2"))),
                          ("XKF1", w.clip(log.instances("XKF1").get(0)))):
            if src is None or src.empty or not {"Roll", "Pitch", "t"} <= set(src.columns):
                continue
            r = np.interp(att["t"].values, src["t"].values, src["Roll"].values)
            pch = np.interp(att["t"].values, src["t"].values, src["Pitch"].values)
            dr = np.abs(((att["Roll"].values - r) + 180) % 360 - 180)
            dp = np.abs(((att["Pitch"].values - pch) + 180) % 360 - 180)
            worst = float(max(np.nanpercentile(dr, 99), np.nanpercentile(dp, 99)))
            sec.add(_grade(worst, "att_div_deg", name=f"ATT vs {name} attitude",
                           summary_fmt="p99 |difference| {v} deg (warn {w}, fail {f})",
                           roll_max=float(np.nanmax(dr)), pitch_max=float(np.nanmax(dp))))
            any_result = True
    baro = w.clip(log.instances("BARO").get(0)) if log.has("BARO") else None
    pos = w.clip(log.df("POS"))
    if baro is not None and not baro.empty and pos is not None and not pos.empty \
            and "Alt" in baro.columns and "RelHomeAlt" in pos.columns:
        b = baro["Alt"].values - baro["Alt"].values[0]
        e = np.interp(baro["t"].values, pos["t"].values, pos["RelHomeAlt"].values)
        e = e - e[0]
        diff = np.abs(b - e)
        sec.add(_grade(float(np.nanpercentile(diff, 99)), "alt_div_m", name="baro vs EKF altitude",
                       summary_fmt="p99 |difference| {v} m (warn {w}, fail {f}); both referenced to the "
                                   "window start", max_m=float(np.nanmax(diff))))
        any_result = True
    if not any_result:
        sec.add(Result("estimates", SKIP, "no second attitude/altitude source (AHR2, XKF1, BARO+POS) to compare"))
        return sec
    sec.note("Two estimators of the same quantity disagreeing is the classic signature of a sensor\n"
             "problem (vibration aliasing, a compass pulling yaw, a baro draught). Thresholds are\n"
             "dronekit-la's; the p99 is used instead of the max to ignore single-sample spikes.")
    return sec


# --------------------------------------------------------------------- compass

def check_compass(log, w):
    mag = log.instances("MAG")
    sec = Section("Compass", key="compass")
    if not mag:
        sec.add(Result("compass", SKIP, "no MAG messages"))
        return sec
    ctun = w.clip(log.df("CTUN"))
    rows = []
    for i, g in sorted(mag.items()):
        gg = w.clip(g)
        if gg is None or gg.empty:
            continue
        b = np.sqrt(gg["MagX"].values ** 2 + gg["MagY"].values ** 2 + gg["MagZ"].values ** 2)
        s = describe(b)
        health = float(gg["Health"].mean() * 100) if "Health" in gg.columns else np.nan
        # LogAnalyzer uses (max-min)/min; percentiles instead, because a single sample near
        # touchdown or a landing-gear magnet will fail an otherwise healthy compass.
        p01, p99 = np.percentile(b, 1), np.percentile(b, 99)
        var = (p99 - p01) / p01 if p01 > 0 else np.nan
        rows.append([f"MAG{i}", s["mean"], s["sd"], s["min"], s["max"], health])
        sec.add(_grade(s["mean"], "compass_field_hi", name=f"MAG{i} field magnitude",
                       summary_fmt="mean {v} mGauss (warn >{w}, fail >{f}); expected band 120-550"))
        if s["mean"] < T["compass_field_lo"]["warn"]:
            sec.add(_grade(s["mean"], "compass_field_lo", higher_is_worse=False,
                           name=f"MAG{i} field magnitude (low side)",
                           summary_fmt="mean {v} mGauss (warn <{w}, fail <{f})"))
        if np.isfinite(var):
            sec.add(_grade(var, "compass_field_var", name=f"MAG{i} field variation",
                           summary_fmt="(p99-p01)/p01 = {v} (warn {w}, fail {f})",
                           raw_maxmin_over_min=(s["max"] - s["min"]) / s["min"] if s["min"] > 0 else None))
        if np.isfinite(health):
            sec.add(Result(f"MAG{i} health", PASS if health > 99.5 else WARN,
                           f"MAG.Health true for {health:.2f}% of samples",
                           evidence=dict(health_pct=health), source="ArduPilot"))
        if ctun is not None and not ctun.empty and "ThO" in ctun.columns:
            thr = np.interp(gg["t"].values, ctun["t"].values, ctun["ThO"].values)
            c = corr(thr, b)
            sec.add(_grade(abs(c), "compass_mot_corr", name=f"MAG{i} motor interference",
                           summary_fmt="corr(throttle, |B|) = %s (warn {w}, fail {f})" % fmt(c),
                           signed=c))
    sec.table("mag", ["mag", "|B| mean", "sd", "min", "max", "health %"], rows)
    ofs = {k: v for k, v in log.params().items() if k.startswith("COMPASS_OFS")}
    if ofs:
        sec.note("Configured offsets: " + ", ".join(f"{k}={fmt(v)}" for k, v in sorted(ofs.items())))
        for label, keys in (("COMPASS_OFS", ("COMPASS_OFS_X", "COMPASS_OFS_Y", "COMPASS_OFS_Z")),
                            ("COMPASS_OFS2", ("COMPASS_OFS2_X", "COMPASS_OFS2_Y", "COMPASS_OFS2_Z")),
                            ("COMPASS_OFS3", ("COMPASS_OFS3_X", "COMPASS_OFS3_Y", "COMPASS_OFS3_Z"))):
            if all(k in ofs for k in keys):
                mag_ofs = float(np.sqrt(sum(ofs[k] ** 2 for k in keys)))
                if mag_ofs == 0.0:
                    continue         # an unused compass slot
                sec.add(_grade(mag_ofs, "compass_offsets", name=f"{label} magnitude",
                               summary_fmt="|offsets| = {v} (warn {w}, fail {f})"))
    prearm = [f"t={t:.1f} {m}" for t, m in log.messages_text() if "mag field" in str(m).lower()]
    if prearm:
        sec.note("Mag-field prearm messages in this log:\n  " + "\n  ".join(prearm))
    sec.note("Field magnitude is in mGauss. A better check than the fixed 120-550 band is the\n"
             "expected field at this lat/lon from the WMM table - see pymavlink's mavextra\n"
             "`expected_earth_field()`, available after running tools/bootstrap_pymavlink.py.")
    return sec


# ----------------------------------------------------------------------- power

def check_power(log, w):
    sec = Section("Power", key="power")
    bat = log.instances("BAT")
    ctun = w.clip(log.df("CTUN"))
    for i, g in sorted(bat.items()):
        gg = w.clip(g)
        if gg is None or gg.empty:
            continue
        v = describe(gg["Volt"]) if "Volt" in gg.columns else {}
        c = describe(gg["Curr"]) if "Curr" in gg.columns else {}
        sec.table(f"bat{i}", ["series", "min", "mean", "max"],
                  [[f"BAT{i} voltage (V)", v.get("min"), v.get("mean"), v.get("max")],
                   [f"BAT{i} current (A)", c.get("min"), c.get("mean"), c.get("max")]])
        parts = []
        if "CurrTot" in gg.columns and len(gg):
            parts.append(f"Consumed: {gg['CurrTot'].iloc[-1] - gg['CurrTot'].iloc[0]:.0f} mAh")
        if "Res" in gg.columns and np.isfinite(gg["Res"]).any():
            parts.append(f"ArduPilot internal resistance estimate BAT.Res: {np.nanmean(gg['Res']) * 1000:.1f} mOhm")
        if ctun is not None and not ctun.empty and "ThO" in ctun.columns and "Curr" in gg.columns:
            thr = np.interp(gg["t"].values, ctun["t"].values, ctun["ThO"].values)
            cc = corr(thr, gg["Curr"].values)
            if np.isfinite(cc):
                status = PASS if cc > 0.8 else (WARN if cc > 0.3 else FAIL)
                sec.add(Result(f"BAT{i} current sensing", status,
                               f"corr(throttle, current) = {cc:.3f}" +
                               ("" if cc > 0.8 else
                                " - a flat, throttle-independent current reading is a wiring or pin "
                                "fault, not a calibration error; a wrong BATT_AMP_PERVLT would change "
                                "the magnitude, not erase the correlation"),
                               evidence=dict(corr=cc), source="throttle-correlation method"))
                parts.append(f"corr(throttle, current) = {cc:.3f}")
            else:
                sec.add(Result(f"BAT{i} current sensing", SKIP,
                               "current is constant or NaN, so no throttle correlation can be computed",
                               source="throttle-correlation method"))
        if "Volt" in gg.columns and "Curr" in gg.columns and len(gg) > 10:
            vv, ii = gg["Volt"].values, gg["Curr"].values
            ok = np.isfinite(vv) & np.isfinite(ii)
            if ok.sum() > 10 and np.nanstd(ii[ok]) > 0.5:
                slope = float(np.polyfit(ii[ok], vv[ok], 1)[0])
                parts.append(f"V/I slope {slope * 1000:.1f} mOhm (whole-pack sag estimate)")
        if parts:
            sec.note(f"**BAT{i}**: " + "; ".join(parts))
    powr = w.clip(log.df("POWR"))
    if powr is not None and not powr.empty and "Vcc" in powr.columns and np.isfinite(powr["Vcc"]).any():
        s = describe(powr["Vcc"])
        sec.add(_grade(s["min"], "vcc_min", higher_is_worse=False, name="board Vcc",
                       summary_fmt="min {v} V (warn <{w}, fail <{f})"))
        sec.add(_grade(s["max"] - s["min"], "vcc_spread", name="board Vcc spread",
                       summary_fmt="{v} V spread (warn {w}, fail {f})"))
    elif powr is not None and not powr.empty:
        sec.add(Result("board Vcc", SKIP, "POWR.Vcc is NaN - this board has no board-voltage sensing"))
    mcu = w.clip(log.df("MCU"))
    if mcu is not None and not mcu.empty and "MTemp" in mcu.columns and np.isfinite(mcu["MTemp"]).any():
        sec.note(f"MCU temperature {np.nanmin(mcu['MTemp']):.0f}-{np.nanmax(mcu['MTemp']):.0f} C"
                 + (f", MCU rail {np.nanmin(mcu['MVmin']):.2f}-{np.nanmax(mcu['MVmax']):.2f} V"
                    if {"MVmin", "MVmax"} <= set(mcu.columns) else ""))
    if not sec.results and not sec.parts:
        sec.add(Result("power", SKIP, "no BAT or POWR messages"))
        return sec
    sec.note("Absolute current scale is only trustworthy after a charger cross-check:\n"
             "fly a pack, note logged CurrTot mAh, recharge and read the mAh put back, then\n"
             "BATT_AMP_PERVLT_new = BATT_AMP_PERVLT * (logged / charger).")
    return sec


# ------------------------------------------------------------------------- GPS

def check_gps(log, w):
    gps = log.instances("GPS")
    sec = Section("GPS", key="gps")
    if not gps:
        sec.add(Result("gps", SKIP, "no GPS messages"))
        return sec
    rows = []
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
            sec.add(_grade(sats["min"], "gps_sats", higher_is_worse=False, name=f"GPS{i} satellites",
                           summary_fmt="min {v} (warn <{w}, fail <{f})", mean=sats.get("mean")))
        if np.isfinite(hdop.get("max", np.nan)):
            sec.add(_grade(hdop["max"], "gps_hdop", name=f"GPS{i} HDOP",
                           summary_fmt="max {v} (warn {w}, fail {f})", mean=hdop.get("mean")))
        if np.isfinite(nofix):
            sec.add(_grade(nofix, "gps_nofix_pct", name=f"GPS{i} fix availability",
                           summary_fmt="{v}% of the airborne window without a 3D fix (warn {w}, fail {f})"))
        # Position glitch: horizontal jump between consecutive fixes, LogAnalyzer-style
        if {"Lat", "Lng", "t"} <= set(gg.columns) and len(gg) > 3:
            lat, lng = gg["Lat"].values, gg["Lng"].values
            fix = status >= 3 if status.size else np.ones(len(gg), dtype=bool)
            if fix.sum() > 3:
                la, ln, tt = lat[fix], lng[fix], gg["t"].values[fix]
                dx = np.diff(ln) * 111320.0 * np.cos(np.radians(la[:-1]))
                dy = np.diff(la) * 110540.0
                dt = np.diff(tt)
                ok = dt > 0
                speed = np.sqrt(dx[ok] ** 2 + dy[ok] ** 2) / dt[ok]
                if speed.size:
                    sec.add(_grade(float(speed.max()), "gps_glitch_speed", name=f"GPS{i} position jumps",
                                   summary_fmt="max implied speed between fixes {v} m/s (warn {w}, fail {f})",
                                   n_over_warn=int((speed > T["gps_glitch_speed"]["warn"]).sum())))
    sec.table("gps", ["gps", "sats mean", "sats min", "HDOP mean", "HDOP max", "% no 3D fix"], rows)
    err = log.df("ERR")
    if not err.empty and {"Subsys", "ECode"} <= set(err.columns):
        gl = err[(err["Subsys"] == 11) & (err["ECode"] == 2)]
        if len(gl):
            sec.add(Result("GPS glitch (ERR)", FAIL, f"{len(gl)} GPS_GLITCH error record(s)",
                           evidence=dict(n=len(gl), t=[float(x) for x in gl["t"].values[:10]]),
                           source="LogAnalyzer TestGPSGlitch (ERR Subsys 11 ECode 2)"))
    if log.has("UBX2"):
        ub = log.instances("UBX2")
        sec.note("UBX2 present for instance(s) " + ", ".join(str(k) for k in sorted(ub)) +
                 " - UBX2 is emitted only by the u-blox driver, so this identifies which physical\n"
                 "unit each GPS instance is. Never infer that from the instance index: the mapping\n"
                 "depends on SERIAL port order and can change between param snapshots.")
    if len(gps) > 1:
        sec.note("With two GPS units, the differential is the diagnostic. Both degraded similarly\n"
                 "implicates a shared external emitter; one much worse than the other implicates that\n"
                 "unit (self-jam, weak front end) rather than a uniform RF problem.")
    return sec


# ------------------------------------------------------------------------- CPU

def check_cpu(log, w):
    pm = w.clip(log.df("PM"))
    sec = Section("CPU and memory", key="cpu")
    if pm is None or pm.empty:
        sec.add(Result("cpu", SKIP, "no PM messages"))
        return sec
    if "Load" in pm.columns:
        load = pm["Load"].values / 10.0          # PM.Load is percent x10
        s = describe(load)
        sec.add(_grade(s["max"], "cpu_load", name="CPU load",
                       summary_fmt="peak {v}%% (warn {w}, fail {f}); mean %.1f%%" % s["mean"],
                       mean=s["mean"]))
    if {"NLon", "NL"} <= set(pm.columns):
        with np.errstate(divide="ignore", invalid="ignore"):
            slow = np.where(pm["NL"].values > 0, pm["NLon"].values / pm["NL"].values * 100, 0.0)
        sec.add(_grade(float(np.nanmax(slow)), "cpu_slow_pct", name="slow loops",
                       summary_fmt="worst window {v}% of loops ran long (warn {w}, fail {f})",
                       total_long=int(pm["NLon"].sum())))
    if "Mem" in pm.columns:
        sec.add(_grade(float(pm["Mem"].min()), "free_mem_bytes", higher_is_worse=False, name="free memory",
                       summary_fmt="min {v} bytes (warn <{w}, fail <{f})"))
    if "ErC" in pm.columns:
        # PM.InE is the internal-error bitmask, PM.ErC the count, PM.ErrL the source line.
        erc = int(pm["ErC"].max())
        ine = int(pm["InE"].max()) if "InE" in pm.columns else None
        sec.add(Result("internal errors", PASS if erc == 0 else FAIL,
                       f"PM.ErC (internal error count) max {erc}"
                       + (f", mask PM.InE=0x{ine:x}, last line PM.ErrL={int(pm['ErrL'].max())}"
                          if erc and ine is not None and "ErrL" in pm.columns else ""),
                       evidence=dict(count=erc, mask=ine), source="AP_InternalError via PM"))
    rows = [[c, describe(pm[c])["mean"], describe(pm[c])["max"]]
            for c in ("Load", "NLon", "MaxT", "Mem", "SPIC", "I2CC", "I2CI", "ErC") if c in pm.columns]
    sec.table("pm", ["field", "mean", "max"], rows)
    sec.note("PM.Load is percent x10 in the raw log (203 = 20.3%); the check above divides it.\n"
             "Free memory is PM.Mem in bytes. SPIC/I2CC/I2CI are transaction and interrupt counters,\n"
             "not error counts. LogAnalyzer deliberately ignores PM.MaxT - it throws false positives\n"
             "around arm and disarm.")
    return sec


# ---------------------------------------------------------------------- events

_ERR_SUBSYS = {1: "MAIN", 2: "RADIO", 3: "COMPASS", 4: "OPTFLOW", 5: "FAILSAFE_RADIO",
               6: "FAILSAFE_BATT", 7: "FAILSAFE_GPS", 8: "FAILSAFE_GCS", 9: "FAILSAFE_FENCE",
               10: "FLIGHT_MODE", 11: "GPS", 12: "CRASH_CHECK", 13: "FLIP", 14: "AUTOTUNE",
               15: "PARACHUTES", 16: "EKFCHECK", 17: "FAILSAFE_EKFINAV", 18: "BARO",
               19: "CPU", 20: "FAILSAFE_ADSB", 21: "TERRAIN", 22: "NAVIGATION",
               23: "FAILSAFE_TERRAIN", 24: "EKF_PRIMARY", 25: "THRUST_LOSS_CHECK",
               26: "FAILSAFE_SENSORS", 27: "FAILSAFE_LEAK", 28: "PILOT_INPUT",
               29: "FAILSAFE_VIBE", 30: "INTERNAL_ERROR", 31: "FAILSAFE_DEADRECKON"}


def check_events(log, w):
    sec = Section("Events and messages", key="events")
    ev = events(log)
    if ev:
        sec.table("ev", ["t (s)", "id", "event"], [[round(t, 1), i, n] for t, i, n in ev],
                  align=["r", "r", "l"])
    err = log.df("ERR")
    if not err.empty:
        rows = [[round(r["t"], 1), int(r["Subsys"]), _ERR_SUBSYS.get(int(r["Subsys"]), "?"), int(r["ECode"])]
                for _, r in err.iterrows()]
        sec.table("err", ["t (s)", "Subsys", "subsystem", "ECode"], rows, align=["r", "r", "l", "r"])
        # ECode 0 is "error resolved"; fence-only is a WARN in LogAnalyzer, the rest FAIL
        real = err[err["ECode"] != 0]
        fence_only = bool(len(real)) and bool((real["Subsys"] == 9).all())
        status = PASS if not len(real) else (WARN if fence_only else FAIL)
        sec.add(Result("subsystem errors", status,
                       f"{len(err)} ERR records, {len(real)} with a non-zero code"
                       + ("" if not len(real) else ": " + ", ".join(
                           f"{_ERR_SUBSYS.get(int(s), s)}/{int(c)}" for s, c in
                           zip(real["Subsys"].values[:8], real["ECode"].values[:8]))),
                       evidence=dict(n=len(err), n_nonzero=len(real)),
                       source="ardupilot LogAnalyzer TestEvents"))
        crash = real[real["Subsys"] == 12]
        if len(crash):
            sec.add(Result("crash check", FAIL, f"CRASH_CHECK triggered {len(crash)} time(s)",
                           evidence=dict(t=[float(x) for x in crash["t"].values]),
                           source="ERR Subsys 12"))
        thrust = real[real["Subsys"] == 25]
        if len(thrust):
            sec.add(Result("thrust loss", FAIL, f"THRUST_LOSS_CHECK triggered {len(thrust)} time(s)",
                           evidence=dict(t=[float(x) for x in thrust["t"].values]),
                           source="ERR Subsys 25"))
    msgs = log.messages_text()
    if msgs:
        sec.table("msg", ["t (s)", "message"], [[round(float(t), 1), m] for t, m in msgs],
                  align=["r", "l"])
        prearm = [m for _, m in msgs if str(m).startswith(("PreArm", "Arm:"))]
        if prearm:
            sec.add(Result("prearm failures", WARN, f"{len(prearm)} PreArm/Arm message(s): "
                           + "; ".join(sorted(set(str(m) for m in prearm))[:5]),
                           evidence=dict(messages=sorted(set(str(m) for m in prearm))),
                           source="MSG"))
    if not sec.parts:
        sec.add(Result("events", SKIP, "no EV/ERR/MSG records"))
    return sec


# ---------------------------------------------------------------------- flight

def check_flight(log, w):
    """Did it arm, did it fly, what did autotune do, did it exceed ANGLE_MAX."""
    sec = Section("Flight", key="flight")
    fl = flights(log, method="auto")
    sec.data["flights"] = [dict(index=f.index, t0=f.t0, t1=f.t1, duration_s=f.duration) for f in fl]
    if fl:
        covered = [f.index for f in fl if w.t0 <= f.t0 + 0.5 and w.t1 >= f.t1 - 0.5]
        listing = ", ".join(f"{f.t0:.1f}-{f.t1:.1f} s" for f in fl)
        source = f"flight segmentation via {fl[0].method.split(', flight')[0]}"
        if len(fl) == 1:
            sec.add(Result("flights in log", PASS, f"1 flight ({listing})",
                           evidence=dict(n_flights=1, flights=sec.data["flights"]), source=source))
        elif w.scope == "all" or len(covered) == len(fl):
            sec.add(Result("flights in log", PASS,
                           f"{len(fl)} flights ({listing}); all of them analysed",
                           evidence=dict(n_flights=len(fl), analysed=covered,
                                         flights=sec.data["flights"]), source=source))
        else:
            this = w.index or (covered[0] if covered else None)
            sec.add(Result("flights in log", WARN,
                           f"{len(fl)} flights ({listing}); this report covers "
                           + (f"flight {this} of {len(fl)}" if this else "only part of the log")
                           + f" ({w.t0:.1f}-{w.t1:.1f} s). Use --flight N for another, "
                             "or --flight all for every one.",
                           evidence=dict(n_flights=len(fl), analysed=covered, flight_index=this,
                                         flights=sec.data["flights"]), source=source))
    ev = events(log)
    arm = [t for t, i, _ in ev if i == 10]
    dis = [t for t, i, _ in ev if i == 11]
    # With LOG_DISARMED=0 the log opens *at* arming, so the ARMED event itself is often
    # written before the file exists. ARM.ArmState and the throttle are the other witnesses.
    armd = log.df("ARM")
    arm_msg = bool(not armd.empty and "ArmState" in armd.columns and (armd["ArmState"].values > 0).any())
    tho = log.field("CTUN", "ThO", "ThrOut")
    peak = None
    if tho is not None and len(tho):
        peak = float(np.nanmax(tho))
        if peak > 1.5:          # legacy 0-1000 scale
            peak /= 1000.0
    flew = peak is not None and peak > 0.2
    if arm or arm_msg:
        st, why = PASS, f"{len(arm)} ARMED event(s), {len(dis)} DISARMED event(s)" + (
            "" if arm else "; armed state from ARM.ArmState (log opened at arming, so EV 10 was never written)")
    elif flew:
        st, why = PASS, (f"no ARMED event or ARM record, but throttle reached {peak:.2f} - the log opened "
                         "after arming (LOG_DISARMED=0)")
    else:
        st, why = FAIL, f"{len(arm)} arm event(s), no ARM record, throttle never rose - the log never armed"
    sec.add(Result("ever armed", st, why,
                   evidence=dict(arm_times=arm, disarm_times=dis, arm_message=arm_msg),
                   source="dronekit-la Ever Armed; EV 10/11, ARM.ArmState"))
    if peak is not None:
        sec.add(Result("ever flew", PASS if flew else FAIL,
                       f"peak CTUN.ThO {peak:.2f}" + ("" if flew else
                                                     " - throttle never above 20%, an empty log"),
                       evidence=dict(peak_throttle=peak), source="LogAnalyzer isLogEmpty"))
    fl = [(t, i) for t, i, _ in ev if 30 <= i <= 37]
    if fl:
        names = {30: "INIT", 31: "OFF", 32: "RESTART", 33: "SUCCESS", 34: "FAILED", 35: "REACHED_LIMIT",
                 36: "PILOT_TESTING", 37: "SAVED_GAINS"}
        outcome = "SUCCESS" if any(i == 33 for _, i in fl) else ("FAILED" if any(i == 34 for _, i in fl) else "INCOMPLETE")
        saved = any(i == 37 for _, i in fl)
        sec.add(Result("autotune", PASS if outcome == "SUCCESS" else WARN,
                       f"autotune {outcome}" + (", gains saved" if saved else ", gains NOT saved")
                       + ": " + ", ".join(f"{names[i]}@{t:.0f}s" for t, i in fl),
                       evidence=dict(outcome=outcome, saved=saved, events=[(float(t), names[i]) for t, i in fl]),
                       source="LogAnalyzer TestAutotune (EV 30-37)"))
    att = w.clip(log.df("ATT"))
    p = _Params(log)
    if att is not None and not att.empty and {"Roll", "Pitch"} <= set(att.columns):
        angle_max = p.get("ANGLE_MAX", 3000.0) / 100.0
        lean = np.sqrt(att["Roll"].values ** 2 + att["Pitch"].values ** 2)
        over = float(np.nanmax(lean) - angle_max)
        sec.add(_grade(max(over, 0.0), "lean_over_max_deg", name="lean vs ANGLE_MAX",
                       summary_fmt="max lean %.1f deg vs ANGLE_MAX %.0f deg, over by {v} deg (warn >{w}, fail >{f})"
                                   % (float(np.nanmax(lean)), angle_max),
                       max_lean_deg=float(np.nanmax(lean)), angle_max_deg=angle_max,
                       pct_over=float((lean > angle_max).mean() * 100)))
    modes = mode_timeline(log)
    if modes:
        reasons = sorted({r for _, _, _, r in modes if r not in (-1, 0, 1, 2)})
        auto_changes = [(t, name, r) for t, _, name, r in modes if r not in (-1, 0, 1, 2, 26, 31)]
        if auto_changes:
            from .flight import MODE_REASONS
            sec.add(Result("uncommanded mode changes", WARN,
                           f"{len(auto_changes)} mode change(s) not from the pilot or GCS: "
                           + "; ".join(f"{name} at {t:.0f}s ({MODE_REASONS.get(r, r)})" for t, name, r in auto_changes[:6]),
                           evidence=dict(changes=[(float(t), n, int(r)) for t, n, r in auto_changes]),
                           source="MODE.Rsn (ModeReason)"))
    p.note(sec)
    return sec


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
    sec = Section("Disturbance rejection", key="gust")
    if att is None or att.empty or not {"Roll", "DesRoll"} <= set(att.columns):
        sec.add(Result("gust", SKIP, "no ATT desired/actual pair"))
        return sec
    n_ev = 0
    detail = []
    for ax, a, d in (("roll", "Roll", "DesRoll"), ("pitch", "Pitch", "DesPitch")):
        err = np.abs(att[a].values - att[d].values)
        quiet = np.abs(att[d].values) < cmd_deg
        hit = (err > dev_deg) & quiet
        runs = int(np.sum(np.diff(hit.astype(int)) == 1)) + int(hit[0] if len(hit) else 0)
        n_ev += runs
        detail.append([ax, runs, float(runs / w.duration) if w.duration else np.nan,
                       float(err[quiet].max()) if quiet.any() else np.nan])
    rate = n_ev / w.duration if w.duration else np.nan
    sec.add(_grade(rate, "gust_event_rate", name="unrequested attitude excursions",
                   summary_fmt="{v} events/s (%d in %.0f s; warn {w}, fail {f})" % (n_ev, w.duration)))
    sec.table("gust", ["axis", "events", "events/s", "max |err| while quiet (deg)"], detail)
    sec.note(f"Event = |actual - desired| > {dev_deg} deg while |desired| < {cmd_deg} deg.\n"
             "Cross-check before blaming the tune: if motor headroom and clipping are both fine,\n"
             "this is a controller-gain problem rather than a power or noise problem. If GPS\n"
             "dropouts are suspected, correlate the event times against the outage windows before\n"
             "blaming either.")
    return sec


# ------------------------------------------------------------------------- IMU

def check_imu(log, w):
    """IMU consistency: per-instance gyro/accel health and the dual-IMU mismatch."""
    imu = log.instances("IMU")
    sec = Section("IMU consistency", key="imu")
    if not imu:
        sec.add(Result("imu", SKIP, "no IMU messages"))
        return sec
    rows, mags = [], {}
    for i, g in sorted(imu.items()):
        gg = w.clip(g)
        if gg is None or gg.empty:
            continue
        acc = [c for c in ("AccX", "AccY", "AccZ") if c in gg.columns]
        gyr = [c for c in ("GyrX", "GyrY", "GyrZ") if c in gg.columns]
        amag = np.sqrt(sum(gg[c].values ** 2 for c in acc)) if len(acc) == 3 else None
        if amag is not None:
            mags[i] = (gg["t"].values, amag)
        gbias = [float(np.nanmean(gg[c])) for c in gyr] if gyr else []
        eg = float(gg["EG"].iloc[-1] - gg["EG"].iloc[0]) if "EG" in gg.columns else np.nan
        ea = float(gg["EA"].iloc[-1] - gg["EA"].iloc[0]) if "EA" in gg.columns else np.nan
        temp = float(np.nanmean(gg["T"])) if "T" in gg.columns else np.nan
        gh = float(gg["GH"].mean() * 100) if "GH" in gg.columns else np.nan
        ah = float(gg["AH"].mean() * 100) if "AH" in gg.columns else np.nan
        rows.append([f"IMU{i}", float(np.nanmean(amag)) if amag is not None else None,
                     *(gbias + [None] * (3 - len(gbias))), eg, ea, temp, gh, ah])
        if (np.isfinite(eg) and eg > 0) or (np.isfinite(ea) and ea > 0):
            sec.add(Result(f"IMU{i} sensor errors", WARN,
                           f"gyro error count +{eg:.0f}, accel error count +{ea:.0f} over the window",
                           evidence=dict(gyro_errors=eg, accel_errors=ea), source="IMU.EG/EA counters"))
        if (np.isfinite(gh) and gh < 100) or (np.isfinite(ah) and ah < 100):
            sec.add(Result(f"IMU{i} health flags", WARN,
                           f"gyro healthy {gh:.2f}%, accel healthy {ah:.2f}% of samples",
                           evidence=dict(gyro_healthy_pct=gh, accel_healthy_pct=ah), source="IMU.GH/AH"))
        if gbias:
            worst = max(abs(b) for b in gbias)
            sec.add(_grade(float(np.degrees(worst)), "gyro_bias_dps", name=f"IMU{i} mean gyro rate",
                           summary_fmt="largest axis mean {v} deg/s over the window (warn {w}, fail {f})",
                           per_axis_dps=[float(np.degrees(b)) for b in gbias]))
    sec.table("imu", ["imu", "|acc| mean", "GyrX mean", "GyrY mean", "GyrZ mean", "gyro err",
                      "accel err", "temp C", "gyro ok %", "accel ok %"], rows)
    if len(mags) >= 2:
        keys = sorted(mags)
        t0, a0 = mags[keys[0]]
        # LogAnalyzer TestIMUMatch: low-pass both, compare magnitudes, warn 0.75 fail 1.5 m/s^2
        for k in keys[1:]:
            t1, a1 = mags[k]
            b = np.interp(t0, t1, a1)
            low0, _ = band_split(t0, a0, 0.2)
            low1, _ = band_split(t0, b, 0.2)
            diff = float(np.nanmax(np.abs(low0 - low1)))
            sec.add(_grade(diff, "imu_match_mss", name=f"IMU{keys[0]} vs IMU{k} accel match",
                           summary_fmt="max low-passed |acc| difference {v} m/s^2 (warn {w}, fail {f})"))
    sec.note("IMU is the filtered signal the EKF consumes. Gyro means over a hover should be near\n"
             "zero; a standing offset is uncorrected bias or a genuinely rotating window. EG/EA are\n"
             "cumulative sensor error counters (deltas shown); GH/AH are the health flags.")
    return sec


# -------------------------------------------------------------------- brownout

def check_brownout(log, w):
    """Did the log end in flight? (LogAnalyzer TestBrownout, plus the parser's own view.)"""
    sec = Section("Log end / brownout", key="brownout")
    ev = events(log)
    arm = [t for t, i, _ in ev if i == 10]
    dis = [t for t, i, _ in ev if i == 11]
    lo, hi = log.duration()
    if hi is None:
        sec.add(Result("brownout", SKIP, "no timestamps"))
        return sec
    still_armed = bool(arm) and (not dis or max(dis) < max(arm))
    alt = log.field("CTUN", "BAlt", "BarAlt")
    last_alt = float(alt[-1]) if alt is not None and len(alt) else None
    truncated = log.diagnostics.has("TRUNCATED_TAIL")
    if still_armed and last_alt is not None and last_alt > T["brownout_alt_m"]["fail"]:
        status = FAIL
        why = f"still armed at log end with BAlt {last_alt:.1f} m - the log stopped in flight"
    elif still_armed:
        status = WARN
        why = "still armed at log end (no DISARM event)" + (
            f", BAlt {last_alt:.1f} m" if last_alt is not None else "")
    else:
        status = PASS
        why = "disarmed before the log ended"
    if truncated:
        why += "; file is physically truncated (see integrity)"
    sec.add(Result("log end", status, why,
                   evidence=dict(still_armed=still_armed, last_alt_m=last_alt, truncated=truncated,
                                 t_end=hi, last_arm=max(arm) if arm else None,
                                 last_disarm=max(dis) if dis else None),
                   source=T["brownout_alt_m"]["source"]))
    bat = log.instances("BAT")
    if bat:
        g = bat[min(bat)]
        if "Volt" in g.columns and len(g) > 10:
            v = g["Volt"].values
            sec.add(Result("battery at log end", PASS if v[-1] > 0.9 * np.nanmax(v) else WARN,
                           f"final voltage {v[-1]:.2f} V vs max {np.nanmax(v):.2f} V",
                           evidence=dict(final_v=float(v[-1]), max_v=float(np.nanmax(v))), source="BAT.Volt"))
    return sec


# ---------------------------------------------------------------------- params

def check_params(log, w):
    """Parameter sanity: NaN values, in-flight changes, and copter-specific rules."""
    sec = Section("Parameters", key="paramcheck")
    d = log.df("PARM")
    if d.empty:
        sec.add(Result("params", SKIP, "no PARM records"))
        return sec
    vals = d["Value"].values.astype(float)
    nan_names = sorted(set(d["Name"].values[~np.isfinite(vals)]))
    sec.add(Result("NaN parameters", FAIL if nan_names else PASS,
                   f"{len(nan_names)} parameter(s) with a NaN/Inf value"
                   + (": " + ", ".join(nan_names[:10]) if nan_names else ""),
                   evidence=dict(names=nan_names), source="LogAnalyzer TestParams"))
    changes = log.param_changes()
    if changes:
        sec.table("changes", ["t (s)", "param", "old", "new"],
                  [[round(t, 1), n, o, v] for t, n, o, v in changes], align=["r", "l", "r", "r"])
        inflight = [c for c in changes if w.t0 <= c[0] <= w.t1]
        sec.add(Result("parameter changes", WARN if inflight else PASS,
                       f"{len(changes)} parameter(s) rewritten after boot, {len(inflight)} inside the "
                       "airborne window" + (" - the tune changed mid-flight (autotune or GCS)" if inflight else ""),
                       evidence=dict(n=len(changes), n_inflight=len(inflight),
                                     names=sorted({c[1] for c in changes})[:20]),
                       source="PARM re-emission"))
    p = log.params()
    defaults = log.param_defaults()
    if defaults:
        changed = {k: (p[k], defaults[k]) for k in p if k in defaults and np.isfinite(defaults[k])
                   and not np.isclose(p[k], defaults[k], rtol=1e-6, atol=1e-9)}
        sec.note(f"{len(changed)} of {len(p)} parameters differ from their firmware defaults "
                 "(PARM.Default column). `alog params --non-default` lists them.")
        sec.data["non_default"] = changed
    rules = []
    if p.get("ARMING_CHECK", 1) == 0:
        rules.append(("ARMING_CHECK", "0 - all arming checks disabled", WARN))
    if p.get("FS_THR_ENABLE", 1) == 0:
        rules.append(("FS_THR_ENABLE", "0 - radio failsafe disabled", WARN))
    if p.get("BATT_FS_LOW_ACT", 1) == 0 and p.get("BATT_LOW_VOLT", 1) not in (0, None):
        rules.append(("BATT_FS_LOW_ACT", "0 - low-battery failsafe takes no action", WARN))
    if "MOT_THST_HOVER" in p and log.has("CTUN") and "ThH" in log.df("CTUN").columns:
        thh = float(np.nanmedian(w.clip(log.df("CTUN"))["ThH"]))
        drift = abs(thh - p["MOT_THST_HOVER"]) / max(p["MOT_THST_HOVER"], 1e-6) * 100
        sec.add(Result("MOT_THST_HOVER vs learned", PASS if drift < 10 else WARN,
                       f"param {p['MOT_THST_HOVER']:.4f} vs learned CTUN.ThH median {thh:.4f} "
                       f"({drift:.1f}% apart)" + (" - the snapshot is stale; anything pinned to it "
                                                  "(INS_HNTCH_REF) drifts too" if drift >= 10 else ""),
                       evidence=dict(param=p["MOT_THST_HOVER"], learned=thh, drift_pct=drift),
                       source="configured vs measured"))
    for name, why, st in rules:
        sec.add(Result(name, st, why, evidence=dict(value=p.get(name)), source="ArduPilot wiki"))
    return sec


# ------------------------------------------------------------------------- FFT

def check_fft(log, w):
    """Local FFT of the best available gyro signal, with peaks labelled in motor orders."""
    from .spectral import analyse, sources, SpectralError
    sec = Section("Spectral analysis (local FFT)", key="spectrum")
    srcs = sources(log)
    if not srcs:
        sec.add(Result("fft", SKIP, "no IMU/GYR/ACC/RATE/ISBD messages to transform"))
        return sec
    sec.table("sources", ["source", "kind", "rate Hz", "Nyquist Hz", "samples", "note"],
              [[s["name"], s["kind"], round(s["rate_hz"], 1), round(s["nyquist_hz"], 1), s["n"], s["note"]]
               for s in srcs], align=["l", "l", "r", "r", "r", "l"])
    try:
        r = analyse(log, window=w)
    except SpectralError as exc:
        sec.add(Result("fft", SKIP, f"cannot transform: {exc}"))
        return sec
    src = r["source"]
    f0 = r["esc_fundamental_hz"]
    rows = []
    for ax, pk in r["peaks"].items():
        for q in pk[:4]:
            rows.append([ax, round(q["freq_hz"], 1), round(q["db_above_floor"], 1),
                         round(q["order"], 2) if q["order"] else None])
    sec.table("peaks", ["axis", "peak Hz", "dB above floor", "order (x motor f0)"], rows)
    nyq = r["fs_hz"] / 2.0
    if f0 and nyq < f0 * 1.1:
        sec.add(Result("spectral reach", WARN,
                       f"{src['name']} Nyquist {nyq:.0f} Hz is below the motor fundamental {f0:.0f} Hz: "
                       "this spectrum cannot show motor noise at all",
                       evidence=dict(nyquist_hz=nyq, fundamental_hz=f0, source=src["name"]),
                       source="Nyquist"))
    elif f0:
        near = [q for pk in r["peaks"].values() for q in pk
                if q["order"] and (abs(q["order"] - 1) < 0.1 or abs(q["order"] - 2) < 0.1)]
        if near:
            worst = max(near, key=lambda q: q["db_above_floor"])
            sec.add(_grade(worst["db_above_floor"], "motor_peak_db", name="motor-order peak",
                           summary_fmt="strongest peak at a motor order is {v} dB above the floor "
                                       "(warn {w}, fail {f}); at %.1f Hz = order %.2f" % (worst["freq_hz"], worst["order"]),
                           freq_hz=worst["freq_hz"], order=worst["order"], source_name=src["name"]))
        else:
            sec.add(Result("motor-order peak", PASS,
                           f"no peak within 10% of order 1 or 2 of the motor fundamental ({f0:.0f} Hz) "
                           f"in {src['name']}", evidence=dict(fundamental_hz=f0), source="dflog spectral"))
    tm = r["timing"]
    sec.note(f"Source {src['name']} at {r['fs_hz']:.1f} Hz (Nyquist {nyq:.1f} Hz), band "
             f"{r['band_hz'][0]:.0f}-{r['band_hz'][1]:.0f} Hz, Welch PSD via scipy.signal. "
             + (f"{tm['batches']} contiguous batches of {tm['samples_per_batch']} samples, {tm['holes']} "
                "discarded for seqno holes." if "batches" in tm else
                f"{tm['n']} samples over {tm['span_s']:.1f} s, interval jitter {tm['jitter'] * 100:.1f}%.")
             + (f" Motor fundamental (ESC) {f0:.1f} Hz." if f0 else " No ESC telemetry: orders not available.")
             + " `alog fft --plot` draws it; `--source` picks another signal.")
    sec.data["spectrum"] = r
    return sec


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
    less energy above INS_GYRO_FILTER; the writer offsets post-filter instances
    by the IMU count, which is only self-describing if you know that count.
    """
    try:
        from scipy import fft as _fft
    except ImportError:  # pragma: no cover
        _fft = np.fft
    isbh, isbd = log.df("ISBH"), log.df("ISBD")
    sec = Section("Batch IMU spectra (notch proof)", key="batchfft")
    if isbh.empty or isbd.empty:
        sec.add(Result("batch", SKIP, "no ISBH/ISBD - set INS_LOG_BAT_MASK=1 and INS_LOG_BAT_OPT=4, fly, "
                       "then set the mask back to 0"))
        return sec
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
        sec.add(Result("batch", SKIP,
                       f"no usable batches inside the window ({holes} discarded for sequence holes)"))
        return sec

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
                f = _fft.rfft(row * win)
                p = np.square(np.abs(f))
                p[0] = 0.0
                p[-1] = 0.0
                acc = p if acc is None else acc + p
                cnt += 1
        if cnt:
            freqs = _fft.rfftfreq(n, 1.0 / fs)
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
                        pp = np.square(np.abs(_fft.rfft(row * win)))
                        pp[0] = 0.0
                        pp[-1] = 0.0
                        pp = 2.0 * pp / (fs * s2)
                        oacc += np.interp(order_axis, freqs / cf, pp, left=0.0, right=0.0)
                        ocnt += 1
                if ocnt:
                    orders[key] = (order_axis, oacc / ocnt, ocnt)

    sec.note(f"Batches used: {sum(v[2] for v in spectra.values())}; {holes} discarded for ISBD sequence holes.")
    gyro = {k: v for k, v in spectra.items() if k[0] == 1}
    p = _Params(log)
    gyro_filt = p.get("INS_GYRO_FILTER", 42.0)
    if len(gyro) >= 2:
        def hf_energy(v):
            f, ps, _ = v
            m = f > gyro_filt * 1.5
            return float(ps[m].sum())
        order = sorted(gyro.items(), key=lambda kv: -hf_energy(kv[1]))
        (pre_k, pre), (post_k, post) = order[0], order[-1]
        sec.note(f"Pre-filter identified as ISBH instance {pre_k[1]}, post-filter as {post_k[1]} "
                 f"(by energy above {gyro_filt * 1.5:.0f} Hz).")
        t, fund = esc_fundamental(log)
        f0 = float(np.nanmedian(fund[w.mask(t)])) if fund is not None and w.mask(t).any() else None
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
                sec.add(_grade(db, "notch_atten_db", higher_is_worse=True,
                               name="notch attenuation at the fundamental",
                               summary_fmt="{v} dB post/pre (warn >{w}, fail >{f})"))
        sec.table("attenuation", ["band", "pre", "post", "post/pre dB"], rows)
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
            sec.table("order_normalised", ["order-normalised", "deepest at order", "dB there"],
                      [["fundamental", o1, d1], ["2nd harmonic", o2, d2]])
            sec.note("Each batch was normalised by the notch centre the FC was tracking at that\n"
                     "instant, so order 1.0 is always the true fundamental. The attenuation minimum\n"
                     "landing within a percent of order 1.0 (and 2.0) is the evidence that the notch\n"
                     "is applied where the noise actually is, not merely configured to be.")
            for o, tgt, label in ((o1, 1.0, "fundamental"), (o2, 2.0, "2nd harmonic")):
                if np.isfinite(o):
                    sec.add(Result(f"notch placement at the {label}",
                                   PASS if abs(o - tgt) / tgt < 0.02 else WARN,
                                   f"deepest attenuation at order {o:.3f} (target {tgt:.1f}, "
                                   f"error {abs(o - tgt) / tgt * 100:.2f}%)",
                                   evidence=dict(order=o, target=tgt),
                                   source="order-normalisation method"))
        sec.note("Caveat: this post/pre ratio is the notch PLUS the INS_GYRO_FILTER low-pass, which\n"
                 "sits after it and attenuates everything above its corner. To isolate the notch alone,\n"
                 "fit a smooth baseline to the transfer function outside the notch bands (excluding\n"
                 "+/-20% around orders 1 and 2) and measure the dip below it.")
    else:
        sec.note("Only one gyro batch series present - set INS_LOG_BAT_OPT=4 to log pre AND post filter.")
        sec.add(Result("pre/post comparison", SKIP, "only one gyro batch series; no pre/post pair"))
    for (stype, inst), (f, ps, nb) in sorted(spectra.items()):
        kind = "gyro" if stype == 1 else "accel"
        sec.note(f"{kind} instance {inst}: {nb} batches, peak at {peak_hz(f, ps, 40, 500):.1f} Hz")
    p.note(sec)
    sec.data.update(spectra=spectra, orders=orders)
    return sec


ALL_CHECKS = [
    ("summary", check_summary),
    ("integrity", check_integrity),
    ("coverage", check_coverage),
    ("events", check_events),
    ("flight", check_flight),
    ("paramcheck", check_params),
    ("brownout", check_brownout),
    ("vibe", check_vibration),
    ("imu", check_imu),
    ("motors", check_motors),
    ("notch", check_notch),
    ("pid", check_pids),
    ("gust", check_gust_response),
    ("ekf", check_ekf),
    ("estimates", check_estimates),
    ("compass", check_compass),
    ("power", check_power),
    ("gps", check_gps),
    ("cpu", check_cpu),
    ("spectrum", check_fft),
    ("batchfft", check_batch_fft),
]


def run(log, names=None, window=None, method="auto", flight=None, **kw):
    """Run checks by name (default: all). Returns [Section].

    A check that raises is reported as a FAIL result naming the exception - a broken
    check must never kill the report, and must never look like a clean one either.

    `flight` (1-based) selects one flight on a log that holds several, so a library
    caller need not construct the window itself. Ignored when `window` is given.
    """
    w = window or airborne_window(log, method=method, flight=flight)
    out = []
    for name, fn in ALL_CHECKS:
        if names and name not in names:
            continue
        try:
            sec = fn(log, w, **({} if fn is not check_motors else
                                {k: v for k, v in kw.items() if k == "normalise"}))
        except Exception as exc:
            sec = Section(name, [Result(name, FAIL, f"check crashed: {type(exc).__name__}: {exc}",
                                        evidence=dict(exception=type(exc).__name__), source="dflog")],
                          key=name)
        if sec.key is None:
            sec.key = name
        out.append(sec)
    return out
