"""alog - ArduPilot dataflash log analysis, built for agents.

    alog info      flight.bin              # identity, integrity, coverage: run this first
    alog all       flight.bin              # the standard battery
    alog all       flight.bin --json       # same, as one JSON document
    alog motors    flight.bin --window rpm # one check
    alog fft       flight.bin --plot fft.png
    alog compare   before.bin after.bin    # like-for-like, identical code both sides
    alog types     flight.bin              # what messages this log actually contains
    alog fields    flight.bin ESC          # a message's fields, units and rate
    alog dump      flight.bin RATE --fields t,RDes,R --every 10
    alog params    flight.bin --diff snapshot.param
    alog files     flight.bin --out ./embedded
    alog schema                            # the JSON output contract

Output is markdown by default, JSON with --json. Exit codes: 0 pass, 1 warn, 2 fail,
3 the input could not be analysed (missing file, not a dataflash log, unknown check,
or --strict and the log has integrity errors). Every report starts with the log's
integrity diagnostics; nothing found wrong with the input is ever silently repaired.
"""
from __future__ import annotations

import argparse
import io
import json
import math
import os
import sys

from . import __version__
from .parser import Log, LogIntegrityError
from .flight import airborne_window, WINDOW_METHODS
from .analysis import ALL_CHECKS, run, Section
from .checks import FAIL, PASS, SKIP, WARN, T
from .report import fmt, table

CHECK_NAMES = [n for n, _ in ALL_CHECKS]
SCHEMA_VERSION = "alog/2"
EXIT_INPUT = 3


# ------------------------------------------------------------------ plumbing

def _utf8_stdout():
    """Windows consoles default to a legacy code page; force UTF-8 so report text
    (degree signs, micro, arrows) never raises UnicodeEncodeError mid-report."""
    for stream in (sys.stdout, sys.stderr):
        try:
            if hasattr(stream, "reconfigure"):
                stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError, io.UnsupportedOperation):
            pass


class InputError(Exception):
    """The input cannot be analysed. Exit code 3."""


def _emit_json(obj):
    print(json.dumps(obj, indent=2, default=_json_default, allow_nan=False))


def _json_default(o):
    try:
        import numpy as np
        if isinstance(o, np.generic):
            v = o.item()
            if isinstance(v, float) and not math.isfinite(v):
                return None
            return v
        if isinstance(o, np.ndarray):
            return [_json_default(x) if not isinstance(x, (int, float, str)) else x for x in o.tolist()]
    except ImportError:  # pragma: no cover
        pass
    if isinstance(o, float) and not math.isfinite(o):
        return None
    if isinstance(o, bytes):
        return o.decode("latin-1", "replace")
    return str(o)


def _finite(obj):
    """Recursively replace NaN/Inf with None so allow_nan=False never trips."""
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: _finite(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_finite(v) for v in obj]
    return obj


def _load(path, args):
    if not os.path.exists(path):
        raise InputError(f"no such file: {path}")
    if os.path.isdir(path):
        raise InputError(f"{path} is a directory, not a log")
    try:
        log = Log(path, use_cache=not args.no_cache, verbose=args.verbose, strict=args.strict)
    except LogIntegrityError as exc:
        raise InputError(f"{path}: rejected under --strict\n{exc.diagnostics.render()}")
    if log.diagnostics.has("NO_FMT") or log.diagnostics.has("EMPTY_FILE") or log.n_messages == 0:
        raise InputError(f"{path} is not a readable DataFlash .bin log:\n{log.diagnostics.render()}")
    return log


def _window(log, args):
    try:
        return airborne_window(log, method=args.window, pad=args.pad)
    except ValueError as exc:
        raise InputError(str(exc))


def _envelope(command, log=None, window=None, **extra):
    env = dict(schema=SCHEMA_VERSION, tool="ardupilot-log-tools", version=__version__, command=command)
    if log is not None:
        env["log"] = dict(path=log.path, file_name=os.path.basename(log.path), file_size=log.file_size,
                          n_messages=log.n_messages, integrity=log.diagnostics.to_dict())
    if window is not None:
        env["window"] = window.to_dict()
    env.update(extra)
    return env


def _integrity_header(log):
    d = log.diagnostics
    lines = [f"**Log integrity:** {d.summary()}."]
    for i in sorted(d.issues, key=lambda i: {"error": 0, "warning": 1, "info": 2}[i.severity]):
        if i.severity != "info":
            lines.append(f"- {i.line()}")
    if d.infos:
        lines.append(f"- {len(d.infos)} info-level note(s); `alog integrity` lists them.")
    return "\n".join(lines)


def _exit_code(secs):
    rs = [r for s in secs for r in s.results]
    if any(r.status == FAIL for r in rs):
        return 2
    if any(r.status == WARN for r in rs):
        return 1
    return 0


def _verdict(secs):
    rs = [r for s in secs for r in s.results]
    counts = {k: sum(r.status == k for r in rs) for k in (PASS, WARN, FAIL, SKIP)}
    bad = sorted([r for r in rs if r.status in (WARN, FAIL)], key=lambda r: -r.severity)
    return counts, bad


def _verdict_md(secs):
    counts, bad = _verdict(secs)
    out = ["\n## Verdict\n",
           f"{counts[PASS]} pass, {counts[WARN]} warn, {counts[FAIL]} fail, {counts[SKIP]} skipped "
           "(not logged or not applicable).\n"]
    if bad:
        out.append(table(["status", "check", "detail"], [[r.status, r.name, r.summary] for r in bad],
                         align=["l", "l", "l"]))
    else:
        out.append("Nothing above threshold. A clean sheet is not the same as a good flight - "
                   "read the numbers, not just the verdicts.")
    skipped = [r for s in secs for r in s.results if r.status == SKIP]
    if skipped:
        out.append("\nSkipped (could not run - NOT a pass): " + "; ".join(f"{r.name}: {r.summary}" for r in skipped))
    return "\n".join(out)


# ------------------------------------------------------------------ commands

def cmd_check(args, names):
    log = _load(args.log, args)
    w = _window(log, args)
    secs = run(log, names, window=w, normalise=not args.raw_factors)
    if args.json:
        counts, bad = _verdict(secs)
        _emit_json(_finite(_envelope("all" if names is None else ",".join(names), log, w,
                                     sections=[s.to_dict() for s in secs],
                                     verdict=dict(counts=counts,
                                                  findings=[r.to_dict() for r in bad]),
                                     exit_code=_exit_code(secs))))
        return _exit_code(secs)
    print(f"# Log analysis - {os.path.basename(args.log)}\n")
    print(_integrity_header(log) + "\n")
    print(f"Window: {w.t0:.1f}-{w.t1:.1f} s ({w.duration:.1f} s) via **{w.method}**. "
          f"Quote this method when comparing flights.\n")
    for s in secs:
        print(s.render())
    print(_verdict_md(secs))
    return _exit_code(secs)


def cmd_info(args):
    log = _load(args.log, args)
    info = log.info()
    q = log.quality()
    w = airborne_window(log, method="auto")
    from .analysis import check_coverage
    cov = check_coverage(log, w)
    if args.json:
        _emit_json(_finite(_envelope("info", log, w, info=info, quality=q.to_dict(),
                                     coverage=cov.to_dict(),
                                     exit_code=0 if log.diagnostics.ok else 2)))
        return 0 if log.diagnostics.ok else 2
    print(f"# {info['file_name']}\n")
    rows = [[k, info[k]] for k in ("firmware", "vehicle", "board", "file_size", "sha256", "n_messages",
                                   "n_types_present", "n_types_declared", "duration_s", "log_start_utc",
                                   "gps_time_available", "n_params", "frame_class", "frame_type")]
    print(table(["item", "value"], rows, align=["l", "l"]))
    print()
    print(log.diagnostics.render())
    print()
    print(q.render())
    print()
    print(f"Auto window: {w.t0:.1f}-{w.t1:.1f} s ({w.duration:.1f} s) via {w.method}")
    print(cov.render())
    return 0 if log.diagnostics.ok else 2


def cmd_integrity(args):
    log = _load(args.log, args)
    q = log.quality()
    if args.json:
        _emit_json(_finite(_envelope("integrity", log, structure=log.diagnostics.to_dict(),
                                     quality=q.to_dict(),
                                     exit_code=2 if not log.diagnostics.ok else (1 if q.warnings or log.diagnostics.warnings else 0))))
    else:
        print(f"# Integrity - {os.path.basename(args.log)}\n")
        print(f"{log.bytes_parsed} of {log.file_size} bytes decoded; {log.resync_bytes} bytes skipped in "
              f"{log.resync_events} resync event(s).\n")
        print("## Structure\n" + log.diagnostics.render())
        print("\n## Data quality\n" + q.render())
    if not log.diagnostics.ok:
        return 2
    return 1 if (q.warnings or log.diagnostics.warnings) else 0


def cmd_types(args):
    log = _load(args.log, args)
    rows = []
    for k, v in log.types().items():
        r = log.rate_hz(k, steady_only=True)
        rows.append([k, v, f"{r:.1f}" if r else "-", len(log.instances(k)), ",".join(log.columns(k))])
    absent = sorted(set(log.declared_types()) - set(log.messages))
    if args.json:
        _emit_json(_finite(_envelope("types", log,
                                     present=[dict(name=r[0], count=r[1], rate_hz=None if r[2] == "-" else float(r[2]),
                                                   instances=r[3], fields=log.columns(r[0]),
                                                   units=log.units(r[0])) for r in rows],
                                     declared_but_absent=absent)))
        return 0
    print(f"# {os.path.basename(args.log)}\n")
    print(f"{log.n_messages} messages, {len(log.messages)} types present, {len(absent)} declared but never "
          f"logged. {log.diagnostics.summary()}.\n")
    print(table(["message", "count", "rate Hz", "inst", "fields"], rows, align=["l", "r", "r", "r", "l"]))
    print(f"\nDeclared but absent: {', '.join(absent)}")
    return 0


def cmd_fields(args):
    log = _load(args.log, args)
    mf = log.formats_by_name.get(args.message)
    if mf is None:
        raise InputError(f"{args.message}: no FMT for this message in the log. Present types: "
                         + ", ".join(sorted(log.messages)))
    d = log.df(args.message)
    units, mults = log.units(args.message), log.multipliers(args.message)
    rows = []
    for col, ch in zip(mf.columns, mf.format):
        stats = ""
        if not d.empty and col in d.columns and d[col].dtype.kind in "fiu":
            v = d[col].values
            import numpy as np
            fin = v[np.isfinite(v.astype(float))]
            if fin.size:
                stats = f"{fin.min():.6g} .. {fin.max():.6g}"
        m = mults.get(col)
        rows.append([col, ch, units.get(col, ""), "none" if m in (None, 0, 0.0) else fmt(m), stats])
    if args.json:
        _emit_json(_finite(_envelope("fields", log, message=args.message, format=mf.to_dict(),
                                     count=len(d), rate_hz=log.rate_hz(args.message),
                                     instance_field=log.instance_field(args.message),
                                     fields=[dict(name=r[0], type=r[1], unit=r[2], mult=mults.get(r[0]),
                                                  range=r[4]) for r in rows])))
        return 0
    r = log.rate_hz(args.message)
    print(f"# {args.message}\n")
    print(f"{len(d)} records, {f'{r:.1f} Hz' if r else 'no rate'}, format `{mf.format}`, "
          f"instance field {log.instance_field(args.message) or 'none'}.\n")
    print(table(["field", "type", "unit", "MULT (not applied)", "range in log"], rows,
                align=["l", "l", "l", "r", "l"]))
    print("\nMULT is display metadata from FMTU; values above are format-char scaled only (c/C/e/E x0.01, L x1e-7).")
    return 0


def cmd_dump(args):
    log = _load(args.log, args)
    d = log.df(args.message)
    if d.empty:
        raise InputError(f"{args.message}: not present in this log. Present types: " + ", ".join(sorted(log.messages)))
    if args.instance is not None:
        inst = log.instances(args.message)
        if args.instance not in inst:
            raise InputError(f"{args.message}: no instance {args.instance}; have {sorted(inst)}")
        d = inst[args.instance]
    if args.window != "none":
        d = _window(log, args).clip(d)
    if args.fields:
        want = [c for c in args.fields.split(",") if c]
        missing = [c for c in want if c not in d.columns]
        if missing:
            raise InputError(f"{args.message}: no field(s) {', '.join(missing)}; have {', '.join(d.columns)}")
        d = d[want]
    if args.every and args.every > 1:
        d = d.iloc[::args.every]
    if args.limit:
        d = d.head(args.limit)
    if args.json:
        recs = json.loads(d.to_json(orient="records", default_handler=_json_default))
        _emit_json(_finite(_envelope("dump", log, message=args.message, n=len(d), rows=recs)))
        return 0
    d.to_csv(sys.stdout, index=False, lineterminator="\n")
    return 0


def _read_param_file(path):
    if not os.path.exists(path):
        raise InputError(f"no such param file: {path}")
    other, bad = {}, 0
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.replace("\t", ",").replace(" ", ",").split(",")
            parts = [p for p in parts if p]
            if len(parts) >= 2:
                try:
                    other[parts[0]] = float(parts[1])
                    continue
                except ValueError:
                    pass
            bad += 1
    if not other:
        raise InputError(f"{path}: no NAME,VALUE lines found - not a .param file?")
    return other, bad


def cmd_params(args):
    log = _load(args.log, args)
    p = log.params()
    if not p:
        raise InputError("no PARM records in this log")
    defaults = log.param_defaults()
    if args.non_default:
        import numpy as np
        p = {k: v for k, v in p.items() if k in defaults and np.isfinite(defaults[k])
             and not np.isclose(v, defaults[k], rtol=1e-6, atol=1e-9)}
    if args.grep:
        p = {k: v for k, v in p.items() if args.grep.upper() in k.upper()}
    if not args.diff:
        if args.json:
            _emit_json(_finite(_envelope("params", log, n=len(p),
                                         params={k: dict(value=v, default=defaults.get(k)) for k, v in sorted(p.items())},
                                         changes=[dict(t=t, name=n, old=o, new=v) for t, n, o, v in log.param_changes()])))
            return 0
        for k in sorted(p):
            print(f"{k},{p[k]:.8g}" + (f",{defaults[k]:.8g}" if k in defaults and defaults[k] == defaults[k] else ""))
        return 0
    other, bad_lines = _read_param_file(args.diff)
    rows = []
    for k in sorted(set(p) | set(other)):
        a, b = p.get(k), other.get(k)
        if a is None:
            rows.append([k, None, b, "only in file"])
        elif b is None:
            rows.append([k, a, None, "only in log"])
        elif not math.isclose(a, b, rel_tol=1e-6, abs_tol=1e-9):
            # float32 in the log vs 7-significant-figure text in the .param file:
            # 0.30000001 and 0.3 are the same number.
            rows.append([k, a, b, "changed"])
    if args.json:
        _emit_json(_finite(_envelope("params", log, diff_file=args.diff, unparsed_lines=bad_lines,
                                     differences=[dict(name=r[0], in_log=r[1], in_file=r[2], note=r[3]) for r in rows])))
        return 0
    print(table(["param", "in log", os.path.basename(args.diff), "note"],
                [[r[0], fmt(r[1]) if r[1] is not None else "-", fmt(r[2]) if r[2] is not None else "-", r[3]] for r in rows],
                align=["l", "r", "r", "l"]))
    print(f"\n{len(rows)} differences" + (f"; {bad_lines} line(s) in the .param file could not be parsed" if bad_lines else "")
          + ". A parameter that *disappears* between two captures is usually\n"
          "ArduPilot hiding a disabled subtree (e.g. all FFT_* when FFT_ENABLE goes to 0), not data loss.")
    return 0


def cmd_compare(args):
    names = args.checks.split(",") if args.checks else CHECK_NAMES
    unknown = [n for n in names if n not in CHECK_NAMES]
    if unknown:
        raise InputError(f"unknown check(s): {', '.join(unknown)}; known: {', '.join(CHECK_NAMES)}")
    logs = [(p, _load(p, args)) for p in args.logs]
    per = []
    for path, log in logs:
        w = _window(log, args)
        secs = run(log, names, window=w, normalise=not args.raw_factors)
        per.append((os.path.basename(path), log, w, {r.name: r for s in secs for r in s.results}))
    keys = []
    for _, _, _, rs in per:
        for k in rs:
            if k not in keys:
                keys.append(k)
    if args.json:
        _emit_json(_finite(_envelope(
            "compare", logs=[dict(file_name=n, path=l.path, integrity=l.diagnostics.to_dict(), window=w.to_dict())
                             for n, l, w, _ in per],
            checks=[dict(name=k, per_log=[(rs[k].to_dict() if k in rs else None) for _, _, _, rs in per]) for k in keys])))
        return 0
    print("# Like-for-like comparison\n")
    print("Both logs run through identical code, so every pair below is comparable.\n")
    for name, log, w, _ in per:
        print(f"- **{name}**: window {w.t0:.1f}-{w.t1:.1f} s ({w.duration:.1f} s) via {w.method}; "
              f"integrity: {log.diagnostics.summary()}")
    print()
    rows = []
    for k in keys:
        row = [k]
        for _, _, _, rs in per:
            r = rs.get(k)
            if r is None:
                row.append("-")
            else:
                v = r.evidence.get("signed", r.evidence.get("value"))
                row.append(f"{r.status} {fmt(v)}" if v is not None else r.status)
        rows.append(row)
    print(table(["check"] + [n for n, _, _, _ in per], rows, align=["l"] + ["l"] * len(per)))
    return 0


def cmd_fft(args):
    from .spectral import analyse, sources, SpectralError
    log = _load(args.log, args)
    w = _window(log, args)
    srcs = sources(log)
    if args.list_sources:
        if args.json:
            _emit_json(_finite(_envelope("fft", log, w, sources=srcs)))
        else:
            print(table(["source", "kind", "rate Hz", "Nyquist Hz", "samples", "note"],
                        [[s["name"], s["kind"], round(s["rate_hz"], 1), round(s["nyquist_hz"], 1), s["n"], s["note"]]
                         for s in srcs], align=["l", "l", "r", "r", "r", "l"]))
        return 0
    try:
        r = analyse(log, source=args.source, window=w, axes=args.axes.split(",") if args.axes else None,
                    fmin=args.fmin, fmax=args.fmax, nperseg=args.nperseg, n_peaks=args.peaks,
                    max_jitter=args.max_jitter)
    except SpectralError as exc:
        raise InputError(f"FFT refused: {exc}. Sources available: " + ", ".join(s["name"] for s in srcs))
    src = r["source"]
    if args.plot:
        _plot_fft(r, args.plot, os.path.basename(args.log))
    if args.csv:
        import numpy as np
        with open(args.csv, "w", newline="") as fh:
            axes = list(r["axes"])
            fh.write("freq_hz," + ",".join(f"psd_{a}" for a in axes) + "\n")
            f = r["axes"][axes[0]][0]
            cols = [r["axes"][a][1] for a in axes]
            for i in range(len(f)):
                fh.write(f"{f[i]:.4f}," + ",".join(f"{c[i]:.6g}" for c in cols) + "\n")
    code = 1 if r["warnings"] else 0
    if args.json:
        out = dict(source=src, fs_hz=r["fs_hz"], band_hz=r["band_hz"], timing=r["timing"],
                   esc_fundamental_hz=r["esc_fundamental_hz"], peaks=r["peaks"], warnings=r["warnings"],
                   spectrum={a: dict(freq_hz=list(map(float, f)), psd=list(map(float, p)))
                             for a, (f, p) in r["axes"].items()} if args.spectrum else None,
                   plot=args.plot, csv=args.csv)
        _emit_json(_finite(_envelope("fft", log, w, fft=out, exit_code=code)))
        return code
    print(f"# FFT - {os.path.basename(args.log)}\n")
    print(_integrity_header(log) + "\n")
    tm = r["timing"]
    print(f"Source **{src['name']}** ({src['note']}) at {r['fs_hz']:.1f} Hz, Nyquist {r['fs_hz'] / 2:.1f} Hz, "
          f"band {r['band_hz'][0]:.0f}-{r['band_hz'][1]:.0f} Hz, window {w.t0:.1f}-{w.t1:.1f} s via {w.method}.")
    if "batches" in tm:
        print(f"{tm['batches']} contiguous batches of {tm['samples_per_batch']} samples; {tm['holes']} discarded for seqno holes.")
    else:
        print(f"{tm['n']} samples over {tm['span_s']:.1f} s; interval jitter {tm['jitter'] * 100:.2f}% "
              f"(limit {args.max_jitter * 100:.0f}%).")
    f0 = r["esc_fundamental_hz"]
    print(f"Motor fundamental from ESC telemetry: {f0:.1f} Hz." if f0 else "No ESC telemetry: motor orders unavailable.")
    print()
    for wmsg in r["warnings"]:
        print(f"**WARNING:** {wmsg}")
    if r["warnings"]:
        print()
    rows = [[a, round(q["freq_hz"], 1), round(q["db_above_floor"], 1), round(q["prominence_db"], 1),
             round(q["order"], 2) if q["order"] else "-"] for a, pk in r["peaks"].items() for q in pk]
    print(table(["axis", "peak Hz", "dB above floor", "prominence dB", "order"], rows))
    if args.plot:
        print(f"\nwrote {args.plot}")
    if args.csv:
        print(f"wrote {args.csv}")
    return code


def _plot_fft(r, out, title):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        raise InputError("matplotlib is not installed; `pip install matplotlib` for --plot")
    fig, ax = plt.subplots(figsize=(11, 4.5))
    lo, hi = r["band_hz"]
    for a, (f, p) in r["axes"].items():
        m = (f >= lo) & (f <= hi)
        ax.semilogy(f[m], p[m], lw=1.0, label=a)
    f0 = r["esc_fundamental_hz"]
    if f0:
        for k, ls in ((1, "--"), (2, ":"), (3, ":")):
            if f0 * k <= hi:
                ax.axvline(f0 * k, color="#7f8c8d", ls=ls, lw=1, label=f"motor x{k} = {f0 * k:.0f} Hz")
    ax.set_xlabel("Hz")
    ax.set_ylabel("PSD (units^2/Hz)")
    ax.set_title(f"{title} - {r['source']['name']} @ {r['fs_hz']:.0f} Hz")
    ax.grid(alpha=0.3, which="both")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=140)


def cmd_files(args):
    log = _load(args.log, args)
    files = log.files()
    if not files:
        raise InputError("no FILE records (embedded files) in this log")
    rows = [[n, len(b)] for n, b in sorted(files.items())]
    written = []
    if args.out:
        os.makedirs(args.out, exist_ok=True)
        for n, b in files.items():
            safe = n.replace("@", "").replace("/", "_").replace("\\", "_")
            path = os.path.join(args.out, safe)
            with open(path, "wb") as fh:
                fh.write(b)
            written.append(path)
    if args.json:
        _emit_json(_finite(_envelope("files", log, files=[dict(name=n, bytes=s) for n, s in rows], written=written)))
        return 0
    print(table(["file", "bytes"], rows, align=["l", "r"]))
    for p in written:
        print(f"wrote {p}")
    return 0


def cmd_schema(args):
    schema = dict(
        schema=SCHEMA_VERSION,
        description="Every JSON document alog emits carries schema, tool, version, command, and "
                    "(when a log was read) log.integrity. Check reports add window, sections, verdict, exit_code.",
        exit_codes={0: "all results PASS or SKIP", 1: "at least one WARN", 2: "at least one FAIL",
                    3: "input error: missing file, not a dataflash log, unknown check, or --strict rejected the log"},
        result=dict(name="str", status="PASS|WARN|FAIL|SKIP", summary="str, always contains the number",
                    evidence="dict of the numbers behind the verdict (value, warn, fail, ...)",
                    severity="int; WARN 10, FAIL 20 unless a check raises it", source="where the threshold came from"),
        section=dict(key="str, the check name", title="str", worst="worst status among results",
                     results="[result]", tables="[{name, columns, rows}]", notes="[markdown str]"),
        integrity=dict(ok="bool: no error-level issue", worst="ok|info|warning|error",
                       issues="[{code, severity, subject, message, count, first_offset, offsets, detail}]",
                       codes="see reference/integrity-codes.md"),
        window=dict(t0="s since boot", t1="s", duration_s="s", method="how it was chosen - quote it",
                    note="str"),
        checks=CHECK_NAMES,
        window_methods=list(WINDOW_METHODS) + ["T0:T1"],
        thresholds={k: dict(warn=v["warn"], fail=v["fail"], source=v["source"], note=v["note"]) for k, v in T.items()},
    )
    _emit_json(schema)
    return 0


# ------------------------------------------------------------------ main

def main(argv=None):
    _utf8_stdout()
    ap = argparse.ArgumentParser(prog="alog", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--version", action="version", version=f"ardupilot-log-tools {__version__}")
    ap.add_argument("--no-cache", action="store_true", help="force a re-parse; do not read or write .dfcache")
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--strict", action="store_true",
                    help="refuse (exit 3) any log with error-level integrity issues instead of analysing it")
    ap.add_argument("--json", action="store_true", help="machine-readable output (one JSON document)")
    sub = ap.add_subparsers(dest="cmd", required=True, metavar="command")

    def add_window(p):
        p.add_argument("--window", default="auto", metavar="METHOD",
                       help="airborne window: " + "|".join(WINDOW_METHODS) + " or T0:T1 seconds; "
                            "'rpm' is the most reproducible for motor/notch work (default auto)")
        p.add_argument("--pad", type=float, default=0.0, help="trim N seconds off each end of the window")

    def add_common(p):
        p.add_argument("log")
        add_window(p)
        p.add_argument("--raw-factors", action="store_true",
                       help="un-normalised cos mix factors, to reproduce pre-2026-09 trim numbers")
        p.add_argument("--json", action="store_true", help=argparse.SUPPRESS)

    q = sub.add_parser("info", help="identity, integrity, data quality and logging coverage - run first")
    q.add_argument("log")
    q.add_argument("--json", action="store_true", help=argparse.SUPPRESS)
    q = sub.add_parser("integrity", help="structural and data-quality diagnostics only")
    q.add_argument("log")
    q.add_argument("--json", action="store_true", help=argparse.SUPPRESS)
    p = sub.add_parser("all", help="run every check")
    add_common(p)
    for name, fn in ALL_CHECKS:
        if name == "integrity":          # has its own richer subcommand above
            continue
        q = sub.add_parser(name, help=(fn.__doc__ or "").strip().split("\n")[0] or f"{name} check")
        add_common(q)
    q = sub.add_parser("types", help="messages present, with counts, rates and fields")
    q.add_argument("log")
    q.add_argument("--json", action="store_true", help=argparse.SUPPRESS)
    q = sub.add_parser("fields", help="one message's fields with units, multipliers and value ranges")
    q.add_argument("log")
    q.add_argument("message")
    q.add_argument("--json", action="store_true", help=argparse.SUPPRESS)
    q = sub.add_parser("dump", help="dump one message to CSV (or JSON rows)")
    q.add_argument("log")
    q.add_argument("message")
    q.add_argument("--fields", help="comma-separated field list")
    q.add_argument("--instance", type=int)
    q.add_argument("--every", type=int, default=1, help="keep every Nth row")
    q.add_argument("--limit", type=int, default=0, help="stop after N rows")
    add_window(q)
    q.add_argument("--json", action="store_true", help=argparse.SUPPRESS)
    q = sub.add_parser("params", help="parameters from the log, optionally diffed against a .param file")
    q.add_argument("log")
    q.add_argument("--diff", help="a .param file to diff against")
    q.add_argument("--non-default", action="store_true", help="only parameters that differ from PARM.Default")
    q.add_argument("--grep", help="substring filter on the name")
    q.add_argument("--json", action="store_true", help=argparse.SUPPRESS)
    q = sub.add_parser("compare", help="run the same checks over several logs")
    q.add_argument("logs", nargs="+")
    q.add_argument("--checks", help=f"comma-separated subset of: {', '.join(CHECK_NAMES)}")
    q.add_argument("--window", default="rpm", metavar="METHOD")
    q.add_argument("--pad", type=float, default=0.0)
    q.add_argument("--raw-factors", action="store_true")
    q.add_argument("--json", action="store_true", help=argparse.SUPPRESS)
    q = sub.add_parser("fft", help="local FFT (scipy) of a logged signal, peaks labelled in motor orders")
    q.add_argument("log")
    add_window(q)
    q.add_argument("--source", help="signal to transform, e.g. ISBD:gyro:0, GYR:0, IMU:0, RATE (default: fastest gyro)")
    q.add_argument("--list-sources", action="store_true", help="list transformable signals and exit")
    q.add_argument("--axes", help="comma-separated axes, e.g. x,y or GyrX,GyrZ")
    q.add_argument("--fmin", type=float, default=5.0)
    q.add_argument("--fmax", type=float, default=None)
    q.add_argument("--nperseg", type=int, default=None, help="Welch segment length (default auto, <=1024)")
    q.add_argument("--peaks", type=int, default=6)
    q.add_argument("--max-jitter", type=float, default=0.05, help="refuse a signal whose p95 sample interval exceeds the median by this fraction")
    q.add_argument("--plot", help="write a PNG of the spectrum")
    q.add_argument("--csv", help="write the PSD to a CSV file")
    q.add_argument("--spectrum", action="store_true", help="include the full PSD arrays in --json output")
    q.add_argument("--json", action="store_true", help=argparse.SUPPRESS)
    q = sub.add_parser("files", help="list or extract files embedded in the log (FILE records)")
    q.add_argument("log")
    q.add_argument("--out", help="directory to write the files into")
    q.add_argument("--json", action="store_true", help=argparse.SUPPRESS)
    q = sub.add_parser("schema", help="print the JSON output contract, check names and thresholds")
    q.add_argument("--json", action="store_true", help=argparse.SUPPRESS)

    args = ap.parse_args(argv)
    # --json may be given before or after the subcommand
    args.json = bool(getattr(args, "json", False)) or "--json" in (argv if argv is not None else sys.argv[1:])
    if not hasattr(args, "window"):
        args.window, args.pad = "auto", 0.0
    if not hasattr(args, "raw_factors"):
        args.raw_factors = False

    dispatch = {"info": cmd_info, "integrity": cmd_integrity, "types": cmd_types, "fields": cmd_fields,
                "dump": cmd_dump, "params": cmd_params, "compare": cmd_compare, "fft": cmd_fft,
                "files": cmd_files, "schema": cmd_schema}
    try:
        if args.cmd in dispatch:
            return dispatch[args.cmd](args)
        if args.cmd == "all":
            return cmd_check(args, None)
        return cmd_check(args, [args.cmd])
    except InputError as exc:
        if args.json:
            _emit_json(dict(schema=SCHEMA_VERSION, tool="ardupilot-log-tools", version=__version__,
                            command=args.cmd, error=str(exc), exit_code=EXIT_INPUT))
        else:
            print(f"error: {exc}", file=sys.stderr)
        return EXIT_INPUT
    except (BrokenPipeError, OSError) as exc:
        # `alog ... | head` closes the pipe early; that is not a failure of the analysis.
        if isinstance(exc, BrokenPipeError) or getattr(exc, "errno", None) in (22, 32):
            try:
                sys.stdout = open(os.devnull, "w")
            except OSError:
                pass
            return 0
        raise


def entry():
    """Console-script entry point: also swallows the pipe-closed flush error at exit."""
    code = main()
    try:
        sys.stdout.flush()
    except (BrokenPipeError, OSError):
        sys.stdout = open(os.devnull, "w")
    return code


if __name__ == "__main__":
    sys.exit(entry())
