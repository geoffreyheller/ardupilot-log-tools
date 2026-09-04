#!/usr/bin/env python3
"""alog - ArduPilot dataflash log analysis CLI.

    ./alog.py all      "2026-09-03 19-44-48.bin"        # the standard battery
    ./alog.py motors   "flight.bin"                      # one check
    ./alog.py notch    "flight.bin" --window rpm
    ./alog.py compare  "before.bin" "after.bin"          # like-for-like diff
    ./alog.py types    "flight.bin"                      # what's in the log
    ./alog.py dump     "flight.bin" RATE --fields t,RDes,R
    ./alog.py params   "flight.bin" --diff other.param

Every command prints markdown, ready to paste into a
`<Vehicle> Log Analysis - YYYY-MM-DD.md` report.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from dflog import Log, airborne_window                      # noqa: E402
from dflog.analysis import ALL_CHECKS, run                  # noqa: E402
from dflog.checks import FAIL, PASS, SKIP, WARN             # noqa: E402
from dflog.report import fmt, table                         # noqa: E402

CHECK_NAMES = [n for n, _ in ALL_CHECKS]


def _load(path, args):
    return Log(path, use_cache=not args.no_cache, verbose=args.verbose)


def _window(log, args):
    return airborne_window(log, method=args.window, pad=args.pad)


def cmd_check(args, names):
    log = _load(args.log, args)
    w = _window(log, args)
    secs = run(log, names, window=w, normalise=not args.raw_factors)
    if args.json:
        print(json.dumps([r.to_dict() for s in secs for r in s.results], indent=2))
        return _exit_code(secs)
    print(f"# Log analysis - {os.path.basename(args.log)}\n")
    print(f"Window: {w.t0:.1f}-{w.t1:.1f} s ({w.duration:.1f} s) via **{w.method}**. "
          f"Quote this method when comparing flights.\n")
    for s in secs:
        print(s.render())
    print(_verdict_block(secs))
    return _exit_code(secs)


def _verdict_block(secs):
    rs = [r for s in secs for r in s.results]
    bad = [r for r in rs if r.status in (WARN, FAIL)]
    out = ["\n## Verdict\n"]
    out.append(f"{sum(r.status == PASS for r in rs)} pass, "
               f"{sum(r.status == WARN for r in rs)} warn, "
               f"{sum(r.status == FAIL for r in rs)} fail, "
               f"{sum(r.status == SKIP for r in rs)} skipped (not logged).\n")
    if bad:
        out.append(table(["status", "check", "detail"],
                         [[r.status, r.name, r.summary] for r in
                          sorted(bad, key=lambda r: -r.severity)], align=["l", "l", "l"]))
    else:
        out.append("Nothing above threshold. Note that a clean sheet is not the same as a good\n"
                   "flight - read the numbers, not just the verdicts.")
    return "\n".join(out)


def _exit_code(secs):
    rs = [r for s in secs for r in s.results]
    if any(r.status == FAIL for r in rs):
        return 2
    if any(r.status == WARN for r in rs):
        return 1
    return 0


def cmd_types(args):
    log = _load(args.log, args)
    print(f"# {os.path.basename(args.log)}\n")
    print(f"{log.n_messages} messages, {len(log.messages)} types, {log.resync_bytes} resync bytes\n")
    rows = [[k, v, ",".join(log.columns(k))] for k, v in log.types().items()]
    print(table(["message", "count", "fields"], rows, align=["l", "r", "l"]))
    return 0


def cmd_dump(args):
    log = _load(args.log, args)
    d = log.df(args.message)
    if d.empty:
        print(f"{args.message}: not present", file=sys.stderr)
        return 3
    if args.instance is not None:
        d = log.instances(args.message).get(args.instance, d)
    if args.window != "none":
        d = _window(log, args).clip(d)
    if args.fields:
        cols = [c for c in args.fields.split(",") if c in d.columns]
        d = d[cols]
    d.to_csv(sys.stdout, index=False)
    return 0


def cmd_params(args):
    log = _load(args.log, args)
    p = log.params()
    if not args.diff:
        for k in sorted(p):
            print(f"{k},{p[k]:.8g}")
        return 0
    if not os.path.exists(args.diff):
        print(f"no such param file: {args.diff}", file=sys.stderr)
        return 3
    other = {}
    with open(args.diff) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.replace("\t", ",").replace(" ", ",").split(",")
            if len(parts) >= 2:
                try:
                    other[parts[0]] = float(parts[1])
                except ValueError:
                    pass
    rows = []
    for k in sorted(set(p) | set(other)):
        a, b = p.get(k), other.get(k)
        if a is None:
            rows.append([k, "-", f"{b:.8g}", "only in file"])
        elif b is None:
            rows.append([k, f"{a:.8g}", "-", "only in log"])
        elif not math.isclose(a, b, rel_tol=1e-6, abs_tol=1e-9):
            # float32 in the log vs 7-significant-figure text in the .param file:
            # 0.30000001 and 0.3 are the same number, so compare with a float32-sized
            # relative tolerance rather than exactly.
            rows.append([k, f"{a:.8g}", f"{b:.8g}", "changed"])
    print(table(["param", "in log", os.path.basename(args.diff), "note"], rows,
                align=["l", "r", "r", "l"]))
    print(f"\n{len(rows)} differences. A parameter that *disappears* between two captures is usually\n"
          "ArduPilot hiding a disabled subtree (e.g. all FFT_* when FFT_ENABLE goes to 0), not data loss.")
    return 0


def cmd_compare(args):
    logs = [(p, _load(p, args)) for p in args.logs]
    names = args.checks.split(",") if args.checks else CHECK_NAMES
    unknown = [n for n in names if n not in CHECK_NAMES]
    if unknown:
        print(f"unknown check(s): {', '.join(unknown)}; known: {', '.join(CHECK_NAMES)}",
              file=sys.stderr)
        return 3
    print("# Like-for-like comparison\n")
    print("Both logs run through identical code, so every pair below is comparable.\n")
    per = []
    for path, log in logs:
        w = _window(log, args)
        secs = run(log, names, window=w, normalise=not args.raw_factors)
        per.append((os.path.basename(path), w, {r.name: r for s in secs for r in s.results}))
    for name, w, _ in per:
        print(f"- **{name}**: window {w.t0:.1f}-{w.t1:.1f} s ({w.duration:.1f} s) via {w.method}")
    print()
    keys = []
    for _, _, rs in per:
        for k in rs:
            if k not in keys:
                keys.append(k)
    rows = []
    for k in keys:
        row = [k]
        for _, _, rs in per:
            r = rs.get(k)
            if r is None:
                row.append("-")
            else:
                v = r.evidence.get("signed", r.evidence.get("value"))
                row.append(f"{r.status} {fmt(v)}" if v is not None else r.status)
        rows.append(row)
    print(table(["check"] + [n for n, _, _ in per], rows, align=["l"] + ["l"] * len(per)))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(prog="alog", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-cache", action="store_true", help="force a re-parse")
    ap.add_argument("--verbose", action="store_true")
    sub = ap.add_subparsers(dest="cmd", required=True)

    def add_common(p, with_log=True):
        if with_log:
            p.add_argument("log")
        p.add_argument("--window", default="auto",
                       choices=["auto", "ev", "rpm", "throttle", "arm", "none"],
                       help="how to pick the airborne window; 'rpm' is the most reproducible "
                            "for motor/notch work")
        p.add_argument("--pad", type=float, default=0.0,
                       help="trim N seconds off each end of the window")
        p.add_argument("--raw-factors", action="store_true",
                       help="un-normalised cos mix factors, to reproduce pre-2026-09 trim numbers")
        p.add_argument("--json", action="store_true")

    p = sub.add_parser("all", help="run every check")
    add_common(p)
    for name, fn in ALL_CHECKS:
        q = sub.add_parser(name, help=(fn.__doc__ or "").strip().split("\n")[0] or f"{name} check")
        add_common(q)

    q = sub.add_parser("types", help="list messages and fields present")
    q.add_argument("log")
    q = sub.add_parser("dump", help="dump one message to CSV")
    q.add_argument("log")
    q.add_argument("message")
    q.add_argument("--fields", help="comma-separated field list")
    q.add_argument("--instance", type=int)
    q.add_argument("--window", default="auto",
                   choices=["auto", "ev", "rpm", "throttle", "arm", "none"])
    q.add_argument("--pad", type=float, default=0.0)
    q = sub.add_parser("params", help="params from the log, optionally diffed against a .param file")
    q.add_argument("log")
    q.add_argument("--diff", help="a .param file to diff against")
    q = sub.add_parser("compare", help="run the same checks over several logs")
    q.add_argument("logs", nargs="+")
    q.add_argument("--checks", help=f"comma-separated subset of: {', '.join(CHECK_NAMES)}")
    q.add_argument("--window", default="rpm",
                   choices=["auto", "ev", "rpm", "throttle", "arm", "none"])
    q.add_argument("--pad", type=float, default=0.0)
    q.add_argument("--raw-factors", action="store_true")
    q.add_argument("--json", action="store_true")

    args = ap.parse_args(argv)
    if args.cmd == "types":
        return cmd_types(args)
    if args.cmd == "dump":
        return cmd_dump(args)
    if args.cmd == "params":
        return cmd_params(args)
    if args.cmd == "compare":
        return cmd_compare(args)
    if args.cmd == "all":
        return cmd_check(args, None)
    return cmd_check(args, [args.cmd])


if __name__ == "__main__":
    sys.exit(main())
