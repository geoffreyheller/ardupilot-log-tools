"""Markdown formatting, so every script's output pastes straight into a report."""

from __future__ import annotations

import math

__all__ = ["table", "kv", "heading", "fmt", "verdict", "bullet"]

PASS, WARN, FAIL, INFO = "PASS", "WARN", "FAIL", "n/a"
_MARK = {PASS: "PASS", WARN: "WARN", FAIL: "FAIL", INFO: "-"}


def fmt(v, nd=3):
    if v is None:
        return "-"
    if isinstance(v, float):
        if math.isnan(v):
            return "-"
        if v == int(v) and abs(v) < 1e15:
            return str(int(v))          # bitmasks, counts, byte sizes: never in scientific notation
        if v != 0 and (abs(v) >= 1e5 or abs(v) < 1e-3):
            return f"{v:.3g}"
        return f"{v:.{nd}f}".rstrip("0").rstrip(".") or "0"
    return str(v)


def heading(text, level=2):
    return f"\n{'#' * level} {text}\n"


def _esc(c):
    # A literal pipe inside a cell breaks the markdown table.
    return str(c).replace("|", "\\|")


def table(headers, rows, align=None):
    headers = [_esc(h) for h in headers]
    rows = [[_esc(fmt(c) if not isinstance(c, str) else c) for c in r] for r in rows]
    widths = [max(len(headers[i]), *(len(r[i]) for r in rows)) if rows else len(headers[i])
              for i in range(len(headers))]
    align = list(align or ["l"] + ["r"] * (len(headers) - 1))
    align += ["r"] * (len(headers) - len(align))
    sep = []
    for w, a in zip(widths, align):
        sep.append("-" * (w - 1) + (":" if a == "r" else "-") if w > 1 else "-")
    out = ["| " + " | ".join(h.rjust(w) if align[i] == "r" else h.ljust(w)
                             for i, (h, w) in enumerate(zip(headers, widths))) + " |",
           "| " + " | ".join(sep) + " |"]
    for r in rows:
        out.append("| " + " | ".join(c.rjust(w) if align[i] == "r" else c.ljust(w)
                                     for i, (c, w) in enumerate(zip(r, widths))) + " |")
    return "\n".join(out)


def kv(pairs, indent=""):
    width = max((len(str(k)) for k, _ in pairs), default=0)
    return "\n".join(f"{indent}{str(k).ljust(width)} : {fmt(v) if not isinstance(v, str) else v}"
                     for k, v in pairs)


def bullet(text, level=0):
    return "  " * level + f"- {text}"


def verdict(status, text):
    """One line stating a pass/warn/fail plus the reason. Always give the number."""
    return f"[{_MARK.get(status, status)}] {text}"
