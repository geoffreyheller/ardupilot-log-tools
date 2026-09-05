#!/usr/bin/env python3
"""Notch verification chart: pre/post-filter spectra plus notch tracking.

    python plot_notch.py flight.bin -o notch-verification-2026-09-04.png

Two panels:
  top    - pre- and post-filter gyro PSD from INS_LOG_BAT batch samples, with
           the tracked notch centre and its 2nd harmonic marked
  bottom - the notch's applied centre frequency (FCNS.CF) against the ESC-RPM
           fundamental over time, which is the measurement that actually says
           whether the notch is tracking

Needs INS_LOG_BAT_MASK=1 and INS_LOG_BAT_OPT=4 for the top panel; the bottom
panel needs only ESC telemetry and FCNS.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

from .parser import Log
from .flight import airborne_window, esc_fundamental
from .analysis import check_batch_fft


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("-o", "--out", default="notch-verification.png")
    ap.add_argument("--window", default="rpm", metavar="METHOD",
                    help="auto|ev|rpm|throttle|arm|none or T0:T1 seconds")
    ap.add_argument("--fmax", type=float, default=500.0)
    args = ap.parse_args(argv)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("error: matplotlib is not installed (pip install matplotlib)", file=sys.stderr)
        return 3
    if not os.path.exists(args.log):
        print(f"error: no such file: {args.log}", file=sys.stderr)
        return 3
    log = Log(args.log)
    if not log.diagnostics.ok:
        print(log.diagnostics.render(), file=sys.stderr)
    w = airborne_window(log, method=args.window)
    sec = check_batch_fft(log, w)
    spectra = sec.data.get("spectra", {})
    t, fund = esc_fundamental(log)
    f0 = float(np.nanmedian(fund[w.mask(t)])) if fund is not None else None

    orders = sec.data.get("orders", {})
    n_panels = 3 if orders else 2
    fig, axes = plt.subplots(n_panels, 1, figsize=(11, 4.2 * n_panels))

    ax = axes[0]
    gyro = {k: v for k, v in spectra.items() if k[0] == 1}
    if gyro:
        gyro_filt = log.params().get("INS_GYRO_FILTER", 42.0)
        order = sorted(gyro.items(),
                       key=lambda kv: -float(kv[1][1][kv[1][0] > gyro_filt * 1.5].sum()))
        for (label, style), (key, (f, ps, nb)) in zip(
                [("pre-filter", dict(color="#c0392b", lw=1.0)),
                 ("post-filter", dict(color="#2471a3", lw=1.2))], [order[0], order[-1]]):
            m = (f > 5) & (f <= args.fmax)
            ax.semilogy(f[m], ps[m], label=f"{label} (ISBH inst {key[1]}, {nb} batches)", **style)
        if f0:
            for k, ls in ((1, "--"), (2, ":")):
                ax.axvline(f0 * k, color="#7f8c8d", ls=ls, lw=1,
                           label=f"ESC fundamental x{k} = {f0*k:.0f} Hz")
        ax.axvline(gyro_filt, color="#f39c12", ls="-.", lw=1,
                   label=f"INS_GYRO_FILTER = {gyro_filt:.0f} Hz")
        ax.set_ylabel("gyro PSD  (deg/s)$^2$/Hz")
        ax.set_title("Pre- vs post-filter gyro spectrum")
        ax.legend(fontsize=8)
    else:
        ax.text(0.5, 0.5, "No batch IMU data.\nSet INS_LOG_BAT_MASK=1 and INS_LOG_BAT_OPT=4, "
                          "fly, then set the mask back to 0.",
                ha="center", va="center", transform=ax.transAxes)
        ax.set_xticks([])
        ax.set_yticks([])
    ax.set_xlabel("Hz")
    ax.grid(alpha=0.3, which="both")

    if orders:
        ax = axes[1]
        gyro_o = {k: v for k, v in orders.items() if k[0] == 1}
        gyro_filt = log.params().get("INS_GYRO_FILTER", 42.0)
        if len(gyro_o) >= 2:
            keys = sorted(gyro_o, key=lambda k: -float(
                spectra[k][1][spectra[k][0] > gyro_filt * 1.5].sum()))
            for (label, style), key in zip(
                    [("pre-filter", dict(color="#c0392b", lw=1.0)),
                     ("post-filter", dict(color="#2471a3", lw=1.2))], [keys[0], keys[-1]]):
                oa, ps, nb = gyro_o[key]
                ax.semilogy(oa, ps, label=f"{label}, order-normalised ({nb} windows)", **style)
            oa, opre, _ = gyro_o[keys[0]]
            _, opost, _ = gyro_o[keys[-1]]
            with np.errstate(divide="ignore", invalid="ignore"):
                tf = np.where(opre > 0, opost / opre, np.nan)
            for tgt in (1.0, 2.0):
                m = np.abs(oa - tgt) <= 0.12
                if m.any() and np.isfinite(tf[m]).any():
                    j = np.nanargmin(tf[m])
                    ax.axvline(oa[m][j], color="#27ae60", ls="--", lw=1,
                               label=f"deepest at order {oa[m][j]:.3f} "
                                     f"({10*np.log10(tf[m][j]):.1f} dB)")
            ax.axvline(1.0, color="#7f8c8d", ls=":", lw=1)
            ax.axvline(2.0, color="#7f8c8d", ls=":", lw=1)
            ax.legend(fontsize=8)
        ax.set_xlabel("order (multiples of the tracked notch centre)")
        ax.set_ylabel("gyro PSD  (deg/s)$^2$/Hz")
        ax.set_title("Order-normalised: each batch scaled by the notch centre the FC was tracking")
        ax.set_xlim(0.2, 3.2)
        ax.grid(alpha=0.3, which="both")

    ax = axes[-1]
    fcns = log.instances("FCNS")
    if fcns and fund is not None:
        ax.plot(t[w.mask(t)], fund[w.mask(t)], color="#111111", lw=2.2, alpha=0.35,
                label="ESC fundamental (RPM/60) - ground truth")
        for i, g in sorted(fcns.items()):
            gg = w.clip(g)
            if gg is None or gg.empty:
                continue
            ax.plot(gg["t"], gg["CF"], lw=1.2, label=f"notch {i} centre (FCNS.CF)")
            if "HF" in gg.columns and np.nanmedian(gg["HF"]) > 0:
                ax.plot(gg["t"], gg["HF"], lw=0.8, alpha=0.7, label=f"notch {i} 2nd harmonic")
        floor = log.params().get("INS_HNTCH_FREQ")
        if floor:
            ax.axhline(floor, color="#c0392b", ls="--", lw=1, label=f"INS_HNTCH_FREQ floor = {floor:.0f}")
        ax.set_ylabel("Hz")
        ax.set_xlabel("time (s)")
        ax.set_title(f"Notch tracking vs ESC ground truth - window {w.t0:.0f}-{w.t1:.0f}s via {w.method}")
        ax.legend(fontsize=8)
    else:
        ax.text(0.5, 0.5, "No FCNS and/or ESC telemetry.", ha="center", va="center",
                transform=ax.transAxes)
    ax.grid(alpha=0.3)

    fig.suptitle(os.path.basename(args.log), fontsize=10)
    fig.tight_layout()
    fig.savefig(args.out, dpi=140)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
