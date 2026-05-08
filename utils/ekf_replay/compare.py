#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# compare.py
#
# Diff an ekf_ins replay output against the recorded INS_Out from the same
# flight (typically produced by cf_ins).  Both inputs are CSV files with an
# 'timestamp' column.  Aligns on timestamp (nearest-neighbour) and reports
# per-channel mean / RMS / max-abs difference.  If matplotlib is available
# also writes a PNG of the side-by-side traces.
#
# Usage:
#     python3 compare.py REFERENCE.csv REPLAY.csv [--plot diff.png]
#
# REFERENCE is normally  out/mlog_msg_<id>_INS_Out.csv  produced by
# parse_mlog.py from the firmware log; REPLAY is the file written by
# ./ekf_replay (default INS_Out_replay.csv).

import argparse
import csv
import math
import sys
from pathlib import Path

CHANNELS = [
    "phi", "theta", "psi",
    "p", "q", "r",
    "ax", "ay", "az",
    "vn", "ve", "vd",
    "x_R", "y_R", "h_R", "h_AGL",
    "lat", "lon", "alt",
]


def read_csv(path):
    """Return (timestamps_ms[list], {col: [values]}).  Skips empty / non-numeric rows."""
    ts = []
    cols = {}
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        if "timestamp" not in reader.fieldnames:
            raise SystemExit(f"{path}: no 'timestamp' column")
        for name in reader.fieldnames:
            cols[name] = []
        for row in reader:
            try:
                ts_v = int(float(row["timestamp"]))
            except (TypeError, ValueError):
                continue
            ts.append(ts_v)
            for k, v in row.items():
                try:
                    cols[k].append(float(v))
                except (TypeError, ValueError):
                    cols[k].append(float("nan"))
    return ts, cols


def align_nearest(ts_ref, ts_rep):
    """Return parallel index pairs (i_ref, i_rep) such that
       ts_rep[i_rep] is the closest sample to ts_ref[i_ref]."""
    pairs = []
    j = 0
    n_rep = len(ts_rep)
    for i, t in enumerate(ts_ref):
        while j + 1 < n_rep and abs(ts_rep[j + 1] - t) < abs(ts_rep[j] - t):
            j += 1
        pairs.append((i, j))
    return pairs


def stats(diffs):
    finite = [d for d in diffs if not math.isnan(d) and not math.isinf(d)]
    if not finite:
        return float("nan"), float("nan"), float("nan")
    n = len(finite)
    mean = sum(finite) / n
    rms  = math.sqrt(sum(d * d for d in finite) / n)
    mx   = max(abs(d) for d in finite)
    return mean, rms, mx


def main():
    ap = argparse.ArgumentParser(description="Compare ekf_ins replay against recorded INS_Out.")
    ap.add_argument("reference", type=Path, help="recorded INS_Out CSV (e.g. cf_ins log)")
    ap.add_argument("replay",    type=Path, help="ekf_replay output CSV")
    ap.add_argument("--plot",    type=Path, default=None,
                    help="optional PNG path; requires matplotlib")
    ap.add_argument("--channels", nargs="*", default=CHANNELS,
                    help="restrict comparison to these channel names")
    args = ap.parse_args()

    ts_ref, cols_ref = read_csv(args.reference)
    ts_rep, cols_rep = read_csv(args.replay)
    if not ts_ref or not ts_rep:
        raise SystemExit("at least one input has no rows")

    pairs = align_nearest(ts_ref, ts_rep)

    print(f"reference rows : {len(ts_ref)}")
    print(f"replay rows    : {len(ts_rep)}")
    print(f"compared       : {len(pairs)}")
    print()
    print(f"{'channel':<10}  {'mean':>12}  {'rms':>12}  {'max-abs':>12}")
    print(f"{'-'*10}  {'-'*12}  {'-'*12}  {'-'*12}")
    summary = {}
    for ch in args.channels:
        if ch not in cols_ref or ch not in cols_rep:
            continue
        diffs = []
        for i_ref, i_rep in pairs:
            r = cols_ref[ch][i_ref]
            p = cols_rep[ch][i_rep]
            diffs.append(p - r)
        m, rms, mx = stats(diffs)
        summary[ch] = (m, rms, mx)
        print(f"{ch:<10}  {m:12.6f}  {rms:12.6f}  {mx:12.6f}")

    if args.plot is not None:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not available; skipping plot", file=sys.stderr)
            return 0

        fig_chs = [c for c in args.channels if c in cols_ref and c in cols_rep]
        n = len(fig_chs)
        if n == 0:
            return 0
        ncols = 3
        nrows = (n + ncols - 1) // ncols
        fig, axes = plt.subplots(nrows, ncols, figsize=(4*ncols, 2*nrows), sharex=True)
        axes = axes.flatten() if hasattr(axes, "flatten") else [axes]
        for k, ch in enumerate(fig_chs):
            ax = axes[k]
            ax.plot([t/1000.0 for t in ts_ref], cols_ref[ch], label="ref",    linewidth=0.7)
            ax.plot([t/1000.0 for t in ts_rep], cols_rep[ch], label="replay", linewidth=0.7)
            ax.set_title(ch, fontsize=9)
            ax.tick_params(labelsize=7)
            ax.grid(True, linewidth=0.3)
            if k == 0:
                ax.legend(fontsize=7, loc="best")
        for k in range(len(fig_chs), len(axes)):
            axes[k].axis("off")
        for ax in axes[-ncols:]:
            ax.set_xlabel("t [s]", fontsize=8)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=120)
        print(f"plot saved to {args.plot}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
