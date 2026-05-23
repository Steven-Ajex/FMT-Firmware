#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# compare_extpos.py
#
# Compare an ekf_ins replay output against External_Pos ground truth.
# Aligns on timestamp (nearest-neighbour) and reports per-channel
# mean / RMS / max-abs error for x_R / y_R / h_R against External_Pos
# x / y / -z (External_Pos.z is down-positive; INS h_R is up-positive,
# typed as height above lat_0/alt_0, so we flip the sign on z).

import argparse
import csv
import math
import sys
from pathlib import Path


def read_csv(path, cols):
    out = {c: [] for c in cols}
    ts = []
    with open(path, "r", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                t = int(float(row["timestamp"]))
            except (TypeError, ValueError):
                continue
            ts.append(t)
            for c in cols:
                try:
                    out[c].append(float(row[c]))
                except (TypeError, ValueError):
                    out[c].append(float("nan"))
    return ts, out


def nearest_index(sorted_ts, t):
    lo, hi = 0, len(sorted_ts) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if sorted_ts[mid] < t:
            lo = mid + 1
        else:
            hi = mid
    if lo > 0 and abs(sorted_ts[lo - 1] - t) <= abs(sorted_ts[lo] - t):
        return lo - 1
    return lo


def stats(diffs):
    n = len(diffs)
    if n == 0:
        return float("nan"), float("nan"), float("nan")
    m = sum(diffs) / n
    rms = math.sqrt(sum(d * d for d in diffs) / n)
    mx = max(abs(d) for d in diffs)
    return m, rms, mx


def main():
    ap = argparse.ArgumentParser(description="ekf_ins replay vs External_Pos ground truth")
    ap.add_argument("ext", type=Path, help="External_Pos CSV (mlog_msg_*_External_Pos.csv)")
    ap.add_argument("replay", type=Path, help="INS_Out_replay.csv")
    args = ap.parse_args()

    ext_ts, ext = read_csv(args.ext, ["x", "y", "z", "field_valid"])
    rep_ts, rep = read_csv(args.replay, ["x_R", "y_R", "h_R"])

    # walk External_Pos timestamps, look up nearest replay sample
    dx, dy, dz = [], [], []
    for i, t in enumerate(ext_ts):
        if ext["field_valid"][i] == 0:
            continue
        j = nearest_index(rep_ts, t)
        dx.append(rep["x_R"][j] - ext["x"][i])
        dy.append(rep["y_R"][j] - ext["y"][i])
        # External_Pos.z is down-positive (NED); INS h_R is height-up.
        # So expected: h_R == -z.  Error = h_R - (-z) = h_R + z.
        dz.append(rep["h_R"][j] + ext["z"][i])

    print(f"reference (External_Pos) rows : {len(ext_ts)}")
    print(f"replay rows                   : {len(rep_ts)}")
    print(f"compared                      : {len(dx)}")
    print()
    print("channel        mean        rms     max-abs")
    print("--------  --------  ---------  ----------")
    for name, arr in (("x_R - x", dx), ("y_R - y", dy), ("h_R - (-z)", dz)):
        m, rms, mx = stats(arr)
        print(f"{name:>10}  {m:8.3f}   {rms:8.3f}   {mx:8.3f}")


if __name__ == "__main__":
    sys.exit(main())
