#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# tune.py
#
# EKF parameter tuner.  Reads a recorded innovation log (either the
# offline replay's INS_Innov.csv or the firmware-recorded mlog bus
# parsed into CSV) and proposes new EKF_*_NSE / EKF_*_GATE values that
# bring the mean NIS of every measurement channel toward 1.0.
#
# Tuning law for measurement noise (R = sigma^2):
#
#       sigma_new = sigma_old * sqrt(mean_NIS)
#
# If the EKF model is good and HPH' << R the formula is exact; in
# general it is an iterative fixed point that converges in 2-3 passes.
# A max-step bound (default 3x) prevents one-shot overcorrection from
# outlier-heavy windows.
#
# Tuning law for innovation gate:
#   - If rejection > target (5 %) AND mean NIS is in [0.5, 5] (i.e.
#     the residuals are statistically reasonable but the gate is too
#     tight), suggest gate * 1.3.
#   - If rejection > target AND mean NIS > 5 (residuals genuinely too
#     large), do NOT touch the gate; the R update alone will fix the
#     rejection rate next iteration.
#
# Outputs:
#   - text report with one row per measurement channel
#   - optional FMT shell script (`param set EKF_FOO 0.123`) ready to
#     paste into the device console
#
# Usage:
#   python3 tune.py --innov-csv INS_Innov.csv \
#                   [--current  current.yml]   \
#                   [--shell-out tune_proposal.sh] \
#                   [--report   tune_report.txt]

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path


# --------------------------------------------------------------------
# Tag <-> parameter / gate mapping.  Mirrors ekf_core.c::k_innov_tags.
# Every tag points to the (R, gate) pair the firmware actually uses
# for that channel; multiple tags can share a parameter (gps_pos_n /
# gps_pos_e -> EKF_GPS_POS_NSE).  grav_* is intentionally absent --
# the gravity pseudo-measurement uses a hard-coded sigma in ekf_mag.c.
# --------------------------------------------------------------------
TAG_TABLE = [
    "mag", "grav_x", "grav_y",
    "gps_pos_n", "gps_pos_e", "gps_pos_d",
    "gps_vel_n", "gps_vel_e", "gps_vel_d",
    "baro", "rf",
    "opf_x", "opf_y",
    "ext_x", "ext_y", "ext_z",
    "ext_phi", "ext_theta", "ext_psi",
]

TAG_TO_PARAM = {
    "mag":       ("EKF_MAG_NSE",     "EKF_MAG_GATE"),
    "gps_pos_n": ("EKF_GPS_POS_NSE", "EKF_GPS_GATE"),
    "gps_pos_e": ("EKF_GPS_POS_NSE", "EKF_GPS_GATE"),
    "gps_pos_d": ("EKF_GPS_ALT_NSE", "EKF_GPS_GATE"),
    "gps_vel_n": ("EKF_GPS_VEL_NSE", "EKF_GPS_GATE"),
    "gps_vel_e": ("EKF_GPS_VEL_NSE", "EKF_GPS_GATE"),
    "gps_vel_d": ("EKF_GPS_VEL_NSE", "EKF_GPS_GATE"),
    "baro":      ("EKF_BARO_NSE",    "EKF_BARO_GATE"),
    "rf":        ("EKF_RF_NSE",      "EKF_RF_GATE"),
    "opf_x":     ("EKF_OPF_NSE",     "EKF_OPF_GATE"),
    "opf_y":     ("EKF_OPF_NSE",     "EKF_OPF_GATE"),
    "ext_x":     ("EKF_EXT_POS_NSE", None),
    "ext_y":     ("EKF_EXT_POS_NSE", None),
    "ext_z":     ("EKF_EXT_POS_NSE", None),
    "ext_phi":   ("EKF_EXT_ATT_NSE", None),
    "ext_theta": ("EKF_EXT_ATT_NSE", None),
    "ext_psi":   ("EKF_EXT_ATT_NSE", None),
}

# Mirror of INS.c::ekf_load_defaults().  Used when --current is not given.
DEFAULTS = {
    "EKF_MAG_NSE":     0.05,
    "EKF_GPS_POS_NSE": 0.5,
    "EKF_GPS_VEL_NSE": 0.3,
    "EKF_GPS_ALT_NSE": 1.5,
    "EKF_BARO_NSE":    2.0,
    "EKF_RF_NSE":      0.1,
    "EKF_OPF_NSE":     0.2,
    "EKF_EXT_POS_NSE": 0.05,
    "EKF_EXT_ATT_NSE": 0.05,

    "EKF_MAG_GATE":  5.0,
    "EKF_GPS_GATE":  5.0,
    "EKF_BARO_GATE": 5.0,
    "EKF_RF_GATE":   5.0,
    "EKF_OPF_GATE":  5.0,
}


# --------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------
def read_csv_lc(path):
    cols = defaultdict(list)
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise SystemExit(f"{path}: empty CSV")
        for row in reader:
            for k, v in row.items():
                key = k.lower() if k else k
                try:
                    cols[key].append(float(v))
                except (TypeError, ValueError):
                    cols[key].append(v)
    return cols


def tag_from_id(tid):
    try:
        i = int(float(tid))
    except (TypeError, ValueError):
        return "?"
    return TAG_TABLE[i] if 0 <= i < len(TAG_TABLE) else "?"


def load_current(path):
    """Parse YAML-ish 'KEY: VALUE' or 'KEY = VALUE' lines.  Avoids a
    PyYAML dependency so the tool runs in any Python install."""
    out = dict(DEFAULTS)
    if path is None or not path.exists():
        return out
    with open(path, "r") as f:
        for line in f:
            s = line.split("#", 1)[0].strip()
            if not s: continue
            for sep in (":", "="):
                if sep in s:
                    k, _, v = s.partition(sep)
                    k = k.strip()
                    v = v.strip().rstrip(",")
                    try:
                        out[k] = float(v)
                    except ValueError:
                        pass
                    break
    return out


def aggregate(rows):
    """Group innov samples by tag and compute the per-tag stats."""
    tags = (rows["tag"] if "tag" in rows
            else [tag_from_id(t) for t in rows.get("tag_id", [])])
    if not tags:
        raise SystemExit("innov CSV: no 'tag' or 'tag_id' column")

    nis_col = rows.get("nis")
    if not nis_col and "innov" in rows and "s" in rows:
        nis_col = [(i * i / s if s and s > 0 else 0.0)
                   for i, s in zip(rows["innov"], rows["s"])]
    if not nis_col:
        raise SystemExit("innov CSV: need 'nis' or both 'innov' and 'S'")

    accepted = rows.get("accepted", [1] * len(tags))

    buckets = defaultdict(lambda: {"nis": [], "accepted": 0, "total": 0})
    for t, n, a in zip(tags, nis_col, accepted):
        if t == "?": continue
        b = buckets[str(t)]
        b["nis"].append(float(n))
        b["accepted"] += 1 if int(a) else 0
        b["total"]    += 1

    out = {}
    for t, b in sorted(buckets.items()):
        if b["total"] == 0: continue
        nis = b["nis"]
        n = len(nis)
        mean_nis = sum(nis) / n
        # crude p95 without numpy
        nis_sorted = sorted(nis)
        p95 = nis_sorted[max(0, int(0.95 * n) - 1)]
        rej_pct = 100.0 * (b["total"] - b["accepted"]) / b["total"]
        out[t] = (n, mean_nis, p95, rej_pct)
    return out


def propose(stats, current, max_step, target_rej_pct):
    """Return list of (tag, param, gate, rec) where rec carries the
    suggested new sigma / gate, the change factor and a status note."""
    # Aggregate per-parameter mean NIS (tags pooled).
    per_param = defaultdict(lambda: {"sum_n_nis": 0.0, "sum_n": 0,
                                     "rej_n": 0, "tags": []})
    for tag, (n, mean_nis, p95, rej) in stats.items():
        param, gate_name = TAG_TO_PARAM.get(tag, (None, None))
        if param is None: continue
        bucket = per_param[(param, gate_name)]
        bucket["sum_n_nis"] += n * mean_nis
        bucket["sum_n"]     += n
        bucket["rej_n"]     += int(round(n * rej / 100.0))
        bucket["tags"].append(tag)

    proposals = []
    for (param, gate_name), b in per_param.items():
        if b["sum_n"] == 0: continue
        mean_nis = b["sum_n_nis"] / b["sum_n"]
        rej_pct  = 100.0 * b["rej_n"] / b["sum_n"]
        sigma_old = float(current.get(param, DEFAULTS.get(param, 1.0)))
        # tuning law sigma <- sigma * sqrt(mean_NIS), clamped
        scale = math.sqrt(max(mean_nis, 1e-6))
        scale = max(1.0 / max_step, min(max_step, scale))
        sigma_new = sigma_old * scale
        # Gate suggestion (only if a gate parameter is associated).
        gate_old = float(current.get(gate_name, 5.0)) if gate_name else None
        gate_new = gate_old
        gate_note = "-"
        if gate_name and rej_pct > target_rej_pct:
            if 0.5 <= mean_nis <= 5.0:
                gate_new = round(min(gate_old * 1.3, 10.0), 2)
                gate_note = f"raise (rej {rej_pct:.1f}% > {target_rej_pct}%, NIS healthy)"
            else:
                gate_note = f"hold (rej caused by NIS={mean_nis:.2f}; R fix first)"
        elif gate_name:
            gate_note = "ok"

        proposals.append({
            "tags":      ", ".join(sorted(b["tags"])),
            "param":     param,
            "gate":      gate_name,
            "n":         b["sum_n"],
            "mean_nis":  mean_nis,
            "rej_pct":   rej_pct,
            "sigma_old": sigma_old,
            "sigma_new": sigma_new,
            "scale":     scale,
            "gate_old":  gate_old,
            "gate_new":  gate_new,
            "gate_note": gate_note,
        })
    return proposals


# --------------------------------------------------------------------
# Output
# --------------------------------------------------------------------
def render_table(proposals):
    rows = [["tags", "param", "n", "<NIS>", "rej%",
             "sigma", "->sigma", "x", "gate", "gate_note"]]
    for p in proposals:
        rows.append([
            p["tags"],
            p["param"],
            f"{p['n']}",
            f"{p['mean_nis']:.3f}",
            f"{p['rej_pct']:.1f}",
            f"{p['sigma_old']:.4f}",
            f"{p['sigma_new']:.4f}",
            f"{p['scale']:.2f}",
            (f"{p['gate_old']:.1f}->{p['gate_new']:.1f}"
             if p['gate_old'] is not None else "-"),
            p["gate_note"],
        ])
    widths = [max(len(r[i]) for r in rows) for i in range(len(rows[0]))]
    out = []
    for ri, r in enumerate(rows):
        out.append("  ".join(c.ljust(widths[i]) for i, c in enumerate(r)))
        if ri == 0:
            out.append("  ".join("-" * w for w in widths))
    return "\n".join(out)


def render_shell(proposals):
    lines = ["# tune.py - paste into the FMT shell.",
             "# Re-runnable: each line is idempotent."]
    seen = set()
    for p in proposals:
        if p["param"] not in seen:
            lines.append(f"param set {p['param']} {p['sigma_new']:.6f}")
            seen.add(p["param"])
        if (p["gate"] is not None and p["gate"] not in seen
                and p["gate_old"] is not None and p["gate_new"] != p["gate_old"]):
            lines.append(f"param set {p['gate']} {p['gate_new']:.2f}")
            seen.add(p["gate"])
    return "\n".join(lines) + "\n"


# --------------------------------------------------------------------
# Main
# --------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="EKF auto-tuner from innov log")
    ap.add_argument("--innov-csv",       required=True, type=Path)
    ap.add_argument("--current",         type=Path, default=None,
                    help="current parameter values (key=value), defaults if omitted")
    ap.add_argument("--shell-out",       type=Path, default=None,
                    help="write FMT shell `param set` lines here")
    ap.add_argument("--report",          type=Path, default=None,
                    help="write the text report here as well")
    ap.add_argument("--max-step",        type=float, default=3.0,
                    help="cap sigma change per pass at +/- this factor")
    ap.add_argument("--target-rej-pct",  type=float, default=5.0,
                    help="acceptable innovation-gate rejection rate (%%)")
    args = ap.parse_args()

    rows = read_csv_lc(args.innov_csv)
    stats = aggregate(rows)
    if not stats:
        print("no usable innovation samples found", file=sys.stderr)
        return 1

    current = load_current(args.current)
    proposals = propose(stats, current, args.max_step, args.target_rej_pct)

    print("Per-tag NIS / rejection")
    print(f"  {'tag':<12} {'n':>6}  {'<NIS>':>8}  {'p95':>8}  {'rej%':>6}")
    print(f"  {'-'*12} {'-'*6}  {'-'*8}  {'-'*8}  {'-'*6}")
    for tag in sorted(stats):
        n, m, p, r = stats[tag]
        print(f"  {tag:<12} {n:>6}  {m:>8.3f}  {p:>8.3f}  {r:>5.1f}%")
    print()
    print("Suggested updates (sigma_new = sigma_old * sqrt(mean_NIS), capped)")
    print(render_table(proposals))

    if args.shell_out:
        args.shell_out.write_text(render_shell(proposals))
        print(f"\nshell script written to {args.shell_out}")

    if args.report:
        with open(args.report, "w") as f:
            for tag in sorted(stats):
                n, m, p, r = stats[tag]
                f.write(f"{tag:<12} n={n:>6} <NIS>={m:.3f} p95={p:.3f} rej={r:.1f}%\n")
            f.write("\n")
            f.write(render_table(proposals))
            f.write("\n\n")
            f.write(render_shell(proposals))
        print(f"report written to {args.report}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
