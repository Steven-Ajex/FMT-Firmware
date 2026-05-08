#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# analyze.py
#
# Diagnostic post-processor for an ekf_ins replay run.  Reads:
#
#   innov.csv  (timestamp, tag, innov, R, S, nis, accepted)
#       written by `ekf_replay --innov-csv ...`.  Records every scalar
#       measurement update with its innovation, predicted variance and
#       whether the gate accepted it.
#
#   INS_Out.csv  (the EKF output bus)
#       used to flag attitude / position / quaternion anomalies that
#       are independent of the measurement layer.
#
#   reference INS_Out.csv  (optional, e.g. cf_ins log)
#       used to bound the per-channel drift relative to a known-good
#       reference run on the same flight.
#
# What it tells you (each problem produces a "FINDING:" line):
#
#   NIS    Mean / 95-percentile of innovation^2/S per measurement tag.
#          Mean ~ 1.0 when R is well tuned.  > 5  -> R is too small or
#          a systematic bias is leaking in.  < 0.1 -> R is too big.
#
#   GATE   Fraction of innovations rejected by the EKF.  >5 % is
#          suspicious.  Sustained streaks (default >= 10 in a row)
#          are reported individually with start / end timestamps.
#
#   JUMP   Frame-to-frame attitude / velocity / position jumps that
#          exceed configurable thresholds.  Helps spot reset events
#          and integration breakdowns.
#
#   QNORM  Quaternion norm drift (1 +/- ...).  Ought to be < 1e-3.
#
#   COV    Detected via the INS_Out flag bits going off after a sensor
#          timeout (when run with --reference).
#
#   DIFF   When --reference is provided, channels whose RMS error
#          exceeds a per-channel sanity threshold.

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path


# ------------------------------------------------------------------
# Defaults / thresholds.  Tuned for typical UAV flights; override on
# the command line for vehicles outside the envelope.
# ------------------------------------------------------------------
DEFAULTS = dict(
    nis_high       = 5.0,    # mean NIS above which R looks too tight
    nis_low        = 0.1,    # below which R looks too loose
    nis_exc_pct    = 5.0,    # > pct samples above chi^2(1, 0.95)=3.84
    gate_pct       = 5.0,    # > pct rejected
    streak_len     = 10,     # consecutive rejected samples to flag
    jump_phi       = 0.087,  # 5 deg in rad
    jump_psi       = 0.175,  # 10 deg in rad
    jump_v         = 5.0,    # m/s
    jump_p         = 20.0,   # m
    qnorm_eps      = 1e-3,
    diff_phi       = 0.087,  # 5 deg
    diff_psi       = 0.175,  # 10 deg
    diff_v         = 1.0,    # m/s
    diff_p         = 5.0,    # m
    bg_max         = 0.05,   # rad/s gyro bias magnitude warning
    ba_max         = 0.8,    # m/s^2 accel bias magnitude warning
    baro_b_max     = 50.0,   # baro bias magnitude warning [m]
    sigma_pos_max  = 20.0,   # sustained position sigma divergence
    sigma_vel_max  = 5.0,    # sustained velocity sigma divergence
    sigma_att_max  = 0.3,    # sustained attitude sigma divergence
)

CHI2_95_DOF1 = 3.84


# ------------------------------------------------------------------
# CSV helpers
# ------------------------------------------------------------------
def read_csv(path):
    """Return dict[col_lc] -> list of values.  Header names are lowercased
    so callers do not have to care whether the column is 'S' (offline
    replay output) or 's' (parsed from mlog binary)."""
    cols = defaultdict(list)
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise SystemExit(f"{path}: empty CSV")
        for row in reader:
            for k, v in row.items():
                key = k.lower() if k is not None else k
                try:
                    cols[key].append(float(v))
                except (TypeError, ValueError):
                    cols[key].append(v)
    return cols


def percentile(values, pct):
    finite = sorted(v for v in values if isinstance(v, float)
                    and not math.isnan(v) and not math.isinf(v))
    if not finite:
        return float("nan")
    k = (len(finite) - 1) * pct / 100.0
    lo = int(math.floor(k))
    hi = int(math.ceil(k))
    if lo == hi:
        return finite[lo]
    return finite[lo] + (finite[hi] - finite[lo]) * (k - lo)


def mean(values):
    finite = [v for v in values if isinstance(v, float)
              and not math.isnan(v) and not math.isinf(v)]
    return (sum(finite) / len(finite)) if finite else float("nan")


def rms(values):
    finite = [v for v in values if isinstance(v, float)
              and not math.isnan(v) and not math.isinf(v)]
    return math.sqrt(sum(v * v for v in finite) / len(finite)) if finite else float("nan")


# ------------------------------------------------------------------
# Findings
# ------------------------------------------------------------------
class Report:
    def __init__(self):
        self.findings = []   # list of (severity, kind, msg)
        self.tables   = []   # list of (title, rows)

    def add(self, severity, kind, msg):
        self.findings.append((severity, kind, msg))

    def add_table(self, title, header, rows):
        self.tables.append((title, header, rows))

    def emit(self, fp=sys.stdout):
        for title, header, rows in self.tables:
            print(title, file=fp)
            widths = [len(h) for h in header]
            for r in rows:
                for i, v in enumerate(r):
                    widths[i] = max(widths[i], len(str(v)))
            fmt = "  " + "  ".join(f"{{:<{w}}}" for w in widths)
            print(fmt.format(*header), file=fp)
            print("  " + "  ".join("-" * w for w in widths), file=fp)
            for r in rows:
                print(fmt.format(*[str(x) for x in r]), file=fp)
            print(file=fp)
        if not self.findings:
            print("FINDING: none -- INS data looks healthy", file=fp)
            return
        for sev, kind, msg in self.findings:
            print(f"FINDING [{sev}] {kind}: {msg}", file=fp)


# ------------------------------------------------------------------
# tag_id <-> string mapping.  Mirrors the order in
# src/model/ins/ekf_ins/lib/ekf_core.c (k_innov_tags); update both in
# lockstep when adding a new measurement.
# ------------------------------------------------------------------
TAG_TABLE = [
    "mag", "grav_x", "grav_y",
    "gps_pos_n", "gps_pos_e", "gps_pos_d",
    "gps_vel_n", "gps_vel_e", "gps_vel_d",
    "baro", "rf",
    "opf_x", "opf_y",
    "ext_x", "ext_y", "ext_z",
    "ext_phi", "ext_theta", "ext_psi",
]


def tag_from_id(tid):
    try:
        i = int(tid)
    except (TypeError, ValueError):
        return "?"
    if 0 <= i < len(TAG_TABLE):
        return TAG_TABLE[i]
    return "?"


# ------------------------------------------------------------------
# Innovation / gate analysis
# ------------------------------------------------------------------
def analyse_innov(path, opts, rep):
    rows = read_csv(path)
    if "tag" in rows:
        tags = rows["tag"]
    elif "tag_id" in rows:
        tags = [tag_from_id(t) for t in rows["tag_id"]]
    else:
        rep.add("error", "innov", f"{path} has no 'tag' or 'tag_id' column")
        return
    nis  = rows.get("nis",       [0.0] * len(tags))
    acc  = rows.get("accepted",  [1]   * len(tags))
    ts   = rows.get("timestamp", [0]   * len(tags))

    # If reading parsed mlog (no NIS column), compute it from innov / S
    if "nis" not in rows and "innov" in rows and "s" in rows:
        nis = [(i*i/s if s and s > 0 else 0.0)
               for i, s in zip(rows["innov"], rows["s"])]

    by_tag = defaultdict(lambda: dict(nis=[], acc=[], ts=[]))
    for i, t in enumerate(tags):
        d = by_tag[t]
        d["nis"].append(nis[i])
        d["acc"].append(int(acc[i]))
        d["ts"].append(int(ts[i]))

    nis_table_rows = []
    for tag in sorted(by_tag.keys()):
        d = by_tag[tag]
        n = len(d["nis"])
        m = mean(d["nis"])
        p95 = percentile(d["nis"], 95)
        rej = sum(1 for a in d["acc"] if a == 0)
        rej_pct = 100.0 * rej / n if n else 0.0
        exc = sum(1 for v in d["nis"] if v > CHI2_95_DOF1)
        exc_pct = 100.0 * exc / n if n else 0.0
        nis_table_rows.append([tag, n, f"{m:.2f}", f"{p95:.2f}",
                               f"{exc_pct:.1f}%", f"{rej_pct:.1f}%"])

        if m > opts.nis_high:
            rep.add("warn", "NIS",
                    f"{tag}: mean NIS {m:.2f} (> {opts.nis_high}); "
                    "R may be too small or measurement biased")
        elif m < opts.nis_low and m == m:
            rep.add("info", "NIS",
                    f"{tag}: mean NIS {m:.2f} (< {opts.nis_low}); "
                    "R may be set too loose")
        if exc_pct > opts.nis_exc_pct:
            rep.add("warn", "NIS",
                    f"{tag}: {exc_pct:.1f}% of innovations exceed "
                    f"chi^2(1, 0.95) = 3.84")
        if rej_pct > opts.gate_pct:
            rep.add("warn", "GATE",
                    f"{tag}: {rej_pct:.1f}% of updates rejected by gate")

        # streak detection
        streak = 0
        streak_start = None
        worst = (0, None, None)
        for i, a in enumerate(d["acc"]):
            if a == 0:
                if streak == 0:
                    streak_start = d["ts"][i]
                streak += 1
                if streak > worst[0]:
                    worst = (streak, streak_start, d["ts"][i])
            else:
                streak = 0
        if worst[0] >= opts.streak_len:
            rep.add("warn", "GATE",
                    f"{tag}: longest rejection streak {worst[0]} samples "
                    f"@ {worst[1]} ms - {worst[2]} ms")

    rep.add_table("Per-tag innovation summary",
                  ["tag", "n", "mean_NIS", "p95_NIS", ">3.84", "rejected"],
                  nis_table_rows)


# ------------------------------------------------------------------
# INS_Out anomaly detection
# ------------------------------------------------------------------
def analyse_ins_out(path, opts, rep):
    cols = read_csv(path)
    needed = ["phi", "theta", "psi", "vn", "ve", "vd",
              "x_r", "y_r", "h_r",
              "quat0", "quat1", "quat2", "quat3"]
    miss = [c for c in needed if c not in cols]
    if miss:
        rep.add("info", "INS_Out", f"missing columns: {miss}")
    n = len(cols.get("timestamp", []))
    ts = cols.get("timestamp", [])

    def jump_check(name, thr, wrap=False):
        if name not in cols:
            return
        v = cols[name]
        worst = 0.0; worst_t = None
        for i in range(1, len(v)):
            d = v[i] - v[i-1]
            if wrap:
                while d >  math.pi: d -= 2*math.pi
                while d < -math.pi: d += 2*math.pi
            if abs(d) > worst:
                worst = abs(d); worst_t = ts[i] if i < len(ts) else None
        if worst > thr:
            rep.add("warn", "JUMP",
                    f"{name}: max single-step jump {worst:.4f} "
                    f"(> {thr}) @ {worst_t} ms")

    jump_check("phi",   opts.jump_phi)
    jump_check("theta", opts.jump_phi)
    jump_check("psi",   opts.jump_psi, wrap=True)
    for ax in ("vn", "ve", "vd"):  jump_check(ax, opts.jump_v)
    for ax in ("x_r", "y_r", "h_r"):  jump_check(ax, opts.jump_p)

    if all(c in cols for c in ("quat0", "quat1", "quat2", "quat3")):
        worst = 0.0; worst_t = None
        for i in range(n):
            qn = math.sqrt(sum(cols[c][i] ** 2 for c in ("quat0","quat1","quat2","quat3")))
            d  = abs(qn - 1.0)
            if d > worst:
                worst = d; worst_t = ts[i] if i < len(ts) else None
        if worst > opts.qnorm_eps:
            rep.add("warn", "QNORM",
                    f"quaternion norm drift {worst:.4e} "
                    f"(> {opts.qnorm_eps}) @ {worst_t} ms")

    rep.add_table("INS_Out summary", ["metric", "value"],
                  [["rows", n],
                   ["t span [s]", f"{(ts[-1]-ts[0])/1000.0:.1f}" if ts else "0"],
                   ["phi range [deg]",   f"{math.degrees(min(cols.get('phi', [0]))):.2f} .. {math.degrees(max(cols.get('phi', [0]))):.2f}"],
                   ["theta range [deg]", f"{math.degrees(min(cols.get('theta', [0]))):.2f} .. {math.degrees(max(cols.get('theta', [0]))):.2f}"],
                   ["psi range [deg]",   f"{math.degrees(min(cols.get('psi', [0]))):.2f} .. {math.degrees(max(cols.get('psi', [0]))):.2f}"],
                   ["|v| max [m/s]",     f"{max(math.sqrt(cols['vn'][i]**2+cols['ve'][i]**2+cols['vd'][i]**2) for i in range(n)) if n and 'vn' in cols else 0:.2f}"],
                   ["|p| max [m]",       f"{max(math.sqrt(cols['x_r'][i]**2+cols['y_r'][i]**2+cols['h_r'][i]**2) for i in range(n)) if n and 'x_r' in cols else 0:.2f}"]])


# ------------------------------------------------------------------
# INS_State analysis -- bias trend, sigma collapse / divergence
# ------------------------------------------------------------------
def analyse_state(path, opts, rep):
    cols = read_csv(path)
    n = len(cols.get("timestamp", []))
    if n == 0:
        rep.add("error", "STATE", f"{path} has no rows")
        return

    def vec_mag(name_x, name_y, name_z, idx):
        if not all(k in cols for k in (name_x, name_y, name_z)):
            return None
        return math.sqrt(cols[name_x][idx] ** 2
                       + cols[name_y][idx] ** 2
                       + cols[name_z][idx] ** 2)

    last = n - 1
    bg = vec_mag("bg_x", "bg_y", "bg_z", last)
    ba = vec_mag("ba_x", "ba_y", "ba_z", last)
    summary = [["rows", n]]
    if "timestamp" in cols and n:
        summary.append(["t span [s]", f"{(cols['timestamp'][-1]-cols['timestamp'][0])/1000.0:.1f}"])
    if bg is not None: summary.append(["|bg| final [rad/s]",  f"{bg:.5f}"])
    if ba is not None: summary.append(["|ba| final [m/s^2]",  f"{ba:.4f}"])
    if "baro_b" in cols and n: summary.append(["baro_b final [m]", f"{cols['baro_b'][-1]:.3f}"])
    if "terr_d" in cols and n: summary.append(["terr_d final [m]", f"{cols['terr_d'][-1]:.3f}"])
    rep.add_table("INS_State summary", ["metric", "value"], summary)

    # Bias drift checks
    if bg is not None and bg > opts.bg_max:
        rep.add("warn", "BIAS",
                f"|bg| final {bg:.5f} rad/s exceeds {opts.bg_max} -- gyro cal / temperature?")
    if ba is not None and ba > opts.ba_max:
        rep.add("warn", "BIAS",
                f"|ba| final {ba:.4f} m/s^2 exceeds {opts.ba_max} -- accel cal?")
    if "baro_b" in cols and n and abs(cols["baro_b"][-1]) > opts.baro_b_max:
        rep.add("info", "BIAS",
                f"baro_b final {cols['baro_b'][-1]:.1f} m -- big GPS/baro reference offset")

    # Sigma divergence: sustained > threshold over the last 25 % of the run
    def sigma_late_max(*names):
        if not all(name in cols for name in names): return None
        i_lo = int(0.75 * n)
        return max(max(cols[name][i:i+1] or [0.0]) for name in names
                   for i in range(i_lo, n)) if n - i_lo > 0 else 0.0

    def sigma_check(label, names, thr, kind):
        if not all(name in cols for name in names): return
        i_lo = int(0.75 * n)
        worst = 0.0
        for i in range(i_lo, n):
            for name in names:
                if cols[name][i] > worst:
                    worst = cols[name][i]
        if worst > thr:
            rep.add("warn", "COV",
                    f"{label} sigma late-window max {worst:.3f} > {thr} ({kind})")

    sigma_check("position", ("sigma_pos_n", "sigma_pos_e", "sigma_pos_d"),
                opts.sigma_pos_max, "filter losing position observability")
    sigma_check("velocity", ("sigma_vel_n", "sigma_vel_e", "sigma_vel_d"),
                opts.sigma_vel_max, "filter losing velocity observability")
    sigma_check("attitude", ("sigma_att_x", "sigma_att_y", "sigma_att_z"),
                opts.sigma_att_max, "filter losing attitude observability")


# ------------------------------------------------------------------
# Reference comparison
# ------------------------------------------------------------------
def analyse_reference(ref_path, replay_path, opts, rep):
    ref = read_csv(ref_path)
    rep_csv = read_csv(replay_path)
    common = [c for c in ("phi","theta","psi","vn","ve","vd","x_r","y_r","h_r")
              if c in ref and c in rep_csv]
    if "timestamp" not in ref or "timestamp" not in rep_csv:
        rep.add("info", "DIFF", "missing timestamp column; skipping align")
        return

    ts_ref = ref["timestamp"]
    ts_rep = rep_csv["timestamp"]
    if not ts_ref or not ts_rep:
        return

    j = 0
    diffs = {c: [] for c in common}
    for i, t in enumerate(ts_ref):
        while j + 1 < len(ts_rep) and abs(ts_rep[j+1] - t) < abs(ts_rep[j] - t):
            j += 1
        for c in common:
            diffs[c].append(rep_csv[c][j] - ref[c][i])

    rows = []
    for c in common:
        m = mean(diffs[c])
        r = rms(diffs[c])
        mx = max((abs(v) for v in diffs[c]), default=0.0)
        rows.append([c, f"{m:.4f}", f"{r:.4f}", f"{mx:.4f}"])

        thr = (opts.diff_phi if c in ("phi","theta") else
               opts.diff_psi if c == "psi" else
               opts.diff_v   if c in ("vn","ve","vd") else
               opts.diff_p)
        if r > thr:
            rep.add("warn", "DIFF",
                    f"{c}: RMS error {r:.3f} exceeds threshold {thr}")
    rep.add_table("Reference comparison (replay - reference)",
                  ["channel", "mean", "rms", "max-abs"], rows)


# ------------------------------------------------------------------
# Main
# ------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Diagnose ekf_ins replay output.")
    ap.add_argument("ins_out",      type=Path, help="INS_Out_replay.csv")
    ap.add_argument("--innov-csv",  type=Path, default=None,
                    help="innovation log (from ekf_replay --innov-csv or "
                         "the parsed-mlog INS_Innov CSV)")
    ap.add_argument("--state-csv",  type=Path, default=None,
                    help="parsed-mlog INS_State CSV (bias / sigma trends)")
    ap.add_argument("--reference",  type=Path, default=None,
                    help="recorded INS_Out CSV from the same flight")
    ap.add_argument("--report",     type=Path, default=None,
                    help="write the textual report here as well")
    for k, v in DEFAULTS.items():
        ap.add_argument(f"--{k.replace('_','-')}", dest=k, type=float, default=v)
    args = ap.parse_args()

    rep = Report()
    if args.innov_csv is not None:
        analyse_innov(args.innov_csv, args, rep)
    analyse_ins_out(args.ins_out, args, rep)
    if args.state_csv is not None:
        analyse_state(args.state_csv, args, rep)
    if args.reference is not None:
        analyse_reference(args.reference, args.ins_out, args, rep)

    rep.emit()
    if args.report is not None:
        with open(args.report, "w") as f:
            rep.emit(f)
        print(f"\nreport written to {args.report}", file=sys.stderr)
    return 0 if not any(s == "error" for s, _, _ in rep.findings) else 1


if __name__ == "__main__":
    sys.exit(main())
