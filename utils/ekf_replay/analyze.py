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
    # ---- IMU vibration / quality rules (Phase 9) ----
    gyr_hf_max     = 0.10,   # rad/s high-frequency RMS
    acc_hf_max     = 2.0,    # m/s^2 high-frequency RMS
    accel_norm_eps = 0.30,   # |mean(|a|) - g| tolerance, m/s^2
    accel_static_var_max = 0.30,  # best 1-sec window variance for static check
    mag_norm_cv_pct  = 5.0,  # mag magnitude coefficient of variation
    # ---- IMU spike / GPS / filter-dead rules (Phase 10) ----
    imu_spike_g    = 1.5,    # rad/s, single-sample gyro outlier (3-pt median)
    imu_spike_a    = 30.0,   # m/s^2, single-sample accel outlier (~3 g)
    imu_spike_pct  = 0.05,   # warn if > pct of samples are spikes
    gps_min_sv     = 6,      # warn if any sample has fewer SV
    gps_max_hacc_m = 5.0,    # warn if hAcc above this in metres
    gps_drop_warn  = 1,      # warn if any fixType regression
    filter_dead_rel = 0.001, # sigma range / max(sigma) below this -> dead
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
# Sensor quality rules
#   IMU      vibration (HF residual after low-pass) + gravity sanity
#   MAG      field magnitude constancy (ferromagnetic interference)
# ------------------------------------------------------------------
def _high_freq_rms(values, alpha=0.05):
    """RMS of a first-order high-pass residual; alpha approx fc / fs."""
    if not values: return 0.0
    lp = values[0]
    acc = 0.0
    for v in values:
        lp += alpha * (v - lp)
        d = v - lp
        acc += d * d
    return math.sqrt(acc / len(values))


def analyse_imu(path, opts, rep):
    cols = read_csv(path)
    n = len(cols.get("timestamp", []))
    if n == 0:
        rep.add("error", "IMU", f"{path} has no rows")
        return

    summary = [["rows", n]]
    if "timestamp" in cols:
        summary.append(["t span [s]", f"{(cols['timestamp'][-1]-cols['timestamp'][0])/1000.0:.1f}"])

    # high-frequency RMS per axis
    for axis, thresh in (("gyr_x", opts.gyr_hf_max), ("gyr_y", opts.gyr_hf_max),
                        ("gyr_z", opts.gyr_hf_max),
                        ("acc_x", opts.acc_hf_max), ("acc_y", opts.acc_hf_max),
                        ("acc_z", opts.acc_hf_max)):
        if axis not in cols: continue
        hf = _high_freq_rms(cols[axis])
        unit = "rad/s" if axis.startswith("gyr") else "m/s^2"
        summary.append([f"HF-RMS {axis} [{unit}]", f"{hf:.4f}"])
        if hf > thresh:
            rep.add("warn", "IMU",
                    f"{axis} HF-RMS {hf:.3f} {unit} exceeds {thresh:.3f} - vibration too high")

    # gravity / accel magnitude check
    if all(k in cols for k in ("acc_x", "acc_y", "acc_z")):
        mags = [math.sqrt(cols['acc_x'][i]**2 + cols['acc_y'][i]**2 + cols['acc_z'][i]**2)
                for i in range(n)]
        mean_mag = sum(mags) / n
        summary.append(["accel norm mean [m/s^2]", f"{mean_mag:.4f}"])
        if abs(mean_mag - 9.80665) > opts.accel_norm_eps:
            rep.add("warn", "IMU",
                    f"long-run accel-norm mean {mean_mag:.3f} m/s^2 (g=9.806); "
                    f"calibration / mounting issue?")

        # find best static 1-second window
        rate_hz = 500
        if n > 1 and "timestamp" in cols:
            dt = (cols['timestamp'][-1]-cols['timestamp'][0]) / max(n-1, 1)
            if dt > 0: rate_hz = max(1, int(round(1000.0 / dt)))
        win = min(rate_hz, n)
        if win >= 50:
            best_var = float("inf")
            best_mean = 0.0
            best_t = 0
            step = max(1, win // 4)
            for i in range(0, n - win, step):
                seg = mags[i:i + win]
                m = sum(seg) / win
                v = sum((s - m)**2 for s in seg) / win
                if v < best_var:
                    best_var, best_mean = v, m
                    best_t = cols['timestamp'][i] if 'timestamp' in cols else i
            summary.append(["best static window var [m^2/s^4]", f"{best_var:.4f}"])
            if best_var > opts.accel_static_var_max:
                rep.add("info", "IMU",
                        f"no calm 1-second window found (best var {best_var:.3f}); "
                        f"vehicle never settles?")
            elif abs(best_mean - 9.80665) > opts.accel_norm_eps:
                rep.add("warn", "IMU",
                        f"static window @ t={best_t} ms: |a|={best_mean:.3f} m/s^2 "
                        f"deviates from g - accel scale / cal?")

    # ---- single-sample spike detection (3-point median residual) ----
    for axis in ("gyr_x", "gyr_y", "gyr_z", "acc_x", "acc_y", "acc_z"):
        if axis not in cols: continue
        d = cols[axis]
        thresh = opts.imu_spike_g if axis.startswith("gyr") else opts.imu_spike_a
        spikes = 0
        worst = 0.0
        worst_t = 0
        for i in range(1, len(d) - 1):
            ref = 0.5 * (d[i - 1] + d[i + 1])
            r = abs(d[i] - ref)
            if r > thresh:
                spikes += 1
                if r > worst:
                    worst = r
                    worst_t = cols['timestamp'][i] if 'timestamp' in cols else i
        if spikes > 0:
            pct = 100.0 * spikes / max(1, len(d) - 2)
            unit = "rad/s" if axis.startswith("gyr") else "m/s^2"
            sev = "warn" if pct > 100.0 * opts.imu_spike_pct else "info"
            rep.add(sev, "IMU",
                    f"{axis}: {spikes} single-sample spikes "
                    f"({pct:.2f}%), worst {worst:.2f} {unit} @ t={worst_t} ms")

    rep.add_table("IMU summary", ["metric", "value"], summary)


def analyse_mag(path, opts, rep):
    cols = read_csv(path)
    n = len(cols.get("timestamp", []))
    if n == 0:
        rep.add("error", "MAG", f"{path} has no rows")
        return
    if not all(k in cols for k in ("mag_x", "mag_y", "mag_z")):
        rep.add("info", "MAG", "mag CSV missing one of mag_x/mag_y/mag_z")
        return

    norms = [math.sqrt(cols['mag_x'][i]**2 + cols['mag_y'][i]**2 + cols['mag_z'][i]**2)
             for i in range(n)]
    m = sum(norms) / n
    var = sum((x - m)**2 for x in norms) / n
    sd = math.sqrt(var)
    cv = (sd / m * 100.0) if m > 1e-9 else 0.0
    rep.add_table("MAG summary", ["metric", "value"], [
        ["rows", n],
        ["t span [s]", f"{(cols['timestamp'][-1]-cols['timestamp'][0])/1000.0:.1f}"],
        ["mag norm mean", f"{m:.4f}"],
        ["mag norm sigma", f"{sd:.4f}"],
        ["coefficient of variation [%]", f"{cv:.2f}"],
    ])
    if cv > opts.mag_norm_cv_pct:
        rep.add("warn", "MAG",
                f"mag norm CV {cv:.1f}% > {opts.mag_norm_cv_pct}% - "
                f"electromagnetic interference or hard-iron error?")


# ------------------------------------------------------------------
# GPS quality / RTK degradation
#   fixType:  0 no fix, 1 dead reckoning, 2 2D, 3 3D, 4 GNSS+DR,
#             5 time only, 6 RTK float, 7 RTK fixed (vendor-specific
#             above 3 - we only treat <3 as 'no usable fix')
#   numSV:    satellites in use
#   hAcc / vAcc / sAcc: uBlox reports millimetres / mm/s
# ------------------------------------------------------------------
def analyse_gps(path, opts, rep):
    cols = read_csv(path)
    n = len(cols.get("timestamp", []))
    if n == 0:
        rep.add("error", "GPS", f"{path} has no rows")
        return

    fix = [int(v) for v in cols.get("fixtype", [])]
    sv  = [int(v) for v in cols.get("numsv",   [])]
    if not fix or not sv:
        rep.add("info", "GPS", "GPS CSV missing fixType / numSV")
        return

    # fix-type histogram
    hist = {}
    for f in fix: hist[f] = hist.get(f, 0) + 1
    summary = [["rows", n],
               ["t span [s]", f"{(cols['timestamp'][-1]-cols['timestamp'][0])/1000.0:.1f}"]]
    for k in sorted(hist.keys()):
        summary.append([f"fixType=={k} samples", f"{hist[k]} ({100.0*hist[k]/n:.1f}%)"])

    # regressions (fixType decreased between consecutive samples)
    drops = sum(1 for i in range(1, n) if fix[i] < fix[i - 1])
    summary.append(["fixType regressions", drops])

    # numSV stats
    summary.append(["numSV mean / min", f"{sum(sv)/n:.1f} / {min(sv)}"])

    # accuracy stats (uBlox: millimetres / mm/s)
    if "hacc" in cols:
        h_max_m = max(cols["hacc"]) / 1000.0
        h_med_m = sorted(cols["hacc"])[n // 2] / 1000.0
        summary.append(["hAcc median / max [m]", f"{h_med_m:.2f} / {h_max_m:.2f}"])
    if "vacc" in cols:
        v_max_m = max(cols["vacc"]) / 1000.0
        summary.append(["vAcc max [m]", f"{v_max_m:.2f}"])

    rep.add_table("GPS summary", ["metric", "value"], summary)

    # ---- findings ----
    final_fix = fix[-1]
    if final_fix < 3:
        rep.add("warn", "GPS", f"final fixType={final_fix} - 3D fix not held to end")
    no_fix_pct = 100.0 * sum(1 for f in fix if f < 3) / n
    if no_fix_pct > 5.0:
        rep.add("warn", "GPS",
                f"{no_fix_pct:.1f}% of samples have fixType<3 (no 3D fix)")
    if drops >= opts.gps_drop_warn and drops > 0:
        # find the first drop and report it
        first_t = next((cols['timestamp'][i] for i in range(1, n)
                        if fix[i] < fix[i - 1]), 0)
        rep.add("warn" if drops > 5 else "info", "GPS",
                f"{drops} fixType regressions; first @ t={first_t} ms")
    if min(sv) < opts.gps_min_sv:
        first_lo = next((cols['timestamp'][i] for i in range(n)
                         if sv[i] < opts.gps_min_sv), 0)
        rep.add("warn", "GPS",
                f"numSV dropped to {min(sv)} (< {opts.gps_min_sv}) "
                f"@ t={first_lo} ms - poor satellite geometry")
    if "hacc" in cols and max(cols["hacc"]) / 1000.0 > opts.gps_max_hacc_m:
        rep.add("info", "GPS",
                f"max hAcc {max(cols['hacc'])/1000.0:.2f} m exceeds "
                f"{opts.gps_max_hacc_m} m - intermittent quality drop")

    # RTK regression: 6/7 -> <=4 means lost RTK lock
    rtk_lost = 0
    for i in range(1, n):
        if fix[i - 1] >= 6 and fix[i] <= 4:
            rtk_lost += 1
    if rtk_lost > 0:
        rep.add("warn", "GPS", f"RTK lock lost {rtk_lost} time(s) (fixType 6/7 -> <=4)")


# ------------------------------------------------------------------
# Filter-dead detector - sigma trace flat lines
#   The EKF must keep moving the covariance every step (predict adds Q,
#   updates subtract).  If a sigma channel never changes for the full
#   recording, the corresponding subsystem is starved of measurements
#   AND not propagating either - i.e. the filter task is hung.
# ------------------------------------------------------------------
def detect_filter_dead(state_csv, opts, rep):
    cols = read_csv(state_csv)
    n = len(cols.get("timestamp", []))
    if n < 10:
        return

    sigma_axes = (
        "sigma_pos_n", "sigma_pos_e", "sigma_pos_d",
        "sigma_vel_n", "sigma_vel_e", "sigma_vel_d",
        "sigma_att_x", "sigma_att_y", "sigma_att_z",
    )
    flat = []
    for ax in sigma_axes:
        if ax not in cols: continue
        d = cols[ax]
        if not d: continue
        peak = max(abs(v) for v in d)
        if peak < 1e-12:
            flat.append((ax, 0.0))
            continue
        rng = max(d) - min(d)
        rel = rng / peak
        if rel < opts.filter_dead_rel:
            flat.append((ax, rel))
    if flat:
        joined = ", ".join(f"{ax} (rel range {r*100:.3f}%)" for ax, r in flat)
        rep.add("warn", "DEAD",
                f"sigma channels did not change over {n} samples: {joined} "
                f"- filter possibly stuck or starved of measurements")


# ------------------------------------------------------------------
# Plotting (matplotlib optional)
# ------------------------------------------------------------------
def _import_pyplot():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        return plt
    except ImportError:
        return None


def make_plots(args, plot_dir, rep):
    plt = _import_pyplot()
    if plt is None:
        rep.add("info", "PLOT", "matplotlib not available; skipping plots")
        return
    plot_dir.mkdir(parents=True, exist_ok=True)
    written = []

    # ---- NIS histograms per tag ----
    if args.innov_csv is not None and args.innov_csv.exists():
        cols = read_csv(args.innov_csv)
        tags = (cols["tag"] if "tag" in cols
                else [tag_from_id(t) for t in cols.get("tag_id", [])])
        nis = cols.get("nis", None)
        if nis is None and "innov" in cols and "s" in cols:
            nis = [(i*i/s if s and s > 0 else 0.0)
                   for i, s in zip(cols["innov"], cols["s"])]
        if tags and nis:
            buckets = {}
            for t, v in zip(tags, nis):
                buckets.setdefault(str(t), []).append(float(v))
            for tag, vs in sorted(buckets.items()):
                if not vs: continue
                fig, ax = plt.subplots(figsize=(6, 3))
                clipped = [min(v, 50.0) for v in vs]
                ax.hist(clipped, bins=40, edgecolor="black", linewidth=0.3)
                ax.axvline(3.84, color="orange", linestyle="--", label="chi^2(1,0.95)=3.84")
                ax.axvline(1.0,  color="green",  linestyle=":",  label="ideal mean=1")
                ax.set_xlabel("NIS (clipped at 50)")
                ax.set_ylabel("count")
                ax.set_title(f"NIS distribution -- {tag}  (n={len(vs)})")
                ax.legend(fontsize=7)
                ax.grid(True, linewidth=0.3)
                fig.tight_layout()
                p = plot_dir / f"nis_{tag}.png"
                fig.savefig(p, dpi=120); plt.close(fig)
                written.append(p)

    # ---- INS_State sigma + bias time-series ----
    if args.state_csv is not None and args.state_csv.exists():
        cols = read_csv(args.state_csv)
        ts = [t / 1000.0 for t in cols.get("timestamp", [])]

        groups = [
            ("sigma_attitude.png", "attitude sigma [rad]",
             [("sigma_att_x", "x"), ("sigma_att_y", "y"), ("sigma_att_z", "z")]),
            ("sigma_velocity.png", "velocity sigma [m/s]",
             [("sigma_vel_n", "N"), ("sigma_vel_e", "E"), ("sigma_vel_d", "D")]),
            ("sigma_position.png", "position sigma [m]",
             [("sigma_pos_n", "N"), ("sigma_pos_e", "E"), ("sigma_pos_d", "D")]),
            ("bias_gyro.png",      "gyro bias [rad/s]",
             [("bg_x", "x"), ("bg_y", "y"), ("bg_z", "z")]),
            ("bias_accel.png",     "accel bias [m/s^2]",
             [("ba_x", "x"), ("ba_y", "y"), ("ba_z", "z")]),
            ("baro_terr.png",      "baro_b / terr_d [m]",
             [("baro_b", "baro_b"), ("terr_d", "terr_d")]),
        ]
        for fname, ylabel, series in groups:
            present = [(k, lbl) for k, lbl in series if k in cols]
            if not present: continue
            fig, ax = plt.subplots(figsize=(7, 3))
            for k, lbl in present:
                ax.plot(ts, cols[k], label=lbl, linewidth=0.7)
            ax.set_xlabel("t [s]")
            ax.set_ylabel(ylabel)
            ax.legend(fontsize=8, loc="best")
            ax.grid(True, linewidth=0.3)
            fig.tight_layout()
            p = plot_dir / fname
            fig.savefig(p, dpi=120); plt.close(fig)
            written.append(p)

    # ---- INS_Out attitude + velocity overview ----
    if args.ins_out is not None and args.ins_out.exists():
        cols = read_csv(args.ins_out)
        ts = [t / 1000.0 for t in cols.get("timestamp", [])]
        for fname, ylabel, series in [
            ("ins_attitude.png", "attitude [rad]",
             [("phi", "phi"), ("theta", "theta"), ("psi", "psi")]),
            ("ins_velocity.png", "velocity [m/s]",
             [("vn", "N"), ("ve", "E"), ("vd", "D")]),
            ("ins_position.png", "position [m]",
             [("x_r", "x_R"), ("y_r", "y_R"), ("h_r", "h_R")]),
        ]:
            present = [(k, lbl) for k, lbl in series if k in cols]
            if not present: continue
            fig, ax = plt.subplots(figsize=(7, 3))
            for k, lbl in present:
                ax.plot(ts, cols[k], label=lbl, linewidth=0.7)
            ax.set_xlabel("t [s]")
            ax.set_ylabel(ylabel)
            ax.legend(fontsize=8); ax.grid(True, linewidth=0.3)
            fig.tight_layout()
            p = plot_dir / fname
            fig.savefig(p, dpi=120); plt.close(fig)
            written.append(p)

    rep.add("info", "PLOT", f"wrote {len(written)} plot(s) to {plot_dir}")


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
    ap.add_argument("--imu-csv",    type=Path, default=None,
                    help="parsed-mlog IMU CSV (vibration + gravity sanity)")
    ap.add_argument("--mag-csv",    type=Path, default=None,
                    help="parsed-mlog MAG CSV (field-norm constancy check)")
    ap.add_argument("--gps-csv",    type=Path, default=None,
                    help="parsed-mlog GPS_uBlox CSV (RTK / fix / numSV check)")
    ap.add_argument("--reference",  type=Path, default=None,
                    help="recorded INS_Out CSV from the same flight")
    ap.add_argument("--report",     type=Path, default=None,
                    help="write the textual report here as well")
    ap.add_argument("--plot-dir",   type=Path, default=None,
                    help="directory to write PNG plots (NIS hist + sigma / "
                         "bias time-series + INS_Out overview).  Requires "
                         "matplotlib; silently skipped if missing.")
    for k, v in DEFAULTS.items():
        ap.add_argument(f"--{k.replace('_','-')}", dest=k, type=float, default=v)
    args = ap.parse_args()

    rep = Report()
    if args.innov_csv is not None:
        analyse_innov(args.innov_csv, args, rep)
    analyse_ins_out(args.ins_out, args, rep)
    if args.state_csv is not None:
        analyse_state(args.state_csv, args, rep)
    if args.imu_csv is not None:
        analyse_imu(args.imu_csv, args, rep)
    if args.mag_csv is not None:
        analyse_mag(args.mag_csv, args, rep)
    if args.gps_csv is not None:
        analyse_gps(args.gps_csv, args, rep)
    if args.state_csv is not None:
        detect_filter_dead(args.state_csv, args, rep)
    if args.reference is not None:
        analyse_reference(args.reference, args.ins_out, args, rep)
    if args.plot_dir is not None:
        make_plots(args, args.plot_dir, rep)

    rep.emit()
    if args.report is not None:
        with open(args.report, "w") as f:
            rep.emit(f)
        print(f"\nreport written to {args.report}", file=sys.stderr)
    return 0 if not any(s == "error" for s, _, _ in rep.findings) else 1


if __name__ == "__main__":
    sys.exit(main())
