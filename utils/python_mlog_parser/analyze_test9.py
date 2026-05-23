"""Triage ekf_test9 - flight with Path A+B fixes deployed.

Look at the post-maneuver recovery windows: each big rotation should be
followed by quick convergence back to accel-implied tilt (within a few
seconds), not the multi-minute divergence we saw in test8."""
import csv
import math
from pathlib import Path

HERE = Path(__file__).parent
OUT = HERE / "out"


def load_csv(name):
    rows = []
    with open(OUT / name, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def f(v):
    try: return float(v)
    except (TypeError, ValueError): return float("nan")


def main():
    ins = load_csv("mlog_msg_0_INS_Out.csv")
    imu = load_csv("mlog_msg_10_IMU.csv")

    # Index raw IMU per ms for ground-truth tilt
    imu_by_t = {int(f(r["timestamp"])): r for r in imu}

    print("===== Detect maneuver bursts (|gyro| > 30 deg/s) =====")
    bursts = []
    in_burst = False
    burst_start = 0
    burst_peak = 0
    for r in imu:
        gx, gy, gz = f(r["gyr_x"]), f(r["gyr_y"]), f(r["gyr_z"])
        rate = math.degrees(math.sqrt(gx*gx + gy*gy + gz*gz))
        ts = f(r["timestamp"])
        if not in_burst and rate > 30.0:
            in_burst = True
            burst_start = ts
            burst_peak = rate
        elif in_burst and rate > 30.0:
            if rate > burst_peak: burst_peak = rate
        elif in_burst and rate <= 30.0:
            in_burst = False
            bursts.append((burst_start, ts, burst_peak))
    print(f"  {len(bursts)} bursts found")
    for s, e, p in bursts:
        print(f"  t = {s/1000:6.2f} .. {e/1000:6.2f}  ({(e-s)/1000:5.2f} s)  peak = {p:6.1f} deg/s")

    # For each burst, check INS_Out 2 seconds before, and 5 seconds after
    print()
    print("===== Recovery check: 5s after each burst ends =====")
    print(f"{'burst_end[s]':>12} {'rep_phi':>8} {'rep_the':>8} {'true_phi':>9} {'true_the':>9} {'|err|':>6}")
    for _, end_ts, _ in bursts:
        # find INS_Out at end_ts + 5s
        target = end_ts / 1000 + 5.0
        nearest = None
        for r in ins:
            t = f(r["timestamp"]) / 1000.0
            if t >= target:
                nearest = r
                break
        if nearest is None: continue
        phi_r = math.degrees(f(nearest["phi"]))
        the_r = math.degrees(f(nearest["theta"]))
        # accel truth at same time
        t_ms = int(f(nearest["timestamp"]))
        im = imu_by_t.get(t_ms) or imu_by_t.get(t_ms-1) or imu_by_t.get(t_ms+1) or imu_by_t.get(t_ms+2)
        if im is None: continue
        ax, ay, az = f(im["acc_x"]), f(im["acc_y"]), f(im["acc_z"])
        an = math.sqrt(ax*ax + ay*ay + az*az)
        if abs(an - 9.80665) > 1.5:
            tag = " (motion)"
            phi_t = the_t = float("nan")
        else:
            tag = ""
            phi_t = math.degrees(math.atan2(-ay, -az))
            the_t = math.degrees(math.atan2(ax, math.sqrt(ay*ay+az*az)))
        err = math.sqrt((phi_r-phi_t)**2 + (the_r-the_t)**2) if phi_t == phi_t else float("nan")
        print(f"{f(nearest['timestamp'])/1000:12.2f} {phi_r:8.1f} {the_r:8.1f} "
              f"{phi_t:9.1f} {the_t:9.1f} {err:6.1f}{tag}")

    # Overall RMS attitude error in quiescent windows (|gyro| < 10 deg/s)
    print()
    print("===== RMS attitude error vs raw-accel truth (quiet windows only) =====")
    # build set of "quiet" timestamps where gyro is small for at least 0.5s
    sum_phi = sum_the = 0.0
    n = 0
    for r in ins:
        t_ms = int(f(r["timestamp"]))
        im = imu_by_t.get(t_ms) or imu_by_t.get(t_ms-1) or imu_by_t.get(t_ms+1)
        if im is None: continue
        gx, gy, gz = f(im["gyr_x"]), f(im["gyr_y"]), f(im["gyr_z"])
        rate = math.degrees(math.sqrt(gx*gx + gy*gy + gz*gz))
        ax, ay, az = f(im["acc_x"]), f(im["acc_y"]), f(im["acc_z"])
        an = math.sqrt(ax*ax + ay*ay + az*az)
        if rate > 15 or abs(an - 9.80665) > 1.0: continue
        phi_t = math.degrees(math.atan2(-ay, -az))
        the_t = math.degrees(math.atan2(ax, math.sqrt(ay*ay+az*az)))
        e_p = math.degrees(f(r["phi"])) - phi_t
        e_t = math.degrees(f(r["theta"])) - the_t
        sum_phi += e_p*e_p
        sum_the += e_t*e_t
        n += 1
    if n > 0:
        print(f"  samples (quiet windows): {n} of {len(ins)} INS frames")
        print(f"  phi RMS error: {math.sqrt(sum_phi/n):5.2f} deg")
        print(f"  theta RMS error: {math.sqrt(sum_the/n):5.2f} deg")


if __name__ == "__main__":
    main()
