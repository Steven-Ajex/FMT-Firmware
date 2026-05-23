"""Test 9 final-stabilization-window precision (76-85s, after all maneuvers)."""
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
    imu_by_t = {int(f(r["timestamp"])): r for r in imu}

    print("===== Final stable window 76-85s (every 0.5s) =====")
    print(f"{'t[s]':>5} {'rep_phi':>8} {'true_phi':>9} {'rep_the':>8} {'true_the':>9}  {'err':>5}")
    last_t = 0
    sum_phi = sum_the = 0.0
    n = 0
    for r in ins:
        t_ms = int(f(r["timestamp"]))
        t = t_ms / 1000.0
        if not (76.0 <= t <= 85.5): continue
        if t - last_t < 0.5: continue
        last_t = t
        im = imu_by_t.get(t_ms) or imu_by_t.get(t_ms-1) or imu_by_t.get(t_ms+1) or imu_by_t.get(t_ms+2)
        if im is None: continue
        ax, ay, az = f(im["acc_x"]), f(im["acc_y"]), f(im["acc_z"])
        phi_t = math.degrees(math.atan2(-ay, -az))
        the_t = math.degrees(math.atan2(ax, math.sqrt(ay*ay+az*az)))
        phi_r = math.degrees(f(r["phi"]))
        the_r = math.degrees(f(r["theta"]))
        e = math.sqrt((phi_r-phi_t)**2 + (the_r-the_t)**2)
        print(f"{t:5.2f} {phi_r:8.2f} {phi_t:9.2f} {the_r:8.2f} {the_t:9.2f}  {e:5.2f}")
        sum_phi += (phi_r-phi_t)**2
        sum_the += (the_r-the_t)**2
        n += 1
    if n > 0:
        print()
        print(f"  N = {n}")
        print(f"  RMS phi error: {math.sqrt(sum_phi/n):.3f} deg")
        print(f"  RMS theta error: {math.sqrt(sum_the/n):.3f} deg")


if __name__ == "__main__":
    main()
