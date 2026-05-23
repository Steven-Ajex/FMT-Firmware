"""Look at the early phase of ekf_test8 - how did the attitude get so wrong?"""
import csv
import math
from pathlib import Path

HERE = Path(__file__).parent
OUT = HERE / "out"

TAG_NAMES = ["mag", "grav_x", "grav_y",
             "gps_pos_n", "gps_pos_e", "gps_pos_d",
             "gps_vel_n", "gps_vel_e", "gps_vel_d",
             "baro", "rf", "opf_x", "opf_y",
             "ext_x", "ext_y", "ext_z", "ext_phi", "ext_theta", "ext_psi"]


def load_csv(name):
    rows = []
    with open(OUT / name, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def main():
    ins = load_csv("mlog_msg_0_INS_Out.csv")
    innov = load_csv("mlog_msg_2_INS_Innov.csv")

    print("===== INS_Out from t=0 every 1s (early phase) =====")
    print(f"{'t[s]':>6} {'phi[deg]':>9} {'theta[deg]':>11} {'psi[deg]':>9} "
          f"{'p[d/s]':>8} {'q[d/s]':>8} {'r[d/s]':>8} "
          f"{'ax':>7} {'ay':>7} {'az':>7}  |a|")
    last_t = -100.0
    for r in ins:
        t = f(r["timestamp"]) / 1000.0
        if t - last_t >= 1.0:
            phi, theta, psi = f(r["phi"]), f(r["theta"]), f(r["psi"])
            p, q, ro = f(r["p"]), f(r["q"]), f(r["r"])
            ax, ay, az = f(r["ax"]), f(r["ay"]), f(r["az"])
            am = math.sqrt(ax*ax + ay*ay + az*az)
            print(f"{t:6.2f} {math.degrees(phi):9.1f} {math.degrees(theta):11.1f} "
                  f"{math.degrees(psi):9.1f} "
                  f"{math.degrees(p):8.1f} {math.degrees(q):8.1f} {math.degrees(ro):8.1f} "
                  f"{ax:7.2f} {ay:7.2f} {az:7.2f}  {am:5.2f}")
            last_t = t

    # ---------- Innovation rejection histogram over full run ----------
    print()
    print("===== INS_Innov rejection rate per channel (full 79s) =====")
    per_tag = {}
    for r in innov:
        tag_id = int(f(r["tag_id"]))
        tag = TAG_NAMES[tag_id] if 0 <= tag_id < len(TAG_NAMES) else f"?{tag_id}"
        acc = int(f(r["accepted"]))
        d = per_tag.setdefault(tag, {"total": 0, "acc": 0, "rej": 0})
        d["total"] += 1
        if acc: d["acc"] += 1
        else:   d["rej"] += 1
    for tag in sorted(per_tag):
        d = per_tag[tag]
        if d["total"] > 0:
            rej_pct = 100.0 * d["rej"] / d["total"]
            print(f"  {tag:>12}: total={d['total']:5d}  accepted={d['acc']:5d}  "
                  f"rejected={d['rej']:5d}  ({rej_pct:5.1f}% rejected)")

    # ---------- When does gravity_x rejection START? ----------
    print()
    print("===== First 30 grav_x innovations (before & after rejection starts) =====")
    print(f"{'t[s]':>8} {'innov':>10} {'sqrt(S)':>9} {'norm':>7} {'acc':>4}")
    cnt = 0
    for r in innov:
        tag_id = int(f(r["tag_id"]))
        tag = TAG_NAMES[tag_id] if 0 <= tag_id < len(TAG_NAMES) else f"?{tag_id}"
        if tag != "grav_x":
            continue
        t = f(r["timestamp"]) / 1000.0
        iv = f(r["innov"])
        S = f(r["S"])
        nv = abs(iv) / math.sqrt(S) if S > 0 else float("inf")
        acc = int(f(r["accepted"]))
        print(f"{t:8.3f} {iv:+10.3f} {math.sqrt(S) if S > 0 else 0:9.3f} {nv:7.2f} {acc:4d}")
        cnt += 1
        if cnt >= 30: break


if __name__ == "__main__":
    main()
