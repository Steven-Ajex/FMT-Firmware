"""Focus on the big rotation event around t=56s in ekf_test8.

Goal: determine whether the attitude estimate is genuinely broken
after the manoeuvre or just looks broken because of Euler-angle
gimbal-lock noise."""
import csv
import math
from pathlib import Path

HERE = Path(__file__).parent
OUT = HERE / "out"

# Innovation tag table - must match k_innov_tags in ekf_core.c
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


def quat_to_euler(q0, q1, q2, q3):
    # body-to-NED, [w x y z]
    phi   = math.atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2))
    s = 2.0 * (q0 * q2 - q3 * q1)
    s = max(-1.0, min(1.0, s))
    theta = math.asin(s)
    psi   = math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))
    return phi, theta, psi


def main():
    ins = load_csv("mlog_msg_0_INS_Out.csv")
    imu = load_csv("mlog_msg_10_IMU.csv")
    innov = load_csv("mlog_msg_2_INS_Innov.csv")

    # ---------- 1. Time-window around the high-rate event (55-58 s) ----------
    print("===== INS_Out around t=55-58s (10Hz logging) =====")
    print(f"{'t[s]':>6} {'phi[deg]':>9} {'theta[deg]':>11} {'psi[deg]':>9} "
          f"{'p[deg/s]':>9} {'q[deg/s]':>9} {'r[deg/s]':>9} "
          f"{'ax':>7} {'ay':>7} {'az':>7}")
    for r in ins:
        t = f(r["timestamp"]) / 1000.0
        if 55.0 <= t <= 58.5:
            phi, theta, psi = f(r["phi"]), f(r["theta"]), f(r["psi"])
            p, q, ro = f(r["p"]), f(r["q"]), f(r["r"])
            ax, ay, az = f(r["ax"]), f(r["ay"]), f(r["az"])
            print(f"{t:6.2f} {math.degrees(phi):9.1f} {math.degrees(theta):11.1f} "
                  f"{math.degrees(psi):9.1f} "
                  f"{math.degrees(p):9.1f} {math.degrees(q):9.1f} {math.degrees(ro):9.1f} "
                  f"{ax:7.2f} {ay:7.2f} {az:7.2f}")

    # ---------- 2. IMU around the event ----------
    print()
    print("===== IMU around t=56.6-56.8s (10ms samples) =====")
    print(f"{'t[s]':>7} {'gx':>8} {'gy':>8} {'gz':>8} |g|[deg/s] "
          f"{'ax':>7} {'ay':>7} {'az':>7}")
    last_print = 0
    for r in imu:
        t = f(r["timestamp"]) / 1000.0
        if 56.5 <= t <= 56.95 and t - last_print >= 0.010:
            gx, gy, gz = f(r["gyr_x"]), f(r["gyr_y"]), f(r["gyr_z"])
            ax, ay, az = f(r["acc_x"]), f(r["acc_y"]), f(r["acc_z"])
            g = math.sqrt(gx*gx + gy*gy + gz*gz)
            print(f"{t:7.3f} {gx:8.2f} {gy:8.2f} {gz:8.2f}    {math.degrees(g):7.1f} "
                  f"{ax:7.2f} {ay:7.2f} {az:7.2f}")
            last_print = t

    # ---------- 3. Quaternion-based attitude vs reported Euler ----------
    print()
    print("===== Quaternion-reconstructed Euler vs reported Euler =====")
    print(f"{'t[s]':>6} {'rep_phi':>9} {'q_phi':>9} {'rep_theta':>11} "
          f"{'q_theta':>9} {'rep_psi':>9} {'q_psi':>9}")
    for r in ins:
        t = f(r["timestamp"]) / 1000.0
        if 55.0 <= t <= 58.5:
            phi_r, theta_r, psi_r = f(r["phi"]), f(r["theta"]), f(r["psi"])
            q0, q1, q2, q3 = f(r["quat[0]"]), f(r["quat[1]"]), f(r["quat[2]"]), f(r["quat[3]"])
            phi_q, theta_q, psi_q = quat_to_euler(q0, q1, q2, q3)
            print(f"{t:6.2f} {math.degrees(phi_r):9.1f} {math.degrees(phi_q):9.1f} "
                  f"{math.degrees(theta_r):11.1f} {math.degrees(theta_q):9.1f} "
                  f"{math.degrees(psi_r):9.1f} {math.degrees(psi_q):9.1f}")

    # ---------- 4. Post-event: did it recover? Look at t=60-79s ----------
    print()
    print("===== INS_Out long-term post-event (60-79s, every 2s) =====")
    print(f"{'t[s]':>6} {'phi[deg]':>9} {'theta[deg]':>11} {'psi[deg]':>9} "
          f"{'p[d/s]':>8} {'ax':>7} {'ay':>7} {'az':>7}  |a|")
    last_t = 0
    for r in ins:
        t = f(r["timestamp"]) / 1000.0
        if t >= 60.0 and t - last_t >= 2.0:
            phi, theta, psi = f(r["phi"]), f(r["theta"]), f(r["psi"])
            p = f(r["p"])
            ax, ay, az = f(r["ax"]), f(r["ay"]), f(r["az"])
            am = math.sqrt(ax*ax + ay*ay + az*az)
            print(f"{t:6.2f} {math.degrees(phi):9.1f} {math.degrees(theta):11.1f} "
                  f"{math.degrees(psi):9.1f} {math.degrees(p):8.1f} "
                  f"{ax:7.2f} {ay:7.2f} {az:7.2f}  {am:5.2f}")
            last_t = t

    # ---------- 5. INS_Innov tag-decoded summary in the event window ----------
    print()
    print("===== INS_Innov in 56-58s window =====")
    print(f"{'t[s]':>7} {'tag':>10} {'accepted':>8} {'innov':>11} "
          f"{'sqrt(S)':>9} {'norm':>7}")
    for r in innov:
        t = f(r["timestamp"]) / 1000.0
        if 56.0 <= t <= 58.0:
            tag_id = int(f(r["tag_id"]))
            tag = TAG_NAMES[tag_id] if 0 <= tag_id < len(TAG_NAMES) else f"?{tag_id}"
            acc = int(f(r["accepted"]))
            iv = f(r["innov"])
            S = f(r["S"])
            norm = abs(iv) / math.sqrt(S) if S > 0 else float("inf")
            print(f"{t:7.3f} {tag:>10} {acc:8d} {iv:+11.3f} {math.sqrt(S) if S > 0 else 0:9.3f} {norm:7.2f}")

    # ---------- 6. Rejected innovations everywhere ----------
    print()
    print("===== All rejected innovations (acc=0) per tag =====")
    rejected = {}
    for r in innov:
        if int(f(r["accepted"])) == 0:
            tag_id = int(f(r["tag_id"]))
            tag = TAG_NAMES[tag_id] if 0 <= tag_id < len(TAG_NAMES) else f"?{tag_id}"
            rejected.setdefault(tag, []).append(f(r["timestamp"]) / 1000.0)
    for tag in sorted(rejected):
        ts_list = rejected[tag]
        print(f"  {tag:>12}: {len(ts_list):4d} rejections, t = "
              f"{ts_list[0]:.2f} .. {ts_list[-1]:.2f} s")


if __name__ == "__main__":
    main()
