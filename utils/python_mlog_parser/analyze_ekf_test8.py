"""Quick triage of ekf_test8: locate where attitude estimation goes bad."""
import csv
import math
import os
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
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def main():
    ins = load_csv("mlog_msg_0_INS_Out.csv")
    imu = load_csv("mlog_msg_10_IMU.csv")
    innov = load_csv("mlog_msg_2_INS_Innov.csv")
    state = load_csv("mlog_msg_1_INS_State.csv")

    print("===== Recording overview =====")
    if ins:
        t0 = f(ins[0]["timestamp"])
        t1 = f(ins[-1]["timestamp"])
        print(f"INS_Out:  {len(ins)} frames, t = {t0:.0f} .. {t1:.0f} ms ({(t1-t0)/1000:.1f} s)")
    if imu:
        t0i = f(imu[0]["timestamp"])
        t1i = f(imu[-1]["timestamp"])
        print(f"IMU:      {len(imu)} frames, t = {t0i:.0f} .. {t1i:.0f} ms (~{len(imu)/((t1i-t0i)/1000):.0f} Hz)")
    if innov:
        t0v = f(innov[0]["timestamp"])
        t1v = f(innov[-1]["timestamp"])
        print(f"INS_Innov:{len(innov)} frames, t = {t0v:.0f} .. {t1v:.0f} ms")

    print()
    print("===== Largest gyro rates (top 15) =====")
    gyro_max = []
    for r in imu:
        gx, gy, gz = f(r["gyr_x"]), f(r["gyr_y"]), f(r["gyr_z"])
        mag = math.sqrt(gx * gx + gy * gy + gz * gz)
        gyro_max.append((mag, f(r["timestamp"]), gx, gy, gz))
    gyro_max.sort(reverse=True)
    print(f"{'rate[rad/s]':>11} {'rate[deg/s]':>11} {'t[s]':>8} {'gx':>9} {'gy':>9} {'gz':>9}")
    for mag, t, gx, gy, gz in gyro_max[:15]:
        print(f"{mag:11.3f} {math.degrees(mag):11.1f} {t/1000:8.2f} {gx:9.3f} {gy:9.3f} {gz:9.3f}")

    print()
    print("===== Largest absolute attitude angles (top 10) =====")
    att_max = []
    for r in ins:
        phi, theta, psi = f(r["phi"]), f(r["theta"]), f(r["psi"])
        # use max of |phi|, |theta| for tilt
        tilt = max(abs(phi), abs(theta))
        att_max.append((tilt, f(r["timestamp"]), phi, theta, psi))
    att_max.sort(reverse=True)
    print(f"{'tilt[rad]':>9} {'tilt[deg]':>9} {'t[s]':>8} {'phi[deg]':>9} {'theta[deg]':>11} {'psi[deg]':>9}")
    for tilt, t, phi, theta, psi in att_max[:10]:
        print(f"{tilt:9.3f} {math.degrees(tilt):9.1f} {t/1000:8.2f} "
              f"{math.degrees(phi):9.1f} {math.degrees(theta):11.1f} {math.degrees(psi):9.1f}")

    print()
    print("===== Quaternion norm errors (top 10 deviations from 1.0) =====")
    qn_dev = []
    for r in ins:
        q0, q1, q2, q3 = f(r["quat[0]"]), f(r["quat[1]"]), f(r["quat[2]"]), f(r["quat[3]"])
        n = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        qn_dev.append((abs(n - 1.0), f(r["timestamp"]), n))
    qn_dev.sort(reverse=True)
    for dev, t, n in qn_dev[:10]:
        print(f"|q|-1 = {n-1:+.6f}  at t = {t/1000:6.2f} s  (|q| = {n:.6f})")

    print()
    print("===== INS_Out finite checks =====")
    nan_rows = 0
    for r in ins:
        for k in ("phi", "theta", "psi", "vn", "ve", "vd"):
            v = f(r[k])
            if not math.isfinite(v):
                nan_rows += 1
                break
    print(f"non-finite rows: {nan_rows} / {len(ins)}")

    print()
    print("===== Attitude jumps (largest single-step delta in tilt) =====")
    jumps = []
    for i in range(1, len(ins)):
        prev = ins[i-1]
        curr = ins[i]
        d_phi = f(curr["phi"]) - f(prev["phi"])
        d_theta = f(curr["theta"]) - f(prev["theta"])
        d_psi = f(curr["psi"]) - f(prev["psi"])
        # wrap psi delta
        while d_psi >  math.pi: d_psi -= 2 * math.pi
        while d_psi < -math.pi: d_psi += 2 * math.pi
        dt = f(curr["delta_ts"]) / 1000.0
        if dt <= 0: dt = 0.1
        rate = math.sqrt(d_phi*d_phi + d_theta*d_theta + d_psi*d_psi) / dt
        jumps.append((rate, f(curr["timestamp"]), d_phi, d_theta, d_psi, dt))
    jumps.sort(reverse=True)
    print(f"{'rate[deg/s]':>11} {'t[s]':>8} {'dphi[deg]':>10} {'dtheta[deg]':>12} {'dpsi[deg]':>10}")
    for rate, t, dp, dt_, dy, dts in jumps[:10]:
        print(f"{math.degrees(rate):11.1f} {t/1000:8.2f} {math.degrees(dp):10.2f} {math.degrees(dt_):12.2f} {math.degrees(dy):10.2f}")

    print()
    print("===== INS_Innov: largest innovations per channel =====")
    # group by tag
    per_tag_max = {}
    for r in innov:
        tag = r.get("tag", "?")
        innov_v = f(r["innov"])
        s = f(r["S"])
        accepted = int(f(r["accepted"]))
        ts = f(r["timestamp"])
        normalised = abs(innov_v) / math.sqrt(s) if s > 0 else float("inf")
        cur = per_tag_max.get(tag)
        if cur is None or normalised > cur[0]:
            per_tag_max[tag] = (normalised, innov_v, s, accepted, ts)
    for tag, (n, iv, s, acc, ts) in sorted(per_tag_max.items()):
        print(f"  tag={tag:12s} max|innov|/sqrt(S) = {n:8.2f}  innov={iv:+9.3e}  S={s:.3e}  acc={acc}  at t={ts/1000:.2f}s")


if __name__ == "__main__":
    main()
