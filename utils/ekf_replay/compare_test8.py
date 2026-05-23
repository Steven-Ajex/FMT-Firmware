"""Compare ekf_test8 original INS_Out vs replayed INS_Out with Path A/B fixes.

The 'accel-tilt' column reads RAW IMU (mlog_msg_10_IMU.csv) so it is
independent of the EKF's bias estimate - it represents the ground truth
under quasi-static conditions."""
import csv
import math
from pathlib import Path

HERE = Path(__file__).parent

ORIG = HERE / "../python_mlog_parser/out/mlog_msg_0_INS_Out.csv"
REP  = HERE / "ekf_test8_replay_INS_Out.csv"
INNOV = HERE / "ekf_test8_replay_INS_Innov.csv"
IMU = HERE / "../python_mlog_parser/out/mlog_msg_10_IMU.csv"


def load_csv(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        rdr = csv.DictReader(f)
        for r in rdr: rows.append(r)
    return rows


def f(v):
    try: return float(v)
    except (TypeError, ValueError): return float("nan")


def main():
    orig = load_csv(ORIG)
    rep  = load_csv(REP)
    imu  = load_csv(IMU)

    # Index IMU by timestamp (ms) - keep latest sample per ms bucket
    imu_by_t = {}
    for r in imu:
        t_ms = int(f(r["timestamp"]))
        imu_by_t[t_ms] = r

    orig_by_t = {round(f(r["timestamp"])): r for r in orig}

    print("===== Attitude comparison vs RAW-accel-implied truth (every 2s) =====")
    print(f"{'t[s]':>5} | {'orig phi':>8}/{'rep phi':<7} | {'orig the':>8}/{'rep the':<7} | "
          f"{'orig psi':>8}/{'rep psi':<7} | {'true phi':>8}/{'true the':<8}")
    last_t = -100
    for r in rep:
        t_ms = round(f(r["timestamp"]))
        t = t_ms / 1000.0
        if t - last_t >= 2.0:
            o = orig_by_t.get(t_ms) or orig_by_t.get(t_ms-1) or orig_by_t.get(t_ms+1)
            # find nearest IMU sample
            im = imu_by_t.get(t_ms) or imu_by_t.get(t_ms-1) or imu_by_t.get(t_ms+1) or imu_by_t.get(t_ms+2)
            if im:
                ax, ay, az = f(im["acc_x"]), f(im["acc_y"]), f(im["acc_z"])
                an = math.sqrt(ax*ax + ay*ay + az*az)
                if an > 0.5:
                    phi_t = math.degrees(math.atan2(-ay, -az))
                    tht_t = math.degrees(math.atan2(ax, math.sqrt(ay*ay+az*az)))
                else:
                    phi_t = tht_t = float("nan")
            else:
                phi_t = tht_t = float("nan")
            ops = f(o['phi']) if o else float('nan')
            ots = f(o['theta']) if o else float('nan')
            opss= f(o['psi']) if o else float('nan')
            print(f"{t:5.1f} | {math.degrees(ops):8.1f}/{math.degrees(f(r['phi'])):<7.1f} | "
                  f"{math.degrees(ots):8.1f}/{math.degrees(f(r['theta'])):<7.1f} | "
                  f"{math.degrees(opss):8.1f}/{math.degrees(f(r['psi'])):<7.1f} | "
                  f"{phi_t:8.1f}/{tht_t:<8.1f}")
            last_t = t

    # Innovation rejection
    print()
    print("===== Replayed innovation rejection (Path A adaptive gate) =====")
    if INNOV.exists():
        innov = load_csv(INNOV)
        per_tag = {}
        max_consec = {}
        cur_consec = {}
        for r in innov:
            tag = r.get("tag", "?")
            acc = int(f(r["accepted"]))
            d = per_tag.setdefault(tag, [0, 0])
            d[0] += 1
            if not acc:
                d[1] += 1
                cur_consec[tag] = cur_consec.get(tag, 0) + 1
                max_consec[tag] = max(max_consec.get(tag, 0), cur_consec[tag])
            else:
                cur_consec[tag] = 0
        for tag in sorted(per_tag):
            tot, rej = per_tag[tag]
            mc = max_consec.get(tag, 0)
            print(f"  {tag:>12}: total={tot:5d}  rejected={rej:5d} ({100.0*rej/tot:5.1f}%)  max_consec_rej={mc:4d}")

    # Mean-square attitude error in 60-79s (post-recovery window)
    print()
    print("===== RMS attitude error vs raw-accel truth (60-79s window) =====")
    sum_orig_phi = sum_orig_the = sum_rep_phi = sum_rep_the = 0.0
    n = 0
    for r in rep:
        t_ms = round(f(r["timestamp"]))
        t = t_ms / 1000.0
        if not (60.0 <= t <= 79.0): continue
        o = orig_by_t.get(t_ms) or orig_by_t.get(t_ms-1) or orig_by_t.get(t_ms+1)
        im = imu_by_t.get(t_ms) or imu_by_t.get(t_ms-1) or imu_by_t.get(t_ms+1)
        if not o or not im: continue
        ax, ay, az = f(im["acc_x"]), f(im["acc_y"]), f(im["acc_z"])
        an = math.sqrt(ax*ax + ay*ay + az*az)
        if abs(an - 9.80665) > 1.5: continue   # only quasi-static window
        phi_t = math.degrees(math.atan2(-ay, -az))
        tht_t = math.degrees(math.atan2(ax, math.sqrt(ay*ay+az*az)))
        e_op = math.degrees(f(o["phi"]))   - phi_t
        e_ot = math.degrees(f(o["theta"])) - tht_t
        e_rp = math.degrees(f(r["phi"]))   - phi_t
        e_rt = math.degrees(f(r["theta"])) - tht_t
        sum_orig_phi += e_op*e_op
        sum_orig_the += e_ot*e_ot
        sum_rep_phi  += e_rp*e_rp
        sum_rep_the  += e_rt*e_rt
        n += 1
    if n > 0:
        print(f"  samples in window: {n}")
        print(f"  original  RMS error: phi={math.sqrt(sum_orig_phi/n):6.1f}°  theta={math.sqrt(sum_orig_the/n):6.1f}°")
        print(f"  replayed  RMS error: phi={math.sqrt(sum_rep_phi/n):6.1f}°  theta={math.sqrt(sum_rep_the/n):6.1f}°")


if __name__ == "__main__":
    main()
