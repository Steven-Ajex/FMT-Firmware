#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# run_diff.py
#
# One-shot orchestrator for the ekf_ins offline replay workflow.  Given a
# firmware mlog binary, it:
#
#   1. invokes utils/python_mlog_parser/parse_mlog.py to break the binary
#      into one CSV per bus type;
#   2. builds utils/ekf_replay/ekf_replay if it is missing or older than
#      the EKF library sources;
#   3. drives the binary through the input bus CSVs and writes
#      INS_Out_replay.csv;
#   4. runs compare.py against the recorded INS_Out CSV from the same
#      flight (typically the cf_ins output captured during the mission).
#
# Usage:
#     python3 run_diff.py PATH/TO/mlog1.bin            # text summary only
#     python3 run_diff.py PATH/TO/mlog1.bin --plot     # also writes diff.png

import argparse
import glob
import os
import shutil
import subprocess
import sys
from pathlib import Path

HERE          = Path(__file__).resolve().parent       # utils/ekf_replay
REPO_ROOT     = HERE.parent.parent
PARSER_DIR    = REPO_ROOT / "utils" / "python_mlog_parser"
PARSER_SCRIPT = PARSER_DIR / "parse_mlog.py"
PARSER_OUT    = PARSER_DIR / "out"
LIB_DIR       = REPO_ROOT / "src" / "model" / "ins" / "ekf_ins" / "lib"
REPLAY_BIN    = HERE / "ekf_replay"


def die(msg, code=1):
    print(f"run_diff: {msg}", file=sys.stderr)
    sys.exit(code)


def newest_mtime(paths):
    return max((Path(p).stat().st_mtime for p in paths), default=0.0)


def ensure_replay_built():
    """Run `make` only if the binary is missing or older than its sources."""
    sources = [str(p) for p in [HERE / "replay.c", HERE / "Makefile"]] \
            + [str(p) for p in LIB_DIR.glob("*.c")] \
            + [str(p) for p in LIB_DIR.glob("*.h")]
    if not REPLAY_BIN.exists() or REPLAY_BIN.stat().st_mtime < newest_mtime(sources):
        print("run_diff: building ekf_replay...")
        subprocess.run(["make"], cwd=HERE, check=True)
    else:
        print("run_diff: ekf_replay up to date")


def run_parser(mlog_path: Path):
    """parse_mlog.py auto-discovers mlog1.bin in the project root.  We give
    it that exact filename in a private working directory so it never
    clobbers anything.  Existing CSVs in utils/python_mlog_parser/out/ are
    cleared first."""
    if PARSER_OUT.exists():
        shutil.rmtree(PARSER_OUT)

    workdir = HERE / ".run_diff_work"
    workdir.mkdir(exist_ok=True)
    target = workdir / "mlog1.bin"
    shutil.copy(mlog_path, target)

    print(f"run_diff: parsing {mlog_path.name} ...")
    subprocess.run([sys.executable, str(PARSER_SCRIPT)],
                   cwd=workdir, check=True)
    # parser writes to PARSER_OUT regardless of cwd
    if not PARSER_OUT.exists():
        die("parser did not produce an out/ directory")
    print(f"run_diff: CSVs at {PARSER_OUT}")
    return PARSER_OUT


def find_csv(out_dir: Path, suffix: str):
    """Locate the parser-generated CSV whose filename ends with _<suffix>.csv."""
    matches = sorted(out_dir.glob(f"mlog_msg_*_{suffix}.csv"))
    return matches[0] if matches else None


def run_replay(csv_dir: Path):
    """Build the ekf_replay command line from whichever CSVs we found and
    run it.  Missing optional buses are simply omitted."""
    cmd = [str(REPLAY_BIN)]
    csvs = {
        "imu":  find_csv(csv_dir, "IMU"),
        "mag":  find_csv(csv_dir, "MAG"),
        "baro": find_csv(csv_dir, "Barometer"),
        "gps":  find_csv(csv_dir, "GPS_uBlox"),
        "rf":   find_csv(csv_dir, "Rangefinder"),
        "opf":  find_csv(csv_dir, "OpticalFlow"),
        "ext":  find_csv(csv_dir, "External_Pos"),
    }
    if csvs["imu"] is None:
        die("no IMU CSV in parser output -- mlog must contain an IMU bus")
    for k, v in csvs.items():
        if v is not None:
            cmd += [f"--{k}", str(v)]

    out_csv = HERE / "INS_Out_replay.csv"
    cmd += ["--out", str(out_csv)]
    print(f"run_diff: replay -> {out_csv.name}")
    subprocess.run(cmd, check=True)
    return out_csv


def run_compare(reference: Path, replay: Path, plot: Path = None):
    cmd = [sys.executable, str(HERE / "compare.py"),
           str(reference), str(replay)]
    if plot is not None:
        cmd += ["--plot", str(plot)]
    subprocess.run(cmd, check=True)


def main():
    ap = argparse.ArgumentParser(description="ekf_ins offline replay + compare")
    ap.add_argument("mlog", type=Path, help="firmware mlog binary (mlog1.bin)")
    ap.add_argument("--plot", action="store_true",
                    help="also produce diff.png alongside the binary")
    args = ap.parse_args()

    if not args.mlog.exists():
        die(f"{args.mlog} not found")

    ensure_replay_built()
    csv_dir   = run_parser(args.mlog)
    reference = find_csv(csv_dir, "INS_Out")
    if reference is None:
        die("recorded INS_Out CSV is missing; cannot compare")
    replay_csv = run_replay(csv_dir)

    plot_path = (HERE / "diff.png") if args.plot else None
    print()
    print(f"run_diff: comparing {reference.name}  <-->  {replay_csv.name}")
    print()
    run_compare(reference, replay_csv, plot_path)
    if plot_path is not None:
        print(f"run_diff: plot at {plot_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
