#!/usr/bin/env python3
"""What a survey flight recorded (docs/survey_mapping_plan.md, phase 1): out/survey/<tag>.radar and <tag>.ulg.

    scripts/survey_check.py out/survey/rtk_2

Prints, per radar, the frame rate, detections per frame and gaps; the size of the recording and its data
rate as the radars send it (TI out-of-box format) and packed; and how the radar time stamps line up with
the PX4 log's clock. That last part compares the vehicle position in both (each from the simulation's
ground truth) over a range of clock offsets: the mapping must look the pose up at the right time.
"""

import argparse
import glob
import os
import sys

import numpy as np
from pyulog import ULog

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from radar_log import FRAME_BYTES_TI, POINT_BYTES_PACKED, POINT_BYTES_TI, load  # noqa: E402

ENU_TO_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])


def base_positions(rec):
    """Position of base_link in the world (ENU) at each frame, from the radar's true pose and its mount."""
    mounts = rec["meta"]["radars"]
    rot = rec["rotation"].reshape(-1, 3, 3)
    out = np.empty((len(rec["stamp"]), 3))

    for i, m in enumerate(mounts):
        sel = rec["radar"] == i
        r_mount, p_mount = np.array(m["mount_rotation"]), np.array(m["mount_position"])
        r_base = rot[sel] @ r_mount.T
        out[sel] = rec["position"][sel] - r_base @ p_mount

    return out


def clock_offset(rec, ulog_path):
    """Offset (s) to add to the radar stamps to get the PX4 log's time, and the residual (m) there."""
    log = ULog(ulog_path, ["vehicle_local_position_groundtruth"])
    g = next(d.data for d in log.data_list if d.name == "vehicle_local_position_groundtruth")
    tg = g["timestamp"] * 1e-6
    ned = np.column_stack((g["x"], g["y"], g["z"]))
    base = base_positions(rec) @ ENU_TO_NED.T
    stamp = rec["stamp"]
    # Coarse: the offset of the first samples; the simulation and PX4 clocks start near each other.
    best = None

    for off in np.concatenate((np.arange(-5.0, 5.0, 0.05), )):
        res = residual(stamp + off, base, tg, ned)
        if res is not None and (best is None or res < best[1]):
            best = (off, res)

    if best is None:
        return None

    for off in np.arange(best[0] - 0.05, best[0] + 0.05, 0.001):
        res = residual(stamp + off, base, tg, ned)
        if res is not None and res < best[1]:
            best = (off, res)

    return best


def residual(t, base, tg, ned):
    """Median distance (m) between the two positions at the same times, the constant offset removed;
    only where the vehicle moves (a vehicle at rest fits any clock offset)."""
    inside = (t > tg[0]) & (t < tg[-1])

    if inside.sum() < 100:
        return None

    p = np.column_stack([np.interp(t[inside], tg, ned[:, k]) for k in range(3)])
    d = base[inside] - p
    speed = np.r_[0.0, np.linalg.norm(np.diff(p, axis=0), axis=1) / np.maximum(np.diff(t[inside]), 1e-3)]
    moving = speed > 1.0

    if moving.sum() < 50:
        return None

    d = d[moving]
    return float(np.median(np.linalg.norm(d - np.median(d, axis=0), axis=1)))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("tag", help="out/survey/<tag> (without extension)")
    args = p.parse_args()

    rec = load(args.tag + ".radar")
    stamp, radar, count = rec["stamp"], rec["radar"], rec["count"]
    span = stamp[-1] - stamp[0]
    size = sum(os.path.getsize(f) for f in glob.glob(os.path.join(args.tag + ".radar", "*")))
    print(f"{args.tag}.radar: {len(stamp)} frames, {len(rec['points'])} detections, {span:.0f} s, "
          f"{size / 1e6:.1f} MB on disk ({size / 1e6 / span * 60:.1f} MB/min)")

    for i, m in enumerate(rec["meta"]["radars"]):
        s = stamp[radar == i]
        gaps = np.diff(s)
        print(f"  {m['name']:12s} {len(s) / span:5.1f} Hz, {count[radar == i].mean():5.1f} detections/frame, "
              f"longest gap {gaps.max() if len(gaps) else 0:.2f} s")

    ti = (len(rec["points"]) * POINT_BYTES_TI + len(stamp) * FRAME_BYTES_TI) * 8 / span / 1e3
    packed = (len(rec["points"]) * POINT_BYTES_PACKED + len(stamp) * 8) * 8 / span / 1e3
    # Per second, the peak: the data rate the link has to carry at its worst.
    sec = np.floor(stamp - stamp[0]).astype(int)
    per_s = np.bincount(sec, weights=count * POINT_BYTES_TI + FRAME_BYTES_TI) * 8 / 1e3
    print(f"data rate, all radars: {ti:.0f} kbit/s as the radars send it (peak {per_s.max():.0f} in a second), "
          f"{packed:.0f} kbit/s packed; 300 kbit/s link: {'fits' if per_s.max() <= 300 else 'does not fit'} as sent")

    if os.path.exists(args.tag + ".ulg"):
        found = clock_offset(rec, args.tag + ".ulg")
        if found:
            print(f"clock: PX4 log time = radar stamp {found[0]:+.3f} s (position residual {found[1]:.3f} m there)")
        else:
            print("clock: the recording and the PX4 log do not overlap in time")

    return 0


if __name__ == "__main__":
    sys.exit(main())
