#!/usr/bin/env python3
"""How far PX4's estimated pose is from the simulation's ground truth, from a PX4 log (SITL logs both).

The mapping places every radar detection with the estimated pose (docs/survey_mapping_plan.md), so this
error goes straight into the map: a position error moves every point by itself, an attitude error moves a
point at range R by about R x angle.

    scripts/pose_check.py out/survey/rtk_1.ulg

Prints, over the airborne part of the flight (more than --min-alt above the start), the horizontal and
vertical position error (median, 95 %, max), its constant part (the offset between the estimate's origin
and the truth's), and the tilt and heading error, with what that is at 20 m range.
"""

import argparse
import sys

import numpy as np
from pyulog import ULog


def topic(log, name):
    for d in log.data_list:
        if d.name == name and d.multi_id == 0:
            return d.data
    raise SystemExit(f"{name} is not in the log")


def quat_angle(q1, q2):
    """Angle (rad) between unit quaternions, rows (w, x, y, z)."""
    dot = np.abs(np.sum(q1 * q2, axis=1))
    return 2.0 * np.arccos(np.clip(dot, -1.0, 1.0))


def tilt_and_yaw(q):
    """Body z axis in NED and yaw (rad) of quaternions (w, x, y, z), body to NED."""
    w, x, y, z = q.T
    zaxis = np.column_stack((2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return zaxis, yaw


def pct(v):
    return f"median {np.median(v):.3f}, 95 % {np.percentile(v, 95):.3f}, max {np.max(v):.3f}"


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("ulog")
    p.add_argument("--min-alt", type=float, default=1.0, help="only where this far above the start (m)")
    p.add_argument("--range", type=float, default=20.0, help="range at which to state the attitude error (m)")
    args = p.parse_args()

    log = ULog(args.ulog, ["vehicle_local_position", "vehicle_local_position_groundtruth",
                           "vehicle_attitude", "vehicle_attitude_groundtruth"])
    est, tru = topic(log, "vehicle_local_position"), topic(log, "vehicle_local_position_groundtruth")
    t = est["timestamp"] * 1e-6
    tt = tru["timestamp"] * 1e-6
    pe = np.column_stack([est[k] for k in "xyz"])
    pt = np.column_stack([np.interp(t, tt, tru[k]) for k in "xyz"])
    inside = (t >= tt[0]) & (t <= tt[-1])
    air = inside & (-(pt[:, 2] - pt[inside, 2][0]) > args.min_alt)

    if not air.any():
        raise SystemExit("the vehicle never left the ground in this log")

    err = pe[air] - pt[air]
    offset = err.mean(axis=0)
    print(f"{args.ulog}: {t[air][-1] - t[air][0]:.0f} s airborne")
    print(f"position, estimate - truth (m): horizontal {pct(np.linalg.norm(err[:, :2], axis=1))}; "
          f"vertical {pct(np.abs(err[:, 2]))}")
    print(f"  of which constant (origin offset) N {offset[0]:+.3f} E {offset[1]:+.3f} D {offset[2]:+.3f}; "
          f"without it: horizontal {pct(np.linalg.norm(err[:, :2] - offset[:2], axis=1))}")

    att, atr = topic(log, "vehicle_attitude"), topic(log, "vehicle_attitude_groundtruth")
    ta, tat = att["timestamp"] * 1e-6, atr["timestamp"] * 1e-6
    keep = (ta >= max(t[air][0], tat[0])) & (ta <= min(t[air][-1], tat[-1]))
    qe = np.column_stack([att[f"q[{i}]"] for i in range(4)])[keep]
    # Truth at the estimate's times: nearest sample (the truth runs at the simulation rate).
    idx = np.clip(np.searchsorted(tat, ta[keep]), 0, len(tat) - 1)
    qt = np.column_stack([atr[f"q[{i}]"] for i in range(4)])[idx]
    ze, ye = tilt_and_yaw(qe)
    zt, yt = tilt_and_yaw(qt)
    tilt = np.degrees(np.arccos(np.clip(np.sum(ze * zt, axis=1), -1.0, 1.0)))
    yaw = np.degrees(np.abs((ye - yt + np.pi) % (2 * np.pi) - np.pi))
    total = np.degrees(quat_angle(qe, qt))
    at_r = args.range * np.radians(np.percentile(total, 95))
    print(f"attitude error (deg): tilt {pct(tilt)}; heading {pct(yaw)}; total {pct(total)}")
    print(f"  95 % of the attitude error is {at_r:.2f} m at {args.range:g} m range")
    return 0


if __name__ == "__main__":
    sys.exit(main())
