#!/usr/bin/env python3
"""Per-obstacle account of an avoidance flight (out/avoid.csv or a trial's CSV): how close the vehicle came to
each wire and obstacle of the world, at what height and offset it passed it, and how long it braked.

    python3 scripts/course_report.py out/trials/power_line_course_5_401.csv --world power_line_course
"""

import argparse
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from avoid_node import distance_to_segments
from scene import load_obstacles, load_wires

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("csv", nargs="+")
    p.add_argument("--world", default="power_line_course")
    p.add_argument("--min-distance", type=float, default=1.0)
    args = p.parse_args()

    sdf = os.path.join(ROOT, "worlds", args.world + ".sdf")
    things = [(name, lambda q, w=(name, p0, p1, r): distance_to_segments(q, [w])[0]) for name, p0, p1, r in load_wires(sdf)]
    things += [(o.name, lambda q, o=o: float(o.surface_distance(q)[0])) for o in load_obstacles(sdf)]

    for path in args.csv:
        rows = list(csv.DictReader(open(path)))
        pos = np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in rows])
        t = np.array([float(r["t"]) for r in rows])
        brake = np.array([int(r["brake"]) for r in rows])
        dt = np.diff(t, prepend=t[0])
        print(f"\n{os.path.basename(path)}: {t[-1]:.0f} s, end ({pos[-1, 0]:.1f}, {pos[-1, 1]:.1f}, {pos[-1, 2]:.1f}), "
              f"height {pos[:, 2].min():.1f}-{pos[:, 2].max():.1f} m, braking {dt[brake == 1].sum():.1f} s")
        print(f"  {'obstacle':12s} {'closest':>8s}  {'at (x, y, z)':>20s}")

        for name, dist in things:
            d = np.array([dist(q) for q in pos])
            i = int(d.argmin())

            if d[i] > 8.0:      # never near it (the pylons and crossarms off to the side)
                continue

            flag = "  < min" if d[i] < args.min_distance else ""
            print(f"  {name:12s} {d[i]:7.2f} m  ({pos[i, 0]:5.1f}, {pos[i, 1]:5.1f}, {pos[i, 2]:5.1f}){flag}")


if __name__ == "__main__":
    main()
