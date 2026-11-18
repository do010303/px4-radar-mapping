#!/usr/bin/env python3
"""Survey flight over worlds/survey_site.sdf for radar mapping (docs/survey_mapping_plan.md).

Flies, with the obstacle avoidance of scripts/avoid_node.py on as the safety net, a pattern that sees every
object of the site from several sides:

  lines   along x at y = -14 and 14 (the power line runs along y = 0), at --low and --high altitude
  line    along the power line: above it (y = 0, --over-alt) and beside it on both sides (y = 8 and -7.5,
          10 m up). The wires run
          along x and the vehicle keeps facing +x, so the forward ISK never looks square at them, and the AOPs
          see a 20 mm wire only within about 7 m (profiles/iwr6843aop.yaml): from the lines at y = +-14 the
          wires were all but invisible (docs/survey_mapping_plan.md, phase 3)
  circle  around the site centre (30, 0), radius --radius, at --circle-alt

Every object is at least --min-clearance from the path (checked against worlds/survey_site.truth.yaml
before take-off). Afterwards the flight's files go to out/survey/<tag>.*: the avoidance CSV, the console
output, hard links to the radar recording (<tag>.radar/, with run_sitl.sh RADAR_LOG=1) and to the PX4 log (run/rootfs/log, which loses old days), whose estimated pose
(vehicle_local_position, vehicle_attitude) and, in SITL, ground truth (*_groundtruth) the mapping uses.

    PX4_GZ_WORLD=survey_site RADARS=aop GPS=rtk RADAR_LOG=1 ./run_sitl.sh   # terminal 1
    scripts/survey_mission.py --tag rtk_1                           # terminal 2
    scripts/survey_mission.py --print                               # only the waypoints
"""

import argparse
import glob
import os
import subprocess
import sys
import time

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CENTRE = np.array([30.0, 0.0])
LINE_Y = 14.0
X_START, X_END = 0.0, 60.0
LOW_PASS_Y = 12.5          # m: the low pass, 3.5 m from the house's north face, 5.3 m from the wall's
CHUNK_WAIT = 7.0           # s: radar_log writes a chunk every 5 s of simulation time


def waypoints(args):
    """The pattern as (x, y, z) points: lawnmower lines at two altitudes, then the circle, then home."""
    low, high = args.low, args.high
    w = [(X_START, -LINE_Y, low), (X_END, -LINE_Y, low), (X_END, LINE_Y, low)]
    if args.low_pass_alt > 0:
        # Back west low beside the house and the wall (y 5-9 and 6.8-7.2): level with their walls instead of
        # over them, where only roof and top were seen (their centres placed 1-1.8 m high, the house 2 m
        # short of its 4 m).
        w += [(X_END, LOW_PASS_Y, args.low_pass_alt), (X_START, LOW_PASS_Y, args.low_pass_alt)]
    w += [(X_START, LINE_Y, low),
          (X_START, LINE_Y, high), (X_END, LINE_Y, high), (X_END, -LINE_Y, high), (X_START, -LINE_Y, high)]
    # Along the line, so that every wire is seen from within the AOPs' 7 m: over it (the pylons' tops 3.5 m
    # below), back beside it on the house side (y = 8, above the house and the wall), out again on the other
    # side (y = -7.5: the lamp post and the trunk stand at y = -10.75).
    w += [(5.0, 0.0, args.over_alt), (55.0, 0.0, args.over_alt), (55.0, 8.0, 10.0), (5.0, 8.0, 10.0),
          (5.0, -7.5, 10.0), (55.0, -7.5, 10.0)]
    # Up to the circle at its east point, once round anticlockwise, home over the line.
    n = args.circle_points
    w.append((55.0, -7.5, args.circle_alt))
    for k in range(n + 1):
        a = 2.0 * np.pi * k / n
        w.append((CENTRE[0] + args.radius * np.cos(a), CENTRE[1] + args.radius * np.sin(a), args.circle_alt))
    w.append((0.0, 0.0, args.circle_alt))
    return [np.array(p, dtype=float) for p in w]


def clearance(path, targets):
    """Smallest distance from the straight legs of path to each object of the targets file (m)."""
    legs = [(a, b) for a, b in zip(path[:-1], path[1:])]
    samples = np.vstack([a + np.linspace(0, 1, max(int(np.linalg.norm(b - a) / 0.25), 2))[:, None] * (b - a)
                         for a, b in legs])
    out = {}

    for name, t in targets.items():
        if t["type"] == "box":
            c, half = np.array(t["c"], float), np.array(t["size"], float) / 2.0
            d = np.linalg.norm(np.maximum(np.abs(samples - c) - half, 0.0), axis=1)
        else:
            if t["type"] == "pole":
                p0 = np.array(t["p"], float)
                p1 = p0 + [0.0, 0.0, t["h"]]
            else:
                p0, p1 = np.array(t["p0"], float), np.array(t["p1"], float)

            d01 = p1 - p0
            s = np.clip((samples - p0) @ d01 / (d01 @ d01), 0.0, 1.0)
            d = np.linalg.norm(samples - (p0 + s[:, None] * d01), axis=1) - t["r"]

        out[name] = float(d.min())

    return out


def newest_ulog():
    logs = glob.glob(os.path.join(ROOT, "run", "rootfs", "log", "*", "*.ulg"))
    return max(logs, key=os.path.getmtime) if logs else None


def link_radar_log(target, started):
    """Hard links in target to the radar recording of this session (run_sitl.sh RADAR_LOG=1), once the
    chunk under way at landing is written."""
    dirs = [d for d in glob.glob(os.path.join(ROOT, "run", "radar_log", "*")) if os.path.isdir(d)]
    current = max(dirs, key=os.path.getmtime) if dirs else None

    if current is None or os.path.getmtime(current) < started:
        print("no radar recording (run_sitl.sh RADAR_LOG=1 records one)")
        return

    time.sleep(CHUNK_WAIT)
    os.makedirs(target, exist_ok=True)

    for f in glob.glob(os.path.join(current, "*")):
        if f.endswith(".tmp.npz"):
            continue
        dst = os.path.join(target, os.path.basename(f))
        if os.path.exists(dst):
            os.remove(dst)
        os.link(f, dst)

    print(f"radar recording: {target} (from {current}, {len(glob.glob(os.path.join(target, 'chunk_*.npz')))} chunks)")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--tag", default="survey", help="name of the out/survey/<tag>.* files")
    p.add_argument("--speed", type=float, default=4.0, help="cruise speed (m/s)")
    p.add_argument("--low", type=float, default=6.0, help="altitude of the first lines (m)")
    p.add_argument("--high", type=float, default=12.0, help="altitude of the second lines (m)")
    p.add_argument("--circle-alt", type=float, default=16.5, help="altitude of the circle (m), above the pylons")
    p.add_argument("--radius", type=float, default=20.0, help="radius of the circle (m)")
    p.add_argument("--over-alt", type=float, default=16.5, help="altitude of the pass above the power line (m)")
    p.add_argument("--circle-points", type=int, default=24)
    p.add_argument("--low-pass-alt", type=float, default=3.0,
                   help="altitude of the low pass beside the house and the wall (m); 0: none (the route before)")
    p.add_argument("--min-clearance", type=float, default=3.0, help="refuse a path closer than this to an object (m)")
    p.add_argument("--timeout", type=float, default=600.0)
    p.add_argument("--print", action="store_true", help="print the waypoints and clearances, do not fly")
    args, extra = p.parse_known_args()     # the rest goes to avoid_node.py

    path = waypoints(args)
    with open(os.path.join(ROOT, "worlds", "survey_site.truth.yaml")) as f:
        targets = yaml.safe_load(f)

    # The legs start from the take-off point above the spawn.
    gaps = clearance([np.array([0.0, 0.0, 2.5])] + path, targets)
    length = sum(np.linalg.norm(b - a) for a, b in zip(path[:-1], path[1:]))
    print(f"{len(path)} waypoints, {length:.0f} m, about {length / args.speed:.0f} s at {args.speed:g} m/s; "
          f"closest object: " + ", ".join(f"{k} {v:.1f} m" for k, v in sorted(gaps.items(), key=lambda kv: kv[1])[:3]),
          flush=True)

    if args.print:
        for w in path:
            print(f"  {w[0]:6.1f} {w[1]:6.1f} {w[2]:5.1f}")
        return 0

    if min(gaps.values()) < args.min_clearance:
        print(f"the path passes within {args.min_clearance:g} m of an object: not flying", file=sys.stderr)
        return 2

    os.makedirs(os.path.join(ROOT, "out", "survey"), exist_ok=True)
    tag = os.path.join(ROOT, "out", "survey", args.tag)
    started = time.time()
    cmd = [sys.executable, "-u", os.path.join(HERE, "avoid_node.py"), "--world", "survey_site",
           "--speed", str(args.speed), "--timeout", str(args.timeout),
           "--waypoints"] + [",".join(f"{c:.2f}" for c in w) for w in path] + extra

    # Line-buffered: the flight's progress can be followed in <tag>.txt while it flies.
    with open(tag + ".txt", "w", buffering=1) as log:
        proc = subprocess.Popen(cmd, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

        for line in proc.stdout:
            sys.stdout.write(line)
            sys.stdout.flush()
            log.write(line)

        code = proc.wait()

    csv = os.path.join(ROOT, "out", "avoid.csv")
    if os.path.exists(csv):
        subprocess.run(["cp", csv, tag + ".csv"], check=False)

    ulog = newest_ulog()
    # This PX4 logs from boot to shutdown (logger mode all): the log still being written is this session's.
    if ulog and os.path.getmtime(ulog) >= started:
        if os.path.exists(tag + ".ulg"):
            os.remove(tag + ".ulg")
        os.link(ulog, tag + ".ulg")
        print(f"PX4 log: {tag}.ulg (from {ulog})")
    else:
        print("no PX4 log of this flight found in run/rootfs/log", file=sys.stderr)

    link_radar_log(tag + ".radar", started)
    return code


if __name__ == "__main__":
    sys.exit(main())
