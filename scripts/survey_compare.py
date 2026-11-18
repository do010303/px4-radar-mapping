#!/usr/bin/env python3
"""One line per survey flight, for comparing radar variants (docs/survey_mapping_plan.md, sensitivity tests).

    scripts/survey_compare.py out/survey/rtk_3 out/survey/sens_angle2 ...

Per flight: the radar variant (meta.yaml), whether the mission passed and how close it came; the data rate
while airborne as the radars send it (TI format; median, 95 %, max per second) and packed; the map (estimated
pose): share of points within 0.5 m of a true surface, median distance on objects, share further than 2 m from
all; the objects (scripts/survey_objects.py): poles and wires found with their largest error, the house and
the wall, found objects that match nothing.
"""

import argparse
import contextlib
import io
import os
import re
import sys
import warnings

import numpy as np

warnings.filterwarnings("ignore")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import survey_objects as so  # noqa: E402
import survey_process as sp  # noqa: E402
from radar_log import FRAME_BYTES_TI, POINT_BYTES_PACKED, POINT_BYTES_TI, load  # noqa: E402
from survey_check import base_positions  # noqa: E402


def variant(meta):
    s = meta.get("sensitivity") or {}
    parts = [f"angle x{s['angle_scale']:g}" if s.get("angle_scale", 1) != 1 else "",
             f"surfaces {s['surface_db']:+g} dB" if s.get("surface_db") else "",
             f"wires/poles {s['thin_db']:+g} dB" if s.get("thin_db") else "",
             f"points x{s['points_scale']:g}" if s.get("points_scale", 1) != 1 else "",
             f"{100 * s['angle_outlier']:g} % ghosts" if s.get("angle_outlier") else ""]
    return ", ".join(p for p in parts if p) or "baseline"


def rates(rec):
    """Airborne data rate per second (kbit/s, TI format) and the mean packed rate."""
    t = rec["stamp"] - rec["stamp"][0]
    sec = np.floor(t).astype(int)
    bits = np.bincount(sec, weights=rec["count"] * POINT_BYTES_TI + FRAME_BYTES_TI) * 8 / 1e3
    z = np.bincount(sec, weights=base_positions(rec)[:, 2]) / np.maximum(np.bincount(sec), 1)
    air = z > 2.0
    packed = np.bincount(sec, weights=rec["count"] * POINT_BYTES_PACKED + 8) * 8 / 1e3
    return bits[air], float(packed[air].mean())


def flight(tag):
    text = open(tag + ".txt").read() if os.path.exists(tag + ".txt") else ""
    result = re.search(r"^result:\s*(\S+)", text, re.M)
    closest = re.search(r"^closest\s+([0-9.]+) m to (\S+)", text, re.M)
    return (result.group(1) if result else "no result"), (f"{closest.group(1)} m ({closest.group(2)})" if closest else "")


def row(tag):
    rec = load(tag + ".radar")
    world = rec["meta"]["world"]
    truth = sp.load_truth(world)
    ok, closest = flight(tag)
    air, packed = rates(rec)

    with contextlib.redirect_stdout(io.StringIO()):
        pts = sp.place(rec, "estimate", sp.EstimatedPose(tag + ".ulg", world))
        d, labels, _ = sp.score(pts, truth)
        s = sp.summary(d, labels)
        objects, _ = so.extract(pts, np.random.default_rng(so.SEED), quiet=True)
    matched, extra = so.match(objects, truth)

    def found(kind):
        errs = [m[1] for n, m in matched.items() if truth[n]["type"] == kind and m is not None]
        total = sum(1 for t in truth.values() if t["type"] == kind)
        return f"{len(errs)}/{total}" + (f" (≤ {max(errs):.2f} m)" if errs else "")

    def seen(name):
        return f"{matched[name][1]:.1f} m off" if matched[name] is not None else "no"

    return [os.path.basename(tag), variant(rec["meta"]), ok, closest,
            f"{np.median(air):.0f} / {np.percentile(air, 95):.0f} / {air.max():.0f}", f"{packed:.0f}",
            f"{100 * s['near']:.0f} %", f"{s['median']:.2f}", f"{100 * s['stray']:.1f} %",
            found("pole"), found("wire"), seen("house"), seen("wall"), str(len(extra))]


HEADER = ["flight", "radar variant", "mission", "closest", "kbit/s airborne, TI (median / 95 % / max)", "packed",
          "within 0.5 m", "median on objects (m)", "> 2 m", "poles", "wires", "house", "wall", "false objects"]


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("tags", nargs="+", help="out/survey/<tag> (without extension)")
    args = p.parse_args()
    print("| " + " | ".join(HEADER) + " |")
    print("|" + "---|" * len(HEADER))

    for tag in args.tags:
        try:
            print("| " + " | ".join(row(tag)) + " |", flush=True)
        except (OSError, ValueError, StopIteration) as e:
            print(f"| {os.path.basename(tag)} | could not be read: {e} |", flush=True)

    return 0


if __name__ == "__main__":
    sys.exit(main())
