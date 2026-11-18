#!/usr/bin/env python3
"""The test matrix of docs/survey_mapping_plan.md, phase 6: the objects found after the flight (survey_objects)
per recorded flight and per pose and added error, against the truth.

Variants (survey_process.place): the true pose, the estimated one (RTK or standard GPS, as flown), and on the
estimated one a radar clock late by 50 / 100 ms and every mount off by 2 / 5 deg.

    scripts/survey_matrix.py out/survey/live_5 out/survey/live_6 out/survey/std_5 > out/survey/matrix.md
"""

import argparse
import contextlib
import io
import os
import sys
import warnings

import numpy as np

warnings.filterwarnings("ignore")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import survey_objects as so  # noqa: E402
import survey_process as sp  # noqa: E402
from radar_log import load  # noqa: E402

VARIANTS = (
    ("true pose", dict(pose="truth")),
    ("estimated pose", dict()),
    ("+ clock 50 ms", dict(clock_offset=0.05)),
    ("+ clock 100 ms", dict(clock_offset=0.10)),
    ("+ mounts 2°", dict(mount_error=2.0)),
    ("+ mounts 5°", dict(mount_error=5.0)),
)


def evaluate(objects, truth):
    """Per kind: found / total and the largest error; blocks by name; false objects."""
    matched, extra = so.match(objects, truth)
    out = {}
    for kind in ("pole", "wire", "box"):
        names = [n for n, t in truth.items() if t["type"] == kind and not n.startswith("crossarm")]
        errs = [matched[n][1] for n in names if matched[n] is not None]
        out[kind] = (len(errs), len(names), max(errs) if errs else None)
    out["false"] = len(extra)
    return out


def cell(found, total, worst):
    return f"{found}/{total}" + (f" (≤ {worst:.2f})" if worst is not None else "")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("tags", nargs="+")
    args = p.parse_args()

    print("| flight | pose / error added | poles (≤ m) | wires (≤ m) | house, wall (≤ m) | false |")
    print("|---|---|---|---|---|---|")
    for tag in args.tags:
        rec = load(tag + ".radar")
        truth = sp.load_truth(rec["meta"]["world"])
        with contextlib.redirect_stdout(io.StringIO()):
            ulog = sp.EstimatedPose(tag + ".ulg", rec["meta"]["world"])
        for label, kw in VARIANTS:
            kw = dict(kw)
            pose = kw.pop("pose", "estimate")
            with contextlib.redirect_stdout(io.StringIO()):
                pts = sp.place(rec, pose, ulog, quiet=True, **kw)
                objects = so.extract(pts, np.random.default_rng(so.SEED), quiet=True)[0]
            r = evaluate(objects, truth)
            print(f"| {os.path.basename(tag)} | {label} | {cell(*r['pole'])} | {cell(*r['wire'])} | {cell(*r['box'])} | "
                  f"{r['false']} |", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
