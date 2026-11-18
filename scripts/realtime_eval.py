#!/usr/bin/env python3
"""The real-time map over the link, end to end, on recorded flights (docs/survey_mapping_plan.md, phase 5).

Per flight and link rate: the recording replayed through onboard_mapper (on board) -> radar_link.Sender
(the link) -> ground_map.GroundMap (the ground), and
  - the link's data rate per second (incl. UDP/IP headers) and the share of the detections that got through
  - per true object: how long after the radar saw it (detections within SEEN of its surface in SIGHTINGS
    frames) the ground map had it (a confirmed voxel above the ground within SEEN of it)
  - the objects found in the ground map (survey_objects' finders) against the truth, and those found after
    the flight from the full recording (survey_objects.extract), for comparison

    scripts/realtime_eval.py out/survey/rtk_3 out/survey/sens_bright --kbps 300 50
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
import ground_map as gmap  # noqa: E402
import onboard_mapper as om  # noqa: E402
import radar_link as rl  # noqa: E402
import survey_objects as so  # noqa: E402
import survey_process as sp  # noqa: E402
from radar_log import load  # noqa: E402

SEEN = 1.0                  # m from a true surface
SIGHTINGS = 3               # frames with a detection near an object before it counts as seen: a single stray
                            # point (the spread of a nearby pylon, say) long before the pass that maps it is
                            # not when the radar saw it


def distances(points, truth):
    """(n, objects) distance of points to each true object's surface."""
    return np.column_stack([sp.surface_distance(points, t) for t in truth.values()])


def found(objects, truth):
    matched, extra = so.match(objects, truth)
    out = {}
    for kind in ("pole", "wire"):
        errs = [m[1] for n, m in matched.items() if truth[n]["type"] == kind and m is not None]
        total = sum(1 for t in truth.values() if t["type"] == kind)
        out[kind] = f"{len(errs)}/{total}" + (f" (≤ {max(errs):.2f})" if errs else "")
    out["false"] = len(extra)
    return out


def run(tag, kbps, link_latency=gmap.LINK_LATENCY):
    rec = load(tag + ".radar")
    truth = sp.load_truth(rec["meta"]["world"])
    names = list(truth)
    first_seen = {}
    sightings = {}
    first_ground = {}
    ground = gmap.GroundMap()
    sent = []

    def transport(now, data):
        t_recv = now + link_latency
        sent.append((now, len(data) + rl.HEADER_UDP))
        kind = ground.handle(data, t_recv)
        if kind == rl.VOXEL:
            _, _, r = rl.decode(data)
            above = r[r["flags"] == 0]
            if len(above):
                d = distances((above["k"] + 0.5) * om.VOXEL, truth)
                for j in np.flatnonzero((d <= SEEN).any(axis=0)):
                    first_ground.setdefault(names[j], t_recv)

    sender = rl.Sender(kbps, transport=transport)

    def on_frame(now, stamp, points, report, mapper):
        if len(points):
            d = distances(points[:, :3], truth)
            for j in np.flatnonzero((d <= SEEN).any(axis=0)):
                sightings[names[j]] = sightings.get(names[j], 0) + 1
                if sightings[names[j]] == SIGHTINGS:
                    first_seen[names[j]] = stamp
        sender.on_frame(now, stamp, points, report, mapper)

    om.replay(tag, 0.0, on_frame, quiet=True)

    t = np.array([s[0] for s in sent])
    size = np.array([s[1] for s in sent])
    kb = np.bincount(np.floor(t - t[0]).astype(int), weights=size) * 8 / 1e3
    # Negative when a voxel near the object reached the ground before its third sighting (from the spread of
    # another object's detections): counted as 0.
    delays = {n: max(first_ground[n] - first_seen[n], 0.0) for n in names if n in first_seen and n in first_ground}

    with contextlib.redirect_stdout(io.StringIO()):
        objects = ground.objects()[0]
        pts = sp.place(rec, "estimate", sp.EstimatedPose(tag + ".ulg", rec["meta"]["world"]))
        offline = so.extract(pts, np.random.default_rng(so.SEED), quiet=True)[0]

    return {"kbps": kb, "points": sender.sent[rl.POINTS] / max(sender.points_in, 1), "delays": delays,
            "never": [n for n in names if n in first_seen and n not in first_ground],
            "unseen": [n for n in names if n not in first_seen],
            "ground": found(objects, truth), "offline": found(offline, truth)}


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("tags", nargs="+")
    p.add_argument("--kbps", type=float, nargs="+", default=[300.0, 50.0])
    args = p.parse_args()

    print("| flight | link (kbit/s) | sent per s (median / 95 % / max) | detections sent | on the ground map after the "
          "radar saw it in 3 frames (median / max s) | never reached the ground | poles: ground / after flight | "
          "wires: ground / after flight | false: ground / after flight |")
    print("|" + "---|" * 9)
    detail = []

    for tag in args.tags:
        for kbps in args.kbps:
            r = run(tag, kbps)
            kb = r["kbps"][r["kbps"] > 0]
            d = np.array(list(r["delays"].values()))
            print(f"| {os.path.basename(tag)} | {kbps:g} | {np.median(kb):.0f} / {np.percentile(kb, 95):.0f} / {kb.max():.0f} | "
                  f"{100 * r['points']:.0f} % | {np.median(d):.1f} / {d.max():.1f} | {', '.join(r['never']) or '-'} | "
                  f"{r['ground']['pole']} / {r['offline']['pole']} | {r['ground']['wire']} / {r['offline']['wire']} | "
                  f"{r['ground']['false']} / {r['offline']['false']} |", flush=True)
            detail.append((tag, kbps, r))

    print()
    for tag, kbps, r in detail:
        print(f"{os.path.basename(tag)} at {kbps:g} kbit/s, per object (s): " +
              ", ".join(f"{n} {v:.1f}" for n, v in sorted(r["delays"].items(), key=lambda kv: kv[1])) +
              (f"; never seen by the radar: {', '.join(r['unseen'])}" if r["unseen"] else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
