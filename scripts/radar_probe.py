#!/usr/bin/env python3
"""Static radar bench: one frame of worlds/radar_test.sdf, as a table and a plot.

The radar stands still on the model radar_stand; its obstacles are read from
the world file (scene.py). The one captured ray grid is run through the radar
model --frames times with fresh noise, which gives each obstacle's detection
rate. The radar model itself is in radar_model.py; radar_viz.py shows it live.

Usage (Gazebo already running the radar_test world, see run_radar_test.sh):
    GZ_PARTITION=px4_mapping_radar ./radar_probe.py --plot out.png [--world radar_test]
"""

import argparse
import os
import sys
import threading

import numpy as np

import plotstyle
from radar_model import add_model_args, detect, scan_to_grid    # first: sets up the gz bindings
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.transport13 import Node
from scene import classify, load_obstacles, model_position

WORLDS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "worlds")
STAND = "radar_stand"


def report(frames, origin, obstacles, args):
    """frames: detections in the world frame, one array per noise realisation."""
    first = classify(frames[0][:, :3], origin, obstacles, args.sigma_el)
    det = frames[0]
    print(f"detections: {len(det)} ({first.ground.sum()} ground, {first.unexplained.sum()} unexplained), "
          f"SNR {det[:, 4].min():.0f}..{det[:, 4].max():.0f} dB")
    print(f"\n {'obstacle':<14}{'range':>7}{'seen in':>10}{'points':>8}{'measured range':>18}{'measured z':>16}"
          f"\n {'':<14}{'':>7}{len(frames):>4} frames{'(this frame)':>14}")

    labels = [classify(f[:, :3], origin, obstacles, args.sigma_el) for f in frames]

    for o in obstacles:
        seen = np.mean([lab.on[o.name].any() for lab in labels]) * 100
        sel = first.on[o.name]
        pts = det[sel, :3]
        line = f" {o.name:<14}{o.surface_distance(origin)[0]:6.1f}m{seen:9.0f}%{sel.sum():8d}"

        if sel.any():
            rng = np.linalg.norm(pts - origin, axis=1)
            line += f"{rng.mean():10.2f} ± {rng.std():4.2f} m{pts[:, 2].min():7.2f}..{pts[:, 2].max():.2f} m"
        else:
            line += "      not detected"

        print(line)


def plot(path, det, origin, obstacles, args):
    pts = det[:, :3]
    labels = classify(pts, origin, obstacles, args.sigma_el)
    plt, fig, (top, side) = plotstyle.figure(2, [2.2, 1], (9, 9))
    # The bench of the short-range radars fits in 13 m; a longer one scales the axes.
    reach = max(13.0, 1.05 * max(o.surface_distance(origin)[0] for o in obstacles))

    for ax, col, ylabel in ((top, 1, "y (m), + = left"), (side, 2, "Height z (m)")):
        plotstyle.style_axes(ax, ylabel=ylabel)
        plotstyle.scatter_labels(ax, pts, labels, col, size=18)
        ax.plot(origin[0], origin[col], marker="o", ms=8, color=plotstyle.INK, label="Radar")
        ax.set_xlim(-13 if reach <= 13 else -0.05 * reach, reach)

    plotstyle.draw_obstacles(plt, top, side, obstacles)
    top.set_aspect("equal")
    top.set_ylim(-9 * reach / 13, 9 * reach / 13)
    plotstyle.title(top, "Emulated 3D mmWave radar, one frame (top view, then side view)")
    plotstyle.legend(top, loc="upper left")
    side.set_ylim(-0.6, 4.5) if reach <= 13 else side.set_ylim(-5, 8)
    side.set_xlabel("x (m); grey outlines are the obstacles", color=plotstyle.INK2)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print(f"\nplot saved to {path}")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--topic", default="/radar/scan")
    p.add_argument("--world", default="radar_test", help="worlds/<name>.sdf, with the model radar_stand")
    add_model_args(p)
    p.add_argument("--frames", type=int, default=50,
                   help="noise realisations of the captured frame used for the detection-rate column")
    p.add_argument("--timeout", type=float, default=30.0)
    p.add_argument("--skip", type=int, default=5, help="scans to discard while rendering warms up")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--plot", help="write a PNG of one frame")
    args = p.parse_args()

    world = os.path.join(WORLDS, args.world + ".sdf")
    obstacles = load_obstacles(world)
    origin = model_position(world, STAND)
    rng = np.random.default_rng(args.seed)
    node = Node()
    got = threading.Event()
    state = {"n": 0, "msg": None}

    def cb(msg):
        state["n"] += 1

        if state["n"] > args.skip and not got.is_set():
            state["msg"] = msg
            got.set()

    if not node.subscribe(LaserScan, args.topic, cb):
        sys.exit(f"failed to subscribe to {args.topic}")

    if not got.wait(args.timeout):
        sys.exit(f"no LaserScan on {args.topic} within {args.timeout:g} s "
                 "(is Gazebo running in the same GZ_PARTITION, and is rendering available?)")

    ranges, az, el = scan_to_grid(state["msg"])
    print(f"ray grid: {len(az)} az x {len(el)} el over "
          f"{np.degrees(az[-1] - az[0]):.0f} x {np.degrees(el[-1] - el[0]):.0f} deg, "
          f"{np.isfinite(ranges).sum()} of {ranges.size} rays returned")

    # The stand does not rotate: radar frame = world frame shifted by the stand's position.
    frames = []

    for _ in range(args.frames):
        det = detect(ranges, az, el, args, rng)
        det[:, :3] += origin
        frames.append(det)

    if len(frames[0]) == 0:
        sys.exit("no detections above threshold")

    report(frames, origin, obstacles, args)

    if args.plot:
        plot(args.plot, frames[0], origin, obstacles, args)


if __name__ == "__main__":
    main()
    sys.stdout.flush()
    os._exit(0)
