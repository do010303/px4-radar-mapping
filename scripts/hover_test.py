#!/usr/bin/env python3
"""Hover test: does the radar on x500_radar see the obstacles of worlds/radar_hover?

Against a running ./run_sitl.sh: takes off (commander takeoff), waits for a
steady hover, records the radar for --duration seconds, lands, and scores every
frame against the obstacles of the world file (scene.py), using Gazebo ground
truth for the radar's pose:

  seen in      share of frames with at least one detection that came from the
               obstacle (the simulation knows each detection's true origin, so
               ground returns next to a far obstacle do not count for it)
  error        distance of those detections from the obstacle's surface
  ground       detections consistent with the ground (ground returns)
  unexplained  other detections on no obstacle (false alarms)

Each obstacle listed in worlds/<world>.targets.yaml has a minimum detection
rate there, and a maximum mean error that grows with range; the test passes
when every listed obstacle meets both. Listed obstacles outside the field of
view of the active radar profile (e.g. behind a forward-looking radar) are
reported but not scored.

Writes out/hover_test.png (all detections, top and side view) and
out/hover_test_frames.csv (one row per frame).

    source env.sh && ./scripts/hover_test.py
    ./scripts/hover_test.py --no-fly      # score only, vehicle already hovering

Exit codes:
  0  all obstacles pass
  1  at least one obstacle fails its limits
  2  setup problem (no simulation found, takeoff refused, ...)
  3  radar stalled: no scan for --stall-timeout seconds while the simulation ran
  4  simulation stopped (run_sitl.sh stops it when another simulation starts)
"""

import argparse
import csv
import os
import subprocess
import sys
import threading
import time

import numpy as np

import plotstyle
from radar_model import add_model_args, load_profile
from radar_stream import RadarStream
from scene import classify, in_view, load_obstacles

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PX4_DIR = os.path.normpath(os.path.join(ROOT, "..", ".."))
_OWN_BUILD = os.path.join(PX4_DIR, "build", "px4_sitl_mapping")    # scripts/build_px4.sh, else the shared build
PX4_BIN = os.path.join(os.environ.get("PX4_BUILD") or (_OWN_BUILD if os.path.exists(os.path.join(_OWN_BUILD, "bin", "px4"))
                                                       else os.path.join(PX4_DIR, "build", "px4_sitl_default")), "bin")

# Mean error limit, widened with range like the match distance in scene.py.
MAX_ERROR = 0.3
MAX_ERROR_PER_M = 0.035

# Movement during the recording above which the hover is reported as not steady.
MAX_DRIFT = 0.5

EXIT_FAIL, EXIT_SETUP, EXIT_STALLED, EXIT_STOPPED = 1, 2, 3, 4


class Abort(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def px4(*cmd):
    return subprocess.run([os.path.join(PX4_BIN, f"px4-{cmd[0]}"), *cmd[1:]],
                          capture_output=True, text=True, timeout=20)


class Watch:
    """Recorder of radar frames that also notices a stalled radar or a stopped simulation."""

    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.latest = None
        self.frames = []
        self.recording = False
        self.stream = RadarStream(args, args.world, args.model, self._on_frame, seed=args.seed)
        rootfs = os.path.join(ROOT, "run", "rootfs")
        world = os.path.join(ROOT, "worlds", f"{args.world}.sdf")
        self.processes = {
            "PX4": f"^{PX4_BIN}/px4 .*-w {rootfs}",
            "Gazebo": f"^px4_mapping_gazebo .* {world}",
        }

    def _on_frame(self, frame):
        with self.lock:
            self.latest = frame

            if self.recording:
                self.frames.append(frame)

    def check(self):
        """Raise Abort if the simulation is gone or the radar has stopped."""
        for name, pattern in self.processes.items():
            if subprocess.run(["pgrep", "-f", pattern], capture_output=True).returncode != 0:
                raise Abort(EXIT_STOPPED, f"{name} of ./run_sitl.sh is not running (it stops itself "
                                          "when another simulation starts; see run/run_sitl.out)")

        last = self.stream.last_scan

        if last is not None and time.monotonic() - last > self.args.stall_timeout:
            raise Abort(EXIT_STALLED, f"radar stalled: no scan for {time.monotonic() - last:.1f} s "
                                      "while PX4 and Gazebo are running")

    def wait_for(self, what, condition, timeout):
        t0 = time.monotonic()

        while time.monotonic() - t0 < timeout:
            self.check()

            with self.lock:
                frame = self.latest

            if frame is not None and condition(frame):
                return frame

            time.sleep(0.2)

        raise Abort(EXIT_SETUP, f"timed out waiting for {what}")

    def sleep(self, seconds):
        t_end = time.monotonic() + seconds

        while time.monotonic() < t_end:
            self.check()
            time.sleep(min(0.5, max(t_end - time.monotonic(), 0.0)))


def fly_to_hover(watch, args):
    watch.wait_for("radar frames", lambda f: True, 60)
    print("radar frames arriving; waiting for PX4 to accept takeoff", flush=True)
    t0 = time.monotonic()

    while True:
        out = px4("commander", "takeoff")
        watch.sleep(3.0)

        with watch.lock:
            z = watch.latest.position[2]

        if z > 0.6:
            break

        if time.monotonic() - t0 > 90:
            raise Abort(EXIT_SETUP, f"vehicle did not take off: {out.stdout.strip()} {out.stderr.strip()}")

    print("taking off", flush=True)
    watch.wait_for("hover altitude", lambda f: f.position[2] > args.altitude - 0.3, 40)

    # Steady: slow for 3 s in a row.
    steady_since = None

    while True:
        watch.check()

        with watch.lock:
            frame = watch.latest

        if np.linalg.norm(frame.velocity) < 0.15:
            steady_since = steady_since or time.monotonic()

            if time.monotonic() - steady_since > 3.0:
                break
        else:
            steady_since = None

        time.sleep(0.2)

    print(f"hovering at {frame.position[2]:.2f} m", flush=True)


def land(watch):
    px4("commander", "land")
    print("landing", flush=True)
    watch.wait_for("landing", lambda f: f.position[2] < 0.4, 60)


def score(frames, obstacles, args):
    """Print the per-obstacle table; return (per-frame rows, all scored obstacles passed)."""
    rows = []
    stats = {o.name: {"seen": 0, "errors": [], "points": 0} for o in obstacles}
    ground = unexplained = total = no_plane = near = 0

    for f in frames:
        pts = f.world_points()
        total += len(pts)
        labels = classify(pts, f.position, obstacles, args.sigma_el, truth=f.world_truth())
        ground += int(labels.ground.sum())
        unexplained += int(labels.unexplained.sum())
        no_plane += labels.plane is None
        # The vehicle's own legs and body sit within --min-range; anything left
        # this close is most likely still the airframe.
        near += int((np.linalg.norm(pts - f.position, axis=1) < 1.0).sum())
        row = {"t": f.stamp, "x": f.position[0], "y": f.position[1], "z": f.position[2],
               "detections": len(pts), "ground": int(labels.ground.sum())}

        for name, mask in labels.on.items():
            row[name] = int(mask.sum())
            stats[name]["points"] += int(mask.sum())

            if mask.any():
                stats[name]["seen"] += 1
                stats[name]["errors"].extend(labels.distance[name][mask])

        row["unexplained"] = int(labels.unexplained.sum())
        rows.append(row)

    n = len(frames)
    sensor = load_profile()["sensor"]
    pos = np.array([f.position for f in frames])
    origin = pos.mean(axis=0)
    duration = frames[-1].stamp - frames[0].stamp
    print(f"\n{n} frames over {duration:.1f} s sim time ({n / max(duration, 1e-6):.1f} Hz), "
          f"{total / n:.0f} detections/frame")
    print(f"radar position during the hover: mean ({origin[0]:+.2f}, {origin[1]:+.2f}, "
          f"{origin[2]:.2f}) m, std {pos.std(axis=0).max():.2f} m")

    if np.ptp(pos, axis=0).max() > MAX_DRIFT:
        print(f"WARNING: the vehicle moved {np.ptp(pos, axis=0).max():.1f} m during the recording "
              f"(z {pos[:, 2].min():.1f} to {pos[:, 2].max():.1f} m): not a steady hover, e.g. another "
              "takeoff or position command arrived meanwhile")
    print(f"\n {'obstacle':<15}{'range':>7}{'seen in':>9}{'(min)':>7}{'points/frame':>14}"
          f"{'mean error':>12}{'(max)':>8}   result")

    passed = True

    for o in obstacles:
        st = stats[o.name]
        rng = o.surface_distance(origin)[0]
        rate = st["seen"] / n
        err = float(np.mean(st["errors"])) if st["errors"] else float("nan")
        err_limit = MAX_ERROR + MAX_ERROR_PER_M * rng
        err_txt = f"{err:9.2f} m" if st["errors"] else "        --"

        if o.min_rate is None:
            verdict, limits = "(not scored)", f"{'':6}{'':8}"
        elif not in_view(o, origin, frames[-1].rotation, sensor):
            verdict, limits = "(out of view)", f"{'':6}{'':8}"
        else:
            ok = rate >= o.min_rate and (not st["errors"] or err <= err_limit)
            passed &= ok
            verdict, limits = ("pass" if ok else "FAIL"), f"{o.min_rate * 100:5.0f}%{err_limit:7.2f}m"

        print(f" {o.name:<15}{rng:6.1f}m{rate * 100:8.0f}%{limits[:6]}{st['points'] / n:14.1f}"
              f"{err_txt:>12}{limits[6:]:>8}   {verdict}")

    print(f"\n ground returns  {ground / n:6.1f} per frame"
          f"{f' (no ground plane found in {no_plane} frames)' if no_plane else ''}")
    print(f" unexplained     {unexplained / n:6.1f} per frame (on no obstacle and not ground)")
    print(f" within 1 m      {near / n:6.1f} per frame (likely the vehicle itself)")
    print(f"\nresult: {'PASS' if passed else 'FAIL'}")
    return rows, passed


def plot(path, frames, obstacles, args):
    pts = np.concatenate([f.world_points() for f in frames])
    truth = np.concatenate([f.world_truth() for f in frames])
    pos = np.array([f.position for f in frames]).mean(axis=0)
    labels = classify(pts, pos, obstacles, args.sigma_el, truth=truth)
    reach = load_profile()["sensor"]["range_max"]
    lim = 1.04 * reach
    plt, fig, (top, side) = plotstyle.figure(2, [2.6, 1], (9, 11))

    for ax, col, ylabel in ((top, 1, "y (m), + = left"), (side, 2, "Height z (m)")):
        plotstyle.style_axes(ax, ylabel=ylabel)
        plotstyle.scatter_labels(ax, pts, labels, col)
        ax.plot(pos[0], pos[col], marker="o", ms=7, color=plotstyle.INK, label="Radar (hover)")
        ax.set_xlim(-lim, lim)

    plotstyle.draw_obstacles(plt, top, side, obstacles, label=lambda o: np.hypot(*o.centre[:2]) > 0.3 * reach)
    top.add_patch(plt.Circle((pos[0], pos[1]), reach, fill=False, ec=plotstyle.FAINT, ls="--", lw=1))
    top.annotate(f"{reach:g} m", (pos[0] + 0.71 * reach, pos[1] + 0.71 * reach), color=plotstyle.INK2, fontsize=9)
    top.set_aspect("equal")
    top.set_ylim(-lim, lim)
    plotstyle.title(top, f"Radar detections over a {frames[-1].stamp - frames[0].stamp:.0f} s hover "
                         f"({len(frames)} frames); grey outlines are the obstacles")
    plotstyle.legend(top)
    side.set_ylim(-2, max(10, max(o.centre[2] + o.size[2] / 2 for o in obstacles) + 2))
    side.set_xlabel("x (m)", color=plotstyle.INK2)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print(f"plot saved to {path}")


def run(args):
    if "GZ_PARTITION" not in os.environ:
        raise Abort(EXIT_SETUP, "GZ_PARTITION is not set: source env.sh first (same partition as ./run_sitl.sh)")

    world_sdf = os.path.join(ROOT, "worlds", f"{args.world}.sdf")
    obstacles = load_obstacles(world_sdf)
    watch = Watch(args)
    watch.check()

    if not args.no_fly:
        fly_to_hover(watch, args)

    with watch.lock:
        watch.recording = True

    print(f"recording {args.duration:g} s", flush=True)
    watch.sleep(args.duration)

    with watch.lock:
        watch.recording = False
        frames = list(watch.frames)

    if not args.no_fly:
        land(watch)

    if not frames:
        raise Abort(EXIT_STALLED, "no radar frames recorded")

    rows, passed = score(frames, obstacles, args)
    out = os.path.join(ROOT, "out")
    os.makedirs(out, exist_ok=True)

    with open(os.path.join(out, "hover_test_frames.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    plot(os.path.join(out, "hover_test.png"), frames, obstacles, args)
    return 0 if passed else EXIT_FAIL


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--world", default="radar_hover")
    p.add_argument("--model", default="x500_radar_0")
    add_model_args(p)
    p.add_argument("--altitude", type=float, default=2.5, help="expected hover altitude (MIS_TAKEOFF_ALT)")
    p.add_argument("--duration", type=float, default=20.0, help="seconds of hover to record")
    p.add_argument("--stall-timeout", type=float, default=2.0,
                   help="seconds without a radar scan that count as a stalled radar")
    p.add_argument("--no-fly", action="store_true", help="do not take off or land, only record and score")
    p.add_argument("--seed", type=int, default=1)
    args = p.parse_args()

    try:
        return run(args)
    except Abort as e:
        print(f"\nABORTED: {e}", file=sys.stderr)
        return e.code


if __name__ == "__main__":
    code = main()
    # Skip interpreter teardown: destroying the gz-transport node while its
    # threads still deliver messages aborts the process.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
