#!/usr/bin/env python3
"""The radar map built on board, frame by frame, as it flies (docs/survey_mapping_plan.md, phase 5).

What the companion computer does with each radar frame as it arrives, knowing only the past:

  1. the pose: PX4's estimate at the scan time, from the pose samples that have reached it by then
     (CausalPose: received at most --pose-delay after they were taken; a scan newer than the newest
     sample is placed by extrapolating that sample with its velocity)
  2. the frame is skipped below MIN_ALT (estimated height above the take-off point) or while EKF2's
     heading sigma is above MAX_HEADING_SIGMA (scripts/survey_process.py, phase 2)
  3. the detections are placed in the world, Doppler-gated, and added to a voxel map (VOXEL): per voxel
     the hits, the frames, the box of the radar positions it was seen from, the sum of the points
  4. a voxel is confirmed once seen in MIN_FRAMES frames from places MIN_BASELINE apart (the ghost filter
     of scripts/survey_objects.py, phase 3, applied as the data comes in); voxels within GROUND_BAND of
     the take-off height are marked as ground

The map's changes (newly confirmed voxels, and confirmed voxels whose hits have doubled since they were
last reported) are what goes to the ground (scripts/ground_map.py, over a 300 kbit/s link).

Replays a recorded survey flight (out/survey/<tag>.radar and .ulg) at the radars' pace or as fast as it
can, and compares the map with the one made after the flight from the same data (survey_objects.py's
ground and ghost filters):

    scripts/onboard_mapper.py out/survey/rtk_3                 # as fast as possible
    scripts/onboard_mapper.py out/survey/rtk_3 --speed 1       # in real time

Writes <tag>.onboard.ply (confirmed voxels: centroid, hits) and <tag>.onboard.png.
"""

import argparse
import os
import sys
import time
import warnings

import numpy as np

warnings.filterwarnings("ignore", message="A NumPy version")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import survey_process as sp  # noqa: E402
from radar_log import load  # noqa: E402

# The C++ core (cpp/mapper_core.cpp, built by cpp/build.sh), unless absent or ONBOARD_PY=1.
try:
    import mapper_core  # noqa: E402
except ImportError:
    mapper_core = None
if os.environ.get("ONBOARD_PY") == "1":
    mapper_core = None

VOXEL = 0.5                 # m (as survey_objects.VOXEL)
MIN_FRAMES = 3
MIN_BASELINE = 2.0          # m
GROUND_BAND = 0.6           # m above the take-off height
MIN_ALT = sp.MIN_ALT
MAX_HEADING_SIGMA = sp.MAX_HEADING_SIGMA
NEAR_RANGE = 5.0            # m: a detection from this close or closer weighs 1, from further (R0 / r)^2 (its
                            # angle error moves it R times the angle: the near ones say where things are)
RADAR_LATENCY = 0.05        # s from the scan to its frame on the companion computer (UART, parsing)
POSE_DELAY = 0.02           # s from PX4's estimate to the companion computer (MAVLink)


class CausalPose:
    """PX4's estimated pose as the companion computer has it at a given moment."""

    def __init__(self, estimated, pose_delay=POSE_DELAY):
        self.e = estimated
        self.delay = pose_delay
        self.cpp = None

        if mapper_core is not None and hasattr(estimated, "q"):     # (a test's stand-in has only at())
            # The quaternions in one hemisphere, as survey_process.interp_quat makes them.
            q = estimated.q.copy()
            flip = np.r_[False, np.sum(q[1:] * q[:-1], axis=1) < 0]
            q[np.cumsum(flip) % 2 == 1] *= -1.0
            e = estimated
            self.cpp = mapper_core.PoseTrack(e.tp, e.p_ned, e.v_ned, e.tq, q, e.tw, e.w_frd, e.ts, e.heading_sigma,
                                             e.origin, pose_delay)

    @property
    def origin(self):
        """EKF2's origin in the world (ENU m), and geodetic (lat, lon, alt): what radar_link sends as ORIGIN."""
        return getattr(self.e, "origin", None)

    @property
    def geo(self):
        return getattr(self.e, "geo", None)

    def at(self, t, now):
        """(position, rotation, velocity, rate, heading sigma) of base_link at scan time t, knowing the pose
        samples taken up to now - delay."""
        if self.cpp is not None:
            return self.cpp.at(t, now)

        e = self.e
        known = now - self.delay
        # The newest sample of every stream that has arrived; the pose is interpolated if t lies before all
        # of them, else the newest is carried forward to t.
        last = min(e.tp[np.searchsorted(e.tp, known, "right") - 1], e.tq[np.searchsorted(e.tq, known, "right") - 1],
                   e.tw[np.searchsorted(e.tw, known, "right") - 1])
        sigma = float(e.heading_sigma[max(np.searchsorted(e.ts, known, "right") - 1, 0)])

        if t <= last:
            p, r, v, w = e.at(np.array([t]))
            return p[0], r[0], v[0], w[0], sigma

        p, r, v, w = e.at(np.array([last]))
        return p[0] + v[0] * (t - last), r[0], v[0], w[0], sigma


class VoxelMap:
    """Voxels of detections, with the ghost filter applied as they fill."""

    def __init__(self, voxel=VOXEL, ground_z=0.0):
        self.voxel = voxel
        self.ground_z = ground_z
        # key -> [hits, frames, last frame, sum x, y, z, sensor min (3), sensor max (3), confirmed, hits sent,
        #         weight (the sum of the detections' NEAR_RANGE weights)]
        self.cells = {}

    def add(self, frame, points, sensor):
        """Add one frame's points (n, 3) seen from sensor (3,); returns the keys to report: confirmed
        just now, or confirmed with twice the hits since they were last reported."""
        report = []
        keys = np.floor(points / self.voxel).astype(np.int64)
        weights = (NEAR_RANGE / np.maximum(np.linalg.norm(points - sensor, axis=1), NEAR_RANGE)) ** 2

        for key, p, wt in zip(map(tuple, keys), points, weights):
            c = self.cells.get(key)

            if c is None:
                c = self.cells[key] = [0, 0, -1, 0.0, 0.0, 0.0, sensor.copy(), sensor.copy(), False, 0, 0.0]

            c[10] += wt
            c[0] += 1
            if c[2] != frame:
                c[1] += 1
                c[2] = frame
            c[3] += p[0]
            c[4] += p[1]
            c[5] += p[2]
            np.minimum(c[6], sensor, out=c[6])
            np.maximum(c[7], sensor, out=c[7])

            if not c[8]:
                if c[1] >= MIN_FRAMES and np.max(c[7] - c[6]) >= MIN_BASELINE:
                    c[8] = True
                    c[9] = c[0]
                    report.append(key)
            elif c[0] >= 2 * c[9]:
                c[9] = c[0]
                report.append(key)

        return report

    def __len__(self):
        return len(self.cells)

    def records(self, keys):
        """(hits, weight, mean z) of the voxels with these keys (radar_link's voxel records)."""
        cells = [self.cells[tuple(k)] for k in keys]
        return (np.array([c[0] for c in cells], np.int64), np.array([c[10] for c in cells]),
                np.array([c[5] / c[0] for c in cells]))

    def confirmed(self):
        """(keys (n, 3), centroids (n, 3), hits (n,), ground (n,)) of the confirmed voxels."""
        rows = [(k, c) for k, c in self.cells.items() if c[8]]

        if not rows:
            return np.empty((0, 3), int), np.empty((0, 3)), np.empty(0, int), np.empty(0, bool)

        keys = np.array([k for k, _ in rows])
        hits = np.array([c[0] for _, c in rows])
        cen = np.array([[c[3], c[4], c[5]] for _, c in rows]) / hits[:, None]
        return keys, cen, hits, cen[:, 2] < self.ground_z + GROUND_BAND


class OnboardMapper:
    """Radar frames in, map changes out."""

    def __init__(self, meta, pose, doppler_gate=3.0, home_z=None):
        from radar_array import doppler_consistent

        self.gate_fn = doppler_consistent
        self.pose = pose
        self.gate = doppler_gate
        self.cfgs = sp.radar_cfgs(meta)
        self.r_mount = [np.array(r["mount_rotation"]) for r in meta["radars"]]
        self.p_mount = [np.array(r["mount_position"]) for r in meta["radars"]]
        # The take-off height: the estimate's origin is the take-off point (an RTK fix is absolute: its height
        # in the world is known on board too).
        self.home_z = pose.e.origin[2] if home_z is None else home_z
        self.map = (mapper_core.VoxelMap(VOXEL, self.home_z, MIN_FRAMES, MIN_BASELINE, NEAR_RANGE, GROUND_BAND)
                    if mapper_core else VoxelMap(ground_z=self.home_z))
        self.sigma_v = [getattr(c, "sigma_velocity", 0.0) or 0.0 for c in self.cfgs]
        self.sigma_angle = [np.radians(max(c.sigma_az, c.sigma_el)) for c in self.cfgs]
        self.frames = 0
        self.skipped = {"low": 0, "heading": 0}
        self.rejected = 0

    def process(self, stamp, radar, det, now):
        """One radar frame (n, 5 in its own frame) at scan time stamp, handled at time now. Returns the placed
        points (m, 5: x, y, z world, radial velocity, SNR) and the voxel keys to report."""
        p_base, r_base, v_base, w_base, sigma = self.pose.at(stamp, now)
        self.frames += 1

        if p_base[2] - self.home_z < MIN_ALT:
            self.skipped["low"] += 1
            return np.empty((0, 5)), []

        if sigma > MAX_HEADING_SIGMA:
            self.skipped["heading"] += 1
            return np.empty((0, 5)), []

        r_radar = r_base @ self.r_mount[radar]
        p_radar = p_base + r_base @ self.p_mount[radar]

        if mapper_core is not None:
            v_world = v_base + r_base @ np.cross(w_base, self.p_mount[radar])
            placed, rejected = mapper_core.place_frame(det, r_radar, p_radar, r_radar.T @ v_world,
                                                       self.sigma_v[radar], self.sigma_angle[radar], self.gate)
            self.rejected += rejected
            if not len(placed):
                return placed, []
            report = [tuple(k) for k in self.map.add(self.frames, placed[:, :3], p_radar)]
            return placed, report

        if len(det) and self.gate > 0:
            v_world = v_base + r_base @ np.cross(w_base, self.p_mount[radar])
            keep = self.gate_fn(det, r_radar.T @ v_world, self.cfgs[radar], self.gate)
            self.rejected += int((~keep).sum())
            det = det[keep]

        if not len(det):
            return np.empty((0, 5)), []

        world = det[:, :3] @ r_radar.T + p_radar
        report = self.map.add(self.frames, world, p_radar)
        return np.column_stack((world, det[:, 3:5])), report


def frames_of(rec):
    """(stamp, radar, detections) of a recording, in time order."""
    for k in np.argsort(rec["stamp"], kind="stable"):
        yield float(rec["stamp"][k]), int(rec["radar"][k]), \
            rec["points"][rec["first"][k]:rec["first"][k] + rec["count"][k]].astype(float)


def replay(tag, speed=0.0, on_frame=None, pose_delay=POSE_DELAY, radar_latency=RADAR_LATENCY, quiet=False):
    """Feed a recorded flight to an OnboardMapper; on_frame(now, stamp, points, report, mapper) after each
    frame. speed 1: at the radars' pace, 0: as fast as possible."""
    rec = load(tag + ".radar")
    world = rec["meta"]["world"]
    mapper = OnboardMapper(rec["meta"], CausalPose(sp.EstimatedPose(tag + ".ulg", world), pose_delay))
    cost = []
    start_wall, start_stamp = time.monotonic(), None

    for stamp, radar, det in frames_of(rec):
        now = stamp + radar_latency

        if speed > 0:
            start_stamp = stamp if start_stamp is None else start_stamp
            wait = (now - start_stamp) / speed - (time.monotonic() - start_wall)
            if wait > 0:
                time.sleep(wait)

        t0 = time.perf_counter()
        points, report = mapper.process(stamp, radar, det, now)
        cost.append(time.perf_counter() - t0)

        if on_frame is not None:
            on_frame(now, stamp, points, report, mapper)

    if not quiet:
        c = np.array(cost) * 1e3
        span = rec["stamp"][-1] - rec["stamp"][0]
        print(f"{mapper.frames} frames in {span:.0f} s of flight: {mapper.skipped['low']} below {MIN_ALT:g} m, "
              f"{mapper.skipped['heading']} while the heading sigma was above {MAX_HEADING_SIGMA:g} deg, "
              f"{mapper.rejected} detections failed the Doppler gate; processing {c.mean():.2f} ms a frame "
              f"(max {c.max():.1f}), {c.sum() / 1e3 / span * 100:.1f} % of real time")

    return mapper, rec


def offline_voxels(tag, voxel=VOXEL):
    """The voxels of the map made after the flight from the same data (survey_objects: ground and ghosts out)."""
    import contextlib
    import io

    import survey_objects as so

    rec = load(tag + ".radar")
    with contextlib.redirect_stdout(io.StringIO()):
        pts = sp.place(rec, "estimate", sp.EstimatedPose(tag + ".ulg", rec["meta"]["world"]))
    above, _, _ = so.remove_ground(pts, np.random.default_rng(so.SEED))
    kept, _ = so.remove_ghosts(above)
    return set(map(tuple, np.floor(kept[:, :3] / voxel).astype(np.int64)))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("tag", help="out/survey/<tag> (without extension)")
    p.add_argument("--speed", type=float, default=0.0, help="1: real time; 0: as fast as possible")
    p.add_argument("--pose-delay", type=float, default=POSE_DELAY)
    p.add_argument("--radar-latency", type=float, default=RADAR_LATENCY)
    args = p.parse_args()

    per_second = {}

    def count(now, stamp, points, report, mapper):
        s = int(now)
        per_second[s] = per_second.get(s, 0) + len(report)

    mapper, rec = replay(args.tag, args.speed, count, args.pose_delay, args.radar_latency)
    keys, cen, hits, ground = mapper.map.confirmed()
    reports = np.array(list(per_second.values()))
    print(f"map ({'C++' if mapper_core else 'Python'}): {len(mapper.map)} voxels touched, {len(keys)} confirmed ({int(ground.sum())} ground); "
          f"voxel reports per second: median {np.median(reports):.0f}, max {reports.max()}")

    truth = sp.load_truth(rec["meta"]["world"])
    obj = cen[~ground]
    d, labels, per = sp.score(obj, truth)
    s = sp.summary(d, labels)
    print(f"confirmed voxels above the ground: {len(obj)}, {100 * s['near']:.0f} % within {sp.NEAR:g} m of a true "
          f"surface (centroid), {100 * s['stray']:.1f} % further than {sp.STRAY:g} m")
    print("covered (true surface within 0.5 m of a confirmed voxel): " + ", ".join(
        f"{n} {100 * r['covered']:.0f} %" for n, r in per.items() if "covered" in r))

    offline = offline_voxels(args.tag)
    online = set(map(tuple, keys[~ground]))
    both = len(online & offline)
    print(f"against the map made after the flight: {both} voxels in both, {len(online - offline)} only on board, "
          f"{len(offline - online)} only after the flight (IoU {both / max(len(online | offline), 1):.2f})")

    sp.write_ply(args.tag + ".onboard.ply", np.column_stack((cen, np.zeros(len(cen)), hits, np.zeros(len(cen)))),
                 np.zeros(len(cen)))
    sp.plot(args.tag + ".onboard.png", obj, d, truth, f"{os.path.basename(args.tag)}: confirmed voxels on board")
    print(f"wrote {args.tag}.onboard.ply and .onboard.png")
    return 0


if __name__ == "__main__":
    sys.exit(main())
