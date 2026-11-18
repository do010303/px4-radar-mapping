#!/usr/bin/env python3
"""Radar map of a survey flight, after the flight (docs/survey_mapping_plan.md, phase 2).

Places every recorded detection (out/survey/<tag>.radar, scripts/radar_log.py) in the world with PX4's
estimated pose from the flight's log (<tag>.ulg), as on real hardware:

    p_world = R(t) (R_m p_radar + l) + t(t)

R(t), t(t): vehicle_attitude and vehicle_local_position interpolated to the scan time, the local frame
placed in the world by its origin (ref_lat, ref_lon, ref_alt) against the world's geographic origin, as an
RTK fix is absolute; R_m, l: the radar's mount (meta.yaml). Detections whose Doppler is not that of a
static point (radar_array.doppler_consistent, with the estimated velocity) are dropped, as are frames on
the ground and frames while EKF2 was unsure of the heading (MAX_HEADING_SIGMA: before the first horizontal
acceleration only the magnetometer gives it, and it was 23 deg off). The result is scored against worlds/survey_site.truth.yaml: how far each point is from the
nearest true surface, per object how many points and how much of it they cover.

    scripts/survey_process.py out/survey/rtk_2                  # estimated pose: the map, scored
    scripts/survey_process.py out/survey/rtk_2 --pose truth     # the simulation's pose (best case)
    scripts/survey_process.py out/survey/rtk_2 --clock-offset 0.05 --mount-error 1
                                                  # errors added: radar clock late by 50 ms, mounts off by 1 deg
    scripts/survey_process.py out/survey/rtk_2 --table          # the error budget: a table of variants
    scripts/survey_process.py out/survey/rtk_2 --find-clock     # the clock offset from the data alone

Writes <tag>.map.ply (x, y, z world ENU, radial velocity, SNR, radar, distance to the truth) and
<tag>.map.png (top and side view against the true objects).
"""

import argparse
import os
import re
import sys
import warnings

import numpy as np
import yaml

warnings.filterwarnings("ignore", message="A NumPy version")
from pyulog import ULog  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from radar_log import load  # noqa: E402

EARTH_RADIUS = 6371000.0            # m, as PX4's map projection
NED_TO_ENU = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])
MIN_ALT = 1.5                       # m: frames below this (take-off, landing) are left out
MAX_HEADING_SIGMA = 1.5             # deg: frames while EKF2 is less sure of the heading are left out. Until the
                                    # first horizontal acceleration the heading rests on the magnetometer alone:
                                    # 23 deg off in rtk_2 (EKF2 said 4-7 deg), 7 m at 20 m range
NEAR = 0.5                          # m: a point this close to a surface is on it; a surface sample this close
                                    # to a point is covered
STRAY = 2.0                         # m: a point further than this from every surface explains nothing


# --- pose ------------------------------------------------------------------------------------------------

def quat_to_matrix(q):
    """Rotation matrices (N, 3, 3) of unit quaternions (N, 4) w, x, y, z."""
    w, x, y, z = q.T
    return np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
                     np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
                     np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)], 1)


def interp_quat(t, tq, q):
    """Quaternions (w, x, y, z) at times t: normalised linear interpolation of the nearest two (250 Hz)."""
    q = q.copy()
    flip = np.r_[False, np.sum(q[1:] * q[:-1], axis=1) < 0]
    q[np.cumsum(flip) % 2 == 1] *= -1.0              # one hemisphere throughout
    i = np.clip(np.searchsorted(tq, t) - 1, 0, len(tq) - 2)
    a = np.clip((t - tq[i]) / np.maximum(tq[i + 1] - tq[i], 1e-9), 0.0, 1.0)[:, None]
    out = (1 - a) * q[i] + a * q[i + 1]
    return out / np.linalg.norm(out, axis=1, keepdims=True)


def world_origin(world):
    """Geographic origin (lat, lon deg, elevation m) of a world SDF."""
    text = open(os.path.join(ROOT, "worlds", world + ".sdf")).read()
    get = lambda tag: float(re.search(rf"<{tag}>\s*([-0-9.eE+]+)\s*</{tag}>", text).group(1))  # noqa: E731
    return get("latitude_deg"), get("longitude_deg"), get("elevation")


class EstimatedPose:
    """base_link's pose in the world (ENU, FLU) from a PX4 log, at any time (s, the log's clock)."""

    def __init__(self, ulog_path, world):
        log = ULog(ulog_path, ["vehicle_local_position", "vehicle_attitude", "vehicle_angular_velocity",
                               "estimator_states"])
        topic = lambda n: next(d.data for d in log.data_list if d.name == n and d.multi_id == 0)  # noqa: E731
        lp, att, gyro = topic("vehicle_local_position"), topic("vehicle_attitude"), topic("vehicle_angular_velocity")
        self.tp = lp["timestamp"] * 1e-6
        self.p_ned = np.column_stack((lp["x"], lp["y"], lp["z"]))
        self.v_ned = np.column_stack((lp["vx"], lp["vy"], lp["vz"]))
        self.tq = att["timestamp"] * 1e-6
        self.q = np.column_stack([att[f"q[{i}]"] for i in range(4)])
        self.tw = gyro["timestamp"] * 1e-6
        self.w_frd = np.column_stack([gyro[f"xyz[{i}]"] for i in range(3)])
        # EKF2's heading uncertainty: the variance of the third (vertical) orientation error state.
        st = topic("estimator_states")
        self.ts = st["timestamp"] * 1e-6
        self.heading_sigma = np.degrees(np.sqrt(np.maximum(st["covariances[2]"], 0.0)))
        # The local frame's origin in the world: an RTK position is absolute, so is the map.
        lat0, lon0, elev0 = world_origin(world)
        lat, lon, alt = lp["ref_lat"][-1], lp["ref_lon"][-1], lp["ref_alt"][-1]
        self.geo = (float(lat), float(lon), float(alt))     # EKF2's origin, geodetic (frames.LocalNed)
        self.origin = np.array([np.radians(lon - lon0) * np.cos(np.radians(lat0)) * EARTH_RADIUS,
                                np.radians(lat - lat0) * EARTH_RADIUS, alt - elev0])

    def at(self, t):
        """position (N, 3), rotation (N, 3, 3) base_link to world, velocity (N, 3) world, rate (N, 3) FLU."""
        p = np.column_stack([np.interp(t, self.tp, self.p_ned[:, k]) for k in range(3)]) @ NED_TO_ENU.T
        v = np.column_stack([np.interp(t, self.tp, self.v_ned[:, k]) for k in range(3)]) @ NED_TO_ENU.T
        r_frd_ned = quat_to_matrix(interp_quat(t, self.tq, self.q))
        rot = NED_TO_ENU @ r_frd_ned @ FLU_TO_FRD
        w = np.column_stack([np.interp(t, self.tw, self.w_frd[:, k]) for k in range(3)]) @ FLU_TO_FRD.T
        return p + self.origin, rot, v, w

    def heading_sigma_at(self, t):
        """EKF2's own standard deviation of the heading (deg) at times t."""
        return np.interp(t, self.ts, self.heading_sigma)


# --- placing the detections ------------------------------------------------------------------------------

def mount_errors(n, degrees, seed=7):
    """One fixed rotation of the given size per radar, about a random axis: a mount that is not as designed."""
    rng = np.random.default_rng(seed)
    out = []

    for _ in range(n):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        a = np.radians(degrees)
        k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        out.append(np.eye(3) + np.sin(a) * k + (1 - np.cos(a)) * k @ k)

    return out


def radar_cfgs(meta):
    """Each radar's model options (for the Doppler gate's noise figures), from its profile."""
    from radar_model import add_model_args

    cfgs = []

    for r in meta["radars"]:
        p = argparse.ArgumentParser()
        add_model_args(p, os.path.join(ROOT, "profiles", r["profile"]))
        cfgs.append(p.parse_args([]))

    return cfgs


def place(rec, pose="estimate", ulog=None, clock_offset=0.0, mount_error=0.0, doppler_gate=3.0, quiet=False,
          max_heading_sigma=MAX_HEADING_SIGMA):
    """All airborne detections in the world: (N, 10) x, y, z, radial velocity, SNR, radar, frame index, and the
    position of the radar that saw it (as placed: estimated or true)."""
    from radar_array import doppler_consistent

    meta = rec["meta"]
    n_radars = len(meta["radars"])
    r_mount = [np.array(r["mount_rotation"]) for r in meta["radars"]]
    p_mount = [np.array(r["mount_position"]) for r in meta["radars"]]
    errors = mount_errors(n_radars, mount_error) if mount_error else [np.eye(3)] * n_radars
    stamp = rec["stamp"]
    # Truth of base_link, for the altitude filter whatever pose places the points.
    rot_true = rec["rotation"].reshape(-1, 3, 3)
    base_z = np.array([rec["position"][k, 2] - (rot_true[k] @ r_mount[rec["radar"][k]].T @ p_mount[rec["radar"][k]])[2]
                       for k in range(len(stamp))])

    if pose == "estimate":
        # The radar's clock read clock_offset late: the pose is looked up that much later than the scan.
        p_base, r_base, v_base, w_base = ulog.at(stamp + clock_offset)
        r_radar = np.empty((len(stamp), 3, 3))
        p_radar = np.empty((len(stamp), 3))
        v_radar = np.empty((len(stamp), 3))

        for i in range(n_radars):
            sel = rec["radar"] == i
            used = errors[i] @ r_mount[i]
            r_radar[sel] = r_base[sel] @ used
            p_radar[sel] = p_base[sel] + r_base[sel] @ p_mount[i]
            # The radar's own velocity: the vehicle's plus its turn about base_link, in the radar frame.
            v_world = v_base[sel] + np.einsum("nij,nj->ni", r_base[sel], np.cross(w_base[sel], p_mount[i]))
            v_radar[sel] = np.einsum("nji,nj->ni", r_radar[sel], v_world)
    else:
        r_radar = rot_true.copy()
        p_radar = rec["position"].copy()
        v_radar = rec["velocity"].astype(float)

        for i in range(n_radars):
            sel = rec["radar"] == i
            r_radar[sel] = r_radar[sel] @ r_mount[i].T @ errors[i] @ r_mount[i]

    cfgs = radar_cfgs(meta)
    out, rejected = [], 0
    # Which frames: airborne, and (whatever pose places them, so that the variants compare the same frames)
    # while the estimate knew its heading.
    use = base_z > MIN_ALT
    unsure = use & (ulog.heading_sigma_at(stamp + clock_offset) > max_heading_sigma) if max_heading_sigma else np.zeros_like(use)
    use &= ~unsure

    for k in np.flatnonzero(use):
        det = rec["points"][rec["first"][k]:rec["first"][k] + rec["count"][k]].astype(float)

        if not len(det):
            continue

        if doppler_gate > 0:
            keep = doppler_consistent(det, v_radar[k], cfgs[rec["radar"][k]], doppler_gate)
            rejected += int((~keep).sum())
            det = det[keep]

        world = det[:, :3] @ r_radar[k].T + p_radar[k]
        out.append(np.column_stack((world, det[:, 3:5], np.full(len(det), rec["radar"][k]), np.full(len(det), k),
                                    np.tile(p_radar[k], (len(det), 1)))))

    if not quiet:
        print(f"placed {sum(len(o) for o in out)} detections of {int(use.sum())} airborne frames ({pose} pose); "
              f"{int(np.sum(unsure))} frames left out while EKF2's heading sigma was above {max_heading_sigma:g} deg; "
              f"{rejected} detections failed the Doppler gate")

    return np.vstack(out) if out else np.empty((0, 10))


# --- scoring against the truth ---------------------------------------------------------------------------

def load_truth(world):
    with open(os.path.join(ROOT, "worlds", world + ".truth.yaml")) as f:
        return yaml.safe_load(f)


def segment(t):
    if t["type"] == "pole":
        p0 = np.array(t["p"], float)
        return p0, p0 + [0.0, 0.0, t["h"]], t["r"]
    return np.array(t["p0"], float), np.array(t["p1"], float), t["r"]


def surface_distance(points, t):
    """Distance (m) of points (N, 3) to the surface of a truth object."""
    if t["type"] == "box":
        q = np.abs(points - np.array(t["c"], float)) - np.array(t["size"], float) / 2.0
        outside = np.linalg.norm(np.maximum(q, 0.0), axis=1)
        inside = np.minimum(q.max(axis=1), 0.0)
        return np.abs(outside + inside)

    p0, p1, r = segment(t)
    d = p1 - p0
    s = np.clip((points - p0) @ d / (d @ d), 0.0, 1.0)
    return np.abs(np.linalg.norm(points - (p0 + s[:, None] * d), axis=1) - r)


def surface_samples(t, step=0.25):
    """Points on a truth object's surface (poles and wires: on the axis; their radius is below the error)."""
    if t["type"] == "box":
        c, h = np.array(t["c"], float), np.array(t["size"], float) / 2.0
        out = []

        for axis in range(3):
            others = [a for a in range(3) if a != axis]
            u = np.arange(-h[others[0]], h[others[0]] + 1e-9, step)
            v = np.arange(-h[others[1]], h[others[1]] + 1e-9, step)
            uu, vv = np.meshgrid(u, v)

            for sign in (-1, 1):
                if axis == 2 and sign == -1:
                    continue        # the underside rests on the ground
                p = np.zeros((uu.size, 3))
                p[:, others[0]], p[:, others[1]], p[:, axis] = uu.ravel(), vv.ravel(), sign * h[axis]
                out.append(c + p)

        return np.vstack(out)

    p0, p1, _ = segment(t)
    n = max(int(np.linalg.norm(p1 - p0) / step), 2)
    return p0 + np.linspace(0, 1, n)[:, None] * (p1 - p0)


def score(points, truth, near=NEAR):
    """Per-point distance to the nearest surface and its label, and a summary per object."""
    from scipy.spatial import cKDTree

    names = list(truth) + ["ground"]
    dist = np.column_stack([surface_distance(points[:, :3], truth[n]) for n in truth] + [np.abs(points[:, 2])])
    nearest = dist.argmin(axis=1)
    d = dist[np.arange(len(points)), nearest]
    tree = cKDTree(points[:, :3]) if len(points) else None
    per = {}

    for j, name in enumerate(names):
        mine = (nearest == j) & (d <= STRAY)
        row = {"points": int(mine.sum()),
               "median": float(np.median(d[mine])) if mine.any() else float("nan"),
               "p95": float(np.percentile(d[mine], 95)) if mine.any() else float("nan")}

        if name != "ground" and tree is not None:
            samples = surface_samples(truth[name])
            reach = near + (truth[name]["r"] if truth[name]["type"] != "box" else 0.0)
            row["covered"] = float(np.mean([len(x) > 0 for x in tree.query_ball_point(samples, reach)]))

        per[name] = row

    return d, np.array(names)[nearest], per


def summary(d, labels):
    obj = labels != "ground"
    return {"points": len(d), "on_objects": int((obj & (d <= STRAY)).sum()),
            "median": float(np.median(d[obj & (d <= STRAY)])) if (obj & (d <= STRAY)).any() else float("nan"),
            "p95": float(np.percentile(d[obj & (d <= STRAY)], 95)) if (obj & (d <= STRAY)).any() else float("nan"),
            "near": float(np.mean(d <= NEAR)), "stray": float(np.mean(d > STRAY))}


# --- output ----------------------------------------------------------------------------------------------

def write_ply(path, pts, d, ned=None):
    """pts as place gives them (world frame); with ned (frames.LocalNed) written in PX4's local NED frame."""
    if ned is not None:
        pts = np.column_stack((ned.points(pts[:, :3]), pts[:, 3:]))
    comment = "world_enu (x east, y north, z up, m)" if ned is None else \
        f"px4_local_ned (x north, y east, z down, m) from EKF2's origin at world {np.round(ned.origin, 3).tolist()}" + \
        (f", lat {ned.geo[0]:.8f} lon {ned.geo[1]:.8f} alt {ned.geo[2]:.3f}" if ned.geo else "")
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n"
                f"comment frame {comment}\n"
                f"element vertex {len(pts)}\n"
                "property float x\nproperty float y\nproperty float z\nproperty float radial_velocity\n"
                "property float snr\nproperty uchar radar\nproperty float truth_distance\nend_header\n")
        for p, dd in zip(pts, d):
            f.write(f"{p[0]:.3f} {p[1]:.3f} {p[2]:.3f} {p[3]:.3f} {p[4]:.1f} {int(p[5])} {dd:.3f}\n")


def plot(path, pts, d, truth, title):
    from plotstyle import FAINT, INK, INK2, figure, style_axes

    plt, fig, axes = figure(2, [1.0, 0.55], (11, 9))
    on = d <= NEAR
    mid = (d > NEAR) & (d <= STRAY)
    far = d > STRAY
    show = pts[:, 2] > 0.3          # the ground's points would cover everything in the top view

    for ax, (i, j), (xl, yl) in zip(axes, ((0, 1), (0, 2)), (("x (m)", "y (m)"), ("x (m)", "z (m)"))):
        for t in truth.values():
            if t["type"] == "box":
                c, s = np.array(t["c"]), np.array(t["size"])
                ax.add_patch(plt.Rectangle((c[i] - s[i] / 2, c[j] - s[j] / 2), s[i], s[j], fill=False, ec=INK, lw=1.0))
            else:
                p0, p1, _ = segment(t)
                ax.plot([p0[i], p1[i]], [p0[j], p1[j]], color=INK, lw=1.0)

        for sel, colour, label in ((far & show, "#d64545", f"> {STRAY:g} m from any object"),
                                   (mid & show, "#e8a33a", f"{NEAR:g}-{STRAY:g} m"),
                                   (on & show, "#2a78d6", f"within {NEAR:g} m")):
            ax.scatter(pts[sel, i], pts[sel, j], s=1.5, color=colour, alpha=0.5, lw=0, label=label)

        style_axes(ax, xl, yl)
        ax.set_aspect("equal")

    axes[0].set_xlim(-5, 65), axes[0].set_ylim(-25, 25)
    axes[1].set_xlim(-5, 65), axes[1].set_ylim(0, 20)
    axes[0].legend(loc="upper right", fontsize=8, markerscale=6, frameon=False, labelcolor=INK2)
    axes[0].set_title(title, color=INK, fontsize=11, loc="left")
    fig.text(0.01, 0.01, "true objects in black; ground points (z < 0.3 m) not drawn", color=FAINT, fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


# --- variants --------------------------------------------------------------------------------------------

def sharpness(pts, voxel=0.3):
    """Occupied voxels per point above the ground: fewer for the same points is a sharper map."""
    above = pts[pts[:, 2] > 0.5, :3]
    return len(np.unique(np.floor(above / voxel).astype(np.int64), axis=0)) / max(len(above), 1)


def table(rec, ulog, truth, gate):
    variants = [("truth pose", dict(pose="truth")),
                ("estimate (RTK)", dict(pose="estimate")),
                ("+ clock 20 ms", dict(pose="estimate", clock_offset=0.02)),
                ("+ clock 50 ms", dict(pose="estimate", clock_offset=0.05)),
                ("+ clock 100 ms", dict(pose="estimate", clock_offset=0.10)),
                ("+ mount 1 deg", dict(pose="estimate", mount_error=1.0)),
                ("+ mount 2 deg", dict(pose="estimate", mount_error=2.0)),
                ("+ mount 5 deg", dict(pose="estimate", mount_error=5.0))]
    print("| variant | points on objects | median (m) | 95 % (m) | within 0.5 m | > 2 m | covered: wires, poles, house, wall |")
    print("|---|---|---|---|---|---|---|")

    for label, kw in variants:
        pts = place(rec, ulog=ulog, doppler_gate=gate, quiet=True, **kw)
        d, labels, per = score(pts, truth)
        s = summary(d, labels)
        cov = lambda names: np.mean([per[n]["covered"] for n in names])  # noqa: E731
        print(f"| {label} | {s['on_objects']} | {s['median']:.2f} | {s['p95']:.2f} | {100 * s['near']:.0f} % | "
              f"{100 * s['stray']:.1f} % | {100 * cov([n for n in truth if truth[n]['type'] == 'wire']):.0f}, "
              f"{100 * cov([n for n in truth if truth[n]['type'] == 'pole']):.0f}, {100 * per['house']['covered']:.0f}, "
              f"{100 * per['wall']['covered']:.0f} % |", flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("tag", help="out/survey/<tag> (without extension)")
    p.add_argument("--pose", choices=("estimate", "truth"), default="estimate")
    p.add_argument("--clock-offset", type=float, default=0.0, help="error added: radar clock late by this (s)")
    p.add_argument("--mount-error", type=float, default=0.0, help="error added: each mount off by this (deg)")
    p.add_argument("--doppler-gate", type=float, default=3.0, help="0: keep moving-looking detections too")
    p.add_argument("--table", action="store_true", help="the error budget: truth, estimate and added errors")
    p.add_argument("--find-clock", action="store_true", help="the clock offset that gives the sharpest map")
    p.add_argument("--frame", choices=("ned", "world"), default="ned",
                   help="coordinates of <tag>.map.ply: PX4's local NED frame from EKF2's origin (default), or the world's")
    args = p.parse_args()

    rec = load(args.tag + ".radar")
    world = rec["meta"]["world"]
    truth = load_truth(world)
    ulog = EstimatedPose(args.tag + ".ulg", world)

    if args.table:
        table(rec, ulog, truth, args.doppler_gate)
        return 0

    if args.find_clock:
        base = args.clock_offset
        rows = []
        for off in np.arange(-0.15, 0.1501, 0.01):
            pts = place(rec, ulog=ulog, clock_offset=base + off, doppler_gate=args.doppler_gate, quiet=True)
            rows.append((off, sharpness(pts)))
            print(f"  shift {off:+.2f} s: {rows[-1][1]:.4f} voxels/point", flush=True)
        best = min(rows, key=lambda r: r[1])
        # In the simulation the radar stamps and the PX4 log share one clock (scripts/survey_check.py: a few ms
        # apart), so the right answer here is about 0, whatever --clock-offset was assumed.
        print(f"sharpest at {best[0]:+.2f} s from the {base:+.2f} s assumed: the radar clock is about "
              f"{base + best[0]:+.2f} s off the PX4 log's")
        return 0

    pts = place(rec, args.pose, ulog, args.clock_offset, args.mount_error, args.doppler_gate)
    d, labels, per = score(pts, truth)
    s = summary(d, labels)
    print(f"{s['points']} points: {100 * s['near']:.0f} % within {NEAR:g} m of a true surface, "
          f"{100 * s['stray']:.1f} % more than {STRAY:g} m from all; on objects: median {s['median']:.2f} m, "
          f"95 % {s['p95']:.2f} m")
    print(f"{'object':12s} {'points':>7s} {'median':>7s} {'95 %':>6s} {'covered':>8s}")

    for name, row in per.items():
        cov = f"{100 * row['covered']:.0f} %" if "covered" in row else ""
        print(f"{name:12s} {row['points']:7d} {row['median']:7.2f} {row['p95']:6.2f} {cov:>8s}")

    suffix = "" if args.pose == "estimate" and not args.clock_offset and not args.mount_error else \
        f".{args.pose}" + (f".clock{args.clock_offset:g}" if args.clock_offset else "") + \
        (f".mount{args.mount_error:g}" if args.mount_error else "")
    import frames
    ned = frames.LocalNed(ulog.origin, ulog.geo) if args.frame == "ned" else None
    write_ply(args.tag + suffix + ".map.ply", pts, d, ned)
    title = f"{os.path.basename(args.tag)}: {args.pose} pose" + (f", clock +{args.clock_offset:g} s" if args.clock_offset else "") + \
        (f", mounts off {args.mount_error:g} deg" if args.mount_error else "")
    plot(args.tag + suffix + ".map.png", pts, d, truth, title)
    print(f"wrote {args.tag + suffix}.map.ply ({frames.NAME if ned is not None else 'world frame'}) and .map.png")
    return 0


if __name__ == "__main__":
    sys.exit(main())
