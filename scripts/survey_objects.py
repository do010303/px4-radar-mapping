#!/usr/bin/env python3
"""Objects from the radar map of a survey flight (docs/survey_mapping_plan.md, phase 3).

From the detections placed by scripts/survey_process.py (estimated pose, Doppler gate, heading filter):

  1. ground   the dominant level plane (RANSAC), and everything within GROUND_BAND of it, removed
  2. ghosts   a detection stays only if its VOXEL holds detections from at least MIN_FRAMES radar frames
              taken from places at least MIN_BASELINE apart: a real object is seen again from elsewhere,
              a sidelobe, multipath or noise detection lands somewhere new each time
  3. poles    vertical structures: horizontal grid cells whose points span at least POLE_HEIGHT in height,
              clustered; the axis is the densest spot of a cluster, the top its 95th height percentile
  4. wires    in what the poles leave (POLE_CLEAR around each axis), near-horizontal lines by RANSAC, each
              at least WIRE_LENGTH long; straight lines (the simulated wires do not sag)
  4. blocks   before the wires, clusters with a body (find_blocks: a roof, a wall), as boxes from the 5th to
              95th percentiles; else RANSAC finds the edges of a roof as wires
  5. other    what is left, clustered: boxes too

Writes <tag>.objects.yaml (type, geometry, number of points, spread; for poles and wires also points_sharp and
spread_sharp), <tag>.objects/<name>.sharp.ply (its sharp point cloud: sharpen), <tag>.objects/<name>.ply (the points of
each object) and <tag>.objects.png; with the world's truth file (worlds/<world>.truth.yaml), scores them:
for each true object, the object found for it and how far off it is; found objects that match nothing.

    scripts/survey_objects.py out/survey/rtk_2
    scripts/survey_objects.py out/survey/rtk_2 --pose truth        # with the simulation's pose
"""

import argparse
import os
import sys
import warnings

import numpy as np
import yaml

warnings.filterwarnings("ignore", message="A NumPy version")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import frames  # noqa: E402
import survey_process as sp  # noqa: E402
from radar_log import load  # noqa: E402

GROUND_BAND = 0.6           # m above the ground plane that counts as ground
VOXEL = 0.5                 # m, ghost filter
MIN_FRAMES = 3
MIN_BASELINE = 2.0          # m between the radar positions that saw a voxel
CLUSTER_EPS = 0.8           # m, neighbourhood of the clustering
POLE_GRID = 0.5             # m, horizontal grid in which vertical structures are looked for
POLE_CELL_POINTS = 10       # points a grid cell needs to count
POLE_HEIGHT = 3.0           # m of height a pole's points must span
POLE_MIN_POINTS = 300         # a pole flown past has thousands (the lamp post 3700); a stack of 2-4 cells on a
                              # wall or roof edge 70-90 (live_6), and as a pole it swallowed the wall around it
POLE_MAX_ELONGATION = 2.5   # ratio of the plan spreads above which a tall cluster is a wall, not a pole, ...
POLE_MAX_SPREAD = 1.2       # m, ... if its long spread (sd) is also above this: a pole passed on one side only is
                            # stretched along the pass by the radar's angle error (sd 0.9 m, ratio 3), a 5 m
                            # wall is 1.4 m
POLE_CLEAR = 3.0            # m around a pole axis: its points (the spread of the radar angles reaches 2 m),
                            # left out of the wire search
WIRE_TOL = 0.5              # m, RANSAC inlier distance
WIRE_LENGTH = 5.0           # m
WIRE_MIN_POINTS = 25
WIRE_MAX_SLOPE = 0.15       # rise over run
WIRE_SHELL = 1.2            # m: the band beside a wire candidate, beyond WIRE_TOL, that must be nearly empty
WIRE_SAME = 0.8             # m from the line of a wire already found: a piece of that wire
WIRE_ABSORB = 1.0           # m from a found wire (to its ends): left-over points there are more of it (the spread
                            # of its detections), not objects of their own
WIRE_MAX_SHELL = 0.8        # points in that band per point on the line above which it is part of a surface
OTHER_MIN_POINTS = 15
SHARP_VOXEL = 0.5           # m: an object's point cloud, sharpened (sharpen): in voxels of this, ...
SHARP_NEAR = 5.0            # ... each detection weighing (SHARP_NEAR / max(range, SHARP_NEAR))^2, ...
SHARP_RADIUS = 1.5          # ... kept where its voxel weighs at least SHARP_SHARE of the most within SHARP_RADIUS
SHARP_SHARE = 0.35          # and at least SHARP_MIN (as the ground map's sharp layer). For poles and wires: a
SHARP_MIN = 0.5             # block's height is in its few far points (live_6's house: 2.3 m tall -> 1.3)
SHARP_TYPES = ("pole", "wire")

BLOCK_MIN_WIDTH = 1.5       # m, both ways in plan, ...
BLOCK_MIN_HEIGHT = 0.8      # m, ... and this tall: a body (a house)
BLOCK_WALL_HEIGHT = 2.0     # m tall and as long: a wall
BLOCK_GROUND_REACH = 3.0    # m: a wall reaching this close to the ground is a wall at any aspect (5 m x 0.4 m:
                            # aspect 8-10); two wires one above the other (10 and 12 m up) never come this low
BLOCK_MAX_ASPECT = 6.0      # longer than this many times its width in plan: a line, not a body
SEED = 3


# --- steps -----------------------------------------------------------------------------------------------

def remove_ground(pts, rng):
    """(above, plane (n, d) with n . p + d = 0, n up)."""
    xyz = pts[:, :3]
    best, best_count = None, -1
    # Planes through three of the lowest points: drawn from all, three ground points together were rare
    # enough (a sixth of the points) that a level plane through the wires won.
    low = xyz[xyz[:, 2] <= np.percentile(xyz[:, 2], 30)]

    for _ in range(300):
        a, b, c = low[rng.choice(len(low), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-6:
            continue
        n /= np.linalg.norm(n)
        n = n if n[2] > 0 else -n
        if n[2] < 0.97:             # level ground only
            continue
        d = -n @ a
        count = int(np.sum(np.abs(xyz @ n + d) < 0.3))
        if count > best_count:
            best, best_count = (n, d), count

    n, d = best
    height = xyz @ n + d
    return pts[height > GROUND_BAND], (n, d), int(np.sum(height <= GROUND_BAND))


def remove_ghosts(pts):
    """Detections in voxels seen in enough frames from far enough apart."""
    key = np.floor(pts[:, :3] / VOXEL).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.ravel()
    keep = np.zeros(len(pts), bool)

    for v in np.unique(inv):
        idx = np.flatnonzero(inv == v)
        if len(np.unique(pts[idx, 6])) < MIN_FRAMES:
            continue
        sensors = pts[idx, 7:10]
        # Spread of the viewpoints: the two furthest apart (along the main direction is enough).
        if np.ptp(sensors, axis=0).max() >= MIN_BASELINE:
            keep[idx] = True

    return pts[keep], int((~keep).sum())


def clusters(xyz, eps, min_points):
    """Connected components of the eps-neighbourhood graph with at least min_points points: list of index arrays."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    from scipy.spatial import cKDTree

    if len(xyz) == 0:
        return []

    pairs = cKDTree(xyz).query_pairs(eps, output_type="ndarray")
    graph = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(len(xyz), len(xyz)))
    _, label = connected_components(graph, directed=False)
    out = [np.flatnonzero(label == lab) for lab in np.unique(label)]
    return sorted([c for c in out if len(c) >= min_points], key=len, reverse=True)


def densest(xy, radius=0.5, cell=0.25):
    """The densest spot of xy (n, 2): a pole's axis, robust to the spread. The peak of a smoothed 2D
    histogram, refined by the median of the points within radius of it (a neighbour count per point took
    seconds for the 10 000 points of a pylon)."""
    lo = xy.min(axis=0)
    ij = np.floor((xy - lo) / cell).astype(int)
    grid = np.zeros(ij.max(axis=0) + 3)
    np.add.at(grid, (ij[:, 0] + 1, ij[:, 1] + 1), 1.0)
    smooth = sum(np.roll(np.roll(grid, dx, 0), dy, 1) for dx in (-1, 0, 1) for dy in (-1, 0, 1))
    peak = np.array(np.unravel_index(np.argmax(smooth), smooth.shape)) - 1
    centre = lo + (peak + 0.5) * cell
    for r in (radius, 0.7 * radius, 0.5 * radius):     # closing in on the peak
        near = np.linalg.norm(xy - centre, axis=1) <= r
        if near.sum() < 5:
            break
        centre = np.median(xy[near], axis=0)
    return centre


def find_poles(pts, ground_z):
    """Vertical structures: grid cells whose points stack up at least POLE_HEIGHT, clustered; per cluster the
    densest spot is the axis. (Clustering all points instead joins the two pylons through their wires.)"""
    cell = np.floor(pts[:, :2] / POLE_GRID).astype(np.int64)
    _, inv, count = np.unique(cell, axis=0, return_inverse=True, return_counts=True)
    inv = inv.ravel()
    order = np.argsort(inv, kind="stable")
    starts = np.r_[0, np.cumsum(count)[:-1]]
    tall = np.zeros(len(count), bool)

    for c in np.flatnonzero(count >= POLE_CELL_POINTS):
        z = pts[order[starts[c]:starts[c] + count[c]], 2]
        tall[c] = np.percentile(z, 95) - np.percentile(z, 5) >= POLE_HEIGHT

    poles, used = [], np.zeros(len(pts), bool)
    # The tall cells, clustered as cells (neighbours within 1.5 cells): clustering their points instead
    # paired the thousands of points stacked over one spot with each other.
    cells_all = np.unique(cell, axis=0)
    tall_cells = cells_all[tall]
    groups = clusters((tall_cells + 0.5) * POLE_GRID, POLE_GRID * 1.5, 1)

    for g in groups:
        members = np.zeros(len(count), bool)
        members[np.flatnonzero(tall)[g]] = True
        idx = np.flatnonzero(members[inv])
        if len(idx) < POLE_MIN_POINTS:
            continue
        # A pole is round in plan; a wall standing up is long: its spread along one way is many times the other.
        ev = np.sort(np.linalg.eigvalsh(np.cov(pts[idx, :2].T)))
        if np.sqrt(ev[1] / max(ev[0], 1e-6)) > POLE_MAX_ELONGATION and np.sqrt(ev[1]) > POLE_MAX_SPREAD:
            continue
        axis = densest(pts[idx, :2])
        # Its points: all within POLE_CLEAR of the axis, as far up as the pole goes.
        top = np.percentile(pts[idx, 2], 95)
        near = np.flatnonzero((np.linalg.norm(pts[:, :2] - axis, axis=1) <= POLE_CLEAR) & ~used)
        poles.append({"type": "pole", "p": [float(axis[0]), float(axis[1]), float(ground_z)],
                      "h": float(top - ground_z), "points": int(len(near)),
                      "spread": float(np.median(np.linalg.norm(pts[near, :2] - axis, axis=1))), "_idx": near})
        used[near] = True

    return poles, used


def find_wires(pts, rng, iterations=400):
    wires = []
    free = np.ones(len(pts), bool)

    while free.sum() >= WIRE_MIN_POINTS:
        idx = np.flatnonzero(free)
        xyz = pts[idx, :3]
        best = None

        for _ in range(iterations):
            a, b = xyz[rng.choice(len(xyz), 2, replace=False)]
            d = b - a
            run = np.linalg.norm(d[:2])
            if run < 2.0 or abs(d[2]) / run > WIRE_MAX_SLOPE:
                continue
            d /= np.linalg.norm(d)
            rel = xyz - a
            dist = np.linalg.norm(rel - np.outer(rel @ d, d), axis=1)
            inl = np.flatnonzero(dist < WIRE_TOL)
            if best is None or len(inl) > len(best):
                best = inl

        if best is None or len(best) < WIRE_MIN_POINTS:
            break

        # A wire stands alone: a line through a surface (a roof's edge, a wall's top) has as many points
        # again just beside it.
        a0 = xyz[best].mean(axis=0)
        d0 = np.linalg.svd(xyz[best] - a0)[2][0]
        rel = xyz - a0
        dist = np.linalg.norm(rel - np.outer(rel @ d0, d0), axis=1)
        along = rel @ d0
        span = (along >= along[best].min()) & (along <= along[best].max())
        shell = int(np.sum(span & (dist > WIRE_TOL + 0.1) & (dist <= WIRE_SHELL)))
        if shell > WIRE_MAX_SHELL * len(best):
            free[idx[best]] = False
            continue

        # Refit on the inliers (principal axis), keep the longest gap-free run along it.
        p = xyz[best]
        c = p.mean(axis=0)
        d = np.linalg.svd(p - c)[2][0]
        s = (p - c) @ d
        order = np.argsort(s)
        gaps = np.flatnonzero(np.diff(s[order]) > 3.0)
        runs = np.split(order, gaps + 1)
        run = max(runs, key=len)
        s_run = s[run]
        length = np.percentile(s_run, 98) - np.percentile(s_run, 2)
        free[idx[best]] = False             # tried: not again, whether accepted or not

        if length < WIRE_LENGTH or len(run) < WIRE_MIN_POINTS:
            continue

        sel = idx[best[run]]
        c = pts[sel, :3].mean(axis=0)
        p0, p1 = c + np.percentile(s_run, 2) * d, c + np.percentile(s_run, 98) * d

        # A piece of a wire already found (along it, within reach of its line): the same wire.
        if any(np.linalg.norm((c - np.array(w["p0"])) - ((c - np.array(w["p0"])) @ wd) * wd) < WIRE_SAME
               and abs(d @ wd) > np.cos(np.radians(10.0))
               for w in wires for wd in [(np.array(w["p1"]) - np.array(w["p0"])) / max(np.linalg.norm(
                   np.array(w["p1"]) - np.array(w["p0"])), 1e-6)]):
            continue
        rel = pts[sel, :3] - c
        spread = np.median(np.linalg.norm(rel - np.outer(rel @ d, d), axis=1))
        wires.append({"type": "wire", "p0": p0.round(3).tolist(), "p1": p1.round(3).tolist(),
                      "points": int(len(sel)), "spread": float(spread), "_idx": sel})

    return wires


def sharpen(pts):
    """Which of an object's points (as place gives them: sensor position in columns 7-9) to keep in its sharp
    point cloud: those near enough the radar, often enough. The radar's angle error spreads an object's
    detections by its range times the error; the near ones lie close to the surface, the far ones are the
    halo (live_6's pylons: median spread 0.65 -> 0.33 m, lamp post 0.50 -> 0.19 m; radii 0.2 and 0.1 m)."""
    from scipy.spatial import cKDTree

    if len(pts) == 0:
        return np.zeros(0, bool)
    w = (SHARP_NEAR / np.maximum(np.linalg.norm(pts[:, :3] - pts[:, 7:10], axis=1), SHARP_NEAR)) ** 2
    keys, inv = np.unique(np.floor(pts[:, :3] / SHARP_VOXEL).astype(np.int64), axis=0, return_inverse=True)
    inv = inv.ravel()
    weight = np.bincount(inv, weights=w)
    centres = (keys + 0.5) * SHARP_VOXEL
    local = np.array([weight[n].max() for n in cKDTree(centres).query_ball_point(centres, SHARP_RADIUS)])
    return ((weight >= SHARP_SHARE * local) & (weight >= SHARP_MIN))[inv]


def object_spread(o, pts):
    """Median distance of the points from a pole's axis or a wire's line (m)."""
    if o["type"] == "pole":
        return float(np.median(np.linalg.norm(pts[:, :2] - np.array(o["p"][:2]), axis=1)))
    a, b = np.array(o["p0"]), np.array(o["p1"])
    d = (b - a) / max(np.linalg.norm(b - a), 1e-9)
    r = pts[:, :3] - a
    return float(np.median(np.linalg.norm(r - np.outer(r @ d, d), axis=1)))


def segment_distance(xyz, p0, p1):
    a, b = np.asarray(p0, float), np.asarray(p1, float)
    d = b - a
    s = np.clip((xyz - a) @ d / max(d @ d, 1e-9), 0.0, 1.0)
    return np.linalg.norm(xyz - (a + np.outer(s, d)), axis=1)


def find_other(pts):
    out = []

    for idx in clusters(pts[:, :3], CLUSTER_EPS, OTHER_MIN_POINTS):
        lo, hi = np.percentile(pts[idx, :3], 5, axis=0), np.percentile(pts[idx, :3], 95, axis=0)
        out.append({"type": "box", "c": ((lo + hi) / 2).round(3).tolist(), "size": (hi - lo).round(3).tolist(),
                    "points": int(len(idx)), "_idx": idx})

    return out


def find_blocks(pts, ground_z=0.0):
    """Clusters with a body: wide both ways in plan (a roof), or tall and long (a wall), and not a long thin
    line (two wires one above the other, joined by their spread). Taken out before the wire search: through
    them RANSAC finds lines (a roof's edges) that are no wires (sens_surface+10: 12 false objects)."""
    out, taken = [], np.zeros(len(pts), bool)

    for idx in clusters(pts[:, :3], CLUSTER_EPS, OTHER_MIN_POINTS):
        lo, hi = np.percentile(pts[idx, :3], 5, axis=0), np.percentile(pts[idx, :3], 95, axis=0)
        ext = hi - lo
        plan = np.sort(ext[:2])
        wide = plan[0] >= BLOCK_MIN_WIDTH and ext[2] >= BLOCK_MIN_HEIGHT
        wall = ext[2] >= BLOCK_WALL_HEIGHT and plan[1] >= BLOCK_WALL_HEIGHT
        standing = lo[2] <= ground_z + BLOCK_GROUND_REACH
        if (wide or wall) and (plan[1] <= BLOCK_MAX_ASPECT * max(plan[0], 0.1) or (wall and standing)):
            out.append({"type": "box", "c": ((lo + hi) / 2).round(3).tolist(), "size": ext.round(3).tolist(),
                        "points": int(len(idx)), "_idx": idx})
            taken[idx] = True

    return out, taken


def find_objects(kept, ground_z, rng):
    """Poles, then bodies (blocks), then wires, then whatever is left, from points above the ground."""
    poles, used = find_poles(kept, ground_z)
    rest = kept[~used]
    blocks, in_block = find_blocks(rest, ground_z)
    loose = rest[~in_block]
    wires = find_wires(loose, rng)
    taken = np.zeros(len(loose), bool)
    for w in wires:
        taken[w["_idx"]] = True
        taken |= segment_distance(loose[:, :3], w["p0"], w["p1"]) <= WIRE_ABSORB
    others = find_other(loose[~taken])

    # Each object's points, for the per-object files.
    for p in poles:
        p["_pts"] = kept[p["_idx"]]
    for b in blocks:
        b["_pts"] = rest[b["_idx"]]
    for w in wires:
        w["_pts"] = loose[w["_idx"]]
    for o in others:
        o["_pts"] = loose[~taken][o["_idx"]]

    return poles + blocks + wires + others


def extract(pts, rng, quiet=False):
    above, (n, d), n_ground = remove_ground(pts, rng)
    ground_z = float(-d / n[2])
    kept, n_ghost = remove_ghosts(above)
    objects = find_objects(kept, ground_z, rng)

    if not quiet:
        count = lambda kind: sum(o["type"] == kind for o in objects)  # noqa: E731
        print(f"{len(pts)} detections: {n_ground} ground (plane at z {ground_z:+.2f} m), {n_ghost} ghosts "
              f"(voxels seen in < {MIN_FRAMES} frames or from < {MIN_BASELINE:g} m apart), {len(kept)} kept; "
              f"found {count('pole')} poles, {count('wire')} wires, {count('box')} blocks and other")

    return objects, ground_z


# --- scoring ---------------------------------------------------------------------------------------------

def line_error(found, true):
    """Mean distance (m) of the true wire to the found line, over the part found; share of the true length found."""
    q0, q1 = np.array(found["p0"]), np.array(found["p1"])
    p0, p1, _ = sp.segment(true)
    d = (p1 - p0) / np.linalg.norm(p1 - p0)
    # Samples along the found segment, against the true line.
    s = q0 + np.linspace(0, 1, 50)[:, None] * (q1 - q0)
    rel = s - p0
    off = np.linalg.norm(rel - np.outer(rel @ d, d), axis=1)
    t = np.clip(np.sort([(q0 - p0) @ d, (q1 - p0) @ d]), 0.0, np.linalg.norm(p1 - p0))
    return float(off.mean()), float((t[1] - t[0]) / np.linalg.norm(p1 - p0))


def match(objects, truth):
    """For each true object, the best found object of its kind and its error; found objects left over."""
    rows, used = {}, set()

    for name, t in truth.items():
        best = None

        for k, o in enumerate(objects):
            if k in used:
                continue
            if t["type"] == "pole" and o["type"] == "pole":
                err = float(np.linalg.norm(np.array(o["p"][:2]) - np.array(t["p"][:2])))
                if err < 3.0 and (best is None or err < best[1]):
                    best = (k, err, {"height": o["h"] - t["h"]})
            elif t["type"] == "wire" and o["type"] == "wire":
                err, share = line_error(o, t)
                if err < 1.5 and (best is None or err < best[1]):
                    best = (k, err, {"length_found": share})
            elif t["type"] == "box" and o["type"] == "box":
                c = np.array(t["c"]); h = np.array(t["size"]) / 2
                inside = np.all(np.abs(np.array(o["c"]) - c) <= h + 1.5)
                err = float(np.linalg.norm(np.array(o["c"]) - c))
                if inside and (best is None or err < best[1]):
                    best = (k, err, {"size": o["size"]})

        if best is not None:
            used.add(best[0])
        rows[name] = best

    return rows, [k for k in range(len(objects)) if k not in used]


def _shifted(o, d):
    """An object (found or true) moved by -d."""
    o = dict(o)
    for k in ("p", "p0", "p1", "c"):
        if k in o:
            o[k] = (np.asarray(o[k], float) - d).tolist()
    return o


def plot(path, objects, pts, truth, title, ned=None):
    """Plan and side view. With ned (frames.LocalNed) the axes are PX4's local frame from its origin: east
    across, north (plan) or up, -z (side), as one looks at a map."""
    from plotstyle import FAINT, INK, INK2, figure, style_axes

    d = ned.origin if ned is not None else np.zeros(3)
    if ned is not None:
        pts = np.column_stack((pts[:, :3] - d, pts[:, 3:]))
        objects = [_shifted(o, d) for o in objects]
        truth = {n: _shifted(t, d) for n, t in truth.items()}
        names = (("y: east (m)", "x: north (m)"), ("y: east (m)", "-z: up (m)"))
    else:
        names = (("x (m)", "y (m)"), ("x (m)", "z (m)"))

    plt, fig, axes = figure(2, [1.0, 0.55], (11, 9))
    colours = {"pole": "#2a78d6", "wire": "#e0632a", "box": "#3a9a5b"}

    for ax, (i, j), (xl, yl) in zip(axes, ((0, 1), (0, 2)), names):
        ax.scatter(pts[:, i], pts[:, j], s=1, color=FAINT, lw=0)

        for t in truth.values():
            if t["type"] == "box":
                c, s = np.array(t["c"]), np.array(t["size"])
                ax.add_patch(plt.Rectangle((c[i] - s[i] / 2, c[j] - s[j] / 2), s[i], s[j], fill=False, ec=INK,
                                           lw=0.8, ls="--"))
            else:
                p0, p1, _ = sp.segment(t)
                ax.plot([p0[i], p1[i]], [p0[j], p1[j]], color=INK, lw=0.8, ls="--")

        for o in objects:
            col = colours[o["type"]]
            if o["type"] == "pole":
                p0 = np.array(o["p"]); p1 = p0 + [0, 0, o["h"]]
                ax.plot([p0[i], p1[i]], [p0[j], p1[j]], color=col, lw=3, alpha=0.8)
            elif o["type"] == "wire":
                ax.plot([o["p0"][i], o["p1"][i]], [o["p0"][j], o["p1"][j]], color=col, lw=2.5, alpha=0.8)
            else:
                c, s = np.array(o["c"]), np.array(o["size"])
                ax.add_patch(plt.Rectangle((c[i] - s[i] / 2, c[j] - s[j] / 2), s[i], s[j], fill=False, ec=col, lw=2))

        style_axes(ax, xl, yl)
        ax.set_aspect("equal")

    axes[0].set_xlim(-5 - d[0], 65 - d[0]), axes[0].set_ylim(-25 - d[1], 25 - d[1])
    axes[1].set_xlim(-5 - d[0], 65 - d[0]), axes[1].set_ylim(0 - d[2], 20 - d[2])
    for lab, col in colours.items():
        axes[0].plot([], [], color=col, lw=2.5, label=f"found {lab}")
    axes[0].plot([], [], color=INK, lw=0.8, ls="--", label="true object")
    axes[0].legend(loc="upper right", fontsize=8, frameon=False, labelcolor=INK2)
    axes[0].set_title(title, color=INK, fontsize=11, loc="left")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def write_objects(path, objects, ground_z, ned=None):
    """<tag>.objects.yaml: in PX4's local NED frame (ned: frames.LocalNed), or in the world frame."""
    clean = [{k: (round(v, 3) if isinstance(v, float) else v) for k, v in o.items() if not k.startswith("_")}
             for o in objects]
    if ned is None:
        doc = {"frame": "world_enu", "ground_z": round(ground_z, 3), "objects": clean}
    else:
        doc = {**ned.header(), "ground_d": round(ned.down(ground_z), 3), "objects": [ned.object(o) for o in clean]}
    with open(path, "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=None)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("tag", help="out/survey/<tag> (without extension)")
    p.add_argument("--pose", choices=("estimate", "truth"), default="estimate")
    p.add_argument("--doppler-gate", type=float, default=3.0)
    p.add_argument("--frame", choices=("ned", "world"), default="ned",
                   help="coordinates of what is written: PX4's local NED frame from EKF2's origin (default), or "
                        "the site's world frame (ENU)")
    args = p.parse_args()

    rng = np.random.default_rng(SEED)
    rec = load(args.tag + ".radar")
    world = rec["meta"]["world"]
    ulog = sp.EstimatedPose(args.tag + ".ulg", world)
    pts = sp.place(rec, args.pose, ulog, doppler_gate=args.doppler_gate)
    objects, ground_z = extract(pts, rng)
    ned = frames.LocalNed(ulog.origin, ulog.geo) if args.frame == "ned" else None

    out = args.tag + ("" if args.pose == "estimate" else "." + args.pose)
    os.makedirs(out + ".objects", exist_ok=True)
    # Only this run's objects: a name from an earlier run with more objects would stay behind.
    for old in os.listdir(out + ".objects"):
        if old.endswith(".ply"):
            os.remove(os.path.join(out + ".objects", old))
    names = {}
    for o in objects:
        names[o["type"]] = names.get(o["type"], 0) + 1
        o["name"] = f"{o['type']}_{names[o['type']]}"
        sp.write_ply(os.path.join(out + ".objects", o["name"] + ".ply"), o["_pts"], np.zeros(len(o["_pts"])), ned)
        # The sharp point cloud: poles and wires without the far detections' halo; blocks as they are.
        keep = sharpen(o["_pts"]) if o["type"] in SHARP_TYPES else np.ones(len(o["_pts"]), bool)
        sharp = o["_pts"][keep]
        sp.write_ply(os.path.join(out + ".objects", o["name"] + ".sharp.ply"), sharp, np.zeros(len(sharp)), ned)
        o["points_sharp"] = int(len(sharp))
        if o["type"] in SHARP_TYPES and len(sharp):
            o["spread_sharp"] = object_spread(o, sharp)

    write_objects(out + ".objects.yaml", objects, ground_z, ned)

    truth_path = os.path.join(sp.ROOT, "worlds", world + ".truth.yaml")
    if os.path.exists(truth_path):
        truth = sp.load_truth(world)
        rows, extra = match(objects, truth)
        print(f"{'true object':12s} {'found':10s} {'error (m)':>9s}  detail")
        for name, row in rows.items():
            if row is None:
                print(f"{name:12s} {'-':10s} {'':>9s}  missed")
                continue
            k, err, detail = row
            text = ", ".join(f"{a} {b:.2f}" if isinstance(b, float) else f"{a} {np.round(b, 1).tolist()}"
                             for a, b in detail.items())
            print(f"{name:12s} {objects[k]['name']:10s} {err:9.2f}  {text}; {objects[k]['points']} points")
        for k in extra:
            o = objects[k]
            where = o.get("c") or o.get("p") or o.get("p0")
            where = where if ned is None else ned.points(where)
            print(f"{'(none)':12s} {o['name']:10s} {'':>9s}  matches no true object, at {np.round(where, 1).tolist()}, "
                  f"{o['points']} points")
        plot(out + ".objects.png", objects, pts, truth, f"{os.path.basename(args.tag)}: objects ({args.pose} pose"
             + (f", {frames.NAME})" if ned is not None else ", world frame)"), ned)

    print(f"wrote {out}.objects.yaml, {out}.objects/ and {out}.objects.png, in "
          + (f"{frames.NAME} (origin at world {np.round(ned.origin, 2).tolist()})" if ned is not None else "the world frame"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
