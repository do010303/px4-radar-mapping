"""What the radar looks at: obstacles read from a world SDF, and labelling of detections.

The obstacles of a world are its static box and cylinder models, so a world
file is the only place they are defined. A sidecar <world>.targets.yaml next
to it gives the pass limit (minimum detection rate) per obstacle for the tests.

Detections are labelled in this order: on an obstacle (within a match
distance of its surface), ground (near a plane fitted to the detections below
the radar), or unexplained.
"""

import collections
import os
import xml.etree.ElementTree as ET

import numpy as np

# A detection counts as on an obstacle within MATCH m of its surface, widened
# with range because the radar's angle error is a lateral error that grows
# with distance; its range must also fall within the obstacle's range from the
# radar, give or take MATCH_RANGE m, since the range error stays small. Without
# that, at 500 m the ground 30 m in front of an obstacle would count as on it.
MATCH = 0.5
MATCH_PER_M = 0.07
MATCH_RANGE = 2.0
# Where the true origin of each detection is known (simulation), it must lie
# within MATCH_TRUE m of the obstacle instead. At long range the radar's angle
# error (15 m at 575 m for 1.5 deg) makes the ground at an obstacle's foot look
# like the obstacle; the true origin does not.
MATCH_TRUE = 1.0


class Obstacle(collections.namedtuple("Obstacle", "name centre size yaw min_rate")):
    """A static obstacle as an oriented box (cylinders as their bounding box).

    centre    (x, y, z) in the world (m)
    size      box extents along its own axes (m)
    yaw       rotation about z (rad)
    min_rate  pass limit for the detection rate, or None when not scored
    """

    def surface_distance(self, points):
        """Distance of each point from the box (0 inside)."""
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        d = np.atleast_2d(points) - np.asarray(self.centre)
        local = np.column_stack((c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1], d[:, 2]))
        outside = np.maximum(np.abs(local) - np.asarray(self.size) / 2.0, 0.0)
        return np.linalg.norm(outside, axis=1)

    def samples(self, n=5):
        """An n x n x n grid of points through the box, in the world (m)."""
        u = np.linspace(-0.5, 0.5, n)
        local = np.stack(np.meshgrid(u, u, u), axis=-1).reshape(-1, 3) * np.asarray(self.size)
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        return local @ np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]]) + np.asarray(self.centre)


def in_view(obstacle, position, rotation, sensor):
    """Whether any part of the obstacle lies within the sensor section of a radar profile.

    position, rotation  radar position and radar-to-world rotation
    """
    local = (obstacle.samples() - position) @ rotation
    rng = np.linalg.norm(local, axis=1)
    az = np.degrees(np.arctan2(local[:, 1], local[:, 0]))
    el = np.degrees(np.arcsin(local[:, 2] / np.maximum(rng, 1e-9)))
    full_circle = sensor["az_max_deg"] - sensor["az_min_deg"] >= 360
    ok = ((rng <= sensor["range_max"])
          & (full_circle | ((az >= sensor["az_min_deg"]) & (az <= sensor["az_max_deg"])))
          & (el >= sensor["el_min_deg"]) & (el <= sensor["el_max_deg"]))
    return bool(ok.any())


def load_obstacles(world_sdf, exclude=()):
    """Static box/cylinder models of a world SDF, with limits from <world>.targets.yaml."""
    limits = {}
    targets = os.path.splitext(world_sdf)[0] + ".targets.yaml"

    if os.path.exists(targets):
        import yaml

        with open(targets) as f:
            limits = yaml.safe_load(f) or {}

    obstacles = []

    for model in ET.parse(world_sdf).getroot().iter("model"):
        name = model.get("name")

        if name in exclude or name.startswith("wire_") or model.findtext("static", "false").strip() != "true":
            continue

        geometry = model.find("link/collision/geometry")

        if geometry is None:
            continue

        if geometry.find("box") is not None:
            size = tuple(float(v) for v in geometry.findtext("box/size").split())
        elif geometry.find("cylinder") is not None:
            r = float(geometry.findtext("cylinder/radius"))
            size = (2 * r, 2 * r, float(geometry.findtext("cylinder/length")))
        else:
            continue        # ground plane and other shapes

        pose = [float(v) for v in (model.findtext("pose") or "0 0 0 0 0 0").split()]
        obstacles.append(Obstacle(name, tuple(pose[:3]), size, pose[5], limits.get(name)))

    unknown = set(limits) - {o.name for o in obstacles}

    if unknown:
        raise ValueError(f"{targets} names obstacles that are not in {world_sdf}: {sorted(unknown)}")

    return obstacles


THIN_CYLINDER = 0.25       # m: static cylinders up to this radius also get the thin-cylinder radar model


def load_wires(world_sdf, poles=False):
    """Thin wires of a world SDF: its models named wire_*, each a cylinder.

    Returns (name, p0, p1, radius) per wire, the ends in the world (m). Their visuals are
    hidden from the radar's rays (visibility flag 2): radar_model.wire_detections handles them.
    With poles, also every other static cylinder up to THIN_CYLINDER in radius (poles, trunks,
    pylons): the ray grid sees those, but scores them by the area they cover in a resolution cell,
    far below what a cylinder actually reflects.
    """
    wires = []

    for model in ET.parse(world_sdf).getroot().iter("model"):
        name = model.get("name")
        geometry = model.find("link/collision/geometry/cylinder")

        if geometry is None:
            continue

        if not name.startswith("wire_") and not (
                poles and model.findtext("static", "false").strip() == "true"
                and float(geometry.findtext("radius")) <= THIN_CYLINDER):
            continue

        pose = [float(v) for v in (model.findtext("pose") or "0 0 0 0 0 0").split()]
        centre = np.array(pose[:3])
        roll, pitch, yaw = pose[3:]
        # The cylinder axis is the model's z axis; SDF rotations are roll, pitch, yaw about x, y, z.
        cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
        axis = np.array([cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr])
        half = float(geometry.findtext("length")) / 2.0
        wires.append((name, centre - half * axis, centre + half * axis, float(geometry.findtext("radius"))))

    return wires


def model_position(world_sdf, name):
    """Position of a top-level model in a world SDF."""
    for model in ET.parse(world_sdf).getroot().iter("model"):
        if model.get("name") == name:
            return np.array([float(v) for v in (model.findtext("pose") or "0 0 0").split()[:3]])

    raise KeyError(f"no model {name} in {world_sdf}")


def fit_ground(points, origin, rng=None, iterations=60, tolerance=0.15):
    """Fit the ground plane to the detections below the radar with RANSAC.

    Returns (normal, offset) with normal . p + offset = 0 on the plane and the
    normal pointing up, or None when too few detections lie below the radar.
    Only planes within 25 deg of level count as ground.
    """
    rng = rng or np.random.default_rng(0)
    below = points[points[:, 2] < origin[2] - 0.3]

    if len(below) < 8:
        return None

    best, best_count = None, 0

    for _ in range(iterations):
        a, b, c = below[rng.choice(len(below), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)

        if norm < 1e-6:
            continue

        n /= norm
        n *= np.sign(n[2]) or 1.0

        if n[2] < np.cos(np.radians(25)):
            continue

        count = int((np.abs(below @ n - n @ a) < tolerance).sum())

        if count > best_count:
            best, best_count = (n, -n @ a), count

    if best is None:
        return None

    # Refine on the inliers: least-squares plane through them.
    n, d = best
    inliers = below[np.abs(below @ n + d) < tolerance]
    centroid = inliers.mean(axis=0)
    # Not full_matrices: that would also build an N x N matrix, ~80 GB for
    # the 100 000 points of a whole hover recording.
    n = np.linalg.svd(inliers - centroid, full_matrices=False)[2][-1]
    n *= np.sign(n[2]) or 1.0
    return n, -n @ centroid


Labels = collections.namedtuple("Labels", "on distance ground unexplained plane")


def classify(points, origin, obstacles, sigma_el_deg, plane=None, truth=None):
    """Label detections (world frame) seen from origin.

    truth, if given, is where each detection really came from (world frame):
    a detection is then on an obstacle when its origin is, whatever its error.

    on           {obstacle name: mask of detections on it}
    distance     {obstacle name: distance of every detection from its surface}
    ground       mask of ground returns: within twice the radar's elevation
                 error of the ground plane (fitted unless given)
    unexplained  mask of the rest
    plane        the ground plane used, or None if none could be fitted
    """
    rng = np.linalg.norm(points - origin, axis=1)
    on, distance = {}, {}
    claimed = np.zeros(len(points), dtype=bool)

    # Obstacles first: near an obstacle's base the radar cannot tell it from the ground either.
    for o in obstacles:
        distance[o.name] = o.surface_distance(points)

        if truth is not None:
            on[o.name] = o.surface_distance(truth) < MATCH_TRUE
        else:
            near = o.surface_distance(origin)[0]
            far = np.linalg.norm(o.samples(2) - origin, axis=1).max()     # farthest corner
            on[o.name] = ((distance[o.name] < MATCH + MATCH_PER_M * rng)
                          & (rng > near - MATCH_RANGE) & (rng < far + MATCH_RANGE))
        claimed |= on[o.name]

    if plane is None:
        plane = fit_ground(points[~claimed], origin)

    if plane is None:
        ground = np.zeros(len(points), dtype=bool)
    else:
        n, d = plane
        ground = ~claimed & (np.abs(points @ n + d) < 0.3 + 2.0 * rng * np.sin(np.radians(sigma_el_deg)))

    return Labels(on, distance, ground, ~claimed & ~ground, plane)
