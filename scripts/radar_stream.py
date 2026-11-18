"""Live radar frames from Gazebo: the radar model's detections plus where the radar was.

Subscribes to the radar's ray grid and to the world's pose list. Every scan
becomes a Frame holding the detections in the radar frame together with the
radar's world pose and velocity, so consumers can draw them in the world or
score them against known obstacles.
"""

import bisect
import collections
import threading
import time

import numpy as np

from radar_model import detect, scan_to_grid    # first: sets up the gz bindings
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node


def quat_to_matrix(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


class Frame(collections.namedtuple("Frame", "stamp det position rotation velocity truth", defaults=(None,))):
    """One radar frame.

    stamp     simulation time of the scan (s)
    det       (N, 5) detections in the radar frame: x, y, z, radial velocity, SNR dB
    position  radar position in the world (m)
    rotation  radar-to-world rotation matrix
    velocity  radar velocity in the radar frame (m/s), from the pose history
    truth     (N, 3) where each detection really came from, radar frame
              (radar_model.detect with truth): for scoring, not a radar output
    """

    def world_points(self):
        return self.det[:, :3] @ self.rotation.T + self.position

    def world_truth(self):
        return None if self.truth is None else self.truth @ self.rotation.T + self.position


HISTORY = 1.0               # s of simulation time of poses kept, for PoseFeed.at


class PoseFeed:
    """World poses and velocities of some links of one model, from the world's pose list.

    One subscription for any number of radars on a vehicle: decoding a Pose_V costs most of a
    millisecond in Python and the world sends hundreds a second, so six radars each decoding it
    on their own took a whole CPU core and the radar frames fell seconds behind.
    """

    def __init__(self, node, world, model, links):
        self.model = model
        self.links = set(links)
        self.lock = threading.Lock()
        self.poses = {}             # link -> (stamp, position, rotation) in the world
        self.velocities = {}        # link -> velocity in the world (m/s)
        self.history = {}           # link -> deque of (stamp, position, rotation, velocity), the last HISTORY s

        if not node.subscribe(Pose_V, f"/world/{world}/pose/info", self._on_poses):
            raise RuntimeError(f"failed to subscribe to /world/{world}/pose/info")

    def get(self, link):
        """(stamp, position, rotation, velocity in the world) of a link, or None before its first pose."""
        with self.lock:
            pose = self.poses.get(link)
            return None if pose is None else (*pose, self.velocities[link].copy())

    def at(self, link, stamp):
        """(stamp, position, rotation, velocity) of a link at simulation time stamp: the position and velocity
        interpolated between the poses either side, the rotation the nearer one's (poses come every few ms).
        The newest pose when stamp is past it, None when stamp is older than the history.

        The newest pose is not where the radar was when it scanned: up to MAX_LAG (0.15 s) later, 0.75 m
        further on at 5 m/s, and turned by whatever the vehicle turned meanwhile.
        """
        with self.lock:
            history = self.history.get(link)

            if not history:
                return None

            if stamp >= history[-1][0]:
                s, p, r, v = history[-1]
                return s, p, r, v.copy()

            if stamp < history[0][0]:
                return None

            stamps = [h[0] for h in history]
            i = bisect.bisect_right(stamps, stamp)
            s0, p0, r0, v0 = history[i - 1]
            s1, p1, r1, v1 = history[i]
            w = (stamp - s0) / max(s1 - s0, 1e-9)
            return stamp, p0 + w * (p1 - p0), (r0 if w < 0.5 else r1), v0 + w * (v1 - v0)

    def _on_poses(self, msg):
        model, links = None, {}

        # Top-level models are posed in the world, their links relative to the model.
        for pose in msg.pose:
            if pose.name == self.model and model is None:
                model = pose
            elif pose.name in self.links and pose.name not in links:
                links[pose.name] = pose

        if model is None:
            return

        r_model = quat_to_matrix(model.orientation)
        p_model = np.array([model.position.x, model.position.y, model.position.z])
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9

        with self.lock:
            for name, link in links.items():
                position = p_model + r_model @ np.array([link.position.x, link.position.y, link.position.z])
                rotation = r_model @ quat_to_matrix(link.orientation)
                velocity = self.velocities.get(name, np.zeros(3))
                previous = self.poses.get(name)

                if previous is not None and stamp - previous[0] > 1e-4:
                    # Light smoothing: pose updates come at uneven sim-time intervals.
                    velocity = 0.5 * velocity + 0.5 * (position - previous[1]) / (stamp - previous[0])

                self.poses[name] = (stamp, position, rotation)
                self.velocities[name] = velocity
                history = self.history.setdefault(name, collections.deque())

                if history and stamp <= history[-1][0]:
                    continue

                history.append((stamp, position, rotation, velocity.copy()))

                while stamp - history[0][0] > HISTORY:
                    history.popleft()


class RadarStream:
    """Turns every radar scan into a Frame and hands it to on_frame.

    model    name of the top-level model carrying the radar in Gazebo
    link     name of the radar link inside that model
    poses    a PoseFeed shared with other radars of the vehicle (one is made otherwise)
    max_lag  drop a scan older than this (s of simulation time) than the newest pose, rather than
             fall behind when the processing cannot keep up; None keeps every scan
    """

    def __init__(self, cfg, world, model, on_frame, link="radar_link", topic="/radar/scan", seed=1,
                 poses=None, node=None, max_lag=None):
        self.cfg = cfg
        self.model = model
        self.link = link
        self.on_frame = on_frame
        self.max_lag = max_lag
        self.rng = np.random.default_rng(seed)
        self.frames = 0
        self.scans_without_pose = 0
        self.scans_dropped = 0      # too old to use (max_lag)
        self.angles = None          # (az_min, az_max, el_min, el_max) of the scan, rad
        self.last_scan = None       # time.monotonic() of the latest scan

        self.node = node or Node()
        self.poses = poses or PoseFeed(self.node, world, model, [link])

        if not self.node.subscribe(LaserScan, topic, self._on_scan):
            raise RuntimeError(f"failed to subscribe to {topic}")

    def _on_scan(self, msg):
        self.last_scan = time.monotonic()

        if self.angles is None:
            self.angles = (msg.angle_min, msg.angle_max, msg.vertical_angle_min, msg.vertical_angle_max)

        latest = self.poses.get(self.link)

        if latest is None:
            self.scans_without_pose += 1
            return

        scan_stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9

        if self.max_lag is not None and latest[0] - scan_stamp > self.max_lag:
            self.scans_dropped += 1
            return

        # Where the radar was when it scanned, not where it is now.
        pose = self.poses.at(self.link, scan_stamp) or latest
        _, position, rotation, velocity_world = pose

        velocity = rotation.T @ velocity_world
        ranges, az, el = scan_to_grid(msg)
        det, truth = detect(ranges, az, el, self.cfg, self.rng, velocity, truth=True)
        self.frames += 1
        self.on_frame(Frame(scan_stamp, det, position, rotation, velocity, truth))
