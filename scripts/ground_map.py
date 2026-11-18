#!/usr/bin/env python3
"""The ground station's map, from what came over the link (docs/survey_mapping_plan.md, phase 5).

Decodes the datagrams of scripts/radar_link.py (pose, confirmed voxels, decimated raw detections), keeps
the map they describe, and how late each one arrived (from the radar scan its data comes from to its
arrival). At the end, finds the objects in it (scripts/survey_objects.py's pole, wire and other finders,
on the received detections inside confirmed voxels, plus the centres of confirmed voxels no detection of
which got through) and scores them against the world's truth.

    scripts/ground_map.py out/survey/rtk_3                  # replay <tag>.link.npz, as fast as possible
    scripts/ground_map.py out/survey/rtk_3 --speed 1 --rviz # at the flight's pace, shown in RViz:
    rviz2 -d rviz/ground.rviz                               #   (source env.sh first)
    scripts/ground_map.py --udp 14600 --rviz --world survey_site   # live, from radar_link's UDP sender

RViz topics (frame map): /ground/voxels (confirmed voxels above the ground, intensity: hits),
/ground/voxels_sharp (those that stand out in their neighbourhood by range-weighted hits: GroundMap.sharp),
/ground/objects (MarkerArray: the poles, wires and blocks found, every OBJECT_PERIOD s),
/ground/ground (ground voxels), /ground/points (the detections received), /ground/vehicle (Marker),
/ground/path (nav_msgs/Path), /ground/truth (the world's true objects, Marker, for comparison),
/ground/frame (PX4's local NED axes at EKF2's origin) and /ground/labels (each object's name and position in
that frame), once the vehicle's ORIGIN has come over the link.
Writes <tag>.ground.png and <tag>.ground.objects.yaml (in PX4's local NED frame: scripts/frames.py).
"""

import argparse
import os
import socket
import sys
import threading
import time
import warnings

import numpy as np
import yaml

warnings.filterwarnings("ignore", message="A NumPy version")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import frames  # noqa: E402
import onboard_mapper as om  # noqa: E402
import radar_link as rl  # noqa: E402
import survey_objects as so  # noqa: E402
import survey_process as sp  # noqa: E402

LINK_LATENCY = 0.05         # s on the radio link itself, added to the time a datagram was sent
MAX_POINTS = 400000         # received detections kept (the oldest go)
SHARP_RADIUS, SHARP_SHARE, SHARP_MIN = so.SHARP_RADIUS, so.SHARP_SHARE, so.SHARP_MIN   # as an object's sharp cloud
OBJECT_PERIOD = 3.0         # s between object searches for RViz (about 0.5 s each)
CENTRE_WEIGHT = 10          # a confirmed voxel without received detections stands for at most this many


class GroundMap:
    def __init__(self):
        self.voxels = {}            # key -> (hits, flags)
        self.points = []
        self.n_points = 0
        self.poses = []             # (t, x, y, z, yaw, speed)
        self.latency = {rl.POSE: [], rl.VOXEL: [], rl.POINTS: [], rl.ORIGIN: []}
        self.datagrams = 0
        self.ned = None             # frames.LocalNed, once the vehicle's ORIGIN has come

    def handle(self, data, t_recv):
        kind, stamp, rec = rl.decode(data)
        self.datagrams += 1
        # The stamps are milliseconds modulo 2^32 (49 days): unwrap against the arrival time. Live (t_recv None)
        # the vehicle's clock (simulation time) is not the ground's: no latency then.
        if t_recv is not None:
            stamp += round((t_recv - stamp) / 4294967.296) * 4294967.296
            self.latency[kind].append(t_recv - stamp)

        if kind == rl.ORIGIN:
            r = rec[0]
            geo = None if np.isnan(r["lat"]) else (float(r["lat"]), float(r["lon"]), float(r["alt"]))
            self.ned = frames.LocalNed(r["enu"].astype(float), geo)
        elif kind == rl.POSE:
            self.poses.append((stamp, *(float(rec[f][0]) for f in ("x", "y", "z", "yaw", "speed"))))
        elif kind == rl.VOXEL:
            for k, h, f, w in zip(map(tuple, rec["k"]), rec["hits"], rec["flags"], rec["weight"]):
                self.voxels[k] = (int(h), int(f), int(w))
        else:
            pts = rl.points_from(rec)
            self.points.append(pts)
            self.n_points += len(pts)
            while self.n_points > MAX_POINTS and len(self.points) > 1:
                self.n_points -= len(self.points.pop(0))

        return kind

    def arrays(self):
        """(centres (n, 3), hits (n,), ground (n,)) of the voxels, and the received points (m, 5)."""
        if self.voxels:
            keys = np.array(list(self.voxels))
            vals = np.array(list(self.voxels.values()))
            centres = (keys + 0.5) * om.VOXEL
        else:
            centres, vals = np.empty((0, 3)), np.empty((0, 3), int)
        pts = np.vstack(self.points) if self.points else np.empty((0, 5))
        return centres, vals[:, 0], vals[:, 1] == 1, pts

    def snapshot(self):
        """A copy of the map (voxels and points) to search for objects in another thread."""
        snap = GroundMap()
        snap.voxels = dict(self.voxels)
        snap.points = list(self.points)
        snap.ned = self.ned
        return snap

    def sharp(self):
        """(centres, weight) of the voxels above the ground that stand out in their neighbourhood: weight at
        least SHARP_SHARE of the most within SHARP_RADIUS. The radar's angle error smears a pole over a metre
        or two each way; the near, many-times-seen middle of it is what this keeps."""
        from scipy.spatial import cKDTree

        if not self.voxels:
            return np.empty((0, 3)), np.empty(0)
        keys = np.array(list(self.voxels))
        vals = np.array(list(self.voxels.values()))
        above = vals[:, 1] == 0
        centres, weight = (keys[above] + 0.5) * om.VOXEL, vals[above, 2] / rl.WEIGHT_SCALE
        if not len(centres):
            return centres, weight
        tree = cKDTree(centres)
        local = np.array([weight[n].max() for n in tree.query_ball_point(centres, SHARP_RADIUS)])
        keep = (weight >= SHARP_SHARE * local) & (weight >= SHARP_MIN)
        return centres[keep], weight[keep]

    def objects(self):
        """The objects in the map: survey_objects' finders on the detections in confirmed voxels above the ground."""
        centres, hits, ground, pts = self.arrays()
        above = {tuple(k) for k, g in zip(np.floor(centres / om.VOXEL).astype(np.int64), ground) if not g}
        ground_z = float(np.median(centres[ground, 2]) - om.VOXEL / 2) if ground.any() else 0.0

        if len(pts):
            keys = map(tuple, np.floor(pts[:, :3] / om.VOXEL).astype(np.int64))
            inside = np.fromiter((k in above for k in keys), bool, len(pts))
            pts = pts[inside, :3]
            covered = set(map(tuple, np.floor(pts / om.VOXEL).astype(np.int64)))
        else:
            pts, covered = np.empty((0, 3)), set()

        missing = [(np.array(k) + 0.5) * om.VOXEL for k in above if k not in covered]
        weights = [min(self.voxels[k][0], CENTRE_WEIGHT) for k in above if k not in covered]
        extra = np.repeat(np.array(missing).reshape(-1, 3), weights, axis=0) if missing else np.empty((0, 3))
        xyz = np.vstack((pts, extra))
        objects = so.find_objects(xyz, ground_z, np.random.default_rng(so.SEED))
        return objects, ground_z, len(pts), len(extra)


class Viewer:
    """The ground map in RViz."""

    def __init__(self, truth=None):
        import rclpy
        from geometry_msgs.msg import Point, PoseStamped
        from nav_msgs.msg import Path
        from sensor_msgs.msg import PointCloud2, PointField
        from visualization_msgs.msg import Marker

        rclpy.init()
        self.node = rclpy.create_node("ground_map")
        self.rclpy, self.Point, self.PoseStamped, self.Marker = rclpy, Point, PoseStamped, Marker
        self.PointCloud2, self.PointField, self.Path = PointCloud2, PointField, Path
        pub = lambda t, name: self.node.create_publisher(t, name, 1)  # noqa: E731
        self.voxel_pub, self.ground_pub = pub(PointCloud2, "/ground/voxels"), pub(PointCloud2, "/ground/ground")
        self.points_pub = pub(PointCloud2, "/ground/points")
        self.sharp_pub = pub(PointCloud2, "/ground/voxels_sharp")
        from visualization_msgs.msg import MarkerArray
        self.MarkerArray = MarkerArray
        self.objects_pub = pub(MarkerArray, "/ground/objects")
        self.labels_pub = pub(MarkerArray, "/ground/labels")
        self.frame_pub = pub(MarkerArray, "/ground/frame")
        self.objects = None
        self.vehicle_pub, self.path_pub, self.truth_pub = pub(Marker, "/ground/vehicle"), pub(Path, "/ground/path"), \
            pub(Marker, "/ground/truth")
        self.truth = truth

    def _cloud(self, stamp, xyz, extra, names):
        fields = [self.PointField(name=n, offset=4 * i, datatype=self.PointField.FLOAT32, count=1)
                  for i, n in enumerate(["x", "y", "z"] + names)]
        data = np.column_stack([xyz] + ([extra] if extra is not None else [])).astype("<f4")
        c = self.PointCloud2()
        c.header.stamp, c.header.frame_id = stamp, "map"
        c.height, c.width, c.fields = 1, len(data), fields
        c.point_step, c.row_step, c.is_dense = 4 * data.shape[1], 4 * data.shape[1] * len(data), True
        c.data = data.tobytes()
        return c

    def _object_markers(self, stamp, objects, ground_z):
        arr = self.MarkerArray()
        clear = self.Marker()
        clear.action = self.Marker.DELETEALL
        arr.markers.append(clear)
        for i, o in enumerate(objects):
            m = self.Marker()
            m.header.stamp, m.header.frame_id = stamp, "map"
            m.ns, m.id, m.action = o["type"], i, self.Marker.ADD
            m.pose.orientation.w = 1.0
            if o["type"] == "pole":
                m.type = self.Marker.CYLINDER
                m.pose.position.x, m.pose.position.y = o["p"][0], o["p"][1]
                m.pose.position.z = ground_z + o["h"] / 2
                m.scale.x, m.scale.y, m.scale.z = 0.35, 0.35, float(o["h"])
                m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.55, 1.0, 0.9
            elif o["type"] == "wire":
                m.type = self.Marker.LINE_LIST
                m.scale.x = 0.12
                m.points = [self.Point(x=float(q[0]), y=float(q[1]), z=float(q[2])) for q in (o["p0"], o["p1"])]
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.45, 0.1, 1.0
            else:
                m.type = self.Marker.CUBE
                m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in o["c"])
                m.scale.x, m.scale.y, m.scale.z = (max(float(v), 0.2) for v in o["size"])
                m.color.r, m.color.g, m.color.b, m.color.a = 0.25, 0.8, 0.35, 0.45
            arr.markers.append(m)
        return arr

    def _text(self, stamp, ns, i, at, text, size=0.6):
        m = self.Marker()
        m.header.stamp, m.header.frame_id = stamp, "map"
        m.ns, m.id, m.type, m.action = ns, i, self.Marker.TEXT_VIEW_FACING, self.Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in at)
        m.pose.orientation.w = 1.0
        m.scale.z = size
        m.color.r = m.color.g = m.color.b = m.color.a = 1.0
        m.text = text
        return m

    def _labels(self, stamp, objects, ground_z, ned):
        """Each object's name and position in PX4's local NED frame, above it."""
        arr = self.MarkerArray()
        clear = self.Marker()
        clear.action = self.Marker.DELETEALL
        arr.markers.append(clear)
        count = {}
        for i, o in enumerate(objects):
            count[o["type"]] = count.get(o["type"], 0) + 1
            name = f"{o['type']}_{count[o['type']]}"
            n = ned.object(o)
            if o["type"] == "pole":
                at, where = np.array(o["p"][:2] + [ground_z + o["h"] + 1.0]), n["p"]
                text = f"{name}  N {where[0]:.1f}  E {where[1]:.1f}  h {o['h']:.1f}"
            elif o["type"] == "wire":
                mid = (np.array(o["p0"]) + np.array(o["p1"])) / 2
                at, a, b = mid + [0, 0, 0.8], n["p0"], n["p1"]
                text = f"{name}  N {a[0]:.1f}..{b[0]:.1f}  E {a[1]:.1f}..{b[1]:.1f}  D {(a[2] + b[2]) / 2:.1f}"
            else:
                at, c = np.array(o["c"]) + [0, 0, o["size"][2] / 2 + 0.8], n["c"]
                text = f"{name}  N {c[0]:.1f}  E {c[1]:.1f}  D {c[2]:.1f}"
            arr.markers.append(self._text(stamp, "labels", i, at, text, 0.5))
        return arr

    def _frame(self, stamp, ned):
        """PX4's local NED axes at EKF2's origin: N red, E green, D blue, 3 m each."""
        arr = self.MarkerArray()
        o = ned.origin
        for i, (axis, col) in enumerate(((ned.world([3, 0, 0]), (1.0, 0.2, 0.2)), (ned.world([0, 3, 0]), (0.2, 1.0, 0.2)),
                                         (ned.world([0, 0, 3]), (0.3, 0.5, 1.0)))):
            m = self.Marker()
            m.header.stamp, m.header.frame_id = stamp, "map"
            m.ns, m.id, m.type, m.action = "ned_axes", i, self.Marker.ARROW, self.Marker.ADD
            m.pose.orientation.w = 1.0
            m.points = [self.Point(x=float(o[0]), y=float(o[1]), z=float(o[2])),
                        self.Point(x=float(axis[0]), y=float(axis[1]), z=float(axis[2]))]
            m.scale.x, m.scale.y, m.scale.z = 0.15, 0.35, 0.5
            m.color.r, m.color.g, m.color.b, m.color.a = *col, 1.0
            arr.markers.append(m)
            arr.markers.append(self._text(stamp, "ned_axes_text", i, axis + [0, 0, 0.5], "NED"[i], 0.7))
        arr.markers.append(self._text(stamp, "ned_axes_text", 3, o + [0, 0, 1.2], "PX4 local NED (EKF origin)", 0.5))
        return arr

    def show(self, gm):
        stamp = self.node.get_clock().now().to_msg()
        centres, hits, ground, pts = gm.arrays()
        self.voxel_pub.publish(self._cloud(stamp, centres[~ground], hits[~ground, None].astype(float), ["hits"]))
        self.ground_pub.publish(self._cloud(stamp, centres[ground], None, []))
        self.points_pub.publish(self._cloud(stamp, pts[:, :3], pts[:, 4:5], ["snr"]))
        sc, sw = gm.sharp()
        self.sharp_pub.publish(self._cloud(stamp, sc, sw[:, None], ["weight"]))
        if self.objects is not None:
            self.objects_pub.publish(self._object_markers(stamp, *self.objects))
            if gm.ned is not None:
                self.labels_pub.publish(self._labels(stamp, self.objects[0], self.objects[1], gm.ned))
        if gm.ned is not None:
            self.frame_pub.publish(self._frame(stamp, gm.ned))

        if gm.poses:
            t, x, y, z, yaw, _ = gm.poses[-1]
            m = self.Marker()
            m.header.stamp, m.header.frame_id = stamp, "map"
            m.ns, m.id, m.type, m.action = "vehicle", 0, self.Marker.ARROW, self.Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
            m.pose.orientation.z, m.pose.orientation.w = np.sin(yaw / 2), np.cos(yaw / 2)
            m.scale.x, m.scale.y, m.scale.z = 1.5, 0.4, 0.4
            m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.9, 0.3, 1.0
            self.vehicle_pub.publish(m)
            path = self.Path()
            path.header.stamp, path.header.frame_id = stamp, "map"
            for p in gm.poses[::5]:
                ps = self.PoseStamped()
                ps.header = path.header
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = p[1], p[2], p[3]
                ps.pose.orientation.w = 1.0
                path.poses.append(ps)
            self.path_pub.publish(path)

        if self.truth:
            m = self.Marker()
            m.header.stamp, m.header.frame_id = stamp, "map"
            m.ns, m.id, m.type, m.action = "truth", 0, self.Marker.LINE_LIST, self.Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = 0.05
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 1.0, 0.5
            for seg in truth_segments(self.truth):
                m.points += [self.Point(x=float(q[0]), y=float(q[1]), z=float(q[2])) for q in seg]
            self.truth_pub.publish(m)

        self.rclpy.spin_once(self.node, timeout_sec=0.0)


def truth_segments(truth):
    """Line segments outlining the true objects."""
    out = []
    for t in truth.values():
        if t["type"] == "box":
            c, h = np.array(t["c"], float), np.array(t["size"], float) / 2
            corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h + c
            for i in range(8):
                for j in range(i + 1, 8):
                    if np.sum(np.abs(np.sign(corners[i] - c) - np.sign(corners[j] - c)) > 0) == 1:
                        out.append((corners[i], corners[j]))
        else:
            p0, p1, _ = sp.segment(t)
            out.append((p0, p1))
    return out


def report(gm, truth, out, title):
    lat = {k: np.array(v) * 1e3 for k, v in gm.latency.items() if v}
    names = {rl.POSE: "pose", rl.VOXEL: "voxels", rl.POINTS: "points", rl.ORIGIN: "origin"}
    centres, hits, ground, pts = gm.arrays()
    print(f"{gm.datagrams} datagrams: {len(centres)} voxels ({int(ground.sum())} ground), {len(pts)} detections, "
          f"{len(gm.poses)} poses")
    print("arrival after the scan (ms): " + ("; ".join(
        f"{names[k]} median {np.median(v):.0f}, 95 % {np.percentile(v, 95):.0f}, max {v.max():.0f}" for k, v in lat.items()) or "not measured live (the vehicle's clock is the simulation's)"))

    objects, ground_z, n_pts, n_extra = gm.objects()
    print(f"objects from {n_pts} detections in confirmed voxels + {n_extra} for voxels without any "
          f"(ground at z {ground_z:+.2f} m): {sum(o['type'] == 'pole' for o in objects)} poles, "
          f"{sum(o['type'] == 'wire' for o in objects)} wires, {sum(o['type'] == 'box' for o in objects)} other")
    names_by_type = {}
    for o in objects:
        names_by_type[o["type"]] = names_by_type.get(o["type"], 0) + 1
        o["name"] = f"{o['type']}_{names_by_type[o['type']]}"

    if truth:
        rows, extra = so.match(objects, truth)
        for name, row in rows.items():
            if row is None:
                print(f"  {name:12s} missed")
            else:
                k, err, detail = row
                text = ", ".join(f"{a} {b:.2f}" if isinstance(b, float) else f"{a} {np.round(b, 1).tolist()}"
                                 for a, b in detail.items())
                print(f"  {name:12s} {objects[k]['name']:9s} {err:5.2f} m  {text}")
        print(f"  {len(extra)} found objects match nothing")
        above = centres[~ground]
        d, labels, _ = sp.score(above, truth)
        sp.plot(out + ".ground.png", above, d, truth, title)

    if out:
        so.write_objects(out + ".ground.objects.yaml", objects, ground_z, gm.ned)
        print(f"wrote {out}.ground.png and {out}.ground.objects.yaml"
              + (f" (in {frames.NAME})" if gm.ned is not None else " (in the world frame: no ORIGIN received)"))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("tag", nargs="?", help="out/survey/<tag>: replay <tag>.link.npz (scripts/radar_link.py)")
    p.add_argument("--speed", type=float, default=0.0, help="replay: 1 real time, 0 as fast as possible")
    p.add_argument("--link-latency", type=float, default=LINK_LATENCY)
    p.add_argument("--udp", type=int, default=None, metavar="PORT", help="live: receive on this UDP port")
    p.add_argument("--world", default=None, help="for the truth (default: the recording's)")
    p.add_argument("--rviz", action="store_true", help="publish the map for rviz/ground.rviz")
    p.add_argument("--rate", type=float, default=2.0, help="RViz updates per second")
    args = p.parse_args()

    world = args.world
    if world is None and args.tag:
        world = yaml.safe_load(open(args.tag + ".radar/meta.yaml"))["world"]
    truth = sp.load_truth(world) if world and os.path.exists(os.path.join(sp.ROOT, "worlds", world + ".truth.yaml")) else None
    gm = GroundMap()
    viewer = Viewer(truth) if args.rviz else None
    last_show = 0.0

    last_objects = 0.0
    worker = None

    def search(snap):
        objects, ground_z, _, _ = snap.objects()
        viewer.objects = (objects, ground_z)

    def maybe_show():
        nonlocal last_show, last_objects, worker
        if viewer is None:
            return
        # The object search takes seconds: on a copy of the map, in a thread of its own, one at a time.
        if gm.voxels and (worker is None or not worker.is_alive()) and time.monotonic() - last_objects >= OBJECT_PERIOD:
            last_objects = time.monotonic()
            worker = threading.Thread(target=search, args=(gm.snapshot(),), daemon=True)
            worker.start()
        if time.monotonic() - last_show >= 1.0 / args.rate:
            last_show = time.monotonic()
            viewer.show(gm)

    if args.udp is not None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", args.udp))
        sock.settimeout(0.2)
        idle_since = None
        print(f"listening on UDP {args.udp}", flush=True)
        try:
            while True:
                try:
                    data = sock.recv(65535)
                    gm.handle(data, None)
                    idle_since = None
                except socket.timeout:
                    idle_since = idle_since or time.monotonic()
                    if gm.datagrams and time.monotonic() - idle_since > 15.0:
                        break               # the flight is over
                maybe_show()
        except KeyboardInterrupt:
            pass
        report(gm, truth, args.tag, "live: map received on the ground")
        return 0

    link = np.load(args.tag + ".link.npz", allow_pickle=True)
    t_sent, data = link["t"], link["data"]
    start_wall = time.monotonic()

    for t, d in zip(t_sent, data):
        t_recv = t + args.link_latency
        if args.speed > 0:
            wait = (t_recv - t_sent[0]) / args.speed - (time.monotonic() - start_wall)
            if wait > 0:
                time.sleep(wait)
        gm.handle(d, t_recv)
        maybe_show()

    if viewer is not None:
        viewer.show(gm)
    report(gm, truth, args.tag, f"{os.path.basename(args.tag)}: map received on the ground "
                                f"({float(link['kbps']):g} kbit/s link)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
