#!/usr/bin/env python3
"""The radars of models/x500_radar3 (level ring), x500_radar6 (3 x 3) or x500_radar_aop (1 ISK + 5 AOP) as one
sensor, published to ROS 2.

Each radar's scan becomes detections (radar_model.detect), plus the wires and poles of the world, which
the ray grid cannot show (radar_model.wire_detections); the detections of all radars are moved into the
vehicle frame (base_link, FLU) and merged. Mounts: scripts/radar_layouts.py. Published, for RViz and for
scripts/avoid_node.py:

  /radar/points   sensor_msgs/PointCloud2 in frame base_link: x, y, z, velocity, snr, radar (index),
                  the detections of all radars from the last 0.25 s of simulation time
  /radar/points_map  the same detections in frame map (world axes), where they were when detected (each
                  radar's frame placed with the pose at its own scan time, PoseFeed.at): a vehicle that has
                  moved since can place them from its own position
  Detections whose radial velocity does not fit a static point where they are placed are dropped first
  (doppler_consistent, --doppler-gate).
  /radar/pose     geometry_msgs/PointStamped, frame map: where base_link was for the newest cloud, stamped
                  with the simulation time of that radar frame; against PX4's clock, how far behind the radars are
  With --log DIR, every radar frame is also recorded as the hardware gives it (scripts/radar_log.py): its
  detections in the radar's own frame, before the Doppler gate, for mapping after the flight.
  /radar/fov      visualization_msgs/MarkerArray: the fields of view, frame base_link
  /radar/wires    visualization_msgs/Marker: the true wires of the world, frame map (for checking)
  TF map -> base_link   from the Gazebo ground truth

    ./radar_array.py --world power_line --layout 3      # against run_sitl.sh RADARS=3 (RADARS=6: --layout 6)
"""

import argparse
import copy
import os
import signal
import socket
import struct
import sys
import threading
import time

import numpy as np

from radar_layouts import VEHICLE, layout_key, mounts, profile_path
from radar_log import RadarLog
from radar_model import add_model_args, display_angles, fov_segments, load_profile, wire_detections
from gz.transport13 import Node
from radar_stream import PoseFeed, RadarStream
from radar_viz import matrix_to_quat
from scene import load_wires

FRAME_HEADER = struct.Struct("<dBH")    # --frames-udp: stamp (s), radar, detections; then (n, 5) float32
MAX_AGE = 0.25              # s of simulation time a radar's detections stay in the merged cloud
MAX_LAG = 0.15              # s of simulation time a scan may be behind the newest pose and still be used
FIELDS = ("x", "y", "z", "velocity", "snr", "radar")
COLOURS = ((0.95, 0.85, 0.25), (0.35, 0.8, 0.95), (0.95, 0.5, 0.8), (0.5, 0.95, 0.4), (1.0, 0.6, 0.2), (0.7, 0.6, 1.0))


class RadarArray:
    """Merged body-frame detections of the radars of a layout (scripts/radar_layouts.py).

    on_cloud(stamp, det, position, rotation) is called with every radar frame: det is (N, 6) in
    base_link (x, y, z, radial velocity, SNR, radar index), position and rotation are those of
    base_link in the world.
    """

    log = None      # a radar_log.RadarLog (--log): every frame is recorded there before the Doppler gate
    frames_to = None    # (socket, port) (--frames-udp): every frame, as recorded, to the on-board mapper

    def __init__(self, cfg, profile, world, model, wires, on_cloud, layout=3, seed=1, doppler_gate=3.0,
                 sensitivity=None):
        self.mounts = mounts(layout)
        self.doppler_gate = doppler_gate
        self.doppler_rejected = 0
        # Each radar's model from its own profile (x500_radar_aop mixes ISK and AOP); the command line's model
        # options apply to the radars of the default profile.
        self.profiles, self.cfgs = [], []

        for name, _, _ in self.mounts:
            path = profile_path(name)

            if load_profile(path) == profile:
                self.profiles.append(profile)
                self.cfgs.append(copy.copy(cfg))        # own copy: the sensitivity changes below apply once each
            else:
                p = argparse.ArgumentParser()
                add_model_args(p, path)
                self.profiles.append(load_profile(path))
                self.cfgs.append(p.parse_args([]))
                # A test knob for every radar, not only those of the command line's profile.
                self.cfgs[-1].angle_outlier = getattr(cfg, "angle_outlier", 0.0)

        # Sensitivity tests (docs/survey_mapping_plan.md): the same change to every radar, whatever its profile.
        for c in self.cfgs:
            s = sensitivity or {}
            c.sigma_az *= s.get("angle_scale", 1.0)
            c.sigma_el *= s.get("angle_scale", 1.0)
            c.surface_db = s.get("surface_db", 0.0)
            c.surface_law = s.get("surface_law", 40.0)
            c.thin_db = s.get("thin_db", 0.0)
            c.per_range_bin = int(round(c.per_range_bin * s.get("points_scale", 1.0)))
            c.max_points = int(round(c.max_points * s.get("points_scale", 1.0)))

        self.wires = wires
        self.on_cloud = on_cloud
        self.lock = threading.Lock()
        self.latest = {}
        self.rng = np.random.default_rng(seed + 100)
        links = [f"radar_link_{name}" for name, _, _ in self.mounts]
        # One gz node and one pose subscription for all radars (radar_stream.PoseFeed); a scan the
        # processing could not take up within MAX_LAG is dropped rather than queued.
        self.node = Node()
        poses = PoseFeed(self.node, world, model, links)
        self.streams = [RadarStream(self.cfgs[i], world, model, lambda f, i=i: self._on_frame(i, f), link=link,
                                    topic=f"/radar/scan_{name}", seed=seed + i, poses=poses, node=self.node,
                                    max_lag=MAX_LAG)
                        for i, ((name, _, _), link) in enumerate(zip(self.mounts, links))]

    @property
    def angles(self):
        """Scan angles of every radar, or None until each has sent a scan."""
        angles = [s.angles for s in self.streams]
        return None if any(a is None for a in angles) else angles

    @property
    def dropped(self):
        return sum(s.scans_dropped for s in self.streams)

    def _on_frame(self, i, frame):
        det = frame.det

        if len(self.wires):
            # The wires and poles within reach, in the radar frame; their detections join those of the rays.
            extra = []

            for _, p0, p1, r in self.wires:
                a, b = (p0 - frame.position) @ frame.rotation, (p1 - frame.position) @ frame.rotation
                d = b - a
                nearest = np.linalg.norm(a + np.clip(-(a @ d) / (d @ d), 0.0, 1.0) * d)

                sensor = self.profiles[i]["sensor"]

                if nearest <= sensor["range_max"]:
                    extra.append(wire_detections(a, b, r, sensor, self.cfgs[i], self.rng, frame.velocity))

            det = np.vstack([det] + extra)

        if self.log is not None:
            self.log.add(i, frame, det)

        if self.frames_to is not None:
            # As the radar sends it to the companion computer: stamp, radar, detections in its own frame.
            sock, port = self.frames_to
            sock.sendto(FRAME_HEADER.pack(frame.stamp, i, len(det)) + np.asarray(det[:, :5], "<f4").tobytes(),
                        ("127.0.0.1", port))

        if self.doppler_gate > 0.0 and len(det):
            keep = doppler_consistent(det, frame.velocity, self.cfgs[i], self.doppler_gate)
            self.doppler_rejected += int((~keep).sum())
            det = det[keep]

        _, r_mount, p_mount = self.mounts[i]
        # Into the world with this frame's own pose. The cloud merges the latest frame of every radar, up to
        # MAX_AGE old: moved by the newest frame's pose instead, the older ones landed where the vehicle had got
        # to since (1.25 m at 5 m/s), and turned by its turn.
        world = np.column_stack((det[:, :3] @ frame.rotation.T + frame.position, det[:, 3:5],
                                 np.full(len(det), float(i))))
        r_base = frame.rotation @ r_mount.T
        p_base = frame.position - r_base @ p_mount

        with self.lock:
            self.latest[i] = (frame.stamp, world)
            merged = np.vstack([w for s, w in self.latest.values() if frame.stamp - s <= MAX_AGE])

        body = merged.copy()
        body[:, :3] = (merged[:, :3] - p_base) @ r_base      # the publisher moves it back with the same pose
        self.on_cloud(frame.stamp, body, p_base, r_base)


def doppler_consistent(det, velocity, cfg, gate):
    """Which detections (N, 5, radar frame) have the radial velocity of a static point where they are placed,
    for a radar moving at velocity (radar frame, m/s): within gate standard deviations of the expected spread,
    sqrt(sigma_v^2 + (|v_perp| sigma_angle)^2): an angle error moves the expected value by v_perp times it,
    v_perp the part of the velocity square to the line of sight. A detection is dropped when its angle error,
    the part of it in the plane of the line of sight and the velocity, is beyond about gate sigma_angle; an
    error square to that plane leaves the Doppler as it is and cannot be seen.

    The radar measures the Doppler shift along the true line of sight, whatever angle it estimates; a detection
    placed at a wrong angle (sidelobe, angle ambiguity, multipath through another direction) disagrees, the
    more so the faster the vehicle flies and the further off the line of flight it lies. At hover there is
    nothing to compare: everything passes. A moving target fails too: this gate assumes a static world.
    """
    unit = det[:, :3] / np.maximum(np.linalg.norm(det[:, :3], axis=1), 1e-6)[:, None]
    expected = -unit @ np.asarray(velocity, dtype=float)
    sigma_v = getattr(cfg, "sigma_velocity", 0.0) or 0.0
    sigma_angle = np.radians(max(cfg.sigma_az, cfg.sigma_el))
    v_perp = np.sqrt(np.maximum(np.dot(velocity, velocity) - expected ** 2, 0.0))
    spread = np.sqrt(sigma_v ** 2 + (v_perp * sigma_angle) ** 2)
    return np.abs(det[:, 3] - expected) <= gate * np.maximum(spread, 0.05)


class Publisher:
    def __init__(self, fov_range, fov_segs):
        import rclpy
        from geometry_msgs.msg import Point, PointStamped, TransformStamped
        from sensor_msgs.msg import PointCloud2, PointField
        from tf2_ros import TransformBroadcaster
        from visualization_msgs.msg import Marker, MarkerArray

        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node("radar_array")
        self.cloud_pub = self.node.create_publisher(PointCloud2, "/radar/points", 10)
        self.map_pub = self.node.create_publisher(PointCloud2, "/radar/points_map", 10)
        self.fov_pub = self.node.create_publisher(MarkerArray, "/radar/fov", 1)
        self.wire_pub = self.node.create_publisher(Marker, "/radar/wires", 1)
        self.pose_pub = self.node.create_publisher(PointStamped, "/radar/pose", 10)
        self.PointStamped = PointStamped
        self.tf = TransformBroadcaster(self.node)
        self.PointCloud2, self.TransformStamped, self.Point = PointCloud2, TransformStamped, Point
        self.Marker, self.MarkerArray = Marker, MarkerArray
        self.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate(FIELDS)]
        self.fov_segs = fov_segs
        self.wires = []
        self.last_markers = 0.0

    def publish(self, stamp, det, position, rotation):
        now = self.node.get_clock().now().to_msg()      # wall clock: RViz has no /clock here

        t = self.TransformStamped()
        t.header.stamp = now
        t.header.frame_id, t.child_frame_id = "map", "base_link"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (float(v) for v in position)
        w, x, y, z = matrix_to_quat(rotation)
        t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z = w, x, y, z
        self.tf.sendTransform(t)

        in_map = det.copy()
        in_map[:, :3] = det[:, :3] @ rotation.T + position
        self.cloud_pub.publish(self._cloud(now, "base_link", det))
        self.map_pub.publish(self._cloud(now, "map", in_map))
        seen_from = self.PointStamped()
        # Simulation time of the newest radar frame: a consumer holds it against its own clock (PX4's).
        seen_from.header.stamp.sec, seen_from.header.stamp.nanosec = int(stamp), int((stamp % 1.0) * 1e9)
        seen_from.header.frame_id = "map"
        seen_from.point.x, seen_from.point.y, seen_from.point.z = (float(v) for v in position)
        self.pose_pub.publish(seen_from)

        if time.monotonic() - self.last_markers > 1.0:      # static: send now and then, for late RViz starts
            self.last_markers = time.monotonic()
            self._publish_markers(now)

    def _cloud(self, stamp, frame, det):
        cloud = self.PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = frame
        cloud.height, cloud.width = 1, len(det)
        cloud.fields = self.fields
        cloud.point_step, cloud.row_step = 4 * len(FIELDS), 4 * len(FIELDS) * len(det)
        cloud.is_dense = True
        cloud.data = det.astype("<f4").tobytes()
        return cloud

    def _publish_markers(self, now):
        fov = self.MarkerArray()

        for i, (segs, colour) in enumerate(self.fov_segs):
            m = self.Marker()
            m.header.stamp, m.header.frame_id = now, "base_link"
            m.ns, m.id, m.type, m.action = "radar_fov", i, self.Marker.LINE_LIST, self.Marker.ADD
            # Moves with base_link: RViz transforms a marker once, on arrival, unless it is frame locked.
            m.frame_locked = True
            m.pose.orientation.w = 1.0
            m.scale.x = 0.02
            m.color.r, m.color.g, m.color.b, m.color.a = (*colour, 0.8)
            m.points = [self.Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in segs.reshape(-1, 3)]
            fov.markers.append(m)

        self.fov_pub.publish(fov)

        if self.wires:
            m = self.Marker()
            m.header.stamp, m.header.frame_id = now, "map"
            m.ns, m.id, m.type, m.action = "wires", 0, self.Marker.LINE_LIST, self.Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = 0.08
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.2, 0.9
            m.points = [self.Point(x=float(q[0]), y=float(q[1]), z=float(q[2])) for _, p0, p1, _ in self.wires for q in (p0, p1)]
            self.wire_pub.publish(m)

    def spin(self):
        self.rclpy.spin(self.node)


def main():
    if os.environ.get("PY_SAMPLE"):         # debugging aid: where the time goes (scripts/sampler.py)
        import sampler
        sampler.start(os.environ["PY_SAMPLE"])

    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--world", default="power_line")
    p.add_argument("--model", default=None, help="default: x500_radar<layout>_0")
    p.add_argument("--layout", type=layout_key, choices=(3, 6, "aop"), default=3,
                   help="3: level ring (x500_radar3), 6: 3 x 3 (x500_radar6), aop: 1 ISK + 5 AOP (x500_radar_aop); "
                        "scripts/radar_layouts.py")
    p.add_argument("--wires", default=None, help="world SDF whose wire_* models are added (default: worlds/<world>.sdf)")
    p.add_argument("--no-wires", action="store_true", help="ray grid only")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--doppler-gate", type=float, default=3.0,
                   help="drop detections whose radial velocity is not that of a static point where they are placed, "
                        "by more than this many standard deviations; 0: keep all")
    sens = p.add_argument_group("sensitivity tests: the same change to every radar (docs/survey_mapping_plan.md)")
    sens.add_argument("--angle-scale", type=float, default=1.0, help="multiply the angle errors (sigma_az, sigma_el)")
    sens.add_argument("--surface-db", type=float, default=0.0, help="dB added to the SNR of surfaces (the ray grid)")
    sens.add_argument("--surface-law", type=float, default=40.0,
                      help="dB per decade of range the SNR of surfaces falls by: 40 (R^-4, the avoidance results), "
                           "20 (R^-2: a surface larger than the cell; the survey runs)")
    sens.add_argument("--thin-db", type=float, default=0.0,
                      help="dB added to the SNR of thin cylinders (wires, poles: radar_model.wire_detections)")
    sens.add_argument("--points-scale", type=float, default=1.0,
                      help="multiply the detections allowed per range bin and per frame")
    p.add_argument("--frames-udp", type=int, default=None, metavar="PORT",
                   help="send every radar frame, as the hardware gives it, to this UDP port on localhost "
                        "(scripts/onboard_live.py)")
    p.add_argument("--log", default=None, metavar="DIR",
                   help="record every radar frame, as the hardware gives it, to this directory (scripts/radar_log.py)")
    add_model_args(p)
    args = p.parse_args()
    args.model = args.model or f"{VEHICLE[args.layout]}_0"

    profile = load_profile()
    sdf = args.wires or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "worlds", args.world + ".sdf")
    # Wires and poles get the thin-cylinder model; RViz shows only the wires.
    cylinders = [] if args.no_wires or not os.path.exists(sdf) else load_wires(sdf, poles=True)
    wires = [c for c in cylinders if c[0].startswith("wire_")]
    count = {"clouds": 0, "points": 0}
    holder = {}

    def on_cloud(stamp, det, position, rotation):
        if "pub" not in holder:         # the first scans arrive before the publisher exists
            return

        count["clouds"] += 1
        count["points"] += len(det)
        holder["pub"].publish(stamp, det, position, rotation)

    sensitivity = {k: getattr(args, k) for k in ("angle_scale", "surface_db", "surface_law", "thin_db", "points_scale")}
    array = RadarArray(args, profile, args.world, args.model, cylinders, on_cloud, layout=args.layout, seed=args.seed,
                       doppler_gate=args.doppler_gate, sensitivity=sensitivity)

    if args.log:
        array.log = RadarLog(args.log, {
            "world": args.world, "model": args.model, "layout": args.layout, "seed": args.seed,
            "sensitivity": dict(sensitivity, angle_outlier=getattr(args, "angle_outlier", 0.0) or 0.0),
            "frame": "each radar's own: x along the boresight, y left, z up (m); world ENU",
            "points": ["x", "y", "z", "radial_velocity", "snr_db"],
            "stamp": "simulation time of the scan (s)",
            "truth": "position and rotation (radar to world), velocity (radar frame): from the simulation, for scoring only",
            "radars": [{"name": name, "profile": os.path.basename(profile_path(name)),
                        "mount_rotation": r.round(9).tolist(), "mount_position": p.tolist()}
                       for name, r, p in array.mounts],
        })

        def stop(signum, _frame):
            array.log.flush()
            os._exit(0)

        signal.signal(signal.SIGTERM, stop)
        signal.signal(signal.SIGINT, stop)
        print(f"radar_array: recording to {args.log}", flush=True)

    if args.frames_udp:
        array.frames_to = (socket.socket(socket.AF_INET, socket.SOCK_DGRAM), args.frames_udp)
        print(f"radar_array: frames to UDP {args.frames_udp}", flush=True)

    while array.angles is None:
        time.sleep(0.2)

    segs = []

    for prof, angles, (_, r_mount, p_mount), colour in zip(array.profiles, array.angles, array.mounts, COLOURS):
        az0, az1, el0, el1 = display_angles(prof, angles)
        s = fov_segments(float(prof.get("display", {}).get("fov_range", 10.0)), az0, az1, el0, el1)
        segs.append((s @ r_mount.T + p_mount, colour))

    holder["pub"] = Publisher(0, segs)
    holder["pub"].wires = wires
    print(f"radar_array: world {args.world}, model {args.model}, {len(wires)} wires, "
          f"{len(cylinders) - len(wires)} poles", flush=True)

    def status():
        while True:
            time.sleep(10)
            n = max(count["clouds"], 1)
            print(f"radar_array: {count['clouds']} clouds, {count['points'] / n:.0f} detections/cloud, "
                  f"{array.dropped} scans dropped as too late, {array.doppler_rejected} detections failed the "
                  f"Doppler gate" + (", recording %.0f kbit/s as sent by the radars (%.0f packed)" % array.log.rates()
                                     if array.log is not None else ""), flush=True)

    threading.Thread(target=status, daemon=True).start()

    try:
        holder["pub"].spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
    sys.stdout.flush()
    os._exit(0)
