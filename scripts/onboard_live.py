#!/usr/bin/env python3
"""The on-board mapper live, against the simulation (docs/survey_mapping_plan.md, phase 5).

As on the companion computer: the radar frames come in from scripts/radar_array.py --frames-udp (as the
radars send them), PX4's estimated pose over MAVLink (its 14550 link: LOCAL_POSITION_NED, ATTITUDE_QUATERNION,
GPS_GLOBAL_ORIGIN), and the map's changes go out over the 300 kbit/s link (scripts/radar_link.py) to
scripts/ground_map.py --udp.

Differences from the replay of scripts/onboard_mapper.py: the pose is only what has arrived; MAVLink carries no
heading variance, so the heading counts as known once the vehicle has flown at least ALIGN_SPEED for ALIGN_TIME
(EKF2 aligns its heading on the first horizontal acceleration: survey_process.MAX_HEADING_SIGMA).

    scripts/onboard_live.py --frames 14610 --to 14600 --kbps 300
"""

import argparse
import os
import socket
import sys
import threading
import time
import warnings

import numpy as np

warnings.filterwarnings("ignore", message="A NumPy version")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import onboard_mapper as om  # noqa: E402
import radar_link as rl  # noqa: E402
import survey_process as sp  # noqa: E402
from radar_layouts import mounts, profile_path  # noqa: E402

FRAME_HEADER_SIZE = 11      # radar_array.FRAME_HEADER: <dBH
ALIGN_SPEED = 2.0           # m/s
ALIGN_TIME = 2.0            # s
HISTORY = 20.0              # s of pose samples kept
RECENT = 100                # samples looked at per frame (1 s at 100 Hz)


class LivePose:
    """PX4's pose from MAVLink, in the world (ENU, FLU), as it arrives."""

    def __init__(self, link, world):
        from pymavlink import mavutil

        self.mav = mavutil.mavlink_connection(link)
        self.mavutil = mavutil
        self.lock = threading.Lock()
        self.tp, self.p, self.v = [], [], []
        self.tq, self.q, self.w = [], [], []
        self.lat0, self.lon0, self.elev0 = sp.world_origin(world)
        self.origin = None          # EKF2's origin in the world (ENU m) ...
        self.geo = None             # ... and geodetic (lat, lon, alt)
        self.aligned_at = None
        self._fast_since = None
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        m = self.mav
        m.wait_heartbeat()
        ml = self.mavutil.mavlink
        for msg_id, hz in ((ml.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50), (ml.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 100)):
            m.mav.command_long_send(m.target_system, m.target_component, ml.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                    msg_id, 1e6 / hz, 0, 0, 0, 0, 0)
        asked = 0.0

        while True:
            if self.origin is None and time.monotonic() - asked > 2.0:
                asked = time.monotonic()
                m.mav.command_long_send(m.target_system, m.target_component, ml.MAV_CMD_REQUEST_MESSAGE, 0,
                                        ml.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, 0, 0, 0, 0, 0, 0)
            msg = m.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE_QUATERNION", "GPS_GLOBAL_ORIGIN"],
                               blocking=True, timeout=1.0)
            if msg is None:
                continue
            kind = msg.get_type()
            with self.lock:
                if kind == "GPS_GLOBAL_ORIGIN":
                    lat, lon, alt = msg.latitude * 1e-7, msg.longitude * 1e-7, msg.altitude * 1e-3
                    self.geo = (lat, lon, alt)
                    self.origin = np.array([np.radians(lon - self.lon0) * np.cos(np.radians(self.lat0)) * sp.EARTH_RADIUS,
                                            np.radians(lat - self.lat0) * sp.EARTH_RADIUS, alt - self.elev0])
                elif kind == "LOCAL_POSITION_NED":
                    t = msg.time_boot_ms * 1e-3
                    self.tp.append(t)
                    self.p.append(sp.NED_TO_ENU @ [msg.x, msg.y, msg.z])
                    self.v.append(sp.NED_TO_ENU @ [msg.vx, msg.vy, msg.vz])
                    fast = np.hypot(msg.vx, msg.vy) >= ALIGN_SPEED
                    self._fast_since = (self._fast_since or t) if fast else None
                    if self.aligned_at is None and fast and t - self._fast_since >= ALIGN_TIME:
                        self.aligned_at = t
                else:
                    self.tq.append(msg.time_boot_ms * 1e-3)
                    self.q.append([msg.q1, msg.q2, msg.q3, msg.q4])
                    self.w.append(sp.FLU_TO_FRD @ [msg.rollspeed, msg.pitchspeed, msg.yawspeed])
                for lst in ((self.tp, self.p, self.v), (self.tq, self.q, self.w)):
                    if len(lst[0]) > 4000 and lst[0][-1] - lst[0][0] > HISTORY:
                        for x in lst:
                            del x[:1000]

    def ready(self):
        with self.lock:
            return self.origin is not None and len(self.tp) > 2 and len(self.tq) > 2

    def at(self, t, now=None):
        """(position, rotation, velocity, rate, heading sigma) at scan time t from what has arrived."""
        with self.lock:
            # The newest second is all the interpolation needs: copying all HISTORY s every frame cost half a core.
            tp, p, v = np.array(self.tp[-RECENT:]), np.array(self.p[-RECENT:]), np.array(self.v[-RECENT:])
            tq, q, w = np.array(self.tq[-RECENT:]), np.array(self.q[-RECENT:]), np.array(self.w[-RECENT:])
            aligned = self.aligned_at is not None and t >= self.aligned_at
            origin = self.origin
        tc = min(t, tp[-1], tq[-1])
        pos = np.array([np.interp(tc, tp, p[:, k]) for k in range(3)])
        vel = np.array([np.interp(tc, tp, v[:, k]) for k in range(3)])
        pos = pos + vel * (t - tc) + origin
        r_frd_ned = sp.quat_to_matrix(sp.interp_quat(np.array([tc]), tq, q))[0]
        rot = sp.NED_TO_ENU @ r_frd_ned @ sp.FLU_TO_FRD
        rate = np.array([np.interp(tc, tq, w[:, k]) for k in range(3)])
        return pos, rot, vel, rate, 0.0 if aligned else 99.0


def meta_for(layout="aop"):
    return {"radars": [{"name": n, "profile": os.path.basename(profile_path(n)), "mount_rotation": r.tolist(),
                        "mount_position": p.tolist()} for n, r, p in mounts(layout)]}


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--frames", type=int, default=14610, help="UDP port of radar_array --frames-udp")
    p.add_argument("--to", type=int, default=14600, help="UDP port of ground_map --udp")
    p.add_argument("--mavlink", default="udpin:0.0.0.0:14550")
    p.add_argument("--world", default="survey_site")
    p.add_argument("--layout", default="aop")
    p.add_argument("--kbps", type=float, default=300.0)
    args = p.parse_args()

    pose = LivePose(args.mavlink, args.world)
    print("waiting for PX4's pose and origin", flush=True)
    while not pose.ready():
        time.sleep(0.2)

    mapper = om.OnboardMapper(meta_for(args.layout), pose, home_z=pose.origin[2])
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sent = []
    sender = rl.Sender(args.kbps, transport=lambda now, d: (out.sendto(d, ("127.0.0.1", args.to)),
                                                              sent.append((time.monotonic(), len(d) + rl.HEADER_UDP))))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", args.frames))
    sock.settimeout(1.0)
    print(f"mapping: frames on UDP {args.frames}, map to UDP {args.to} at {args.kbps:g} kbit/s", flush=True)
    last_status, idle = time.monotonic(), 0

    try:
        while True:
            try:
                data = sock.recv(65535)
                idle = 0
            except socket.timeout:
                idle += 1
                if mapper.frames and idle > 20:
                    break
                continue
            stamp = np.frombuffer(data, "<f8", 1)[0]
            radar = data[8]
            n = int.from_bytes(data[9:11], "little")
            det = np.frombuffer(data, "<f4", n * 5, FRAME_HEADER_SIZE).reshape(n, 5).astype(float)
            with pose.lock:
                now = max(pose.tp[-1], pose.tq[-1])
            points, report = mapper.process(float(stamp), radar, det, now)
            sender.on_frame(now, float(stamp), points, report, mapper)

            if time.monotonic() - last_status > 10.0:
                last_status = time.monotonic()
                recent = [b for t, b in sent if t > last_status - 10.0]
                keys = mapper.map.confirmed()[0]
                print(f"onboard: {mapper.frames} frames ({mapper.skipped['low']} low, {mapper.skipped['heading']} before "
                      f"the heading aligned), {len(keys)} confirmed voxels, link {sum(recent) * 8 / 10e3:.0f} kbit/s "
                      f"over the last 10 s, {sender.sent[rl.POINTS]} of {sender.points_in} detections sent", flush=True)
    except KeyboardInterrupt:
        pass

    t = np.array([s[0] for s in sent])
    size = np.array([s[1] for s in sent])
    if len(t):
        kb = np.bincount(np.floor(t - t[0]).astype(int), weights=size) * 8 / 1e3
        kb = kb[kb > 0]
        print(f"done: {mapper.frames} frames, link per second median {np.median(kb):.0f}, 95 % {np.percentile(kb, 95):.0f}, "
              f"max {kb.max():.0f} kbit/s; {sender.sent[rl.VOXEL]} voxels, {sender.sent[rl.POINTS]} of "
              f"{sender.points_in} detections sent", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
