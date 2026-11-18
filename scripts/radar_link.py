#!/usr/bin/env python3
"""The link from the vehicle to the ground: the on-board map's changes within a data rate
(docs/survey_mapping_plan.md, phase 5).

Datagrams (UDP: every one counts HEADER_UDP more on the link), each a HEADER of
type (u8), count (u16), stamp (u32 ms), then count records:

  POSE    20 B  x, y, z (f32, world ENU m), yaw (f32 rad), speed (f32): where the vehicle is, 10 Hz
  VOXEL    9 B  voxel index x, y, z (i16, of onboard_mapper.VOXEL), hits (u8, saturating), weight (u8: the
                range-weighted hits x WEIGHT_SCALE, saturating), flags (u8: 1 ground)
  POINTS   8 B  x, y, z (i16 cm, world), radial velocity (i8, 5 cm/s), SNR (u8, 0.5 dB): the raw detections
  ORIGIN  32 B  EKF2's origin: lat, lon (f64 deg), alt (f32 m), and where it lies in the world frame (e, n, u
                f32 m), every ORIGIN_PERIOD: what the ground needs to give the map in PX4's local NED frame
                (scripts/frames.py)

Sender: a token bucket at (1 - margin) of the link rate, at most BURST seconds of it saved up, so no
second carries more than the rate. Each radar frame: the origin (if due), the pose (if due), then every voxel the map reports
(queued, never dropped: what the ground map is made of; strict priority), then, once no voxel waits, as
many of the frame's detections as the bucket still allows, strongest first (the rest dropped: the full data stays in the on-board log).

    scripts/radar_link.py out/survey/rtk_3                  # replay a flight: the link's data rate
    scripts/radar_link.py out/survey/sens_bright --kbps 300

Writes <tag>.link.npz (every datagram: time sent, bytes) for scripts/ground_map.py.
"""

import argparse
import os
import struct
import sys
import warnings

import numpy as np

warnings.filterwarnings("ignore", message="A NumPy version")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import onboard_mapper as om  # noqa: E402

POSE, VOXEL, POINTS, ORIGIN = 1, 2, 3, 4
HEADER = struct.Struct("<BHI")              # 7 B
HEADER_UDP = 28                             # IPv4 + UDP
MTU = 1400                                  # payload bytes per datagram
POSE_REC = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("yaw", "<f4"), ("speed", "<f4")])
VOXEL_REC = np.dtype([("k", "<i2", 3), ("hits", "u1"), ("weight", "u1"), ("flags", "u1")])
WEIGHT_SCALE = 4.0          # VOXEL weight: onboard_mapper's range weight times this, saturating
POINT_REC = np.dtype([("p", "<i2", 3), ("v", "i1"), ("snr", "u1")])
ORIGIN_REC = np.dtype([("lat", "<f8"), ("lon", "<f8"), ("alt", "<f4"), ("enu", "<f4", 3)])
ORIGIN_PERIOD = 1.0                         # s
POSE_PERIOD = 0.1                           # s
BURST = 0.1                                 # s of the rate the bucket may save up


# --- records ---------------------------------------------------------------------------------------------

def datagrams(kind, stamp, records):
    """records (structured array) split into datagrams of at most MTU bytes."""
    per = max((MTU - HEADER.size) // records.dtype.itemsize, 1)
    ms = int(round(stamp * 1e3)) & 0xFFFFFFFF
    return [HEADER.pack(kind, len(chunk), ms) + chunk.tobytes() for chunk in
            (records[i:i + per] for i in range(0, len(records), per))]


def decode(data):
    """(type, stamp s, records) of one datagram."""
    kind, count, ms = HEADER.unpack_from(data)
    dtype = {POSE: POSE_REC, VOXEL: VOXEL_REC, POINTS: POINT_REC, ORIGIN: ORIGIN_REC}[kind]
    return kind, ms * 1e-3, np.frombuffer(data, dtype, count, HEADER.size)


def pose_record(p, rot, v):
    r = np.zeros(1, POSE_REC)
    r["x"], r["y"], r["z"] = p
    r["yaw"] = np.arctan2(rot[1, 0], rot[0, 0])
    r["speed"] = np.linalg.norm(v)
    return r


def origin_record(origin_enu, geo):
    r = np.zeros(1, ORIGIN_REC)
    if geo is not None:
        r["lat"], r["lon"], r["alt"] = geo
    else:
        r["lat"] = r["lon"] = np.nan
    r["enu"] = origin_enu
    return r


def voxel_records(vmap, keys):
    """The records of these voxels of an onboard_mapper map (Python VoxelMap or the C++ one)."""
    r = np.zeros(len(keys), VOXEL_REC)
    hits, weight, mean_z = vmap.records(keys)
    r["k"] = np.array(keys, dtype=np.int64).reshape(-1, 3)
    r["hits"] = np.minimum(hits, 255)
    r["weight"] = np.minimum(np.round(weight * WEIGHT_SCALE), 255)
    r["flags"] = (mean_z < vmap.ground_z + om.GROUND_BAND).astype(np.uint8)
    return r


def point_records(points):
    r = np.zeros(len(points), POINT_REC)
    r["p"] = np.clip(np.round(points[:, :3] * 100), -32768, 32767)
    r["v"] = np.clip(np.round(points[:, 3] / 0.05), -128, 127)
    r["snr"] = np.clip(np.round(points[:, 4] * 2), 0, 255)
    return r


def points_from(records):
    return np.column_stack((records["p"] / 100.0, records["v"] * 0.05, records["snr"] / 2.0))


# --- the link --------------------------------------------------------------------------------------------

class TokenBucket:
    def __init__(self, bits_per_s, burst=BURST):
        self.rate = bits_per_s / 8.0            # bytes per second
        self.capacity = self.rate * burst
        self.tokens = self.capacity
        self.t = None

    def refill(self, now):
        if self.t is not None:
            self.tokens = min(self.capacity, self.tokens + (now - self.t) * self.rate)
        self.t = now

    def take(self, nbytes):
        if nbytes > self.tokens:
            return False
        self.tokens -= nbytes
        return True


class Sender:
    """What goes on the link after each radar frame; transport(now, datagram) sends one."""

    def __init__(self, kbps=300.0, margin=0.1, transport=None):
        self.bucket = TokenBucket(kbps * 1e3 * (1.0 - margin))
        self.transport = transport or (lambda now, d: None)
        self.queue = []             # (time reported, voxel key)
        self.next_pose = 0.0
        self.next_origin = 0.0
        self.sent = {POSE: 0, VOXEL: 0, POINTS: 0, ORIGIN: 0}
        self.points_in = 0
        self.voxel_wait = []        # s from reported to sent

    def _send(self, now, data):
        if not self.bucket.take(len(data) + HEADER_UDP):
            return False
        self.transport(now, data)
        return True

    def on_frame(self, now, stamp, points, report, mapper):
        self.bucket.refill(now)

        origin = getattr(mapper.pose, "origin", None)
        if origin is not None and now >= self.next_origin:
            if self._send(now, datagrams(ORIGIN, stamp, origin_record(origin, getattr(mapper.pose, "geo", None)))[0]):
                self.sent[ORIGIN] += 1
                self.next_origin = now + ORIGIN_PERIOD

        if now >= self.next_pose:
            p, rot, v, _, _ = mapper.pose.at(stamp, now)
            if self._send(now, datagrams(POSE, stamp, pose_record(p, rot, v))[0]):
                self.sent[POSE] += 1
                self.next_pose = now + POSE_PERIOD

        self.queue += [(now, k) for k in report]

        while self.queue:
            # As many as the bucket allows now (a full datagram needs 1.4 kB saved up, which the points
            # would never leave: at 50 kbit/s the voxels starved).
            room = int((self.bucket.tokens - HEADER_UDP - HEADER.size) // VOXEL_REC.itemsize)
            batch = self.queue[:min(room, (MTU - HEADER.size) // VOXEL_REC.itemsize)]
            if not batch:
                break
            data = datagrams(VOXEL, stamp, voxel_records(mapper.map, [k for _, k in batch]))[0]
            if not self._send(now, data):
                break
            self.queue = self.queue[len(batch):]
            self.sent[VOXEL] += len(batch)
            self.voxel_wait += [now - t for t, _ in batch]

        if len(points):
            self.points_in += len(points)

        # The detections only once every reported voxel is out, as many as the bucket allows, strongest first.
        if len(points) and not self.queue:
            room = int((self.bucket.tokens - HEADER_UDP - HEADER.size) // POINT_REC.itemsize)
            if room > 0:
                keep = points[np.argsort(-points[:, 4])[:room]]
                for data in datagrams(POINTS, stamp, point_records(keep)):
                    if not self._send(now, data):
                        break
                    self.sent[POINTS] += (len(data) - HEADER.size) // POINT_REC.itemsize


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("tag", help="out/survey/<tag> (without extension)")
    p.add_argument("--kbps", type=float, default=300.0, help="link rate (kbit/s)")
    p.add_argument("--margin", type=float, default=0.1, help="share of the rate kept free")
    args = p.parse_args()

    log_t, log_d = [], []
    sender = Sender(args.kbps, args.margin, lambda now, d: (log_t.append(now), log_d.append(d)))
    mapper, rec = om.replay(args.tag, 0.0, sender.on_frame, quiet=True)

    t = np.array(log_t)
    size = np.array([len(d) + HEADER_UDP for d in log_d])
    sec = np.floor(t - t[0]).astype(int)
    kbps = np.bincount(sec, weights=size) * 8 / 1e3
    kinds = np.array([d[0] for d in log_d])
    share = {k: 100 * size[kinds == k].sum() / size.sum() for k in (POSE, VOXEL, POINTS, ORIGIN)}
    wait = np.array(sender.voxel_wait) if sender.voxel_wait else np.zeros(1)
    flying = kbps[kbps > 0]
    print(f"{os.path.basename(args.tag)}: {len(t)} datagrams over {t[-1] - t[0]:.0f} s; link {args.kbps:g} kbit/s "
          f"(bucket at {args.kbps * (1 - args.margin):g}): per second median {np.median(flying):.0f}, "
          f"95 % {np.percentile(flying, 95):.0f}, max {kbps.max():.0f} kbit/s incl. UDP/IP headers")
    print(f"  of it: pose {share[POSE]:.0f} %, voxels {share[VOXEL]:.0f} %, points {share[POINTS]:.0f} %, "
          f"origin {share[ORIGIN]:.1f} %")
    print(f"  voxels: {sender.sent[VOXEL]} reported and sent, {len(sender.queue)} still queued at the end; "
          f"wait median {1e3 * np.median(wait):.0f} ms, max {1e3 * wait.max():.0f} ms")
    print(f"  points: {sender.sent[POINTS]} of {sender.points_in} sent ({100 * sender.sent[POINTS] / max(sender.points_in, 1):.0f} %)")

    np.savez(args.tag + ".link.npz", t=t, data=np.array(log_d, dtype=object),
             kbps=args.kbps, margin=args.margin)
    print(f"wrote {args.tag}.link.npz")
    return 0


if __name__ == "__main__":
    sys.exit(main())
