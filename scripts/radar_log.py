"""Recording of the radars' output as the hardware gives it, for mapping after the flight
(docs/survey_mapping_plan.md, phase 1).

Every frame of every radar: its time stamp and its detections in that radar's own frame (x along the
boresight, y left, z up; m), with radial velocity (m/s) and SNR (dB) - what the TI out-of-box firmware
sends over its UART - before any processing of ours (the Doppler gate of radar_array is processing).
Alongside, for scoring only, the radar's true pose and velocity from the simulation: the mapping itself
must place the detections with PX4's estimated pose from the PX4 log.

A recording is a directory:

  meta.yaml           world, model, layout, the radars' names and mounts on base_link (FLU), profiles,
                      and what the arrays below hold
  chunk_NNNN.npz      one per CHUNK seconds of simulation time:
                        stamp (F,)      float64  simulation time of the scan (s)
                        radar (F,)      int16    index into meta.yaml's radars
                        first, count (F,) int32  the frame's rows of points
                        position (F, 3) float64  the radar's true position in the world (ENU)
                        rotation (F, 9) float32  its true rotation, radar to world, row-major
                        velocity (F, 3) float32  its true velocity, in the radar frame
                        points (N, 5)   float32  x, y, z, radial velocity, SNR

Chunks are written as they fill and on stop (SIGTERM, SIGINT), so an interrupted flight keeps all but
at most its last CHUNK seconds. load() reads a recording back into one set of arrays.
"""

import glob
import os
import threading

import numpy as np
import yaml

CHUNK = 5.0                 # s of simulation time per file
POINT_BYTES_TI = 20         # TI out-of-box UART: x, y, z, v as float32 + SNR, noise as int16
FRAME_BYTES_TI = 40 + 2 * 8  # its frame header and two TLV headers (points, side info)
POINT_BYTES_PACKED = 8      # x, y, z as int16 cm, v as int8 (5 cm/s), SNR as uint8


class RadarLog:
    def __init__(self, directory, meta):
        self.directory = directory
        os.makedirs(directory, exist_ok=True)

        with open(os.path.join(directory, "meta.yaml"), "w") as f:
            yaml.safe_dump(meta, f, sort_keys=False)

        self.lock = threading.Lock()
        self.chunk = 0
        self.start = None
        self._reset()
        # For the data rate: totals since the start.
        self.frames = 0
        self.points = 0
        self.first_stamp = None
        self.last_stamp = None

    def _reset(self):
        self.rows = {k: [] for k in ("stamp", "radar", "first", "count", "position", "rotation", "velocity")}
        self.pts = []
        self.n = 0

    def add(self, radar, frame, det):
        det = np.asarray(det[:, :5], dtype=np.float32)

        with self.lock:
            if self.start is None:
                self.start = frame.stamp
                self.first_stamp = frame.stamp

            r = self.rows
            r["stamp"].append(frame.stamp)
            r["radar"].append(radar)
            r["first"].append(self.n)
            r["count"].append(len(det))
            r["position"].append(np.asarray(frame.position, dtype=np.float64))
            r["rotation"].append(np.asarray(frame.rotation, dtype=np.float32).reshape(9))
            r["velocity"].append(np.asarray(frame.velocity, dtype=np.float32))
            self.pts.append(det)
            self.n += len(det)
            self.frames += 1
            self.points += len(det)
            self.last_stamp = frame.stamp

            if frame.stamp - self.start >= CHUNK:
                self._write()
                self.start = frame.stamp

    def _write(self):
        if not self.rows["stamp"]:
            return

        r = self.rows
        path = os.path.join(self.directory, f"chunk_{self.chunk:04d}.npz")
        tmp = path + ".tmp.npz"
        np.savez(tmp, stamp=np.array(r["stamp"], dtype=np.float64), radar=np.array(r["radar"], dtype=np.int16),
                 first=np.array(r["first"], dtype=np.int32), count=np.array(r["count"], dtype=np.int32),
                 position=np.array(r["position"]), rotation=np.array(r["rotation"]),
                 velocity=np.array(r["velocity"]),
                 points=np.vstack(self.pts) if self.pts else np.empty((0, 5), np.float32))
        os.replace(tmp, path)      # a reader never sees half a chunk
        self.chunk += 1
        self._reset()

    def flush(self):
        with self.lock:
            self._write()

    def rates(self):
        """Data rate of the recording so far (kbit/s): as the TI firmware sends it, and packed."""
        with self.lock:
            span = (self.last_stamp or 0.0) - (self.first_stamp or 0.0)

            if span <= 0.0:
                return 0.0, 0.0

            ti = (self.points * POINT_BYTES_TI + self.frames * FRAME_BYTES_TI) * 8 / span / 1e3
            packed = (self.points * POINT_BYTES_PACKED + self.frames * 8) * 8 / span / 1e3
            return ti, packed


def load(directory):
    """A recording as one dict of arrays (the chunk arrays concatenated, first re-based), plus meta."""
    with open(os.path.join(directory, "meta.yaml")) as f:
        meta = yaml.safe_load(f)

    parts = [np.load(p) for p in sorted(glob.glob(os.path.join(directory, "chunk_[0-9][0-9][0-9][0-9].npz")))]

    if not parts:
        raise ValueError(f"{directory}: no chunks")

    out = {k: np.concatenate([p[k] for p in parts]) for k in parts[0].files if k not in ("first", "points")}
    offset = np.cumsum([0] + [len(p["points"]) for p in parts[:-1]])
    out["first"] = np.concatenate([p["first"] + o for p, o in zip(parts, offset)])
    out["points"] = np.concatenate([p["points"] for p in parts])
    order = np.argsort(out["stamp"], kind="stable")
    out = {k: (v if k == "points" else v[order]) for k, v in out.items()}
    out["meta"] = meta
    return out


def frame_points(rec, k):
    """The detections (n, 5) of frame k of a loaded recording."""
    return rec["points"][rec["first"][k]:rec["first"][k] + rec["count"][k]]
