"""survey_process.place: a detection seen from a tilted, turned vehicle lands on the world point it came from,
with the estimated pose (base_link plus each radar's mount) and with the recorded true one."""

import os
import sys
import types
import warnings

import numpy as np

warnings.filterwarnings("ignore", message="A NumPy version")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import survey_process as sp  # noqa: E402
from radar_layouts import mounts, profile_path  # noqa: E402


def _rot(roll, pitch, yaw):
    """base_link (FLU) to world (ENU): yaw about z, then pitch about y, then roll about x (degrees)."""
    r, p, y = np.radians([roll, pitch, yaw])
    rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


# Vehicle poses (position, roll, pitch, yaw) and one world point per frame.
POSES = [((10.0, 5.0, 8.0), 15.0, -10.0, 40.0), ((30.0, -4.0, 12.0), -20.0, 12.0, -110.0),
         ((22.0, 9.0, 10.0), 5.0, 25.0, 175.0)]
TARGET = np.array([28.0, 2.0, 6.0])


def _recording():
    layout = mounts("aop")
    meta = {"world": "survey_site", "radars": [{"name": n, "profile": os.path.basename(profile_path(n)),
                                                "mount_rotation": r.tolist(), "mount_position": p.tolist()}
                                               for n, r, p in layout]}
    stamp, radar, rot, pos, det = [], [], [], [], []
    base = []
    for k, (p_base, roll, pitch, yaw) in enumerate(POSES):
        r_base = _rot(roll, pitch, yaw)
        for i, (_, r_mount, p_mount) in enumerate(layout):
            r_radar = r_base @ r_mount
            p_radar = np.array(p_base) + r_base @ p_mount
            stamp.append(1.0 + 0.1 * k)
            radar.append(i)
            rot.append(r_radar)
            pos.append(p_radar)
            # The target in the radar's frame, at rest: radial velocity 0, a strong SNR.
            det.append(np.r_[r_radar.T @ (TARGET - p_radar), 0.0, 30.0])
            base.append((p_base, r_base))
    n = len(stamp)
    rec = {"meta": meta, "stamp": np.array(stamp), "radar": np.array(radar), "rotation": np.array(rot),
           "position": np.array(pos), "velocity": np.zeros((n, 3)), "points": np.array(det, np.float32),
           "first": np.arange(n), "count": np.ones(n, int)}

    def at(t):
        idx = [int(np.argmin(np.abs(np.array(stamp) - x))) for x in np.atleast_1d(t)]
        return (np.array([base[i][0] for i in idx], float), np.array([base[i][1] for i in idx]),
                np.zeros((len(idx), 3)), np.zeros((len(idx), 3)))

    ulog = types.SimpleNamespace(at=at, heading_sigma_at=lambda t: np.zeros(len(np.atleast_1d(t))))
    return rec, ulog


def test_tilted_vehicle_detections_land_on_the_target():
    rec, ulog = _recording()
    for pose in ("estimate", "truth"):
        pts = sp.place(rec, pose, ulog, quiet=True)
        assert len(pts) == len(rec["stamp"])
        # float32 detections: within a millimetre.
        assert np.abs(pts[:, :3] - TARGET).max() < 1e-3, pose


def test_ignoring_the_tilt_misplaces_them():
    rec, ulog = _recording()
    level = types.SimpleNamespace(heading_sigma_at=ulog.heading_sigma_at)

    def at(t):
        p, r, v, w = ulog.at(t)
        # Only the heading kept: what a map that does not correct for roll and pitch would use.
        yaw = np.arctan2(r[:, 1, 0], r[:, 0, 0])
        return p, np.array([_rot(0, 0, np.degrees(a)) for a in yaw]), v, w

    level.at = at
    pts = sp.place(rec, "estimate", level, quiet=True)
    assert np.median(np.linalg.norm(pts[:, :3] - TARGET, axis=1)) > 2.0
