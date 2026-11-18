"""onboard_mapper: the voxel map's ghost filter as data comes in, and the pose known only from the past."""

import os
import sys
import types
import warnings

import numpy as np

warnings.filterwarnings("ignore", message="A NumPy version")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import onboard_mapper as om  # noqa: E402


def test_confirmed_after_frames_from_apart():
    m = om.VoxelMap(ground_z=0.0)
    p = np.array([[10.2, 3.1, 10.1]])
    # Three frames from the same spot: not yet (no baseline).
    assert all(m.add(f, p, np.array([0.0, 14.0, 6.0])) == [] for f in range(3))
    # From 2 m along: confirmed, reported once.
    report = m.add(3, p, np.array([2.0, 14.0, 6.0]))
    assert report == [tuple(np.floor(p[0] / om.VOXEL).astype(int))]
    assert m.add(4, p, np.array([2.5, 14.0, 6.0])) == []          # 5 hits: not yet twice the 4 reported
    keys, cen, hits, ground = m.confirmed()
    assert len(keys) == 1 and hits[0] == 5 and not ground[0]
    np.testing.assert_allclose(cen[0], p[0])


def test_single_frame_is_never_confirmed():
    m = om.VoxelMap()
    pts = np.random.default_rng(0).uniform(0, 50, (200, 3))
    assert m.add(0, pts, np.zeros(3)) == []
    assert len(m.confirmed()[0]) == 0


def test_causal_pose_extrapolates_past_the_newest_sample():
    t = np.arange(0.0, 1.0, 0.01)
    e = types.SimpleNamespace(tp=t, tq=t, tw=t, ts=t, heading_sigma=np.zeros_like(t))
    # Moving at 2 m/s along x, level, not turning.
    e.at = lambda tt: (np.column_stack((2.0 * tt, 0 * tt, 0 * tt)), np.repeat(np.eye(3)[None], len(tt), 0),
                       np.tile([2.0, 0, 0], (len(tt), 1)), np.zeros((len(tt), 3)))
    pose = om.CausalPose(e, pose_delay=0.02)
    # Samples up to 0.48 s have arrived at 0.5 s: a scan at 0.45 s is interpolated, one at 0.6 s extrapolated.
    assert np.isclose(pose.at(0.45, now=0.5)[0][0], 0.9)
    assert np.isclose(pose.at(0.6, now=0.5)[0][0], 1.2)
