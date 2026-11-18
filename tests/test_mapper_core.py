"""The C++ mapper core (cpp/mapper_core.cpp) against the Python it stands in for."""

import argparse
import os
import sys
import warnings

import numpy as np
import pytest

warnings.filterwarnings("ignore", message="A NumPy version")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import onboard_mapper as om  # noqa: E402

try:
    import mapper_core as mc  # noqa: E402
except ImportError:            # a module-level skip would skip the whole session under pytest 6 + ROS plugins
    mc = None
pytestmark = pytest.mark.skipif(mc is None, reason="cpp/build.sh not run")


def test_voxel_map_same_reports_and_map():
    rng = np.random.default_rng(4)
    py = om.VoxelMap(ground_z=0.2)
    cpp = mc.VoxelMap(om.VOXEL, 0.2, om.MIN_FRAMES, om.MIN_BASELINE, om.NEAR_RANGE, om.GROUND_BAND)
    for frame in range(400):
        sensor = np.array([frame * 0.1, 14.0, 6.0])
        pts = np.vstack((rng.normal([15, 0, 6], 0.7, (8, 3)), rng.uniform([0, -10, 0], [40, 10, 12], (4, 3))))
        assert py.add(frame, pts, sensor) == [tuple(k) for k in cpp.add(frame, pts, sensor)]
    a, b = py.confirmed(), cpp.confirmed()
    oa, ob = np.lexsort(a[0].T), np.lexsort(b[0].T)
    for x, y in zip(a, b):
        np.testing.assert_allclose(x[oa], y[ob])
    keys = [tuple(k) for k in a[0][:50]]
    for x, y in zip(py.records(keys), cpp.records(keys)):
        np.testing.assert_allclose(x, y)
    assert len(py) == len(cpp)


def test_place_frame_matches_the_gate_and_transform():
    from radar_array import doppler_consistent

    rng = np.random.default_rng(5)
    det = np.column_stack((rng.uniform(1, 20, (300, 3)), rng.normal(0, 2, 300), rng.uniform(10, 40, 300)))
    v = np.array([4.0, 0.5, -0.2])
    det[:, 3] = -(det[:, :3] / np.linalg.norm(det[:, :3], axis=1, keepdims=True)) @ v + rng.normal(0, 0.3, 300)
    cfg = argparse.Namespace(sigma_velocity=0.05, sigma_az=3.0, sigma_el=5.0)
    c, s = np.cos(0.3), np.sin(0.3)
    rot, pos = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]]), np.array([10.0, -2.0, 6.0])
    keep = doppler_consistent(det, v, cfg, 3.0)
    out, rejected = mc.place_frame(det, rot, pos, v, 0.05, np.radians(5.0), 3.0)
    assert rejected == int((~keep).sum()) and 0 < rejected < 300
    np.testing.assert_allclose(out[:, :3], det[keep, :3] @ rot.T + pos)
    np.testing.assert_allclose(out[:, 3:], det[keep, 3:])


def test_pose_track_matches_causal_pose():
    import types

    t = np.arange(0.0, 20.0, 0.008)
    ang = 0.3 * np.sin(0.5 * t)                      # yawing
    q = np.column_stack((np.cos(ang / 2), 0 * t, 0 * t, np.sin(ang / 2)))
    p = np.column_stack((2 * t, np.sin(t), -5 - 0.1 * t))
    v = np.gradient(p, t, axis=0)
    w = np.column_stack((0 * t, 0 * t, np.gradient(ang, t)))
    e = types.SimpleNamespace(tp=t, p_ned=p, v_ned=v, tq=t, q=q, tw=t, w_frd=w, ts=t,
                              heading_sigma=np.linspace(5, 0.3, len(t)), origin=np.array([1.0, -2.0, 0.5]))

    import survey_process as sp
    e.at = lambda tt: sp.EstimatedPose.at(e, tt)
    py = om.CausalPose(e)
    py.cpp = None
    cpp = om.CausalPose(e)
    assert cpp.cpp is not None
    for s in np.linspace(1.0, 19.0, 97):
        for now in (s + 0.05, s - 0.2):             # interpolated, extrapolated
            for a, b in zip(py.at(s, now), cpp.at(s, now)):
                np.testing.assert_allclose(a, b, atol=1e-9)
