"""Doppler gate, pose at scan time and the world-frame merge of radar_array: no Gazebo needed.

    cd examples/px4-mapping && python3 -m pytest tests
"""

import argparse
import collections
import os
import sys
import threading

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import radar_array as ra  # noqa: E402
import radar_model as rm  # noqa: E402
import radar_stream as rs  # noqa: E402

AOP = os.path.join(os.path.dirname(__file__), "..", "profiles", "iwr6843aop.yaml")


def config(**overrides):
    p = argparse.ArgumentParser()
    rm.add_model_args(p, AOP)
    cfg = p.parse_args([])

    for k, v in overrides.items():
        setattr(cfg, k, v)

    return cfg


def targets(n=4000, seed=0):
    """Points 2-8 m away across the field of view, as (r, az, el) in rad."""
    rng = np.random.default_rng(seed)
    return rng.uniform(2.0, 8.0, n), np.radians(rng.uniform(-37, 37, n)), np.radians(rng.uniform(-50, 50, n))


def test_doppler_is_that_of_the_true_direction():
    cfg = config(sigma_velocity=0.0, sigma_az=10.0, sigma_el=10.0)
    r, a, e = targets()
    det = rm.measure(r, a, e, np.zeros(len(r)), (5.0, 0.0, 0.0), cfg, np.random.default_rng(1), 10.0, 10.0)
    assert np.allclose(det[:, 3], -5.0 * np.cos(e) * np.cos(a))


def _error_along_velocity(det, a, e, v):
    """Angle error of each detection (deg) in the plane of its true line of sight and the velocity."""
    u = det[:, :3] / np.linalg.norm(det[:, :3], axis=1)[:, None]
    t = rm.unit_vectors(a, e)
    v = np.asarray(v, dtype=float) / np.linalg.norm(v)
    across = v - (t @ v)[:, None] * t
    across /= np.maximum(np.linalg.norm(across, axis=1), 1e-9)[:, None]
    return np.degrees(np.abs(((u - t) * across).sum(axis=1)))


@pytest.mark.parametrize("v", [(5.0, 0.0, 0.0), (0.0, 5.0, 0.0)])      # along the boresight, across it
def test_the_gate_keeps_good_detections_and_drops_misplaced_ones(v):
    cfg = config(angle_outlier=0.0)
    r, a, e = targets()
    good = rm.measure(r, a, e, np.zeros(len(r)), v, cfg, np.random.default_rng(2), cfg.sigma_az, cfg.sigma_el)
    assert ra.doppler_consistent(good, v, cfg, 3.0).mean() > 0.99

    bad = rm.measure(r, a, e, np.zeros(len(r)), v, config(angle_outlier=1.0), np.random.default_rng(3),
                     cfg.sigma_az, cfg.sigma_el)
    far = _error_along_velocity(bad, a, e, v) > 15.0
    assert far.sum() > 100 and (~ra.doppler_consistent(bad[far], v, cfg, 3.0)).mean() > 0.8


def test_at_hover_the_gate_has_nothing_to_go_on():
    cfg = config(angle_outlier=1.0)
    r, a, e = targets()
    det = rm.measure(r, a, e, np.zeros(len(r)), (0.0, 0.0, 0.0), cfg, np.random.default_rng(4), 3.0, 4.0)
    assert ra.doppler_consistent(det, (0.0, 0.0, 0.0), cfg, 3.0).mean() > 0.99      # Doppler noise only


def _feed(samples):
    feed = rs.PoseFeed.__new__(rs.PoseFeed)
    feed.lock = threading.Lock()
    feed.history = {"radar": collections.deque(samples)}
    return feed


def test_the_pose_is_taken_at_the_scan_time():
    eye = np.eye(3)
    feed = _feed([(t, np.array([5.0 * t, 0.0, 10.0]), eye, np.array([5.0, 0.0, 0.0])) for t in np.arange(0, 1, 0.01)])
    stamp, position, _, velocity = feed.at("radar", 0.505)
    assert position == pytest.approx([2.525, 0.0, 10.0]) and velocity == pytest.approx([5.0, 0.0, 0.0])
    assert feed.at("radar", 5.0)[1] == pytest.approx([4.95, 0.0, 10.0])     # past the history: the newest
    assert feed.at("radar", -1.0) is None


def test_each_radar_frame_is_placed_with_its_own_pose():
    # A pole at (10, 0, 10). Radar 0 sees it at t = 0.0 from x = 0, radar 1 at t = 0.2 from x = 1 after a
    # 30 deg turn; the merged cloud must put both detections on the pole.
    array = ra.RadarArray.__new__(ra.RadarArray)
    array.mounts = [("a", np.eye(3), np.zeros(3)), ("b", np.eye(3), np.zeros(3))]
    array.wires, array.latest, array.lock = [], {}, threading.Lock()
    array.doppler_gate, array.doppler_rejected = 0.0, 0
    out = []
    array.on_cloud = lambda stamp, det, position, rotation: out.append(det[:, :3] @ rotation.T + position)
    pole = np.array([10.0, 0.0, 10.0])
    c, s = np.cos(np.radians(30)), np.sin(np.radians(30))
    turned = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    for i, (t, position, rotation) in enumerate(((0.0, np.array([0.0, 0.0, 10.0]), np.eye(3)),
                                                 (0.2, np.array([1.0, 0.0, 10.0]), turned))):
        local = (pole - position) @ rotation
        frame = rs.Frame(t, np.array([[*local, 0.0, 20.0]]), position, rotation, np.zeros(3))
        array._on_frame(i, frame)

    assert np.allclose(out[-1], pole)
