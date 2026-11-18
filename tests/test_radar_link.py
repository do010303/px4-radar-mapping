"""radar_link: records survive the wire, the bucket holds the rate, the voxels go before the points."""

import os
import sys
import types
import warnings

import numpy as np

warnings.filterwarnings("ignore", message="A NumPy version")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import radar_link as rl  # noqa: E402


def test_points_round_trip_within_quantisation():
    rng = np.random.default_rng(0)
    pts = np.column_stack((rng.uniform(-100, 100, (500, 3)), rng.uniform(-5, 5, 500), rng.uniform(5, 60, 500)))
    out = []
    for d in rl.datagrams(rl.POINTS, 12.345, rl.point_records(pts)):
        assert len(d) <= rl.MTU
        kind, stamp, rec = rl.decode(d)
        assert kind == rl.POINTS and abs(stamp - 12.345) < 1e-3
        out.append(rl.points_from(rec))
    out = np.vstack(out)
    assert np.abs(out[:, :3] - pts[:, :3]).max() <= 0.005 + 1e-9
    assert np.abs(out[:, 3] - pts[:, 3]).max() <= 0.025 + 1e-9
    assert np.abs(out[:, 4] - pts[:, 4]).max() <= 0.25 + 1e-9


def test_bucket_holds_the_rate_in_every_second():
    b = rl.TokenBucket(100e3)
    sent = np.zeros(10)
    for k in range(10000):
        now = k * 1e-3
        b.refill(now)
        if b.take(200):
            sent[int(now)] += 200
    assert sent.max() * 8 <= 100e3 * (1 + rl.BURST) + 1


def test_voxels_before_points_on_a_thin_link():
    vmap = types.SimpleNamespace(ground_z=0.0, records=lambda keys: (np.full(len(keys), 5), np.full(len(keys), 2.0),
                                                                     np.full(len(keys), 5.0)))
    cells = {(i, 0, 0): None for i in range(2000)}
    mapper = types.SimpleNamespace(map=vmap,
                                   pose=types.SimpleNamespace(at=lambda s, n: (np.zeros(3), np.eye(3), np.zeros(3), 0, 0)))
    s = rl.Sender(kbps=50.0)
    pts = np.tile([10.0, 0, 5, 0, 30], (100, 1))
    keys = list(cells)
    for k in range(600):                        # 10 s of frames at 60 Hz, 100 points each
        report = keys[k * 10:(k + 1) * 10] if k < 200 else []
        s.on_frame(k / 60, k / 60, pts, report, mapper)
    assert not s.queue and s.sent[rl.VOXEL] == 2000
    assert 0 < s.sent[rl.POINTS] < s.points_in
