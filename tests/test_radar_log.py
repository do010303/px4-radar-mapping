"""radar_log: a recording reads back as written, across chunks and after a stop."""

import collections
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import radar_log  # noqa: E402

Frame = collections.namedtuple("Frame", "stamp det position rotation velocity")


def _record(directory, n_frames, dt=0.1):
    rng = np.random.default_rng(0)
    log = radar_log.RadarLog(str(directory), {"radars": [{"name": "a"}, {"name": "b"}]})
    written = []

    for k in range(n_frames):
        det = rng.normal(size=(int(rng.integers(0, 6)), 5))
        frame = Frame(k * dt, det, rng.normal(size=3), np.eye(3), np.zeros(3))
        log.add(k % 2, frame, det)
        written.append((k % 2, det.astype(np.float32)))

    log.flush()
    return log, written


def test_round_trip_over_chunks(tmp_path):
    _, written = _record(tmp_path, 160)      # 16 s: four chunks
    assert len(list(tmp_path.glob("chunk_*.npz"))) >= 3
    rec = radar_log.load(str(tmp_path))
    assert len(rec["stamp"]) == len(written)
    assert np.all(np.diff(rec["stamp"]) > 0)

    for k, (radar, det) in enumerate(written):
        assert rec["radar"][k] == radar
        np.testing.assert_array_equal(radar_log.frame_points(rec, k), det)

    assert rec["meta"]["radars"][1]["name"] == "b"


def test_rates(tmp_path):
    log, written = _record(tmp_path, 101)
    ti, packed = log.rates()
    points = sum(len(d) for _, d in written)
    assert np.isclose(ti, (points * 20 + 101 * 56) * 8 / 10.0 / 1e3)
    assert packed < ti
