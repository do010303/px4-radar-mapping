"""frames.LocalNed: the map in PX4's local NED frame; the vehicle's origin over the link to the ground map."""

import os
import sys
import types
import warnings

import numpy as np
import yaml

warnings.filterwarnings("ignore", message="A NumPy version")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import frames  # noqa: E402
import ground_map as gmap  # noqa: E402
import radar_link as rl  # noqa: E402
import survey_objects as so  # noqa: E402

ORIGIN = np.array([2.0, -1.0, 0.25])         # EKF2's origin in the world (ENU)
GEO = (47.397971, 8.546164, 0.25)


def test_points_east_north_up_to_north_east_down():
    ned = frames.LocalNed(ORIGIN, GEO)
    # 10 m east, 5 m north, 3 m up of the origin.
    p = ned.points(ORIGIN + [10.0, 5.0, 3.0])
    assert np.allclose(p, [5.0, 10.0, -3.0])
    assert np.allclose(ned.world(p), ORIGIN + [10.0, 5.0, 3.0])
    assert ned.down(ORIGIN[2] - 0.25) == 0.25


def test_objects_in_ned():
    ned = frames.LocalNed(ORIGIN, GEO)
    pole = ned.object({"type": "pole", "p": [17.0, -1.0, 0.0], "h": 13.0})
    assert np.allclose(pole["p"], [0.0, 15.0, 0.25]) and pole["h"] == 13.0
    wire = ned.object({"type": "wire", "p0": [17.0, 2.0, 10.25], "p1": [47.0, 2.0, 10.25]})
    assert np.allclose(wire["p0"], [3.0, 15.0, -10.0]) and np.allclose(wire["p1"], [3.0, 45.0, -10.0])
    box = ned.object({"type": "box", "c": [27.0, 6.0, 2.25], "size": [6.0, 4.0, 4.0]})
    assert np.allclose(box["c"], [7.0, 25.0, -2.0]) and box["size"] == [4.0, 6.0, 4.0]


def test_objects_file_says_its_frame(tmp_path):
    objects = [{"type": "pole", "p": [17.0, -1.0, 0.0], "h": 13.0, "points": 100, "name": "pole_1", "_idx": None}]
    so.write_objects(str(tmp_path / "a.yaml"), objects, 0.0, frames.LocalNed(ORIGIN, GEO))
    doc = yaml.safe_load(open(tmp_path / "a.yaml"))
    assert doc["frame"] == frames.NAME and doc["origin_lat"] == GEO[0]
    assert np.allclose(doc["objects"][0]["p"], [0.0, 15.0, 0.25]) and "_idx" not in doc["objects"][0]
    so.write_objects(str(tmp_path / "b.yaml"), objects, 0.0)
    assert yaml.safe_load(open(tmp_path / "b.yaml"))["frame"] == "world_enu"


def test_origin_reaches_the_ground_map():
    ground = gmap.GroundMap()
    sender = rl.Sender(300.0, transport=lambda now, d: ground.handle(d, now + 0.05))
    pose = types.SimpleNamespace(origin=ORIGIN, geo=GEO,
                                 at=lambda t, now: (np.zeros(3), np.eye(3), np.zeros(3), np.zeros(3), 0.0))
    mapper = types.SimpleNamespace(pose=pose, map=None)
    for k in range(25):
        sender.on_frame(0.1 * k, 0.1 * k, np.empty((0, 5)), [], mapper)
    # Every ORIGIN_PERIOD: at 0, 1 and 2 s.
    assert sender.sent[rl.ORIGIN] == 3
    assert ground.ned is not None
    assert np.allclose(ground.ned.origin, ORIGIN, atol=1e-5) and np.allclose(ground.ned.geo, GEO)
